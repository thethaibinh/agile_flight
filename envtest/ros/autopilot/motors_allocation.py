# -*- coding: utf-8 -*-
"""
author: Thai Binh Nguyen
email: thethaibinh@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""
import numpy as np
from enum import Enum, IntEnum
from math import radians

class Frame(Enum):
        MOTOR_FRAME_TYPE_PLUS = 0
        MOTOR_FRAME_TYPE_X = 1
        MOTOR_FRAME_TYPE_V = 2
        MOTOR_FRAME_TYPE_H = 3
        MOTOR_FRAME_TYPE_VTAIL = 4
        MOTOR_FRAME_TYPE_ATAIL = 5
        MOTOR_FRAME_TYPE_PLUSREV = 6   # plus with reversed motor direction
        MOTOR_FRAME_TYPE_Y6B = 10,
        MOTOR_FRAME_TYPE_Y6F = 11   # for FireFlyY6
        MOTOR_FRAME_TYPE_BF_X = 12   # X frame, betaflight ordering
        MOTOR_FRAME_TYPE_DJI_X = 13   # X frame, DJI ordering
        MOTOR_FRAME_TYPE_CW_X = 14   # X frame, clockwise ordering
        MOTOR_FRAME_TYPE_I = 15   # (sideways H) octo only
        MOTOR_FRAME_TYPE_NYT_PLUS = 16   # plus frame, no differential torque for yaw
        MOTOR_FRAME_TYPE_NYT_X = 17   # X frame, no differential torque for yaw
        MOTOR_FRAME_TYPE_BF_X_REV = 18   # X frame, betaflight ordering, reversed motors
class Motors(IntEnum):
    AP_MOTORS_MOT_1 = 0 
    AP_MOTORS_MOT_2 = 1
    AP_MOTORS_MOT_3 = 2
    AP_MOTORS_MOT_4 = 3

FRAME                               = Frame.MOTOR_FRAME_TYPE_X
AP_MOTORS_MATRIX_YAW_FACTOR_CW      = -1
AP_MOTORS_MATRIX_YAW_FACTOR_CCW     = 1
AP_MOTORS_MAX_NUM_MOTORS            = 4

class MotorsAllocator:
    
    def __init__(self):
        self._roll_factor    = np.zeros(4)
        self._pitch_factor   = np.zeros(4)
        self._yaw_factor     = np.zeros(4)
        # parameters
        if FRAME == Frame.MOTOR_FRAME_TYPE_X:
            self.add_motor(Motors.AP_MOTORS_MOT_1, 45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW)
            self.add_motor(Motors.AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW)
            self.add_motor(Motors.AP_MOTORS_MOT_3, -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW)
            self.add_motor(Motors.AP_MOTORS_MOT_4, 135, AP_MOTORS_MATRIX_YAW_FACTOR_CW)
    
    def add_motor_raw(self, motor_num, roll_fac, pitch_fac, yaw_fac):
        self._roll_factor[motor_num] = roll_fac
        self._pitch_factor[motor_num] = pitch_fac
        self._yaw_factor[motor_num] = yaw_fac
        
    def add_motor(self, motor_num, roll_pitch_factor_in_degrees, yaw_factor):
        self.add_motor_raw(
        motor_num,
        np.cos(radians(roll_pitch_factor_in_degrees)),
        np.sin(radians(roll_pitch_factor_in_degrees)),
        yaw_factor)

    def update(self, des_CTBT):
        return self.output_armed_stabilizing(des_CTBT)

# output_armed - sends commands to the motors
# includes new scaling stability patch
    def output_armed_stabilizing(self, des_CTBT):
        _thrust_rpyt_out = np.zeros(4)
        throttle_thrust = des_CTBT[0]
        roll_thrust = des_CTBT[1]
        pitch_thrust = des_CTBT[2]
        yaw_thrust = des_CTBT[3]

        # throttle_thrust = 0.35
        # roll_thrust = 0.01
        # pitch_thrust = 0
        # yaw_thrust = 0

        yaw_allowed = 1.0
        rpy_scale = 1.0

        # sanity check throttle is above zero and below current limited throttle
        if (throttle_thrust <= 0.0):
            throttle_thrust = 0.0
        
        if (throttle_thrust >= 1.0):
            throttle_thrust = 1.0
        
        throttle_thrust_best_rpy = 0.5
           
        for i in range(AP_MOTORS_MAX_NUM_MOTORS):
            _thrust_rpyt_out[i] = roll_thrust * self._roll_factor[i] + pitch_thrust * self._pitch_factor[i]
            if (yaw_thrust * self._yaw_factor[i] > 0.0):
                unused_range = abs(max(1.0 - (throttle_thrust_best_rpy + _thrust_rpyt_out[i]), 0.0)/self._yaw_factor[i])
                if (yaw_allowed > unused_range):
                    yaw_allowed = unused_range
            else:
                unused_range = abs(max(throttle_thrust_best_rpy + _thrust_rpyt_out[i], 0.0)/self._yaw_factor[i])
                if (yaw_allowed > unused_range):
                    yaw_allowed = unused_range
        
        if (abs(yaw_thrust) > yaw_allowed):
            yaw_thrust = np.clip(yaw_thrust,-yaw_allowed, yaw_allowed)

        # add yaw control to thrust outputs
        rpy_low     = 1.0   # lowest thrust value
        rpy_high    = -1.0  # highest thrust value               
        for i in range(AP_MOTORS_MAX_NUM_MOTORS):
            _thrust_rpyt_out[i] = _thrust_rpyt_out[i] + yaw_thrust * self._yaw_factor[i]
            # record lowest roll+pitch+yaw command
            if (_thrust_rpyt_out[i] < rpy_low):
                rpy_low = _thrust_rpyt_out[i]
            # record highest roll+pitch+yaw command
            if (_thrust_rpyt_out[i] > rpy_high):
                rpy_high = _thrust_rpyt_out[i]
    
        # calculate any scaling needed to make the combined thrust outputs fit within the output range
        if (rpy_high - rpy_low > 1.0):
            rpy_scale = 1.0 / (rpy_high - rpy_low)
        if (1 + rpy_low < 0):
            rpy_scale = min(rpy_scale, -1.0 / rpy_low)
        
        # calculate how close the motors can come to the desired throttle
        rpy_high *= rpy_scale
        rpy_low *= rpy_scale
        throttle_thrust_best_rpy = -rpy_low
        thr_adj = throttle_thrust - throttle_thrust_best_rpy
        if (rpy_scale < 1.0):
            # Full range is being used by roll, pitch, and yaw.
            thr_adj = 0.0
        else:
            if (thr_adj < 0.0):
                # Throttle can't be reduced to desired value
                # todo: add lower limit flag and ensure it is handled correctly in altitude controller
                thr_adj = 0.0
            elif (thr_adj > 1.0 - (throttle_thrust_best_rpy + rpy_high)):
                # Throttle can't be increased to desired value
                thr_adj = 1.0 - (throttle_thrust_best_rpy + rpy_high)

        # add scaled roll, pitch, constrained yaw and throttle for each motor
        for i in range(AP_MOTORS_MAX_NUM_MOTORS):
            _thrust_rpyt_out[i] = throttle_thrust_best_rpy + thr_adj + (rpy_scale * _thrust_rpyt_out[i])
        
        return _thrust_rpyt_out * 5.5