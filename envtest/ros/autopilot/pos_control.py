# -*- coding: utf-8 -*-
"""
author: Thai Binh Nguyen
email: thethaibinh@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

from cmath import pi
import numpy as np
from simple_pid import PID
import math
from autopilot.utils import SqrtController, SqrtControllerAtt, limit_vector_length, quat_to_euler_angles

GRAVITY_MSS                 = 9.80665
POSCONTROL_ACCELERATION_MIN = 0.50      # minimum horizontal acceleration in cm/s/s - used for sanity checking acceleration in leash length calculation
POSCONTROL_ACCEL_XY_MAX     = 5         # max horizontal acceleration in cm/s/s that the position velocity controller will ask from the lower accel controller
POSCONTROL_ANGLE_XY_MAX     = 60.0      # maximum autopilot commanded angle (in degrees) for position control
POSCONTROL_ACCEL_Z          = 2.0       # max vertical acceleration in cm/s/s.
POSCONTROL_SPEED            = 1.0       # max horizontal speed in cm/s
POSCONTROL_SPEED_DOWN       = -1.5      # max descent rate in cm/s
POSCONTROL_SPEED_UP         = 2.5       # max climb rate in cm/s
POSCONTROL_LEASH_LENGTH_MIN = 0.5       # minimum leash lengths in cm
# M_PI                        = 3.141592653589793

# ---------------------------
# Position P gains
POSCONTROL_POS_Z_P      = 1.0
POSCONTROL_VEL_Z_P      = 5.0
POSCONTROL_ACC_Z_P      = 0.5    # 0.5 vertical acceleration controller P gain default
POSCONTROL_ACC_Z_I      = 1.0    # 1 vertical acceleration controller I gain default
POSCONTROL_ACC_Z_D      = 0
POSCONTROL_POS_XY_P     = 1     # 1
POSCONTROL_VEL_XY_P     = 1     # 2 horizontal velocity controller P gain default
POSCONTROL_VEL_XY_I     = 0.1     # 1 horizontal velocity controller I gain default
POSCONTROL_VEL_XY_D     = 0.0   # 0.5 horizontal velocity controller D gain default

class PosControllers:
    
    def __init__(self):
        # controllers
        self.sqrt_pos_xy = SqrtController()
        self.sqrt_pos_z = SqrtControllerAtt()
        self.p_vel_z = PID(POSCONTROL_VEL_Z_P, 0, 0, sample_time = 0.001, output_limits = (-5,5))
        self.pid_accel_z = PID(POSCONTROL_ACC_Z_P, POSCONTROL_ACC_Z_I, POSCONTROL_ACC_Z_D, sample_time = 0.001, output_limits = (-1,1))
        self.pid_vel_x = PID(POSCONTROL_VEL_XY_P, POSCONTROL_VEL_XY_I, POSCONTROL_VEL_XY_D, sample_time = 0.001, output_limits= (-POSCONTROL_ACCEL_XY_MAX, POSCONTROL_ACCEL_XY_MAX))
        self.pid_vel_y = PID(POSCONTROL_VEL_XY_P, POSCONTROL_VEL_XY_I, POSCONTROL_VEL_XY_D, sample_time = 0.001, output_limits= (-POSCONTROL_ACCEL_XY_MAX, POSCONTROL_ACCEL_XY_MAX))

        # internal variables
        self._leash             = POSCONTROL_LEASH_LENGTH_MIN       # horizontal leash length in cm.  target will never be further than this distance from the vehicle
        self._leash_down_z      = POSCONTROL_LEASH_LENGTH_MIN       # vertical leash down in cm.  target will never be further than this distance below the vehicle
        self._leash_up_z        = POSCONTROL_LEASH_LENGTH_MIN       # vertical leash up in cm.  target will never be further than this distance above the vehicle
    
    def update(self, state, des_pos, des_yaw):
        self.calc_leash_length_xy()
        self.calc_leash_length_z()
        # run posotion controllers
        des_vel = self.pos_controller(state, des_pos)
        # run velocity controllers
        des_accel = self.vel_controller(state, des_vel)
        # update angle targets that will be passed to attitude controller
        # Collective thrust & body orientation: [throttle, roll, pitch, yaw]
        des_CTBO = np.concatenate((self.accel_z_controller(state, des_accel), self.accel_to_lean_angles(state, des_accel), [des_yaw]))
        return des_CTBO
    
    def accel_z_controller(self, states, des_acc):
        # the following section calculates a desired throttle needed to achieve the acceleration target
        # Calculate Earth Frame Z acceleration
        z_accel_meas = states.acc[2]
        z_accel_error = des_acc[2] - z_accel_meas
        thr_out = -self.pid_accel_z(z_accel_error)
        thr_out += 0.34
        thr_out = np.clip(thr_out, 0.0, 1.0)
        return [thr_out]

    def pos_controller(self, states, des_pos):
        kP = POSCONTROL_POS_XY_P # scale gains to compensate for noisy optical flow measurement in the EKF
        vel_target = np.zeros(3)
        # calculate distance error
        pos_error = des_pos - states.pos
        
        # xy - horizontal
        # avoid divide by zero
        if kP <= 0.0:
            vel_target[0] = 0.0
            vel_target[1] = 0.0
        else:
            # Constrain pos_error and target position
            # Constrain the maximum length of vel_target to the maximum position correction velocity
            # TODO: replace the leash length with a user definable maximum position correction
            pos_error = limit_vector_length(pos_error, self._leash)
            vel_target = self.sqrt_pos_xy(pos_error, kP, POSCONTROL_ACCEL_XY_MAX)
        
        # z - vertical
        # do not let target altitude get too far from current altitude
        if (pos_error[2] > self._leash_up_z):
            pos_error[2] = self._leash_up_z
        if (pos_error[2] < -self._leash_down_z):
            pos_error[2] = -self._leash_down_z
        # calculate des_vel[2] using from pos_error[2] using sqrt controller
        vel_target[2] = self.sqrt_pos_z(pos_error[2], POSCONTROL_POS_Z_P, POSCONTROL_ACCEL_Z)
        return vel_target

    def vel_controller(self, state, des_vel):
        # calculate velocity error
        if (des_vel[2] < POSCONTROL_SPEED_DOWN):
            des_vel[2] = POSCONTROL_SPEED_DOWN
        
        if (des_vel[2] > POSCONTROL_SPEED_UP):
            des_vel[2] = POSCONTROL_SPEED_UP
        vel_error = des_vel - state.vel

        accel_target = np.zeros(3)
        # acceleration to correct for velocity error and scale PID output to compensate for optical flow measurement induced EKF noise
        accel_target[0] = -self.pid_vel_x(vel_error[0])
        accel_target[1] = -self.pid_vel_y(vel_error[1])

        # limit acceleration using maximum lean angles
        accel_max = min(GRAVITY_MSS * math.tan(math.radians(POSCONTROL_ANGLE_XY_MAX)), POSCONTROL_ACCEL_XY_MAX)
        
        # the following section calculates acceleration required to achieve the velocity target
        accel_target[2] = -self.p_vel_z(vel_error[2])
        np.put(accel_target, [0,1], limit_vector_length(accel_target, accel_max))
        return accel_target
    
    # get_lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    def accel_to_lean_angles(self, states, des_accel_cmss):
        euler_angles = np.squeeze(quat_to_euler_angles(states.att))
        # rotate accelerations into body forward-right frame
        # todo: this should probably be based on the desired heading not the current heading
        accel_forward = des_accel_cmss[0] * math.cos(euler_angles[2]) + des_accel_cmss[1] * math.sin(euler_angles[2])
        accel_right = -des_accel_cmss[0] * math.sin(euler_angles[2]) + des_accel_cmss[1] * math.cos(euler_angles[2])

        # update angle targets that will be passed to stabilize controller
        pitch_target = math.atan(-accel_forward / GRAVITY_MSS)
        cos_pitch_target = math.cos(pitch_target)
        roll_target = math.atan(accel_right * cos_pitch_target / GRAVITY_MSS)
        return [roll_target, pitch_target]
    
    # calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration and position kP gain
    def calc_leash_length(self, speed_cms, accel_cms, kP):
        # sanity check acceleration and avoid divide by zero
        if (accel_cms <= 0.0):
            accel_cms = POSCONTROL_ACCELERATION_MIN
        
        # avoid divide by zero
        if (kP <= 0.0):
            return POSCONTROL_LEASH_LENGTH_MIN
        
        # calculate leash length
        if (speed_cms <= accel_cms / kP):
            # linear leash length based on speed close in
            leash_length = speed_cms / kP
        else:
            # leash length grows at sqrt of speed further out
            leash_length = (accel_cms / (2.0 * kP * kP)) + (speed_cms * speed_cms / (2.0 * accel_cms))
        
        # ensure leash is at least 1m long
        if (leash_length < POSCONTROL_LEASH_LENGTH_MIN):
            leash_length = POSCONTROL_LEASH_LENGTH_MIN

        return leash_length
    
    # calc_leash_length - calculates the vertical leash lengths from maximum speed, acceleration
    #     called by update_z_controller if z-axis speed or accelerations are changed
    def calc_leash_length_z(self):
    
        self._leash_up_z = self.calc_leash_length(POSCONTROL_SPEED_UP, POSCONTROL_ACCEL_Z, POSCONTROL_POS_Z_P);
        self._leash_down_z = self.calc_leash_length(-POSCONTROL_SPEED_DOWN, POSCONTROL_ACCEL_Z, POSCONTROL_POS_Z_P);
    

    # calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration
    #     should be called whenever the speed, acceleration or position kP is modified
    def calc_leash_length_xy(self):
        # todo: remove _flags.recalc_leash_xy or don't call this function after each variable change.
        self._leash = self.calc_leash_length(POSCONTROL_SPEED, POSCONTROL_ACCEL_XY_MAX, POSCONTROL_POS_XY_P);
    
        
