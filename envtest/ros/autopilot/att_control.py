# -*- coding: utf-8 -*-
"""
author: Thai Binh Nguyen
email: thethaibinh@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from simple_pid import PID
from autopilot.utils import quat_to_euler_angles, SqrtControllerAtt, wrap_180_rad
from math import radians

GRAVITY_MSS             = 9.80665
M_PI                    = 3.141592653589793

# Attitude P gains
ATTITUDE_CONTROL_ANGLE_RP_P    = 0.05
ATTITUDE_CONTROL_ANGLE_YAW_P    = 0.015

# default rate controller PID gains
RATE_CONTROL_RP_P       = 0.135
RATE_CONTROL_RP_I       = 0.135
RATE_CONTROL_RP_D       = 0.0036   # 0.0036

RATE_CONTROL_YAW_P      = 0.07
RATE_CONTROL_YAW_I      = 0.0
RATE_CONTROL_YAW_D      = 0.0

ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS   = radians(720.0)
ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS    = radians(20.0)

class AttControllers:
    
    def __init__(self):
        # attributes
        self._last_time = None
        self._last_output = None
        self._last_input = None

        self.des_rate = np.zeros(3)
        self.pid_rate_roll     = PID(RATE_CONTROL_RP_P, RATE_CONTROL_RP_I, RATE_CONTROL_RP_D, sample_time = 0.001, output_limits=(-1.0, 1.0))
        self.pid_rate_pitch    = PID(RATE_CONTROL_RP_P, RATE_CONTROL_RP_I, RATE_CONTROL_RP_D, sample_time = 0.001, output_limits=(-1.0, 1.0))
        self.pid_rate_yaw      = PID(RATE_CONTROL_YAW_P, RATE_CONTROL_YAW_I, RATE_CONTROL_YAW_D, sample_time = 0.001, output_limits=(-1.0, 1.0))
        self.pid_ang_roll      = SqrtControllerAtt()
        self.pid_ang_pitch     = SqrtControllerAtt()
        self.pid_ang_yaw       = SqrtControllerAtt()
    
    def update(self, state, des_CTBO):
        curr_att = quat_to_euler_angles(state.att)
        
        # run angular sqrt controllers
        des_rate = np.zeros(3)
        des_rate[0] = self.pid_ang_roll(wrap_180_rad(des_CTBO[1]) - (-curr_att[0]), ATTITUDE_CONTROL_ANGLE_RP_P, ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS)
        des_rate[1] = self.pid_ang_pitch(wrap_180_rad(des_CTBO[2]) - (-curr_att[1]), ATTITUDE_CONTROL_ANGLE_RP_P, ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS)
        des_rate[2] = self.pid_ang_yaw(wrap_180_rad(wrap_180_rad(des_CTBO[3]) - (-curr_att[2])), ATTITUDE_CONTROL_ANGLE_YAW_P, ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS)

        # run angular rate PID controllers
        des_thrust = np.zeros(4)
        des_thrust[0] = des_CTBO[0]
        des_thrust[1] = -self.pid_rate_roll(des_rate[0] - radians(-state.omega[0]))
        des_thrust[2] = -self.pid_rate_pitch(des_rate[1] - radians(-state.omega[1]))
        des_thrust[3] = -self.pid_rate_yaw(des_rate[2] - radians(-state.omega[2]))

        return des_thrust
