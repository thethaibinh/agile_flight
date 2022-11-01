# -*- coding: utf-8 -*-
"""
author: Thai Binh Nguyen
email: thethaibinh@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy import sqrt, square, pi
from numpy.linalg import norm
import time
import warnings

GRAVITY_MSS             = 9.80665
POSCONTROL_ACCEL_XY_MAX = 9.8
# M_PI                    = 3.141592653589793

try:
    # Get monotonic time to ensure that time deltas are always positive
    _current_time = time.monotonic
except AttributeError:
    # time.monotonic() not available (using python < 3.3), fallback to time.time()
    _current_time = time.time
    warnings.warn('time.monotonic() not available in python < 3.3, using time.time() as fallback')


def limit_vector_length(vector, max_length):
    vector_length = norm(vector[0:2])
    if (vector_length > max_length) & (vector_length >= 0):
        vector[0] *= (max_length / vector_length)
        vector[1] *= (max_length / vector_length)    
    return vector

# return bearing in centi-degrees between two positions
def get_bearing_cd(origin, destination):
    bearing = np.rad2deg(np.arctan2(destination[1]-origin[1], destination[0]-origin[0])) * 100
    if (bearing < 0):
        bearing += 36000.0
    return wrap_180_cd(bearing)

def get_bearing_rad(origin, destination):
    bearing = np.arctan2(destination[1]-origin[1], destination[0]-origin[0])
    if (bearing < 0):
        bearing += (2 * pi)
    return wrap_180_rad(bearing)

def get_bearing_deg(origin, destination):
    bearing = np.rad2de(np.arctan2(destination[1]-origin[1], destination[0]-origin[0]))
    if (bearing < 0):
        bearing += 360
    return wrap_180_deg(bearing)

def wrap_360_cd(angle):
    res = np.fmod(angle, 36000.0)
    if (res < 0):
        res += 36000.0
    return res

def wrap_180_cd(angle):
    res = wrap_360_cd(angle)
    if (res > 18000.0): 
        res -= 36000.0
    return res

def wrap_360_deg(angle):
    res = np.fmod(angle, 360.0)
    if (res < 0):
        res += 360.0
    return res

def wrap_180_deg(angle):
    res = wrap_360_deg(angle)
    if (res > 180.0): 
        res -= 360.0
    return res

def wrap_360_rad(angle):
    res = np.fmod(angle, 2*pi)
    if (res < 0):
        res += 2*pi
    return res

def wrap_180_rad(angle):
    res = wrap_360_rad(angle)
    if (res > pi): 
        res -= 2*pi
    return res

def quat_to_euler_angles(q):
    #  Computes the euler angles from a unit quaternion using the
    #  z-y-x convention.
    #  euler_angles = [roll pitch yaw]'
    #  A quaternion is defined as q = [qw qx qy qz]'
    #  where qw is the real part.
    euler_angles = np.zeros((3, 1))
    euler_angles[0] = np.arctan2(
        2 * q[0] * q[1] + 2 * q[2] * q[3], q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
    euler_angles[1] = -np.arcsin(2 * q[1] * q[3] - 2 * q[0] * q[2])
    euler_angles[2] = np.arctan2(
        2 * q[0] * q[3] + 2 * q[1] * q[2], q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
    return euler_angles

class SqrtController():
    def __init__(
        self,
        sample_time=0.001):
        self.sample_time = sample_time
        self._last_time = None
        self._last_output = None
        self._last_input = None
        self.reset()

    # Proportional controller with piecewise sqrt sections to constrain second derivative
    def __call__(self, error, p, second_ord_lim, dt=None):
        now = _current_time()
        if dt is None:
            dt = now - self._last_time if (now - self._last_time) else 1e-16
        elif dt <= 0:
            raise ValueError('dt has negative value {}, must be positive'.format(dt))
        
        if self.sample_time is not None and dt < self.sample_time and self._last_output is not None:
            # Only update every sample_time seconds
            output = self._last_output
            return output

        if second_ord_lim < 0.0 or (not np.all(second_ord_lim)) or (not np.all(p)):
            output = [error[0] * p, error[1] * p, error[2]]
            # Keep track of state
            self._last_output = output
            self._last_input = error
            self._last_time = now
            return output

        linear_dist = second_ord_lim / square(p)
        error_length = norm(error[0:2])
        if (error_length > linear_dist):
            first_order_scale = safe_sqrt(2.0 * second_ord_lim * (error_length - (linear_dist * 0.5))) / error_length
            output = [error[0] * first_order_scale, error[1] * first_order_scale, error[2]]
        else:
            output = [error[0] * p, error[1] * p, error[2]]

        # Keep track of state
        self._last_output = output
        self._last_input = error
        self._last_time = now
        return output

    def reset(self):
        """
        Reset the SQRT controller internals.
        This sets each term to 0 as well as clearing the last output and the last
        input.
        """
        self._last_time = _current_time()
        self._last_output = None
        self._last_input = None

class SqrtControllerAtt():
    def __init__(
        self,
        sample_time=0.001):
        self.sample_time = sample_time
        self._last_time = None
        self._last_output = None
        self._last_input = None
        self.reset()

    # Proportional controller with piecewise sqrt sections to constrain second derivative
    def __call__(self, error, p, second_ord_lim, dt=None):
        now = _current_time()
        if dt is None:
            dt = now - self._last_time if (now - self._last_time) else 1e-16
        elif dt <= 0:
            raise ValueError('dt has negative value {}, must be positive'.format(dt))
        
        if self.sample_time is not None and dt < self.sample_time and self._last_output is not None:
            # Only update every sample_time seconds
            output = self._last_output
            return output

        if ((second_ord_lim < 0) or (not np.all(second_ord_lim))):
            # second order limit is zero or negative.
            correction_rate = error * p
        elif (not np.all(p)):
            # P term is zero but we have a second order limit.
            if error > 0:
                correction_rate = safe_sqrt(2.0 * second_ord_lim * (error))
            elif error < 0:
                correction_rate = -safe_sqrt(2.0 * second_ord_lim * (-error))
            else:
                correction_rate = 0.0
        else:
            # Both the P and second order limit have been defined.
            linear_dist = second_ord_lim / square(p)
            if (error > linear_dist):
                correction_rate = safe_sqrt(2.0 * second_ord_lim * (error - (linear_dist / 2.0)))
            elif (error < -linear_dist):
                correction_rate = -safe_sqrt(2.0 * second_ord_lim * (-error - (linear_dist / 2.0)))
            else:
                correction_rate = error * p

        if np.all(dt):
            # this ensures we do not get small oscillations by over shooting the error correction in the last time step.
            output = np.clip(correction_rate, -abs(error) / dt, abs(error) / dt)
            # output = correction_rate
        else:
            output = correction_rate
        
        # Keep track of state
        self._last_output = output
        self._last_input = error
        self._last_time = now
        return output
    
    def reset(self):
        """
        Reset the SQRT controller internals.
        This sets each term to 0 as well as clearing the last output and the last
        input.
        """
        self._last_time = _current_time()
        self._last_output = None
        self._last_input = None

def safe_sqrt(v):
    ret = sqrt(v)
    if (np.isnan(ret)):
        return 0
    return ret