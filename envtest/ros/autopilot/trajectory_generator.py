# -*- coding: utf-8 -*-
"""
author: Thai Binh Nguyen
email: thethaibinh@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""
from math import pi
import time
import warnings
import numpy as np
from autopilot.utils import quat_to_euler_angles, get_bearing_rad
import visualization_msgs.msg as visualization_msgs
from geometry_msgs.msg import Pose, PoseStamped
import rospy

# from agile_flight.envtest.ros.autopilot.rapid_trajectory_generator import RapidTrajectory

try:
    # Get monotonic time to ensure that time deltas are always positive
    _current_time = time.monotonic
except AttributeError:
    # time.monotonic() not available (using python < 3.3), fallback to time.time()
    _current_time = time.time
    warnings.warn('time.monotonic() not available in python < 3.3, using time.time() as fallback')


class TrajectoryGenerator:
    
    def __init__(self):
        # controllers
        self._init_time = None
        # self._des_target1 = [-3.0, -23.0, 5.0]
        self._des_target1 = [0.0, 0.0, 5.0, 0.0]
        self._des_target = None
        self._des_yaw = None
        self._des_traj = None
        self._navigate = False
        self._bearing = 0

        self._ref = visualization_msgs.Marker()
        self._ref.header.frame_id = "world"
        self._ref.ns = "ref"
        self._ref.lifetime = rospy.Duration(1)
        self._ref.type = visualization_msgs.Marker.LINE_STRIP;
        self._ref.pose.orientation.w = 1
        self._ref.scale.x = 0.1
        self._ref.scale.y = 0.1
        self._ref.scale.z = 0.1
        self._ref.color.a = 1
        self._ref.color.g = 1
        # self._ref.color.r = 1

        quad_namespace = 'kingfisher'
        self.trajectory_sub = rospy.Subscriber("/trajectory", Pose, self.update_callback,
                                          queue_size=1, tcp_nodelay=True)
        self._reference_waypoint_pub = rospy.Publisher(
            quad_namespace + '/references/markers', visualization_msgs.Marker,
            queue_size=1)
            
    # update_wpnav - run the wp controller - should be called at 100hz or higher
    def update_callback(self, traj):
        if traj.orientation.x == 0:
            self._des_traj = traj.position
            self._des_target = [traj.position.x, traj.position.y, traj.position.z]
        else:
            self._des_yaw = traj.orientation.z/5

    def update(self, state, steering, goal):
        if self._init_time is None:
            self._init_time = _current_time()
            out =  self._des_target1
        elif ((state.pos[2] > 4.8) or self._navigate) and (self._des_target is not None):
            # get bearing for yaw reference
            curr_att = quat_to_euler_angles(state.att)
            if self._navigate == False:
                self._navigate = True
            if np.linalg.norm(self._des_target - state.pos) > 1:
                bearing = [-get_bearing_rad(state.pos, self._des_target)] if steering else [-get_bearing_rad(state.pos, ([goal, 0, 5]))]
            else:
                if steering and self._des_yaw is not None:
                    bearing = (-curr_att[2] + self._des_yaw)
                else:
                    bearing = self._bearing
            self._bearing = bearing
            out = np.concatenate((self._des_target, bearing))
        else:
            out = self._des_target1
        # publish executed trajectories
        self._ref.header.stamp = rospy.Time.now()
        go_to_pose_msg = PoseStamped()
        go_to_pose_msg.pose.position.x = float(state.pos[0])
        go_to_pose_msg.pose.position.y = float(state.pos[1])
        go_to_pose_msg.pose.position.z = float(state.pos[2])
        if self._des_traj is not None:
            self._ref.points.append(go_to_pose_msg.pose.position)
            if not rospy.is_shutdown():
                self._reference_waypoint_pub.publish(self._ref)
        return out
    