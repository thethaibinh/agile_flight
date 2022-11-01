# -*- coding: utf-8 -*-
"""
author: Thai Binh Nguyen
email: thethaibinh@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""
import rospy
from autopilot.pos_control import PosControllers
from autopilot.att_control import AttControllers
from autopilot.motors_allocation import MotorsAllocator
from autopilot.trajectory_generator import TrajectoryGenerator
from std_msgs.msg import Empty

class Autopilot:
    
    def __init__(self):
        # controllers
        self.pos_controller = PosControllers()
        self.att_controller = AttControllers()
        self.motors_allocator = MotorsAllocator()
        self.traj_generator = TrajectoryGenerator()
        self.publish_commands = False
        quad_name = 'kingfisher'
        self.start_sub = rospy.Subscriber("/" + quad_name + "/start_navigation", Empty, self.start_callback,
                                          queue_size=1, tcp_nodelay=True)
    
    def update(self, state, steering, goal):
        if (not self.publish_commands):
            return [0.0, 0.0, 0.0, 0.0]
        else:
            des_traj = self.traj_generator.update(state, steering, goal)
            des_pos = des_traj[:3]
            des_yaw = des_traj[3]
            # Collective thrust & body orientation: [throttle thrust, roll, pitch, yaw]
            des_CTBO = self.pos_controller.update(state, des_pos, des_yaw)
            # Collective thrust & body thrust: [throttle thrust, roll thrust, pitch thrust, yaw thrust]
            des_CTBT = self.att_controller.update(state, des_CTBO)
            # SRT - Single rotor thrust
            return self.motors_allocator.update(des_CTBT)
    
    def start_callback(self, data):
        print("Autopilot have taken control!")
        self.publish_commands = True