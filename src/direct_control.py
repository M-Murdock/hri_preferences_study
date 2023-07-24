#!/usr/bin/env python3

import rospy
import armpy.gen2_teleop
from teleop_lib.gui.teleop_config_frame import TeleopConfigFrame, get_teleop_info
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np
import traceback
import yaml
import teleop_lib.input_profile
from sensor_msgs.msg import Joy
import os
import armpy.gripper


class Direct_Control:
    def __init__(self):
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/XYZMode.yaml")
        with open(path, 'r') as file:
            self.cfg = yaml.safe_load(file)

        self.cmd = None

        self.gripper = armpy.gripper.Gripper()

        self.mode = teleop_lib.input_profile.build_profile(self.cfg)
        rospy.Subscriber("/joy", Joy, self.callback)
        rospy.Subscriber("/j2s7s300_driver/out/tool_pose", PoseStamped, self.check_pose)

        self.pose = PoseStamped()
        self.arm = armpy.gen2_teleop.Gen2Teleop(ns="/j2s7s300_driver")

        self.allow_gripper_control = False

    def check_pose(self, data):
        self.pose = data 

    def callback(self, data):
        if self.allow_gripper_control:
            if data.buttons[4] == 1 and data.buttons[5] == 1: 
                self.arm.stop()
                self.open_gripper() 
                


        command = self.mode.process_input(data).twist
        new_cmd = Twist()
        new_cmd.linear.x = command.linear.x
        new_cmd.linear.y = command.linear.y
        new_cmd.linear.z = command.linear.z

        # print(self.pose)

        # If arm is too close to participant
        if self.pose.pose.position.y > 0.02:
            if command.linear.y > 0:
                # print("y is too low")
                new_cmd.linear.y = 0
        # if arm is going to hit table
        if self.pose.pose.position.z < 0.01:
            if command.linear.z < 0:
                # print("z is too low")
                new_cmd.linear.z = 0

        self.arm.set_velocity(new_cmd)

# stop arm right after opening gripper
    def open_gripper(self):
        self.gripper.open()

    def close_gripper(self):
        self.gripper.close()
