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


class Direct_Control:
    def __init__(self):
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/XYZMode.yaml")
        with open(path, 'r') as file:
            self.cfg = yaml.safe_load(file)

        self.cmd = None

        self.mode = teleop_lib.input_profile.build_profile(self.cfg)
        rospy.Subscriber("/joy", Joy, self.callback)

        self.arm = armpy.gen2_teleop.Gen2Teleop(ns="/j2s7s300_driver")

    def callback(self, data):
        self.arm.set_velocity(self.mode.process_input(data).twist)
