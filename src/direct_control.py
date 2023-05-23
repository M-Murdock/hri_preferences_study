#!/usr/bin/env python3

import rospy
import armpy.gen2_teleop
import teleop_lib.plugins.user_cmd
from teleop_lib.gui.teleop_config_frame import TeleopConfigFrame, get_teleop_info
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np
import traceback
import yaml
import teleop_lib.input_profile
from sensor_msgs.msg import Joy



# if __name__ == "__main__":
#     rospy.init_node("direct_control", anonymous=True)
#     arm = armpy.gen2_teleop.Gen2Teleop(ns="/j2s7s300_driver")
#     action = teleop_lib.plugins.user_cmd.User_Action() # A plugin that gets the user's velocity command

#     try:
#         while not rospy.is_shutdown():
#             if action.cmd == None:
#                 continue
#             arm.set_velocity(action.cmd.twist) # perform the action given by the user
#             rospy.sleep(0.1)
#     except:
#         traceback.print_exc()
#         pass


class Direct_Control:
    def __init__(self):
        with open("/home/mavis/catkin_ws/src/hri_preferences_study/config/XYZMode.yaml") as f:
            self.cfg = yaml.safe_load(f)

        self.cmd = None

        self.mode = teleop_lib.input_profile.build_profile(self.cfg)
        rospy.Subscriber("/joy", Joy, self.callback)

        self.arm = armpy.gen2_teleop.Gen2Teleop(ns="/j2s7s300_driver")
        self.action = teleop_lib.plugins.user_cmd.User_Action() # A plugin that gets the user's velocity command

    def callback(self, data):
        self.arm.set_velocity(self.mode.process_input(data).twist)
        