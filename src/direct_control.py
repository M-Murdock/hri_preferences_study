#!/usr/bin/env python3

import rospy
import armpy.gen2_teleop
import teleop_lib.plugins.user_cmd
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np
import traceback


if __name__ == "__main__":
    rospy.init_node("direct_control", anonymous=True)
    arm = armpy.gen2_teleop.Gen2Teleop(ns="/j2s7s300_driver")
    action = teleop_lib.plugins.user_cmd.User_Action() # A plugin that gets the user's velocity command

    try:
        while not rospy.is_shutdown():
            if action.cmd == None:
                continue
            arm.set_velocity(action.cmd.twist) # perform the action given by the user
            rospy.sleep(0.1)
    except:
        traceback.print_exc()
        pass