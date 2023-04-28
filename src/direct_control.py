#!/usr/bin/env python3

import rospy
import armpy.gen2_teleop
import teleop_lib.plugins.user_cmd
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np

if __name__ == "__main__":
    rospy.init_node("direct_control", anonymous=True)
    arm = armpy.gen2_teleop.Gen2Teleop(ns="/j2s7s300_driver")
    action = teleop_lib.plugins.user_cmd.User_Action() # A plugin that gets the user's velocity command


    try:
        while not rospy.is_shutdown():
            action.listen() # when we get a Joy message
            arm.set_velocity(action.cmd.twist) # perform the action given by the user
     
    except:
        pass