#!/usr/bin/env python3
import rospy
import armpy.gen2_teleop
import teleop_lib.plugins.user_cmd
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np

if __name__ == "__main__":
    rospy.init_node("teleop_test", anonymous=True)

    arm = armpy.gen2_teleop.Gen2Teleop(ns="/j2s7s300_driver")

    action = teleop_lib.plugins.user_cmd.User_Action() # A plugin that gets the user's velocity command


    try:
        while not rospy.is_shutdown():
            action.listen() # when we get a Joy message
            arm.set_velocity(action.cmd.twist)
            position = rospy.wait_for_message('/j2s7s300_driver/out/tool_pose', PoseStamped)

            user_action = np.array([action.cmd.twist.linear.x, action.cmd.twist.linear.y, action.cmd.twist.linear.z])
            end_eff_pos = np.array([position.pose.position.x, position.pose.position.y, position.pose.position.z])
            print(end_eff_pos)
            print(user_action)

            unit_vector = user_action / np.linalg.norm(user_action)
            print('unit vector: ', unit_vector)

            # Get user's action as a unit vector (u)

            # Get robot's predicted action (a)

            # Merge the two actions

            # Perform resulting action
     
    except:
        pass

    
