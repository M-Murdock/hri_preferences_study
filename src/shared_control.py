#!/usr/bin/env python3
import rospy
import armpy.gen2_teleop
import teleop_lib.plugins.user_cmd
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np

if __name__ == "__main__":
    rospy.init_node("shared_control", anonymous=True)
    arm = armpy.gen2_teleop.Gen2Teleop(ns="/j2s7s300_driver")
    action = teleop_lib.plugins.user_cmd.User_Action() # A plugin that gets the user's velocity command

    try:
        while not rospy.is_shutdown():
            action.listen() # wait for user's command
            position = rospy.wait_for_message('/j2s7s300_driver/out/tool_pose', PoseStamped) # wait for robot's position

            # Store user action, state as numpy arrays
            user_action = np.array([action.cmd.twist.linear.x, action.cmd.twist.linear.y, action.cmd.twist.linear.z])
            end_eff_pos = np.array([position.pose.position.x, position.pose.position.y, position.pose.position.z])
 
            print('state: ', end_eff_pos)
            print('user action: ', user_action)


            # Get user's action as a unit vector (u)
            if not np.all(user_action==0):
                unit_vector = user_action / np.linalg.norm(user_action)
            else:
                unit_vector = np.array([0, 0, 0])

            print('unit vector: ', unit_vector)

            # Get robot's predicted action (a)
            predicted_action = np.array([0, 0, 1]) # expressed as a unit vector NOTE: this needs to be calculated

            # Merge the two actions
            merged_action = (unit_vector + predicted_action) / 2
            
            # Convert the action vector to a Twist message
            merged_action_twist = Twist()
            merged_action_twist.linear.x = merged_action[0]
            merged_action_twist.linear.y = merged_action[1]
            merged_action_twist.linear.z = merged_action[2]

            print('merged action: ', merged_action_twist, '\n')

            # Perform resulting action
            arm.set_velocity(merged_action_twist)

    except:
        pass

    
