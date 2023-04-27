#!/usr/bin/env python3
import rospy
import armpy.gen2_teleop
import teleop_lib.plugins.user_cmd
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rospy.init_node("teleop_test", anonymous=True)

    # v = Twist()
    # v.linear.x = 0.1

    arm = armpy.gen2_teleop.Gen2Teleop(ns="/j2s7s300_driver")
    # arm.set_velocity(v)

    action = teleop_lib.plugins.user_cmd.User_Action() # A plugin that gets the user's velocity command
    # action.listen() # this is always 0 for some reason
 
    # arm.set_velocity(vel.twist)

    try:
        while not rospy.is_shutdown():
            action.listen() # when we get a Joy message
            print(action.cmd)
     
    except:
        pass

    # Get user's action as a unit vector (u)

    # Get robot's predicted action (a)
        # use dummy action
    # dummy_vel = Twist()
    # dummy_vel.linear.x = 0.1

    # Merge the two actions

    # Perform resulting action

# def not_zero(cmd):
#     if cmd.twist.linear.x == 0 and cmd.twist.linear.y == 0 and cmd.twist.linear.z == 0 and cmd.twist.angular.x == 0 and cmd.twist.angular.y == 0 and cmd.twist.angular.z == 0:
#         return False
#     return True