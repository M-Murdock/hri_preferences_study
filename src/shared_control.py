#!/usr/bin/env python3
import rospy
import armpy.gen2_teleop
import teleop_lib.plugins.user_cmd
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rospy.init_node("teleop_test", anonymous=True)

    v = Twist()
    v.linear.x = 0.1

    arm = armpy.gen2_teleop.Gen2Teleop(ns="/j2s7s300_driver")
    # arm.set_velocity(v)

    action=teleop_lib.plugins.user_cmd.User_Action() # A plugin that gets the user's velocity command
    vel = action.listen() # this is always 0 for some reason
    print(vel.twist)
    arm.set_velocity(vel.twist)



    try:
        while not rospy.is_shutdown():
            rospy.spin() 
    except:
        pass

    # Get user's action as a unit vector (u)

    # Get robot's predicted action (a)
        # use dummy action
    # dummy_vel = Twist()
    # dummy_vel.linear.x = 0.1

    # Merge the two actions

    # Perform resulting action