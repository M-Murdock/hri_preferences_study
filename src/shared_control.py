#!/usr/bin/env python3
import rospy
import armpy.gen2_teleop
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    rospy.init_node("teleop_test", anonymous=True)

    vel = Twist()
    vel.linear.x = 0.2

    arm = armpy.gen2_teleop.Gen2Teleop(ns="/j2s7s300_driver")
    arm.set_velocity(vel)
    # try:
    #     while not rospy.is_shutdown():
    #         rospy.spin() 
    # except:
    #     pass