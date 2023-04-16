#!/usr/bin/env python3


import rospy

from geometry_msgs.msg import Twist
import armpy.gen2_teleop

if __name__ == "__main__":
    rospy.init_node("teleop_test", anonymous=True)
    arm = armpy.gen2_teleop.Gen2Teleop(ns="/j2s7s300_driver")
    
    vel=Twist()
    vel.linear.y = 0.02

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        r.sleep()
        arm.set_velocity(vel)
   