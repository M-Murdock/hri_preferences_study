#!/usr/bin/env python3
import rospy
from kinova_msgs.msg import KinovaPose
import numpy as np

    
def listener():
  rospy.init_node('listener', anonymous=True)
  rospy.Subscriber("/j2s7s300_driver/out/cartesian_command", KinovaPose, callback)
  
  rospy.spin()

def callback(data):
    print(data)


if __name__ == '__main__':
    listener()