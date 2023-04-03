#!/usr/bin/env python3
import rospy
from kinova_msgs.msg import KinovaPose

global last_x 


def callback(data):
    global last_x 
    if last_x == None:
        last_x = data.X
        print("Initializing X")
    
    elif last_x == data.X:
        print("same")
        
    else:
        last_x = data.X
        rospy.loginfo(rospy.get_caller_id() + "\n%s", data) 

def listener():
  rospy.init_node('listener', anonymous=True)
  rospy.Subscriber("/j2s7s300_driver/out/cartesian_command", KinovaPose, callback)
  
  rospy.spin()

if __name__ == '__main__':
    global last_x 
    last_x = None
    listener()