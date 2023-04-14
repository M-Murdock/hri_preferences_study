#!/usr/bin/env python3
import rospy
from kinova_msgs.msg import KinovaPose
from geometry_msgs.msg import PoseStamped
import armpy
import numpy as np

    
def listener():
  rospy.init_node('listener', anonymous=True)
  rospy.Subscriber("/j2s7s300_driver/out/cartesian_command", KinovaPose, callback)
  rospy.spin()

def callback(data):
    print(data)


if __name__ == '__main__':
    listener()


    # /j2s7s300_driver/pose_action/tool_pose/goal kinova_msgs/ArmPoseActionGoal
    # to get current tool pose: /j2s7s300_driver/out/tool_pose  geometry_msgs/PoseStamped
    
    
    # to execute command, goto_eef_waypoints(self, waypoints) in `Teleop_lib gen3_movement_utils`
    # rosservice /j2s7s300_driver/in/add_pose_to_Cartesian_trajectory