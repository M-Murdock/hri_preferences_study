#!/usr/bin/env python3

import moveit_commander
import rospy
import sys
from geometry_msgs.msg import Pose, PoseStamped
import yaml
import armpy.arm
import traceback
import os
import kinova_msgs.srv 

class Autonomous:
    def __init__(self, goal):
        try:
            print("MOVING TO GOAL ", goal)
            moveit_commander.roscpp_initialize(sys.argv)
            self.arm = armpy.arm.Arm()
            
            path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/goal_poses.yaml")
            with open(path, 'r') as file:
                self.goal = yaml.safe_load(file)[goal]
            
            self.move()

        except:
            traceback.print_exc()
            pass

    def move(self):
        print(self.goal)
        pos = [self.goal['X'], self.goal['Y'], self.goal['Z'], self.goal['ThetaX'], self.goal['ThetaY'], self.goal['ThetaZ']]
        print(pos)
        rospy.wait_for_service("/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory")
        add_service = rospy.ServiceProxy("/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory", kinova_msgs.srv.AddPoseToCartesianTrajectory)
        add_service(*pos)

        add_service.close()
        print("MOVED TO GOAL")
