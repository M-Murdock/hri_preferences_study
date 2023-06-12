#!/usr/bin/env python3

import moveit_commander
import rospy
import sys
from geometry_msgs.msg import Pose, PoseStamped
import yaml
import armpy.arm
import traceback
import pickle
import os
import kinova_msgs.srv 

class Autonomous:
    def __init__(self, goal):
        try:
            print("MOVING TO GOAL ", goal)
            moveit_commander.roscpp_initialize(sys.argv)
            self.arm = armpy.arm.Arm()
            # with open('/home/mavis/catkin_ws/src/hri_preferences_study/config/jointstates_goals.yaml', 'r') as file:
            with open('/home/mavis/catkin_ws/src/hri_preferences_study/config/goal_poses.yaml', 'r') as file:
                self.goal = yaml.safe_load(file)[goal]
            
            # to set the goals, move arm to desired position, then `rostopic echo /joint_states`:
            self.move()

        except:
            traceback.print_exc()
            pass

    def move(self):
        # plan = self.arm.plan_pose(self.goal, is_joint_pos=True) # True if joint states. False if eef
        # if plan != None:
        #     self.arm.move_robot(plan, wait=True)
        print(self.goal)
        pos = [self.goal['X'], self.goal['Y'], self.goal['Z'], self.goal['ThetaX'], self.goal['ThetaY'], self.goal['ThetaZ']]
        print(pos)
        rospy.wait_for_service("/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory")
        add_service = rospy.ServiceProxy("/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory", kinova_msgs.srv.AddPoseToCartesianTrajectory)
        add_service(*pos)
 
