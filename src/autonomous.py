#!/usr/bin/env python3

import moveit_commander
import rospy
import sys
from geometry_msgs.msg import Pose, PoseStamped
import yaml
import armpy.arm
import armpy.gripper
import traceback
import os
import kinova_msgs.srv
 

class Autonomous:
    def __init__(self, goal, file=True):

        try:
            print("MOVING TO GOAL ", goal)
            moveit_commander.roscpp_initialize(sys.argv)
            self.gripper = armpy.gripper.Gripper()
            self.arm = armpy.arm.Arm()
            
            if file:
                path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/joint_states.yaml")
                with open(path, 'r') as file:
                    self.goal = yaml.safe_load(file)[goal]
                self.pos = self.goal['position'][0:7]
                print("POSITION", self.pos)
                # self.pos = [self.goal['X'], self.goal['Y'], self.goal['Z'], self.goal['ThetaX'], self.goal['ThetaY'], self.goal['ThetaZ']]
            else:
                self.pos = goal


        except:
            traceback.print_exc()
            pass

    def move(self):

        #target = [-5.113821632562219, 3.2215010684137293, 3.2468996357298154, 0.9070291911761929, 4.402456417991775, 4.194839575202506, 5.160864242689597]
        #self.arm.move_to_joint_pose(target)
        plan = self.arm.plan_joint_pos(self.pos)
        if plan != None:
            self.arm.move_robot(plan, True)


        # rospy.wait_for_service("/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory")
        # add_service = rospy.ServiceProxy("/j2s7s300_driver/in/add_pose_to_Cartesian_trajectory", kinova_msgs.srv.AddPoseToCartesianTrajectory)
        # add_service(*self.pos)

        # add_service.close()
       
        print("MOVED TO GOAL")
    
    def open_gripper(self):
        self.gripper.open()

    def close_gripper(self):
        self.gripper.close()
