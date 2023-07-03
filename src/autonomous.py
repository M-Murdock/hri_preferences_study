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
                print("position loaded")
            else:
                self.pos = goal


        except:
            traceback.print_exc()
            pass

    def move(self):

        plan = self.arm.plan_joint_pos(self.pos)
        if plan != None:
            self.arm.move_robot(plan, True)
       
        print("MOVED TO GOAL")
    
    def open_gripper(self):
        self.gripper.open()

    def close_gripper(self):
        self.gripper.close()
