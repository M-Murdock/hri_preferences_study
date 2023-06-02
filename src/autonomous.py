#!/usr/bin/env python3

import moveit_commander
import rospy
import sys
from geometry_msgs.msg import Pose
import yaml
import armpy.arm
import traceback

class Autonomous:
    def __init__(self):
        try:
            moveit_commander.roscpp_initialize(sys.argv)
            self.arm = armpy.arm.Arm()

            self.goal_dict = None
            GOAL_NAME = 'goal1'

            with open('/home/mavis/catkin_ws/src/hri_preferences_study/config/goals.yaml', 'r') as file:
                output = yaml.safe_load(file)
                # read in goals from yaml file
                self.goal_dict = output.get(GOAL_NAME) 

            self.move()

        except:
            traceback.print_exc()
            pass

    def move(self):
        # self.pos = self.arm.get_current_ee_pose()
        # self.pos.pose.position.x += 0.1
        self.pos = Pose()
        self.pos.position.x = self.goal_dict.get('position')['x'] 
        self.pos.position.y = self.goal_dict.get('position')['y']
        self.pos.position.z = self.goal_dict.get('position')['z']

        self.pos.orientation.x = self.goal_dict.get('orientation')['x']
        self.pos.orientation.y = self.goal_dict.get('orientation')['y']
        self.pos.orientation.z = self.goal_dict.get('orientation')['z']
        self.pos.orientation.w = self.goal_dict.get('orientation')['w']

        self.arm.move_to_ee_pose(self.pos) # Move to specified end effector position
        
