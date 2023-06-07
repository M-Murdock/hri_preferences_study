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
import yaml

def move(self):
        plan = self.arm.plan_pose(self.goal, is_joint_pos=True) # True if joint states. False if eef
        if plan != None:
            self.arm.move_robot(plan, wait=True)
            self.save_trajectory("test_trajectory", plan)

def save_trajectory(self, name, plan):
    filename = "trajectories/"+name+".pkl"
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        pickle.dump(plan, f)

def load_trajectory(self, name):
    filename = "trajectories/"+name+".pkl"
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
    with open (path, "rb") as f:
        plan = pickle.load(f)
    return plan 