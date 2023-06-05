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

class Autonomous:
    def __init__(self):
        try:
            moveit_commander.roscpp_initialize(sys.argv)
            self.arm = armpy.arm.Arm()
            # to set the goals, move arm to desired position, then `rostopic echo /joint_states`:
            self.goal = [0.5080172525610042, 2.3989328993659362, 3.810842904997834, 2.0779030778687164, 4.201798148814774, 3.874152103063415, 6.318032770043272]
            # self.goal2 = [0.7520974644948796, 2.5134703978441735, 4.0290772384210305, 2.820384000825374, 5.131428855793169, 3.9699422790480394, -1.2558463689082726]
            self.move()

        except:
            traceback.print_exc()
            pass

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
 
