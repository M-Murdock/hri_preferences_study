#!/usr/bin/env python3
# How to run: 
#   - This should be launched with init_goals.launch
#   - Pass the goal name as an argument goal_name (e.g. goal_name:="goal1")
#   - Then, teleoperate the arm to a goal position and it will be saved to the yaml file

import rospy
import armpy.gen2_teleop
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np
import yaml
import sys
import direct_control 
import tkinter 
import study_runner
import os

def eef_callback(data, GOAL_TO_SET):

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/eef_goals.yaml")
    with open(path, 'r') as f:
        goals_doc = yaml.safe_load(f)

    goals_doc[GOAL_TO_SET] = {'position':{'x': data.pose.position.x, 'y': data.pose.position.y, 'z':data.pose.position.z}, \
    'orientation':{'x': data.pose.orientation.x, 'y': data.pose.orientation.y, 'z': data.pose.orientation.z, 'w': data.pose.orientation.w}}

    with open(path, "w") as f:
        yaml.dump(goals_doc, f)

def joint_callback(data, GOAL_TO_SET):

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/jointstates_goals.yaml")
    with open(path, 'r') as f:
        goals_doc = yaml.safe_load(f)

    joints = list(data.position)
    goals_doc[GOAL_TO_SET] = joints[5:12]

    with open(path, "w") as f:
        yaml.dump(goals_doc, f)

class SetGoalFrame(tkinter.Frame):

    def __init__(self, root):
        super().__init__(root)
        self._root = root
        self.grid(sticky="NSEW")
        root.rowconfigure(0, weight=1)
        root.columnconfigure(0, weight=1)

        # set up bottom frame with buttons
        self.start_frame = tkinter.Frame(self)
        self.start_frame.grid(row=1, column=0, columnspan=2, sticky='esw', padx=2, pady=2)


        self.start_button = tkinter.Button(self.start_frame,
                                           text="Record position",
                                           command=self._record_button_callback)
        self.start_button.grid(row=0, column=2, sticky="EW")

        self.quit_button = tkinter.Button(
            self.start_frame, text="Quit", command=self._quit_button_callback)
        self.quit_button.grid(row=1, column=1, sticky='ne', padx=5)
        self.quit_button = tkinter.Button(
            self.start_frame, text="Reset", command=self._reset_button_callback)
        self.quit_button.grid(row=1, column=0, sticky='ne', padx=5)

        # set up top frame with goal name
        self.goal_label = tkinter.Label(
            self.start_frame, text="Goal Name" )
        self.goal_label.grid(row=0, column=0, sticky="W")

        self.goal_name = tkinter.Entry(
            self.start_frame, bd=2)
        self.goal_name.grid(row=0, column=1, sticky="E", pady=5)

    def _reset_button_callback(self):
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/eef_goals.yaml")
        with open(path, "w") as f:
            yaml.dump({}, f)

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/jointstates_goals.yaml")
        with open(path, "w") as f:
            yaml.dump({}, f)

    def _quit_button_callback(self):
        self._root.quit()

    def _record_button_callback(self):
        print("recording")
        eef_callback(rospy.wait_for_message("/j2s7s300_driver/out/tool_pose", PoseStamped), self.goal_name.get())
        joint_callback(rospy.wait_for_message("/joint_states", JointState), self.goal_name.get())

if __name__ == "__main__":
    print("INIT GOALS")
    
    try: 
        rospy.init_node("set_goals", anonymous=True)

        direct_controller = direct_control.Direct_Control()

        root = tkinter.Tk()
        recorder = SetGoalFrame(root)
        study_runner.runner.main(root)

        rospy.spin()
    except:
        pass


    

