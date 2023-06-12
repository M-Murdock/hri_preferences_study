#!/usr/bin/env python3

# roslaunch example taken from here: http://wiki.ros.org/roslaunch/API%20Usage


import asyncio
import os
import study_runner
from  study_runner.frames.logging import LoggingFrame, RunLogging
from study_runner.frames.loggers.rosbag_recorder import RosbagRecorder, RosbagRecorderConfigFrame, get_rosbag_recorder
import tkinter
import roslaunch
import rospy
import direct_control
import shared_control
import autonomous
import sys
import yaml
from arm_trajectory import Trajectory

global GOAL_TO_SET 

class ConditionConfigFrame(tkinter.Frame):
    def __init__(self, parent, _):
        super().__init__(parent)

        self._entry = tkinter.StringVar(self, "Autonomous")  # Create a variable for strings, and initialize the variable
        tkinter.Radiobutton(self, text="Autonomous", variable=self._entry, value="Autonomous").pack(anchor='w')
        tkinter.Radiobutton(self, text="Shared", variable=self._entry, value="Shared").pack(anchor='w')
        tkinter.Radiobutton(self, text="Teleop", variable=self._entry, value="Teleop").pack(anchor='w')
        print(self._entry)

    def get_config(self):
        return {"Condition": self._entry.get()}
    def set_state(self, state):
        self._entry.value = state


class SetGoalFrame(tkinter.Frame):
    def __init__(self, parent, _):
        super().__init__(parent)

        # grab all the goals from the goal yaml files
        with open('/home/mavis/catkin_ws/src/hri_preferences_study/config/eef_goals.yaml', 'r') as file:
            OPTIONS = list(yaml.safe_load(file))

        self._entry = tkinter.StringVar(self)
        self._entry.set(OPTIONS[0]) # default value

        question_menu = tkinter.OptionMenu(self, self._entry, *OPTIONS)
        question_menu.pack()

    def get_config(self):
        return {"Goal_Name": self._entry.get()}
    def set_state(self, state):
        self._entry.value = state

# override the study runner to record arm trajectories
class HRIStudyRunner(study_runner.StudyRunner):
    def __init__(self, root, trial_runner):
        study_runner.StudyRunner.__init__(self, root, trial_runner)
        self.home_button = tkinter.Button(self.start_frame,
                                           text="Return Home",
                                           command=self._home_button_callback)
        self.home_button.grid(row=1, column=1, columnspan=2, sticky="EW")

    def _cancel_button_callback(self):
        # stop recording the arm poses
        try:
            self.trajectory_recorder.stop_recording()
        except:
            pass
        study_runner.StudyRunner._cancel_button_callback(self)

    def _start_button_callback(self):
        # start recording the arm's trajectory
        self.trajectory_recorder = Trajectory()
        self.trajectory_recorder.record()
        study_runner.StudyRunner._start_button_callback(self)

    def _home_button_callback(self):
        try:
            # move back to home position by executing reverse trajectory
            print("homing...")
            self.trajectory_recorder.execute_reverse_trajectory()
        except:
            pass

async def run_autonomy_level(config, status_cb):
    global GOAL_TO_SET

    print(config["Condition"])
    with RunLogging(config):
        if config["Condition"] == "Autonomous":
            autonomous_controller = autonomous.Autonomous(config["Goal_Name"])
        if config["Condition"] == "Shared":
            shared_controller = shared_control.Shared_Control()
        if config["Condition"] == "Teleop":
            direct_controller = direct_control.Direct_Control()
        rospy.spin()
            
def main():

    rospy.init_node("gui", anonymous=True)
    root = tkinter.Tk()
    root.geometry("400x400+700+300")
    runner = HRIStudyRunner(root, run_autonomy_level)
    runner.add_config_frame(ConditionConfigFrame, "Condition")

    runner.add_config_frame(SetGoalFrame, "Goal")
    
    logging_frame = runner.add_config_frame(LoggingFrame, "Logging")
    logging_frame.add_logger_frame(RosbagRecorderConfigFrame)
    logging_frame.add_logger("recorder", get_rosbag_recorder)

    study_runner.runner.main(root)


if __name__ == "__main__":
    main()
