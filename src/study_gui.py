#!/usr/bin/env python3


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
import kinova_msgs.srv
import time

arm = None

class ConditionConfigFrame(tkinter.Frame):
    def __init__(self, parent, _):
        super().__init__(parent)

        self._entry = tkinter.StringVar(self, "Autonomous")  # Create a variable for strings, and initialize the variable
        tkinter.Radiobutton(self, text="Autonomous", variable=self._entry, value="Autonomous").pack(anchor='w')
        tkinter.Radiobutton(self, text="Shared", variable=self._entry, value="Shared").pack(anchor='w')
        tkinter.Radiobutton(self, text="Teleop", variable=self._entry, value="Teleop").pack(anchor='w')


    def get_config(self):
        return {"Condition": self._entry.get()}
    def set_state(self, state):
        self._entry.value = state


class SetGoalFrame(tkinter.Frame):
    def __init__(self, parent, _):
        super().__init__(parent)

        # grab all the goals from the goal yaml files
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/eef_goals.yaml")
        with open(path, 'r') as file:
            OPTIONS = list(yaml.safe_load(file))

        self._entry = tkinter.StringVar(self)
        self._entry.set(OPTIONS[0]) # default value

        question_menu = tkinter.OptionMenu(self, self._entry, *OPTIONS)
        question_menu.pack()

    def get_config(self):
        return {"Goal_Name": self._entry.get()}
    def set_state(self, state):
        self._entry.value = state

class SetController(tkinter.Frame):
    def __init__(self, parent, _):
        super().__init__(parent)

        OPTIONS = ["keyboard", "web", "xbox"]
        self._entry = tkinter.StringVar(self)
        self._entry.set(OPTIONS[0]) # default value

        controller_menu = tkinter.OptionMenu(self, self._entry, *OPTIONS)
        controller_menu.pack()

    def get_config(self):
        return {"Controller_Name": self._entry.get()}
    def set_state(self, state):
        self._entry.value = state



# override the study runner to record arm trajectories
class HRIStudyRunner(study_runner.StudyRunner):
    def __init__(self, root, trial_runner):
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/study_config.yaml")
        study_runner.StudyRunner.__init__(self, root, trial_runner, initial_config_file=path)
        self.save_config_button.destroy()

        self.open_gripper = tkinter.Button(self.start_frame,
                                           text="Open Gripper",
                                           command=self.gripper_open_button_callback)
        self.open_gripper.grid(row=0, column=1, columnspan=1, sticky="EW")


        self.close_gripper = tkinter.Button(self.start_frame,
                                           text="Close Gripper",
                                           command=self.gripper_close_button_callback)
        self.close_gripper.grid(row=1, column=1, columnspan=1, sticky="EW")


        self.start_pos_button = tkinter.Button(self.start_frame,
                                           text="Go To Start Position",
                                           command=self._start_pos_button_callback)
        self.start_pos_button.grid(row=1, column=0, columnspan=1, sticky="EW")

        self.estop_button = tkinter.Button(self.start_frame,
                                           text="Estop", bg = "red",
                                           command=self._estop_button_callback)
        self.estop_button.grid(row=2, column=0, columnspan=1, sticky="EW")

        self.home_button = tkinter.Button(self.start_frame,
                                           text="Home", bg = "yellow",
                                           command=self._home_button_callback)
        self.home_button.grid(row=2, column=1, columnspan=1, sticky="EW")

        self.user_id = 0



    def _home_button_callback(self):
        rospy.wait_for_service("/j2s7s300_driver/in/home_arm")
        home_arm = rospy.ServiceProxy("/j2s7s300_driver/in/home_arm", kinova_msgs.srv.HomeArm)
        home_arm()

    def _estop_button_callback(self):
        rospy.wait_for_service("/j2s7s300_driver/in/stop")
        stop_arm = rospy.ServiceProxy("/j2s7s300_driver/in/stop", kinova_msgs.srv.Stop)
        stop_arm()

    def _start_pos_button_callback(self):
        self.autonomous = autonomous.Autonomous([3.1573585672430626, 1.669121033188949, 2.927425233467256, 1.3752290895157453, 4.042685725275227, 4.556754378912677, 7.900009047094323], file=False)
        self.autonomous.move()

    def _cancel_button_callback(self):
        # stop recording the arm poses
        try:
            self.trajectory_recorder.stop_recording()
        except:
            pass
        study_runner.StudyRunner._cancel_button_callback(self)
    
    def _save_config(self): # we don't need to have a save config button
        pass

    def _start_button_callback(self):
        # start recording the arm's trajectory
        self.trajectory_recorder = Trajectory()
        self.trajectory_recorder.record()
        study_runner.StudyRunner._start_button_callback(self)

    def gripper_open_button_callback(self):
        global arm
        arm.open_gripper()

    def gripper_close_button_callback(self):
        global arm
        arm.close_gripper()

class SetUserID(LoggingFrame):
    def __init__(self, parent, initial_config={}):
        LOGGING_CONFIG_NAME = 'logging'
        super().__init__(parent, initial_config)
        initial_config = initial_config.get(LOGGING_CONFIG_NAME, {})
        self._initial_config = initial_config

        self.left_column = tkinter.Frame(self)
        self.left_column.grid(row=0, column=0, sticky='nesw')
        self.left_column.columnconfigure(0, weight=1)

        self.rowconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)


        self.logging_frame = tkinter.LabelFrame(self.left_column, text='User ID')
        base_dir = initial_config.get('base_dir', os.path.expanduser("~"))

        # choose user id
        self.user_id_var = tkinter.StringVar()
        self.user_id_var.set(0)
        self.user_id_entry = tkinter.Entry(
            self.logging_frame, textvariable=self.user_id_var)
        self.user_id_entry.grid(row=2, column=1, sticky="nwe")
        self.user_id_label = tkinter.Label(self.logging_frame, text='')
        self.user_id_label.grid(row=2, column=0, sticky="nw")
 
        self.user_id_auto_var = tkinter.IntVar()
        self.user_id_auto_var.set(False)


        # initialize fields
        self.logging_frame.columnconfigure(1, weight=1)
        self.logging_frame.grid(row=0, column=0, padx=1, pady=1, sticky='nesw')
        self.loggers = []
        self.added_logger_names = []

    def add_logger_frame(self, frame_fn, side='left', **kwargs):
        parent = self.logging_frame
        frame = frame_fn(parent, self._initial_config)
        self.loggers.append(frame)
        return frame

    def _validate_user_id(self, *_):
        pass
    def _update_user_id(self):
        pass

async def run_autonomy_level(config, status_cb):


    with RunLogging(config):
        # global arm 

        if config["Condition"] == "Autonomous":
            autonomous_controller = autonomous.Autonomous(config["Goal_Name"])
            autonomous_controller.close_gripper()
            arm = autonomous_controller
            # autonomous_controller.open_gripper()
            time.sleep(2)
            autonomous_controller.move()
            autonomous_controller.open_gripper()
        if config["Condition"] == "Shared":
            shared_controller = shared_control.Shared_Control(config["Controller_Name"])
            shared_controller.close_gripper()
            arm = shared_controller
            shared_controller.run_shared_control()
            time.sleep(2)
        if config["Condition"] == "Teleop":
            rospy.loginfo("Creating Direct Controller")
            direct_controller = direct_control.Direct_Control(config["Controller_Name"])
            direct_controller.start()
            rospy.loginfo("Successfully Created Direct controller")
            direct_controller.close_gripper()
            # arm = direct_controller
            direct_controller.allow_gripper_control = True
        rospy.spin()

def disable_event():
    pass 

def main():

    rospy.init_node("gui", anonymous=True)
    root = tkinter.Tk()
    root.geometry("400x400+700+300")
    # root.protocol("WM_DELETE_WINDOW", disable_event)

    runner = HRIStudyRunner(root, run_autonomy_level)
    logging_frame = runner.add_config_frame(SetUserID, "Logging")
    logging_frame.add_logger_frame(RosbagRecorderConfigFrame)
    logging_frame.add_logger("recorder", get_rosbag_recorder)

    runner.add_config_frame(ConditionConfigFrame, "Condition")

    runner.add_config_frame(SetGoalFrame, "Goal")

    runner.add_config_frame(SetController, "Controller")
    

    # set the user id (equivalent to the one created by StudyRunner)
    runner.user_id = logging_frame.user_id_var.get()

    study_runner.runner.main(root)


if __name__ == "__main__":
    main()
