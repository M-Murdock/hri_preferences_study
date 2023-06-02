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


# class BasicLogger:
#     def __init__(self, data_dir, config):
#         self._file = open(os.path.join(data_dir, "log.txt"), "w")

#     def start(self):
#         self._file.write("started")

#     def stop(self):
#         self._file.write("stopped")
#         self._file.close()

# async def log_trial(config, status_cb):
#     # do some setup stuff
#     await setup()

#     # start the actual trial
#     with RunLogging(config):
#         await run_trial()

#     # finalize + get ready for next trial
#     await cleanup()


async def run_autonomy_level(config, status_cb):
    print(config["Condition"])
    with RunLogging(config):
        if config["Condition"] == "Autonomous":
            autonomous_controller = autonomous.Autonomous()
        if config["Condition"] == "Shared":
            shared_controller = shared_control.Shared_Control()
        if config["Condition"] == "Teleop":
            direct_controller = direct_control.Direct_Control()
        rospy.spin()
            
def main():
    rospy.init_node("gui", anonymous=True)
    root = tkinter.Tk()
    runner = study_runner.StudyRunner(root, run_autonomy_level)
    runner.add_config_frame(ConditionConfigFrame, "Condition")
    
    logging_frame = runner.add_config_frame(LoggingFrame, "Logging")
    logging_frame.add_logger_frame(RosbagRecorderConfigFrame)
    logging_frame.add_logger("recorder", get_rosbag_recorder)

    study_runner.runner.main(root)


if __name__ == "__main__":
    main()
