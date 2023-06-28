import os
import rospy
import sys
import tkinter
from webcam import Webcam



ROSBAG_RECORDER_CONFIG_NAME = 'rosbag'

class VideoRecorder:
    def __init__(self, log_dir, config):
        self.filename = os.path.join(log_dir, 'trial_data') # rosbag exec adds .bag extension
        self.webcam = Webcam(log_dir)
        
    def start(self):
        print("running...")
        self.webcam.start()

    def stop(self):
        self.webcam.quit()
        print("stopping...")
    


def get_video_recorder(log_dir, config):
    return VideoRecorder(log_dir, config)

