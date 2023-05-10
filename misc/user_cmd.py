#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from teleop_lib.gui.teleop_config_frame import TeleopConfigFrame, get_teleop_info
import yaml
import teleop_lib.input_profile
import sensor_msgs.msg


# Listens for Joy input and converts it to velocity commands
class User_Action:
    def __init__(self):

        with open("/home/mavis/catkin_ws/src/hri_preferences_study/config/XYZMode.yaml") as f:
            self.cfg = yaml.safe_load(f)

        self.cmd = None

        self.mode = teleop_lib.input_profile.build_profile(self.cfg)
        rospy.Subscriber("/joy", sensor_msgs.msg.Joy, self.callback)

        # rospy.spin()

    def callback(self, data):
        self.cmd = self.mode.process_input(data)
        # print(self.cmd)


def main():
    rospy.init_node('cmd_listener', anonymous=True)
    action=User_Action()

if __name__ == "__main__":
    main()
    