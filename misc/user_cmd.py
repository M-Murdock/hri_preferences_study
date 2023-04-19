# this is actually stored in teleop_lib/plugins. This is just a copy for safekeeping.


#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from teleop_lib.gui.teleop_config_frame import TeleopConfigFrame, get_teleop_info
import yaml
import teleop_lib.input_profile
import sensor_msgs.msg

class User_Action:
    def __init__(self):
        with open("/home/mavis/catkin_ws/src/teleop_lib/data/XYMode.yaml") as f:
            self.cfg = yaml.safe_load(f)


        self.joy = sensor_msgs.msg.Joy(axes=[0.3, -3], buttons=[1, 0])
        self.mode = teleop_lib.input_profile.build_profile(self.cfg)
        self.cmd = None
        # rospy.init_node("listener_test", anonymous=True)

    def listen(self):
        rospy.loginfo("Listening")
        # sub = rospy.Subscriber("joy", sensor_msgs.msg.Joy, self.joy_cb, queue_size=1)
        # rospy.spin()
        self.input = rospy.wait_for_message("/joy", sensor_msgs.msg.Joy)
        self.cmd = self.mode.process_input(self.input)
        # print(self.cmd)
        return self.cmd

    # def joy_cb(self, msg):
    #     self.cmd = self.mode.process_input(msg)
    #     print(self.cmd)
 
def main():
    
    action=User_Action()
    # action.listen()
    # print(action.cmd)

if __name__ == "__main__":
    main()
    