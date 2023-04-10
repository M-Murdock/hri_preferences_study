#!/usr/bin/env python3
import rospy
from kinova_msgs.msg import KinovaPose
import numpy as np
# import shared_control 
# from shared_control.policies import FixedPolicy

global last_positions 
global start_position

class FixedPolicy:
    def __init__(self, action):
        self._action = action

    def get_q_value(self, x, a):
        return np.dot(a, self._action)
    
    def get_q_values(self, x, a):
        return np.dot(a, self._action)

def callback(data):

    global start_position
    global last_positions # last X/Y/Z position of the end effector
    delta_positions = [0,0,0] #  change in the X/Y/Z position of the end effector 
    action = [0,0,0] # current action: represented as a 1 or -1 in the X/Y/Z direction

    # get the starting position of the end effector
    if start_position == [None, None, None]:
        start_position = [data.X, data.Y, data.Z]

    delta_positions[0] = last_positions[0] - data.X 
    last_positions[0] = data.X

    delta_positions[1] = last_positions[1] - data.Y
    last_positions[1] = data.Y

    delta_positions[2] = last_positions[2] - data.Z
    last_positions[2] = data.Z

    if delta_positions == [0, 0, 0]: # if there is no command being given
        action = [0, 0, 0]
        return 

    # if a command is being given, is it in the x, y, or z direction?
    max_index = np.argmax(np.abs(delta_positions))
    
    if delta_positions[max_index] > 0:
        action[max_index] = 1 # check whether the x, y, or z command has the greatest absolute value
    else:
        action[max_index] = -1


    print(action)


def listener():
  rospy.init_node('listener', anonymous=True)
  rospy.Subscriber("/j2s7s300_driver/out/cartesian_command", KinovaPose, callback)
  
  rospy.spin()

if __name__ == '__main__':
    global last_positions 
    global start_position 
    start_position = [None, None, None]
    last_positions = [0, 0, 0]

    # action_space =((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)) # up, down, left, right
    
    # policies = [FixedPolicy((1, 0, 0)), FixedPolicy((0, 1, 0)), FixedPolicy((0, 0, 1))]

    # pred = shared_control.predictors.MaxEntPredictor(policies)
    # policy = shared_control.policies.SharedAutoPolicy(policies, action_space)

    listener()