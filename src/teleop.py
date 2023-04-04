#!/usr/bin/env python3
import rospy
from kinova_msgs.msg import KinovaPose
import numpy as np
# import shared_control 
# from shared_control.policies import FixedPolicy

global last_commands

class FixedPolicy:
    def __init__(self, action):
        self._action = action

    def get_q_value(self, x, a):
        return np.dot(a, self._action)
    
    def get_q_values(self, x, a):
        return np.dot(a, self._action)

def callback(data):

    global last_commands
    delta_commands = [0,0,0]
    action = [0,0,0]

    if last_commands[0] == None:
        last_commands[0] = data.X
    elif not (last_commands[0] == data.X): # if the user enters a command 
        delta_commands[0] = last_commands[0]-data.X
        last_commands[0] = data.X
        

    if last_commands[1] == None:
        last_commands[1] = data.Y
    elif not (last_commands[1] == data.Y): 
        delta_commands[1] = last_commands[1]-data.Y
        last_commands[1] = data.Y
        

    if last_commands[2] == None:
        last_commands[2] = data.Z
    elif not (last_commands[2] == data.Z): 
        delta_commands[2] = last_commands[2]-data.Z
        last_commands[2] = data.Z 


    max_index = np.argmax(np.abs(delta_commands))
    if last_commands[max_index] > 0:
        action[max_index] = 1 # check whether the x, y, or z command has the greatest absolute value
    else:
        action[max_index] = -1


    if not action == [0, 0, 0]:
        print(action)


def listener():
  rospy.init_node('listener', anonymous=True)
  rospy.Subscriber("/j2s7s300_driver/out/cartesian_command", KinovaPose, callback)
  
  rospy.spin()

if __name__ == '__main__':
    global last_commands
    last_commands = [None, None, None]

    # action_space =((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)) # up, down, left, right
    
    # policies = [FixedPolicy((1, 0, 0)), FixedPolicy((0, 1, 0)), FixedPolicy((0, 0, 1))]

    # pred = shared_control.predictors.MaxEntPredictor(policies)
    # policy = shared_control.policies.SharedAutoPolicy(policies, action_space)

    listener()