#!/usr/bin/env python3
# How to run: 
#   - This should be launched with init_goals.launch
#   - Pass the goal name as an argument goal_name (e.g. goal_name:="goal1")
#   - Then, teleoperate the arm to a goal position and it will be saved to the yaml file

import rospy
import armpy.gen2_teleop
# import teleop_lib.plugins.user_cmd
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np
import yaml
import sys
import direct_control 

def eef_callback(data, GOAL_TO_SET):

    with open('/home/mavis/catkin_ws/src/hri_preferences_study/config/eef_goals.yaml', 'r') as file:
        goals_doc = yaml.safe_load(file)

    goals_doc[GOAL_TO_SET] = {'position':{'x': data.pose.position.x, 'y': data.pose.position.y, 'z':data.pose.position.z}, \
    'orientation':{'x': data.pose.orientation.x, 'y': data.pose.orientation.y, 'z': data.pose.orientation.z, 'w': data.pose.orientation.w}}

    with open("/home/mavis/catkin_ws/src/hri_preferences_study/config/eef_goals.yaml", "w") as f:
        yaml.dump(goals_doc, f)

def joint_callback(data, GOAL_TO_SET):

    with open('/home/mavis/catkin_ws/src/hri_preferences_study/config/jointstates_goals.yaml', 'r') as file:
        goals_doc = yaml.safe_load(file)

    joints = list(data.position)
    goals_doc[GOAL_TO_SET] = joints[5:12]

    with open("/home/mavis/catkin_ws/src/hri_preferences_study/config/jointstates_goals.yaml", "w") as f:
        yaml.dump(goals_doc, f)



if __name__ == "__main__":
    print("INIT GOALS")
    try: 
        GOAL_TO_SET = str(sys.argv[1])

        rospy.init_node("set_goals", anonymous=True)
        direct_controller = direct_control.Direct_Control()
        rospy.Subscriber("/j2s7s300_driver/out/tool_pose", PoseStamped, eef_callback, GOAL_TO_SET) # record end effector position
        rospy.Subscriber("/joint_states", JointState, joint_callback, GOAL_TO_SET) # record joint states
        rospy.sleep(0.01)
        rospy.spin()
    except:
        pass


    

