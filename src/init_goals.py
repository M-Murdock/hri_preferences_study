#!/usr/bin/env python3
# How to run: 
#   - This should be launched with init_goals.launch
#   - Pass the goal name as an argument goal_name (e.g. goal_name:="goal1")
#   - Then, teleoperate the arm to a goal position and it will be saved to the yaml file

import rospy
import armpy.gen2_teleop
import teleop_lib.plugins.user_cmd
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np
import yaml
import sys

def callback(data, goals_doc):
    goals_doc.get(GOAL_TO_SET).get('position')['x'] = data.pose.position.x
    goals_doc.get(GOAL_TO_SET).get('position')['y'] = data.pose.position.y
    goals_doc.get(GOAL_TO_SET).get('position')['z'] = data.pose.position.z

    goals_doc.get(GOAL_TO_SET).get('orientation')['x'] = data.pose.orientation.x
    goals_doc.get(GOAL_TO_SET).get('orientation')['y'] = data.pose.orientation.y
    goals_doc.get(GOAL_TO_SET).get('orientation')['z'] = data.pose.orientation.z
    goals_doc.get(GOAL_TO_SET).get('orientation')['w'] = data.pose.orientation.w

    with open("/home/mavis/catkin_ws/src/hri_preferences_study/config/goals.yaml", "w") as f:
        yaml.dump(goals_doc, f)


if __name__ == "__main__":

    try: 
        GOAL_TO_SET = str(sys.argv[1])
        goals_doc = None

        with open("/home/mavis/catkin_ws/src/hri_preferences_study/config/goals.yaml") as f:
            goals_doc = yaml.safe_load(f)


        rospy.init_node("set_goals", anonymous=True)
        rospy.Subscriber("/j2s7s300_driver/out/tool_pose", PoseStamped, callback, goals_doc)
        rospy.sleep(0.01)
        rospy.spin()
    except:
        pass


    

