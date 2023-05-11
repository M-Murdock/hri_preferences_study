#!/usr/bin/env python3

import moveit_commander
import rospy
import sys
from geometry_msgs.msg import Pose
import yaml
import armpy.arm
import traceback

def main():
    try:
        rospy.init_node('autonomous', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        arm = armpy.arm.Arm()

        goal_dict = None
        GOAL_NAME = 'goal2'

        with open('/home/mavis/catkin_ws/src/hri_preferences_study/config/goals.yaml', 'r') as file:
            output = yaml.safe_load(file)
            # read in goals from yaml file
            goal_dict = output.get(GOAL_NAME) 


        pos = Pose()
        pos.position.x = goal_dict.get('position')['x'] 
        pos.position.y = goal_dict.get('position')['y']
        pos.position.z = goal_dict.get('position')['z']

        pos.orientation.x = goal_dict.get('orientation')['x']
        pos.orientation.y = goal_dict.get('orientation')['y']
        pos.orientation.z = goal_dict.get('orientation')['z']
        pos.orientation.w = goal_dict.get('orientation')['w']

        # print(pos)

        arm.move_to_ee_pose(pos) # Move to specified end effector position
    except:
        traceback.print_exc()
        pass

    # p = arm.group.get_current_pose()
    # p.pose.position.x -= 0.01
    # print(p)

if __name__ == "__main__":
    main()