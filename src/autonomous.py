#!/usr/bin/env python3

import moveit_commander
import rospy
import sys
from geometry_msgs.msg import Pose
import yaml
import armpy.arm

def main():
    rospy.init_node('autonomous', anonymous=True)

    pos1_dict = None
    pos2_dict = None
    pos3_dict = None

    with open('/home/mavis/catkin_ws/src/hri_preferences_study/config/goals.yaml', 'r') as file:
        output = yaml.safe_load(file)

        # read in goals from yaml file
        pos1_dict = output.get('goal1') 
        pos2_dict = output.get('goal2')
        pos3_dict = output.get('goal3')


    pos1 = Pose()
    pos1.position.x = pos1_dict.get('position')['x'] 
    pos1.position.y = pos1_dict.get('position')['y']
    pos1.position.z = pos1_dict.get('position')['z']

    pos1.orientation.x = pos1_dict.get('orientation')['x']
    pos1.orientation.y = pos1_dict.get('orientation')['y']
    pos1.orientation.z = pos1_dict.get('orientation')['z']
    pos1.orientation.w = pos1_dict.get('orientation')['w']


    pos2 = Pose()
    pos2.position.x = pos2_dict.get('position')['x'] 
    pos2.position.y = pos2_dict.get('position')['y']
    pos2.position.z = pos2_dict.get('position')['z']

    pos2.orientation.x = pos2_dict.get('orientation')['x']
    pos2.orientation.y = pos2_dict.get('orientation')['y']
    pos2.orientation.z = pos2_dict.get('orientation')['z']
    pos2.orientation.w = pos2_dict.get('orientation')['w']


    pos3 = Pose()
    pos3.position.x = pos3_dict.get('position')['x'] 
    pos3.position.y = pos3_dict.get('position')['y']
    pos3.position.z = pos3_dict.get('position')['z']

    pos3.orientation.x = pos3_dict.get('orientation')['x']
    pos3.orientation.y = pos3_dict.get('orientation')['y']
    pos3.orientation.z = pos3_dict.get('orientation')['z']
    pos3.orientation.w = pos3_dict.get('orientation')['w']


    moveit_commander.roscpp_initialize(sys.argv)
    arm = armpy.arm.Arm()
    arm.move_to_ee_pose(pos1) # Move to specified end effector position


    # p = arm.group.get_current_pose()
    # p.pose.position.x -= 0.01
    # print(p)

    # rospy.sleep(1)
    # arm.move_to_ee_pose(pos2) # Move to specified end effector position
    # rospy.sleep(1)
    # arm.move_to_ee_pose(pos3) # Move to specified end effector position
  
    # arm.move_to_ee_pose(pos3)

if __name__ == "__main__":
    main()