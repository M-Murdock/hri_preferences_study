#!/usr/bin/env python3

import moveit_commander
import rospy
import sys
from geometry_msgs.msg import Pose

import armpy.arm

def main():
    rospy.init_node('autonomous', anonymous=True)


    pos1 = Pose()
    pos1.position.x = -0.0987742692232132 # NOTE: I should make a yaml file or something to hold these values
    pos1.position.y = -0.5367502570152283
    pos1.position.z = 0.08353482186794281

    pos1.orientation.x = 0.45725175738334656
    pos1.orientation.y = 0.5576337575912476
    pos1.orientation.z = -0.3721840977668762
    pos1.orientation.w = 0.5843324065208435


    pos2 = Pose()
    pos2.position.x = 0.3203457295894623
    pos2.position.y = -0.4727548360824585
    pos2.position.z = 0.057524073868989944

    pos2.orientation.x = 0.21972160041332245
    pos2.orientation.y = 0.6870991587638855
    pos2.orientation.z = -0.13112148642539978
    pos2.orientation.w = 0.68001788854599


    pos3 = Pose()
    pos3.position.x = 0.4564712643623352
    pos3.position.y = -0.23805353045463562
    pos3.position.z = 0.09125393629074097

    pos3.orientation.x = 0.040665097534656525
    pos3.orientation.y = 0.7207533717155457
    pos3.orientation.z = 0.044518694281578064
    pos3.orientation.w = 0.6905642747879028 


    moveit_commander.roscpp_initialize(sys.argv)

    arm = armpy.arm.Arm()
    # p = arm.group.get_current_pose()
    # p.pose.position.x -= 0.01
    # print(p)
    arm.move_to_ee_pose(pos1) # Move to specified end effector position
    # rospy.sleep(1)
    # arm.move_to_ee_pose(pos2) # Move to specified end effector position
    # rospy.sleep(1)
    # arm.move_to_ee_pose(pos3) # Move to specified end effector position
  
    # arm.move_to_ee_pose(pos3)

if __name__ == "__main__":
    main()