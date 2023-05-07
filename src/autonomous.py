#!/usr/bin/env python3

import moveit_commander
import rospy
import sys
from geometry_msgs.msg import Pose

import armpy.arm

def main():
    rospy.init_node('autonomous', anonymous=True)


    pos1 = Pose()
    pos1.position.x = 0.9068138748184399
    pos1.position.y = -0.11917496962086757
    pos1.position.z = 0.7814014833345678

    pos1.orientation.x = -0.971766634611512
    pos1.orientation.y = -0.21283734820995873
    pos1.orientation.z = 0.04404968268430674
    pos1.orientation.w = 0.0918122895814124 


    pos2 = Pose()
    pos2.position.x = 0.5670415227984527
    pos2.position.y = 0.19374786506227307
    pos2.position.z = 0.7925633906833817

    pos2.orientation.x = -0.8528981286648915
    pos2.orientation.y = -0.5121313297775412
    pos2.orientation.z = 0.06778568719424277
    pos2.orientation.w = 0.07544126053969476 


    pos3 = Pose()
    pos3.position.x = 0.9257684152970672
    pos3.position.y = 0.27870213111860737
    pos3.position.z = 0.7619229772198917

    pos3.orientation.x = -0.9003275364358055
    pos3.orientation.y = -0.4225090486811876
    pos3.orientation.z = 0.06655115425059198
    pos3.orientation.w = 0.08041998996434441 


    moveit_commander.roscpp_initialize(sys.argv)

    arm = armpy.arm.Arm()
    # p = arm.group.get_current_pose()
    # p.pose.position.x -= 0.01
    # print(p)
    arm.move_to_ee_pose(pos3) # Move to specified end effector position
    # arm.move_to_ee_pose(pos3)

if __name__ == "__main__":
    main()