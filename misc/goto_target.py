#!/usr/bin/env python3

import moveit_commander
import rospy
import sys
from geometry_msgs.msg import Pose

import armpy.arm

def main():
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
    rospy.init_node('arm_test', anonymous=True)

    arm = armpy.arm.Arm()
    # p = arm.group.get_current_pose()
    # p.pose.position.x -= 0.1
    arm.move_to_ee_pose(pos3)

if __name__ == "__main__":
    main()



# Position 1: 
# header: 
#   seq: 0
#   stamp: 
#     secs: 1681658956
#     nsecs: 181579113
#   frame_id: "base_link"
# pose: 
#   position: 
#     x: 0.9068138748184399
#     y: -0.11917496962086757
#     z: 0.7814014833345678
#   orientation: 
#     x: -0.971766634611512
#     y: -0.21283734820995873
#     z: 0.04404968268430674
#     w: 0.0918122895814124

# Position 2:
# header: 
#   seq: 0
#   stamp: 
#     secs: 1681659072
#     nsecs: 529951333
#   frame_id: "base_link"
# pose: 
#   position: 
#     x: 0.5670415227984527
#     y: 0.19374786506227307
#     z: 0.7925633906833817
#   orientation: 
#     x: -0.8528981286648915
#     y: -0.5121313297775412
#     z: 0.06778568719424277
#     w: 0.07544126053969476


# Position 3:
# header: 
#   seq: 0
#   stamp: 
#     secs: 1681659099
# Position 3:
# header: 
#   seq: 0
#   stamp: 
#     secs: 1681659099
#     nsecs: 845754384
#   frame_id: "base_link"
# pose: 
#   position: 
#     x: 0.9257684152970672
#     y: 0.27870213111860737
#     z: 0.7619229772198917
#   orientation: 
#     x: -0.9003275364358055
#     y: -0.4225090486811876
#     z: 0.06655115425059198
#     w: 0.08041998996434441
#     nsecs: 845754384
#   frame_id: "base_link"
# pose: 
#   position: 
#     x: 0.9257684152970672
#     y: 0.27870213111860737
#     z: 0.7619229772198917
#   orientation: 
#     x: -0.9003275364358055
#     y: -0.4225090486811876
#     z: 0.06655115425059198
#     w: 0.08041998996434441