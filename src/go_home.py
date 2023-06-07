#!/usr/bin/env python3

import moveit_commander
import rospy
import armpy.arm
import traceback
import sys

# - 4.941741478726749
# - 2.8438911911352855
# - 6.2800784634519
# - 0.7579889760373799
# - 4.626885530426605
# - 4.489698113193354
# - -1.2493087079062917
if __name__ == "__main__":
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        arm = armpy.arm.Arm()
        goal = [4.941741478726749, 2.8438911911352855, 6.2800784634519, 0.7579889760373799, 4.626885530426605, 4.489698113193354, -1.2493087079062917]
        
        # to set the goals, move arm to desired position, then `rostopic echo /joint_states`:
        plan = arm.plan_pose(goal, is_joint_pos=True) # True if joint states. False if eef
        if plan != None:
            arm.move_robot(plan, wait=True)

    except:
        traceback.print_exc()
        pass