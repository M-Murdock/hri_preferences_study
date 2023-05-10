#!/usr/bin/env python3
import rospy
import armpy.gen2_teleop
import teleop_lib.plugins.user_cmd
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import shared_control
from shared_auto import SharedAutoPolicy
from maxent_pred import MaxEntPredictor
from arm_policy import ArmPolicy
from geometry_msgs.msg import Pose
import yaml
import armpy.arm
import moveit_commander
import traceback
import visualization_msgs.msg

def convert_actions(actions): # convert action array to the correct format
    for i in range(0, len(actions)):
        if actions[i] > 0:
            actions[i] = 1
        elif actions[i] < 0:
            actions[i] = -1
        else:
            actions[i] = 0
    return actions


if __name__ == "__main__":
    rospy.init_node("shared_control", anonymous=True)
    arm = armpy.gen2_teleop.Gen2Teleop(ns="/j2s7s300_driver")
    action = teleop_lib.plugins.user_cmd.User_Action() # A plugin that gets the user's velocity command
    auton_arm = armpy.arm.Arm()
    

    DIST_THRESHOLD = 0.2 # how close the end effector can be to a goal before we predict that goal

    pos1_xyz = None
    pos2_xyz = None 
    pos3_xyz = None

    pos1 = None
    pos2 = None
    pos3 = None

    with open('/home/mavis/catkin_ws/src/hri_preferences_study/config/goals.yaml', 'r') as file:
        output = yaml.safe_load(file)

        # read in goals from yaml file
        pos1 = output.get('goal1') 
        pos2 = output.get('goal2')
        pos3 = output.get('goal3')

        pos1_xyz = (pos1.get('position')['x'], pos1.get('position')['y'], pos1.get('position')['z'])
        pos2_xyz = (pos2.get('position')['x'], pos2.get('position')['y'], pos2.get('position')['z'])
        pos3_xyz = (pos3.get('position')['x'], pos3.get('position')['y'], pos3.get('position')['z'])
    # print("Before the publisher")
    # rviz_pub = rospy.Publisher("/visualization_marker", visualization_msgs.msg.Marker, queue_size=10)
    # message1 = visualization_msgs.msg.Marker() 
    # message1.color.a = 1
    # message1.type = visualization_msgs.msg.Marker.SPHERE
    # message1.pose.position.x = pos1_xyz[0]
    # message1.pose.position.y = pos1_xyz[1]
    # message1.pose.position.z = pos1_xyz[2]
    # message1.color.g = 1
    # rviz_pub.publish(message1)
    # import IPython
    # IPython.embed()
    # print("After we publish")

    has_reached_goal = False

    try:
        while not rospy.is_shutdown() and not has_reached_goal:
             # wait for user's command
            position = rospy.wait_for_message('/j2s7s300_driver/out/tool_pose', PoseStamped) # wait for robot's position
            
            if action.cmd == None:
                continue
            # Store user action, state as numpy arrays
            user_action = [action.cmd.twist.linear.x, action.cmd.twist.linear.y, action.cmd.twist.linear.z]
            state = [position.pose.position.x, position.pose.position.y, position.pose.position.z]
 
            u_h = convert_actions(user_action) # convert user's action to correct format

            # ------------------------------
            action_space = [(-1, -1, -1), (-1, -1, 0), (-1, -1, 1), (-1, 0, -1), (-1, 0, 0), (-1, 0, 1), (-1, 1, -1), (-1, 1, 0), (-1, 1, 1), (0, -1, -1), (0, -1, 0), (0, -1, 1), (0, 0, -1), (0, 0, 0), (0, 0, 1), (0, 1, -1), (0, 1, 0), (0, 1, 1), (1, -1, -1), (1, -1, 0), (1, -1, 1), (1, 0, -1), (1, 0, 0), (1, 0, 1), (1, 1, -1), (1, 1, 0), (1, 1, 1)]
            policies = [ArmPolicy(pos1_xyz, action_space), ArmPolicy(pos2_xyz, action_space), ArmPolicy(pos3_xyz, action_space)] # define all our goals
            pred = MaxEntPredictor(policies)
            policy = SharedAutoPolicy(policies, list(range(len(action_space))))

            u_h_index = action_space.index(tuple(u_h)) # remap action to an index

            if (np.array(user_action) == 0).all():
                prob = pred.get_prob()
            else:
                prob = pred.update(state, u_h_index) 
         
            u_r_index = policy.get_action(state, prob)

            q_all = [policy.get_q_value(state, u_h_index) for policy in policies]
            best_actions = [action_space[policy.get_action(state)] for policy in policies]

            print(f"{state} -> {u_h} -> {best_actions} -> {q_all} -> {prob} -> {action_space[u_r_index]}") # For debugging
            u_r = np.array(action_space[u_r_index]) # Get robot's predicted action

            # Merge the two actions
            merged_action = (u_h + u_r) / 2

            # Convert the action vector to a Twist message
            merged_action_twist = Twist()
            merged_action_twist.linear.x = merged_action[0]
            merged_action_twist.linear.y = merged_action[1]
            merged_action_twist.linear.z = merged_action[2]


            # Perform resulting action
            arm.set_velocity(merged_action_twist)

            # moveit_commander.roscpp_initialize(sys.argv)
            
            # if we're very close to a goal, or there's a high likelihood that we've predicted a goal, stop.

            if math.dist(state, pos1_xyz) < DIST_THRESHOLD: 
                has_reached_goal = True
                print("The predicted goal is: GOAL1")

                # pose1 = Pose()
                # pose1.position.x = pos1.get('position')['x'] 
                # pose1.position.y = pos1.get('position')['y']
                # pose1.position.z = pos1.get('position')['z']

                # pose1.orientation.x = pos1.get('orientation')['x']
                # pose1.orientation.y = pos1.get('orientation')['y']
                # pose1.orientation.z = pos1.get('orientation')['z']
                # pose1.orientation.w = pos1.get('orientation')['w']
                # auton_arm.move_to_ee_pose(pose1)

                
            elif math.dist(state, pos2_xyz) < DIST_THRESHOLD: 
                has_reached_goal = True
                # p = auton_arm.group.get_current_pose()
                # p.pose.position.x -= 0.01
                # auton_arm.move_to_ee_pose(p)
                print("The predicted goal is: GOAL2")
            elif math.dist(state, pos3_xyz) < DIST_THRESHOLD:
                has_reached_goal = True
                # p = auton_arm.group.get_current_pose()
                # p.pose.position.x -= 0.01
                # auton_arm.move_to_ee_pose(p)
                print("The predicted goal is: GOAL3")
     
    except:
        traceback.print_exc()
        pass


    
