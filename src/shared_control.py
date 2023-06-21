#!/usr/bin/env python3
import rospy
import armpy.gen2_teleop
import teleop_lib
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
import sys
from sensor_msgs.msg import Joy
import os
import autonomous


class Shared_Control:
    def __init__(self):
        self.arm = armpy.gen2_teleop.Gen2Teleop(ns="/j2s7s300_driver")

        moveit_commander.roscpp_initialize(sys.argv)

        self.DIST_THRESHOLD = 0.1 # how close the end effector can be to a goal before we predict that goal

        self.pos1_xyz = None
        self.pos2_xyz = None 
        self.pos3_xyz = None

        self.output = None

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/eef_goals.yaml")
        with open(path, 'r') as f:
            self.output = yaml.safe_load(f)
            self.goals_xyz = []
            self.goal_names = []

            # save the x/y/z coordinates of each goal
            for goal in self.output:
                self.goal_names.append(goal)
                self.goals_xyz.append((self.output.get(goal).get('position')['x'], self.output.get(goal).get('position')['y'], self.output.get(goal).get('position')['z']))

        # get the controller mapping
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/XYZMode.yaml")
        with open(path, 'r') as f:
            self.cfg = yaml.safe_load(f)

        self.cmd = None

        self.mode = teleop_lib.input_profile.build_profile(self.cfg)
        rospy.Subscriber("/joy", Joy, self.callback)

        self.run_shared_control()

    def convert_actions(self, actions): # convert action array to the correct format
        for i in range(0, len(actions)):
            if actions[i] > 0:
                actions[i] = 1
            elif actions[i] < 0:
                actions[i] = -1
            else:
                actions[i] = 0
        return actions

    def callback(self, data):
        self.cmd = self.mode.process_input(data).twist

    def run_shared_control(self):

        has_reached_goal = False
        goal = None

        try:
            while not rospy.is_shutdown() and not has_reached_goal:
                print("LISTENING FOR CONTROLLER INPUT")
                # wait for user's command
                position = rospy.wait_for_message('/j2s7s300_driver/out/tool_pose', PoseStamped) # wait for robot's position
                
                if self.cmd == None:
                    continue
                # Store user action, state as numpy arrays
                user_action = [self.cmd.linear.x, self.cmd.linear.y, self.cmd.linear.z]
                state = [position.pose.position.x, position.pose.position.y, position.pose.position.z]
    
                u_h = self.convert_actions(user_action) # convert user's action to correct format

                # ------------------------------
                action_space = [(-1, -1, -1), (-1, -1, 0), (-1, -1, 1), (-1, 0, -1), (-1, 0, 0), (-1, 0, 1), (-1, 1, -1), (-1, 1, 0), (-1, 1, 1), (0, -1, -1), (0, -1, 0), (0, -1, 1), (0, 0, -1), (0, 0, 0), (0, 0, 1), (0, 1, -1), (0, 1, 0), (0, 1, 1), (1, -1, -1), (1, -1, 0), (1, -1, 1), (1, 0, -1), (1, 0, 0), (1, 0, 1), (1, 1, -1), (1, 1, 0), (1, 1, 1)]
                policies = []

                for goal in self.goals_xyz: # create policy for every goal position
                    policies.append(ArmPolicy(goal, action_space))
                
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
                self.arm.set_velocity(merged_action_twist)
                
                # if we're very close to a goal, stop.
                i = 0
                pred_goal = None
                for goal in self.goals_xyz:
                    if math.dist(state, goal) < self.DIST_THRESHOLD: 
                        has_reached_goal = True
                        pred_goal = self.goal_names[i]
                        print("The predicted goal is: ", pred_goal)     
                        i += 1


            print("DONE")
            zero = Twist()
            zero.linear.x = 0
            zero.linear.y = 0
            zero.linear.z = 0
            self.arm.set_velocity(zero)

            rospy.sleep(0.2)

            # once we've predicted the goal, move to the goal position
            auton_arm = autonomous.Autonomous(pred_goal)

            rospy.sleep(0.3)


        except:
            traceback.print_exc()
            pass


    
