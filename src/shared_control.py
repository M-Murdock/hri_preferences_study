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
import sys
from sensor_msgs.msg import Joy


class Shared_Control:
    def __init__(self):
        self.arm = armpy.gen2_teleop.Gen2Teleop(ns="/j2s7s300_driver")
        # self.action = teleop_lib.plugins.user_cmd.User_Action() # A plugin that gets the user's velocity command
        self.auton_arm = armpy.arm.Arm()
        moveit_commander.roscpp_initialize(sys.argv)

        DIST_THRESHOLD = 0.1 # how close the end effector can be to a goal before we predict that goal

        self.pos1_xyz = None
        self.pos2_xyz = None 
        self.pos3_xyz = None

        self.output = None

        with open('/home/mavis/catkin_ws/src/hri_preferences_study/config/goals.yaml', 'r') as file:
            self.output = yaml.safe_load(file)

            # read in goals from yaml file
            self.pos1_xyz = (self.output.get('goal1') .get('position')['x'], self.output.get('goal1') .get('position')['y'], self.output.get('goal1') .get('position')['z'])
            self.pos2_xyz = (self.output.get('goal2').get('position')['x'], self.output.get('goal2').get('position')['y'], self.output.get('goal2').get('position')['z'])
            self.pos3_xyz = (self.output.get('goal3').get('position')['x'], self.output.get('goal3').get('position')['y'], self.output.get('goal3').get('position')['z'])

        with open("/home/mavis/catkin_ws/src/hri_preferences_study/config/XYZMode.yaml") as f:
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
                policies = [ArmPolicy(self.pos1_xyz, action_space), ArmPolicy(self.pos2_xyz, action_space), ArmPolicy(self.pos3_xyz, action_space)] # define all our goals
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
                if math.dist(state, pos1_xyz) < DIST_THRESHOLD: 
                    has_reached_goal = True
                    print("The predicted goal is: GOAL1")     
                    goal = 'goal1'    
                elif math.dist(state, pos2_xyz) < DIST_THRESHOLD: 
                    has_reached_goal = True
                    print("The predicted goal is: GOAL2")
                    goal = 'goal2'
                elif math.dist(state, pos3_xyz) < DIST_THRESHOLD:
                    has_reached_goal = True
                    print("The predicted goal is: GOAL3")
                    goal = 'goal3'


            rospy.sleep(0.1)
            # once we've predicted the goal, move to the goal position
            pos = Pose()
            pos.position.x = self.output.get(goal).get('position')['x'] 
            pos.position.y = self.output.get(goal).get('position')['y']
            pos.position.z = self.output.get(goal).get('position')['z']

            pos.orientation.x = self.output.get(goal).get('orientation')['x']
            pos.orientation.y = self.output.get(goal).get('orientation')['y']
            pos.orientation.z = self.output.get(goal).get('orientation')['z']
            pos.orientation.w = self.output.get(goal).get('orientation')['w']

            # auton_arm.move_to_ee_pose(pos)
                    
        
        except:
            traceback.print_exc()
            pass


    

# if __name__ == "__main__":
#     rospy.init_node("shared_control", anonymous=True)
#     arm = armpy.gen2_teleop.Gen2Teleop(ns="/j2s7s300_driver")
#     action = teleop_lib.plugins.user_cmd.User_Action() # A plugin that gets the user's velocity command
#     auton_arm = armpy.arm.Arm()
#     moveit_commander.roscpp_initialize(sys.argv)
    

#     DIST_THRESHOLD = 0.1 # how close the end effector can be to a goal before we predict that goal

#     pos1_xyz = None
#     pos2_xyz = None 
#     pos3_xyz = None

#     output = None

#     with open('/home/mavis/catkin_ws/src/hri_preferences_study/config/goals.yaml', 'r') as file:
#         output = yaml.safe_load(file)

#         # read in goals from yaml file
#         pos1_xyz = (output.get('goal1') .get('position')['x'], output.get('goal1') .get('position')['y'], output.get('goal1') .get('position')['z'])
#         pos2_xyz = (output.get('goal2').get('position')['x'], output.get('goal2').get('position')['y'], output.get('goal2').get('position')['z'])
#         pos3_xyz = (output.get('goal3').get('position')['x'], output.get('goal3').get('position')['y'], output.get('goal3').get('position')['z'])


#     has_reached_goal = False
#     goal = None

#     try:
#         while not rospy.is_shutdown() and not has_reached_goal:
#              # wait for user's command
#             position = rospy.wait_for_message('/j2s7s300_driver/out/tool_pose', PoseStamped) # wait for robot's position
            
#             if action.cmd == None:
#                 continue
#             # Store user action, state as numpy arrays
#             user_action = [action.cmd.twist.linear.x, action.cmd.twist.linear.y, action.cmd.twist.linear.z]
#             state = [position.pose.position.x, position.pose.position.y, position.pose.position.z]
 
#             u_h = convert_actions(user_action) # convert user's action to correct format

#             # ------------------------------
#             action_space = [(-1, -1, -1), (-1, -1, 0), (-1, -1, 1), (-1, 0, -1), (-1, 0, 0), (-1, 0, 1), (-1, 1, -1), (-1, 1, 0), (-1, 1, 1), (0, -1, -1), (0, -1, 0), (0, -1, 1), (0, 0, -1), (0, 0, 0), (0, 0, 1), (0, 1, -1), (0, 1, 0), (0, 1, 1), (1, -1, -1), (1, -1, 0), (1, -1, 1), (1, 0, -1), (1, 0, 0), (1, 0, 1), (1, 1, -1), (1, 1, 0), (1, 1, 1)]
#             policies = [ArmPolicy(pos1_xyz, action_space), ArmPolicy(pos2_xyz, action_space), ArmPolicy(pos3_xyz, action_space)] # define all our goals
#             pred = MaxEntPredictor(policies)
#             policy = SharedAutoPolicy(policies, list(range(len(action_space))))

#             u_h_index = action_space.index(tuple(u_h)) # remap action to an index

#             if (np.array(user_action) == 0).all():
#                 prob = pred.get_prob()
#             else:
#                 prob = pred.update(state, u_h_index) 
         
#             u_r_index = policy.get_action(state, prob)

#             q_all = [policy.get_q_value(state, u_h_index) for policy in policies]
#             best_actions = [action_space[policy.get_action(state)] for policy in policies]

#             print(f"{state} -> {u_h} -> {best_actions} -> {q_all} -> {prob} -> {action_space[u_r_index]}") # For debugging
#             u_r = np.array(action_space[u_r_index]) # Get robot's predicted action

#             # Merge the two actions
#             merged_action = (u_h + u_r) / 2

#             # Convert the action vector to a Twist message
#             merged_action_twist = Twist()
#             merged_action_twist.linear.x = merged_action[0]
#             merged_action_twist.linear.y = merged_action[1]
#             merged_action_twist.linear.z = merged_action[2]


#             # Perform resulting action
#             arm.set_velocity(merged_action_twist)
            
#             # if we're very close to a goal, stop.
#             if math.dist(state, pos1_xyz) < DIST_THRESHOLD: 
#                 has_reached_goal = True
#                 print("The predicted goal is: GOAL1")     
#                 goal = 'goal1'    
#             elif math.dist(state, pos2_xyz) < DIST_THRESHOLD: 
#                 has_reached_goal = True
#                 print("The predicted goal is: GOAL2")
#                 goal = 'goal2'
#             elif math.dist(state, pos3_xyz) < DIST_THRESHOLD:
#                 has_reached_goal = True
#                 print("The predicted goal is: GOAL3")
#                 goal = 'goal3'


#         rospy.sleep(0.1)
#         # once we've predicted the goal, move to the goal position
#         pos = Pose()
#         pos.position.x = output.get(goal).get('position')['x'] 
#         pos.position.y = output.get(goal).get('position')['y']
#         pos.position.z = output.get(goal).get('position')['z']

#         pos.orientation.x = output.get(goal).get('orientation')['x']
#         pos.orientation.y = output.get(goal).get('orientation')['y']
#         pos.orientation.z = output.get(goal).get('orientation')['z']
#         pos.orientation.w = output.get(goal).get('orientation')['w']

#         # auton_arm.move_to_ee_pose(pos)
                
     
#     except:
#         traceback.print_exc()
#         pass


    
