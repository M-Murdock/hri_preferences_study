#!/usr/bin/env python3
import numpy as np


class ArmPolicy:
    def __init__(self, goals, actions):
        # goals: a goal is a tuple of goal positions
        self.goals = goals
        self.num_actions = len(actions)
        self.actions = actions

    def get_q_value(self, state, action): 
        # return q value for a given action
        q_values = self.get_q_value_map(state)
        return q_values[action]

    def get_q_value_map(self, state):
        # return list of q values for each action
        curr_state = np.array([state[0], state[1], state[2]]) # (x, y, z)
        q_values = np.zeros(self.num_actions)
        

        for dimension in range(0, len(self.goals)): # go through all the dimensions (3, in this case)

            if curr_state[dimension] < self.goals[dimension]: # if we haven't reached the goal yet
                for i in self.get_action_indices(dimension, 1): # give a positive reward for moving in positive direction
                    q_values[i] += 0.5
                for i in self.get_action_indices(dimension, -1): # give a negative reward for moving in negative direction
                    q_values[i] -= 0.5
                for i in self.get_action_indices(dimension, 0): # give small negative reward for not moving
                    q_values[i] -= 0.2

            elif curr_state[dimension] < self.goals[dimension]: # if we've gone past the goal
                for i in self.get_action_indices(dimension, -1): # give a positive reward for moving in negative direction
                    q_values[i] += 0.5
                for i in self.get_action_indices(dimension, 1): # give a negative reward for moving in positive direction
                    q_values[i] -= 0.5
                for i in self.get_action_indices(dimension, 0): # give small negative reward for not moving
                    q_values[i] -= 0.2

            else: # if we're at the goal
                for i in self.get_action_indices(dimension, 1): # give a negative reward for moving in positive direction
                    q_values[i] -= 0.5
                for i in self.get_action_indices(dimension, -1): # give a negative reward for moving in negative direction
                    q_values[i] -= 0.5
                for i in self.get_action_indices(dimension, 0): # give a large reward for not moving
                    q_values[i] = 1

        return q_values

    def get_action_indices(self, dimension, value):
        indices = []
        for action_index in range(0, len(self.actions)):
            if self.actions[action_index][dimension] == value:
                indices.append(action_index)
        return indices

    def get_action(self, state):
        # return best of 9 actionss, q-val distribution, and probability distribution
        q_values = self.get_q_values(state)
        action = np.argmax(q_values)
        return action, q_values
