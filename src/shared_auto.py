#!/usr/bin/env python3
import numpy as np
import scipy

from util import iter_space, linprog


class SharedAutoPolicy:
    def __init__(self, policies, action_space):
        self._action_space = action_space
        self._policies = policies
        # TODO (somehow): check that action spaces are compatible and discrete!
    
    def get_action(self, x, prob_policy, return_dist=False, sample=False):
        actions = tuple(iter_space(self._action_space))
        qs = np.zeros((len(self._policies), len(actions)))
        for i, policy in enumerate(self._policies):
            if hasattr(policy, "get_q_values"):
                qs[i] = policy.get_q_values(x, actions)
            else:
                qs[i] = [ policy.get_q_value(x, a) for a in actions]
        expected_q = np.dot(prob_policy, qs)
        expected_q -= scipy.special.logsumexp(expected_q)

        if return_dist:
            return actions[np.argmax(expected_q)], np.exp(expected_q)
        elif sample:
            return np.random.choice(actions, p=np.exp(expected_q))
        else:
            return actions[np.argmax(expected_q)]


if __name__ == "__main__":
    SharedAutoPolicy(None, None)