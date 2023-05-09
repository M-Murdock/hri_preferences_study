import numpy as np
import scipy.special

class MaxEntPredictor:
    def __init__(self, policies, clip=True):
        self._policies = policies
        self._log_probs = np.full((len(policies)), np.log(1./len(policies)))
        self._clip = clip

    def get_prob(self):
        return np.exp(self._log_probs)

    def update(self, x, u):
        q_vals = np.array([p.get_q_value(x, u) for p in self._policies])
        self._log_probs += q_vals - scipy.special.logsumexp(q_vals)
        self._log_probs = self._log_probs - scipy.special.logsumexp(self._log_probs)
        if self._clip:
            self._log_probs = clip_probability(self._log_probs)
        return self.get_prob()

    def get_prob_after_obs(self, x, u):
        q_vals = np.array([p.get_q_value(x, u) for p in self._policies])
        print(q_vals)
        log_probs = self._log_probs + q_vals - scipy.special.logsumexp(q_vals)
        log_probs = log_probs - scipy.special.logsumexp(log_probs)
        if self._clip:
            log_probs = clip_probability(log_probs)
        return np.exp(log_probs)



MAX_PROB_ANY_GOAL = 0.95
LOG_MAX_PROB_ANY_GOAL = np.log(MAX_PROB_ANY_GOAL)
def clip_probability(log_goal_distribution):
    if len(log_goal_distribution) <= 1:
        return log_goal_distribution
    #check if any too high
    max_prob_ind = np.argmax(log_goal_distribution)
    if log_goal_distribution[max_prob_ind] > LOG_MAX_PROB_ANY_GOAL:
        #see how much we will remove from probability
        diff = np.exp(
            log_goal_distribution[max_prob_ind]) - MAX_PROB_ANY_GOAL
        #want to distribute this evenly among other goals
        diff_per = diff/(len(log_goal_distribution)-1.)

        #distribute this evenly in the probability space...this corresponds to doing so in log space
        # e^x_new = e^x_old + diff_per, and this is formulate for log addition
        log_goal_distribution += np.log(1. +
                diff_per/np.exp(log_goal_distribution))
        #set old one
        log_goal_distribution[max_prob_ind] = LOG_MAX_PROB_ANY_GOAL
    return log_goal_distribution

