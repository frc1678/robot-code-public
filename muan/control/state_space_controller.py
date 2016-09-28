#!/usr/bin/python3

import numpy as np
import controls
from state_space_gains import *

"""
A state-space controller class with support for feedforward control.
"""
__author__ = 'Kyle Stachowicz (kylestach99@gmail.com)'

class StateSpaceController(object):
    def __init__(self, gains, u_min, u_max):
        if u_min is None:
            u_min = np.asmatrix(np.full(plant.x.shape, -np.inf))
        if u_max is None:
            u_max = np.asmatrix(-u_min)
        self.u_min = u_min
        self.u_max = u_max

        assert (isinstance(gains, list) and isinstance(gains[0], StateSpaceGains))              \
                or isinstance(gains, StateSpaceGains),                                          \
                "gains must be a StateSpaceGains object or a list of StateSpaceGains objects"

        if not isinstance(gains, list):
            gains = [gains]
        self.gains = gains
        self.current_gains_idx = 0

        B = gains[self.current_gains_idx].B
        assert u_min.shape[0] == B.shape[1] and u_min.shape[1] == 1, "u_min must be a vector compatible with B"
        assert u_max.shape[0] == B.shape[1] and u_max.shape[1] == 1, "u_max must be a vector compatible with B"

        self.r = np.asmatrix(np.zeros(self.gains[0].B.shape[1]))

    def set_gains(self, gains_idx):
        assert gains_idx < len(self.gains), "Gains id must be in range."
        self.current_gains_idx = gains_idx

    def update(self, x, goal_next = None):
        gains = self.gains[self.current_gains_idx]

        # To use a moving goal, pass in a value for goal_next
        if goal_next is None:
            goal_next = self.r

        assert goal_next.shape[0] == gains.A.shape[0] and goal_next.shape[1] == 1, "r must be a vector compatible with A"

        # u from the feed-forwards term
        uff = gains.Kff * (goal_next - gains.A * self.r)

        # u from the closed-loop controller
        uc = gains.K * (self.r - x)

        self.r = goal_next

        u = uff + uc
        u = np.clip(u, self.u_min, self.u_max)

        return u
