#!/usr/bin/python3

import numpy as np
import controls
import matplotlib.pyplot as plt

"""
A state-space controller class with support for feedforward control.
"""
__author__ = 'Kyle Stachowicz (kylestach99@gmail.com)'

class StateSpaceController(object):
    def __init__(self, gains, u_min, u_max):
        self.u_min = np.asmatrix(u_min or np.full(plant.x.shape, -np.inf))
        self.u_max = np.asmatrix(u_max or -u_min)

        if not isinstance(gains, list):
            gains = [gains]
        self.gains = gains
        self.current_gains_idx = 0

        assert len(self.gains) > 0, "Must have at least one set of gains."

        self.r = np.asmatrix(np.zeros(self.gains[0].B.shape[1]))

    def set_gains(self, gains_idx):
        assert gains_idx < len(self.gains), "Gains id must be in range."
        self.current_gains_idx = gains_idx

    def update(self, x, goal_next = None):
        gains = self.gains[self.current_gains_idx]

        # To use a moving goal, pass in a value for goal_next
        if goal_next is None:
            goal_next = self.r

        # u from the feed-forwards term
        uff = gains.Kff * (goal_next - gains.A * self.r)

        # u from the closed-loop controller
        uc = gains.K * (self.r - x)

        self.r = goal_next

        u = uff + uc
        u = np.clip(u, self.u_min, self.u_max)

        return u
