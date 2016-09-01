#!/usr/bin/python3

import numpy as np
import state_space_plant
import controls
import matplotlib.pyplot as plt

"""
A state-space controller class with support for feedforward control.
"""
__author__ = 'Kyle Stachowicz (kylestach99@gmail.com)'

class StateSpaceController(object):
    def __init__(self, K, Kff, plant, u_min = None, u_max = None):
        if u_min is None:
            u_min = np.full(plant.x.shape, -np.inf)
        if u_max is None:
            u_max = np.full(plant.x.shape, np.inf)

        self.u_min = np.asmatrix(u_min)
        self.u_max = np.asmatrix(u_max)
        self.K = np.asmatrix(K)
        self.Kff = np.asmatrix(Kff)
        self.sys = plant

        self.r = np.asmatrix(np.zeros(K.shape[1]))

    def update(self, x, goal_next = None):
        # To use a moving goal, pass in a value for goal_next
        if goal_next is None:
            goal_next = self.r

        # u from the feed-forwards term
        uff = self.Kff * (goal_next - self.sys.A_d * self.r)

        # u from the closed-loop controller
        uc = self.K * (self.r - x)

        self.r = goal_next

        u = uff + uc
        u = np.clip(u, self.u_min, self.u_max)

        return u

"""
The formula for Kff can be derived as follows:
    r(n+1) = A*r(n) + B*u_ff
    B*u_ff = r(n+1) - A*r(n)
    u_ff = pinv(B)*(r(n+1) - A*r(n))
    Kff = B
    u_ff = Kff*(r(n+1) - A*r(n))
There is also an LQR-weighted solution, but it gives the same u_ff assuming
that there is some u_ff that satisfies the equation above.
"""

def placement(plant, poles, u_min = None, u_max = None):
    K = controls.place(plant.A_d, plant.B_d, poles)
    Kff = np.linalg.pinv(plant.B_d)
    return StateSpaceController(K, Kff, plant, u_min, u_max)

def lqr(plant, Q, R, u_min = None, u_max = None):
    K = controls.dlqr(plant.A_d, plant.B_d, Q, R)
    Kff = np.linalg.inv(plant.B_d.T * Q * plant.B_d) * plant.B_d.T * Q.T
    return StateSpaceController(K, Kff, plant, u_min, u_max)
