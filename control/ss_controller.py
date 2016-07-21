#!/usr/bin/python3

import numpy as np
import ss_plant
import controls
import matplotlib.pyplot as plt

class ss_controller:
    def __init__(self, sys, *, Q = None, R = None, poles = None, K = None, Kff = None, u_min = None, u_max = None):
        self.sys = sys

        self._generate_gains(Q, R, poles, K, Kff)

        self.u_min = np.asmatrix(u_min)
        self.u_max = np.asmatrix(u_max)

        self.r = np.zeros(sys.A_d.shape[0])

    def _generate_gains(self, Q, R, poles, K, Kff):
        self.gains_method = None

        if K is not None:
            self.K = np.asmatrix(K)
            if self.gains_method is not None:
                raise Exception('Passing in conflicting controller gains methods')
            self.gains_method = 'manual'

        if Q is not None and R is not None:
            self.Q = np.asmatrix(Q)
            self.R = np.asmatrix(R)
            self.lqr(self.Q, self.R)
            if self.gains_method is not None:
                raise Exception('Passing in conflicting controller gains methods')
            self.gains_method = 'lqr'

        if poles is not None:
            self.poles = poles
            self.place(self.poles)
            if self.gains_method is not None:
                raise Exception('Passing in conflicting controller gains methods')
            self.gains_method = 'place'

        # If a Kff matrix is not supplied, generate one
        if Kff is None:
            if Q is None:
                Q = np.asmatrix(np.identity(self.sys.A_d.shape[0]))

            # In the case of Q=aI, Kff = pinv(B)
            self.Kff = np.linalg.inv(self.sys.B_d.T * Q * self.sys.B_d) * self.sys.B_d.T * Q.T

    def update(self, x, goal_next):
        if goal_next is None:
            goal_next = self.r

        # u from the feed-forwards term
        uff = self.Kff * (goal_next - self.sys.A_d * self.r)

        # u from the closed-loop controller
        uc = self.K * (self.r - x)

        self.r = goal_next

        u = uff + uc
        if self.u_min is not None and self.u_max is not None:
            u = np.clip(u, self.u_min, self.u_max)

        return u

    def place(self, poles):
        self.K = controls.place(self.sys.A_d, self.sys.B_d, poles)

    def lqr(self, Q, R):
        self.K = controls.dlqr(self.sys.A_d, self.sys.B_d, Q, R)

    def set_goal_state(self, goal, nbar):
        self.r = goal
