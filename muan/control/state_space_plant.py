#!/usr/bin/python3

import numpy as np
import numpy.matlib as mat
import scipy as sci
import controls

"""
A state-space plant class with support for process and measurement noise
"""
__author__ = 'Kyle Stachowicz (kylestach99@gmail.com)'

class StateSpacePlant:
    def __init__(self, dt, x_initial, A, B, C, D = np.asmatrix([[0]]), Q = None, R = None):
        self.num_states = A.shape[0]
        self.num_inputs = B.shape[1]
        self.num_outputs = C.shape[0]

        self.set_system(dt, A, B, C, D, Q, R)
        self.x = x_initial
        self.u = mat.zeros((B.shape[1], 1))
        self.y = self.C * self.x + self.D * self.u

    def update(self, u):
        # Calculate noise
        process_noise = self.Q_d * np.random.randn(self.num_states, 1)
        measurement_noise = self.R_c * np.random.randn(self.num_outputs, 1)

        self.u = u
        self.x = self.A_d * self.x + self.B_d * self.u + process_noise
        self.y = self.C * self.x + self.D * self.u + measurement_noise
        return self.y

    def set_system(self, dt, A, B, C, D = 0, Q = None, R = None):
        self.A_c = np.asmatrix(A)
        self.B_c = np.asmatrix(B)
        self.C = np.asmatrix(C)
        self.D = np.asmatrix(D)

        if Q is None:
            Q = np.zeros(self.A_c.shape)
        self.Q_c = np.asmatrix(Q)

        if R is None:
            R = mat.zeros((self.num_outputs, self.num_outputs))
        self.R_c = np.asmatrix(R)

        self.dt = dt

        self.A_d, self.B_d, self.Q_d, self.R_d = controls.c2d(self.A_c, self.B_c, dt, self.Q_c, self.R_c)
