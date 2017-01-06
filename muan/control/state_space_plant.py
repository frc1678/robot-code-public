#!/usr/bin/python3

import numpy as np
import numpy.matlib as mat
import scipy as sci
import muan.control.controls
from muan.control.state_space_gains import *

"""
A state-space plant class with support for process and measurement noise
"""
__author__ = 'Kyle Stachowicz (kylestach99@gmail.com)'

class StateSpacePlant(object):
    def __init__(self, gains, x_initial = None):
        assert (isinstance(gains, list) and isinstance(gains[0], StateSpaceGains))              \
                or isinstance(gains, StateSpaceGains),                                          \
                "gains must be a StateSpaceGains object or a list of StateSpaceGains objects"

        if not isinstance(gains, list):
            gains = [gains]
        self.gains = gains
        self.current_gains_idx = 0

        if x_initial is None:
            x_initial = mat.zeros((A.shape[0], 1))
        self.x = x_initial

        gains = self.get_current_gains()

        assert self.x.shape[0] == gains.A.shape[0] and self.x.shape[1] == 1, "x0 must be a vector compatible with A"

        self.u = mat.zeros((gains.B.shape[1], 1))
        self.y = gains.C * self.x + gains.D * self.u

    def set_gains(self, gains_idx):
        assert gains_idx < len(self.gains), "Gains id must be in range."
        self.current_gains_idx = gains_idx

    def get_current_gains(self):
        return self.gains[self.current_gains_idx]

    def update(self, u):
        gains = self.get_current_gains()

        # Calculate noise
        process_noise = gains.Q * np.random.randn(gains.A.shape[0], 1)
        measurement_noise = gains.R * np.random.randn(gains.C.shape[0], 1)

        self.u = u
        self.x = gains.A * self.x + gains.B * self.u + process_noise
        self.y = gains.C * self.x + gains.D * self.u + measurement_noise

        return self.y
