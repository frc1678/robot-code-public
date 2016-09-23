import numpy as np
import controls
from state_space_gains import *

"""
A state-space observer class
"""
__author__ = 'Kyle Stachowicz (kylestach99@gmail.com)'


class StateSpaceObserver(object):
    def __init__(self, gains, xhat0 = None):
        if xhat0 is None:
            xhat0 = np.zeros(sys.A_d.shape[0])
        self.x_hat = xhat0

        assert (isinstance(gains, list) and isinstance(gains[0], StateSpaceGains))              \
                or isinstance(gains, StateSpaceGains),                                          \
                "gains must be a StateSpaceGains object or a list of StateSpaceGains objects"

        if not isinstance(gains, list):
            gains = [gains]
        self.gains = gains
        self.current_gains_idx = 0

    def set_gains(self, gains_idx):
        assert gains_idx < len(self.gains), "Gains id must be in range."
        self.current_gains_idx = gains_idx

    def update(self, y, u):
        gains = self.gains[self.current_gains_idx]

        self.x_hat = gains.A * self.x_hat +            \
                     gains.B * u +                     \
                     gains.L * (y - gains.C * self.x_hat)

        return self.x_hat
