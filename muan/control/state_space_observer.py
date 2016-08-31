import numpy as np
from state_space_plant import *
import controls

"""
A state-space observer class
"""
__author__ = 'Kyle Stachowicz (kylestach99@gmail.com)'


class StateSpaceObserver(object):
    def __init__(self, gains, xhat0 = None):
        if xhat0 is None:
            xhat0 = np.zeros(sys.A_d.shape[0])
        self.x_hat = xhat0

        if not isinstance(gains, list):
            gains = [gains]
        self.gains = gains
        self.current_gains = 0

        assert len(self.gains) > 0, "Must have at least one set of gains."

    def set_gains(self, gains):
        assert gains < len(self.gains), "Gains id must be in range."
        self.current_gains = gains

    def update(self, y, u):
        gains = self.gains[self.current_gains]

        self.x_hat = gains.A * self.x_hat +            \
                     gains.B * u +                     \
                     gains.L * (y - gains.C * self.x_hat)

        return self.x_hat
