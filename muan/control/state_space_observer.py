import numpy as np
import controls

"""
A state-space observer class
"""
__author__ = 'Kyle Stachowicz (kylestach99@gmail.com)'


class StateSpaceObserver(object):
    def __init__(self, plant, L, x0 = None):
        if x0 is None:
            x0 = np.zeros(sys.A_d.shape[0])

        self.sys = plant
        self.L = np.asmatrix(L)
        self.x_hat = np.asmatrix(x0)

    def update(self, y, u):
        self.x_hat = self.sys.A_d * self.x_hat +            \
                     self.sys.B_d * u +                     \
                     self.L * (y - self.sys.C * self.x_hat)
        return self.x_hat

def kalman(plant):
    L = controls.dkalman(plant.A_d, plant.C, plant.Q_d, plant.R_d)
    return StateSpaceObserver(plant, L, plant.x)

def placement(plant, poles):
    L = controls.place(plant.A_d.T, plant.C.T, poles).T
    return StateSpaceObserver(plant, L, plant.x)
