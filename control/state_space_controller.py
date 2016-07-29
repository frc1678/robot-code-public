#!/usr/bin/python3

import numpy as np
import state_space_plant
import controls
import math
import matplotlib.pyplot as plt

class state_space_controller:
    def __init__(self, K, Kff, plant, u_min = None, u_max = None):
        if u_min is None:
            u_min = np.full(plant.x.shape, -math.inf)
        if u_max is None:
            u_max = np.full(plant.x.shape, math.inf)

        self.u_min = np.asmatrix(u_min)
        self.u_max = np.asmatrix(u_max)
        self.K = np.asmatrix(K)
        self.Kff = np.asmatrix(Kff)
        self.sys = plant

        self.r = np.asmatrix(np.zeros(K.shape[1]))

    def update(self, x, goal_next = None):
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

def placement(plant, poles, u_min = None, u_max = None):
    K = controls.place(plant.A_d, plant.B_d, poles)
    Kff = np.linalg.pinv(plant.B_d)
    return state_space_controller(K, Kff, plant, u_min, u_max)

def lqr(plant, Q, R, u_min = None, u_max = None):
    K = controls.dlqr(plant.A_d, plant.B_d, Q, R)
    Kff = np.linalg.inv(plant.B_d.T * Q * plant.B_d) * plant.B_d.T * Q.T
    return state_space_controller(K, Kff, plant, u_min, u_max)
