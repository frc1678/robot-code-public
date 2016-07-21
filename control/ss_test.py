#!/usr/bin/python3

import numpy as np
import math
from ss_controller import ss_controller
from ss_plant import ss_plant
from ss_observer import ss_observer
from ss_scenario import ss_scenario
from error_u import error_u_observer, error_u_controller
from trapezoidal_profile import trapezoidal_profile

# Simple linear second-order system

A = np.asmatrix([[0, 1],
                 [0, -1]])
B = np.asmatrix([0, 1]).T
C = np.asmatrix([1, 0])
dt = .01

x_initial = np.asmatrix([0, 0]).T

Q_c = np.asmatrix([[10, 0],
                   [0, 100]])
R_c = np.asmatrix([[1]])

Q_o = np.asmatrix([
    [1, 0],
    [0, 1]
])

R_o = np.asmatrix([
    [0.3]
])

u_max = np.asmatrix([[12]])

plant = ss_plant(dt, x_initial, A, B, C, Q = Q_o, R = R_o)
controller = ss_controller(plant, Q = Q_c, R = R_c, u_min = -u_max, u_max = u_max)
observer = ss_observer(plant, Q = Q_o, R = R_o)

scenario = ss_scenario(plant, x_initial, controller, observer, x_initial, 'test')

profile = trapezoidal_profile(10, 5, 5)

def goal(t):
    return np.asmatrix([profile.distance(t), profile.velocity(t)]).T

scenario.run(goal, profile.total_time)
scenario.write('test.h', 'test.cpp')

# Nonlinear system with error-u controller

class nonlinear_plant(ss_plant):
    def __init__(self, x_initial):
        A = np.asmatrix([[0, 1, 0],
                         [0, -1, 1],
                         [0, 0, 0]])
        B = np.asmatrix([0, 1, 0]).T
        C = np.asmatrix([1, 0, 0])

        super(nonlinear_plant, self).__init__(.01, x_initial, A, B, C);

    def update(self, u):
        self.x[2] = 3 * math.cos(self.x[0])
        super(nonlinear_plant, self).update(u)

Q_nl = np.asmatrix([[1e-4, 0, 0],
                    [0, 1e-5, 0],
                    [0, 0, 4e7]])
R_nl = np.asmatrix([[1e-2]])

x_initial_nl = np.asmatrix([0, 0, 0]).T
plant_nl = nonlinear_plant(x_initial_nl)

Au = np.asmatrix([[0, 1],
                 [0, -1]])
Bu = np.asmatrix([0, 1]).T
Cu = np.asmatrix([1, 0])
plant_lin = ss_plant(.01, x_initial_nl[0:2, 0], Au, Bu, Cu)
controller_nl = error_u_controller(plant_lin, poles = [.97, .98], u_min = -u_max, u_max = u_max)
observer_nl = error_u_observer(plant_lin, Q = Q_nl, R = R_nl)

scenario = ss_scenario(plant_nl, x_initial_nl, controller_nl, observer_nl, x_initial_nl, 'test')

profile = trapezoidal_profile(10, 5, 5)

def goal_nl(t):
    return np.asmatrix([profile.distance(t), profile.velocity(t), 0]).T

scenario.run(goal_nl, profile.total_time)
