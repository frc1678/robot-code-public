#!/usr/bin/python3

import numpy as np
import math

import controls
from state_space_gains import *
from state_space_plant import *
from state_space_controller import *
from state_space_observer import *

from state_space_scenario import StateSpaceScenario
from trapezoidal_profile import TrapezoidalMotionProfile
from state_space_writer import StateSpaceWriter

# Simple linear second-order system
A = np.asmatrix([[0, 1],
                 [0, -1]])
B = np.asmatrix([0, 1]).T
C = np.asmatrix([1, 0])
D = np.asmatrix([0])
dt = .01

x_initial = np.asmatrix([0, 0]).T

# Controller weighting factors
Q_c = np.asmatrix([[10, 0],
                   [0, 100]])
R_c = np.asmatrix([[1]])

# Noise properties
Q_o = np.asmatrix([[1, 0],
                   [0, 1]])
R_o = np.asmatrix([[0.3]])

# Calculate gain matrices and discrete-time matrices
A_d, B_d, Q_d, R_d = controls.c2d(A, B, dt, Q_o, R_o)
K = controls.clqr(A, B, Q_c, R_c)
Kff = controls.feedforwards(A_d, B_d)
L = controls.dkalman(A_d, C, Q_d, R_d)

# Create a single set of gains
gains = StateSpaceGains('test_gains', dt, A_d, B_d, C, D, Q_d, R_o, K, Kff, L)

u_max = np.asmatrix([[12]])

plant = StateSpacePlant(gains, x_initial)
controller = StateSpaceController(gains, -u_max, u_max)
observer = StateSpaceObserver(gains, x_initial)

# Create and run the scenario
scenario = StateSpaceScenario(plant, x_initial, controller, observer, x_initial, 'test_controller')
writer = StateSpaceWriter(gains, 'test_controller')

profile = TrapezoidalMotionProfile(10, 5, 5)

def goal(t):
    return np.asmatrix([profile.distance(t), profile.velocity(t)]).T

scenario.run(goal, profile.total_time)
writer.write('/tmp/test.h', '/tmp/test.cpp')
