import numpy as np
import sys

from muan.control.controls import *

from muan.control.state_space_gains import StateSpaceGains
from muan.control.state_space_plant import StateSpacePlant
from muan.control.state_space_controller import StateSpaceController
from muan.control.state_space_observer import StateSpaceObserver

dt = .005

def make_gains():
    # x = |      Angle       |
    #     | Angular velocity |
    #
    # u = | Voltage |
    #
    # y = | Encoder |
    name = 'gains'

    # System parameters (in SI units)
    inertia_moment = 1. # Needs to be measured
    gear_ratio = 1. / 138.89
    efficiency = .8 # Needs to be measured

    # Motor characteristics (775pro)
    free_speed = 18700.
    free_current = .7
    stall_torque = .71
    stall_current = 134.
    resistance = 12. / stall_current
    torque_constant = stall_torque / stall_current
    velocity_constant = (12. - free_current * resistance) / free_speed

    num_motors = 1
    sensor_ratio = 2.78

    # Back-EMF torque = emf * angular velocity
    emf = -(torque_constant * velocity_constant) / (num_motors * resistance * gear_ratio**2.)

    # Motor torque = mtq * voltage
    mtq = efficiency * torque_constant / (gear_ratio * resistance * num_motors)

    # Torque * t2a = rotational acceleration
    t2a = 1. / inertia_moment

    # System characteristics
    A_c = np.asmatrix([
        [0., 1.],
        [0., t2a * emf * 2.0]
    ])

    B_c = np.asmatrix([
        [0.],
        [t2a * mtq]
    ])

    C = np.asmatrix([
        [1, 0.]
    ])

    # Controller weighting factors
    Q_controller = np.asmatrix([
        [1e2, 0.],
        [0., 1.]
    ])

    R_controller = np.asmatrix([
        [1.]
    ])

    # Noise characteristics
    Q_noise = np.asmatrix([
        [1e-2, 0.],
        [0., 1e-1]
    ])

    R_noise = np.asmatrix([
        [0.1]
    ])

    A_d, B_d, Q_d, R_d = c2d(A_c, B_c, dt, Q_noise, R_noise)
    K = clqr(A_c, B_c, Q_controller, R_controller)
    L = dkalman(A_d, C, Q_d, R_d)

    gains = StateSpaceGains(name, dt, A_d, B_d, C, None, Q_d, R_noise, K, None, L)
    gains.A_c = A_c
    gains.B_c = B_c
    gains.Q_c = Q_noise

    return gains

u_max = np.asmatrix([12.]).T
x0 = np.asmatrix([0., 0.]).T

gains = make_gains()

plant = StateSpacePlant(gains, x0)
controller = StateSpaceController(gains, -u_max, u_max)
observer = StateSpaceObserver(gains, x0)

def goal(t):
    return np.asmatrix([1., 0.]).T

if len(sys.argv) == 3:
    # The output files were specified
    from muan.control.state_space_writer import StateSpaceWriter
    writer = StateSpaceWriter(gains, 'stop')
    writer.write(sys.argv[1], sys.argv[2])
else:
    # No outputs were specified, so we can assume we just want to graph things
    from muan.control.state_space_scenario import StateSpaceScenario
    scenario = StateSpaceScenario(plant, x0, controller, observer, x0, 'stop')
    scenario.run(goal, 4)
