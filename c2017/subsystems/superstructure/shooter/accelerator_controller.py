import numpy as np
import sys

from muan.control.state_space_gains import StateSpaceGains
from muan.control.state_space_plant import StateSpacePlant
from muan.control.state_space_controller import StateSpaceController
from muan.control.state_space_observer import StateSpaceObserver
from muan.control.controls import *

dt = 0.005

def make_gains():
    # x = |       Angle      |
    #     | Angular velocity |
    # u = voltage
    # y = encoder

    name = 'gains'

    # Parameters
    moment_inertia = 0.226796 * (1 * .0256)**2.0 / 2.0 + 0.226796 * (0.5 * 0.0256)**2.0
    gear_ratio = 1.0 / 4.0
    efficiency = .91

    # motor characteristics
    free_speed = 18700. / 60.0 * 6.28
    free_current = .67
    stall_torque = .71
    stall_current = 134.
    resistance = 12. / stall_current
    torque_constant = stall_torque / stall_current
    velocity_constant = (12. -free_current * resistance) / free_speed

    num_motors = 2.0
    sensor_ratio = 1.0

    # back emf torque
    emf = -(torque_constant * velocity_constant) / (resistance * gear_ratio**2. / num_motors)

    # motor torque
    mtq = efficiency * torque_constant / (gear_ratio * resistance / num_motors)

    # rotational acceleration
    t2a = 1. / moment_inertia


    A_c = np.asmatrix([
        [0., 1.],
        [0., t2a * emf]
    ])

    B_c = np.asmatrix([
        [0.],
        [t2a * mtq]
    ])

    C = np.asmatrix([
        [sensor_ratio, 0.]
    ])

    # Controller weighting
    Q_controller = np.asmatrix([
        [0., 0.],
        [0., 5e-3]
    ])

    R_controller = np.asmatrix([
        [1.]
    ])

    # Noise
    Q_noise = np.asmatrix([
        [1e-2, 0.],
        [0., 1e3]
    ])

    R_noise = np.asmatrix([
        [0.1]
    ])

    Q_ff = np.asmatrix([
        [0., 0.],
        [0., 1.]
    ])

    A_d, B_d, Q_d, R_d = c2d(A_c, B_c, dt, Q_noise, R_noise)
    K = clqr(A_c, B_c, Q_controller, R_controller)
    Kff = feedforwards(A_d, B_d, Q_ff)
    L = place(A_d.T, C.T, [0.05, 0.12]).T

    gains = StateSpaceGains(name, dt, A_d, B_d, C, None, Q_d, R_noise, K, Kff, L)
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
    return np.asmatrix([0., 300.]).T

if __name__ == '__main__':
    if len(sys.argv) == 3:
        from muan.control.state_space_writer import StateSpaceWriter
        writer = StateSpaceWriter(gains, 'accelerator_controller')
        writer.write(sys.argv[1], sys.argv[2])
    else:
        from muan.control.state_space_scenario import StateSpaceScenario

        scenario = StateSpaceScenario(plant, x0, controller, observer, x0, 'accelerator_controller')
        scenario.run(goal, 4)
