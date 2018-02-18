import numpy as np
import sys

from muan.control.state_space_gains import StateSpaceGains
from muan.control.state_space_plant import StateSpacePlant
from muan.control.state_space_controller import StateSpaceController
from muan.control.state_space_observer import StateSpaceObserver
from muan.control.trapezoidal_profile import TrapezoidalMotionProfile
from muan.control.controls import *

dt = 0.005

def make_gains(second_stage, has_cube, subname='gains'):
    # x = |   Linear Height   |
    #     |  Linear Velocity  |
    # u = voltage
    # y = encoder

    name = subname

    mass_carriage = 6.0

    if second_stage:
        mass_carriage += 2.5

    if has_cube:
        mass_carriage += 1.59

    # Parameters
    r = (1.0 + 1.0 / 16.0) * 0.0254
    J = 0.68 * r ** 2 + mass_carriage * r ** 2
    G = (12.0 / 100.0) * (14.0 / 30.0)
    eff = .8

    w_free = 18730. / 60. * 6.28
    I_free = .7
    T_stall = 0.71
    I_stall = 134.
    R = 12. / I_stall
    Kt = T_stall / I_stall
    Kv = (12. - I_free * R) / w_free

    num_motors = 2.
    sensor_ratio = 1.

    A_c = np.asmatrix([
        [0., 1.0],
        [0., -Kt * Kv / (J * G * G * R)],
    ])

    B_c = np.asmatrix([
        [0.],
        [Kt * r / (J * G * R)]
    ])

    C = np.asmatrix([
        [sensor_ratio, 0.]
    ])

    # Noise
    Q_noise = np.asmatrix([
        [0., 0.],
        [0., 0.]
    ])

    R_noise = np.asmatrix([
        [1e-5]
    ])

    Q_ff = np.asmatrix([
        [0., 0.],
        [0., 1.]
    ])

    A_d, B_d, Q_d, R_d = c2d(A_c, B_c, dt, Q_noise, R_noise)
    K = place(A_c, B_c, [-10.0, -7.0])
    Kff = feedforwards(A_d, B_d, Q_ff)
    L = dkalman(A_d, C, Q_d, R_d)

    gains = StateSpaceGains(name, dt, A_d, B_d, C, None, Q_d, R_noise, K, Kff, L)
    gains.A_c = A_c
    gains.B_c = B_c
    gains.Q_c = Q_noise

    return gains

def make_augmented_gains(second_stage, has_cube, subname):
    unaugmented_gains = make_gains(second_stage, has_cube, subname)

    dt = unaugmented_gains.dt

    A_c = np.asmatrix(np.zeros((3, 3)))
    A_c[:2, :2] = unaugmented_gains.A_c
    A_c[:2, 2:3] = unaugmented_gains.B_c

    B_c = np.asmatrix(np.zeros((3, 1)))
    B_c[:2, :] = unaugmented_gains.B_c

    C = np.asmatrix(np.zeros((1, 3)))
    C[:, :2] = unaugmented_gains.C

    D = np.asmatrix(np.zeros((1, 1)))

    K = np.zeros((1, 3))
    K[:, :2] = unaugmented_gains.K
    K[0, 2] = 1.

    Q_noise = np.zeros((3, 3))
    Q_noise[:2, :2] = unaugmented_gains.Q_c
    Q_noise[2, 2] = 10.

    R_noise = np.asmatrix([
        [1e-5]
    ])

    # Kalman noise matrix
    Q_kalman = np.asmatrix([
        [1e-2, 0.0, 0.0],
        [0.0, 2e-1, 0.0],
        [0.0, 0.0, 3e3]
    ])

    Q_ff = np.asmatrix([
        [0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0]
    ])

    A_d, B_d, Q_d, R_d = c2d(A_c, B_c, dt, Q_noise, R_noise)
    _, _, Q_dkalman, R_dkalman = c2d(A_c, B_c, dt, Q_kalman, R_noise)
    L = dkalman(A_d, C, Q_dkalman, R_dkalman)
    Kff = feedforwards(A_d, B_d, Q_ff)

    name = unaugmented_gains.name + '_integral'

    gains = StateSpaceGains(name, dt, A_d, B_d, C, None, Q_d, R_noise, K, Kff, L)

    return gains


u_max = np.asmatrix([12.]).T
x0 = np.asmatrix([0., 0., 0.]).T

gains = [make_augmented_gains(True, True, 'second_stage_cube'), make_augmented_gains(True, False, 'second_stage'), make_augmented_gains(False, True, 'first_stage_cube'), make_augmented_gains(False, False, 'first_stage')]

plant = StateSpacePlant(gains, x0)
controller = StateSpaceController(gains, -u_max, u_max)
observer = StateSpaceObserver(gains, x0)

profile = TrapezoidalMotionProfile(1.0, 3.0, 3.0)

def goal(t):
    return np.asmatrix([profile.distance(t), profile.velocity(t), 0.]).T

if __name__ == '__main__':
    if len(sys.argv) == 3:
        from muan.control.state_space_writer import StateSpaceWriter
        writer = StateSpaceWriter(gains, 'elevator_controller')
        writer.write(sys.argv[1], sys.argv[2])
    else:
        from muan.control.state_space_scenario import StateSpaceScenario

        scenario = StateSpaceScenario(plant, x0, controller, observer, x0, 'elevator_controller')
        scenario.run(goal, 4)
