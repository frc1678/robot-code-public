import numpy as np
import sys

from muan.control.controls import *

from muan.control.state_space_gains import StateSpaceGains
from muan.control.state_space_plant import StateSpacePlant
from muan.control.state_space_controller import StateSpaceController
from muan.control.state_space_observer import StateSpaceObserver

from muan.control.trapezoidal_profile import TrapezoidalMotionProfile

dt = .005

def make_gains(high_gear):
    # x = | Distance travelled |
    #     |  Forward velocity  |
    #     |  Angular velocity  |
    #     |   Heading angle    |
    #
    # u = | Right voltage |
    #     | Left voltage  |
    #
    # y = | Right encoder |
    #     | Left encoder  |
    #     |  Gyro angle   |
    name = 'high_gear' if high_gear else 'low_gear'

    # System parameters (in SI units)
    mass = 40.8
    wheelbase_radius = 0.24
    wheel_radius = 0.0762
    inertia_moment = 10.
    gear_ratio = 1. / (9.17 if high_gear else 20.9)
    efficiency = .81

    # Motor characteristics (MiniCIM)
    free_speed = 611.6
    free_current = 3.
    stall_torque = 1.41
    stall_current = 89.
    resistance = 12. / stall_current
    torque_constant = stall_torque / stall_current
    velocity_constant = (12. - free_current * resistance) / free_speed

    num_motors = 2 # Per side

    # Wheel velocity = forward velocity + wheelbase_radius * angular velocity

    # Back-EMF torque = emf * wheel velocity
    emf = -(torque_constant * velocity_constant) / (num_motors * resistance * gear_ratio**2. * wheel_radius)

    # Empirical "turning skid", because turning isn't as fast as going straight
    skid = 0.

    # Motor torque = mtq * voltage
    mtq = efficiency * torque_constant / (gear_ratio * resistance * num_motors)

    # Wheel torque * t2v = forward robot acceleration
    t2v = 1. / (wheel_radius * mass)

    # Wheel torque * t2w = robot rotational acceleration
    t2w = wheelbase_radius / (wheel_radius * inertia_moment)

    # System characteristics
    A_c = np.asmatrix([
        [0., 1., 0., 0.],
        [0., t2v * emf * 2.0, 0., 0.],
        [0., 0., 0., 1.],
        [0., 0., 0., t2w * emf * wheelbase_radius * 2.0 + skid]
    ])

    B_c = np.asmatrix([
        [0., 0.],
        [t2v * mtq, t2v * mtq],
        [0., 0.],
        [t2w * mtq, -t2w * mtq]
    ])

    C = np.asmatrix([
        [1. / wheel_radius, 0., wheelbase_radius / wheel_radius, 0.],
        [1. / wheel_radius, 0., -wheelbase_radius / wheel_radius, 0.],
        [0., 0., 1., 0.]
    ])

    # Controller weighting factors
    Q_controller = np.asmatrix([
        [370., 0., 0., 0.],
        [0., 200., 0., 0.],
        [0., 0., 800., 0.],
        [0., 0., 0., 200.],
    ])

    R_controller = np.asmatrix([
        [1., 0.],
        [0., 1.]
    ])

    # Noise characteristics
    Q_noise = np.asmatrix([
        [1e-2, 0., 0., 0.],
        [0., 1e-1, 0., 0.],
        [0., 0., 1e-2, 0.],
        [0., 0., 0., 1e-1]
    ])

    R_noise = np.asmatrix([
        [0.1, 0., 0.],
        [0., 0.1, 0.],
        [0., 0., .005]
    ])

    # Ignore everything except for velocities in feedforward
    Q_ff = np.asmatrix([
        [0., 0., 0., 0.],
        [0., 1., 0., 0.],
        [0., 0., 0., 0.],
        [0., 0., 0., 1.]
    ])

    A_d, B_d, Q_d, R_d = c2d(A_c, B_c, dt, Q_noise, R_noise)
    K = clqr(A_c, B_c, Q_controller, R_controller)
    Kff = feedforwards(A_d, B_d)
    L = dkalman(A_d, C, Q_d, R_d)
    L[0:2, 2:3] = np.asmatrix(np.zeros((2, 1)))
    L[2:4, 0:2] = np.asmatrix(np.zeros((2, 2)))

    gains = StateSpaceGains(name, dt, A_d, B_d, C, None, Q_d, R_noise, K, Kff, L)
    gains.A_c = A_c
    gains.B_c = B_c
    gains.Q_c = Q_noise

    return gains

def make_augmented_gains(high_gear):
    unaugmented_gains = make_gains(high_gear)

    # The augmented controller adds three new "exogenous" states to x:
    #
    # x = | Distance travelled |
    #     |  Forward velocity  |
    #     |  Angular velocity  |
    #     |   Heading angle    |
    #     |Right voltage error |
    #     | Left voltage error |
    #     |Angular encoder slip|
    #
    # u = | Right voltage |
    #     | Left voltage  |
    #
    # y = | Right encoder |
    #     | Left encoder  |
    #     |  Gyro angle   |
    #
    # These exogenous states are computed by the observer and measure
    # quantites that, while not a part of the system's dynamics, correspond to
    # the unpredictable outside disturbances acting upon the system.
    #
    # The right and left voltage errors are estimates of how much voltage must
    # be applied to each side of the drivetrain to compensate for external
    # disturbances. The angular encoder slip is an estimate of how the
    # encoders' measurement of the robot's angle is incorrect - in other words,
    # by how much the encoders have slipped. This allows the encoders to be
    # taken into account in the short-term while making sure that their slip
    # doesn't affect the absolute gyro angle. The defining equation is as
    # follows:
    # (encoder_right - encoder_left)/(2*wheelbase_radius) = angle + angle_slip
    #
    # Credit goes to 971 Spartan Robotics for the concept behind this augmented
    # controller

    dt = unaugmented_gains.dt

    A_c = np.asmatrix(np.zeros((7, 7)))
    A_c[:4, :4] = unaugmented_gains.A_c
    A_c[:4, 4:6] = unaugmented_gains.B_c

    B_c = np.asmatrix(np.zeros((7, 2)))
    B_c[:4, :] = unaugmented_gains.B_c

    C = np.asmatrix(np.zeros((3, 7)))
    C[:, :4] = unaugmented_gains.C
    C[:, 4:7] = np.asmatrix([[0., 0., C[0, 2]],
                             [0., 0., C[1, 2]],
                             [0., 0., 0.]])
    D = np.asmatrix(np.zeros((3, 2)))

    K = np.zeros((2, 7))
    K[:, :4] = unaugmented_gains.K
    K[0, 4] = 1.
    K[1, 5] = 1.

    Q_noise = np.zeros((7, 7))
    Q_noise[:4, :4] = unaugmented_gains.Q_c
    # Make the exogenous states noisy to simulate a rough environment
    Q_noise[4:7, 4:7] = np.asmatrix([
        [1., -1.5, 0.],
        [1., 1.5, 0.],
        [0., 0., 1.]
    ])

    R_noise = np.asmatrix([
        [1e-2, 0., 0.],
        [0., 1e-2, 0.],
        [0., 0., 1e-5]
    ])

    # Kalman noise matrix - have the estimator use the sensors heavily when
    # calculating all of the exogenous states.
    Q_kalman = np.zeros((7, 7))
    Q_kalman[:4, :4] = unaugmented_gains.Q_c
    Q_kalman[4:7, 4:7] = np.asmatrix([
        [10., -5., 0.],
        [10., 5., 0.],
        [0., 0., 2.]
    ])

    # Ignore everything except for velocities in feedforward
    Q_ff = np.asmatrix([
        [0., 0., 0., 0., 0., 0., 0.],
        [0., 1., 0., 0., 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., 0.],
        [0., 0., 0., 1., 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., 0.],
        [0., 0., 0., 0., 0., 0., 0.]
    ])

    A_d, B_d, Q_d, R_d = c2d(A_c, B_c, dt, Q_noise, R_noise)
    _, _, Q_dkalman, R_dkalman = c2d(A_c, B_c, dt, Q_kalman, R_noise)
    L = dkalman(A_d, C, Q_dkalman, R_dkalman)
    Kff = feedforwards(A_d, B_d, Q_ff)

    name = unaugmented_gains.name + '_integral'

    gains = StateSpaceGains(name, dt, A_d, B_d, C, None, Q_d, R_noise, K, Kff, L)
    gains.A_c = A_c
    gains.B_c = B_c

    return gains

u_max = np.asmatrix([12., 12.]).T
x0 = np.asmatrix([0., 0., 0., 0., 0., 0., 0.]).T

gains = [make_augmented_gains(True), make_augmented_gains(False)]

plant = StateSpacePlant(gains, x0)
controller = StateSpaceController(gains, -u_max, u_max)
observer = StateSpaceObserver(gains, x0)

dprofile = TrapezoidalMotionProfile(10., 2.7, 1.2)
aprofile = TrapezoidalMotionProfile(0.1, 1, .7)

def goal(t):
    return np.asmatrix([
        [dprofile.distance(t)],
        [dprofile.velocity(t)],
        [aprofile.distance(t)],
        [aprofile.velocity(t)],
        [0.],
        [0.],
        [0.]
    ])

if len(sys.argv) == 3:
    # The output files were specified
    from muan.control.state_space_writer import StateSpaceWriter
    writer = StateSpaceWriter(gains, 'drivetrain')
    writer.write(sys.argv[1], sys.argv[2])
else:
    # No outputs were specified, so we can assume we just want to graph things
    from muan.control.state_space_scenario import StateSpaceScenario

    scenario = StateSpaceScenario(plant, x0, controller, observer, x0, 'drivetrain')
    scenario.run(goal, dprofile.total_time)
