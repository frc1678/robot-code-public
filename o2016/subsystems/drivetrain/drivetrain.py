import numpy as np
import sys
from muan.control.state_space_controller import *
from muan.control.state_space_plant import *
from muan.control.state_space_observer import *
from muan.control.state_space_scenario import state_space_scenario
from muan.control.trapezoidal_profile import trapezoidal_profile

dt = .005

# x = | Distance travelled |
#     |  Forward velocity  | |   Heading angle    |
#     |  Angular velocity  |
#
# u = | Right voltage |
#     | Left voltage  |
#
# y = | Right encoder |
#     | Left encoder  |
#     |  Gyro angle   |

# System parameters (in SI units)
mass = 40.8
wheelbase_radius = 0.24
wheel_radius = 0.0762
inertia_moment = 10.
gear_ratio = 1. / 9.17
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
    [17., 0., 0., 0.],
    [0., 10., 0., 0.],
    [0., 0., 30., 0.],
    [0., 0., 0., 20.],
])

R_controller = np.asmatrix([
    [1., 0.],
    [0., 1.]
])

# Noise characteristics
Q_noise = np.asmatrix([
    [0., 0., 0., 0.],
    [0., 0.05, 0., 0.],
    [0., 0., 0., 0.],
    [0., 0., 0., 0.05]
])

R_noise = np.asmatrix([
    [0.1, 0., 0.],
    [0., 0.1, 0.],
    [0., 0., .005]
])

u_max = np.asmatrix([12., 12.]).T
x0 = np.asmatrix([0., 0., 0., 0.]).T

plant = state_space_plant(dt, x0, A_c, B_c, C, np.asmatrix(np.zeros((3, 2))), Q_noise, R_noise)

controller = lqr(plant, Q_controller, R_controller, -u_max, u_max)

observer = kalman(plant)
observer.L[0:2, 2:3] = np.asmatrix(np.zeros((2, 1)))
observer.L[2:4, 0:2] = np.asmatrix(np.zeros((2, 2)))

dprofile = trapezoidal_profile(10., 3.0, 1.2)
aprofile = trapezoidal_profile(0.1, 1, .7)

def goal(t):
    return np.asmatrix([
        [dprofile.distance(t)],
        [dprofile.velocity(t)],
        [aprofile.distance(t)],
        [aprofile.velocity(t)]
    ])

scenario = state_space_scenario(plant, x0, controller, observer, x0, 'drivetrain')

if len(sys.argv) == 3:
  # The output files were specified
  scenario.write(sys.argv[1], sys.argv[2])
else:
  # No outputs were specified, so we can assume we just want to graph things
  scenario.run(goal, dprofile.total_time)
