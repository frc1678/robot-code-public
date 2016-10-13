import numpy as np
import sys

from muan.control.controls import *

from muan.control.state_space_gains import StateSpaceGains
from muan.control.state_space_plant import StateSpacePlant
from muan.control.state_space_controller import StateSpaceController
from muan.control.state_space_observer import StateSpaceObserver

from muan.control.trapezoidal_profile import TrapezoidalMotionProfile

dt = .005

def make_gains():
    name = 'cpp_test'

    # System characteristics
    A_c = np.asmatrix([
        [0.0, 1.0],
        [-1.0, -10.0],
    ])

    B_c = np.asmatrix([
        [0.0],
        [10.0]
    ])

    C = np.asmatrix([
        [1.0, 0.0]
    ])

    Q_controller = np.asmatrix([
        [1.0, 0.0],
        [0.0, 5.0]
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
    Kff = feedforwards(A_d, B_d)
    L = dkalman(A_d, C, Q_d, R_d)

    gains = StateSpaceGains(name, dt, A_d, B_d, C, None, Q_d, R_noise, K, Kff, L)
    gains.add_writable_constant('A_c', A_c)
    gains.add_writable_constant('B_c', B_c)

    return gains

u_max = np.asmatrix([12., 12.]).T
x0 = np.asmatrix([0., 0., 0., 0., 0., 0., 0.]).T

gains = [make_gains()]

if len(sys.argv) == 3:
    # The output files were specified
    from muan.control.state_space_writer import StateSpaceWriter
    writer = StateSpaceWriter(gains, 'cpp_test')
    writer.write(sys.argv[1], sys.argv[2])
else:
    raise Exception("Invalid arguments: please specify a header and a cpp file to write to!")
