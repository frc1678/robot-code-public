import numpy as np
from ss_controller import ss_controller
from ss_plant import ss_plant
from ss_observer import ss_observer

class error_u_controller(ss_controller):
    def __init__(self, sys, *, Q = None, R = None, poles = None, K = None, Kff = None, u_min = None, u_max = None):
        unaugmented_controller = ss_controller(sys, Q = Q, R = R, poles = poles, K = K, Kff = Kff, u_min = u_min, u_max = u_max)

        n_ua = sys.A_c.shape[0]

        K = np.zeros((1, n_ua + 1))
        print(unaugmented_controller.K)
        K[0:1, 0:n_ua] = unaugmented_controller.K
        K[0, n_ua] = 1

        A_c = np.zeros((n_ua + 1, n_ua + 1))
        A_c[0:n_ua, 0:n_ua] = sys.A_c
        A_c[0:n_ua, n_ua:n_ua + 1] = sys.B_c
        B_c = np.zeros((sys.B_c.shape[0] + 1, sys.B_c.shape[1]))
        B_c[0:n_ua, 0:1] = sys.B_c[0:n_ua]
        C = np.zeros((sys.C.shape[0], sys.C.shape[1] + 1))
        C[0:1, 0:-1] = sys.C[0]

        x = np.zeros((sys.x.shape[0] + 1, 1))
        x[0:-2, 0] = sys.x[0:-1]

        sys_aug = ss_plant(sys.dt, x, A_c, B_c, C, sys.D)

        gains_method = unaugmented_controller.gains_method

        super(error_u_controller, self).__init__(sys_aug, K = K, u_min = u_min, u_max = u_max)

class error_u_observer(ss_observer):
    def __init__(self, sys, x_initial = None, *, Q = None, R = None, poles = None, L = None):
        n_ua = sys.A_c.shape[0]

        A_c = np.zeros((n_ua + 1, n_ua + 1))
        A_c[0:n_ua, 0:n_ua] = sys.A_c
        A_c[0:n_ua, n_ua:n_ua + 1] = sys.B_c

        B_c = np.zeros((sys.B_c.shape[0] + 1, sys.B_c.shape[1]))
        B_c[0:n_ua, 0:1] = sys.B_c[0:n_ua]

        C = np.zeros((sys.C.shape[0], sys.C.shape[1] + 1))
        C[0:1, 0:-1] = sys.C[0]

        x = np.zeros((n_ua + 1, 1))
        if x_initial is not None:
            x[0:-2, 0] = x_initial[0:-1]

        sys_aug = ss_plant(sys.dt, x, A_c, B_c, C, sys.D, Q = Q, R = R)

        super(error_u_observer, self).__init__(sys_aug, x_initial, Q = Q, R = R, poles = poles, L = L)
