import numpy as np
import controls

class ss_observer:
    def __init__(self, sys, x_initial = None, *, Q = None, R = None, poles = None, L = None):
        if x_initial is None:
            x_initial = np.zeros(sys.A_d.shape[0])

        self.sys = sys

        self._generate_gains(Q, R, poles, L)

        self.x_hat = x_initial

    def _generate_gains(self, Q, R, poles, L):
        self.gains_method = None

        if Q is not None and R is not None:
            self.Q = np.asmatrix(Q)
            self.R = np.asmatrix(R)
            _, _, self.Q_d, self.R_d = controls.c2d(self.sys.A_c, self.sys.B_c, self.sys.dt, self.Q, self.R)
            self.L = controls.dkalman(self.sys.A_d, self.sys.C, self.Q_d, self.R_d)

            if self.gains_method is not None:
                raise Exception('Passing in conflicting controller gains methods')
            self.gains_method = 'lqr'

        if poles is not None:
            self.poles = poles
            self.L = np.asmatrix(controls.place(self.sys.A_d.T, self.sys.C.T, self.poles).T)

            if self.gains_method is not None:
                raise Exception('Passing in conflicting controller gains methods')
            self.gains_method = 'place'

        if L is not None:
            self.L = np.asmatrix(L)
            if self.gains_method is not None:
                raise Exception('Passing in conflicting controller gains methods')
            self.gains_method = 'manual'

    def update(self, y, u):
        self.x_hat = self.sys.A_d * self.x_hat +            \
                     self.sys.B_d * u +                     \
                     self.L * (y - self.sys.C * self.x_hat)
        return self.x_hat
