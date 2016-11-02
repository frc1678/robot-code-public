import numpy as np

def _matrix_or_zeros(A, shape):
    if A is None:
        return np.asmatrix(np.zeros(shape))
    else:
        A = np.asmatrix(A)
        assert A.shape == shape, "Matrix must match specified shape."
        return A

class StateSpaceGains(object):
    def __init__(self, name, dt, A, B, C, D, Q, R, K, Kff, L):
        """
        Note: all matrices refer to the discrete-time variant unless specified otherwise
        Args:
            name: A string name for this set of gains
            dt: The system's timestep
            A: System dynamics matrix
            B: Control signal dynamics matrix
            C: Measurement matrix
            D: Feedthrough matrix
            Q: *discrete-time* process noise matrix
            R: *continuous-time* measurement noise matrix
            K: Feedback gains matrix
            Kff: Feedforwards gains matrix
            L: Observer gains matrix
        """
        self.name = name
        self.dt = dt
        self.A = np.asmatrix(A)
        self.B = np.asmatrix(B)
        self.C = np.asmatrix(C)
        self.D = _matrix_or_zeros(D, (C.shape[0], B.shape[1]))
        self.Q = _matrix_or_zeros(Q, A.shape)
        self.R = _matrix_or_zeros(R, (C.shape[0], C.shape[0]))
        self.K = np.asmatrix(K)
        self.Kff = _matrix_or_zeros(Kff, K.shape)
        self.L = _matrix_or_zeros(L, (A.shape[0], C.shape[0]))

        self.extra_constants = {}

    def add_writable_constant(self, name, value):
        self.extra_constants[name] = value

    def writable_constants(self):
        ret = {
            'A': self.A,
            'B': self.B,
            'C': self.C,
            'D': self.D,
            'Q': self.Q,
            'R': self.R,
            'K': self.K,
            'Kff': self.Kff,
            'L': self.L,
            'dt': self.dt
        }
        ret.update(self.extra_constants)
        return ret
