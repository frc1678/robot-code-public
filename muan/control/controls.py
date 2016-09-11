#!/usr/bin/python

import scipy
import scipy.signal
import numpy

"""
Control theory helper functions library.
Wraps scipy routines to provide control functions including:
 - Pole placement
 - Steady-state LQR gain
 - Steady-state Kalman gain
 - Controllability and observability matrices
 - Continuous to discrete transformations for the system and noise matrices

Inspired by 971's control helpers library
"""
__author__ = 'Kyle Stachowicz (kylestach99@gmail.com)'

def _validate_system(A, B, C, D):
    if A is not None:
        A = numpy.asmatrix(A)
    if B is not None:
        B = numpy.asmatrix(B)
    if C is not None:
        C = numpy.asmatrix(C)
    if D is not None:
        D = numpy.asmatrix(D)

    assert A is not None and A.shape[0] == A.shape[1], "A must be square"

    if B is not None:
        assert B.shape[0] == A.shape[0], "A.shape %s and B.shape %s must be compatible" % (A.shape, B.shape)

    if C is not None:
        assert C.shape[1] == A.shape[0], "A.shape %s and C.shape %s must be compatible" % (A.shape, C.shape)

    if B is not None and C is not None and D is not None:
        assert D.shape[0] == C.shape[0], "C.shape %s and D.shape %s must be compatible" % (C.shape, D.shape)
        assert D.shape[1] == B.shape[1], "B.shape %s and D.shape %s must be compatible" % (B.shape, D.shape)

def place(A, B, poles):
    """
    Find the m*n matrix K such that the poles (eigenvalues) of A-BK are at the
    desired locations. Works on both discrete-time and continuous-time systems.

    Note: If you are using continuous-time matrices, poles should be negative
    to acheive stability while with discrete-time matrices they should just be
    less than 1

    Args:
        A: n*n system dynamics matrix
        B: n*m control signal matrix
        poles: complex array of desired pole locations
            For every complex pole a+bi, its conjugate a-bi must also be a pole
    Returns:
        K: m*n gains matrix such that u = -Kx
    """
    A = numpy.asmatrix(A)
    B = numpy.asmatrix(B)
    _validate_system(A, B, None, None)

    assert len(poles) == A.shape[0], "The number of poles (%i) must be equal to the rank of the system (%i)." % (len(poles), A.shape[0])
    assert numpy.linalg.matrix_rank(controllability(A, B)) == A.shape[0], "System must be completely controllable to do pole placement."

    result = scipy.signal.place_poles(A, B, poles)

    for req, res in zip(result.requested_poles, result.computed_poles):
        if abs(req - res) > 1e-3:
            print("Warning: Pole %s could not be assigned as given and was instead assigned as %s" % (req, res))

    return result.gain_matrix

def controllability(A, B):
    """
    Calculate the controllability matrix of the system defined by A and B.
    Works on both discrete-time and continuous-time systems.

    In a fully controllable system, rank(controllability(A, B)) == n
    Args:
        A: n*n system dynamics matrix
        B: n*m control signal matrix
    Returns:
        E: n*nm controllability matrix
    """
    A = numpy.asmatrix(A)
    B = numpy.asmatrix(B)
    _validate_system(A, B, None, None)

    n = A.shape[0]
    m = B.shape[1]
    E = numpy.asmatrix(numpy.zeros((n, n*m)))
    x = B
    for i in range(0, n):
        j = i * m
        E[:n, j:j+m] = x
        x = A * x
    return E

def observability(A, C):
    """
    Calculate the observability matrix of the system defined by A and C.
    Works on both discrete-time and continuous-time systems.

    In a fully observable system, rank(controllability(A, C)) == n

    Observability is the dual of controllability, meaning that
    observability(A, C) = controllability(A.T, C.T).T

    Args:
        A: n*n system dynamics matrix
        C: n*q measurement signal matrix
    Returns:
        O: nq*n observability matrix
    """
    A = numpy.asmatrix(A)
    C = numpy.asmatrix(C)
    _validate_system(A, None, C, None)

    n = A.shape[0]
    q = C.shape[0]
    O = numpy.asmatrix(numpy.zeros((n*q, n)))
    y = C
    for i in range(0, n):
        j = i * q
        O[j:j+q, :n] = y
        y = y * A
    return O

def c2d(A, B, dt, Q = None, R = None):
    """
    Transform the continuous-time system dx/dt = Ax + Bu into the discrete-time
    formulation x(n+1) = Ax(n) + Bu(n).
    Args:
        A: n*n continuous-time system dynamics matrix
        B: n*m continuous-time control signal matrix
        dt: time step of the discretized process
        Q (optional): n*n continuous-time process noise covariance matrix
        R (optional): q*q continuous-time measurement noise covariance matrix
    Returns
        Tuple (A_d, B_d, Q_d, R_d)
        A_d: n*n discrete-time system dynamics matrix
        B_d: n*m discrete-time control signal matrix
        Q_d: n*n discrete-time process noise covariance matrix (None if no Q given)
        R_d: q*q discrete-time measurement noise covariance matrix (None if no R given)
    """
    _validate_system(A, B, None, None)

    A = numpy.asmatrix(A)
    B = numpy.asmatrix(B)

    n = A.shape[0]
    m = B.shape[1]
    F = numpy.asmatrix(numpy.zeros((n + m, n + m)))
    F[:n, :n] = A
    F[:n, n:n+m] = B
    G = scipy.linalg.expm(F * dt)

    A_d = G[:n, :n]
    B_d = G[:n, n:n+m]

    Q_d = R_d = None

    if Q is not None and R is not None:
        Q = numpy.asmatrix(Q)
        R = numpy.asmatrix(R)

        assert Q.shape == A.shape, "The dimensions of Q %s must match those of A %s" % (Q.shape, A.shape)
        assert R.shape[0] == R.shape[1], "R must be square but is instead %ix%i" % (R.shape[0], R.shape[1])

        H = numpy.asmatrix(numpy.zeros((n+n, n+n)))
        H[:n, :n] = -A
        H[n:n+n, n:n+n] = A
        H[:n, n:n+n] = Q
        I = numpy.asmatrix(scipy.linalg.expm(H * dt))

        Q_d = numpy.asmatrix(I[n:n+n, n:n+n].T * I[:n, n:n+n])
        R_d = numpy.asmatrix(R / dt)
        return (A_d, B_d, Q_d, R_d)
    else:
        return (A_d, B_d)

def dlqr(A, B, Q, R):
    """
    Calculate the discrete-time steady-state LQR gain matrix.
    Minimize sum{0, inf}(x'Qx + u'Ru) for the system x(n+1) = Ax(n) + Bu(n).
    Args:
        A: n*n discrete-time system dynamics matrix
        B: n*m discrete-time control signal matrix
        Q: n*n quadratic state error weighting factor
        R: n*n quadratic control signal weighting factor
    Returns:
        K: m*n gains matrix such that u = -Kx
    """
    A = numpy.asmatrix(A)
    B = numpy.asmatrix(B)
    Q = numpy.asmatrix(Q)
    R = numpy.asmatrix(R)

    _validate_system(A, B, None, None)
    assert Q.shape[0] == Q.shape[1] and Q.shape[0] == A.shape[0], "The dimensions of Q %s must match those of A %s" % (Q.shape, A.shape)
    assert R.shape[0] == R.shape[1] and R.shape[0] == B.shape[1], "R %i must be square and must be compatible with B %i" % (R.shape, B.shape)

    assert numpy.linalg.matrix_rank(controllability(A, B)) == A.shape[0], "System must be completely controllable to do LQR."

    # TODO(Kyle): Ensure Q is positive-semidefninite and R is positive-definite

    # Solve the ARE for the cost-to-go matrix
    M = numpy.asmatrix(scipy.linalg.solve_discrete_are(A, B, Q, R))

    # Finally, solve for the optimal gain matrix using the cost-to-go matrix
    return numpy.asmatrix(numpy.linalg.inv(R) * B.T * M)

def clqr(A, B, Q, R):
    """
    Calculate the continuous-time steady-state LQR gain matrix.
    Minimize integral{0, inf}(x'Qx + u'Ru) for the system dx/dt = Ax + Bu.
    Args:
        A: n*n continuous-time system dynamics matrix
        B: n*m continuous-time control signal matrix
        Q: n*n quadratic state error weighting factor
        R: n*n quadratic control signal weighting factor
    Returns:
        K: m*n gain matrix such that u = -Kx
    """
    A = numpy.asmatrix(A)
    B = numpy.asmatrix(B)
    Q = numpy.asmatrix(Q)
    R = numpy.asmatrix(R)

    _validate_system(A, B, None, None)
    assert Q.shape[0] == Q.shape[1] and Q.shape[0] == A.shape[0], "The dimensions of Q %s must match those of A %s" % (Q.shape, A.shape)
    assert R.shape[0] == R.shape[1] and R.shape[0] == B.shape[1], "R %i must be square and must be compatible with B %i" % (R.shape, B.shape)

    assert numpy.linalg.matrix_rank(controllability(A, B)) == A.shape[0], "System must be completely controllable to do LQR."

    # TODO(Kyle): Ensure Q is positive-semidefninite and R is positive-definite

    # Solve the ARE for the cost-to-go matrix
    M = numpy.asmatrix(scipy.linalg.solve_continuous_are(A, B, Q, R))

    # Finally, solve for the optimal gain matrix using the cost-to-go matrix
    return numpy.asmatrix(numpy.linalg.inv(R) * B.T * M)

def dkalman(A, C, Q, R):
    """
    Calculate the continuous-time steady state Kalman gain matrix.

    Args:
        A: n*n discrete-time system dynamics matrix
        C: n*q measurement matrix
        Q: n*n discrete-time process noise covariance matrix
        R: q*q discrete-time measurement noise covariance matrix
    Returns:
        L: n*q gain matrix such that x_hat(n + 1) = (A-LC)x_hat(n) + Ly(n)
    """
    A = numpy.asmatrix(A)
    C = numpy.asmatrix(C)
    Q = numpy.asmatrix(Q)
    R = numpy.asmatrix(R)
    _validate_system(A, None, C, None)

    assert Q.shape[0] == Q.shape[1] and Q.shape[0] == A.shape[0], "The dimensions of Q %s must match those of A %s" % (Q.shape, A.shape)
    assert R.shape[0] == R.shape[1] and R.shape[0] == C.shape[0], "R %i must be square and must be compatible with C %i" % (R.shape, B.shape)

    assert numpy.linalg.matrix_rank(observability(A, C)) == A.shape[0], "System must be completely observable to compute Kalman gains."

    P = numpy.asmatrix(scipy.linalg.solve_discrete_are(A.T, C.T, Q, R))

    return numpy.asmatrix(P * C.T * numpy.linalg.inv(R))

def ckalman(A, C, Q, R):
    """
    Calculate the continuous-time steady state Kalman gain matrix.
    I can't even remember exactly what this optimizes because I suck
    at stochastic control theory.
    Args:
        A: n*n continuous-time system dynamics matrix
        C: n*q measurement matrix
        Q: n*n continuous-time process noise covariance matrix
        R: q*q continuous-time measurement noise covariance matrix
    Returns:
        L: n*q gain matrix such that dx_hat/dt = (A-LC)x_hat + Ly
    """
    A = numpy.asmatrix(A)
    C = numpy.asmatrix(C)
    Q = numpy.asmatrix(Q)
    R = numpy.asmatrix(R)
    _validate_system(A, None, C, None)

    assert Q.shape[0] == Q.shape[1] and Q.shape[0] == A.shape[0], "The dimensions of Q %s must match those of A %s" % (Q.shape, A.shape)
    assert R.shape[0] == R.shape[1] and R.shape[0] == C.shape[0], "R %i must be square and must be compatible with C %i" % (R.shape, B.shape)

    assert numpy.linalg.matrix_rank(observability(A, C)) == A.shape[0], "System must be completely observable to compute Kalman gains."

    P = numpy.asmatrix(scipy.linalg.solve_continuous_are(A.T, C.T, Q, R))

    return numpy.asmatrix(P * C.T * numpy.linalg.inv(R))

def feedforwards(A, B, Q = None):
    """
    The formula for Kff can be derived as follows:
        r(n+1) = A*r(n) + B*u_ff
        B*u_ff = r(n+1) - A*r(n)
        u_ff = pinv(B)*(r(n+1) - A*r(n))
        Kff = pinv(B)
        u_ff = Kff*(r(n+1) - A*r(n))
    There is also an LQR-weighted solution, but it gives the same u_ff assuming
    that there is some u_ff that satisfies the equation above.

    Args:
        A: n*n discrete-time system dynamics matrix
        B: n*m discrete-time control signal matrix
        Q: n*n LQR feedforwards weight matrix (optional)
    Returns:
        Kff: m*n feedforwards matrix such that u_ff = Kff * (r(n+1) - A*r(n))
    """
    if Q is None:
        return numpy.linalg.pinv(B)
    else:
        return numpy.linalg.inv(B.T * Q * B) * B.T * Q
