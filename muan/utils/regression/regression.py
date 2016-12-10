import numpy as np
import sys

"""
Regression for state space systems. Given a .csv log file (noise is
acceptable), the A and B matrices are generated. The csv file must be
in the following form:

    | x1' u1' |
    | x2' u2' |
    | x3' u3' |
    | ....... |
    | xn' un' |

"""

# least-squares approximation of Ax = b
def least_squares(A, b):
    assert np.linalg.cond(A) < 1 / sys.float_info.epsilon, "A.T * A is singular."
    return np.linalg.inv(A.T.dot(A)).dot(A.T).dot(b)

def state_space_regression(data, numstates):

    """
    The equation we want to solve is:

        | x1'   u1'   |           | x2' |
        | x2'   u2'   |  | A' |   | x3' |
        | x3'   u3'   |  |    | = | x4' |
        | .........   |  | B' |   | ... |
        | xn-1' un-1' |           | xn' |

    """
    A = data[:-1, :]
    b = data[1:, :numstates]
    x = least_squares(A, b)

    gains_A = x[:numstates, :].T
    gains_B = x[numstates:, :].T

    return gains_A, gains_B
