import numpy as np
import re

def state_space_regression(xt, ut):

    """
    The equation we want to solve is:

        | x1'   u1'   |           | x2' |
        | x2'   u2'   |  | A' |   | x3' |
        | x3'   u3'   |  |    | = | x4' |
        | .........   |  | B' |   | ... |
        | xn-1' un-1' |           | xn' |

    """

    # Combine xt and ut, drop the last row
    solver_A = np.hstack((xt, ut))[:-1, :]
    # All but the first for of xt
    solver_b = xt[1:, :]
    solution = np.linalg.lstsq(solver_A, solver_b)[0]
    
    # The first n rows of the solution, where x is
    # the number of states (number of cols in xt)
    gains_A = solution[:xt.shape[1], :].T
    # All but the first n rows of the solution
    gains_B = solution[xt.shape[1]:, :].T

    return gains_A, gains_B


def seperate_data(data, labels):
    """
    Given a data file such as

        | x11 u11 x12 other x13 |
	| x21 u21 x22 other x23 |
	| ..................... |
    
    And labels for each column such as

          'x' 'u' 'x' '_'   'x'

    Split the data into x' and u'

    """

    # Take the index of all instances of 'x', get the cols with
    # that index, which should be the states.
    xt = data[:, [m.start() for m in re.finditer('x', labels)]]
    # Save as above, but with 'u' and inputs.
    ut = data[:, [m.start() for m in re.finditer('u', labels)]]
    return xt, ut
