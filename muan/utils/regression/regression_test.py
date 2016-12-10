import regression
import numpy as np

labels =          'x' 'x'  'x'  'x' 'u'
data = np.matrix([[1,  1,   2,   3,  0],
                  [2,  3,   5,   3, -1],
                  [4,  9,   8,   2,  1],
                  [8,  27,  10,  3, -2],
                  [16, 81,  13,  1, -1],
                  [32, 243, 14,  0, -1],
                  [64, 729, 14, -1,  0]])
numstates = 4
A, B = regression.state_space_regression(data, numstates)
goal_A = np.matrix([[2, 0, 0, 0],
                    [0, 3, 0, 0],
                    [0, 0, 1, 1],
                    [0, 0, 0, 1]])

goal_B = np.matrix([[0],
                    [0],
                    [0],
                    [1]])

np.testing.assert_array_almost_equal(A, goal_A)
np.testing.assert_array_almost_equal(B, goal_B)
