import regression
import numpy as np

def test_simple_system():
    labels =          'x' 'x'  'x'  'x' 'u'
    data = np.matrix([[1,  1,   2,   3,  0],
                      [2,  3,   5,   3, -1],
                      [4,  9,   8,   2,  1],
                      [8,  27,  10,  3, -2],
                      [16, 81,  13,  1, -1],
                      [32, 243, 14,  0, -1],
                      [64, 729, 14, -1,  0]])

    xt, ut = regression.seperate_data(data, labels)
    A, B = regression.state_space_regression(xt, ut)

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

def test_organizing():
    labels =           'x' 'u' '_' 'x' 'u'
    data = np.matrix([[ 1,  1,  4,  3,  0],
                      [ 2,  3,  5,  3, -1],
                      [ 5,  1,  8,  1,  1],
                      [ 6,  1,  2,  3, -2],
                      [ 7,  5, 13, -1, -1],
                      [12, -6, 14, -3, -1],
                      [ 6,  0, 14, -5,  0]])

    xt, ut = regression.seperate_data(data, labels)
    A, B = regression.state_space_regression(xt, ut)

    goal_A = np.matrix([[1, 0],
                        [0, 1]])

    goal_B = np.matrix([[1, 0],
                        [0, 2]])

    np.testing.assert_array_almost_equal(A, goal_A)
    np.testing.assert_array_almost_equal(B, goal_B)

def test_dense_matrix():
    labels =           'x' 'x' 'x'
    data = np.matrix([[ 2,  3,  0],
                      [-1,  5,  3],
                      [-3,  4,  8],
                      [ 1,  1, 12],
                      [12,  2, 13],
                      [23, 14, 15]])

    xt, ut = regression.seperate_data(data, labels)
    A, B = regression.state_space_regression(xt, ut)

    goal_A = np.matrix([[1, -1, 1],
                        [1,  1, 0],
                        [0,  1, 1]])

    np.testing.assert_array_almost_equal(A, goal_A)

def test_velocity_missing():
    labels =          'x''u'
    data = np.matrix([[0, 1],
                      [0, 1],
                      [1, 1],
                      [3, 1],
                      [6, 1]])

    position, ut = regression.seperate_data(data, labels)
    xt = regression.calculate_state(position, 1)
    A, B = regression.state_space_regression(xt, ut)

    goal_A = np.matrix([[1, 1],
                        [0, 1]])

    goal_B = np.matrix([[0],
                        [1]])

    np.testing.assert_array_almost_equal(A, goal_A)
    np.testing.assert_array_almost_equal(B, goal_B)

test_simple_system()
test_organizing()
test_dense_matrix()
test_velocity_missing()
