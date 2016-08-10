import numpy as np
import controls
import state_space_controller
from state_space_plant import state_space_plant
import state_space_observer

def _augment(plant):
    n_ua = plant.A_c.shape[0]

    A_c = np.zeros((n_ua + 1, n_ua + 1))
    A_c[:n_ua, :n_ua] = plant.A_c
    A_c[:n_ua, n_ua:n_ua+1] = plant.B_c

    B_c = np.zeros((plant.B_c.shape[0] + 1, plant.B_c.shape[1]))
    B_c[:n_ua, :1] = plant.B_c[0:n_ua]

    C = np.zeros((plant.C.shape[0], plant.C.shape[1] + 1))
    C[:1, :-1] = plant.C[0]

    D = plant.D

    x = np.zeros((n_ua + 1, 1))
    x[0:-2, 0] = plant.x[0:-1]

    return state_space_plant(plant.dt, x, A_c, B_c, C, D)

def error_u_lqr(plant, u_min = None, u_max = None, Q_c = None, R_c = None, Q_d = None, R_d = None):
    augmented_system = _augment(plant)

    if Q_c is not None and R_c is not None:
        _, _, Q_d, R_d = controls.c2d(augmented_system.A_c, augmented_system.B_c, augmented_system.dt, Q_c, R_c)
    elif Q_d is not None and R_d is not None:
        pass
    else:
        Q_d = plant.Q_d
        R_d = plant.R_d

    augmented_system.Q_c = Q_c
    augmented_system.R_c = R_c
    augmented_system.Q_d = Q_d
    augmented_system.R_d = R_d
    return state_space_controller.lqr(augmented_system, u_min, u_max)

def error_u_controller_poles(plant, poles, u_min = None, u_max = None):
    K = np.asmatrix(np.zeros((plant.B_d.shape[1], plant.A_d.shape[0] + 1)))
    K[0:1, 0:plant.A_d.shape[0]] = state_space_controller.placement(plant, poles).K
    augmented_system = _augment(plant)
    Kff = np.linalg.pinv(augmented_system.B_d)
    return state_space_controller.state_space_controller(K, Kff, augmented_system, u_min, u_max)

def error_u_kalman(plant, Q_c = None, R_c = None, Q_d = None, R_d = None):
    augmented_system = _augment(plant)

    if Q_c is not None and R_c is not None:
        _, _, Q_d, R_d = controls.c2d(augmented_system.A_c, augmented_system.B_c, augmented_system.dt, Q_c, R_c)
    elif Q_d is not None and R_d is not None:
        pass
    else:
        Q_d = plant.Q_d
        R_d = plant.R_d

    augmented_system.Q_c = Q_c
    augmented_system.R_c = R_c
    augmented_system.Q_d = Q_d
    augmented_system.R_d = R_d
    return state_space_observer.kalman(augmented_system)

def error_u_observer_poles(plant, poles):
    augmented_system = _augment(plant)
    return state_space_observer.placement(augmented_system)
