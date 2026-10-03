import numpy as np
from scipy.signal import cont2discrete

def linearise_error_model(v_ref: float, omega_ref: float):
    A = np.array([
        [0.0,        omega_ref, 0.0],
        [-omega_ref, 0.0,       v_ref],
        [0.0,        0.0,       0.0]
    ], dtype=float)

    B = np.array([
        [1.0, 0.0],
        [0.0, 0.0],
        [0.0, 1.0]
    ], dtype=float)

    C = np.eye(3, dtype=float)
    D = np.zeros((3, 2), dtype=float)
    return A, B, C, D


def discretise_zoh(A, B, C, D, dt: float):
    Ad, Bd, Cd, Dd, _ = cont2discrete((A, B, C, D), dt, method='zoh')
    return Ad, Bd, Cd, Dd