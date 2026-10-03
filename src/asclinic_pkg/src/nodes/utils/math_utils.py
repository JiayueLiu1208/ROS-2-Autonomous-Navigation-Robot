import numpy as np


def wrap_to_pi(angle: float) -> float:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def compute_error_state(state: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """
    state = [x, y, phi]
    ref   = [x_ref, y_ref, phi_ref]

    error expressed in reference frame:
    e = [ex, ey, ephi]
    """
    x, y, phi = state
    x_ref, y_ref, phi_ref = ref

    dx = x - x_ref
    dy = y - y_ref

    rot = np.array([
        [np.cos(phi_ref),  np.sin(phi_ref)],
        [-np.sin(phi_ref), np.cos(phi_ref)]
    ])

    ex, ey = rot @ np.array([dx, dy], dtype=float)
    ephi = wrap_to_pi(phi - phi_ref)

    return np.array([ex, ey, ephi], dtype=float)