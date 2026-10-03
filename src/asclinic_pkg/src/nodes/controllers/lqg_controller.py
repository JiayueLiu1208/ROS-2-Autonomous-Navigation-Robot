import numpy as np
from scipy.linalg import solve_discrete_are

try:
    from estimator.kalman_filter import DiscreteKalmanFilter
    from control_models.kinematic_error_model import linearise_error_model, discretise_zoh
except ModuleNotFoundError:
    import sys
    from pathlib import Path

    _NODES_DIR = Path(__file__).resolve().parents[1]
    if str(_NODES_DIR) not in sys.path:
        sys.path.insert(0, str(_NODES_DIR))

    from estimator.kalman_filter import DiscreteKalmanFilter
    from control_models.kinematic_error_model import linearise_error_model, discretise_zoh


class LQGController:
    def __init__(self, dt, Q_lqr, R_lqr, Qn, Rn, u_min=None, u_max=None):
        self.dt = float(dt)

        self.Q_lqr = np.array(Q_lqr, dtype=float)
        self.R_lqr = np.array(R_lqr, dtype=float)
        self.Qn = np.array(Qn, dtype=float)
        self.Rn = np.array(Rn, dtype=float)

        self.u_min = np.array(u_min, dtype=float).reshape(-1, 1) if u_min is not None else None
        self.u_max = np.array(u_max, dtype=float).reshape(-1, 1) if u_max is not None else None

        self.kf = DiscreteKalmanFilter(x0=np.zeros((3, 1)), P0=np.eye(3) * 1e-3)
        self.u_prev = np.zeros((2, 1))

    def reset(self):
        self.kf.reset(np.zeros((3, 1)), np.eye(3) * 1e-3)
        self.u_prev = np.zeros((2, 1))

    def dlqr(self, Ad, Bd):
        P = solve_discrete_are(Ad, Bd, self.Q_lqr, self.R_lqr)
        K = np.linalg.inv(Bd.T @ P @ Bd + self.R_lqr) @ (Bd.T @ P @ Ad)
        return K

    def step(self, e_meas, v_ref, omega_ref):
        A, B, C, D = linearise_error_model(v_ref, omega_ref)
        Ad, Bd, Cd, Dd = discretise_zoh(A, B, C, D, self.dt)

        K_lqr = self.dlqr(Ad, Bd)

        xhat, Kf, innov = self.kf.step(
            Ad, Bd, Cd, self.Qn, self.Rn,
            self.u_prev,
            np.array(e_meas, dtype=float).reshape(3, 1)
        )

        delta_u = -K_lqr @ xhat

        if self.u_min is not None:
            delta_u = np.maximum(delta_u, self.u_min)
        if self.u_max is not None:
            delta_u = np.minimum(delta_u, self.u_max)

        self.u_prev = delta_u.copy()

        aux = {
            "K_lqr": K_lqr,
            "xhat": xhat,
            "Kf": Kf,
            "innovation": innov,
            "label": "LQG"
        }

        return delta_u.flatten(), aux
