import numpy as np

try:
    from control_models.kinematic_error_model import linearise_error_model, discretise_zoh
except ModuleNotFoundError:
    import sys
    from pathlib import Path

    _NODES_DIR = Path(__file__).resolve().parents[1]
    if str(_NODES_DIR) not in sys.path:
        sys.path.insert(0, str(_NODES_DIR))

    from control_models.kinematic_error_model import linearise_error_model, discretise_zoh


class MPCController:
    def __init__(self, dt, Q, R, Np=10, u_min=None, u_max=None):
        self.dt = float(dt)
        self.Q = np.array(Q, dtype=float)
        self.R = np.array(R, dtype=float)
        self.Np = int(Np)

        self.u_min = np.array(u_min, dtype=float).reshape(-1, 1) if u_min is not None else None
        self.u_max = np.array(u_max, dtype=float).reshape(-1, 1) if u_max is not None else None

    def _build_prediction_matrices(self, Ad, Bd):
        nx = Ad.shape[0]
        nu = Bd.shape[1]
        Np = self.Np

        Phi = np.zeros((nx * Np, nx))
        Gamma = np.zeros((nx * Np, nu * Np))

        for i in range(Np):
            Phi[i*nx:(i+1)*nx, :] = np.linalg.matrix_power(Ad, i+1)
            for j in range(i+1):
                Gamma[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = np.linalg.matrix_power(Ad, i-j) @ Bd

        return Phi, Gamma

    def step(self, e_hat, v_ref, omega_ref):
        A, B, C, D = linearise_error_model(v_ref, omega_ref)
        Ad, Bd, Cd, Dd = discretise_zoh(A, B, C, D, self.dt)

        nx = Ad.shape[0]
        nu = Bd.shape[1]
        Np = self.Np

        Phi, Gamma = self._build_prediction_matrices(Ad, Bd)

        Qbar = np.kron(np.eye(Np), self.Q)
        Rbar = np.kron(np.eye(Np), self.R)

        x0 = np.array(e_hat, dtype=float).reshape(nx, 1)

        H = Gamma.T @ Qbar @ Gamma + Rbar
        H = 0.5 * (H + H.T) + 1e-8 * np.eye(H.shape[0])

        f = (Gamma.T @ Qbar @ Phi @ x0).flatten()

        # unconstrained solution first
        U_star = -np.linalg.solve(H, f.reshape(-1, 1))

        u0 = U_star[:nu].copy()

        if self.u_min is not None:
            u0 = np.maximum(u0, self.u_min)
        if self.u_max is not None:
            u0 = np.minimum(u0, self.u_max)

        aux = {
            "H": H,
            "label": "MPC"
        }

        return u0.flatten(), aux
