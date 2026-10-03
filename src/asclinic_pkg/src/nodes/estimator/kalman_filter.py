import numpy as np


class DiscreteKalmanFilter:
    def __init__(self, x0, P0):
        self.xhat = np.array(x0, dtype=float).reshape(-1, 1)
        self.P = np.array(P0, dtype=float)

    def reset(self, x0, P0):
        self.xhat = np.array(x0, dtype=float).reshape(-1, 1)
        self.P = np.array(P0, dtype=float)

    def step(self, Ad, Bd, Cd, Qn, Rn, u, y):
        u = np.array(u, dtype=float).reshape(-1, 1)
        y = np.array(y, dtype=float).reshape(-1, 1)

        # Predict
        x_pred = Ad @ self.xhat + Bd @ u
        P_pred = Ad @ self.P @ Ad.T + Qn

        # Update
        S = Cd @ P_pred @ Cd.T + Rn
        K = P_pred @ Cd.T @ np.linalg.inv(S)

        innovation = y - Cd @ x_pred
        self.xhat = x_pred + K @ innovation
        I = np.eye(P_pred.shape[0])
        residual = I - K @ Cd
        self.P = residual @ P_pred @ residual.T + K @ Rn @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        return self.xhat.copy(), K.copy(), innovation.copy()
