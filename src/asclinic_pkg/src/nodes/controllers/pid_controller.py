from dataclasses import dataclass
import numpy as np


@dataclass
class PIDController:
    kp_x: float
    ki_x: float
    kd_x: float
    ky: float
    kphi: float
    kiy: float = 0.0

    int_ex: float = 0.0
    int_ey: float = 0.0
    prev_ex: float = 0.0

    def reset(self) -> None:
        self.int_ex = 0.0
        self.int_ey = 0.0
        self.prev_ex = 0.0

    def step(self, e_hat: np.ndarray, dt: float) -> tuple[np.ndarray, dict]:
        """
        e_hat = [ex, ey, ephi]
        returns delta_u = [delta_v, delta_w]
        """
        ex = float(e_hat[0])
        ey = float(e_hat[1])
        ephi = float(e_hat[2])

        self.int_ex += ex * dt
        self.int_ey += ey * dt

        dex = (ex - self.prev_ex) / max(dt, 1e-9)
        self.prev_ex = ex

        delta_v = -(self.kp_x * ex + self.ki_x * self.int_ex + self.kd_x * dex)
        delta_w = -(self.ky * ey + self.kiy * self.int_ey + self.kphi * ephi)

        delta_u = np.array([delta_v, delta_w], dtype=float)

        aux = {
            "label": "PID-family baseline"
        }
        return delta_u, aux


def init_pid_controller(targets: dict, v_nom: float, opts: dict | None = None) -> PIDController:
    """
    Python version of your MATLAB init_pid_controller.
    """
    if opts is None:
        opts = {}

    sigma = max(float(targets["sigma"]), 1e-3)
    zeta = float(targets["zeta"])
    wn = float(targets["wn"])

    kp_x = 1.5 * sigma
    ki_x = 0.0
    kd_x = 0.0

    ky = wn**2 / max(v_nom, 1e-6)
    kphi = 2.0 * zeta * wn
    kiy = 0.0

    kp_x = opts.get("Kp_x", kp_x)
    ki_x = opts.get("Ki_x", ki_x)
    kd_x = opts.get("Kd_x", kd_x)
    ky = opts.get("Ky", ky)
    kphi = opts.get("Kphi", kphi)
    kiy = opts.get("Kiy", kiy)

    return PIDController(
        kp_x=kp_x,
        ki_x=ki_x,
        kd_x=kd_x,
        ky=ky,
        kphi=kphi,
        kiy=kiy,
    )