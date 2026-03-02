"""sim.sensors.baro — Mô hình Barometer 50 Hz với noise & bias drift.

Đặc điểm:
- Rate: 50 Hz.
- Noise: Gaussian 0.5 m (1σ).
- Bias: drift dạng random walk.
- Altitude = −p_D (NED: D hướng xuống → alt dương hướng lên).
"""
import numpy as np
from typing import Optional

from sim.config import SimConfig, SensorConfig
from sim.dynamics.rigid_body_6dof import UAV6DOF
from sim.io.schema import BaroMeas


class BaroModel:
    """Mô hình barometer với noise & bias drift."""

    def __init__(self, cfg: SimConfig, scfg: SensorConfig, rng: np.random.Generator):
        self.rng = rng
        self.dt = cfg.dt

        # Rate control
        self.dt_baro = 1.0 / scfg.baro_rate
        self._t_next = 0.0

        # Noise
        self.sigma_noise = scfg.baro_noise

        # Bias drift (random walk)
        self._bias = rng.normal(0.0, 0.3)   # bias ban đầu nhỏ
        self._sigma_drift = scfg.baro_bias_drift_rate * np.sqrt(cfg.dt)

    def update(self, uav: UAV6DOF) -> Optional[BaroMeas]:
        """Cập nhật baro mỗi tick IMU.

        Returns:
            BaroMeas nếu đến lượt lấy mẫu, None nếu chưa.
        """
        t = uav.t
        if t < self._t_next:
            return None

        self._t_next += self.dt_baro

        # Bias drift
        self._bias += self.rng.normal(0.0, self._sigma_drift)

        # Altitude thật & đo
        alt_true = -uav.p[2]   # NED: D xuống → alt = -D
        alt_meas = alt_true + self._bias + self.rng.normal(0.0, self.sigma_noise)

        return BaroMeas(t=t, alt_m=alt_meas, valid=True)
