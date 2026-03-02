"""sim.sensors.mag — Mô hình Magnetometer 3 trục.

Mục tiêu:
- Cung cấp dữ liệu đo từ trường trong body frame để phục vụ quan sát bay.
- Tạo dữ liệu đầu vào thực tế cho thuật toán ước lượng heading/yaw sau này.

Mô hình:
    mag_meas = mag_true_body + b_m + w_m
    b_m(k+1) = b_m(k) + N(0, sigma_rw^2 * dt)

Trong đó mag_true_body lấy từ vector từ trường chuẩn trong NED quay về body.
"""
import numpy as np
from typing import Optional

from sim.config import SimConfig, SensorConfig
from sim.dynamics.rigid_body_6dof import UAV6DOF
from sim.io.schema import MagMeas


class MagModel:
    """Mô hình cảm biến magnetometer với rate/noise/bias drift."""

    def __init__(self, cfg: SimConfig, scfg: SensorConfig, rng: np.random.Generator):
        self.rng = rng
        self.dt = cfg.dt

        self.dt_mag = 1.0 / scfg.mag_rate
        self._t_next = 0.0

        self.sigma_noise = scfg.mag_noise
        self.sigma_rw = scfg.mag_bias_rw * np.sqrt(cfg.dt)
        self.bias = rng.normal(0.0, scfg.mag_bias_init_sigma, 3)

        # Từ trường địa cầu xấp xỉ tại vĩ độ trung bình (uT) trong NED.
        # Thành phần D dương theo NED (hướng xuống).
        self.mag_ned = np.array([22.0, 1.0, 41.0], dtype=float)

    def truth_body(self, uav: UAV6DOF) -> np.ndarray:
        """Tính từ trường thật trong body frame tại trạng thái hiện tại."""
        return uav.R_body_to_ned.T @ self.mag_ned

    def update(self, uav: UAV6DOF) -> Optional[MagMeas]:
        """Cập nhật sensor mag theo rate cấu hình."""
        t = uav.t
        if t < self._t_next:
            return None

        self._t_next += self.dt_mag
        self.bias += self.rng.normal(0.0, self.sigma_rw, 3)

        mag_true = self.truth_body(uav)
        mag_meas = mag_true + self.bias + self.rng.normal(0.0, self.sigma_noise, 3)
        return MagMeas(t=t, mag_body=mag_meas, valid=True)
