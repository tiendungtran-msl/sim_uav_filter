"""sim.sensors.gps — Mô hình GPS 10 Hz với noise, delay cố định, và dropout.

Đặc điểm:
- Rate: 10 Hz (mỗi 0.1 s lấy 1 mẫu).
- Noise: Gaussian trên pos (1.5 m 1σ) và vel (0.15 m/s 1σ).
- Delay: 200 ms cố định — dùng DelayBuffer.
- Dropout: khi gps_active=False (do scenario), không lấy mẫu.
"""
import numpy as np
from numpy.typing import NDArray
from typing import Optional

from sim.config import SimConfig, SensorConfig
from sim.dynamics.rigid_body_6dof import UAV6DOF
from sim.utils.ring_buffer import DelayBuffer
from sim.io.schema import GpsMeas
from .models import RawGpsSample


class GpsModel:
    """Mô hình GPS với noise, delay, và dropout support."""

    def __init__(self, cfg: SimConfig, scfg: SensorConfig, rng: np.random.Generator):
        self.rng = rng

        # Rate control
        self.dt_gps = 1.0 / scfg.gps_rate
        self._t_next = 0.0     # thời điểm lấy mẫu tiếp theo

        # Noise
        self.sigma_p = scfg.gps_pos_noise
        self.sigma_v = scfg.gps_vel_noise

        # Delay buffer
        self._delay_buf = DelayBuffer(delay=scfg.gps_delay)

    def update(self, uav: UAV6DOF, gps_active: bool) -> Optional[GpsMeas]:
        """Cập nhật GPS mỗi tick IMU.

        Args:
            uav:        Trạng thái UAV hiện tại.
            gps_active: True nếu GPS đang hoạt động (không dropout).

        Returns:
            GpsMeas nếu có mẫu đã hết delay, None nếu chưa.
        """
        t = uav.t

        # 1) Lấy mẫu mới nếu đến lượt & GPS active
        if t >= self._t_next:
            self._t_next += self.dt_gps
            if gps_active:
                raw = RawGpsSample(
                    t_stamp=t,
                    pos_ned=uav.p + self.rng.normal(0.0, self.sigma_p, 3),
                    vel_ned=uav.v + self.rng.normal(0.0, self.sigma_v, 3),
                )
                self._delay_buf.push(t, raw)

        # 2) Lấy mẫu cũ đã hết delay
        result = self._delay_buf.pop(t)
        if result is not None:
            raw: RawGpsSample = result
            return GpsMeas(
                t_stamp=raw.t_stamp,
                t_receive=t,
                pos_ned=raw.pos_ned.copy(),
                vel_ned=raw.vel_ned.copy(),
                valid=True,
            )
        return None
