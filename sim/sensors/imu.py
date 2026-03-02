"""sim.sensors.imu — Mô hình IMU (Gyroscope + Accelerometer) 400 Hz.

Mô hình noise:
    gyro_meas  = omega_true + b_g + w_g
    accel_meas = sf_true    + b_a + w_a

Trong đó:
    b_g, b_a: bias, drift theo random walk (mỗi dt).
    w_g, w_a: white noise (Gaussian).

Tham số noise lấy từ SensorConfig, tương ứng MEMS consumer-grade.
"""
import numpy as np
from numpy.typing import NDArray

from sim.config import SimConfig, SensorConfig
from sim.dynamics.rigid_body_6dof import UAV6DOF
from sim.io.schema import ImuMeas
from .models import RawImuSample


class ImuModel:
    """Mô hình IMU với bias random walk + white noise."""

    def __init__(self, cfg: SimConfig, scfg: SensorConfig, rng: np.random.Generator):
        self.dt = cfg.dt
        self.rng = rng

        # ---- Tính sigma cho mỗi bước dt ----
        # White noise: σ_discrete = σ_spectral / √(dt)
        self.sigma_g_white = scfg.gyro_noise_spectral_density / np.sqrt(self.dt)
        self.sigma_a_white = scfg.accel_noise_spectral_density / np.sqrt(self.dt)

        # Bias random walk: σ_step = σ_rw * √(dt)
        self.sigma_g_rw = scfg.gyro_bias_random_walk * np.sqrt(self.dt)
        self.sigma_a_rw = scfg.accel_bias_random_walk * np.sqrt(self.dt)

        # ---- Bias ban đầu (random, simulator biết nhưng filter không) ----
        self.b_g = rng.normal(0.0, scfg.gyro_bias_init_sigma, 3)
        self.b_a = rng.normal(0.0, scfg.accel_bias_init_sigma, 3)

    def sample(self, uav: UAV6DOF) -> RawImuSample:
        """Lấy 1 mẫu IMU tại trạng thái hiện tại của UAV.

        Returns:
            RawImuSample chứa cả giá trị đo và giá trị thật (để debug/log).
        """
        # 1) Drift bias (random walk mỗi bước)
        self.b_g += self.rng.normal(0.0, self.sigma_g_rw, 3)
        self.b_a += self.rng.normal(0.0, self.sigma_a_rw, 3)

        # 2) Giá trị thật
        omega_true = uav.omega.copy()
        sf_true = uav.specific_force_body  # specific force body (không có gravity)

        # 3) Đo = thật + bias + white noise
        gyro_meas  = omega_true + self.b_g + self.rng.normal(0.0, self.sigma_g_white, 3)
        accel_meas = sf_true    + self.b_a + self.rng.normal(0.0, self.sigma_a_white, 3)

        return RawImuSample(
            t=uav.t,
            gyro_meas=gyro_meas,
            accel_meas=accel_meas,
            gyro_true=omega_true,
            accel_true=sf_true,
            gyro_bias=self.b_g.copy(),
            accel_bias=self.b_a.copy(),
        )

    def to_schema(self, raw: RawImuSample) -> ImuMeas:
        """Chuyển RawImuSample → ImuMeas (schema chuẩn)."""
        return ImuMeas(t=raw.t, gyro=raw.gyro_meas.copy(), accel=raw.accel_meas.copy())
