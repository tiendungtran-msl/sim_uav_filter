"""sim.config — Cấu hình mô phỏng tập trung.

Bao gồm:
- SimConfig: tham số chung (dt, thời gian, seed, vật lý UAV).
- SensorConfig: thông số noise/bias/rate/delay cho các sensor.

Tất cả giá trị mặc định là realistic MEMS consumer-grade.
KHÔNG giảm noise để "filter đẹp" — phải test robustness.
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class SimConfig:
    """Tham số vật lý & chung của mô phỏng."""

    # --- Thời gian ---
    dt: float = 0.0025              # bước dt IMU (400 Hz)
    total_time: float = 60.0        # tổng thời gian mô phỏng (s)
    seed: int = 42                  # seed cho reproducibility

    # --- Vật lý UAV ---
    gravity: float = 9.80665        # m/s²
    mass: float = 1.5               # kg
    Ixx: float = 0.0147             # kg·m² — momen quán tính trục x body
    Iyy: float = 0.0147             # kg·m² — trục y body
    Izz: float = 0.0283             # kg·m² — trục z body

    # --- Drag đơn giản ---
    drag_xy: float = 0.15           # hệ số drag mặt phẳng ngang (N·s/m)
    drag_z: float = 0.20            # hệ số drag trục Z NED

    # --- GUI ---
    gui_plot_duration: float = 15.0  # thời lượng hiển thị trên plot (s)
    gui_update_ms: int = 50          # chu kỳ cập nhật GUI (ms) → ~20 FPS


@dataclass
class SensorConfig:
    """Thông số noise & hoạt động của các sensor."""

    # ---- Gyroscope ----
    gyro_noise_spectral_density: float = 0.005   # rad/s/√Hz  (ARW)
    gyro_bias_random_walk: float = 0.0002         # rad/s²/√Hz
    gyro_bias_init_sigma: float = 0.01            # rad/s — sigma bias ban đầu

    # ---- Accelerometer ----
    accel_noise_spectral_density: float = 0.1     # m/s²/√Hz  (VRW)
    accel_bias_random_walk: float = 0.005          # m/s²/√Hz
    accel_bias_init_sigma: float = 0.2             # m/s² — sigma bias ban đầu

    # ---- GPS ----
    gps_rate: float = 10.0            # Hz
    gps_pos_noise: float = 1.5        # m   per-axis (1σ)
    gps_vel_noise: float = 0.15       # m/s per-axis (1σ)
    gps_delay: float = 0.20           # s   (200 ms delay cố định)

    # ---- Barometer ----
    baro_rate: float = 50.0            # Hz
    baro_noise: float = 0.5            # m  (1σ)
    baro_bias_drift_rate: float = 0.1  # m/s — tốc độ drift bias (random walk)

    # ---- Magnetometer ----
    mag_rate: float = 50.0             # Hz
    mag_noise: float = 0.8             # uT (1σ)
    mag_bias_init_sigma: float = 2.0   # uT
    mag_bias_rw: float = 0.03          # uT/sqrt(s)
