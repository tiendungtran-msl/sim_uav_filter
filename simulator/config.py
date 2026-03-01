"""Simulation and Sensor Configuration.

All noise parameters are realistic MEMS consumer-grade values.
DO NOT reduce these to make the filter look good — test robustness.
"""
from dataclasses import dataclass, field
import numpy as np


@dataclass
class SimConfig:
    # Simulation timing
    dt: float = 0.0025          # 400 Hz IMU
    total_time: float = 120.0   # 2-minute flight
    gps_rate: float = 10.0      # Hz
    baro_rate: float = 50.0     # Hz

    # Mission randomization
    seed: int = 42

    # Physical constants
    gravity: float = 9.80665   # m/s^2

    # UAV physical parameters
    mass: float = 1.5           # kg
    Ixx: float = 0.0147         # kg·m²
    Iyy: float = 0.0147
    Izz: float = 0.0283

    # Drag coefficients (linear)
    drag_xy: float = 0.15
    drag_z: float = 0.20


@dataclass
class SensorConfig:
    # ── IMU (MEMS consumer grade, e.g. ICM-42688-P) ──────────────────────
    # Gyroscope
    gyro_noise_spectral_density: float = 0.005    # rad/s/sqrt(Hz)  ARW
    gyro_bias_random_walk: float = 0.0002          # rad/s^2/sqrt(Hz)

    # Accelerometer
    accel_noise_spectral_density: float = 0.1      # m/s^2/sqrt(Hz)  VRW
    accel_bias_random_walk: float = 0.005          # m/s^2/sqrt(Hz)

    # Initial bias magnitudes (true bias injected at sim start)
    gyro_bias_init_sigma: float = 0.01             # rad/s
    accel_bias_init_sigma: float = 0.2             # m/s^2

    # ── GPS (civilian, open-sky, 1-2 m CEP) ──────────────────────────────
    gps_pos_noise_ned: float = 1.5                 # m  per-axis
    gps_vel_noise_ned: float = 0.15                # m/s per-axis
    gps_delay: float = 0.20                        # s  (200 ms)

    # ── Barometer ─────────────────────────────────────────────────────────
    baro_noise: float = 0.5                        # m
    baro_bias_drift_rate: float = 0.1              # m/s  random walk rate
