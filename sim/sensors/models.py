"""sim.sensors.models — Dataclasses nội bộ cho sensor samples.

File này chứa các cấu trúc dữ liệu trung gian mà các sensor model dùng nội bộ.
Dữ liệu cuối cùng xuất ra ngoài qua sim.io.schema (ImuMeas, GpsMeas, BaroMeas).
"""
from dataclasses import dataclass
import numpy as np
from numpy.typing import NDArray


@dataclass
class RawImuSample:
    """Mẫu IMU thô (nội bộ sensor model, chưa format thành ImuMeas)."""
    t: float
    gyro_meas: NDArray       # gyro đo được (body) rad/s
    accel_meas: NDArray      # accel đo được (body) m/s²
    gyro_true: NDArray       # omega thật (body) rad/s
    accel_true: NDArray      # specific force thật (body) m/s²
    gyro_bias: NDArray       # bias gyro hiện tại
    accel_bias: NDArray      # bias accel hiện tại


@dataclass
class RawGpsSample:
    """Mẫu GPS thô (trước khi qua delay buffer)."""
    t_stamp: float
    pos_ned: NDArray
    vel_ned: NDArray


@dataclass
class RawBaroSample:
    """Mẫu Baro thô."""
    t: float
    alt_m: float
