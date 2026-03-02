"""sim.io.schema — Dataclasses định nghĩa dữ liệu truth & measurement.

Đây là "interface" giữa simulator và bất kỳ consumer nào (filter, logger, GUI).
Mọi thứ simulator xuất ra đều thông qua SimFrame.

Quy ước frame:
- Vị trí, vận tốc: NED (North-East-Down)
- Quaternion: body → NED, dạng [w, x, y, z] (Hamilton, scalar-first)
- Specific force, omega, bias: body frame
- Altitude (baro): = −p_D  (dương hướng lên)
"""
from dataclasses import dataclass, field
from typing import Optional
import numpy as np
from numpy.typing import NDArray


# ======================================================================
# Truth state (ground truth — simulator biết, filter KHÔNG được biết)
# ======================================================================

@dataclass
class TruthState:
    """Trạng thái thật của UAV tại 1 thời điểm."""
    t: float                        # thời gian mô phỏng (s)
    pos_ned: NDArray                # vị trí NED (3,) m
    vel_ned: NDArray                # vận tốc NED (3,) m/s
    quat: NDArray                   # quaternion body→NED (4,) [w,x,y,z]
    omega_body: NDArray             # vận tốc góc body (3,) rad/s
    specific_force_body: NDArray    # specific force body (3,) m/s² (cái IMU đo)
    mag_body: NDArray               # từ trường thật trong body frame (3,) uT
    euler_deg: NDArray              # [roll, pitch, yaw] (3,) độ — tiện hiển thị

    # Bias thật (ẩn với filter, simulator dùng để debug)
    gyro_bias: NDArray              # bias gyro body (3,) rad/s
    accel_bias: NDArray             # bias accel body (3,) m/s²


# ======================================================================
# Sensor measurements
# ======================================================================

@dataclass
class ImuMeas:
    """Mẫu đo IMU (400 Hz): gyro + accelerometer."""
    t: float                # timestamp (s)
    gyro: NDArray           # gyro đo được (3,) rad/s  (= omega + bias + noise)
    accel: NDArray          # accel đo được (3,) m/s²  (= sf + bias + noise)


@dataclass
class GpsMeas:
    """Mẫu đo GPS (10 Hz), có delay."""
    t_stamp: float          # timestamp lúc GPS lấy mẫu (trước delay)
    t_receive: float        # timestamp lúc nhận được (sau delay)
    pos_ned: NDArray        # vị trí NED (3,) m
    vel_ned: NDArray        # vận tốc NED (3,) m/s
    valid: bool             # False khi GPS dropout


@dataclass
class BaroMeas:
    """Mẫu đo Barometer (50 Hz)."""
    t: float                # timestamp (s)
    alt_m: float            # altitude đo được (m) — dương hướng lên
    valid: bool


@dataclass
class MagMeas:
    """Mẫu đo Magnetometer (thường 50 Hz)."""
    t: float                # timestamp (s)
    mag_body: NDArray       # từ trường đo được body (3,) uT
    valid: bool


# ======================================================================
# SimFrame — gói dữ liệu 1 tick của simulator
# ======================================================================

@dataclass
class SimFrame:
    """Một "khung hình" dữ liệu mà simulator xuất ra mỗi tick (dt).

    Consumer (GUI, logger, filter) chỉ cần nhận SimFrame.
    - truth:  luôn có.
    - imu:    luôn có (mỗi tick = 1 IMU sample).
    - gps:    Optional — chỉ có khi GPS phát mẫu & đã hết delay.
    - baro:   Optional — chỉ có khi baro phát mẫu.
    """
    truth: TruthState
    imu: ImuMeas
    gps: Optional[GpsMeas] = None
    baro: Optional[BaroMeas] = None
    mag: Optional[MagMeas] = None

    # Metadata (để debug / log)
    phase_name: str = ""            # tên phase hiện tại
    gps_active: bool = True         # GPS có đang hoạt động không (trước delay)
