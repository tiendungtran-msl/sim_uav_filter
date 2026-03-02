"""sim.utils.units — Hằng số vật lý & hàm chuyển đổi đơn vị.

Tập trung tất cả hằng số ở 1 chỗ để tránh magic number rải rác.
"""
import numpy as np

# ---------------------------------------------------------------------------
# Hằng số vật lý
# ---------------------------------------------------------------------------
GRAVITY = 9.80665       # m/s²  — gia tốc trọng trường chuẩn
G_NED = np.array([0.0, 0.0, GRAVITY])   # vector trọng lực trong frame NED (down = +)

# ---------------------------------------------------------------------------
# Chuyển đổi góc
# ---------------------------------------------------------------------------
DEG2RAD = np.pi / 180.0
RAD2DEG = 180.0 / np.pi


def deg2rad(deg: float) -> float:
    """Chuyển độ sang radian."""
    return deg * DEG2RAD


def rad2deg(rad: float) -> float:
    """Chuyển radian sang độ."""
    return rad * RAD2DEG


# ---------------------------------------------------------------------------
# Chuyển đổi áp suất / độ cao (ISA đơn giản)
# ---------------------------------------------------------------------------
_P0 = 101325.0          # Pa  — áp suất mực nước biển
_T0 = 288.15            # K   — nhiệt độ mực nước biển
_L  = 0.0065            # K/m — lapse rate
_R  = 287.058           # J/(kg·K)


def pressure_to_altitude(p_pa: float) -> float:
    """Chuyển áp suất (Pa) → độ cao ISA (m). Chỉ dùng dưới 11 km."""
    return (_T0 / _L) * (1.0 - (p_pa / _P0) ** (_R * _L / GRAVITY))


def altitude_to_pressure(alt_m: float) -> float:
    """Chuyển độ cao ISA (m) → áp suất (Pa)."""
    return _P0 * (1.0 - _L * alt_m / _T0) ** (GRAVITY / (_R * _L))
