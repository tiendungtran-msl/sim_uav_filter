"""sim.scenario.phases — Định nghĩa các phase bay (hành vi UAV).

Mỗi phase là 1 hành vi trong 1 khoảng thời gian:
1) straight    — bay thẳng, giữ/đổi tốc độ
2) turn        — cua trái/phải với yaw_rate
3) climb       — tăng/giảm độ cao
4) pitch_maneuver — pitch lên/xuống nhẹ (gia tốc dọc)
5) aggressive_yaw — yaw nhanh, ngắn hạn
6) wind_gust   — gió nhiễu đột ngột
7) gps_dropout — mất GPS trong khoảng thời gian

Mỗi phase KHÔNG tự tích phân dynamics — chỉ đặt thrust_body & torque_body
cho UAV6DOF. PhaseScheduler gọi phase.apply() mỗi tick.
"""
from dataclasses import dataclass, field
from typing import Dict, Any
import numpy as np
from numpy.typing import NDArray

from sim.config import SimConfig
from sim.dynamics.rigid_body_6dof import UAV6DOF
from sim.dynamics.atmosphere import Atmosphere


@dataclass
class Phase:
    """Mô tả 1 giai đoạn bay."""
    kind: str                   # tên loại phase
    duration: float             # thời lượng (s)
    params: Dict[str, Any] = field(default_factory=dict)


def apply_phase(phase: Phase, uav: UAV6DOF, atmo: Atmosphere,
                elapsed: float, cfg: SimConfig) -> bool:
    """Áp dụng phase lên UAV: đặt thrust_body & torque_body.

    Args:
        phase:   Phase hiện tại.
        uav:     UAV state hiện tại (sẽ bị thay đổi thrust/torque).
        atmo:    Atmosphere model (dùng cho wind_gust phase).
        elapsed: Thời gian đã trôi qua trong phase này (s).
        cfg:     SimConfig.

    Returns:
        gps_active: True nếu GPS đang hoạt động trong phase này.
    """
    g = cfg.gravity
    m = cfg.mass
    max_thrust = 4.0 * m * g   # giới hạn vật lý hợp lý

    def clamp(val: float) -> float:
        return float(np.clip(val, -max_thrust, max_thrust))

    def speed_control_thrust_x(target_speed: float) -> float:
        """Điều khiển thrust dọc trục X để bám tốc độ tiến."""
        v_body = uav.R_body_to_ned.T @ uav.v
        speed_err = target_speed - v_body[0]
        a_x_des = np.clip(1.8 * speed_err, -4.0, 4.0)
        return clamp(m * a_x_des)

    def altitude_hold_thrust_z(target_alt: float) -> float:
        """Giữ độ cao theo target_alt (m, dương hướng lên)."""
        alt_err = -target_alt - uav.p[2]
        vel_d_err = 0.0 - uav.v[2]
        a_z_des = np.clip(1.2 * alt_err + 1.8 * vel_d_err, -4.5, 4.5)
        return clamp(-m * (g - a_z_des))

    def climb_rate_thrust_z(climb_rate: float) -> float:
        """Điều khiển theo tốc độ leo/hạ (m/s, dương = lên)."""
        vel_d_target = -climb_rate
        vel_d_err = vel_d_target - uav.v[2]
        a_z_des = np.clip(2.5 * vel_d_err, -5.0, 5.0)
        return clamp(-m * (g - a_z_des))

    kind = phase.kind
    gps_active = True

    if kind == "straight":
        # Bay thẳng: giữ độ cao + cruise speed, có thể lắc yaw nhẹ để tránh đơn điệu
        target_speed = phase.params.get("speed", 15.0)   # m/s
        target_alt = phase.params.get("alt", -uav.p[2])
        yaw_dither = phase.params.get("yaw_dither", 0.0)
        dither_freq = phase.params.get("dither_freq", 0.5)

        thrust_x = speed_control_thrust_x(target_speed)
        thrust_z = altitude_hold_thrust_z(target_alt)
        uav.thrust_body = np.array([thrust_x, 0.0, thrust_z])

        yaw_rate_cmd = yaw_dither * np.sin(2.0 * np.pi * dither_freq * elapsed)
        omega_des = np.array([0.0, 0.0, yaw_rate_cmd])
        uav.torque_body = 1.8 * (omega_des - uav.omega)

    elif kind == "turn":
        # Cua có giữ tốc độ tiến + giữ độ cao
        yaw_rate = phase.params.get("yaw_rate", 0.3)
        direction = phase.params.get("direction", 1)
        target_speed = phase.params.get("speed", 14.0)
        target_alt = phase.params.get("alt", -uav.p[2])

        thrust_x = speed_control_thrust_x(target_speed)
        thrust_z = altitude_hold_thrust_z(target_alt)
        uav.thrust_body = np.array([thrust_x, 0.0, thrust_z])

        omega_des = np.array([0.0, 0.0, yaw_rate * direction])
        uav.torque_body = 2.2 * (omega_des - uav.omega)

    elif kind == "climb":
        # Tăng/giảm độ cao nhưng vẫn có tiến tới để quỹ đạo đẹp hơn
        climb_rate = phase.params.get("climb_rate", 2.0)  # m/s dương = lên
        target_speed = phase.params.get("speed", 12.0)
        yaw_rate = phase.params.get("yaw_rate", 0.0)

        thrust_x = speed_control_thrust_x(target_speed)
        thrust_z = climb_rate_thrust_z(climb_rate)
        uav.thrust_body = np.array([thrust_x, 0.0, thrust_z])

        omega_des = np.array([0.0, 0.0, yaw_rate])
        uav.torque_body = 1.8 * (omega_des - uav.omega)

    elif kind == "pitch_maneuver":
        # Pitch nhẹ + speed burst ngắn
        pitch_rate = phase.params.get("pitch_rate", 0.2)  # rad/s
        target_speed = phase.params.get("speed", 16.0)
        target_alt = phase.params.get("alt", -uav.p[2])

        thrust_x = speed_control_thrust_x(target_speed)
        thrust_z = altitude_hold_thrust_z(target_alt)
        uav.thrust_body = np.array([thrust_x, 0.0, thrust_z])

        omega_des = np.array([0.0, pitch_rate, 0.0])
        uav.torque_body = 2.0 * (omega_des - uav.omega)

    elif kind == "aggressive_yaw":
        # Yaw nhanh nhưng vẫn có tiến động để tránh quay tại chỗ
        yaw_rate = phase.params.get("yaw_rate", 1.0)
        target_speed = phase.params.get("speed", 10.0)
        target_alt = phase.params.get("alt", -uav.p[2])

        thrust_x = speed_control_thrust_x(target_speed)
        thrust_z = altitude_hold_thrust_z(target_alt)
        uav.thrust_body = np.array([thrust_x, 0.0, thrust_z])

        omega_des = np.array([0.0, 0.0, yaw_rate])
        uav.torque_body = 3.0 * (omega_des - uav.omega)

    elif kind == "wind_gust":
        # Kích hoạt gust trong atmosphere
        gust_ned = np.array(phase.params.get("gust_ned", [0, 0, 0]), dtype=float)
        decay = phase.params.get("decay_rate", 0.5)

        # Chỉ kích hoạt gust 1 lần ở đầu phase
        if elapsed < cfg.dt * 2:
            atmo.set_gust(gust_ned, decay)

        # Giữ cruise để thấy hiệu ứng lệch quỹ đạo do gió
        target_speed = phase.params.get("speed", 13.0)
        target_alt = phase.params.get("alt", -uav.p[2])

        thrust_x = speed_control_thrust_x(target_speed)
        thrust_z = altitude_hold_thrust_z(target_alt)
        uav.thrust_body = np.array([thrust_x, 0.0, thrust_z])
        uav.torque_body = -1.6 * uav.omega

    elif kind == "gps_dropout":
        # GPS dropout: bay bình thường nhưng GPS mất tín hiệu
        dropout_dur = phase.params.get("dropout_duration", 5.0)
        if elapsed < dropout_dur:
            gps_active = False

        target_speed = phase.params.get("speed", 12.0)
        target_alt = phase.params.get("alt", -uav.p[2])
        thrust_x = speed_control_thrust_x(target_speed)
        thrust_z = altitude_hold_thrust_z(target_alt)
        uav.thrust_body = np.array([thrust_x, 0.0, thrust_z])
        uav.torque_body = -1.8 * uav.omega

    else:
        # Mặc định: hover
        thrust_z = clamp(-m * g)
        uav.thrust_body = np.array([0.0, 0.0, thrust_z])
        uav.torque_body = -2.0 * uav.omega

    return gps_active
