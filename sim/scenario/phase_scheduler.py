"""sim.scenario.phase_scheduler — Lên lịch & thực thi chuỗi phase bay.

PhaseScheduler:
1) Chia tổng thời gian mô phỏng → nhiều time window (3–8 s mỗi window).
2) Mỗi window chọn ngẫu nhiên 1 hành vi từ tập hợp realistic.
3) Tham số ngẫu nhiên nhưng lấy từ "menu" rời rạc → tránh phi vật lý.
4) Hỗ trợ seed để reproducible.

Tối thiểu 7 loại hành vi (xem phases.py).
"""
import numpy as np
from typing import List

from sim.config import SimConfig
from sim.dynamics.rigid_body_6dof import UAV6DOF
from sim.dynamics.atmosphere import Atmosphere
from .phases import Phase, apply_phase


class PhaseScheduler:
    """Quản lý chuỗi phase bay ngẫu nhiên nhưng thực tế."""

    def __init__(self, cfg: SimConfig, rng: np.random.Generator):
        """
        Args:
            cfg: Cấu hình mô phỏng.
            rng: Generator ngẫu nhiên (seeded).
        """
        self.cfg = cfg
        self.rng = rng
        self.phases: List[Phase] = []

        # State nội bộ
        self._phase_idx: int = 0
        self._phase_elapsed: float = 0.0
        self._gps_active: bool = True
        self._last_kind: str = ""

        # Sinh mission
        self._generate_mission()

    # ------------------------------------------------------------------
    # Sinh kịch bản bay
    # ------------------------------------------------------------------

    def _generate_mission(self) -> None:
        """Tạo chuỗi phase cho toàn bộ thời gian mô phỏng."""
        r = self.rng

        # Phase 0: warmup ngắn + có chuyển động nhẹ để người dùng thấy quỹ đạo ngay
        warmup_alt = float(r.uniform(40, 70))
        self.phases.append(Phase("straight", 2.5, {
            "speed": 8.0, "alt": warmup_alt,
        }))

        remaining = self.cfg.total_time - 2.5

        # Đảm bảo tính đa dạng sớm: luôn có turn + climb nếu đủ thời gian
        if remaining > 9.0:
            forced_seq = ["turn", "climb", "straight"]
            for kind in forced_seq:
                if remaining <= 3.0:
                    break
                dur = float(r.uniform(2.8, 4.2))
                dur = min(dur, remaining)
                params = self._random_params(kind)
                params.setdefault("alt", warmup_alt)
                self.phases.append(Phase(kind, dur, params))
                self._last_kind = kind
                remaining -= dur

        # Tập các loại hành vi có thể chọn (trọng số để cân bằng)
        phase_pool = [
            ("straight",       3),   # bay thẳng — hay gặp
            ("turn",           3),   # cua
            ("climb",          2),   # lên/xuống
            ("pitch_maneuver", 2),   # pitch nhẹ
            ("aggressive_yaw", 1),   # yaw mạnh — ít gặp hơn
            ("wind_gust",      2),   # gió nhiễu
            ("gps_dropout",    1),   # mất GPS
        ]
        kinds = [p[0] for p in phase_pool]
        weights = np.array([p[1] for p in phase_pool], dtype=float)
        weights /= weights.sum()

        while remaining > 2.5:
            # Chọn loại phase nhưng hạn chế lặp lại 1 loại quá nhiều
            kind = str(r.choice(kinds, p=weights))
            if kind == self._last_kind and r.random() < 0.6:
                alt_kinds = [k for k in kinds if k != self._last_kind]
                kind = str(r.choice(alt_kinds))

            # Duration: đa dạng hơn, có cả phase ngắn và trung bình
            dur = float(r.uniform(2.5, 7.0))
            dur = min(dur, remaining)

            # Tham số từ "menu" rời rạc
            params = self._random_params(kind)
            self.phases.append(Phase(kind, dur, params))
            self._last_kind = kind
            remaining -= dur

        # Phase cuối: hover trở về ổn định
        if remaining > 0.5:
            self.phases.append(Phase("straight", remaining, {"speed": 6.0}))

    def _random_params(self, kind: str) -> dict:
        """Sinh tham số ngẫu nhiên cho từng loại phase.

        Dùng menu rời rạc (discrete choices) để tránh giá trị phi thực tế.
        """
        r = self.rng

        if kind == "straight":
            return {
                "speed": float(r.choice([10.0, 15.0, 20.0])),   # m/s
                "yaw_dither": float(r.choice([0.0, 0.04, 0.08])),
                "dither_freq": float(r.choice([0.25, 0.4, 0.6])),
            }

        elif kind == "turn":
            return {
                "yaw_rate":  float(r.choice([0.1, 0.3, 0.6])),  # rad/s
                "direction": int(r.choice([-1, 1])),
                "speed": float(r.choice([12.0, 15.0, 18.0])),
            }

        elif kind == "climb":
            return {
                "climb_rate": float(r.choice([1.0, 3.0, -1.0, -3.0])),  # m/s
                "speed": float(r.choice([10.0, 14.0, 17.0])),
                "yaw_rate": float(r.choice([0.0, 0.08, -0.08])),
            }

        elif kind == "pitch_maneuver":
            return {
                "pitch_rate": float(r.choice([-0.3, -0.15, 0.15, 0.3])),  # rad/s
                "speed": float(r.choice([14.0, 18.0, 22.0])),
            }

        elif kind == "aggressive_yaw":
            return {
                "yaw_rate": float(r.choice([0.8, 1.2, -0.8, -1.2])),  # rad/s
                "speed": float(r.choice([9.0, 12.0])),
            }

        elif kind == "wind_gust":
            # Gust NED: biên độ hợp lý (< 8 m/s)
            gust = r.uniform(-6.0, 6.0, size=3).tolist()
            return {
                "gust_ned": gust,
                "decay_rate": float(r.choice([0.3, 0.5, 0.8])),
                "speed": float(r.choice([11.0, 14.0, 17.0])),
            }

        elif kind == "gps_dropout":
            return {
                "dropout_duration": float(r.choice([3.0, 5.0, 8.0])),
                "speed": float(r.choice([10.0, 14.0, 18.0])),
            }

        return {}

    # ------------------------------------------------------------------
    # Apply mỗi tick
    # ------------------------------------------------------------------

    def apply(self, uav: UAV6DOF, atmo: Atmosphere) -> bool:
        """Áp dụng phase hiện tại lên UAV. Gọi mỗi tick (dt).

        Args:
            uav:  UAV model.
            atmo: Atmosphere model.

        Returns:
            gps_active: True nếu GPS đang hoạt động.
        """
        if self._phase_idx >= len(self.phases):
            # Hết mission → hover
            uav.thrust_body = np.array([0.0, 0.0, -self.cfg.mass * self.cfg.gravity])
            uav.torque_body = -2.0 * uav.omega
            return True

        phase = self.phases[self._phase_idx]
        self._gps_active = apply_phase(
            phase, uav, atmo, self._phase_elapsed, self.cfg
        )

        self._phase_elapsed += self.cfg.dt
        if self._phase_elapsed >= phase.duration:
            self._phase_elapsed = 0.0
            self._phase_idx += 1
            self._gps_active = True  # reset GPS khi chuyển phase

        return self._gps_active

    @property
    def current_phase_name(self) -> str:
        """Tên phase đang thực thi."""
        if self._phase_idx < len(self.phases):
            return self.phases[self._phase_idx].kind
        return "idle"

    @property
    def progress(self) -> float:
        """Tiến trình 0→1 qua mission."""
        total_dur = sum(p.duration for p in self.phases)
        elapsed = sum(p.duration for p in self.phases[:self._phase_idx]) + self._phase_elapsed
        return min(elapsed / max(total_dur, 1e-6), 1.0)
