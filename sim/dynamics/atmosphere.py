"""sim.dynamics.atmosphere — Mô hình gió & nhiễu khí quyển đơn giản.

Hỗ trợ:
- Gió nền (steady wind) NED.
- Gust (đột ngột) với decay theo thời gian.
- Turbulence dạng Dryden đơn giản hoá (white noise lọc bậc 1).

Atmosphere không tự chạy — PhaseScheduler sẽ gọi set_gust() khi cần.
UAV6DOF đọc wind_ned mỗi bước.
"""
import numpy as np
from numpy.typing import NDArray


class Atmosphere:
    """Mô hình gió đơn giản cho sim UAV."""

    def __init__(self, dt: float, rng: np.random.Generator):
        """
        Args:
            dt:  Bước thời gian mô phỏng (s).
            rng: Generator ngẫu nhiên (để reproducible).
        """
        self.dt = dt
        self.rng = rng

        # Gió nền cố định (NED, m/s) — scenario có thể thay đổi
        self.steady_wind: NDArray = np.zeros(3)

        # Gust hiện tại (NED, m/s) — decay dần
        self._gust: NDArray = np.zeros(3)
        self._gust_decay: float = 0.0   # hệ số decay (1/s), 0 = không gust

        # Turbulence bậc 1 (xấp xỉ Dryden đơn giản)
        self._turb: NDArray = np.zeros(3)
        self._turb_sigma: float = 0.0     # sigma white noise input
        self._turb_tau: float = 2.0       # time constant (s)

    # ------------------------------------------------------------------
    # API cho PhaseScheduler
    # ------------------------------------------------------------------

    def set_steady_wind(self, wind_ned: NDArray) -> None:
        """Đặt gió nền (có thể gọi lại để thay đổi)."""
        self.steady_wind = np.array(wind_ned, dtype=float)

    def set_gust(self, gust_ned: NDArray, decay_rate: float = 0.5) -> None:
        """Kích hoạt 1 đợt gust.

        Args:
            gust_ned:   Vector gust ban đầu (NED, m/s).
            decay_rate: Tốc độ suy giảm (1/s). Ví dụ 0.5 → giảm ~60% sau 2s.
        """
        self._gust = np.array(gust_ned, dtype=float)
        self._gust_decay = decay_rate

    def set_turbulence(self, sigma: float = 0.3, tau: float = 2.0) -> None:
        """Bật turbulence nhẹ (lọc bậc 1 white noise).

        Args:
            sigma: Độ lớn nhiễu (m/s).
            tau:   Time constant bộ lọc (s). Lớn hơn = biến đổi chậm hơn.
        """
        self._turb_sigma = sigma
        self._turb_tau = max(tau, self.dt)  # tránh chia 0

    # ------------------------------------------------------------------
    # Cập nhật mỗi tick
    # ------------------------------------------------------------------

    def step(self) -> NDArray:
        """Tính & trả tổng gió NED sau 1 bước dt.

        Returns:
            wind_total_ned (3,): vector gió tổng hợp.
        """
        dt = self.dt

        # 1) Gust decay
        if self._gust_decay > 0:
            factor = np.exp(-self._gust_decay * dt)
            self._gust *= factor
            # Cắt gust nhỏ để tránh tích luỹ rác số
            if np.linalg.norm(self._gust) < 1e-4:
                self._gust[:] = 0.0
                self._gust_decay = 0.0

        # 2) Turbulence (lọc bậc 1: dx/dt = -(1/τ)x + σ·w)
        if self._turb_sigma > 0:
            alpha = dt / self._turb_tau
            noise = self.rng.normal(0, self._turb_sigma, 3) * np.sqrt(dt)
            self._turb = (1.0 - alpha) * self._turb + noise

        # 3) Tổng hợp
        return self.steady_wind + self._gust + self._turb
