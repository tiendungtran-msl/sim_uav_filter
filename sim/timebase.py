"""sim.timebase — Quản lý thời gian mô phỏng & realtime factor.

Lớp Timebase cung cấp:
- Bộ đếm bước (tick) với dt cố định.
- Hỗ trợ realtime factor: 1× = thời gian thực, 0 = chạy nhanh nhất có thể.
- Tính toán sim_time, wall_time, realtime ratio.
"""
import time as _time


class Timebase:
    """Đồng hồ mô phỏng, có khả năng đồng bộ với wall-clock."""

    def __init__(self, dt: float, realtime_factor: float = 0.0):
        """
        Args:
            dt:              Bước thời gian mô phỏng (s). Ví dụ 0.0025 cho 400 Hz.
            realtime_factor: 0 = chạy tối đa tốc độ (headless).
                             1.0 = chạy đúng realtime.
                             2.0 = chạy nhanh gấp đôi, v.v.
        """
        self.dt = dt
        self.realtime_factor = realtime_factor

        self._step = 0              # bước hiện tại
        self._wall_start = 0.0      # wall-clock lúc bắt đầu
        self._running = False

    # ------------------------------------------------------------------
    # Truy vấn thời gian
    # ------------------------------------------------------------------

    @property
    def sim_time(self) -> float:
        """Thời gian mô phỏng hiện tại (s)."""
        return self._step * self.dt

    @property
    def step_count(self) -> int:
        """Bước hiện tại."""
        return self._step

    @property
    def wall_elapsed(self) -> float:
        """Thời gian thực đã trôi qua kể từ start() (s)."""
        if not self._running:
            return 0.0
        return _time.perf_counter() - self._wall_start

    # ------------------------------------------------------------------
    # Điều khiển
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Bắt đầu đồng hồ (reset bước về 0)."""
        self._step = 0
        self._wall_start = _time.perf_counter()
        self._running = True

    def tick(self) -> float:
        """Tiến 1 bước dt. Nếu realtime_factor > 0 thì sleep để giữ nhịp.

        Returns:
            Thời gian mô phỏng sau tick.
        """
        self._step += 1
        sim_t = self.sim_time

        # Đồng bộ realtime nếu cần
        if self.realtime_factor > 0 and self._running:
            target_wall = sim_t / self.realtime_factor
            actual_wall = _time.perf_counter() - self._wall_start
            sleep_s = target_wall - actual_wall
            if sleep_s > 0.0001:          # tránh sleep quá nhỏ (OS overhead)
                _time.sleep(sleep_s)

        return sim_t

    def stop(self) -> None:
        """Dừng đồng hồ."""
        self._running = False
