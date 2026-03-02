"""sim.utils.ring_buffer — Ring buffer cho dữ liệu time-series & sensor delay.

Hai lớp chính:
- RingBuffer:          buffer kích thước cố định, dùng cho plot realtime.
- DelayBuffer:         hàng đợi FIFO dựa trên thời gian, dùng mô phỏng delay sensor.
"""
import numpy as np
from numpy.typing import NDArray
from collections import deque
from typing import Optional, Any


class RingBuffer:
    """Buffer vòng với kích thước cố định, tối ưu cho plotting realtime.

    Mỗi phần tử là 1 vector (width-dim). Khi đầy, phần tử cũ nhất bị ghi đè.
    Dùng numpy nội bộ để truy xuất nhanh toàn bộ dữ liệu.
    """

    def __init__(self, capacity: int, width: int = 1):
        """
        Args:
            capacity: Số phần tử tối đa trong buffer.
            width:    Kích thước mỗi phần tử (ví dụ 3 cho vector 3D).
        """
        self.capacity = capacity
        self.width = width
        self._buf = np.zeros((capacity, width), dtype=np.float64)
        self._idx = 0          # vị trí ghi tiếp theo
        self._count = 0        # số phần tử đã ghi (max = capacity)

    def push(self, value: NDArray) -> None:
        """Thêm 1 phần tử vào buffer."""
        self._buf[self._idx] = value
        self._idx = (self._idx + 1) % self.capacity
        if self._count < self.capacity:
            self._count += 1

    def get_array(self) -> NDArray:
        """Trả toàn bộ dữ liệu đã ghi, theo thứ tự cũ → mới (shape: [count, width])."""
        if self._count < self.capacity:
            return self._buf[:self._count].copy()
        # Buffer đã đầy: xoay vòng
        return np.roll(self._buf, -self._idx, axis=0).copy()

    def __len__(self) -> int:
        return self._count

    @property
    def is_full(self) -> bool:
        return self._count >= self.capacity


class DelayBuffer:
    """Hàng đợi FIFO dựa trên timestamp, dùng để mô phỏng delay cố định của sensor.

    Cách dùng:
        buf = DelayBuffer(delay=0.2)
        buf.push(t_sample, data)     # khi sensor lấy mẫu
        result = buf.pop(t_now)      # lấy mẫu nào đã "đến" (t_now >= t_sample + delay)
    """

    def __init__(self, delay: float):
        """
        Args:
            delay: Thời gian trễ cố định (giây).
        """
        self.delay = delay
        self._queue: deque = deque()

    def push(self, t_stamp: float, data: Any) -> None:
        """Đẩy 1 sample vào hàng đợi."""
        self._queue.append((t_stamp, data))

    def pop(self, t_now: float) -> Optional[Any]:
        """Lấy sample cũ nhất đã quá delay. Trả None nếu chưa có sample nào sẵn sàng."""
        if self._queue:
            t_stamp, data = self._queue[0]
            if t_now >= t_stamp + self.delay:
                self._queue.popleft()
                return data
        return None

    def clear(self) -> None:
        """Xoá toàn bộ buffer."""
        self._queue.clear()

    def __len__(self) -> int:
        return len(self._queue)
