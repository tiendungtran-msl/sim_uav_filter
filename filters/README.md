# filters/ — Placeholder cho thuật toán lọc (ESKF, EKF, ...)

Thư mục này **chưa implement** bất kỳ filter nào.

## Quy tắc kiến trúc
- `filters/` **KHÔNG ĐƯỢC import** bất kỳ module nào từ `sim/`.
- `sim/` **KHÔNG ĐƯỢC import** bất kỳ module nào từ `filters/`.
- Hai module giao tiếp thông qua `SimFrame` (định nghĩa trong `sim/io/schema.py`).

## Cách "cắm" filter vào simulator
1. **Realtime**: subscribe `SimFrame` từ simulator loop.
2. **Offline**: đọc file CSV log (xem `sim/io/logger_csv.py`).

## Ví dụ (TODO khi implement)
```python
from sim.io.schema import SimFrame

class ESKF:
    def process_frame(self, frame: SimFrame):
        # IMU predict
        self.imu_update(frame.imu.accel, frame.imu.gyro)
        # GPS update (nếu có)
        if frame.gps is not None and frame.gps.valid:
            self.gps_update(frame.gps.pos_ned, frame.gps.vel_ned)
        # Baro update (nếu có)
        if frame.baro is not None and frame.baro.valid:
            self.baro_update(frame.baro.alt_m)
```
