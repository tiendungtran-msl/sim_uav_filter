# sim/gui/ — GUI Dashboard

## Layout (1 cửa sổ, 4 vùng)

```
┌──────────────────┬──────────────────┐
│  Attitude (3D)   │  Data Table      │
│  widgets_attitude│  widgets_table   │
├──────────────────┴──────────────────┤
│  Trajectory (N-E + Alt)             │
│  widgets_plots.TrajectoryWidget     │
├─────────────────────────────────────┤
│  Timeseries (Gyro/Accel/GPS/Baro)  │
│  widgets_plots.TimeseriesWidget     │
└─────────────────────────────────────┘
```

## Widgets

### 1. AttitudeWidget (`widgets_attitude.py`)
- OpenGL 3D view hiển thị trục body (X đỏ, Y xanh lá, Z xanh dương).
- UAV "cố định vị trí" nhưng xoay theo quaternion truth realtime.
- Header hiển thị Euler angles (roll, pitch, yaw).

### 2. DataTableWidget (`widgets_table.py`)
- Bảng 28 hàng × 3 cột (Label | Truth | Meas).
- Gồm: pos, vel, attitude, omega, specific force, bias, GPS, baro.
- Highlight đỏ khi GPS dropout; tím nhạt cho bias (ẩn với filter).

### 3. TrajectoryWidget (`widgets_plots.py`)
- **N-E plot**: quỹ đạo top-down. Truth = đường xanh, GPS = chấm đỏ.
- **Altitude plot**: altitude truth + baro measurement theo thời gian.

### 4. TimeseriesWidget (`widgets_plots.py`)
- 4 đồ thị xếp dọc: Gyro, Accel, GPS pos error, Baro alt.
- Mỗi đồ thị có 3 trục (X/Y/Z). Truth = nét liền, Meas = nét chấm nhạt.
- Dùng RingBuffer → giới hạn ~15s gần nhất → mượt.

## Hiệu năng
- GUI cập nhật mỗ 50ms (~20 FPS).
- Mỗi frame chạy batch ~20 bước IMU (0.05s / 0.0025s).
- Dữ liệu push vào RingBuffer (capacity 400-3000 điểm) → không ngốn RAM.
- PyQtGraph + OpenGL → nhẹ, nhanh.
