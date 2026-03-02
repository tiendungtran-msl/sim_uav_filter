# sim/ — UAV Simulator Module (độc lập với ESKF)

## Mô tả
Module mô phỏng UAV 6DOF hoàn chỉnh, **không phụ thuộc** bất kỳ thuật toán lọc nào.
Xuất dữ liệu truth + measurement chuẩn để sau này feed vào ESKF hoặc filter khác.

## Cách chạy

### GUI mode (realtime)
```bash
python -m sim.app --seed 42 --time 60
```
Mở cửa sổ PyQt6 với 4 vùng:
1. **Attitude view** — trục body xoay theo quaternion truth
2. **Data table** — bảng truth vs measurement realtime
3. **Trajectory** — quỹ đạo N-E + altitude
4. **Timeseries** —
    - Tab `Truth States`: attitude, position, velocity
    - Tab `Sensor Streams`: gyro, accel, GPS, baro, magnetometer

### Headless mode (xuất log nhanh)
```bash
python -m sim.cli --headless --time 60 --seed 1 --out logs/run1.csv
```
Chạy nhanh nhất có thể, xuất file CSV duy nhất chứa toàn bộ truth + measurement.

## Cấu trúc thư mục
```
sim/
├── app.py                  # Entry point GUI
├── cli.py                  # Entry point headless/log
├── config.py               # Cấu hình (SimConfig, SensorConfig)
├── timebase.py             # Quản lý thời gian mô phỏng
├── dynamics/
│   ├── rigid_body_6dof.py  # Động lực học 6DOF (quaternion, RK4)
│   └── atmosphere.py       # Gió / gust / turbulence
├── scenario/
│   ├── phases.py           # Định nghĩa 7 loại phase bay
│   └── phase_scheduler.py  # Lập lịch phase ngẫu nhiên
├── sensors/
│   ├── imu.py              # IMU 400 Hz (bias RW + white noise)
│   ├── gps.py              # GPS 10 Hz (noise + delay 200ms + dropout)
│   ├── baro.py             # Baro 50 Hz (noise + bias drift)
│   └── models.py           # Dataclasses nội bộ sensor
├── io/
│   ├── schema.py           # Dataclasses: TruthState, ImuMeas, GpsMeas, BaroMeas, SimFrame
│   └── logger_csv.py       # Ghi log CSV
├── gui/
│   ├── dashboard.py        # GUI layout + sim loop
│   ├── widgets_attitude.py # 3D attitude view
│   ├── widgets_plots.py    # Trajectory + timeseries plots
│   └── widgets_table.py    # Bảng truth vs meas
└── utils/
    ├── math3d.py           # Quaternion, rotation matrix, skew
    ├── ring_buffer.py      # RingBuffer + DelayBuffer
    └── units.py            # Hằng số vật lý & chuyển đổi
```

## Dependencies
- numpy
- PyQt6
- pyqtgraph
- PyOpenGL (cho attitude 3D widget)

## Cấu hình
Sửa `sim/config.py`:
- `SimConfig`: dt, total_time, seed, UAV mass/inertia, drag
- `SensorConfig`: noise levels, rates, delays, bias parameters

## Quy ước
- **Frame NED**: North-East-Down. Altitude = −p_D.
- **Quaternion**: [w, x, y, z] Hamilton scalar-first. body → NED.
- **Specific force**: cái IMU thật sự đo (không chứa gravity).
- **Seed**: đặt 1 seed → kết quả reproducible 100%.
