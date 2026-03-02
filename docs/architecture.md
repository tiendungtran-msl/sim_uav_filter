# Kiến trúc tổng quan — ESKF System

## Pipeline dữ liệu

```
┌─────────────┐       sensor_log.csv       ┌──────────────┐
│  Simulator   │ ─────────────────────────▶ │  ESKF C++    │
│  (Python)    │   47 cột, 400 Hz          │  Replay Tool │
└─────────────┘                             └──────┬───────┘
                                                   │
                                          estimated.csv + innovations.csv
                                                   │
                                            ┌──────▼───────┐
                                            │  Python GUI   │
                                            │  (PyQt6)      │
                                            └──────────────┘
```

## Nguyên tắc thiết kế

1. **Tách biệt hoàn toàn**: ESKF C++ không import GUI, GUI không chứa filter logic
2. **File-based interface**: Giao tiếp qua CSV — dễ debug, reproducible
3. **Embedded-portable**: C++ core không dùng heap, STL containers, exceptions
4. **Offline-first**: GUI đọc kết quả đã chạy, không chạy filter real-time

## Thành phần

### 1. Simulator (`sim/`)

Python simulator tạo dữ liệu UAV bay theo kịch bản:
- **Dynamics**: Rigid body 6-DOF, quaternion integration
- **Sensors**: IMU (400Hz), GPS (10Hz + delay), Baro (50Hz), Mag (50Hz)
- **Scenario**: Phase scheduler (hover → climb → cruise → turn → descend)
- **Output**: `sensor_log.csv` (47 cột)

### 2. ESKF C++ Core (`filters/eskf_core_cpp/`)

Error-State Kalman Filter 15 chiều, thiết kế embedded:

```
include/eskf/
├── config.hpp          # Compile-time constants
├── static_matrix.hpp   # Stack-only Mat<R,C> template
├── math.hpp            # Quaternion, rotation, skew
├── eskf_types.hpp      # NominalState, ImuSample, Snapshot, ...
├── ring_buffer.hpp     # RingBuffer<T, CAP> template
├── eskf.hpp            # ESKF15 class API
├── sensors.hpp         # CSV reader (tools only)
└── innovations.hpp     # CSV writer (tools only)

src/
├── eskf.cpp            # Init, predict wrapper, inject, snapshot
├── imu_propagation.cpp # F matrix, Q matrix, P propagation
├── gps_update.cpp      # 6D GPS + delay compensation
├── mag_update.cpp      # 3D magnetometer
├── baro_update.cpp     # 1D barometer
└── numerics.cpp        # (placeholder)

tools/
├── replay_main.cpp     # Offline replay: CSV in → ESKF → CSV out
└── bench_main.cpp      # Timing benchmark
```

#### Error-State Vector (15D)

```
δx = [δp(3), δv(3), δθ(3), δbg(3), δba(3)]
       ├──────┤ ├──────┤ ├──────┤ ├───────┤ ├───────┤
       pos err  vel err  att err  gyro bias accel bias
                                  error     error
```

#### Delay Compensation (PX4-style)

```
Time ─────────────────────────────────▶
     │    snapshot      GPS t_stamp    GPS t_receive = now
     │       ▼              ▼              ▼
IMU: ═══════╬══════════════╬══════════════╬
     │       │              │              │
     │  1) Find nearest     │              │
     │     snapshot         │              │
     │       │              │              │
     │  2) Rollback state/P │              │
     │     to snapshot      │              │
     │       │              │              │
     │  3) Re-propagate IMU │              │
     │     to t_stamp       │              │
     │       ├──▶ 4) GPS update            │
     │       │              │              │
     │  5) Re-propagate IMU │              │
     │     from t_stamp ──────────────────▶│ now
```

### 3. Python GUI (`gui/`)

Offline evaluation viewer (PyQt6 + pyqtgraph):

| Tab | Nội dung |
|-----|----------|
| Vị trí | Position NED: truth vs estimated vs GPS measurement + RMSE |
| Vận tốc | Velocity NED: truth vs estimated |
| Tư thế Euler | Roll, Pitch, Yaw (degrees) |
| Quaternion | q components + ‖q‖ norm check |
| Bias | Gyro bias + Accel bias convergence (6 plots) |
| Innovations | GPS/MAG/Baro innovations + NIS with χ² bounds |

## Build & Run

```bash
# 1) Generate sensor data
cd sim_ekf2
python3 -m sim.cli --headless --time 30 --seed 42 --out data/examples/sensor_log.csv

# 2) Build ESKF
cd filters/eskf_core_cpp
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# 3) Run replay
./eskf_replay ../../../data/examples/sensor_log.csv

# 4) View results
cd ../../..
python -m gui.app
```
