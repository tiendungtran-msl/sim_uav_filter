# ESKF Core C++ — Error-State Kalman Filter 15D

Implementation C++ của ESKF 15 chiều cho UAV navigation,
thiết kế theo phong cách PX4/EKF2, portable cho embedded (NuttX/RTOS).

## Tổng quan

**Error-state vector (15D):**
```
δx = [δp(3), δv(3), δθ(3), δbg(3), δba(3)]
```

**Nominal state (16D):**
```
x = [p(3), v(3), q(4), bg(3), ba(3)]
```

**Sensor fusion:**
- IMU propagation (400 Hz) — bias-compensated, F/Q/P propagation
- GPS position + velocity (10 Hz, 6D) — delay compensation via rollback/replay
- Magnetometer (50 Hz, 3D) — full vector observation
- Barometer (50 Hz, 1D) — altitude update

## Build

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### Build targets

| Target | Mô tả |
|--------|-------|
| `eskf_core` | Thư viện tĩnh (static lib) chứa ESKF logic |
| `eskf_replay` | Tool offline replay: đọc sensor CSV → chạy ESKF → xuất estimated + innovations CSV |
| `eskf_bench` | Tool benchmark: đo timing per-step |

## Sử dụng

### Replay tool

```bash
./eskf_replay <sensor_log.csv> [output_dir]
# Mặc định output: estimated.csv, innovations.csv cùng thư mục với input

# Ví dụ:
./eskf_replay ../../../data/examples/sensor_log.csv
```

Output:
- `estimated.csv` — kết quả ước lượng + truth (38 cột)
- `innovations.csv` — innovation records cho GPS/MAG/Baro

### Benchmark tool

```bash
./eskf_bench <sensor_log.csv>
```

### Tích hợp vào project C++ khác

```cpp
#include <eskf/eskf.hpp>

eskf::ESKF15 filter;
filter.init(0.0);

// Mỗi tick IMU:
eskf::ImuSample imu;
imu.t = current_time;
imu.gyro = ...; imu.accel = ...; imu.dt = 0.0025;
filter.predict(imu);

// Khi có GPS:
eskf::GpsSample gps;
gps.t_stamp = measurement_time;
gps.t_receive = current_time;
gps.pos = ...; gps.vel = ...;
auto innov = filter.update_gps(gps);

// Lấy kết quả:
const auto& est = filter.state();
printf("pos: %.3f %.3f %.3f\n", est.p[0], est.p[1], est.p[2]);
```

## Cấu trúc file

```
include/eskf/
├── config.hpp           # Compile-time constants, sensor params, P0, gating
├── static_matrix.hpp    # Mat<R,C> template trên stack, invert3/6/1
├── math.hpp             # Quaternion ops, rotation, skew, Euler, cross/dot
├── eskf_types.hpp       # NominalState, ImuSample, StateSnapshot, GpsSample, ...
├── ring_buffer.hpp      # RingBuffer<T,CAP> — tìm gần nhất theo timestamp
├── eskf.hpp             # ESKF15 class — API chính
├── sensors.hpp          # CSV reader (chỉ dùng trong tools)
└── innovations.hpp      # CSV writer (chỉ dùng trong tools)

src/
├── eskf.cpp             # Init, predict wrapper, inject_error_state, snapshot
├── imu_propagation.cpp  # Nominal update + F(15×15) + Q + P propagation
├── gps_update.cpp       # 6D update, rollback/update/re-propagate, Joseph form
├── mag_update.cpp       # 3D update, h(x) = R^T * mag_earth, Joseph form
├── baro_update.cpp      # 1D altitude update, Joseph form
└── numerics.cpp         # Placeholder cho future numerical utilities

tools/
├── replay_main.cpp      # Đọc sensor_log.csv → chạy ESKF → xuất results
└── bench_main.cpp       # Đo timing predict/GPS/MAG/Baro per step

tests/
├── test_math.cpp        # Unit tests cho quaternion, rotation, skew
├── test_buffer.cpp      # Unit tests cho RingBuffer
└── test_consistency.cpp # ESKF consistency: P symmetry, NIS bounds
```

## Quy ước

- **Frame**: NED (North-East-Down)
- **Quaternion**: Hamilton, scalar-first `[w, x, y, z]`, body → NED
- **Covariance update**: Joseph form `P = (I-KH)P(I-KH)^T + KRK^T`
- **Gating**: Chi-square test trên NIS trước khi apply update
- **Memory**: Tất cả trên stack, không `new`/`delete`
- **Comment**: Tiếng Việt

## Kết quả

Với sensor_log 30s (UAV bay hover → climb → cruise → turn):

| Metric | Value |
|--------|-------|
| RMSE Position | 2.998 m |
| RMSE Velocity | 0.120 m/s |
| RMSE Attitude | 1.31° |
| GPS accepted | 299/299 (100%) |
| Predict timing | 3.19 µs (mean) |
| GPS update timing | 251.93 µs (mean, incl. replay) |
| Throughput | ~71,000 steps/s |
