# filters/ — Bộ lọc ESKF (C++ Core)

Thư mục chứa implementation C++ của Error-State Kalman Filter.

## Cấu trúc

```
filters/
├── README.md           ← File này
├── __init__.py         ← Python package marker
└── eskf_core_cpp/      ← C++ ESKF implementation
    ├── CMakeLists.txt
    ├── README.md       ← Hướng dẫn build & sử dụng chi tiết
    ├── include/eskf/   ← Header files (API)
    ├── src/            ← Implementation
    ├── tools/          ← Replay & benchmark executables
    ├── tests/          ← Unit tests
    └── cmake/          ← CMake modules
```

## Quy tắc kiến trúc

- `filters/` **KHÔNG ĐƯỢC import** bất kỳ module nào từ `sim/` hay `gui/`
- `sim/` và `gui/` **KHÔNG ĐƯỢC import** bất kỳ module nào từ `filters/`
- Giao tiếp qua file CSV (sensor_log.csv → ESKF → estimated.csv + innovations.csv)

## Thiết kế

- **Embedded-portable**: Không dùng heap, exceptions, STL containers
- **C++17**: Chỉ dùng `constexpr`, static allocation
- **Joseph form**: Covariance update `P = (I-KH)P(I-KH)^T + KRK^T` — numerically stable
- **Chi-square gating**: Reject outlier measurements
- **Delay compensation**: PX4-style rollback/update/re-propagate cho GPS delay

## Benchmark

| Metric | Value |
|--------|-------|
| Predict (IMU) | ~3 µs/step |
| GPS update (6D + delay comp) | ~250 µs |
| Throughput | ~71,000 steps/s |
| RMSE Position | ~3.0 m |
| RMSE Velocity | ~0.12 m/s |
| RMSE Attitude | ~1.3° |
