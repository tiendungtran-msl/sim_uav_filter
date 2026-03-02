# sim/sensors/ — Mô hình cảm biến

## Tổng quan
Mô phỏng 4 loại sensor realistic (MEMS consumer-grade):

| Sensor | Rate    | Noise model                              |
|--------|---------|------------------------------------------|
| IMU    | 400 Hz  | Bias random walk + white Gaussian noise  |
| GPS    | 10 Hz   | Position/velocity noise + delay 200ms    |
| Baro   | 50 Hz   | White noise + bias drift (random walk)   |
| Mag    | 50 Hz   | Vector noise + bias random walk          |

## Thông số mặc định (SensorConfig)

### Gyroscope
- **ARW** (Angular Random Walk): 0.005 rad/s/√Hz
- **Bias Random Walk**: 0.0002 rad/s²/√Hz
- **Bias ban đầu** (1σ): 0.01 rad/s

### Accelerometer
- **VRW** (Velocity Random Walk): 0.1 m/s²/√Hz
- **Bias Random Walk**: 0.005 m/s²/√Hz
- **Bias ban đầu** (1σ): 0.2 m/s²

### GPS
- **Position noise** (1σ): 1.5 m per-axis
- **Velocity noise** (1σ): 0.15 m/s per-axis
- **Delay**: 200 ms cố định (dùng DelayBuffer)
- **Dropout**: hỗ trợ (scenario điều khiển)

### Barometer
- **Altitude noise** (1σ): 0.5 m
- **Bias drift rate**: 0.1 m/s (random walk)

### Magnetometer
- **Rate**: 50 Hz
- **Noise** (1σ): 0.8 uT
- **Bias init** (1σ): 2.0 uT
- **Bias random walk**: 0.03 uT/sqrt(s)

## Mô hình đo

### IMU
```
gyro_meas  = ω_true + b_g(t) + w_g     (w_g ~ N(0, σ_g²))
accel_meas = f_true + b_a(t) + w_a     (w_a ~ N(0, σ_a²))

b_g(t+dt) = b_g(t) + N(0, σ_bg_rw²·dt)
b_a(t+dt) = b_a(t) + N(0, σ_ba_rw²·dt)
```
- `f_true` = specific force body (= accel − gravity, cái IMU thật sự cảm nhận)
- `ω_true` = vận tốc góc body

### GPS
```
pos_gps = pos_true + N(0, σ_gps_p²)
vel_gps = vel_true + N(0, σ_gps_v²)
```
- Mẫu đẩy vào DelayBuffer, lấy ra sau 200ms.
- Khi `gps_active=False` → không lấy mẫu (dropout).

### Barometer
```
alt_baro = alt_true + bias(t) + N(0, σ_baro²)
bias(t+dt) = bias(t) + N(0, σ_drift²·dt)
```

### Magnetometer
```
mag_meas = mag_true_body + b_m + N(0, σ_mag²)
b_m(t+dt) = b_m(t) + N(0, σ_mrw²·dt)
```
- `mag_true_body` lấy từ vector từ trường chuẩn NED quay về body frame.
