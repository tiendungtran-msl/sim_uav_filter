# Định dạng CSV — Sensor Log & Output Files

## 1. `sensor_log.csv` — Input từ Simulator (47 cột)

File CSV chứa toàn bộ dữ liệu truth + measurement từ 1 lần chạy simulator.
Tần số: 400 Hz (1 hàng = 1 tick IMU = 2.5 ms).

### Bảng cột

| # | Tên cột | Đơn vị | Mô tả |
|---|---------|--------|-------|
| 0 | `t` | s | Thời gian mô phỏng |
| 1 | `phase` | — | Tên phase kịch bản (hover, climb, cruise, …) |
| 2 | `truth_pN` | m | Vị trí North truth |
| 3 | `truth_pE` | m | Vị trí East truth |
| 4 | `truth_pD` | m | Vị trí Down truth |
| 5 | `truth_vN` | m/s | Vận tốc North truth |
| 6 | `truth_vE` | m/s | Vận tốc East truth |
| 7 | `truth_vD` | m/s | Vận tốc Down truth |
| 8 | `truth_qw` | — | Quaternion w (scalar) truth |
| 9 | `truth_qx` | — | Quaternion x truth |
| 10 | `truth_qy` | — | Quaternion y truth |
| 11 | `truth_qz` | — | Quaternion z truth |
| 12 | `truth_wx` | rad/s | Vận tốc góc body X truth |
| 13 | `truth_wy` | rad/s | Vận tốc góc body Y truth |
| 14 | `truth_wz` | rad/s | Vận tốc góc body Z truth |
| 15 | `truth_ax` | m/s² | Specific force body X truth |
| 16 | `truth_ay` | m/s² | Specific force body Y truth |
| 17 | `truth_az` | m/s² | Specific force body Z truth |
| 18 | `truth_mx` | µT | Từ trường body X truth |
| 19 | `truth_my` | µT | Từ trường body Y truth |
| 20 | `truth_mz` | µT | Từ trường body Z truth |
| 21 | `truth_roll` | ° | Roll truth |
| 22 | `truth_pitch` | ° | Pitch truth |
| 23 | `truth_yaw` | ° | Yaw truth |
| 24 | `truth_bg_x` | rad/s | Gyro bias X truth |
| 25 | `truth_bg_y` | rad/s | Gyro bias Y truth |
| 26 | `truth_bg_z` | rad/s | Gyro bias Z truth |
| 27 | `truth_ba_x` | m/s² | Accel bias X truth |
| 28 | `truth_ba_y` | m/s² | Accel bias Y truth |
| 29 | `truth_ba_z` | m/s² | Accel bias Z truth |
| 30 | `imu_gx` | rad/s | Gyro đo X (= truth_wx + bias + noise) |
| 31 | `imu_gy` | rad/s | Gyro đo Y |
| 32 | `imu_gz` | rad/s | Gyro đo Z |
| 33 | `imu_ax` | m/s² | Accel đo X (= truth_ax + bias + noise) |
| 34 | `imu_ay` | m/s² | Accel đo Y |
| 35 | `imu_az` | m/s² | Accel đo Z |
| 36 | `gps_valid` | 0/1 | GPS có dữ liệu hợp lệ ở tick này không |
| 37 | `gps_t_stamp` | s | Timestamp lúc GPS đo (trước delay) |
| 38 | `gps_pN` | m | GPS position North |
| 39 | `gps_pE` | m | GPS position East |
| 40 | `gps_pD` | m | GPS position Down |
| 41 | `gps_vN` | m/s | GPS velocity North |
| 42 | `gps_vE` | m/s | GPS velocity East |
| 43 | `gps_vD` | m/s | GPS velocity Down |
| 44 | `baro_valid` | 0/1 | Barometer có dữ liệu ở tick này không |
| 45 | `baro_alt` | m | Barometric altitude (dương = lên = −pD) |
| 46 | `mag_valid` | 0/1 | Magnetometer có dữ liệu ở tick này không |
| 47 | `mag_mx` | µT | Mag đo body X |
| 48 | `mag_my` | µT | Mag đo body Y |
| 49 | `mag_mz` | µT | Mag đo body Z |

> **Ghi chú**: Cột `phase` là string, các cột còn lại là số (float/int).
> GPS delay: `gps_t_stamp < t` khi GPS valid. Độ delay mặc định ~200 ms.
> Khi `gps_valid = 0`, các cột gps_* chứa giá trị 0.0 (bỏ qua).
> Tương tự cho `baro_valid = 0` và `mag_valid = 0`.

### Tần số sensor

| Sensor | Rate | Delay |
|--------|------|-------|
| IMU | 400 Hz | 0 ms |
| GPS | 10 Hz | ~200 ms |
| Baro | 50 Hz | 0 ms |
| Mag | 50 Hz | 0 ms |

---

## 2. `estimated.csv` — Output từ ESKF Replay Tool

File CSV chứa kết quả ước lượng của ESKF song song với truth.
Tần số: 400 Hz (1 hàng = 1 bước predict).

### Bảng cột (38 cột)

| # | Tên cột | Đơn vị | Mô tả |
|---|---------|--------|-------|
| 0 | `t` | s | Timestamp |
| 1-3 | `est_pN`, `est_pE`, `est_pD` | m | Vị trí ước lượng NED |
| 4-6 | `est_vN`, `est_vE`, `est_vD` | m/s | Vận tốc ước lượng NED |
| 7-10 | `est_qw`, `est_qx`, `est_qy`, `est_qz` | — | Quaternion ước lượng |
| 11-13 | `est_bgx`, `est_bgy`, `est_bgz` | rad/s | Gyro bias ước lượng |
| 14-16 | `est_bax`, `est_bay`, `est_baz` | m/s² | Accel bias ước lượng |
| 17-19 | `truth_pN`, `truth_pE`, `truth_pD` | m | Vị trí truth |
| 20-22 | `truth_vN`, `truth_vE`, `truth_vD` | m/s | Vận tốc truth |
| 23-26 | `truth_qw`, `truth_qx`, `truth_qy`, `truth_qz` | — | Quaternion truth |
| 27-29 | `truth_bgx`, `truth_bgy`, `truth_bgz` | rad/s | Gyro bias truth |
| 30-32 | `truth_bax`, `truth_bay`, `truth_baz` | m/s² | Accel bias truth |
| 33-35 | `P_pos`, `P_vel`, `P_att` | m², (m/s)², rad² | Trace P cho pos/vel/att |
| 36-37 | `P_gbias`, `P_abias` | varies | Trace P cho bias |

---

## 3. `innovations.csv` — Innovation Records từ ESKF

File CSV chứa innovation (sai lệch measurement - prediction) mỗi lần update.

### Bảng cột (10 cột)

| # | Tên cột | Mô tả |
|---|---------|-------|
| 0 | `t` | Timestamp (s) |
| 1 | `sensor` | Loại sensor: `GPS`, `MAG`, `BARO` |
| 2-7 | `innov_0` … `innov_5` | Innovation vector (tối đa 6D cho GPS) |
| 8 | `nis` | Normalized Innovation Squared = y^T S^{-1} y |
| 9 | `dim` | Số chiều innovation (6=GPS, 3=MAG, 1=BARO) |
| 10 | `accepted` | 1 = passed gating, 0 = rejected |

### NIS Interpretation

NIS phải xấp xỉ phân phối χ² với `dim` bậc tự do:
- **GPS (dim=6)**: E[NIS] ≈ 6, χ²(0.95) ≈ 12.59
- **MAG (dim=3)**: E[NIS] ≈ 3, χ²(0.95) ≈ 7.81
- **BARO (dim=1)**: E[NIS] ≈ 1, χ²(0.95) ≈ 3.84

Nếu NIS vượt ngưỡng chi-square → measurement bị reject (gating).
