# sim/scenario/ — Phase Scheduler & Kịch bản bay

## Cách hoạt động
`PhaseScheduler` chia tổng thời gian mô phỏng thành nhiều **time window** (3–8s).
Mỗi window chọn ngẫu nhiên 1 hành vi từ 7 loại, với tham số từ menu rời rạc.

## 7 loại Phase

| # | Phase            | Tham số (menu rời rạc)                           | Ghi chú                        |
|---|------------------|--------------------------------------------------|---------------------------------|
| 1 | `straight`       | speed ∈ {10, 15, 20} m/s                         | Bay thẳng, giữ alt             |
| 2 | `turn`           | yaw_rate ∈ {0.1, 0.3, 0.6} rad/s; dir ∈ {±1}   | Cua trái/phải                  |
| 3 | `climb`          | climb_rate ∈ {+1, +3, −1, −3} m/s               | Tăng/giảm độ cao               |
| 4 | `pitch_maneuver` | pitch_rate ∈ {±0.15, ±0.3} rad/s                | Pitch nhẹ → gia tốc dọc       |
| 5 | `aggressive_yaw` | yaw_rate ∈ {±0.8, ±1.2} rad/s                   | Yaw mạnh, thời gian ngắn      |
| 6 | `wind_gust`      | gust NED ∈ [−6, +6]³ m/s; decay ∈ {0.3,0.5,0.8} | Gió nhiễu đột ngột           |
| 7 | `gps_dropout`    | dropout_duration ∈ {3, 5, 8} s                   | GPS mất tín hiệu              |

## Trọng số chọn Phase
```
straight:       3  (hay gặp)
turn:           3
climb:          2
pitch_maneuver: 2
aggressive_yaw: 1  (hiếm hơn)
wind_gust:      2
gps_dropout:    1  (hiếm)
```

## Mission structure
1. **Warmup** (5s): hover ở độ cao random 40–70m.
2. **Main**: chuỗi phases ngẫu nhiên, mỗi phase 3–8s.
3. **Cooldown**: hover cuối nếu còn thời gian.

## Reproducibility
Dùng `np.random.default_rng(seed)` → cùng seed = cùng kịch bản bay chính xác.

## Giới hạn vật lý
- Thrust tối đa: 4× trọng lượng.
- Omega damping (torque controller đơn giản) giữ vận tốc góc hợp lý.
- Tham số rời rạc → không phát sinh giá trị phi thực tế.
