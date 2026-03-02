# gui/ — ESKF Evaluation GUI

GUI offline cho đánh giá kết quả ESKF. Đọc CSV output từ ESKF C++ replay tool,
hiển thị so sánh Truth vs Estimated vs Measured qua 6 tabs.

## Yêu cầu

```bash
pip install PyQt6 pyqtgraph numpy
```

## Chạy

```bash
# Từ thư mục gốc sim_ekf2:
python -m gui.app

# Hoặc chỉ định file:
python -m gui.app \
    --sensor data/examples/sensor_log.csv \
    --est    data/examples/estimated.csv \
    --innov  data/examples/innovations.csv
```

## Tabs

| Tab | Nội dung |
|-----|----------|
| 📍 Vị trí | Position NED (3 subplot), truth vs est vs GPS, RMSE |
| 🚀 Vận tốc | Velocity NED (3 subplot), truth vs est, RMSE |
| 🧭 Tư thế Euler | Roll, Pitch, Yaw (°), yaw wrapping for RMSE |
| 🔄 Quaternion | w, x, y, z, ‖q‖ (5 subplot) |
| 📊 Bias | Gyro bias (3) + Accel bias (3), truth vs est |
| 📈 Innovations | GPS/MAG/Baro inner tabs: innovations + NIS + χ² bounds |

## Cấu trúc file

```
gui/
├── app.py              # Entry point, QMainWindow, dark Fusion theme
├── data_adapter.py     # DataAdapter: đọc CSV → NumPy arrays
├── assets/             # (icons, stylesheets — future)
├── widgets/
│   ├── __init__.py
│   └── plot_timeseries.py  # TimeseriesPlot widget: subplots + checkboxes
└── tabs/
    ├── __init__.py
    ├── tab_position.py
    ├── tab_velocity.py
    ├── tab_attitude_euler.py
    ├── tab_quaternion.py
    ├── tab_bias.py
    └── tab_innovations.py
```

## Thiết kế

- **Không chứa filter logic** — chỉ đọc CSV và vẽ
- **Dark theme**: Fusion palette, phù hợp hiển thị nhiều đường
- **Màu tiêu chuẩn**: Truth = vàng, Estimated = cyan, Measured = cam
- **Linked x-axes**: Zoom/pan 1 subplot → tất cả sync
- **Checkboxes**: Ẩn/hiện Truth / Estimated / Measured
- **RMSE display**: Tính và hiển thị trên title mỗi subplot (position, velocity, attitude)
