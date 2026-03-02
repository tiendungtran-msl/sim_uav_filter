# data/ — Dữ liệu mẫu và log

Thư mục chứa dữ liệu input/output cho ESKF.

## Cấu trúc

```
data/
└── examples/
    ├── sensor_log.csv      # Input: 47 cột, 30s @ 400Hz = 12000 hàng
    ├── estimated.csv        # Output ESKF: 38 cột, ước lượng + truth
    └── innovations.csv      # Output ESKF: innovation records
```

## Tạo dữ liệu mới

```bash
# Từ thư mục gốc sim_ekf2:
python3 -m sim.cli --headless --time 30 --seed 42 --out data/examples/sensor_log.csv

# Chạy ESKF replay:
cd filters/eskf_core_cpp/build
./eskf_replay ../../../data/examples/sensor_log.csv
```

## Xem chi tiết format

Xem [docs/log_format.md](../docs/log_format.md) cho mô tả chi tiết từng cột.
