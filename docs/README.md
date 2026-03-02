# docs/ — Tài liệu dự án ESKF

Thư mục chứa tài liệu kỹ thuật cho hệ thống ESKF C++ và GUI đánh giá.

## Nội dung

| File | Mô tả |
|------|-------|
| [log_format.md](log_format.md) | Mô tả chi tiết 47 cột trong sensor_log.csv |
| [architecture.md](architecture.md) | Kiến trúc tổng quan: Simulator → CSV → ESKF C++ → CSV → GUI |

## Quy ước chung

- **Frame**: NED (North-East-Down)
- **Quaternion**: Hamilton, scalar-first `[w, x, y, z]`, body → NED
- **Đơn vị**: SI (m, m/s, rad/s, µT)
- **Thời gian**: giây (s) kể từ t = 0 (bắt đầu mô phỏng)
- **Comment**: tiếng Việt trong code, tiếng Anh/Việt trong docs
