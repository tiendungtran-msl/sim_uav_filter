"""sim — Gói mô phỏng UAV 6DOF độc lập.

Chạy GUI:      python -m sim.app
Chạy headless:  python -m sim.cli --headless --time 60 --seed 1 --out logs/run1.csv

Module này KHÔNG phụ thuộc vào bất kỳ thuật toán lọc (ESKF, EKF, …) nào.
Dữ liệu được xuất qua SimFrame để bên filter "cắm vào" sau.
"""
