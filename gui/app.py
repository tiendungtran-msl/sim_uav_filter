"""gui.app — Main GUI application cho đánh giá ESKF.

Đây là entry point cho GUI. Đọc output từ ESKF C++ replay tool
và hiển thị so sánh Truth vs Estimated vs Measured theo tabs.

Cách chạy:
    python -m gui.app [--sensor sensor_log.csv] [--est estimated.csv] [--innov innovations.csv]

Hoặc dùng đường dẫn mặc định từ data/examples/.

GUI KHÔNG chứa logic ESKF — chỉ đọc CSV và vẽ.
"""
import sys
import argparse
from pathlib import Path

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QTabWidget, QMessageBox, QStatusBar
)
from PyQt6.QtCore import Qt

from gui.data_adapter import DataAdapter
from gui.tabs.tab_position import TabPosition
from gui.tabs.tab_velocity import TabVelocity
from gui.tabs.tab_attitude_euler import TabAttitudeEuler
from gui.tabs.tab_quaternion import TabQuaternion
from gui.tabs.tab_bias import TabBias
from gui.tabs.tab_innovations import TabInnovations


class EskfViewerWindow(QMainWindow):
    """Cửa sổ chính — chứa tab widget với 6 tabs."""

    def __init__(self, adapter: DataAdapter):
        super().__init__()
        self.adapter = adapter

        self.setWindowTitle("ESKF Evaluation — Truth vs Estimated vs Measured")
        self.resize(1400, 900)

        # Tab widget trung tâm
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # Tạo các tab
        self.tabs.addTab(TabPosition(adapter), "📍 Vị trí")
        self.tabs.addTab(TabVelocity(adapter), "🚀 Vận tốc")
        self.tabs.addTab(TabAttitudeEuler(adapter), "🧭 Tư thế Euler")
        self.tabs.addTab(TabQuaternion(adapter), "🔄 Quaternion")
        self.tabs.addTab(TabBias(adapter), "📊 Bias")
        self.tabs.addTab(TabInnovations(adapter), "📈 Innovations")

        # Status bar
        t_min, t_max = adapter.time_range()
        n_est = len(adapter.est.t)
        n_gps = len(adapter.meas.gps_t)
        status = QStatusBar()
        status.showMessage(
            f"Time: {t_min:.1f}s — {t_max:.1f}s  |  "
            f"Estimated: {n_est} samples  |  "
            f"GPS meas: {n_gps} samples"
        )
        self.setStatusBar(status)


def main():
    parser = argparse.ArgumentParser(description="ESKF Evaluation GUI")
    parser.add_argument("--sensor", type=str, default="data/examples/sensor_log.csv",
                        help="Path to sensor log CSV")
    parser.add_argument("--est", type=str, default="data/examples/estimated.csv",
                        help="Path to estimated CSV (từ ESKF replay)")
    parser.add_argument("--innov", type=str, default="data/examples/innovations.csv",
                        help="Path to innovations CSV")
    args = parser.parse_args()

    # Đọc dữ liệu
    adapter = DataAdapter()

    print("Đang đọc dữ liệu...")

    if not adapter.load_sensor_log(args.sensor):
        print(f"Cảnh báo: Không đọc được sensor log: {args.sensor}")

    if not adapter.load_estimated(args.est):
        print(f"LỖI: Không đọc được estimated file: {args.est}")
        print("Hãy chạy ESKF replay tool trước:")
        print("  cd filters/eskf_core_cpp/build")
        print("  ./eskf_replay ../../../data/examples/sensor_log.csv")
        sys.exit(1)

    if not adapter.load_innovations(args.innov):
        print(f"Cảnh báo: Không đọc được innovations: {args.innov}")

    print(f"  Estimated: {len(adapter.est.t)} samples")
    print(f"  GPS meas:  {len(adapter.meas.gps_t)} samples")
    print(f"  GPS innov: {len(adapter.innov.gps_t)} records")
    print(f"  MAG innov: {len(adapter.innov.mag_t)} records")

    # Khởi chạy GUI
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    # Dark palette
    from PyQt6.QtGui import QPalette, QColor
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(30, 30, 30))
    palette.setColor(QPalette.ColorRole.WindowText, QColor(200, 200, 200))
    palette.setColor(QPalette.ColorRole.Base, QColor(20, 20, 20))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(35, 35, 35))
    palette.setColor(QPalette.ColorRole.Text, QColor(200, 200, 200))
    palette.setColor(QPalette.ColorRole.Button, QColor(40, 40, 40))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(200, 200, 200))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(50, 100, 200))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor(255, 255, 255))
    app.setPalette(palette)

    window = EskfViewerWindow(adapter)
    window.show()

    print("GUI đang chạy. Đóng cửa sổ để thoát.")
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
