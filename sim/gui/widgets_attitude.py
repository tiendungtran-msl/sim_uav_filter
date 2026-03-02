"""sim.gui.widgets_attitude — Widget tư thế UAV theo phong cách flight instrument 2D.

Mục tiêu:
- Dễ đọc, khoa học, đo được trực tiếp (roll/pitch/yaw).
- Nhẹ hơn OpenGL để giảm lag khi chạy realtime lâu.

Bố cục gồm 2 hàng:
1) Artificial horizon (roll + pitch)
2) Heading trend (yaw theo thời gian)
"""
import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtCore import Qt

from sim.utils.ring_buffer import RingBuffer


class AttitudeWidget(QWidget):
    """Hiển thị attitude theo dạng dụng cụ bay 2D."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._label = QLabel("Attitude")
        self._label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._label.setStyleSheet("color: #ddd; font-weight: 600; font-size: 13px;")
        layout.addWidget(self._label)

        # ------------------------------------------------------------------
        # Plot 1: Artificial horizon
        # ------------------------------------------------------------------
        self._horizon_plot = pg.PlotWidget(title="Artificial Horizon")
        layout.addWidget(self._horizon_plot)
        self._horizon_plot.setAspectLocked(True)
        self._horizon_plot.showGrid(x=True, y=True, alpha=0.25)
        self._horizon_plot.setXRange(-1.2, 1.2)
        self._horizon_plot.setYRange(-1.2, 1.2)
        self._horizon_plot.hideAxis("left")
        self._horizon_plot.hideAxis("bottom")

        # Đường chân trời (xoay theo roll, tịnh tiến theo pitch)
        self._horizon_line = self._horizon_plot.plot(
            pen=pg.mkPen((120, 220, 255), width=3)
        )
        # Cánh máy bay cố định ở tâm
        self._aircraft = self._horizon_plot.plot(
            np.array([-0.35, 0.35]), np.array([0.0, 0.0]),
            pen=pg.mkPen((255, 230, 90), width=4),
        )
        self._center_dot = self._horizon_plot.plot(
            np.array([0.0]), np.array([0.0]),
            pen=None, symbol="o", symbolSize=8, symbolBrush=(255, 230, 90),
        )

        # ------------------------------------------------------------------
        # Plot 2: Heading trend
        # ------------------------------------------------------------------
        self._heading_plot = pg.PlotWidget(title="Heading (Yaw) Trend")
        layout.addWidget(self._heading_plot)
        self._heading_plot.setLabel("left", "Yaw", units="deg")
        self._heading_plot.setLabel("bottom", "Sample")
        self._heading_plot.showGrid(x=True, y=True, alpha=0.25)
        self._heading_plot.setYRange(-180, 180)

        self._yaw_buf = RingBuffer(400, 1)
        self._yaw_curve = self._heading_plot.plot(
            pen=pg.mkPen((220, 130, 255), width=2)
        )

    def update_attitude(self, quat: np.ndarray) -> None:
        """Update horizon từ quaternion (không cần dùng trực tiếp ở đây)."""
        # Hàm giữ API tương thích với dashboard.
        _ = quat

    def update_label(self, euler_deg: np.ndarray) -> None:
        """Cập nhật trạng thái roll/pitch/yaw và vẽ instrument."""
        roll_deg, pitch_deg, yaw_deg = float(euler_deg[0]), float(euler_deg[1]), float(euler_deg[2])

        # Label số liệu
        self._label.setText(
            f"Attitude  Roll={roll_deg:+6.1f}°   Pitch={pitch_deg:+6.1f}°   Yaw={yaw_deg:+6.1f}°"
        )

        # Horizon line
        roll = np.deg2rad(roll_deg)
        pitch_offset = np.clip(pitch_deg / 40.0, -0.9, 0.9)

        span = 1.3
        x = np.array([-span, span])
        y = np.tan(roll) * x + pitch_offset
        self._horizon_line.setData(x, y)

        # Heading trend
        yaw_norm = ((yaw_deg + 180.0) % 360.0) - 180.0
        self._yaw_buf.push(np.array([yaw_norm], dtype=float))
        yv = self._yaw_buf.get_array()
        if len(yv) > 1:
            xv = np.arange(len(yv), dtype=float)
            self._yaw_curve.setData(xv, yv[:, 0])
