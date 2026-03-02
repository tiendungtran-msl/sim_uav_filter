"""gui.tabs.tab_attitude_euler — Tab so sánh tư thế Euler: Truth vs Estimated.

Hiển thị 3 subplot: Roll, Pitch, Yaw (degrees).
Cẩn thận wrap yaw ±180°.
"""
import numpy as np
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from gui.widgets.plot_timeseries import TimeseriesPlot
from gui.data_adapter import DataAdapter


class TabAttitudeEuler(QWidget):
    """Tab tư thế Euler (Roll/Pitch/Yaw) in degrees."""

    def __init__(self, adapter: DataAdapter, parent=None):
        super().__init__(parent)
        self.adapter = adapter

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        self.rmse_label = QLabel("RMSE Attitude: —")
        self.rmse_label.setStyleSheet("color: #aaa; font-size: 11px;")
        layout.addWidget(self.rmse_label)

        self.plot = TimeseriesPlot(
            title="Tư thế Euler",
            y_labels=["Roll (°)", "Pitch (°)", "Yaw (°)"],
            n_axes=3
        )
        layout.addWidget(self.plot, stretch=1)

        self._populate()

    def _populate(self):
        est = self.adapter.est

        if len(est.t) == 0:
            return

        step = max(1, len(est.t) // 3000)

        # Dùng euler_deg
        self.plot.set_data(
            t_truth=est.t[::step], y_truth=est.truth_euler_deg[::step],
            t_est=est.t[::step],   y_est=est.euler_deg[::step],
        )

        # RMSE tính trên radian, wrap yaw
        err_rad = est.euler_rad - est.truth_euler_rad
        # Wrap yaw error
        err_rad[:, 2] = np.arctan2(np.sin(err_rad[:, 2]), np.cos(err_rad[:, 2]))
        rmse_deg = np.degrees(np.sqrt(np.mean(err_rad**2, axis=0)))
        self.rmse_label.setText(
            f"RMSE Attitude: Roll={rmse_deg[0]:.2f}°  Pitch={rmse_deg[1]:.2f}°  "
            f"Yaw={rmse_deg[2]:.2f}°"
        )
