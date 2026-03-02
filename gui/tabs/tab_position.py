"""gui.tabs.tab_position — Tab so sánh vị trí: Truth vs Estimated vs GPS.

Hiển thị 3 subplot: North, East, Down.
Mỗi subplot có 3 đường:
- Truth (ground truth từ simulator)
- Estimated (output ESKF)
- Measured (GPS, scatter)

Có thêm hiển thị RMSE position.
"""
import numpy as np
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from gui.widgets.plot_timeseries import TimeseriesPlot
from gui.data_adapter import DataAdapter


class TabPosition(QWidget):
    """Tab vị trí NED: truth vs est vs GPS measurement."""

    def __init__(self, adapter: DataAdapter, parent=None):
        super().__init__(parent)
        self.adapter = adapter

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        # RMSE label
        self.rmse_label = QLabel("RMSE Position: —")
        self.rmse_label.setStyleSheet("color: #aaa; font-size: 11px;")
        layout.addWidget(self.rmse_label)

        # Plot widget
        self.plot = TimeseriesPlot(
            title="Vị trí NED",
            y_labels=["North (m)", "East (m)", "Down (m)"],
            n_axes=3
        )
        layout.addWidget(self.plot, stretch=1)

        self._populate()

    def _populate(self):
        """Đổ dữ liệu vào plot."""
        est = self.adapter.est
        meas = self.adapter.meas

        if len(est.t) == 0:
            return

        # Downsample truth/est cho performance (mỗi 10 sample)
        step = max(1, len(est.t) // 3000)

        self.plot.set_data(
            t_truth=est.t[::step], y_truth=est.truth_pos[::step],
            t_est=est.t[::step],   y_est=est.pos[::step],
            t_meas=meas.gps_t if len(meas.gps_t) > 0 else None,
            y_meas=meas.gps_pos if len(meas.gps_pos) > 0 else None,
        )

        # Tính RMSE
        err = est.pos - est.truth_pos
        rmse = np.sqrt(np.mean(err**2, axis=0))
        rmse_total = np.sqrt(np.mean(np.sum(err**2, axis=1)))
        self.rmse_label.setText(
            f"RMSE Position: N={rmse[0]:.3f}m  E={rmse[1]:.3f}m  "
            f"D={rmse[2]:.3f}m  |  Total={rmse_total:.3f}m"
        )
