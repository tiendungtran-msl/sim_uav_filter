"""gui.tabs.tab_velocity — Tab so sánh vận tốc: Truth vs Estimated vs GPS.

Hiển thị 3 subplot: vN, vE, vD.
"""
import numpy as np
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from gui.widgets.plot_timeseries import TimeseriesPlot
from gui.data_adapter import DataAdapter


class TabVelocity(QWidget):
    """Tab vận tốc NED: truth vs est vs GPS measurement."""

    def __init__(self, adapter: DataAdapter, parent=None):
        super().__init__(parent)
        self.adapter = adapter

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        self.rmse_label = QLabel("RMSE Velocity: —")
        self.rmse_label.setStyleSheet("color: #aaa; font-size: 11px;")
        layout.addWidget(self.rmse_label)

        self.plot = TimeseriesPlot(
            title="Vận tốc NED",
            y_labels=["vN (m/s)", "vE (m/s)", "vD (m/s)"],
            n_axes=3
        )
        layout.addWidget(self.plot, stretch=1)

        self._populate()

    def _populate(self):
        est = self.adapter.est
        meas = self.adapter.meas

        if len(est.t) == 0:
            return

        step = max(1, len(est.t) // 3000)

        self.plot.set_data(
            t_truth=est.t[::step], y_truth=est.truth_vel[::step],
            t_est=est.t[::step],   y_est=est.vel[::step],
            t_meas=meas.gps_t if len(meas.gps_t) > 0 else None,
            y_meas=meas.gps_vel if len(meas.gps_vel) > 0 else None,
        )

        err = est.vel - est.truth_vel
        rmse = np.sqrt(np.mean(err**2, axis=0))
        rmse_total = np.sqrt(np.mean(np.sum(err**2, axis=1)))
        self.rmse_label.setText(
            f"RMSE Velocity: N={rmse[0]:.4f}m/s  E={rmse[1]:.4f}m/s  "
            f"D={rmse[2]:.4f}m/s  |  Total={rmse_total:.4f}m/s"
        )
