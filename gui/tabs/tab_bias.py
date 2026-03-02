"""gui.tabs.tab_bias — Tab bias: Gyro bias + Accel bias (truth vs estimated).

Hiển thị 6 subplot (2 nhóm × 3 trục):
- Gyro bias x, y, z
- Accel bias x, y, z

Truth bias thường là random walk chậm — estimated bias cần hội tụ về truth.
"""
import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QCheckBox, QLabel
from PyQt6.QtCore import Qt
from gui.data_adapter import DataAdapter

CLR_TRUTH = (100, 200, 100, 200)
CLR_EST = (100, 150, 255, 255)


class TabBias(QWidget):
    """Tab bias: gyro bias + accel bias truth vs estimated."""

    def __init__(self, adapter: DataAdapter, parent=None):
        super().__init__(parent)
        self.adapter = adapter

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        self.info_label = QLabel("Bias convergence")
        self.info_label.setStyleSheet("color: #aaa; font-size: 11px;")
        layout.addWidget(self.info_label)

        # Checkbox
        cb_row = QHBoxLayout()
        self.cb_truth = QCheckBox("Truth")
        self.cb_truth.setChecked(True)
        self.cb_truth.stateChanged.connect(self._update_vis)
        self.cb_est = QCheckBox("Estimated")
        self.cb_est.setChecked(True)
        self.cb_est.stateChanged.connect(self._update_vis)
        cb_row.addWidget(self.cb_truth)
        cb_row.addWidget(self.cb_est)
        cb_row.addStretch()
        layout.addLayout(cb_row)

        # 2 cột: gyro (trái) + accel (phải)
        self.pw = pg.GraphicsLayoutWidget()
        self.pw.setBackground('k')
        layout.addWidget(self.pw, stretch=1)

        labels_bg = ['bg_x (rad/s)', 'bg_y (rad/s)', 'bg_z (rad/s)']
        labels_ba = ['ba_x (m/s²)', 'ba_y (m/s²)', 'ba_z (m/s²)']

        self.plots = []
        self.curves_truth = []
        self.curves_est = []

        for row_i in range(3):
            # Gyro bias
            p_bg = self.pw.addPlot(title=labels_bg[row_i])
            p_bg.setLabel('bottom', 't (s)')
            p_bg.showGrid(x=True, y=True, alpha=0.3)
            ct = p_bg.plot(pen=pg.mkPen(color=CLR_TRUTH, width=1.5))
            ce = p_bg.plot(pen=pg.mkPen(color=CLR_EST, width=2, style=Qt.PenStyle.DashLine))
            self.plots.append(p_bg)
            self.curves_truth.append(ct)
            self.curves_est.append(ce)

            # Accel bias
            p_ba = self.pw.addPlot(title=labels_ba[row_i])
            p_ba.setLabel('bottom', 't (s)')
            p_ba.showGrid(x=True, y=True, alpha=0.3)
            ct2 = p_ba.plot(pen=pg.mkPen(color=CLR_TRUTH, width=1.5))
            ce2 = p_ba.plot(pen=pg.mkPen(color=CLR_EST, width=2, style=Qt.PenStyle.DashLine))
            self.plots.append(p_ba)
            self.curves_truth.append(ct2)
            self.curves_est.append(ce2)

            if row_i < 2:
                self.pw.nextRow()

        # Link x axes
        for i in range(1, len(self.plots)):
            self.plots[i].setXLink(self.plots[0])

        self._populate()

    def _populate(self):
        est = self.adapter.est
        if len(est.t) == 0:
            return

        step = max(1, len(est.t) // 3000)
        t = est.t[::step]

        for axis_i in range(3):
            idx_bg = axis_i * 2       # gyro bias plot index
            idx_ba = axis_i * 2 + 1   # accel bias plot index

            # Gyro bias
            self.curves_truth[idx_bg].setData(t, est.truth_bg[::step, axis_i])
            self.curves_est[idx_bg].setData(t, est.bg[::step, axis_i])

            # Accel bias
            self.curves_truth[idx_ba].setData(t, est.truth_ba[::step, axis_i])
            self.curves_est[idx_ba].setData(t, est.ba[::step, axis_i])

        # Tính bias error cuối
        if len(est.t) > 100:
            bg_err = est.bg[-1] - est.truth_bg[-1]
            ba_err = est.ba[-1] - est.truth_ba[-1]
            self.info_label.setText(
                f"Final bg error: [{bg_err[0]:.5f}, {bg_err[1]:.5f}, {bg_err[2]:.5f}] rad/s  |  "
                f"Final ba error: [{ba_err[0]:.4f}, {ba_err[1]:.4f}, {ba_err[2]:.4f}] m/s²"
            )

    def _update_vis(self):
        show_t = self.cb_truth.isChecked()
        show_e = self.cb_est.isChecked()
        for i in range(len(self.curves_truth)):
            self.curves_truth[i].setVisible(show_t)
            self.curves_est[i].setVisible(show_e)
