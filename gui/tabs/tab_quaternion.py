"""gui.tabs.tab_quaternion — Tab quaternion: Truth vs Estimated + norm(q).

Hiển thị 4 subplot: w, x, y, z + 1 thêm cho norm(q).
Norm lý tưởng = 1.0 — nếu trượt khỏi 1 → vấn đề numerical.
"""
import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QCheckBox
from PyQt6.QtCore import Qt
from gui.data_adapter import DataAdapter

# Màu
CLR_TRUTH = (100, 200, 100, 200)
CLR_EST = (100, 150, 255, 255)


class TabQuaternion(QWidget):
    """Tab quaternion [w,x,y,z] truth vs est + norm(q)."""

    def __init__(self, adapter: DataAdapter, parent=None):
        super().__init__(parent)
        self.adapter = adapter

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        # Checkbox row
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

        # Plot layout
        self.pw = pg.GraphicsLayoutWidget()
        self.pw.setBackground('k')
        layout.addWidget(self.pw, stretch=1)

        # 5 subplot: w, x, y, z, norm
        labels = ['q_w', 'q_x', 'q_y', 'q_z', '||q||']
        self.plots = []
        self.curves_truth = []
        self.curves_est = []

        for i, lbl in enumerate(labels):
            if i > 0:
                self.pw.nextRow()
            p = self.pw.addPlot(title=lbl)
            p.setLabel('bottom', 't (s)')
            p.showGrid(x=True, y=True, alpha=0.3)

            ct = p.plot(pen=pg.mkPen(color=CLR_TRUTH, width=1.5), name='Truth')
            ce = p.plot(pen=pg.mkPen(color=CLR_EST, width=2, style=Qt.PenStyle.DashLine), name='Est')

            self.plots.append(p)
            self.curves_truth.append(ct)
            self.curves_est.append(ce)

        # Link x
        for i in range(1, len(self.plots)):
            self.plots[i].setXLink(self.plots[0])

        self._populate()

    def _populate(self):
        est = self.adapter.est
        if len(est.t) == 0:
            return

        step = max(1, len(est.t) // 3000)
        t = est.t[::step]

        # q components (truth + est)
        for i in range(4):
            self.curves_truth[i].setData(t, est.truth_quat[::step, i])
            self.curves_est[i].setData(t, est.quat[::step, i])

        # norm(q)
        q_est = est.quat[::step]
        q_truth = est.truth_quat[::step]
        norm_est = np.sqrt(np.sum(q_est**2, axis=1))
        norm_truth = np.sqrt(np.sum(q_truth**2, axis=1))
        self.curves_truth[4].setData(t, norm_truth)
        self.curves_est[4].setData(t, norm_est)

    def _update_vis(self):
        for i in range(5):
            self.curves_truth[i].setVisible(self.cb_truth.isChecked())
            self.curves_est[i].setVisible(self.cb_est.isChecked())
