"""gui.widgets.plot_timeseries — Widget vẽ timeseries tiện dụng (pyqtgraph).

Dùng cho tất cả các tab: position, velocity, attitude, bias, innovations.
Hỗ trợ:
- Nhiều đường (truth, est, meas) với legend
- Checkbox bật/tắt series
- Time range zoom
- Linked x-axis giữa các subplot
"""
import pyqtgraph as pg
import numpy as np
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QCheckBox
from PyQt6.QtCore import Qt

# Bảng màu cho 3 loại đường: truth, estimated, measured
COLORS = {
    'truth': (100, 200, 100, 200),   # xanh lá nhạt
    'est':   (100, 150, 255, 255),    # xanh dương sáng
    'meas':  (255, 150, 50, 150),     # cam nhạt (meas thường scatter)
}

# Bảng màu cho 3 axes: x/N, y/E, z/D
AXIS_COLORS = [
    (255, 80, 80),    # đỏ     → x/N axis
    (80, 200, 80),    # xanh lá → y/E axis
    (80, 80, 255),    # xanh dương → z/D axis
]


class TimeseriesPlot(QWidget):
    """Widget vẽ 1-3 channels timeseries với truth/est/meas.

    Ví dụ: Vị trí N, E, D — 3 subplot dọc, mỗi cái có 3 đường.
    """

    def __init__(self, title: str = "", y_labels: list = None,
                 n_axes: int = 3, parent=None):
        """
        Args:
            title:    Tiêu đề widget
            y_labels: Nhãn cho mỗi subplot (ví dụ: ["N (m)", "E (m)", "D (m)"])
            n_axes:   Số subplot (1-3)
        """
        super().__init__(parent)
        self.n_axes = n_axes
        if y_labels is None:
            y_labels = [f"Axis {i}" for i in range(n_axes)]

        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)

        # Checkbox row
        cb_row = QHBoxLayout()
        self.cb_truth = QCheckBox("Truth")
        self.cb_truth.setChecked(True)
        self.cb_truth.stateChanged.connect(self._update_visibility)
        self.cb_est = QCheckBox("Estimated")
        self.cb_est.setChecked(True)
        self.cb_est.stateChanged.connect(self._update_visibility)
        self.cb_meas = QCheckBox("Measured")
        self.cb_meas.setChecked(True)
        self.cb_meas.stateChanged.connect(self._update_visibility)
        cb_row.addWidget(self.cb_truth)
        cb_row.addWidget(self.cb_est)
        cb_row.addWidget(self.cb_meas)
        cb_row.addStretch()
        layout.addLayout(cb_row)

        # Tạo plot layout
        self.pw = pg.GraphicsLayoutWidget()
        self.pw.setBackground('k')
        layout.addWidget(self.pw, stretch=1)

        # Tạo subplot cho mỗi axis
        self.plots = []
        self.curves_truth = []
        self.curves_est = []
        self.curves_meas = []

        for i in range(n_axes):
            if i > 0:
                self.pw.nextRow()

            p = self.pw.addPlot(title=f"{title} — {y_labels[i]}" if title else y_labels[i])
            p.setLabel('left', y_labels[i])
            p.setLabel('bottom', 't (s)')
            p.showGrid(x=True, y=True, alpha=0.3)
            p.addLegend(offset=(10, 10), labelTextSize='8pt')

            # Truth line (solid, medium width)
            c_truth = p.plot(pen=pg.mkPen(color=COLORS['truth'], width=1.5),
                             name='Truth')
            # Estimated line (dashed, thicker)
            c_est = p.plot(pen=pg.mkPen(color=COLORS['est'], width=2, style=Qt.PenStyle.DashLine),
                           name='Est')
            # Measured scatter (small dots)
            c_meas = pg.ScatterPlotItem(pen=None,
                                         brush=pg.mkBrush(*COLORS['meas']),
                                         size=4, name='Meas')
            p.addItem(c_meas)

            self.plots.append(p)
            self.curves_truth.append(c_truth)
            self.curves_est.append(c_est)
            self.curves_meas.append(c_meas)

        # Link x axes
        for i in range(1, n_axes):
            self.plots[i].setXLink(self.plots[0])

    def set_data(self, t_truth=None, y_truth=None,
                 t_est=None, y_est=None,
                 t_meas=None, y_meas=None):
        """Cập nhật toàn bộ dữ liệu.

        Args:
            t_truth, y_truth: Time + values cho truth (y shape: N×n_axes)
            t_est, y_est:     Time + values cho estimated
            t_meas, y_meas:   Time + values cho measured (có thể None)
        """
        for i in range(self.n_axes):
            if t_truth is not None and y_truth is not None and len(t_truth) > 0:
                yi = y_truth[:, i] if y_truth.ndim > 1 else y_truth
                self.curves_truth[i].setData(t_truth, yi)

            if t_est is not None and y_est is not None and len(t_est) > 0:
                yi = y_est[:, i] if y_est.ndim > 1 else y_est
                self.curves_est[i].setData(t_est, yi)

            if t_meas is not None and y_meas is not None and len(t_meas) > 0:
                yi = y_meas[:, i] if y_meas.ndim > 1 else y_meas
                self.curves_meas[i].setData(t_meas, yi)

    def _update_visibility(self):
        """Bật/tắt series theo checkbox."""
        show_t = self.cb_truth.isChecked()
        show_e = self.cb_est.isChecked()
        show_m = self.cb_meas.isChecked()
        for i in range(self.n_axes):
            self.curves_truth[i].setVisible(show_t)
            self.curves_est[i].setVisible(show_e)
            self.curves_meas[i].setVisible(show_m)
