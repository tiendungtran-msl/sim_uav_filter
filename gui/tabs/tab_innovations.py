"""gui.tabs.tab_innovations — Tab Innovations & NIS cho GPS/MAG/Baro.

Hiển thị:
- Innovation values per sensor
- NIS (Normalized Innovation Squared) với chi-square bounds
- Đường chi² threshold (dùng để kiểm tra consistency)

Chi-square bounds:
- GPS 6D: χ²(0.95, 6) = 12.59, χ²(0.99, 6) = 16.81
- MAG 3D: χ²(0.95, 3) = 7.81,  χ²(0.99, 3) = 11.34
- Baro 1D: χ²(0.95, 1) = 3.84, χ²(0.99, 1) = 6.63
"""
import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QTabWidget, QLabel
from gui.data_adapter import DataAdapter

# Chi-square 95% thresholds
CHI2_95 = {6: 12.59, 3: 7.81, 1: 3.84}


class TabInnovations(QWidget):
    """Tab innovations/NIS — inner tabs cho GPS, MAG, Baro."""

    def __init__(self, adapter: DataAdapter, parent=None):
        super().__init__(parent)
        self.adapter = adapter

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        self.info_label = QLabel("Innovations và NIS — kiểm tra consistency")
        self.info_label.setStyleSheet("color: #aaa; font-size: 11px;")
        layout.addWidget(self.info_label)

        # Inner tabs: GPS, MAG, Baro
        self.inner_tabs = QTabWidget()
        layout.addWidget(self.inner_tabs, stretch=1)

        self._build_gps_tab()
        self._build_mag_tab()
        self._build_baro_tab()

    def _build_gps_tab(self):
        """GPS innovations (6D) + NIS."""
        innov = self.adapter.innov
        w = pg.GraphicsLayoutWidget()
        w.setBackground('k')
        self.inner_tabs.addTab(w, "GPS")

        if len(innov.gps_t) == 0:
            return

        labels = ['innov_pN (m)', 'innov_pE (m)', 'innov_pD (m)',
                  'innov_vN (m/s)', 'innov_vE (m/s)', 'innov_vD (m/s)']

        # 6 innovation plots (2 cột × 3 hàng)
        for i in range(6):
            if i > 0 and i % 2 == 0:
                w.nextRow()
            p = w.addPlot(title=labels[i])
            p.setLabel('bottom', 't (s)')
            p.showGrid(x=True, y=True, alpha=0.3)

            # Accepted vs rejected
            t_acc = innov.gps_t[innov.gps_accepted > 0.5]
            y_acc = innov.gps_innov[innov.gps_accepted > 0.5, i]
            t_rej = innov.gps_t[innov.gps_accepted < 0.5]
            y_rej = innov.gps_innov[innov.gps_accepted < 0.5, i]

            if len(t_acc) > 0:
                p.plot(t_acc, y_acc, pen=None,
                       symbol='o', symbolSize=3,
                       symbolBrush=(100, 200, 100, 180), name='Accepted')
            if len(t_rej) > 0:
                p.plot(t_rej, y_rej, pen=None,
                       symbol='x', symbolSize=5,
                       symbolBrush=(255, 80, 80, 200), name='Rejected')

        # NIS plot
        w.nextRow()
        p_nis = w.addPlot(title='GPS NIS', colspan=2)
        p_nis.setLabel('bottom', 't (s)')
        p_nis.setLabel('left', 'NIS')
        p_nis.showGrid(x=True, y=True, alpha=0.3)

        if len(innov.gps_t) > 0:
            p_nis.plot(innov.gps_t, innov.gps_nis, pen=None,
                       symbol='o', symbolSize=3,
                       symbolBrush=(100, 150, 255, 180))
            # Chi² threshold line
            chi2_line = CHI2_95[6]
            p_nis.addLine(y=chi2_line, pen=pg.mkPen('r', width=1, style=pg.QtCore.Qt.PenStyle.DashLine))

    def _build_mag_tab(self):
        """MAG innovations (3D) + NIS."""
        innov = self.adapter.innov
        w = pg.GraphicsLayoutWidget()
        w.setBackground('k')
        self.inner_tabs.addTab(w, "MAG")

        if len(innov.mag_t) == 0:
            return

        labels = ['innov_mx (µT)', 'innov_my (µT)', 'innov_mz (µT)']

        for i in range(3):
            if i > 0:
                w.nextRow()
            p = w.addPlot(title=labels[i])
            p.setLabel('bottom', 't (s)')
            p.showGrid(x=True, y=True, alpha=0.3)

            t_acc = innov.mag_t[innov.mag_accepted > 0.5]
            y_acc = innov.mag_innov[innov.mag_accepted > 0.5, i]
            t_rej = innov.mag_t[innov.mag_accepted < 0.5]
            y_rej = innov.mag_innov[innov.mag_accepted < 0.5, i]

            if len(t_acc) > 0:
                p.plot(t_acc, y_acc, pen=None,
                       symbol='o', symbolSize=3,
                       symbolBrush=(100, 200, 100, 180))
            if len(t_rej) > 0:
                p.plot(t_rej, y_rej, pen=None,
                       symbol='x', symbolSize=5,
                       symbolBrush=(255, 80, 80, 200))

        # NIS
        w.nextRow()
        p_nis = w.addPlot(title='MAG NIS')
        p_nis.setLabel('bottom', 't (s)')
        p_nis.showGrid(x=True, y=True, alpha=0.3)

        if len(innov.mag_t) > 0:
            p_nis.plot(innov.mag_t, innov.mag_nis, pen=None,
                       symbol='o', symbolSize=3,
                       symbolBrush=(100, 150, 255, 180))
            p_nis.addLine(y=CHI2_95[3], pen=pg.mkPen('r', width=1, style=pg.QtCore.Qt.PenStyle.DashLine))

    def _build_baro_tab(self):
        """Baro innovation (1D) + NIS."""
        innov = self.adapter.innov
        w = pg.GraphicsLayoutWidget()
        w.setBackground('k')
        self.inner_tabs.addTab(w, "Baro")

        if len(innov.baro_t) == 0:
            return

        # Innovation
        p_innov = w.addPlot(title='Baro innovation (m)')
        p_innov.setLabel('bottom', 't (s)')
        p_innov.showGrid(x=True, y=True, alpha=0.3)

        t_acc = innov.baro_t[innov.baro_accepted > 0.5]
        y_acc = innov.baro_innov[innov.baro_accepted > 0.5]
        t_rej = innov.baro_t[innov.baro_accepted < 0.5]
        y_rej = innov.baro_innov[innov.baro_accepted < 0.5]

        if len(t_acc) > 0:
            p_innov.plot(t_acc, y_acc, pen=None,
                         symbol='o', symbolSize=3,
                         symbolBrush=(100, 200, 100, 180))
        if len(t_rej) > 0:
            p_innov.plot(t_rej, y_rej, pen=None,
                         symbol='x', symbolSize=5,
                         symbolBrush=(255, 80, 80, 200))

        # NIS
        w.nextRow()
        p_nis = w.addPlot(title='Baro NIS')
        p_nis.setLabel('bottom', 't (s)')
        p_nis.showGrid(x=True, y=True, alpha=0.3)

        if len(innov.baro_t) > 0:
            p_nis.plot(innov.baro_t, innov.baro_nis, pen=None,
                       symbol='o', symbolSize=3,
                       symbolBrush=(100, 150, 255, 180))
            p_nis.addLine(y=CHI2_95[1], pen=pg.mkPen('r', width=1, style=pg.QtCore.Qt.PenStyle.DashLine))
