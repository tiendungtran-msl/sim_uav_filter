"""sim.gui.widgets_plots — Bộ đồ thị tối giản theo dữ liệu cần thiết.

Truth cần hiển thị:
- Attitude (roll/pitch/yaw)
- Angular rate (wx/wy/wz)
- Position (N/E/D)
- Velocity (Vn/Ve/Vd)
- Acceleration (ax/ay/az)

Measurement cần hiển thị:
- IMU gyro + accel
- Mag heading
- GPS position
"""
import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QSplitter, QTabWidget
from PyQt6.QtCore import Qt
from typing import cast

from sim.utils.ring_buffer import RingBuffer

COLOR_T = [(255, 100, 100), (120, 240, 120), (120, 170, 255)]
COLOR_M = [(255, 180, 180), (180, 255, 180), (180, 210, 255)]
COLOR_YAW = (230, 150, 255)
COLOR_YAW_M = (255, 220, 130)


class TrajectoryWidget(QWidget):
    """Quỹ đạo chất điểm: N-E top view + D theo thời gian."""

    def __init__(self, capacity: int = 3500, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        splitter = QSplitter(Qt.Orientation.Vertical)
        layout.addWidget(splitter)

        self._ne_plot = pg.PlotWidget(title="Trajectory N-E (Truth vs GPS)")
        self._style_plot(self._ne_plot)
        self._ne_plot.setLabel("left", "North", units="m")
        self._ne_plot.setLabel("bottom", "East", units="m")
        self._ne_plot.setAspectLocked(True)
        splitter.addWidget(self._ne_plot)

        self._ne_truth = self._ne_plot.plot(pen=pg.mkPen((80, 220, 120), width=2), name="Truth")
        self._ne_gps = self._ne_plot.plot(pen=None, symbol="o", symbolSize=4, symbolBrush=(255, 120, 120), name="GPS")
        self._ne_cur = self._ne_plot.plot(pen=None, symbol="t", symbolSize=12, symbolBrush=(255, 230, 80), name="UAV")

        self._d_plot = pg.PlotWidget(title="Down Position D (Truth vs GPS)")
        self._style_plot(self._d_plot)
        self._d_plot.setLabel("left", "D", units="m")
        self._d_plot.setLabel("bottom", "Time", units="s")
        splitter.addWidget(self._d_plot)

        self._d_truth = self._d_plot.plot(pen=pg.mkPen((80, 220, 120), width=2), name="D truth")
        self._d_gps = self._d_plot.plot(pen=pg.mkPen((255, 170, 170), width=1, style=Qt.PenStyle.DotLine), name="D gps")

        splitter.setSizes([420, 240])

        cap = capacity
        self._buf_t = RingBuffer(cap, 1)
        self._buf_pos = RingBuffer(cap, 3)
        self._buf_gps_t = RingBuffer(max(80, cap // 8), 1)
        self._buf_gps_pos = RingBuffer(max(80, cap // 8), 3)

    @staticmethod
    def _style_plot(plot: pg.PlotWidget) -> None:
        plot.showGrid(x=True, y=True, alpha=0.25)
        plot.addLegend(offset=(8, 8), labelTextColor="#ddd")
        item = cast(pg.PlotItem, plot.getPlotItem())
        item.setClipToView(True)
        item.setDownsampling(auto=True, mode="peak")

    def push_truth(self, t: float, pos_ned: np.ndarray) -> None:
        self._buf_t.push(np.array([t], dtype=float))
        self._buf_pos.push(pos_ned)

    def push_gps(self, t: float, pos_ned: np.ndarray) -> None:
        self._buf_gps_t.push(np.array([t], dtype=float))
        self._buf_gps_pos.push(pos_ned)

    def refresh(self) -> None:
        t = self._buf_t.get_array()
        p = self._buf_pos.get_array()
        if len(p) > 1:
            self._ne_truth.setData(p[:, 1], p[:, 0])
            self._ne_cur.setData([p[-1, 1]], [p[-1, 0]])
            self._d_truth.setData(t[:, 0], p[:, 2])

            e_min, e_max = float(np.min(p[:, 1])), float(np.max(p[:, 1]))
            n_min, n_max = float(np.min(p[:, 0])), float(np.max(p[:, 0]))
            pad_e = max(5.0, 0.15 * (e_max - e_min + 1e-6))
            pad_n = max(5.0, 0.15 * (n_max - n_min + 1e-6))
            self._ne_plot.setXRange(e_min - pad_e, e_max + pad_e)
            self._ne_plot.setYRange(n_min - pad_n, n_max + pad_n)

        tg = self._buf_gps_t.get_array()
        pgps = self._buf_gps_pos.get_array()
        if len(pgps) > 0:
            self._ne_gps.setData(pgps[:, 1], pgps[:, 0])
            self._d_gps.setData(tg[:, 0], pgps[:, 2])


class TimeseriesWidget(QWidget):
    """Đồ thị states/sensors tối giản và đủ dùng cho đánh giá thuật toán."""

    def __init__(self, capacity: int = 1200, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        self._tabs = QTabWidget()
        self._tabs.setDocumentMode(True)
        layout.addWidget(self._tabs)

        cap = capacity
        gps_cap = max(80, cap // 8)
        mag_cap = max(100, cap // 6)

        self._setup_truth_tab(cap)
        self._setup_sensor_tab(cap, gps_cap, mag_cap)

    @staticmethod
    def _style_plot(plot: pg.PlotWidget) -> None:
        plot.showGrid(x=True, y=True, alpha=0.25)
        plot.addLegend(offset=(8, 8), labelTextColor="#ddd")
        item = cast(pg.PlotItem, plot.getPlotItem())
        item.setClipToView(True)
        item.setDownsampling(auto=True, mode="peak")

    def _setup_truth_tab(self, cap: int) -> None:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)

        rows = QSplitter(Qt.Orientation.Vertical)
        layout.addWidget(rows)

        top = QSplitter(Qt.Orientation.Horizontal)
        self._att_plot = pg.PlotWidget(title="Attitude Truth (deg)")
        self._style_plot(self._att_plot)
        top.addWidget(self._att_plot)

        self._omega_plot = pg.PlotWidget(title="Angular Rate Truth (rad/s)")
        self._style_plot(self._omega_plot)
        top.addWidget(self._omega_plot)
        rows.addWidget(top)

        bottom = QSplitter(Qt.Orientation.Horizontal)
        self._pos_plot = pg.PlotWidget(title="Position Truth (N/E/D)")
        self._style_plot(self._pos_plot)
        bottom.addWidget(self._pos_plot)

        self._vel_plot = pg.PlotWidget(title="Velocity Truth (Vn/Ve/Vd)")
        self._style_plot(self._vel_plot)
        bottom.addWidget(self._vel_plot)
        rows.addWidget(bottom)
        rows.setSizes([310, 310])

        self._tabs.addTab(panel, "Truth States")

        self._truth_t = RingBuffer(cap, 1)
        self._att_truth = RingBuffer(cap, 3)
        self._omega_truth = RingBuffer(cap, 3)
        self._pos_truth = RingBuffer(cap, 3)
        self._vel_truth = RingBuffer(cap, 3)

        self._att_curves = [
            self._att_plot.plot(pen=pg.mkPen(COLOR_T[0], width=1.8), name="roll"),
            self._att_plot.plot(pen=pg.mkPen(COLOR_T[1], width=1.8), name="pitch"),
            self._att_plot.plot(pen=pg.mkPen(COLOR_YAW, width=1.8), name="yaw"),
        ]
        self._omega_curves = [
            self._omega_plot.plot(pen=pg.mkPen(COLOR_T[0], width=1.8), name="wx"),
            self._omega_plot.plot(pen=pg.mkPen(COLOR_T[1], width=1.8), name="wy"),
            self._omega_plot.plot(pen=pg.mkPen(COLOR_T[2], width=1.8), name="wz"),
        ]
        self._pos_curves = [
            self._pos_plot.plot(pen=pg.mkPen(COLOR_T[0], width=1.8), name="N"),
            self._pos_plot.plot(pen=pg.mkPen(COLOR_T[1], width=1.8), name="E"),
            self._pos_plot.plot(pen=pg.mkPen(COLOR_T[2], width=1.8), name="D"),
        ]
        self._vel_curves = [
            self._vel_plot.plot(pen=pg.mkPen(COLOR_T[0], width=1.8), name="Vn"),
            self._vel_plot.plot(pen=pg.mkPen(COLOR_T[1], width=1.8), name="Ve"),
            self._vel_plot.plot(pen=pg.mkPen(COLOR_T[2], width=1.8), name="Vd"),
        ]

    def _setup_sensor_tab(self, cap: int, gps_cap: int, mag_cap: int) -> None:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)

        rows = QSplitter(Qt.Orientation.Vertical)
        layout.addWidget(rows)

        top = QSplitter(Qt.Orientation.Horizontal)
        self._gyro_plot = pg.PlotWidget(title="IMU Gyro (truth vs meas)")
        self._style_plot(self._gyro_plot)
        top.addWidget(self._gyro_plot)

        self._accel_plot = pg.PlotWidget(title="IMU Accel (truth vs meas)")
        self._style_plot(self._accel_plot)
        top.addWidget(self._accel_plot)
        rows.addWidget(top)

        bottom = QSplitter(Qt.Orientation.Horizontal)
        self._gps_plot = pg.PlotWidget(title="GPS Position (truth vs meas)")
        self._style_plot(self._gps_plot)
        bottom.addWidget(self._gps_plot)

        self._mag_heading_plot = pg.PlotWidget(title="Mag Heading (truth vs meas)")
        self._style_plot(self._mag_heading_plot)
        bottom.addWidget(self._mag_heading_plot)
        rows.addWidget(bottom)
        rows.setSizes([310, 310])

        self._tabs.addTab(panel, "Measurements")

        self._imu_t = RingBuffer(cap, 1)
        self._gyro_truth = RingBuffer(cap, 3)
        self._gyro_meas = RingBuffer(cap, 3)
        self._acc_truth = RingBuffer(cap, 3)
        self._acc_meas = RingBuffer(cap, 3)

        self._gps_t = RingBuffer(gps_cap, 1)
        self._gps_truth = RingBuffer(gps_cap, 3)
        self._gps_meas = RingBuffer(gps_cap, 3)

        self._mag_t = RingBuffer(mag_cap, 1)
        self._mag_heading_truth = RingBuffer(mag_cap, 1)
        self._mag_heading_meas = RingBuffer(mag_cap, 1)

        self._gyro_t_curves = [
            self._gyro_plot.plot(pen=pg.mkPen(COLOR_T[0], width=1.5), name="wx t"),
            self._gyro_plot.plot(pen=pg.mkPen(COLOR_T[1], width=1.5), name="wy t"),
            self._gyro_plot.plot(pen=pg.mkPen(COLOR_T[2], width=1.5), name="wz t"),
        ]
        self._gyro_m_curves = [
            self._gyro_plot.plot(pen=pg.mkPen(COLOR_M[0], width=1, style=Qt.PenStyle.DotLine), name="wx m"),
            self._gyro_plot.plot(pen=pg.mkPen(COLOR_M[1], width=1, style=Qt.PenStyle.DotLine), name="wy m"),
            self._gyro_plot.plot(pen=pg.mkPen(COLOR_M[2], width=1, style=Qt.PenStyle.DotLine), name="wz m"),
        ]

        self._acc_t_curves = [
            self._accel_plot.plot(pen=pg.mkPen(COLOR_T[0], width=1.5), name="ax t"),
            self._accel_plot.plot(pen=pg.mkPen(COLOR_T[1], width=1.5), name="ay t"),
            self._accel_plot.plot(pen=pg.mkPen(COLOR_T[2], width=1.5), name="az t"),
        ]
        self._acc_m_curves = [
            self._accel_plot.plot(pen=pg.mkPen(COLOR_M[0], width=1, style=Qt.PenStyle.DotLine), name="ax m"),
            self._accel_plot.plot(pen=pg.mkPen(COLOR_M[1], width=1, style=Qt.PenStyle.DotLine), name="ay m"),
            self._accel_plot.plot(pen=pg.mkPen(COLOR_M[2], width=1, style=Qt.PenStyle.DotLine), name="az m"),
        ]

        self._gps_t_curves = [
            self._gps_plot.plot(pen=pg.mkPen(COLOR_T[0], width=1.5), name="N t"),
            self._gps_plot.plot(pen=pg.mkPen(COLOR_T[1], width=1.5), name="E t"),
            self._gps_plot.plot(pen=pg.mkPen(COLOR_T[2], width=1.5), name="D t"),
        ]
        self._gps_m_curves = [
            self._gps_plot.plot(pen=pg.mkPen(COLOR_M[0], width=1, style=Qt.PenStyle.DotLine), name="N m"),
            self._gps_plot.plot(pen=pg.mkPen(COLOR_M[1], width=1, style=Qt.PenStyle.DotLine), name="E m"),
            self._gps_plot.plot(pen=pg.mkPen(COLOR_M[2], width=1, style=Qt.PenStyle.DotLine), name="D m"),
        ]

        self._mag_heading_t_curve = self._mag_heading_plot.plot(
            pen=pg.mkPen(COLOR_YAW, width=1.8), name="heading t"
        )
        self._mag_heading_m_curve = self._mag_heading_plot.plot(
            pen=pg.mkPen(COLOR_YAW_M, width=1.2, style=Qt.PenStyle.DotLine), name="heading m"
        )

    @staticmethod
    def _wrap_deg_180(x: np.ndarray) -> np.ndarray:
        return ((x + 180.0) % 360.0) - 180.0

    # ------------ API push ------------
    def push_truth_state(self, t: float, euler_deg: np.ndarray, omega_body: np.ndarray,
                         pos_ned: np.ndarray, vel_ned: np.ndarray) -> None:
        self._truth_t.push(np.array([t], dtype=float))
        self._att_truth.push(euler_deg)
        self._omega_truth.push(omega_body)
        self._pos_truth.push(pos_ned)
        self._vel_truth.push(vel_ned)

    def push_imu(self, t: float, gyro_truth: np.ndarray, gyro_meas: np.ndarray,
                 accel_truth: np.ndarray, accel_meas: np.ndarray) -> None:
        self._imu_t.push(np.array([t], dtype=float))
        self._gyro_truth.push(gyro_truth)
        self._gyro_meas.push(gyro_meas)
        self._acc_truth.push(accel_truth)
        self._acc_meas.push(accel_meas)

    def push_gps(self, t: float, pos_truth: np.ndarray, pos_meas: np.ndarray) -> None:
        self._gps_t.push(np.array([t], dtype=float))
        self._gps_truth.push(pos_truth)
        self._gps_meas.push(pos_meas)

    def push_mag_heading(self, t: float, yaw_truth_deg: float, mag_meas_xyz: np.ndarray) -> None:
        heading_meas = float(np.degrees(np.arctan2(mag_meas_xyz[1], mag_meas_xyz[0])))
        heading_meas = float(self._wrap_deg_180(np.array([heading_meas]))[0])
        heading_truth = float(self._wrap_deg_180(np.array([yaw_truth_deg]))[0])

        self._mag_t.push(np.array([t], dtype=float))
        self._mag_heading_truth.push(np.array([heading_truth], dtype=float))
        self._mag_heading_meas.push(np.array([heading_meas], dtype=float))

    # ------------ refresh ------------
    def refresh_active_tab(self) -> None:
        if self._tabs.currentIndex() == 0:
            self._refresh_truth_tab()
        else:
            self._refresh_sensor_tab()

    def refresh(self) -> None:
        self._refresh_truth_tab()
        self._refresh_sensor_tab()

    def _refresh_truth_tab(self) -> None:
        t = self._truth_t.get_array()
        if len(t) < 2:
            return

        att = self._att_truth.get_array()
        omg = self._omega_truth.get_array()
        pos = self._pos_truth.get_array()
        vel = self._vel_truth.get_array()

        for i in range(3):
            self._att_curves[i].setData(t[:, 0], att[:, i])
            self._omega_curves[i].setData(t[:, 0], omg[:, i])
            self._pos_curves[i].setData(t[:, 0], pos[:, i])
            self._vel_curves[i].setData(t[:, 0], vel[:, i])

    def _refresh_sensor_tab(self) -> None:
        t = self._imu_t.get_array()
        if len(t) > 1:
            gt = self._gyro_truth.get_array()
            gm = self._gyro_meas.get_array()
            at = self._acc_truth.get_array()
            am = self._acc_meas.get_array()
            for i in range(3):
                self._gyro_t_curves[i].setData(t[:, 0], gt[:, i])
                self._gyro_m_curves[i].setData(t[:, 0], gm[:, i])
                self._acc_t_curves[i].setData(t[:, 0], at[:, i])
                self._acc_m_curves[i].setData(t[:, 0], am[:, i])

        tg = self._gps_t.get_array()
        if len(tg) > 0:
            pg_t = self._gps_truth.get_array()
            pg_m = self._gps_meas.get_array()
            for i in range(3):
                self._gps_t_curves[i].setData(tg[:, 0], pg_t[:, i])
                self._gps_m_curves[i].setData(tg[:, 0], pg_m[:, i])

        tm = self._mag_t.get_array()
        if len(tm) > 0:
            ht = self._mag_heading_truth.get_array()
            hm = self._mag_heading_meas.get_array()
            self._mag_heading_t_curve.setData(tm[:, 0], ht[:, 0])
            self._mag_heading_m_curve.setData(tm[:, 0], hm[:, 0])
