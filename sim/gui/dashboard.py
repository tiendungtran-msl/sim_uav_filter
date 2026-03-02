"""sim.gui.dashboard — Dashboard realtime tối ưu cho quan sát bay UAV.

Thiết kế layout ưu tiên khả năng đọc đồ thị:
- Khối trên: Attitude + Data Table.
- Khối dưới: Tab đồ thị toàn chiều ngang, giúp mỗi plot cao và dễ quan sát.
"""
import numpy as np
from typing import Optional
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QSplitter, QStatusBar, QTabWidget,
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QCloseEvent

from sim.config import SimConfig, SensorConfig
from sim.timebase import Timebase
from sim.dynamics.rigid_body_6dof import UAV6DOF
from sim.dynamics.atmosphere import Atmosphere
from sim.scenario.phase_scheduler import PhaseScheduler
from sim.sensors.imu import ImuModel
from sim.sensors.gps import GpsModel
from sim.sensors.baro import BaroModel
from sim.sensors.mag import MagModel
from sim.io.schema import TruthState, ImuMeas, SimFrame

from .widgets_attitude import AttitudeWidget
from .widgets_plots import TrajectoryWidget, TimeseriesWidget
from .widgets_table import DataTableWidget


class SimDashboard(QMainWindow):
    """Cửa sổ GUI chính — chạy simulation realtime + hiển thị."""

    def __init__(self, cfg: Optional[SimConfig] = None, scfg: Optional[SensorConfig] = None,
                 seed: int = 42, total_time: float = 60.0):
        super().__init__()
        self.setWindowTitle("UAV Flight Observer — Realtime Dashboard")
        self.setStyleSheet("background-color: #1a1a1f; color: #ccc;")
        self.resize(1520, 960)

        # --- Config ---
        self.cfg  = cfg  or SimConfig(seed=seed, total_time=total_time)
        self.scfg = scfg or SensorConfig()

        # --- Seed & RNG ---
        self.rng = np.random.default_rng(self.cfg.seed)

        # --- Simulation objects ---
        self.timebase  = Timebase(self.cfg.dt, realtime_factor=1.0)
        self.uav       = UAV6DOF(self.cfg, self.rng)
        self.atmo      = Atmosphere(self.cfg.dt, self.rng)
        self.scheduler = PhaseScheduler(self.cfg, self.rng)
        self.imu_model = ImuModel(self.cfg, self.scfg, self.rng)
        self.gps_model = GpsModel(self.cfg, self.scfg, self.rng)
        self.baro_model = BaroModel(self.cfg, self.scfg, self.rng)
        self.mag_model = MagModel(self.cfg, self.scfg, self.rng)

        # Đặt UAV ở độ cao ban đầu (warmup phase)
        warmup_alt = self.scheduler.phases[0].params.get("alt", 50.0)
        self.uav.p[2] = -warmup_alt   # NED: alt → D = -alt

        # Bước giảm tải: chỉ push GUI data mỗi N IMU steps
        self._gui_push_interval = max(1, int(0.04 / self.cfg.dt))
        self._step_count = 0
        self._total_steps = int(self.cfg.total_time / self.cfg.dt)
        self._steps_per_timer = max(1, int(self.cfg.gui_update_ms / 1000.0 / self.cfg.dt))

        # --- Xây GUI ---
        self._build_ui()

        # --- Timer ---
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_timer)

    # ------------------------------------------------------------------
    # Build UI
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(4)

        vertical_split = QSplitter(Qt.Orientation.Vertical)
        vertical_split.setChildrenCollapsible(False)
        main_layout.addWidget(vertical_split)

        # ---- Top: Attitude + Table ----
        top_panel = QWidget()
        top_layout = QVBoxLayout(top_panel)
        top_layout.setContentsMargins(0, 0, 0, 0)

        top_splitter = QSplitter(Qt.Orientation.Horizontal)
        top_splitter.setChildrenCollapsible(False)

        self.attitude_widget = AttitudeWidget()
        top_splitter.addWidget(self.attitude_widget)

        self.table_widget = DataTableWidget()
        top_splitter.addWidget(self.table_widget)
        top_splitter.setSizes([860, 620])

        top_layout.addWidget(top_splitter)
        vertical_split.addWidget(top_panel)

        # ---- Bottom: Plot tabs lớn ----
        self._plot_tabs = QTabWidget()
        self._plot_tabs.setDocumentMode(True)
        self._plot_tabs.setTabPosition(QTabWidget.TabPosition.North)

        self.trajectory_widget = TrajectoryWidget(capacity=4000)
        self.timeseries_widget = TimeseriesWidget(capacity=1200)

        self._plot_tabs.addTab(self.trajectory_widget, "Trajectory")
        self._plot_tabs.addTab(self.timeseries_widget, "States & Sensors")
        vertical_split.addWidget(self._plot_tabs)

        vertical_split.setSizes([300, 620])

        # ---- Status bar ----
        self._status = QStatusBar()
        self._status.setStyleSheet("color: #888; font-size: 11px;")
        self.setStatusBar(self._status)
        self._status.showMessage("Ready — realtime flight observer")

    # ------------------------------------------------------------------
    # Start / Stop
    # ------------------------------------------------------------------

    def start_simulation(self) -> None:
        """Khởi chạy simulation loop."""
        self.timebase.start()
        self._timer.start(self.cfg.gui_update_ms)
        self._status.showMessage("Simulation running...")

    def stop_simulation(self) -> None:
        """Dừng simulation."""
        self._timer.stop()
        self.timebase.stop()
        self._status.showMessage(f"Simulation stopped at t={self.uav.t:.2f}s")

    # ------------------------------------------------------------------
    # Timer callback: chạy batch IMU steps + cập nhật GUI
    # ------------------------------------------------------------------

    def _on_timer(self) -> None:
        """Mỗi lần timer: chạy nhiều bước IMU rồi refresh GUI."""
        last_frame = None
        for _ in range(self._steps_per_timer):
            if self._step_count >= self._total_steps:
                self.stop_simulation()
                return

            frame = self._sim_step()
            self._step_count += 1

            if self._step_count % self._gui_push_interval == 0:
                self._push_to_gui(frame)
                last_frame = frame

        # Refresh plots
        if last_frame is not None:
            self.attitude_widget.update_attitude(last_frame.truth.quat)
            self.attitude_widget.update_label(last_frame.truth.euler_deg)
            self.table_widget.update_from_frame(last_frame)
            if self._plot_tabs.currentIndex() == 0:
                self.trajectory_widget.refresh()
            else:
                self.timeseries_widget.refresh_active_tab()

            # Status bar
            pct = self._step_count / max(self._total_steps, 1) * 100
            self._status.showMessage(
                f"t={last_frame.truth.t:.2f}s  |  "
                f"phase: {last_frame.phase_name}  |  "
                f"alt: {last_frame.truth.pos_ned[2]*-1:.1f}m  |  "
                f"tab: {self._plot_tabs.tabText(self._plot_tabs.currentIndex())}  |  "
                f"progress: {pct:.0f}%"
            )

    # ------------------------------------------------------------------
    # Một bước simulation
    # ------------------------------------------------------------------

    def _sim_step(self) -> SimFrame:
        """Chạy 1 bước IMU (dt) và trả SimFrame."""
        # 1) Atmosphere
        self.uav.wind_ned = self.atmo.step()

        # 2) Scenario → đặt thrust/torque
        gps_active = self.scheduler.apply(self.uav, self.atmo)

        # 3) Dynamics
        self.uav.step()

        # 4) Sensors
        raw_imu = self.imu_model.sample(self.uav)
        gps_meas = self.gps_model.update(self.uav, gps_active)
        baro_meas = self.baro_model.update(self.uav)
        mag_meas = self.mag_model.update(self.uav)
        mag_truth = self.mag_model.truth_body(self.uav)

        # 5) Đóng gói SimFrame
        truth = TruthState(
            t=self.uav.t,
            pos_ned=self.uav.p.copy(),
            vel_ned=self.uav.v.copy(),
            quat=self.uav.q.copy(),
            omega_body=self.uav.omega.copy(),
            specific_force_body=self.uav.specific_force_body.copy(),
            mag_body=mag_truth.copy(),
            euler_deg=self.uav.euler_deg.copy(),
            gyro_bias=raw_imu.gyro_bias.copy(),
            accel_bias=raw_imu.accel_bias.copy(),
        )
        imu = ImuMeas(t=raw_imu.t, gyro=raw_imu.gyro_meas.copy(),
                       accel=raw_imu.accel_meas.copy())

        return SimFrame(
            truth=truth,
            imu=imu,
            gps=gps_meas,
            baro=baro_meas,
            mag=mag_meas,
            phase_name=self.scheduler.current_phase_name,
            gps_active=gps_active,
        )

    def _push_to_gui(self, frame: SimFrame) -> None:
        """Đẩy dữ liệu từ SimFrame vào các widget (ring buffers)."""
        tr = frame.truth

        self.trajectory_widget.push_truth(tr.t, tr.pos_ned)
        self.timeseries_widget.push_truth_state(
            tr.t,
            euler_deg=tr.euler_deg,
            omega_body=tr.omega_body,
            pos_ned=tr.pos_ned,
            vel_ned=tr.vel_ned,
        )

        # IMU timeseries
        self.timeseries_widget.push_imu(
            tr.t,
            gyro_truth=tr.omega_body,
            gyro_meas=frame.imu.gyro,
            accel_truth=tr.specific_force_body,
            accel_meas=frame.imu.accel,
        )

        # GPS
        if frame.gps is not None and frame.gps.valid:
            self.trajectory_widget.push_gps(tr.t, frame.gps.pos_ned)
            self.timeseries_widget.push_gps(
                tr.t,
                pos_truth=tr.pos_ned,
                pos_meas=frame.gps.pos_ned,
            )

        if frame.mag is not None and frame.mag.valid:
            self.timeseries_widget.push_mag_heading(
                tr.t,
                yaw_truth_deg=float(tr.euler_deg[2]),
                mag_meas_xyz=frame.mag.mag_body,
            )

    # ------------------------------------------------------------------
    # Override close
    # ------------------------------------------------------------------

    def closeEvent(self, a0: QCloseEvent | None) -> None:
        self.stop_simulation()
        super().closeEvent(a0)
