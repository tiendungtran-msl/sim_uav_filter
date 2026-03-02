"""sim.gui.widgets_table — Bảng dữ liệu truth vs measurement (tối giản, khoa học).

Theo yêu cầu hiện tại, bảng chỉ giữ:
- Truth: attitude(3), omega(3), position(3), velocity(3), acceleration(3)
- Meas : IMU gyro(3), IMU accel(3), GPS position(3), Mag heading(1)
"""
import numpy as np
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem, QLabel, QHeaderView
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor, QBrush

from sim.io.schema import SimFrame

_HEADER_STYLE = "color:#e6e6e6; font-weight:700; font-size:13px;"
_BG_A = QBrush(QColor(33, 35, 44))
_BG_B = QBrush(QColor(28, 30, 38))
_BG_DIM = QBrush(QColor(43, 45, 56))
_BG_DROP = QBrush(QColor(90, 36, 36))
_TEXT = QBrush(QColor(225, 228, 233))
_TEXT_DIM = QBrush(QColor(160, 165, 175))


_ROWS = [
    # group, label, meas_source
    ("ATT", "roll (deg)", "none"),
    ("ATT", "pitch (deg)", "none"),
    ("ATT", "yaw (deg)", "mag_heading"),

    ("OMEGA", "wx (rad/s)", "imu_gyro"),
    ("OMEGA", "wy (rad/s)", "imu_gyro"),
    ("OMEGA", "wz (rad/s)", "imu_gyro"),

    ("POS", "N (m)", "gps_pos"),
    ("POS", "E (m)", "gps_pos"),
    ("POS", "D (m)", "gps_pos"),

    ("VEL", "Vn (m/s)", "none"),
    ("VEL", "Ve (m/s)", "none"),
    ("VEL", "Vd (m/s)", "none"),

    ("ACC", "ax (m/s²)", "imu_accel"),
    ("ACC", "ay (m/s²)", "imu_accel"),
    ("ACC", "az (m/s²)", "imu_accel"),
]


class DataTableWidget(QWidget):
    """Bảng dữ liệu realtime cho observer."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        self._header = QLabel("Flight Data Matrix")
        self._header.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._header.setStyleSheet(_HEADER_STYLE)
        layout.addWidget(self._header)

        self._table = QTableWidget(len(_ROWS), 4)
        self._table.setHorizontalHeaderLabels(["Group", "State", "Truth", "Meas"])
        vh = self._table.verticalHeader()
        if vh is not None:
            vh.setVisible(False)
        self._table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self._table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        self._table.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self._table.setStyleSheet(
            """
            QTableWidget {
                background-color: #1a1c23;
                color: #e0e0e0;
                gridline-color: #3b4050;
                font-family: 'DejaVu Sans Mono', monospace;
                font-size: 12px;
            }
            QHeaderView::section {
                background-color: #252a36;
                color: #f0f0f0;
                border: 1px solid #3b4050;
                padding: 4px;
                font-weight: 700;
            }
            """
        )

        hh = self._table.horizontalHeader()
        if hh is not None:
            hh.setSectionResizeMode(0, QHeaderView.ResizeMode.Fixed)
            hh.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
            hh.setSectionResizeMode(2, QHeaderView.ResizeMode.Fixed)
            hh.setSectionResizeMode(3, QHeaderView.ResizeMode.Fixed)
        self._table.setColumnWidth(0, 72)
        self._table.setColumnWidth(2, 110)
        self._table.setColumnWidth(3, 110)

        vh2 = self._table.verticalHeader()
        if vh2 is not None:
            vh2.setDefaultSectionSize(26)

        for i, (grp, label, _) in enumerate(_ROWS):
            bg = _BG_A if i % 2 == 0 else _BG_B

            item_g = QTableWidgetItem(grp)
            item_g.setForeground(_TEXT_DIM)
            item_g.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            item_g.setBackground(bg)
            self._table.setItem(i, 0, item_g)

            item_s = QTableWidgetItem(label)
            item_s.setForeground(_TEXT)
            item_s.setBackground(bg)
            self._table.setItem(i, 1, item_s)

            for col in (2, 3):
                item = QTableWidgetItem("—")
                item.setForeground(_TEXT)
                item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                item.setBackground(bg)
                self._table.setItem(i, col, item)

        layout.addWidget(self._table)

    @staticmethod
    def _fmt(v: float, p: int = 3) -> str:
        return f"{v:+.{p}f}"

    @staticmethod
    def _wrap_deg_180(x: float) -> float:
        return ((x + 180.0) % 360.0) - 180.0

    @staticmethod
    def _mag_heading_deg(mag_xyz) -> float:
        return DataTableWidget._wrap_deg_180(float(np.degrees(np.arctan2(mag_xyz[1], mag_xyz[0]))))

    def update_from_frame(self, frame: SimFrame) -> None:
        tr = frame.truth
        imu = frame.imu
        gps = frame.gps
        mag = frame.mag

        truth_vals = [
            self._fmt(float(tr.euler_deg[0]), 2),
            self._fmt(float(tr.euler_deg[1]), 2),
            self._fmt(float(tr.euler_deg[2]), 2),

            self._fmt(float(tr.omega_body[0]), 4),
            self._fmt(float(tr.omega_body[1]), 4),
            self._fmt(float(tr.omega_body[2]), 4),

            self._fmt(float(tr.pos_ned[0]), 2),
            self._fmt(float(tr.pos_ned[1]), 2),
            self._fmt(float(tr.pos_ned[2]), 2),

            self._fmt(float(tr.vel_ned[0]), 3),
            self._fmt(float(tr.vel_ned[1]), 3),
            self._fmt(float(tr.vel_ned[2]), 3),

            self._fmt(float(tr.specific_force_body[0]), 3),
            self._fmt(float(tr.specific_force_body[1]), 3),
            self._fmt(float(tr.specific_force_body[2]), 3),
        ]

        gps_ok = gps is not None and gps.valid
        mag_ok = mag is not None and mag.valid

        if gps_ok and gps is not None:
            gps_pos_vals = [
                self._fmt(float(gps.pos_ned[0]), 2),
                self._fmt(float(gps.pos_ned[1]), 2),
                self._fmt(float(gps.pos_ned[2]), 2),
            ]
        else:
            gps_pos_vals = ["N/A", "N/A", "N/A"]

        if mag_ok and mag is not None:
            mag_heading = self._fmt(self._mag_heading_deg(mag.mag_body), 2)
        else:
            mag_heading = "N/A"

        meas_vals = [
            "—",
            "—",
            mag_heading,

            self._fmt(float(imu.gyro[0]), 4),
            self._fmt(float(imu.gyro[1]), 4),
            self._fmt(float(imu.gyro[2]), 4),

            gps_pos_vals[0],
            gps_pos_vals[1],
            gps_pos_vals[2],

            "—",
            "—",
            "—",

            self._fmt(float(imu.accel[0]), 3),
            self._fmt(float(imu.accel[1]), 3),
            self._fmt(float(imu.accel[2]), 3),
        ]

        for i, (_, _, src) in enumerate(_ROWS):
            bg = _BG_A if i % 2 == 0 else _BG_B

            t_item = self._table.item(i, 2)
            m_item = self._table.item(i, 3)
            if t_item is None or m_item is None:
                continue

            t_item.setText(truth_vals[i])
            m_item.setText(meas_vals[i])

            # highlight dropout sensor hàng cần đo
            if src == "gps_pos" and not gps_ok:
                m_item.setBackground(_BG_DROP)
            elif src == "mag_heading" and not mag_ok:
                m_item.setBackground(_BG_DROP)
            elif src == "none":
                m_item.setBackground(_BG_DIM)
                m_item.setForeground(_TEXT_DIM)
            else:
                m_item.setBackground(bg)
                m_item.setForeground(_TEXT)

            t_item.setBackground(bg)
            t_item.setForeground(_TEXT)

        self._header.setText(f"Flight Data Matrix — t={tr.t:.2f}s — phase: {frame.phase_name}")
