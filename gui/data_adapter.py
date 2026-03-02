"""gui.data_adapter — Đọc và align dữ liệu truth/sensor/estimated cho GUI.

Module này là cầu nối giữa file CSV (output từ simulator + ESKF)
và các tab GUI. GUI KHÔNG chứa logic filter — chỉ đọc dữ liệu và vẽ.

Dữ liệu đầu vào:
- sensor_log.csv:  truth + IMU/GPS/MAG/Baro measurements (từ simulator)
- estimated.csv:   state ước lượng từ ESKF C++ replay tool
- innovations.csv: innovation records từ ESKF

Chức năng:
- Parse CSV
- Align theo timestamp (interpolate nếu cần)
- Cung cấp numpy arrays cho plotting
"""
import numpy as np
import csv
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class TruthData:
    """Dữ liệu ground truth từ simulator."""
    t: np.ndarray = field(default_factory=lambda: np.empty(0))
    pos: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))     # NED (m)
    vel: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))     # NED (m/s)
    quat: np.ndarray = field(default_factory=lambda: np.empty((0, 4)))    # [w,x,y,z]
    euler_deg: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))  # [roll,pitch,yaw] (deg)
    bg: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))      # gyro bias (rad/s)
    ba: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))      # accel bias (m/s²)


@dataclass
class MeasData:
    """Dữ liệu measurement từ sensor."""
    # GPS (chỉ ở thời điểm có mẫu)
    gps_t: np.ndarray = field(default_factory=lambda: np.empty(0))
    gps_pos: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    gps_vel: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))


@dataclass
class EstimatedData:
    """Dữ liệu ước lượng từ ESKF."""
    t: np.ndarray = field(default_factory=lambda: np.empty(0))
    pos: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    vel: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    quat: np.ndarray = field(default_factory=lambda: np.empty((0, 4)))
    euler_rad: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    euler_deg: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    bg: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    ba: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    # Truth (aligned — cùng timestamp với estimated)
    truth_pos: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    truth_vel: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    truth_quat: np.ndarray = field(default_factory=lambda: np.empty((0, 4)))
    truth_euler_rad: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    truth_euler_deg: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    truth_bg: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    truth_ba: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))


@dataclass
class InnovationData:
    """Dữ liệu innovation từ ESKF."""
    # GPS innovations
    gps_t: np.ndarray = field(default_factory=lambda: np.empty(0))
    gps_innov: np.ndarray = field(default_factory=lambda: np.empty((0, 6)))
    gps_nis: np.ndarray = field(default_factory=lambda: np.empty(0))
    gps_accepted: np.ndarray = field(default_factory=lambda: np.empty(0))

    # MAG innovations
    mag_t: np.ndarray = field(default_factory=lambda: np.empty(0))
    mag_innov: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    mag_nis: np.ndarray = field(default_factory=lambda: np.empty(0))
    mag_accepted: np.ndarray = field(default_factory=lambda: np.empty(0))

    # Baro innovations
    baro_t: np.ndarray = field(default_factory=lambda: np.empty(0))
    baro_innov: np.ndarray = field(default_factory=lambda: np.empty(0))
    baro_nis: np.ndarray = field(default_factory=lambda: np.empty(0))
    baro_accepted: np.ndarray = field(default_factory=lambda: np.empty(0))


class DataAdapter:
    """Đọc và align dữ liệu từ CSV files cho GUI.

    Sử dụng:
        adapter = DataAdapter()
        adapter.load_sensor_log("sensor_log.csv")
        adapter.load_estimated("estimated.csv")
        adapter.load_innovations("innovations.csv")
        # Bây giờ truy cập: adapter.truth, adapter.meas, adapter.est, adapter.innov
    """

    def __init__(self):
        self.truth = TruthData()
        self.meas = MeasData()
        self.est = EstimatedData()
        self.innov = InnovationData()

    def load_sensor_log(self, path: str) -> bool:
        """Đọc sensor_log.csv — chứa truth + measurements.

        Trích xuất:
        - Truth: pos, vel, quat, bg, ba (mỗi tick)
        - GPS measurements (chỉ khi gps_valid=1)
        """
        try:
            with open(path, 'r') as f:
                reader = csv.DictReader(f)
                rows = list(reader)
        except (FileNotFoundError, IOError):
            return False

        n = len(rows)
        if n == 0:
            return False

        # Preallocate arrays
        t = np.zeros(n)
        pos = np.zeros((n, 3))
        vel = np.zeros((n, 3))
        quat = np.zeros((n, 4))
        bg = np.zeros((n, 3))
        ba = np.zeros((n, 3))

        gps_t_list = []
        gps_pos_list = []
        gps_vel_list = []

        for i, row in enumerate(rows):
            t[i] = float(row['t'])
            pos[i] = [float(row['truth_pN']), float(row['truth_pE']), float(row['truth_pD'])]
            vel[i] = [float(row['truth_vN']), float(row['truth_vE']), float(row['truth_vD'])]
            quat[i] = [float(row['truth_qw']), float(row['truth_qx']),
                        float(row['truth_qy']), float(row['truth_qz'])]
            bg[i] = [float(row['truth_bgx']), float(row['truth_bgy']), float(row['truth_bgz'])]
            ba[i] = [float(row['truth_bax']), float(row['truth_bay']), float(row['truth_baz'])]

            # GPS measurements
            if float(row['gps_valid']) > 0.5:
                gps_t_list.append(float(row.get('gps_ts', row['t'])))
                gps_pos_list.append([float(row['gps_pN']), float(row['gps_pE']), float(row['gps_pD'])])
                gps_vel_list.append([float(row['gps_vN']), float(row['gps_vE']), float(row['gps_vD'])])

        # Tính Euler angles từ quaternion (deg)
        euler_deg = np.zeros((n, 3))
        for i in range(n):
            euler_deg[i] = _quat_to_euler_deg(quat[i])

        self.truth = TruthData(t=t, pos=pos, vel=vel, quat=quat, euler_deg=euler_deg, bg=bg, ba=ba)
        self.meas = MeasData(
            gps_t=np.array(gps_t_list) if gps_t_list else np.empty(0),
            gps_pos=np.array(gps_pos_list) if gps_pos_list else np.empty((0, 3)),
            gps_vel=np.array(gps_vel_list) if gps_vel_list else np.empty((0, 3)),
        )
        return True

    def load_estimated(self, path: str) -> bool:
        """Đọc estimated.csv — output từ ESKF replay tool.

        Chứa cả estimated state và truth (aligned cùng timestamp).
        """
        try:
            with open(path, 'r') as f:
                reader = csv.DictReader(f)
                rows = list(reader)
        except (FileNotFoundError, IOError):
            return False

        n = len(rows)
        if n == 0:
            return False

        t = np.zeros(n)
        pos = np.zeros((n, 3))
        vel = np.zeros((n, 3))
        quat = np.zeros((n, 4))
        euler_rad = np.zeros((n, 3))
        bg = np.zeros((n, 3))
        ba = np.zeros((n, 3))

        truth_pos = np.zeros((n, 3))
        truth_vel = np.zeros((n, 3))
        truth_quat = np.zeros((n, 4))
        truth_euler_rad = np.zeros((n, 3))
        truth_bg = np.zeros((n, 3))
        truth_ba = np.zeros((n, 3))

        for i, row in enumerate(rows):
            t[i] = float(row['t'])

            pos[i] = [float(row['p_est_N']), float(row['p_est_E']), float(row['p_est_D'])]
            vel[i] = [float(row['v_est_N']), float(row['v_est_E']), float(row['v_est_D'])]
            quat[i] = [float(row['q_est_w']), float(row['q_est_x']),
                        float(row['q_est_y']), float(row['q_est_z'])]
            euler_rad[i] = [float(row['roll_est']), float(row['pitch_est']), float(row['yaw_est'])]
            bg[i] = [float(row['bg_est_x']), float(row['bg_est_y']), float(row['bg_est_z'])]
            ba[i] = [float(row['ba_est_x']), float(row['ba_est_y']), float(row['ba_est_z'])]

            truth_pos[i] = [float(row['p_true_N']), float(row['p_true_E']), float(row['p_true_D'])]
            truth_vel[i] = [float(row['v_true_N']), float(row['v_true_E']), float(row['v_true_D'])]
            truth_quat[i] = [float(row['q_true_w']), float(row['q_true_x']),
                              float(row['q_true_y']), float(row['q_true_z'])]
            truth_euler_rad[i] = [float(row['roll_true']), float(row['pitch_true']), float(row['yaw_true'])]
            truth_bg[i] = [float(row['bg_true_x']), float(row['bg_true_y']), float(row['bg_true_z'])]
            truth_ba[i] = [float(row['ba_true_x']), float(row['ba_true_y']), float(row['ba_true_z'])]

        euler_deg = np.degrees(euler_rad)
        truth_euler_deg = np.degrees(truth_euler_rad)

        self.est = EstimatedData(
            t=t, pos=pos, vel=vel, quat=quat,
            euler_rad=euler_rad, euler_deg=euler_deg,
            bg=bg, ba=ba,
            truth_pos=truth_pos, truth_vel=truth_vel, truth_quat=truth_quat,
            truth_euler_rad=truth_euler_rad, truth_euler_deg=truth_euler_deg,
            truth_bg=truth_bg, truth_ba=truth_ba,
        )
        return True

    def load_innovations(self, path: str) -> bool:
        """Đọc innovations.csv — innovation records từ ESKF.

        Tách theo sensor: GPS, MAG, BARO.
        """
        try:
            with open(path, 'r') as f:
                reader = csv.DictReader(f)
                rows = list(reader)
        except (FileNotFoundError, IOError):
            return False

        # Tách theo sensor
        gps_rows = [r for r in rows if r['sensor'] == 'GPS']
        mag_rows = [r for r in rows if r['sensor'] == 'MAG']
        baro_rows = [r for r in rows if r['sensor'] == 'BARO']

        # GPS
        if gps_rows:
            self.innov.gps_t = np.array([float(r['t']) for r in gps_rows])
            self.innov.gps_innov = np.array([
                [float(r[f'innov_{j}']) for j in range(6)] for r in gps_rows
            ])
            self.innov.gps_nis = np.array([float(r['nis']) for r in gps_rows])
            self.innov.gps_accepted = np.array([int(r['accepted']) for r in gps_rows])

        # MAG
        if mag_rows:
            self.innov.mag_t = np.array([float(r['t']) for r in mag_rows])
            self.innov.mag_innov = np.array([
                [float(r[f'innov_{j}']) for j in range(3)] for r in mag_rows
            ])
            self.innov.mag_nis = np.array([float(r['nis']) for r in mag_rows])
            self.innov.mag_accepted = np.array([int(r['accepted']) for r in mag_rows])

        # BARO
        if baro_rows:
            self.innov.baro_t = np.array([float(r['t']) for r in baro_rows])
            self.innov.baro_innov = np.array([float(r['innov_0']) for r in baro_rows])
            self.innov.baro_nis = np.array([float(r['nis']) for r in baro_rows])
            self.innov.baro_accepted = np.array([int(r['accepted']) for r in baro_rows])

        return True

    def time_range(self) -> tuple:
        """Trả về (t_min, t_max) từ estimated data."""
        if len(self.est.t) > 0:
            return (self.est.t[0], self.est.t[-1])
        if len(self.truth.t) > 0:
            return (self.truth.t[0], self.truth.t[-1])
        return (0.0, 1.0)


# =====================================================================
# Hàm helper
# =====================================================================

def _quat_to_euler_deg(q: np.ndarray) -> np.ndarray:
    """Chuyển quaternion [w,x,y,z] → [roll, pitch, yaw] (degrees).

    ZYX convention, giống C++ core.
    """
    w, x, y, z = q
    sinp = 2.0 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)

    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(sinp)
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    return np.degrees(np.array([roll, pitch, yaw]))
