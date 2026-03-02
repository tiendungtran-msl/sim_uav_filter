"""sim.io.logger_csv — Ghi log CSV cho replay & phân tích offline.

Xuất 1 file CSV duy nhất chứa cả truth và measurement.
Cột được đặt tên rõ ràng, có header mô tả đơn vị & frame.

Schema CSV (mỗi hàng = 1 IMU tick):
────────────────────────────────────────────────────────────
Cột              | Đơn vị  | Frame | Ghi chú
────────────────────────────────────────────────────────────
t                | s       |       | sim time
phase            |         |       | tên phase hiện tại
truth_pN/pE/pD   | m       | NED   | vị trí thật
truth_vN/vE/vD   | m/s     | NED   | vận tốc thật
truth_qw/qx/qy/qz|        |       | quaternion body→NED
truth_wx/wy/wz   | rad/s   | body  | omega thật
truth_sfx/sfy/sfz| m/s²    | body  | specific force thật
truth_mx/my/mz   | uT      | body  | từ trường thật
truth_bgx/bgy/bgz| rad/s   | body  | gyro bias thật
truth_bax/bay/baz| m/s²    | body  | accel bias thật
imu_gx/gy/gz     | rad/s   | body  | gyro đo
imu_ax/ay/az     | m/s²    | body  | accel đo
gps_valid        | 0/1     |       | có GPS mẫu không
gps_ts           | s       |       | GPS timestamp (lúc lấy mẫu)
gps_pN/pE/pD     | m       | NED   | GPS pos (có noise)
gps_vN/vE/vD     | m/s     | NED   | GPS vel (có noise)
baro_valid       | 0/1     |       | có baro mẫu không
baro_alt         | m       |       | altitude đo (dương lên)
mag_valid        | 0/1     |       | có mag mẫu không
mag_mx/my/mz     | uT      | body  | từ trường đo
────────────────────────────────────────────────────────────
"""
import csv
import os
from typing import TextIO, Optional, Any
from .schema import SimFrame


# Thứ tự cột — phải khớp với _format_row()
COLUMNS = [
    "t", "phase",
    # Truth
    "truth_pN", "truth_pE", "truth_pD",
    "truth_vN", "truth_vE", "truth_vD",
    "truth_qw", "truth_qx", "truth_qy", "truth_qz",
    "truth_wx", "truth_wy", "truth_wz",
    "truth_sfx", "truth_sfy", "truth_sfz",
    "truth_mx", "truth_my", "truth_mz",
    "truth_bgx", "truth_bgy", "truth_bgz",
    "truth_bax", "truth_bay", "truth_baz",
    # IMU
    "imu_gx", "imu_gy", "imu_gz",
    "imu_ax", "imu_ay", "imu_az",
    # GPS
    "gps_valid", "gps_ts",
    "gps_pN", "gps_pE", "gps_pD",
    "gps_vN", "gps_vE", "gps_vD",
    # Baro
    "baro_valid", "baro_alt",
    # Mag
    "mag_valid", "mag_mx", "mag_my", "mag_mz",
]


class CsvLogger:
    """Ghi SimFrame liên tục ra file CSV.

    Cách dùng:
        logger = CsvLogger("logs/run1.csv")
        logger.open()
        for frame in sim:
            logger.write(frame)
        logger.close()
    """

    def __init__(self, path: str):
        """
        Args:
            path: Đường dẫn file CSV đầu ra. Thư mục sẽ được tạo nếu chưa có.
        """
        self.path = path
        self._file: Optional[TextIO] = None
        self._writer: Optional[Any] = None

    # ------------------------------------------------------------------
    # Context manager support
    # ------------------------------------------------------------------

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    # ------------------------------------------------------------------
    # Mở / đóng file
    # ------------------------------------------------------------------

    def open(self) -> None:
        """Mở file CSV, ghi header."""
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        self._file = open(self.path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(COLUMNS)

    def close(self) -> None:
        """Đóng file CSV."""
        if self._file:
            self._file.close()
            self._file = None

    # ------------------------------------------------------------------
    # Ghi từng frame
    # ------------------------------------------------------------------

    def write(self, frame: SimFrame) -> None:
        """Ghi 1 SimFrame thành 1 dòng CSV."""
        if self._writer is None:
            raise RuntimeError("CsvLogger chưa được open().")
        self._writer.writerow(self._format_row(frame))

    @staticmethod
    def _format_row(f: SimFrame) -> list:
        """Chuyển SimFrame → list giá trị để ghi CSV."""
        tr = f.truth
        im = f.imu

        row = [
            f"{tr.t:.6f}",
            f.phase_name,
            # Truth position
            f"{tr.pos_ned[0]:.6f}", f"{tr.pos_ned[1]:.6f}", f"{tr.pos_ned[2]:.6f}",
            # Truth velocity
            f"{tr.vel_ned[0]:.6f}", f"{tr.vel_ned[1]:.6f}", f"{tr.vel_ned[2]:.6f}",
            # Truth quaternion
            f"{tr.quat[0]:.8f}", f"{tr.quat[1]:.8f}", f"{tr.quat[2]:.8f}", f"{tr.quat[3]:.8f}",
            # Truth omega
            f"{tr.omega_body[0]:.6f}", f"{tr.omega_body[1]:.6f}", f"{tr.omega_body[2]:.6f}",
            # Truth specific force
            f"{tr.specific_force_body[0]:.6f}", f"{tr.specific_force_body[1]:.6f}", f"{tr.specific_force_body[2]:.6f}",
            # Truth magnetic field
            f"{tr.mag_body[0]:.6f}", f"{tr.mag_body[1]:.6f}", f"{tr.mag_body[2]:.6f}",
            # Truth gyro bias
            f"{tr.gyro_bias[0]:.8f}", f"{tr.gyro_bias[1]:.8f}", f"{tr.gyro_bias[2]:.8f}",
            # Truth accel bias
            f"{tr.accel_bias[0]:.8f}", f"{tr.accel_bias[1]:.8f}", f"{tr.accel_bias[2]:.8f}",
            # IMU gyro
            f"{im.gyro[0]:.6f}", f"{im.gyro[1]:.6f}", f"{im.gyro[2]:.6f}",
            # IMU accel
            f"{im.accel[0]:.6f}", f"{im.accel[1]:.6f}", f"{im.accel[2]:.6f}",
        ]

        # GPS
        if f.gps is not None and f.gps.valid:
            row += [
                "1", f"{f.gps.t_stamp:.6f}",
                f"{f.gps.pos_ned[0]:.4f}", f"{f.gps.pos_ned[1]:.4f}", f"{f.gps.pos_ned[2]:.4f}",
                f"{f.gps.vel_ned[0]:.6f}", f"{f.gps.vel_ned[1]:.6f}", f"{f.gps.vel_ned[2]:.6f}",
            ]
        else:
            row += ["0", "0", "0", "0", "0", "0", "0", "0"]

        # Baro
        if f.baro is not None and f.baro.valid:
            row += ["1", f"{f.baro.alt_m:.4f}"]
        else:
            row += ["0", "0"]

        # Magnetometer
        if f.mag is not None and f.mag.valid:
            row += [
                "1",
                f"{f.mag.mag_body[0]:.6f}",
                f"{f.mag.mag_body[1]:.6f}",
                f"{f.mag.mag_body[2]:.6f}",
            ]
        else:
            row += ["0", "0", "0", "0"]

        return row
