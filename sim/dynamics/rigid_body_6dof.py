"""sim.dynamics.rigid_body_6dof — Động lực học 6DOF thân cứng (rigid body).

State vector liên tục:
    p     : vị trí         NED  (3)
    v     : vận tốc        NED  (3)
    q     : quaternion     body→NED  (4) [w, x, y, z]
    omega : vận tốc góc    body (3)

Tích phân RK4 mỗi bước dt (400 Hz).
Control inputs: thrust_body (3) và torque_body (3) — do PhaseScheduler cài đặt.
"""
import numpy as np
from numpy.typing import NDArray

from sim.config import SimConfig
from sim.utils.math3d import (
    quat_mult, quat_normalize, quat_rotate, quat_to_euler, rot_from_quat,
)


class UAV6DOF:
    """Mô hình động lực học 6DOF UAV dạng thân cứng.

    Cách dùng:
        uav = UAV6DOF(cfg, rng)
        # Scenario đặt thrust/torque mỗi bước
        uav.thrust_body = np.array([0, 0, -m*g])
        uav.torque_body = np.zeros(3)
        uav.step()   # tiến thêm 1 dt
    """

    def __init__(self, cfg: SimConfig, rng: np.random.Generator):
        self.cfg = cfg
        self.rng = rng

        # Vector trọng lực NED (down = +)
        self.g_ned = np.array([0.0, 0.0, cfg.gravity])

        # ----- State -----
        self.p: NDArray     = np.zeros(3)                        # vị trí NED (m)
        self.v: NDArray     = np.zeros(3)                        # vận tốc NED (m/s)
        self.q: NDArray     = np.array([1.0, 0.0, 0.0, 0.0])   # quaternion body→NED
        self.omega: NDArray = np.zeros(3)                        # vận tốc góc body (rad/s)

        # ----- Control inputs (scenario đặt mỗi tick) -----
        self.thrust_body: NDArray = np.array([0.0, 0.0, 0.0])   # lực (N) trong body frame
        self.torque_body: NDArray = np.zeros(3)                  # moment (N·m) trong body

        # Gió (NED) — atmosphere module sẽ cập nhật
        self.wind_ned: NDArray = np.zeros(3)

        self.t: float = 0.0   # thời gian mô phỏng (s)

    # ------------------------------------------------------------------
    # Tích phân RK4
    # ------------------------------------------------------------------

    def step(self) -> None:
        """Tiến 1 bước dt bằng tích phân Runge-Kutta bậc 4."""
        dt = self.cfg.dt
        s = self._pack()
        k1 = self._deriv(s)
        k2 = self._deriv(s + 0.5 * dt * k1)
        k3 = self._deriv(s + 0.5 * dt * k2)
        k4 = self._deriv(s + dt * k3)
        s_new = s + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4)
        self._unpack(s_new)
        self.q = quat_normalize(self.q)
        self.t += dt

    # ------------------------------------------------------------------
    # Pack / unpack state vector (13-dim)
    # ------------------------------------------------------------------

    def _pack(self) -> NDArray:
        return np.concatenate([self.p, self.v, self.q, self.omega])

    def _unpack(self, s: NDArray) -> None:
        self.p     = s[0:3].copy()
        self.v     = s[3:6].copy()
        self.q     = s[6:10].copy()
        self.omega = s[10:13].copy()

    # ------------------------------------------------------------------
    # Phương trình vi phân
    # ------------------------------------------------------------------

    def _deriv(self, s: NDArray) -> NDArray:
        """Tính đạo hàm state vector (RHS phương trình chuyển động)."""
        p     = s[0:3]
        v     = s[3:6]
        q     = quat_normalize(s[6:10])
        omega = s[10:13]

        # Tensor quán tính (đường chéo, cố định trong body)
        I_diag = np.array([self.cfg.Ixx, self.cfg.Iyy, self.cfg.Izz])
        I_mat  = np.diag(I_diag)

        # ---- Lực tịnh tiến ----
        # Thrust chuyển sang NED
        thrust_ned = quat_rotate(q, self.thrust_body)

        # Airspeed (tính drag theo vận tốc tương đối gió)
        v_air = v - self.wind_ned
        drag_coeffs = np.array([self.cfg.drag_xy, self.cfg.drag_xy, self.cfg.drag_z])
        drag_ned = -drag_coeffs * v_air

        # Gia tốc NED: F/m + g
        accel_ned = thrust_ned / self.cfg.mass + self.g_ned + drag_ned / self.cfg.mass

        # ---- Kinematics quaternion ----
        # dq/dt = 0.5 * q ⊗ [0, omega]
        omega_quat = np.array([0.0, omega[0], omega[1], omega[2]])
        q_dot = 0.5 * quat_mult(q, omega_quat)

        # ---- Euler rotation equation ----
        # I·dω/dt = τ − ω × (I·ω)
        I_omega = I_mat @ omega
        omega_dot = np.linalg.solve(I_mat, self.torque_body - np.cross(omega, I_omega))

        return np.concatenate([v, accel_ned, q_dot, omega_dot])

    # ------------------------------------------------------------------
    # Properties tiện dụng
    # ------------------------------------------------------------------

    @property
    def R_body_to_ned(self) -> NDArray:
        """Ma trận xoay 3×3 body → NED."""
        return rot_from_quat(self.q)

    @property
    def euler_deg(self) -> NDArray:
        """[roll, pitch, yaw] tính bằng độ."""
        return np.degrees(quat_to_euler(self.q))

    @property
    def altitude(self) -> float:
        """Độ cao = -p_D (vì NED: D hướng xuống)."""
        return -self.p[2]

    @property
    def specific_force_body(self) -> NDArray:
        """Specific force trong body frame (= accel - gravity, là cái IMU cảm nhận).

        f_body = R^T · (a_NED - g_NED)
        Tương đương thrust_body/m + drag_body/m  (không có gravity).
        """
        R = self.R_body_to_ned
        thrust_ned = R @ self.thrust_body
        v_air = self.v - self.wind_ned
        drag_coeffs = np.array([self.cfg.drag_xy, self.cfg.drag_xy, self.cfg.drag_z])
        drag_ned = -drag_coeffs * v_air
        f_ned = (thrust_ned + drag_ned) / self.cfg.mass
        return R.T @ f_ned
