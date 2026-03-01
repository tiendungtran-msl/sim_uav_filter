"""UAV 6-DOF Rigid-Body Dynamics using unit quaternion for attitude.

State vector (continuous):
    p  : position       NED  (3)
    v  : velocity       NED  (3)
    q  : quaternion     body-to-NED  (4)  [w, x, y, z]
    b_g: gyro bias      body (3)
    b_a: accel bias     body (3)

This module only propagates physics. It has NO knowledge of the EKF.
"""
import numpy as np
from numpy.typing import NDArray
from .config import SimConfig


def quat_mult(q: NDArray, r: NDArray) -> NDArray:
    """Hamilton product q ⊗ r, where q = [w, x, y ,z]."""
    w0, x0, y0, z0 = q
    w1, x1, y1, z1 = r
    return np.array([
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1,
    ])


def quat_conj(q: NDArray) -> NDArray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_normalize(q: NDArray) -> NDArray:
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([1., 0., 0., 0.])


def quat_rotate(q: NDArray, v: NDArray) -> NDArray:
    """Rotate vector v from body frame to NED using quaternion q."""
    v_q = np.concatenate([[0.0], v])
    return quat_mult(quat_mult(q, v_q), quat_conj(q))[1:]


def skew(v: NDArray) -> NDArray:
    return np.array([
        [ 0.0,  -v[2],  v[1]],
        [ v[2],  0.0,  -v[0]],
        [-v[1],  v[0],   0.0],
    ])


class UAV6DOF:
    """Propagates true UAV state at cfg.dt (IMU rate)."""

    def __init__(self, cfg: SimConfig, rng: np.random.Generator):
        self.cfg = cfg
        self.rng = rng
        self.g = np.array([0.0, 0.0, cfg.gravity])   # NED gravity

        # State
        self.p = np.zeros(3)              # position NED
        self.v = np.zeros(3)              # velocity NED
        self.q = np.array([1., 0., 0., 0.])  # quaternion body→NED
        self.omega = np.zeros(3)          # angular rate body
        self.t = 0.0

        # Force / torque setpoints (set externally by mission module)
        self.thrust_body = np.array([0.0, 0.0, 0.0])   # N, body frame
        self.torque_body = np.zeros(3)                   # N·m, body frame

    def set_thrust_ned(self, thrust_ned: NDArray) -> None:
        """Accept desired thrust in NED frame, rotate to body for propagation."""
        pass  # unused — we control via body-frame directly

    def step(self) -> None:
        """One Runge-Kutta 4 integration step at dt."""
        dt = self.cfg.dt
        state = self._pack()
        k1 = self._deriv(state)
        k2 = self._deriv(state + 0.5 * dt * k1)
        k3 = self._deriv(state + 0.5 * dt * k2)
        k4 = self._deriv(state + dt * k3)
        new_state = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        self._unpack(new_state)
        self.q = quat_normalize(self.q)
        self.t += dt

    def _pack(self) -> NDArray:
        return np.concatenate([self.p, self.v, self.q, self.omega])

    def _unpack(self, s: NDArray) -> None:
        self.p     = s[0:3]
        self.v     = s[3:6]
        self.q     = s[6:10]
        self.omega = s[10:13]

    def _deriv(self, s: NDArray) -> NDArray:
        p, v, q, omega = s[0:3], s[3:6], s[6:10], s[10:13]
        q = quat_normalize(q)
        I = np.diag([self.cfg.Ixx, self.cfg.Iyy, self.cfg.Izz])

        # Translational dynamics: NED accelerations
        thrust_ned = quat_rotate(q, self.thrust_body)
        drag = -np.array([self.cfg.drag_xy, self.cfg.drag_xy, self.cfg.drag_z]) * v
        accel_ned = thrust_ned / self.cfg.mass + self.g + drag / self.cfg.mass

        # Quaternion kinematics: q_dot = 0.5 * q ⊗ [0, omega]
        omega_quat = np.concatenate([[0.0], omega])
        q_dot = 0.5 * quat_mult(q, omega_quat)

        # Rotational dynamics: Euler's equation  I·omega_dot = tau - omega × I·omega
        Iomega = I @ omega
        omega_dot = np.linalg.solve(I, self.torque_body - np.cross(omega, Iomega))

        return np.concatenate([v, accel_ned, q_dot, omega_dot])

    @property
    def R_body_to_ned(self) -> NDArray:
        """Rotation matrix body → NED from current quaternion."""
        w, x, y, z = self.q
        return np.array([
            [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
            [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
            [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
        ])
