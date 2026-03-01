"""Python Error-State Kalman Filter (ESKF) — EKF2-style.

Mirrors the C++ implementation in ekf2_core/ equation-by-equation.
This is the reference implementation used by the Python pipeline.

Nominal state:
    x = [p(3), v(3), q(4), b_g(3), b_a(3)]   → 16 elements (q is redundant)

Error state (15-dim):
    δx = [δp(3), δv(3), δθ(3), δb_g(3), δb_a(3)]

IMU propagation is the predict step.
GPS and Baro are sequential update steps.

References:
  Sola (2017) "Quaternion kinematics for ESKF"
  PX4 EKF2 source  (github.com/PX4/PX4-Autopilot)
"""
import numpy as np
from numpy.typing import NDArray
from dataclasses import dataclass
from typing import Optional, Deque
from collections import deque


# ────────────────────────────────────────────────────────────────────────────
# Quaternion helpers (identical logic to C++ inlines in state.h)
# ────────────────────────────────────────────────────────────────────────────

def qmul(q: NDArray, r: NDArray) -> NDArray:
    w0, x0, y0, z0 = q
    w1, x1, y1, z1 = r
    return np.array([
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1,
    ])


def qnorm(q: NDArray) -> NDArray:
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([1., 0., 0., 0.])


def qconj(q: NDArray) -> NDArray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qrot(q: NDArray, v: NDArray) -> NDArray:
    """Rotate v by quaternion q (body → NED)."""
    pv = np.concatenate([[0.0], v])
    return qmul(qmul(q, pv), qconj(q))[1:]


def q_from_rotvec(phi: NDArray) -> NDArray:
    """Small-angle quaternion from rotation vector φ."""
    angle = np.linalg.norm(phi)
    if angle < 1e-10:
        return qnorm(np.array([1.0, phi[0]/2, phi[1]/2, phi[2]/2]))
    axis = phi / angle
    return np.array([np.cos(angle/2), *(np.sin(angle/2) * axis)])


def skew(v: NDArray) -> NDArray:
    return np.array([
        [ 0.0,  -v[2],  v[1]],
        [ v[2],  0.0,  -v[0]],
        [-v[1],  v[0],  0.0],
    ])


def rot_from_quat(q: NDArray) -> NDArray:
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])


# ────────────────────────────────────────────────────────────────────────────
# Buffered state for delay compensation
# ────────────────────────────────────────────────────────────────────────────

BUFFER_SIZE = 400   # ~1 second at 400 Hz


@dataclass
class BufferedState:
    t:  float
    p:  NDArray
    v:  NDArray
    q:  NDArray
    bg: NDArray
    ba: NDArray
    P:  NDArray    # error-state covariance (15×15)


# ────────────────────────────────────────────────────────────────────────────
# EKF2 / ESKF
# ────────────────────────────────────────────────────────────────────────────

class ESKF:
    """
    Error-State Kalman Filter for UAV navigation.

    Parameters
    ----------
    dt          : IMU sample interval (s)
    sigma_g     : gyro white-noise σ per-axis (rad/s) — from PSD
    sigma_a     : accel white-noise σ per-axis (m/s²)
    sigma_bg_rw : gyro-bias random-walk σ per-step (rad/s)
    sigma_ba_rw : accel-bias random-walk σ per-step (m/s²)
    sigma_gps_p : GPS position noise per-axis (m)
    sigma_gps_v : GPS velocity noise per-axis (m/s)
    sigma_baro  : barometer altitude noise (m)
    """

    N = 15   # error-state dimension

    def __init__(
        self,
        dt: float,
        sigma_g: float   = 0.005 / np.sqrt(0.0025),
        sigma_a: float   = 0.1   / np.sqrt(0.0025),
        sigma_bg_rw: float = 0.0002 * np.sqrt(0.0025),
        sigma_ba_rw: float = 0.005  * np.sqrt(0.0025),
        sigma_gps_p: float = 1.5,
        sigma_gps_v: float = 0.15,
        sigma_baro:  float = 0.5,
        sigma_bg_init: float = 0.01,    # rad/s  — matches gyro_bias_init_sigma
        sigma_ba_init: float = 0.2,     # m/s²   — matches accel_bias_init_sigma
    ):
        self.dt = dt

        # ── Process noise (Q, 15×15) ──────────────────────────────────────
        # Position: substantial noise to prevent P from collapsing below actual
        # errors. Accounts for unmodeled dynamics (attitude-gravity coupling,
        # delay compensation imperfections, baro bias).
        # Tuned so P_pos settles ~1m after GPS convergence.
        q_p  = (0.08)**2 * np.ones(3)          # prevents P_pos collapsing
        # Velocity: inflated for unmodeled dynamics + delay compensation
        q_v  = 6.0 * (sigma_a * dt)**2 * np.ones(3)
        q_th = (sigma_g  * dt)**2 * np.ones(3)
        q_bg = sigma_bg_rw**2 * np.ones(3)
        q_ba = sigma_ba_rw**2 * np.ones(3)
        self.Q = np.diag(np.concatenate([q_p, q_v, q_th, q_bg, q_ba]))

        # ── Measurement noise ─────────────────────────────────────────────
        # GPS velocity R: no inflation, rely on Q_v for model mismatch
        self.R_gps = np.diag(
            np.concatenate([
                sigma_gps_p**2 * np.ones(3),
                sigma_gps_v**2 * np.ones(3),
            ])
        )
        self.R_baro = np.array([[sigma_baro**2 + 0.3**2]])  # inflated for residual baro bias

        # ── Nominal state ─────────────────────────────────────────────────
        self.p  = np.zeros(3)
        self.v  = np.zeros(3)
        self.q  = np.array([1., 0., 0., 0.])
        self.bg = np.zeros(3)
        self.ba = np.zeros(3)

        # ── Error-state covariance ────────────────────────────────────────
        # Initialise with small values matching expected initial errors:
        # pos: ±5m, vel: ±1 m/s, att: ±10 deg, biases: realistic
        P_diag = np.array([
            5.0**2, 5.0**2, 5.0**2,             # pos error  m
            1.0**2, 1.0**2, 1.0**2,             # vel error  m/s
            np.radians(10)**2,                   # roll error
            np.radians(10)**2,                   # pitch error
            np.radians(30)**2,                   # yaw error (least observable)
            sigma_bg_init**2,                    # gyro bias  (rad/s)²
            sigma_bg_init**2,
            sigma_bg_init**2,
            sigma_ba_init**2,                    # accel bias (m/s²)²
            sigma_ba_init**2,
            sigma_ba_init**2,
        ])
        self.P = np.diag(P_diag)

        # ── Gravity ───────────────────────────────────────────────────────
        self.g_ned = np.array([0., 0., 9.80665])

        # ── Ring buffer for delay compensation ───────────────────────────
        self._buf: Deque[BufferedState] = deque(maxlen=BUFFER_SIZE)

        self.t = 0.0

        # ── Last estimated NED acceleration (for adaptive R_gps_v) ────────
        self._a_ned_est = np.zeros(3)
        self._gps_delay = 0.2   # nominal GPS delay (s)

        # ── Innovation logging ────────────────────────────────────────────
        self.innovations_gps: list = []      # (t, innov, S)
        self.innovations_baro: list = []

    # ── Public interface ──────────────────────────────────────────────────

    def imu_update(self, accel_meas: NDArray, gyro_meas: NDArray) -> None:
        """Predict step: integrate IMU measurement, propagate P."""
        dt = self.dt
        om = gyro_meas - self.bg          # corrected angular rate
        am = accel_meas - self.ba         # corrected specific force

        R = rot_from_quat(self.q)         # body → NED

        # ── Nominal state propagation ─────────────────────────────────────
        p_new = self.p + self.v * dt
        v_new = self.v + (R @ am + self.g_ned) * dt
        dq    = q_from_rotvec(om * dt)
        q_new = qnorm(qmul(self.q, dq))
        bg_new = self.bg.copy()
        ba_new = self.ba.copy()

        # ── Error-state Jacobian F (15×15) ────────────────────────────────
        F = np.eye(self.N)
        # δp_dot ← δv
        F[0:3, 3:6] = np.eye(3) * dt
        # δv_dot ← -R[am×] δθ
        F[3:6, 6:9] = -R @ skew(am) * dt
        # δv_dot ← -R δba
        F[3:6, 12:15] = -R * dt
        # δθ_dot ← -[om×] δθ
        F[6:9, 6:9] = np.eye(3) - skew(om) * dt
        # δθ_dot ← -δbg
        F[6:9, 9:12] = -np.eye(3) * dt

        # ── Covariance propagation ────────────────────────────────────────
        self.P = F @ self.P @ F.T + self.Q

        # Symmetrize to prevent numerical drift
        self.P = 0.5 * (self.P + self.P.T)

        # Guard against NaN/Inf (clip, don't allow divergence)
        if not np.all(np.isfinite(self.P)):
            self.P = np.diag(np.clip(np.diag(self.P), 0.0, 1e6))
            self.P = np.where(np.isfinite(self.P), self.P, 0.0)
        # Ensure diagonal stays positive
        np.fill_diagonal(self.P, np.maximum(np.diag(self.P), 1e-8))

        # ── Commit nominal state ──────────────────────────────────────────
        self.p  = p_new
        self.v  = v_new
        self.q  = q_new
        self.bg = bg_new
        self.ba = ba_new
        self.t += dt

        # Save estimated NED acceleration for adaptive GPS velocity R
        self._a_ned_est = R @ am + self.g_ned

        # ── Save to history buffer ────────────────────────────────────────
        self._buf.append(BufferedState(
            t=self.t,
            p=self.p.copy(), v=self.v.copy(), q=self.q.copy(),
            bg=self.bg.copy(), ba=self.ba.copy(),
            P=self.P.copy(),
        ))

    def gps_update(self, t_stamp: float,
                   pos_ned: NDArray, vel_ned: NDArray) -> None:
        """Sequential GPS measurement update with delay compensation.

        Position innovation: GPS_pos(t_stamp) – buffered_pos(t_stamp)
          → delay-compensated since GPS measurement reflects position at
            measurement time, not receive time.

        Velocity innovation: GPS_vel(t_stamp) – current_vel
          → use current velocity because the buffer doesn't include
            previous GPS-applied corrections (no full state replay).
            The velocity timing mismatch is absorbed by inflated Q_v.

        Kalman gain: computed from CURRENT P.
        """
        # ------- references at t_stamp ------------------------------------
        bs = self._find_buffered(t_stamp)
        p_ref = bs.p if bs is not None else self.p

        # ------- innovation -----------------------------------------------
        innov = np.concatenate([pos_ned - p_ref,
                                vel_ned - self.v])

        # ------- Kalman gain with CURRENT P --------------------------------
        H = np.zeros((6, self.N))
        H[0:3, 0:3] = np.eye(3)
        H[3:6, 3:6] = np.eye(3)

        # Adaptive GPS velocity R: inflate based on delay × non-gravitational
        # acceleration.  During hover a_manoeuvre ≈ 0, during turns/climbs
        # the delay-induced velocity bias is |a_man|·delay.
        a_manoeuvre = self._a_ned_est - self.g_ned  # remove gravity component
        delay_vel_bias = np.abs(a_manoeuvre) * self._gps_delay
        R_gps = self.R_gps.copy()
        for i in range(3):
            R_gps[3+i, 3+i] += delay_vel_bias[i]**2

        S = H @ self.P @ H.T + R_gps
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        K = self.P @ H.T @ S_inv

        dx = K @ innov
        if not np.all(np.isfinite(dx)):
            return

        # Innovation gating: reject if NIS exceeds 5× chi2(6, 0.999) ≈ 140
        try:
            nis = float(innov @ S_inv @ innov)
            if not np.isfinite(nis) or nis > 140.0:
                self.innovations_gps.append((self.t, innov.copy(), S.copy()))
                return
        except np.linalg.LinAlgError:
            return

        # ------- apply correction ------------------------------------------
        self._inject(dx)

        # Joseph form update
        IKH   = np.eye(self.N) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R_gps @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        self.innovations_gps.append((self.t, innov.copy(), S.copy()))

    def baro_update(self, alt_m: float) -> None:
        """Barometric altitude update (simple — no delay model)."""
        # Observation: z = -p[2]  (NED z → altitude)
        H = np.zeros((1, self.N))
        H[0, 2] = -1.0    # d(-p_z)/d(δp_z) = -1

        z_pred = -self.p[2]
        innov  = alt_m - z_pred

        S = H @ self.P @ H.T + self.R_baro
        K = self.P @ H.T @ np.linalg.inv(S)

        dx = (K @ np.array([innov])).flatten()
        self._inject(dx)
        # Joseph form
        IKH   = np.eye(self.N) - K @ H
        R_mat = self.R_baro
        self.P = IKH @ self.P @ IKH.T + K @ R_mat @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        self.innovations_baro.append((self.t, innov, float(S[0, 0])))

    # ── Internal helpers ──────────────────────────────────────────────────

    def _gps_meas_update(self, p, v, P, pos_ned, vel_ned):
        dx, P_new = self._gps_meas_update_dx(p, v, P, pos_ned, vel_ned)
        self._inject(dx)
        self.P = P_new
        self.P = 0.5 * (self.P + self.P.T)

    def _gps_meas_update_dx(self, p, v, P, pos_ned, vel_ned):
        # H (6×15): pos and vel are directly observable
        H = np.zeros((6, self.N))
        H[0:3, 0:3] = np.eye(3)   # δp
        H[3:6, 3:6] = np.eye(3)   # δv

        innov = np.concatenate([pos_ned - p, vel_ned - v])
        S = H @ P @ H.T + self.R_gps
        K = P @ H.T @ np.linalg.inv(S)
        dx = K @ innov
        # Joseph form for numerical stability: (I-KH)P(I-KH)^T + KRK^T
        IKH   = np.eye(self.N) - K @ H
        P_new = IKH @ P @ IKH.T + K @ self.R_gps @ K.T
        P_new = 0.5 * (P_new + P_new.T)    # symmetrise

        self.innovations_gps.append((self.t, innov.copy(), S.copy()))
        return dx, P_new

    def _inject(self, dx: NDArray) -> None:
        """Inject error-state correction into nominal state and reset δx=0."""
        self.p  += dx[0:3]
        self.v  += dx[3:6]
        self.q   = qnorm(qmul(self.q, q_from_rotvec(dx[6:9])))
        self.bg += dx[9:12]
        self.ba += dx[12:15]

    def _find_buffered(self, t_stamp: float) -> Optional[BufferedState]:
        best: Optional[BufferedState] = None
        best_dt = np.inf
        for bs in self._buf:
            dt = abs(bs.t - t_stamp)
            if dt < best_dt:
                best_dt = dt
                best = bs
        # Accept if within 3 × GPS period
        if best_dt < 0.35:
            return best
        return None

    # ── State accessors ───────────────────────────────────────────────────

    @property
    def state(self):
        return dict(p=self.p.copy(), v=self.v.copy(),
                    q=self.q.copy(), bg=self.bg.copy(), ba=self.ba.copy())

    @property
    def pos_std(self) -> NDArray:
        return np.sqrt(np.diag(self.P[0:3, 0:3]))

    @property
    def vel_std(self) -> NDArray:
        return np.sqrt(np.diag(self.P[3:6, 3:6]))

    @property
    def att_std_deg(self) -> NDArray:
        return np.rad2deg(np.sqrt(np.diag(self.P[6:9, 6:9])))
