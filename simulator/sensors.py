"""Sensor Models: IMU, GPS, Barometer.

All noise levels are realistic MEMS / civilian-grade.
Models explicitly track:
  * bias random walk (true bias evolves over time)
  * measurement white noise
  * GPS measurement delay (ring-buffer based)
  * sensor sample rates

The EKF receives only what these sensors output. Nothing else.
"""
import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import NamedTuple, Optional, Tuple
from .config import SensorConfig, SimConfig
from .dynamics import UAV6DOF, quat_rotate, skew


# ── Output types ──────────────────────────────────────────────────────────

@dataclass
class IMUSample:
    t: float
    accel_meas: np.ndarray    # m/s^2  measured  (body)
    gyro_meas:  np.ndarray    # rad/s  measured  (body)
    true_accel: np.ndarray    # ground-truth specific force (body)
    true_gyro:  np.ndarray    # ground-truth angular rate   (body)


@dataclass
class GPSSample:
    t_stamp: float            # timestamp when measurement was taken (earlier)
    t_receive: float          # time measurement arrives at filter
    pos_ned: np.ndarray       # m   NED
    vel_ned: np.ndarray       # m/s NED
    valid: bool


@dataclass
class BaroSample:
    t: float
    alt_m: float              # altitude in metres (positive upwards)
    valid: bool


# ── IMU Model ─────────────────────────────────────────────────────────────

class IMUModel:
    """
    Simulates MEMS IMU at cfg.dt (IMU rate).

    True-body-frame specific force = R^T (a_ned - g) where g=[0,0,g] NED.
    """

    def __init__(self, cfg_sim: SimConfig, cfg_s: SensorConfig,
                 rng: np.random.Generator):
        self.cfg_sim = cfg_sim
        self.cfg_s = cfg_s
        self.rng = rng

        dt = cfg_sim.dt
        # Power Spectral Density → discrete σ
        self.sigma_g_white  = cfg_s.gyro_noise_spectral_density  / np.sqrt(dt)
        self.sigma_a_white  = cfg_s.accel_noise_spectral_density / np.sqrt(dt)
        self.sigma_g_rw     = cfg_s.gyro_bias_random_walk  * np.sqrt(dt)   # bias step
        self.sigma_a_rw     = cfg_s.accel_bias_random_walk * np.sqrt(dt)

        # True (hidden) biases
        self.b_g = rng.normal(0, cfg_s.gyro_bias_init_sigma,  3)
        self.b_a = rng.normal(0, cfg_s.accel_bias_init_sigma, 3)

        self.g_ned = np.array([0.0, 0.0, cfg_sim.gravity])

    def sample(self, uav: UAV6DOF) -> IMUSample:
        # Update bias random walk
        self.b_g += self.rng.normal(0, self.sigma_g_rw, 3)
        self.b_a += self.rng.normal(0, self.sigma_a_rw, 3)

        R   = uav.R_body_to_ned        # body → NED
        R_T = R.T                       # NED  → body

        # True NED specific force = (thrust + drag) / mass
        # This is what an ideal accelerometer measures (gravity NOT included).
        thrust_ned = R @ uav.thrust_body           # N in NED
        drag_ned   = -np.array([
            self.cfg_sim.drag_xy,
            self.cfg_sim.drag_xy,
            self.cfg_sim.drag_z,
        ]) * uav.v                                  # N in NED (linear drag)
        f_ned_true  = (thrust_ned + drag_ned) / self.cfg_sim.mass
        f_body_true = R_T @ f_ned_true              # body-frame specific force

        # Angular velocity (true)
        omega_true = uav.omega.copy()

        # Add sensor noise and bias
        accel_meas = f_body_true + self.b_a + self.rng.normal(0, self.sigma_a_white, 3)
        gyro_meas  = omega_true  + self.b_g + self.rng.normal(0, self.sigma_g_white, 3)

        return IMUSample(
            t=uav.t,
            accel_meas=accel_meas,
            gyro_meas=gyro_meas,
            true_accel=f_body_true,
            true_gyro=omega_true,
        )


# ── GPS Model ─────────────────────────────────────────────────────────────

class GPSModel:
    """
    Simulates GPS at gps_rate Hz with cfg_s.gps_delay latency.

    Maintains a ring buffer of past (t, pos, vel) pairs.
    When the delay elapses, the measurement is released to the filter.
    """

    def __init__(self, cfg_sim: SimConfig, cfg_s: SensorConfig,
                 rng: np.random.Generator):
        self.cfg_sim = cfg_sim
        self.cfg_s   = cfg_s
        self.rng     = rng
        self.dt_gps  = 1.0 / cfg_sim.gps_rate
        self._t_last_gps = -self.dt_gps      # so first sample fires at t=0
        self._buffer: deque = deque()         # pending (t_stamp, pos, vel)
        self.sigma_p = cfg_s.gps_pos_noise_ned
        self.sigma_v = cfg_s.gps_vel_noise_ned
        self.delay   = cfg_s.gps_delay

    def update(self, uav: UAV6DOF, gps_active: bool) -> Optional[GPSSample]:
        """Call every dynamics step. Returns a GPSSample only when a new
        delayed measurement is ready for consumption, else None."""
        t = uav.t

        # Generate a new raw GPS fix if it's time
        if t >= self._t_last_gps + self.dt_gps:
            self._t_last_gps += self.dt_gps
            if gps_active:
                pos_noisy = uav.p + self.rng.normal(0, self.sigma_p, 3)
                vel_noisy = uav.v + self.rng.normal(0, self.sigma_v, 3)
                self._buffer.append((t, pos_noisy.copy(), vel_noisy.copy(), True))
            # If GPS dropped out, we still mark the slot but as invalid
            # (no measurement emitted)

        # Check if the oldest buffered fix has waited long enough
        if self._buffer:
            t_stamp, pos, vel, valid = self._buffer[0]
            if t >= t_stamp + self.delay:
                self._buffer.popleft()
                return GPSSample(
                    t_stamp=t_stamp,
                    t_receive=t,
                    pos_ned=pos,
                    vel_ned=vel,
                    valid=valid,
                )
        return None


# ── Barometer ─────────────────────────────────────────────────────────────

class BarometerModel:
    def __init__(self, cfg_sim: SimConfig, cfg_s: SensorConfig,
                 rng: np.random.Generator):
        self.cfg_sim = cfg_sim
        self.cfg_s   = cfg_s
        self.rng     = rng
        self.dt_baro = 1.0 / cfg_sim.baro_rate
        self._t_last = -self.dt_baro
        self._bias   = rng.normal(0, 0.3)    # initial barometric bias (m)
        self._sigma_drift = cfg_s.baro_bias_drift_rate * np.sqrt(cfg_sim.dt)

    def update(self, uav: UAV6DOF) -> Optional[BaroSample]:
        t = uav.t
        if t < self._t_last + self.dt_baro:
            return None
        self._t_last += self.dt_baro

        # Drift bias
        self._bias += self.rng.normal(0, self._sigma_drift)

        # NED z is negative altitude → altitude = -p[2]
        alt_true = -uav.p[2]
        alt_meas = alt_true + self._bias + self.rng.normal(0, self.cfg_s.baro_noise)
        return BaroSample(t=t, alt_m=alt_meas, valid=True)
