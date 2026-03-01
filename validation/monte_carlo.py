"""Monte Carlo simulation runner.

Runs the full simulator + ESKF pipeline for N random seeds and collects:
  - Position error vs time
  - Velocity error vs time
  - Attitude error vs time
  - Filter covariance (σ-bounds)
  - Per-step innovation vectors and S matrices
"""
import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict
from numpy.typing import NDArray

from simulator.config import SimConfig, SensorConfig
from simulator.dynamics import UAV6DOF
from simulator.mission import MissionGenerator
from simulator.sensors import IMUModel, GPSModel, BarometerModel
from eskf.eskf import ESKF


@dataclass
class RunResult:
    seed: int
    time: NDArray           # (T,)
    p_err: NDArray          # (T,3) position error NED  m
    v_err: NDArray          # (T,3) velocity error NED  m/s
    att_err_deg: NDArray    # (T,3) attitude error       deg
    pos_std: NDArray        # (T,3) estimated 1-σ pos
    vel_std: NDArray        # (T,3) estimated 1-σ vel
    att_std_deg: NDArray    # (T,3) estimated 1-σ att  deg
    innov_gps: list         # list of (t, innov[6], S[6,6])
    innov_baro: list        # list of (t, innov, S)
    bias_err_g: NDArray     # (T,3) gyro bias error  rad/s
    bias_err_a: NDArray     # (T,3) accel bias error m/s²


def _quat_to_euler_deg(q: NDArray) -> NDArray:
    """Quick ZYX Euler angles (roll, pitch, yaw) from quaternion [w,x,y,z]."""
    w, x, y, z = q
    roll  = np.degrees(np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
    pitch = np.degrees(np.arcsin(np.clip(2*(w*y-z*x), -1, 1)))
    yaw   = np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
    return np.array([roll, pitch, yaw])


def _angle_diff(q1: NDArray, q2: NDArray) -> NDArray:
    """Attitude error in degrees (small-angle approx via quaternion error)."""
    from simulator.dynamics import quat_mult, quat_conj
    q_err = quat_mult(quat_conj(q1), q2)
    # Extract rotation-vector portion
    if q_err[0] < 0:
        q_err = -q_err
    phi = 2.0 * q_err[1:4]
    return np.degrees(phi)


class MonteCarlo:
    def __init__(self, n_runs: int = 50, total_time: float = 60.0):
        self.n_runs = n_runs
        self.total_time = total_time

    def run(self, verbose: bool = True) -> List[RunResult]:
        results = []
        for i in range(self.n_runs):
            seed = i * 7 + 13
            if verbose:
                print(f"  MC run {i+1}/{self.n_runs}  seed={seed}", end="\r")
            try:
                r = self._single_run(seed)
                results.append(r)
            except Exception as e:
                print(f"\n  [WARNING] Run {i} seed={seed} failed: {e}")
        if verbose:
            print(f"\n  MC complete: {len(results)}/{self.n_runs} runs OK.")
        return results

    def _single_run(self, seed: int) -> RunResult:
        rng = np.random.default_rng(seed)
        cfg_sim = SimConfig(dt=0.0025, total_time=self.total_time, seed=seed)
        cfg_s   = SensorConfig()

        uav     = UAV6DOF(cfg_sim, rng)
        mission = MissionGenerator(cfg_sim, rng)
        imu_m   = IMUModel(cfg_sim, cfg_s, rng)
        gps_m   = GPSModel(cfg_sim, cfg_s, rng)
        baro_m  = BarometerModel(cfg_sim, cfg_s, rng)

        # Set initial altitude to match warmup hover target
        warmup_alt = mission.phases[0].params.get("alt", 50.0)
        uav.p[2] = -warmup_alt

        eskf = ESKF(
            dt=cfg_sim.dt,
            sigma_g      = cfg_s.gyro_noise_spectral_density  / np.sqrt(cfg_sim.dt),
            sigma_a      = cfg_s.accel_noise_spectral_density / np.sqrt(cfg_sim.dt),
            sigma_bg_rw  = cfg_s.gyro_bias_random_walk  * np.sqrt(cfg_sim.dt),
            sigma_ba_rw  = cfg_s.accel_bias_random_walk * np.sqrt(cfg_sim.dt),
            sigma_gps_p  = cfg_s.gps_pos_noise_ned,
            sigma_gps_v  = cfg_s.gps_vel_noise_ned,
            sigma_baro   = cfg_s.baro_noise,
            sigma_bg_init = cfg_s.gyro_bias_init_sigma,    # ← was too small before
            sigma_ba_init = cfg_s.accel_bias_init_sigma,   # ← was too small before
        )
        # Initialise filter close to truth (with position/velocity error)
        eskf.p = uav.p.copy() + rng.normal(0, 5.0, 3)
        eskf.v = uav.v.copy() + rng.normal(0, 0.5, 3)

        n_steps = int(cfg_sim.total_time / cfg_sim.dt)
        downsample = 40   # record at 10 Hz to save memory

        time_log     = []
        p_err_log    = []
        v_err_log    = []
        att_err_log  = []
        pos_std_log  = []
        vel_std_log  = []
        att_std_log  = []
        bg_err_log   = []
        ba_err_log   = []

        for step in range(n_steps):
            gps_active = mission.apply(uav)
            uav.step()

            imu_s  = imu_m.sample(uav)
            gps_s  = gps_m.update(uav, gps_active)
            baro_s = baro_m.update(uav)

            eskf.imu_update(imu_s.accel_meas, imu_s.gyro_meas)

            if gps_s is not None and gps_s.valid:
                eskf.gps_update(gps_s.t_stamp, gps_s.pos_ned, gps_s.vel_ned)

            if baro_s is not None and baro_s.valid:
                eskf.baro_update(baro_s.alt_m)

            if step % downsample == 0:
                p_err  = eskf.p - uav.p
                v_err  = eskf.v - uav.v
                att_e  = _angle_diff(uav.q, eskf.q)

                time_log.append(uav.t)
                p_err_log.append(p_err.copy())
                v_err_log.append(v_err.copy())
                att_err_log.append(att_e.copy())
                pos_std_log.append(eskf.pos_std.copy())
                vel_std_log.append(eskf.vel_std.copy())
                att_std_log.append(eskf.att_std_deg.copy())
                bg_err_log.append((eskf.bg - imu_m.b_g).copy())
                ba_err_log.append((eskf.ba - imu_m.b_a).copy())

        return RunResult(
            seed=seed,
            time=np.array(time_log),
            p_err=np.array(p_err_log),
            v_err=np.array(v_err_log),
            att_err_deg=np.array(att_err_log),
            pos_std=np.array(pos_std_log),
            vel_std=np.array(vel_std_log),
            att_std_deg=np.array(att_std_log),
            innov_gps=eskf.innovations_gps,
            innov_baro=eskf.innovations_baro,
            bias_err_g=np.array(bg_err_log),
            bias_err_a=np.array(ba_err_log),
        )
