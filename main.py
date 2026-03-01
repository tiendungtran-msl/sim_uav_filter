"""sim_ekf2/main.py — Single-run UAV EKF2 simulation and visualisation.

Usage:
    python main.py [--seed N] [--time T] [--no-anim]

Produces outputs/ directory with:
  * single_run.html   — estimation error, innovations, NIS dashboard
  * trajectory_3d.html — 3D trajectory
  * uav_animation.html — animated 3D flight (may take ~30s to render)
  * sensor_log.csv    — raw sensor log for C++ replay tool
"""
import sys
import argparse
import numpy as np
import csv
import os

# Ensure project root is on path when running from project directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from simulator.config import SimConfig, SensorConfig
from simulator.dynamics import UAV6DOF
from simulator.mission import MissionGenerator
from simulator.sensors import IMUModel, GPSModel, BarometerModel
from eskf.eskf import ESKF
from visualization.dashboard import EKFDashboard
from visualization.animation import UAVAnimation3D
from validation.innovation import InnovationAnalyzer


def run_simulation(seed: int = 42, total_time: float = 60.0,
                   animate: bool = True) -> None:
    print(f"\n{'='*60}")
    print(f"  sim_ekf2 — EKF2 (ESKF) UAV Simulation")
    print(f"  seed={seed}  duration={total_time}s")
    print(f"{'='*60}\n")

    rng     = np.random.default_rng(seed)
    cfg_sim = SimConfig(dt=0.0025, total_time=total_time, seed=seed)
    cfg_s   = SensorConfig()

    uav     = UAV6DOF(cfg_sim, rng)
    mission = MissionGenerator(cfg_sim, rng)
    imu_m   = IMUModel(cfg_sim, cfg_s, rng)
    gps_m   = GPSModel(cfg_sim, cfg_s, rng)
    baro_m  = BarometerModel(cfg_sim, cfg_s, rng)

    # Initial hover altitude (match warmup hover target)
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
        sigma_bg_init = cfg_s.gyro_bias_init_sigma,
        sigma_ba_init = cfg_s.accel_bias_init_sigma,
    )
    # Initialise filter at true state + small perturbation
    eskf.p = uav.p.copy() + rng.normal(0, 3.0, 3)
    eskf.v = uav.v.copy() + rng.normal(0, 0.3, 3)

    n_steps = int(total_time / cfg_sim.dt)
    log_every = 40    # 10 Hz logging

    # Logging arrays
    time_log   = []; p_true_log  = []; p_est_log   = []
    v_true_log = []; v_est_log   = []; p_std_log   = []
    v_std_log  = []; att_err_log = []; att_std_log = []
    gps_pos_log = []  # sparse GPS positions for plotting

    # CSV sensor log (for C++ replay)
    out_dir  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "outputs")
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, "sensor_log.csv")

    print("  Running simulation...")
    with open(csv_path, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow([
            "t",
            "imu_gx","imu_gy","imu_gz","imu_ax","imu_ay","imu_az",
            "gps_t","gps_px","gps_py","gps_pz","gps_vx","gps_vy","gps_vz","gps_valid",
            "baro_alt","baro_valid",
        ])

        for step in range(n_steps):
            gps_active = mission.apply(uav)
            uav.step()

            imu_s  = imu_m.sample(uav)
            gps_s  = gps_m.update(uav, gps_active)
            baro_s = baro_m.update(uav)

            # EKF predict
            eskf.imu_update(imu_s.accel_meas, imu_s.gyro_meas)

            gps_valid_this = False
            gps_row_t = 0.0
            gps_row_p = [0.0]*3; gps_row_v = [0.0]*3

            if gps_s is not None and gps_s.valid:
                eskf.gps_update(gps_s.t_stamp, gps_s.pos_ned, gps_s.vel_ned)
                gps_valid_this = True
                gps_row_t = gps_s.t_stamp
                gps_row_p = gps_s.pos_ned.tolist()
                gps_row_v = gps_s.vel_ned.tolist()
                gps_pos_log.append(gps_s.pos_ned.copy())

            baro_valid_this = False
            baro_row_alt = 0.0
            if baro_s is not None and baro_s.valid:
                eskf.baro_update(baro_s.alt_m)
                baro_valid_this = True
                baro_row_alt = baro_s.alt_m

            # CSV row
            writer.writerow([
                f"{uav.t:.6f}",
                *[f"{v:.6f}" for v in imu_s.gyro_meas],
                *[f"{v:.6f}" for v in imu_s.accel_meas],
                f"{gps_row_t:.6f}",
                *[f"{v:.4f}" for v in gps_row_p],
                *[f"{v:.6f}" for v in gps_row_v],
                int(gps_valid_this),
                f"{baro_row_alt:.4f}", int(baro_valid_this),
            ])

            # Logging
            if step % log_every == 0:
                from validation.monte_carlo import _angle_diff
                att_e = _angle_diff(uav.q, eskf.q)

                time_log.append(uav.t)
                p_true_log.append(uav.p.copy())
                p_est_log.append(eskf.p.copy())
                v_true_log.append(uav.v.copy())
                v_est_log.append(eskf.v.copy())
                p_std_log.append(eskf.pos_std.copy())
                v_std_log.append(eskf.vel_std.copy())
                att_err_log.append(att_e.copy())
                att_std_log.append(eskf.att_std_deg.copy())

    # Convert to arrays
    time      = np.array(time_log)
    p_true    = np.array(p_true_log)
    p_est     = np.array(p_est_log)
    v_true    = np.array(v_true_log)
    v_est     = np.array(v_est_log)
    p_std     = np.array(p_std_log)
    v_std     = np.array(v_std_log)
    att_err   = np.array(att_err_log)
    att_std   = np.array(att_std_log)
    gps_pos   = np.array(gps_pos_log) if gps_pos_log else np.zeros((0, 3))

    # Print summary
    pos_rmse = np.sqrt(np.mean((p_est - p_true)**2, axis=0))
    vel_rmse = np.sqrt(np.mean((v_est - v_true)**2, axis=0))
    print(f"\n  Final state:")
    print(f"    Pos RMSE (N,E,D): {pos_rmse[0]:.2f}  {pos_rmse[1]:.2f}  {pos_rmse[2]:.2f} m")
    print(f"    Vel RMSE (N,E,D): {vel_rmse[0]:.3f}  {vel_rmse[1]:.3f}  {vel_rmse[2]:.3f} m/s")
    print(f"    GPS innovations:  {len(eskf.innovations_gps)}")
    print(f"    Baro innovations: {len(eskf.innovations_baro)}")
    print(f"    Sensor log:       {csv_path}")

    # Innovation analysis
    ia = InnovationAnalyzer()
    ia.feed_from_eskf(eskf)
    gps_innov  = ia.gps_innovation_summary()
    baro_innov = ia.baro_innovation_summary()
    if gps_innov:
        print(f"\n  GPS innovation analysis:")
        print(f"    zero-mean OK:       {gps_innov['zero_mean_ok']}")
        print(f"    NIS within χ²(95%): {gps_innov['frac_within_chi2']*100:.0f}%")
        print(f"    PASS: {gps_innov['pass']}")

    # ── Visualisation ──────────────────────────────────────────────────────
    print("\n  Generating plots...")
    dash = EKFDashboard(out_dir=out_dir)

    dash.plot_single_run(
        time=time,
        p_true=p_true, p_meas=gps_pos, p_est=p_est, p_std=p_std,
        v_true=v_true, v_est=v_est, v_std=v_std,
        att_err_deg=att_err, att_std_deg=att_std,
        innov_gps=eskf.innovations_gps,
        innov_baro=eskf.innovations_baro,
        title=f"EKF2 Single Run (seed={seed}, {total_time:.0f}s)",
        filename="single_run.html",
    )

    dash.plot_3d_trajectory(
        p_true=p_true, p_est=p_est, p_meas=gps_pos,
        filename="trajectory_3d.html",
    )

    if animate:
        print("  Generating 3D animation (may take ~30s)...")
        anim = UAVAnimation3D(out_dir=out_dir)
        anim.animate(
            time=time, p_true=p_true, p_est=p_est, p_std=p_std,
            gps_t=np.arange(len(gps_pos)), gps_pos=gps_pos,
            downsample=5,
            filename="uav_animation.html",
        )

    print(f"\n  All outputs written to: {out_dir}/")
    print("  Open single_run.html and trajectory_3d.html in a browser.\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="sim_ekf2 single run")
    parser.add_argument("--seed",  type=int,   default=42)
    parser.add_argument("--time",  type=float, default=60.0)
    parser.add_argument("--no-anim", action="store_true")
    args = parser.parse_args()
    run_simulation(seed=args.seed, total_time=args.time,
                   animate=not args.no_anim)
