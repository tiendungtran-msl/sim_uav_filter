"""sim_ekf2/run_validation.py — Full Monte Carlo + NEES + Innovation validation.

Usage:
    python run_validation.py [--runs N] [--time T] [--quick]

  --quick : 10 runs, 30 s each (fast check)
  --runs  : number of Monte Carlo runs (default 50)
  --time  : duration per run in seconds  (default 60)

Produces outputs/ directory with:
  * monte_carlo.html    — aggregated NEES + RMS error plots
  * mc_report.txt       — text summary  (pass/fail criteria)
"""
import sys
import os
import argparse
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from validation.monte_carlo import MonteCarlo
from validation.nees import nees_report
from validation.innovation import InnovationAnalyzer
from visualization.dashboard import EKFDashboard


def run_validation(n_runs: int = 50, total_time: float = 60.0) -> None:
    print(f"\n{'='*60}")
    print(f"  sim_ekf2 — Monte Carlo Validation")
    print(f"  runs={n_runs}  duration={total_time}s each")
    print(f"{'='*60}\n")

    mc = MonteCarlo(n_runs=n_runs, total_time=total_time)
    results = mc.run(verbose=True)

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "outputs")
    os.makedirs(out_dir, exist_ok=True)

    # ── NEES report ────────────────────────────────────────────────────────
    rep = nees_report(results)

    # ── Innovation analysis (use last run) ─────────────────────────────────
    print("\n  Running innovation analysis on last run...")
    # Re-run a clean single simulation to get innovation data
    from simulator.config import SimConfig, SensorConfig
    from simulator.dynamics import UAV6DOF
    from simulator.mission import MissionGenerator
    from simulator.sensors import IMUModel, GPSModel, BarometerModel
    from eskf.eskf import ESKF

    seed    = 999
    rng     = np.random.default_rng(seed)
    cfg_sim = SimConfig(dt=0.0025, total_time=total_time, seed=seed)
    cfg_s   = SensorConfig()

    uav     = UAV6DOF(cfg_sim, rng)
    mission = MissionGenerator(cfg_sim, rng)
    imu_m   = IMUModel(cfg_sim, cfg_s, rng)
    gps_m   = GPSModel(cfg_sim, cfg_s, rng)
    baro_m  = BarometerModel(cfg_sim, cfg_s, rng)
    warmup_alt = mission.phases[0].params.get("alt", 50.0)
    uav.p[2] = -warmup_alt

    eskf = ESKF(
        dt=cfg_sim.dt,
        sigma_g     = cfg_s.gyro_noise_spectral_density  / np.sqrt(cfg_sim.dt),
        sigma_a     = cfg_s.accel_noise_spectral_density / np.sqrt(cfg_sim.dt),
        sigma_bg_rw = cfg_s.gyro_bias_random_walk  * np.sqrt(cfg_sim.dt),
        sigma_ba_rw = cfg_s.accel_bias_random_walk * np.sqrt(cfg_sim.dt),
        sigma_gps_p = cfg_s.gps_pos_noise_ned,
        sigma_gps_v = cfg_s.gps_vel_noise_ned,
        sigma_baro  = cfg_s.baro_noise,
    )
    eskf.p = uav.p.copy() + rng.normal(0, 3.0, 3)

    for _ in range(int(total_time / cfg_sim.dt)):
        gps_active = mission.apply(uav)
        uav.step()
        imu_s  = imu_m.sample(uav)
        gps_s  = gps_m.update(uav, gps_active)
        baro_s = baro_m.update(uav)
        eskf.imu_update(imu_s.accel_meas, imu_s.gyro_meas)
        if gps_s and gps_s.valid:
            eskf.gps_update(gps_s.t_stamp, gps_s.pos_ned, gps_s.vel_ned)
        if baro_s and baro_s.valid:
            eskf.baro_update(baro_s.alt_m)

    ia = InnovationAnalyzer()
    ia.feed_from_eskf(eskf)
    gps_innov   = ia.gps_innovation_summary()
    baro_innov  = ia.baro_innovation_summary()
    acf_result  = ia.autocorrelation_test()

    # ── Dashboard ──────────────────────────────────────────────────────────
    print("\n  Generating Monte Carlo dashboard...")
    dash = EKFDashboard(out_dir=out_dir)
    _, rep_check = dash.plot_monte_carlo(results, filename="monte_carlo.html")

    # ── Text report ────────────────────────────────────────────────────────
    T   = min(len(r.time) for r in results)
    p_err_all = np.array([r.p_err[:T] for r in results])
    v_err_all = np.array([r.v_err[:T] for r in results])
    rms_p = float(np.sqrt(np.mean(p_err_all**2)))
    rms_v = float(np.sqrt(np.mean(v_err_all**2)))

    diverged = sum(1 for r in results
                   if np.max(np.linalg.norm(r.p_err, axis=1)) > 50.0)

    report_lines = [
        "=" * 60,
        "  sim_ekf2 — Validation Report",
        "=" * 60,
        f"  Runs:               {len(results)}",
        f"  Duration per run:   {total_time:.0f} s",
        "",
        "── NEES ──────────────────────────────────────────────────",
        f"  NEES position:      {'PASS' if rep['pass_pos'] else 'FAIL'}"
        f"  (in-bounds {rep['frac_in_pos']*100:.0f}%  "
        f"bounds [{rep['bounds_pos'][0]:.2f}, {rep['bounds_pos'][1]:.2f}])",
        f"  NEES velocity:      {'PASS' if rep['pass_vel'] else 'FAIL'}"
        f"  (in-bounds {rep['frac_in_vel']*100:.0f}%  "
        f"bounds [{rep['bounds_vel'][0]:.2f}, {rep['bounds_vel'][1]:.2f}])",
        "",
        "── Errors ────────────────────────────────────────────────",
        f"  Overall pos RMSE:   {rms_p:.3f} m",
        f"  Overall vel RMSE:   {rms_v:.3f} m/s",
        f"  Diverged runs:      {diverged}  (> 50 m pos error)",
        "",
        "── Innovation Analysis ───────────────────────────────────",
    ]

    if gps_innov:
        report_lines += [
            f"  GPS zero-mean:      {'OK' if gps_innov['zero_mean_ok'] else 'FAIL'}",
            f"  GPS NIS χ² pass:    {gps_innov['frac_within_chi2']*100:.0f}%  "
            f"  ({'PASS' if gps_innov['pass'] else 'FAIL'})",
        ]
    if baro_innov:
        report_lines += [
            f"  Baro NIS χ² pass:   {baro_innov['frac_within_chi2']*100:.0f}%"
            f"  ({'PASS' if baro_innov['pass'] else 'FAIL'})",
        ]
    if acf_result:
        report_lines += [
            f"  Whiteness (ACF):    {acf_result.get('pass_frac', 0)*100:.0f}% lags OK"
            f"  ({'PASS' if acf_result.get('pass') else 'FAIL'})",
        ]

    overall_pass = (
        rep["pass_pos"] and rep["pass_vel"] and diverged == 0
        and (gps_innov.get("pass", False) if gps_innov else True)
    )
    report_lines += [
        "",
        "── Overall ───────────────────────────────────────────────",
        f"  RESULT: {'✅ PASS' if overall_pass else '❌ FAIL (see details above)'}",
        "=" * 60,
    ]

    report_text = "\n".join(report_lines)
    print(report_text)

    report_path = os.path.join(out_dir, "mc_report.txt")
    with open(report_path, "w") as f:
        f.write(report_text + "\n")
    print(f"\n  Report saved: {report_path}")
    print(f"  All outputs: {out_dir}/\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="sim_ekf2 Monte Carlo validation")
    parser.add_argument("--runs",  type=int,   default=50)
    parser.add_argument("--time",  type=float, default=60.0)
    parser.add_argument("--quick", action="store_true",
                        help="Quick mode: 10 runs, 30s each")
    args = parser.parse_args()

    if args.quick:
        run_validation(n_runs=10, total_time=30.0)
    else:
        run_validation(n_runs=args.runs, total_time=args.time)
