"""sim.cli — Entry point headless (không GUI): python -m sim.cli

Chạy simulator nhanh nhất có thể, xuất log CSV.
Usage:
    python -m sim.cli --headless --time 60 --seed 1 --out logs/run1.csv

Nếu không có --headless thì chuyển sang GUI mode (gọi sim.app).
"""
import sys
import argparse
import time
import numpy as np

from sim.config import SimConfig, SensorConfig
from sim.timebase import Timebase
from sim.dynamics.rigid_body_6dof import UAV6DOF
from sim.dynamics.atmosphere import Atmosphere
from sim.scenario.phase_scheduler import PhaseScheduler
from sim.sensors.imu import ImuModel
from sim.sensors.gps import GpsModel
from sim.sensors.baro import BaroModel
from sim.sensors.mag import MagModel
from sim.io.schema import TruthState, ImuMeas, SimFrame
from sim.io.logger_csv import CsvLogger


def run_headless(seed: int, total_time: float, out_path: str) -> None:
    """Chạy simulator headless (không GUI) → xuất CSV."""
    print(f"\n{'='*60}")
    print(f"  UAV Simulator — Headless Mode")
    print(f"  seed={seed}  duration={total_time}s")
    print(f"  output: {out_path}")
    print(f"{'='*60}\n")

    rng  = np.random.default_rng(seed)
    cfg  = SimConfig(dt=0.0025, total_time=total_time, seed=seed)
    scfg = SensorConfig()

    # Khởi tạo modules
    uav   = UAV6DOF(cfg, rng)
    atmo  = Atmosphere(cfg.dt, rng)
    sched = PhaseScheduler(cfg, rng)
    imu_m = ImuModel(cfg, scfg, rng)
    gps_m = GpsModel(cfg, scfg, rng)
    baro_m = BaroModel(cfg, scfg, rng)
    mag_m = MagModel(cfg, scfg, rng)

    # Đặt UAV ở độ cao ban đầu
    warmup_alt = sched.phases[0].params.get("alt", 50.0)
    uav.p[2] = -warmup_alt

    n_steps = int(total_time / cfg.dt)
    t_start = time.perf_counter()

    print(f"  Simulating {n_steps} steps ({total_time}s at {1/cfg.dt:.0f} Hz)...")

    with CsvLogger(out_path) as logger:
        for step in range(n_steps):
            # 1) Atmosphere
            uav.wind_ned = atmo.step()

            # 2) Scenario
            gps_active = sched.apply(uav, atmo)

            # 3) Dynamics
            uav.step()

            # 4) Sensors
            raw_imu  = imu_m.sample(uav)
            gps_meas = gps_m.update(uav, gps_active)
            baro_meas = baro_m.update(uav)
            mag_meas = mag_m.update(uav)
            mag_truth = mag_m.truth_body(uav)

            # 5) SimFrame
            truth = TruthState(
                t=uav.t,
                pos_ned=uav.p.copy(),
                vel_ned=uav.v.copy(),
                quat=uav.q.copy(),
                omega_body=uav.omega.copy(),
                specific_force_body=uav.specific_force_body.copy(),
                mag_body=mag_truth.copy(),
                euler_deg=uav.euler_deg.copy(),
                gyro_bias=raw_imu.gyro_bias.copy(),
                accel_bias=raw_imu.accel_bias.copy(),
            )
            imu = ImuMeas(t=raw_imu.t, gyro=raw_imu.gyro_meas.copy(),
                           accel=raw_imu.accel_meas.copy())
            frame = SimFrame(
                truth=truth, imu=imu,
                gps=gps_meas, baro=baro_meas, mag=mag_meas,
                phase_name=sched.current_phase_name,
                gps_active=gps_active,
            )

            logger.write(frame)

            # Progress mỗi 10%
            if step % (n_steps // 10) == 0 and step > 0:
                pct = step / n_steps * 100
                print(f"    {pct:.0f}% ... t={uav.t:.1f}s  phase={sched.current_phase_name}")

    elapsed = time.perf_counter() - t_start
    print(f"\n  Done in {elapsed:.2f}s (realtime ratio: {total_time/elapsed:.1f}×)")
    print(f"  Output: {out_path}")
    print(f"  Phases executed: {len(sched.phases)}")
    print()


def main():
    parser = argparse.ArgumentParser(description="UAV Simulator CLI")
    parser.add_argument("--headless", action="store_true",
                        help="Run without GUI, output CSV log only")
    parser.add_argument("--seed",  type=int,   default=42)
    parser.add_argument("--time",  type=float, default=60.0)
    parser.add_argument("--out",   type=str,   default="logs/sim_log.csv",
                        help="Output CSV path (headless mode)")
    args = parser.parse_args()

    if args.headless:
        run_headless(args.seed, args.time, args.out)
    else:
        # Chuyển sang GUI mode
        from sim.app import main as gui_main
        gui_main()


if __name__ == "__main__":
    main()
