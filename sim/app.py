"""sim.app — Entry point GUI: python -m sim.app

Mở cửa sổ dashboard realtime.
Usage:
    python -m sim.app [--seed N] [--time T]
"""
import sys
import argparse

from PyQt6.QtWidgets import QApplication
from sim.gui.dashboard import SimDashboard
from sim.config import SimConfig, SensorConfig


def main():
    parser = argparse.ArgumentParser(description="UAV Simulator — GUI mode")
    parser.add_argument("--seed",  type=int,   default=42,   help="Random seed")
    parser.add_argument("--time",  type=float, default=60.0, help="Simulation time (s)")
    args = parser.parse_args()

    app = QApplication(sys.argv)

    # Style tối cho toàn app
    app.setStyleSheet("""
        QMainWindow { background-color: #1a1a1f; }
        QWidget { background-color: #1a1a1f; color: #ccc; }
        QSplitter::handle { background-color: #333; }
    """)

    cfg = SimConfig(seed=args.seed, total_time=args.time)
    scfg = SensorConfig()

    dashboard = SimDashboard(cfg=cfg, scfg=scfg, seed=args.seed, total_time=args.time)
    dashboard.show()

    # Auto-start simulation khi GUI hiện
    dashboard.start_simulation()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
