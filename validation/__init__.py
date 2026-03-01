"""Validation Package.

Provides:
  - MonteCarlo  : run N seeds, collect statistics
  - nees_check  : Normalised Estimation Error Squared test
  - InnovationAnalyzer : whiteness / zero-mean test

All functions are filter-agnostic: they accept lists of per-run data.
"""
from .monte_carlo import MonteCarlo, RunResult
from .nees import nees_statistic, nees_bounds, nees_report
from .innovation import InnovationAnalyzer

__all__ = [
    "MonteCarlo", "RunResult",
    "nees_statistic", "nees_bounds", "nees_report",
    "InnovationAnalyzer",
]
