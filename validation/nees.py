"""Normalised Estimation Error Squared (NEES) test.

NEES at time k for run i:
    ε_k = δx_k^T P_k^{-1} δx_k

Under correct filter assumptions, ε_k ~ χ²(n) where n = dim(δx).
For N runs, the time-averaged NEES is:
    ε̄_k = (1/N) Σ_i ε_{k,i}   →  E[ε̄_k] = n

Confidence bounds (95%):
    χ²(Nn, 0.025) / N  ≤  ε̄_k  ≤  χ²(Nn, 0.975) / N
"""
import numpy as np
from scipy.stats import chi2
from typing import List, Tuple
from numpy.typing import NDArray
from .monte_carlo import RunResult


def nees_statistic(err: NDArray, P_diag: NDArray) -> NDArray:
    """Compute scalar NEES from error vector and diagonal covariance.

    Parameters
    ----------
    err    : (T, n) array of estimation errors
    P_diag : (T, n) array of covariance diagonal entries (σ²)

    Returns
    -------
    nees : (T,) array of NEES values
    """
    var = np.maximum(P_diag, 1e-12)
    return np.sum(err**2 / var, axis=1)


def nees_bounds(n_dim: int, n_runs: int, alpha: float = 0.05) -> Tuple[float, float]:
    """Chi-square confidence bounds for time-averaged NEES.

    Returns (lower, upper) bounds such that a consistent filter
    satisfies: lower ≤ mean_NEES ≤ upper with probability 1 - alpha.
    """
    dof = n_dim * n_runs
    lo  = chi2.ppf(alpha / 2,   dof) / n_runs
    hi  = chi2.ppf(1 - alpha/2, dof) / n_runs
    return lo, hi


def nees_report(results: List[RunResult]) -> dict:
    """Compute NEES statistics from Monte Carlo results.

    Returns a summary dict with per-axis NEES and pass/fail for 95% CI.
    The first 5 seconds are excluded (filter convergence transient).
    Pass criterion: ≥ 70% of post-convergence time points within bounds.
    """
    N = len(results)
    T = min(len(r.time) for r in results)

    # Stack errors across runs: (N, T, 3)
    p_errs = np.array([r.p_err[:T] for r in results])    # (N,T,3)
    v_errs = np.array([r.v_err[:T] for r in results])
    p_stds = np.array([r.pos_std[:T] for r in results])  # (N,T,3)  σ not σ²
    v_stds = np.array([r.vel_std[:T] for r in results])

    # Per-run NEES
    nees_p_runs = np.array([
        nees_statistic(p_errs[i], p_stds[i]**2) for i in range(N)
    ])   # (N, T)
    nees_v_runs = np.array([
        nees_statistic(v_errs[i], v_stds[i]**2) for i in range(N)
    ])

    # Time-averaged over all runs
    avg_nees_p = nees_p_runs.mean(axis=0)   # (T,)
    avg_nees_v = nees_v_runs.mean(axis=0)

    lo_p, hi_p = nees_bounds(3, N)
    lo_v, hi_v = nees_bounds(3, N)

    # Exclude first 5 seconds (convergence transient) from pass/fail
    time = results[0].time[:T]
    mask = time >= 5.0
    if mask.sum() > 0:
        frac_in_p = float(np.mean((avg_nees_p[mask] >= lo_p) & (avg_nees_p[mask] <= hi_p)))
        frac_in_v = float(np.mean((avg_nees_v[mask] >= lo_v) & (avg_nees_v[mask] <= hi_v)))
    else:
        frac_in_p = float(np.mean((avg_nees_p >= lo_p) & (avg_nees_p <= hi_p)))
        frac_in_v = float(np.mean((avg_nees_v >= lo_v) & (avg_nees_v <= hi_v)))

    return {
        "time":       time,
        "nees_pos":   avg_nees_p,
        "nees_vel":   avg_nees_v,
        "bounds_pos": (lo_p, hi_p),
        "bounds_vel": (lo_v, hi_v),
        "frac_in_pos": frac_in_p,
        "frac_in_vel": frac_in_v,
        "pass_pos":   frac_in_p >= 0.70,   # practical threshold for ESKF
        "pass_vel":   frac_in_v >= 0.50,   # velocity harder due to delay
        "n_runs": N,
    }
