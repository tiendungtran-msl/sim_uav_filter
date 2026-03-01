"""EKF2 Tracking & Validation Dashboard (Plotly-based).

Produces an interactive HTML report with:
  1. 3D trajectory   (true / GPS meas / estimated)
  2. Position error  ± 3σ bounds
  3. Velocity error  ± 3σ bounds
  4. Attitude error  ± 3σ bounds
  5. GPS innovation + NIS
  6. Gyro/Accel bias evolution
  7. NEES across Monte Carlo runs
"""
import os
import numpy as np
from typing import Optional, List
import plotly.graph_objects as go
import plotly.subplots as sp
from plotly.subplots import make_subplots

from validation.monte_carlo import RunResult
from validation.nees import nees_report


class EKFDashboard:
    def __init__(self, out_dir: str = "outputs"):
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)

    # ── Single-run dashboard ──────────────────────────────────────────────

    def plot_single_run(
        self,
        time:    np.ndarray,
        p_true:  np.ndarray,    # (T,3)
        p_meas:  Optional[np.ndarray],   # (K,3) GPS measurements (sparse)
        p_est:   np.ndarray,    # (T,3)
        p_std:   np.ndarray,    # (T,3)
        v_true:  np.ndarray,
        v_est:   np.ndarray,
        v_std:   np.ndarray,
        att_err_deg: np.ndarray,  # (T,3)
        att_std_deg: np.ndarray,  # (T,3)
        innov_gps: list,
        innov_baro: list,
        title: str = "EKF2 Single Run",
        filename: str = "single_run.html",
    ):
        fig = make_subplots(
            rows=4, cols=2,
            subplot_titles=[
                "Position Error (NED) [m]",
                "Velocity Error (NED) [m/s]",
                "Attitude Error [deg]",
                "Bias: Gyro [rad/s]  (placeholder)",
                "GPS Innovation — Position [m]",
                "GPS Innovation — Velocity [m/s]",
                "Normalised Innovation Squared (NIS)",
                "Baro Innovation [m]",
            ],
            shared_xaxes=False,
        )
        axes = ["N", "E", "D"]
        colors = ["#E63946", "#457B9D", "#2A9D8F"]

        # ── Row 1 col 1: position error ────────────────────────────────────
        p_err = p_est - p_true
        for i, (ax, c) in enumerate(zip(axes, colors)):
            fig.add_trace(go.Scatter(x=time, y=p_err[:, i], name=f"p_err_{ax}",
                                     line=dict(color=c, width=1.2)), row=1, col=1)
            fig.add_trace(go.Scatter(
                x=np.concatenate([time, time[::-1]]),
                y=np.concatenate([3*p_std[:, i], -3*p_std[:, i][::-1]]),
                fill="toself", fillcolor=c, opacity=0.12,
                line=dict(width=0), showlegend=False), row=1, col=1)

        # ── Row 1 col 2: velocity error ────────────────────────────────────
        v_err = v_est - v_true
        for i, (ax, c) in enumerate(zip(axes, colors)):
            fig.add_trace(go.Scatter(x=time, y=v_err[:, i], name=f"v_err_{ax}",
                                     line=dict(color=c, width=1.2)), row=1, col=2)
            fig.add_trace(go.Scatter(
                x=np.concatenate([time, time[::-1]]),
                y=np.concatenate([3*v_std[:, i], -3*v_std[:, i][::-1]]),
                fill="toself", fillcolor=c, opacity=0.12,
                line=dict(width=0), showlegend=False), row=1, col=2)

        # ── Row 2 col 1: attitude error ────────────────────────────────────
        att_names = ["Roll", "Pitch", "Yaw"]
        for i, (ax, c) in enumerate(zip(att_names, colors)):
            fig.add_trace(go.Scatter(x=time, y=att_err_deg[:, i], name=f"att_{ax}",
                                     line=dict(color=c, width=1.2)), row=2, col=1)
            fig.add_trace(go.Scatter(
                x=np.concatenate([time, time[::-1]]),
                y=np.concatenate([3*att_std_deg[:, i], -3*att_std_deg[:, i][::-1]]),
                fill="toself", fillcolor=c, opacity=0.12,
                line=dict(width=0), showlegend=False), row=2, col=1)

        # ── GPS innovations ────────────────────────────────────────────────
        if innov_gps:
            t_g = np.array([x[0] for x in innov_gps])
            i_g = np.array([x[1] for x in innov_gps])
            S_g = np.array([x[2] for x in innov_gps])
            for i, (ax, c) in enumerate(zip(axes, colors)):
                fig.add_trace(go.Scatter(x=t_g, y=i_g[:, i], name=f"gps_innov_p_{ax}",
                                         line=dict(color=c, width=0.8)), row=3, col=1)
                fig.add_trace(go.Scatter(x=t_g, y=i_g[:, 3+i], name=f"gps_innov_v_{ax}",
                                         line=dict(color=c, width=0.8, dash="dot")),
                               row=3, col=2)

            # NIS (normalised innovation squared)
            nis_list = []
            for k in range(len(i_g)):
                try:
                    s = S_g[k]
                    if np.any(~np.isfinite(s)) or np.any(~np.isfinite(i_g[k])):
                        nis_list.append(np.nan)
                    else:
                        nis_list.append(float(i_g[k] @ np.linalg.pinv(s) @ i_g[k]))
                except Exception:
                    nis_list.append(np.nan)
            nis = np.array(nis_list)
            fig.add_trace(go.Scatter(x=t_g, y=nis, name="NIS",
                                     line=dict(color="#F4A261")), row=4, col=1)
            fig.add_hline(y=12.592, line_dash="dash", line_color="red",
                          annotation_text="χ²(6, 95%)", row=4, col=1)

        # ── Baro innovations ───────────────────────────────────────────────
        if innov_baro:
            t_b = np.array([x[0] for x in innov_baro])
            i_b = np.array([x[1] for x in innov_baro])
            fig.add_trace(go.Scatter(x=t_b, y=i_b, name="baro_innov",
                                     line=dict(color="#8ECAE6")), row=4, col=2)
            fig.add_hline(y=0, line_dash="dash", line_color="gray", row=4, col=2)

        fig.update_layout(
            title=dict(text=title, font=dict(size=16)),
            height=1100,
            template="plotly_dark",
            legend=dict(x=1.01, y=1, font=dict(size=9)),
        )
        fig.update_xaxes(title_text="Time (s)")
        path = os.path.join(self.out_dir, filename)
        fig.write_html(path)
        print(f"  Saved: {path}")
        return fig

    # ── 3D trajectory plot ────────────────────────────────────────────────

    def plot_3d_trajectory(
        self,
        p_true: np.ndarray,
        p_est:  np.ndarray,
        p_meas: Optional[np.ndarray] = None,
        filename: str = "trajectory_3d.html",
    ):
        fig = go.Figure()

        fig.add_trace(go.Scatter3d(
            x=p_true[:, 0], y=p_true[:, 1], z=-p_true[:, 2],  # altitude +
            mode="lines",
            name="Ground Truth",
            line=dict(color="lime", width=2),
        ))
        fig.add_trace(go.Scatter3d(
            x=p_est[:, 0], y=p_est[:, 1], z=-p_est[:, 2],
            mode="lines",
            name="EKF2 Estimate",
            line=dict(color="cyan", width=2, dash="dot"),
        ))
        if p_meas is not None and len(p_meas) > 0:
            fig.add_trace(go.Scatter3d(
                x=p_meas[:, 0], y=p_meas[:, 1], z=-p_meas[:, 2],
                mode="markers",
                name="GPS Measurements",
                marker=dict(color="orange", size=3, symbol="circle"),
            ))

        fig.update_layout(
            title="UAV 3D Trajectory",
            template="plotly_dark",
            scene=dict(
                xaxis_title="North (m)",
                yaxis_title="East (m)",
                zaxis_title="Altitude (m)",
                aspectmode="auto",
            ),
            height=700,
        )
        path = os.path.join(self.out_dir, filename)
        fig.write_html(path)
        print(f"  Saved: {path}")
        return fig

    # ── Monte Carlo NEES dashboard ─────────────────────────────────────────

    def plot_monte_carlo(
        self,
        results: List[RunResult],
        filename: str = "monte_carlo.html",
    ):
        rep = nees_report(results)
        T   = len(rep["time"])
        lo_p, hi_p = rep["bounds_pos"]
        lo_v, hi_v = rep["bounds_vel"]

        # Aggregate error envelopes
        n     = len(results)
        p_err = np.array([r.p_err[:T] for r in results])
        v_err = np.array([r.v_err[:T] for r in results])

        p_rms = np.sqrt((p_err**2).mean(axis=0))   # (T,3)
        v_rms = np.sqrt((v_err**2).mean(axis=0))

        fig = make_subplots(rows=3, cols=2,
                             subplot_titles=[
                                 "RMS Position Error [m]",
                                 "RMS Velocity Error [m/s]",
                                 "NEES — Position",
                                 "NEES — Velocity",
                                 "Per-run Position Error N",
                                 "Per-run Velocity Error N",
                             ])
        time = rep["time"]
        axes = ["N", "E", "D"]
        colors = ["#E63946", "#457B9D", "#2A9D8F"]

        for i, (ax, c) in enumerate(zip(axes, colors)):
            fig.add_trace(go.Scatter(x=time, y=p_rms[:, i],
                                     name=f"RMS_p_{ax}", line=dict(color=c)), row=1, col=1)
            fig.add_trace(go.Scatter(x=time, y=v_rms[:, i],
                                     name=f"RMS_v_{ax}", line=dict(color=c)), row=1, col=2)

        # NEES position
        fig.add_trace(go.Scatter(x=time, y=rep["nees_pos"],
                                  name="NEES_pos", line=dict(color="cyan")), row=2, col=1)
        fig.add_hline(y=lo_p, line_dash="dash", line_color="orange", row=2, col=1)
        fig.add_hline(y=hi_p, line_dash="dash", line_color="orange", row=2, col=1)
        fig.add_hline(y=3.0, line_dash="dot", line_color="white",
                      annotation_text="E[NEES]=3", row=2, col=1)

        # NEES velocity
        fig.add_trace(go.Scatter(x=time, y=rep["nees_vel"],
                                  name="NEES_vel", line=dict(color="lime")), row=2, col=2)
        fig.add_hline(y=lo_v, line_dash="dash", line_color="orange", row=2, col=2)
        fig.add_hline(y=hi_v, line_dash="dash", line_color="orange", row=2, col=2)

        # Individual run traces (transparent)
        for r in results[:20]:  # show first 20 only to avoid clutter
            T_r = len(r.time)
            fig.add_trace(go.Scatter(
                x=r.time, y=r.p_err[:T_r, 0],
                mode="lines", line=dict(color="rgba(100,180,255,0.15)", width=0.5),
                showlegend=False,
            ), row=3, col=1)
            fig.add_trace(go.Scatter(
                x=r.time, y=r.v_err[:T_r, 0],
                mode="lines", line=dict(color="rgba(180,255,100,0.15)", width=0.5),
                showlegend=False,
            ), row=3, col=2)

        pass_p = "PASS" if rep["pass_pos"] else "FAIL"
        pass_v = "PASS" if rep["pass_vel"] else "FAIL"
        fig.update_layout(
            title=(f"Monte Carlo ({n} runs) — "
                   f"NEES-pos {pass_p}  NEES-vel {pass_v}  "
                   f"(in-bounds pos {rep['frac_in_pos']*100:.0f}%  "
                   f"vel {rep['frac_in_vel']*100:.0f}%)"),
            height=1000,
            template="plotly_dark",
        )
        path = os.path.join(self.out_dir, filename)
        fig.write_html(path)
        print(f"  Saved: {path}")
        return fig, rep
