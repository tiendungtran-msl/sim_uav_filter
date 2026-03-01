"""UAV 3D Animation (Plotly animated scatter3d).

Creates an HTML file with a frame-by-frame animation of the UAV trajectory.
Shows:
  - Green ribbon: ground truth
  - Cyan marker:  EKF2 estimate (current position)
  - Orange dots:  GPS measurements
  - Red sphere:   covariance ellipsoid (simplified as axes)
"""
import os
import numpy as np
import plotly.graph_objects as go


class UAVAnimation3D:
    def __init__(self, out_dir: str = "outputs"):
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)

    def animate(
        self,
        time:   np.ndarray,   # (T,)
        p_true: np.ndarray,   # (T,3) NED
        p_est:  np.ndarray,   # (T,3) NED
        p_std:  np.ndarray,   # (T,3) 1-sigma
        gps_t:  np.ndarray,   # (K,)
        gps_pos: np.ndarray,  # (K,3) NED
        downsample: int = 10,
        filename: str = "uav_animation.html",
    ):
        # Subsample for animation
        idx  = np.arange(0, len(time), downsample)
        t_s  = time[idx]
        pt_s = p_true[idx]
        pe_s = p_est[idx]
        ps_s = p_std[idx]

        frames = []
        trail_len = 50   # number of past positions shown

        for fi, i in enumerate(range(len(t_s))):
            start = max(0, i - trail_len)

            trail_true = pt_s[start:i+1]
            trail_est  = pe_s[start:i+1]

            # Covariance axes at current position (scaled 3×σ)
            pos = pe_s[i]
            sig = ps_s[i] * 3.0
            # 3 axes lines: x-line, y-line, z-line
            ex = [pos[0]-sig[0], pos[0]+sig[0], None]
            ey = [pos[1]-sig[1], pos[1]+sig[1], None]
            ez_alt = [pos[2]    , pos[2]        , None]   # alt = -z

            frame_data = [
                # True trail
                go.Scatter3d(
                    x=trail_true[:, 0], y=trail_true[:, 1], z=-trail_true[:, 2],
                    mode="lines", line=dict(color="lime", width=2),
                    name="True", showlegend=fi==0,
                ),
                # Estimated trail
                go.Scatter3d(
                    x=trail_est[:, 0], y=trail_est[:, 1], z=-trail_est[:, 2],
                    mode="lines", line=dict(color="cyan", width=2, dash="dot"),
                    name="EKF2", showlegend=fi==0,
                ),
                # Current estimated position (big marker)
                go.Scatter3d(
                    x=[pos[0]], y=[pos[1]], z=[-pos[2]],
                    mode="markers",
                    marker=dict(color="cyan", size=6, symbol="diamond"),
                    name="EKF2 now", showlegend=fi==0,
                ),
                # GPS measurements (historical)
                go.Scatter3d(
                    x=gps_pos[:, 0], y=gps_pos[:, 1], z=-gps_pos[:, 2],
                    mode="markers",
                    marker=dict(color="orange", size=2, opacity=0.5),
                    name="GPS", showlegend=fi==0,
                ),
                # Covariance ellipsoid axes (N and E only)
                go.Scatter3d(
                    x=ex, y=[pos[1]]*2 + [None], z=[-pos[2]]*2 + [None],
                    mode="lines", line=dict(color="red", width=3),
                    name="σ_N", showlegend=False,
                ),
                go.Scatter3d(
                    x=[pos[0]]*2 + [None], y=ey, z=[-pos[2]]*2 + [None],
                    mode="lines", line=dict(color="magenta", width=3),
                    name="σ_E", showlegend=False,
                ),
            ]
            frames.append(go.Frame(
                data=frame_data,
                name=f"{t_s[i]:.1f}s",
                layout=go.Layout(title_text=f"UAV Flight  t = {t_s[i]:.1f} s"),
            ))

        # Initial frame
        init_data = frames[0].data if frames else []

        fig = go.Figure(
            data=init_data,
            frames=frames,
            layout=go.Layout(
                title="UAV 3D Trajectory Animation — EKF2",
                template="plotly_dark",
                scene=dict(
                    xaxis_title="North (m)",
                    yaxis_title="East (m)",
                    zaxis_title="Altitude (m)",
                    aspectmode="auto",
                ),
                height=700,
                updatemenus=[dict(
                    type="buttons",
                    showactive=False,
                    y=0.0, x=0.5, xanchor="center",
                    pad=dict(t=10),
                    buttons=[
                        dict(label="▶ Play",
                             method="animate",
                             args=[None, dict(
                                 frame=dict(duration=50, redraw=True),
                                 fromcurrent=True,
                                 transition=dict(duration=0),
                             )]),
                        dict(label="⏸ Pause",
                             method="animate",
                             args=[[None], dict(
                                 frame=dict(duration=0, redraw=False),
                                 mode="immediate",
                             )]),
                    ],
                )],
                sliders=[dict(
                    steps=[
                        dict(method="animate",
                             args=[[f.name], dict(mode="immediate",
                                                   frame=dict(duration=50, redraw=True),
                                                   transition=dict(duration=0))],
                             label=f.name)
                        for f in frames
                    ],
                    active=0,
                    x=0.0, y=0.0,
                    len=1.0,
                )],
            ),
        )

        path = os.path.join(self.out_dir, filename)
        fig.write_html(path, auto_play=False)
        print(f"  Saved animation: {path}")
        return fig
