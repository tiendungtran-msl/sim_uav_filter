"""Random Mission Generator for UAV Simulator.

Produces a sequence of flight phases with randomized parameters.
The EKF has NO access to this information — it only receives sensor data.

Phases:
  hover        — constant altitude hold
  coordinated_turn  — banked turn at constant altitude
  aggressive_yaw    — rapid heading change
  spiral_climb — climbing helix
  wind_disturbance  — sudden body-frame force perturbation
  gps_dropout  — GPS signal lost for random duration
"""
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
from .config import SimConfig
from .dynamics import UAV6DOF, quat_mult, quat_normalize


@dataclass
class Phase:
    kind: str
    duration: float       # seconds
    params: dict


class MissionGenerator:
    def __init__(self, cfg: SimConfig, rng: np.random.Generator):
        self.cfg = cfg
        self.rng = rng
        self.phases: List[Phase] = []
        self._phase_idx = 0
        self._phase_elapsed = 0.0
        self._wind = np.zeros(3)      # wind disturbance in NED (m/s)
        self._gps_active = True        # GPS availability flag
        self._generate_mission()

    # ──────────────────────────────────────────────────────────────────────
    def _generate_mission(self):
        """Build a randomised sequence of phases summing to total_time."""
        phase_types = [
            "hover",
            "coordinated_turn",
            "aggressive_yaw",
            "spiral_climb",
            "wind_disturbance",
            "gps_dropout",
        ]
        # Always start with a 5s warmup hover so the UAV reaches steady state
        # and the filter can initialise GPS-aided before aggressive manoeuvres.
        warmup_alt = float(self.rng.uniform(40, 70))
        self.phases.append(Phase("hover", 8.0, {"alt": warmup_alt}))
        remaining = self.cfg.total_time - 8.0

        while remaining > 2.0:
            kind = self.rng.choice(phase_types)
            dur = float(self.rng.uniform(
                max(2.0, remaining * 0.05),
                min(remaining, remaining * 0.25 + 5.0)
            ))
            dur = min(dur, remaining)
            params = self._random_params(kind)
            self.phases.append(Phase(kind, dur, params))
            remaining -= dur
        # Always end with a hover
        self.phases.append(Phase("hover", max(remaining, 2.0), {}))

    def _random_params(self, kind: str) -> dict:
        r = self.rng
        if kind == "hover":
            return {"alt": float(r.uniform(30, 120))}
        elif kind == "coordinated_turn":
            return {
                "bank_angle": float(r.uniform(15, 45)) * np.pi / 180,
                "turn_rate":  float(r.uniform(0.1, 0.4)),  # rad/s
                "direction":  int(r.choice([-1, 1])),
            }
        elif kind == "aggressive_yaw":
            return {"yaw_rate": float(r.uniform(0.5, 1.5)) * r.choice([-1, 1])}
        elif kind == "spiral_climb":
            return {
                "climb_rate": float(r.uniform(1.0, 4.0)),
                "turn_rate":  float(r.uniform(0.1, 0.3)),
            }
        elif kind == "wind_disturbance":
            return {"wind_ned": (r.uniform(-8, 8, size=3)).tolist()}
        elif kind == "gps_dropout":
            return {"dropout_duration": float(self.rng.uniform(3.0, 5.0))}
        return {}

    # ──────────────────────────────────────────────────────────────────────
    def apply(self, uav: UAV6DOF) -> bool:
        """Apply current mission phase to UAV actuator setpoints.
        Returns False when GPS should be considered unavailable.
        """
        dt = self.cfg.dt
        if self._phase_idx >= len(self.phases):
            uav.thrust_body = np.array([0, 0, -self.cfg.gravity * self.cfg.mass])
            uav.torque_body = np.zeros(3)
            return True

        phase = self.phases[self._phase_idx]
        self._dispatch(phase, uav)

        self._phase_elapsed += dt
        if self._phase_elapsed >= phase.duration:
            self._phase_elapsed = 0.0
            self._phase_idx += 1
            self._gps_active = True   # reset GPS on new phase

        return self._gps_active

    def _dispatch(self, phase: Phase, uav: UAV6DOF):
        g = self.cfg.gravity
        m = self.cfg.mass
        max_thrust = 4.0 * m * g   # 4g = 58.9 N maximum thrust magnitude

        def clamp_thrust(t: float) -> float:
            return float(np.clip(t, -max_thrust, max_thrust))

        if phase.kind == "hover":
            target_alt = -phase.params.get("alt", 50.0)   # NED z (negative = up)
            alt_err = target_alt - uav.p[2]   # positive → UAV is above target
            vel_err = 0.0 - uav.v[2]          # positive → UAV moving downward
            # PD controller on altitude: desired a_z = Kp*alt_err + Kd*vel_err
            # Clamp desired acceleration to ±5 m/s² (roughly ±0.5 g) to prevent
            # extreme manoeuvres that overwhelm the EKF.
            a_z_des = np.clip(1.5 * alt_err + 2.0 * vel_err, -5.0, 5.0)
            thrust_z = clamp_thrust(-m * (g - a_z_des))
            uav.thrust_body = np.array([0.0, 0.0, thrust_z])
            uav.torque_body = -2.0 * uav.omega

        elif phase.kind == "coordinated_turn":
            bank   = phase.params["bank_angle"]
            rate_z = phase.params["turn_rate"] * phase.params["direction"]
            # Maintain altitude + bank
            thrust_z = float(np.clip(-m * g / max(np.cos(bank), 0.3),
                                     -max_thrust, max_thrust))
            uav.thrust_body = np.array([0.0, 0.0, thrust_z])
            omega_des = np.array([0.0, 0.0, rate_z])
            uav.torque_body = 0.5 * (omega_des - uav.omega)

        elif phase.kind == "aggressive_yaw":
            yaw_rate = phase.params["yaw_rate"]
            uav.thrust_body = np.array([0.0, 0.0, -m * g])
            omega_des = np.array([0.0, 0.0, yaw_rate])
            uav.torque_body = 2.0 * (omega_des - uav.omega)

        elif phase.kind == "spiral_climb":
            climb = phase.params["climb_rate"]   # desired upward speed (m/s)
            tr    = phase.params["turn_rate"]
            # v_z_desired = -climb (NED: negative z = upward)
            # a_z_des = Kd*(v_z_des - v_z) = Kd*(-climb - v_z)
            a_z_des = np.clip(2.0 * (-climb - uav.v[2]), -5.0, 5.0)
            thrust_ned_z = clamp_thrust(-m * (g - a_z_des))
            uav.thrust_body = np.array([0.0, 0.0, thrust_ned_z])
            omega_des = np.array([0.0, 0.0, tr])
            uav.torque_body = 0.5 * (omega_des - uav.omega)

        elif phase.kind == "wind_disturbance":
            wind = np.array(phase.params["wind_ned"], dtype=float)
            vel_err = uav.v - wind
            drag_force = -0.3 * vel_err * m
            thrust_raw = np.array([0.0, 0.0, -m * g]) + drag_force
            thrust_raw = np.clip(thrust_raw, -max_thrust, max_thrust)
            uav.thrust_body = thrust_raw
            uav.torque_body = -1.0 * uav.omega

        elif phase.kind == "gps_dropout":
            drop_dur = phase.params["dropout_duration"]
            if self._phase_elapsed < drop_dur:
                self._gps_active = False
            else:
                self._gps_active = True
            uav.thrust_body = np.array([0.0, 0.0, -m * g])
            uav.torque_body = -2.0 * uav.omega
