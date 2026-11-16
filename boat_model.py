#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
boat_model.py
Data-driven 3-DOF (x, y, heading) dynamics model of the Haesucheon boat, fitted from telemetry logs.

[Model] (state: x east m, y north m, psi heading rad clockwise from north, u surge m/s, r yaw rate rad/s)
- Surge:  du/dt = (U(esc) - u) / T  -  c_turn * |r| * u
          U(esc) is the steady speed for a throttle pulse (piecewise linear), T = T_acc or T_dec.
- Yaw:    dr/dt = (r_ss - r) / T_r   (first-order Nomoto model)
          r_ss = kappa(servo) * (u * (1 + a_u * |u|) + b_thrust * thrust_norm) + yaw_bias * thrust_norm * u
          kappa(servo) is the turn curvature (1/m) for a servo pulse (piecewise linear, saturating).
          yaw_bias: throttle-dependent turn curvature (1/m) with the rudder centred. It differs between logs
          (14:38 +7 deg/s, 17:13 +16 deg/s, 17:27 -1 deg/s at full throttle), so it is fitted per session.
- Kinematics: dx/dt = u sin(psi) + drift_x,  dy/dt = u cos(psi) + drift_y
          drift is the water current + mean wind push for that session (m/s).
- Actuator delay: servo/ESC commands take effect servo_delay/esc_delay control cycles later.

All PWM values are the physical pulses sent to the PCA9685 (servo_us, esc_us in the logs), so the model
is independent of calibration.json and of the RC transmitter mapping.
"""

import csv
import json
import math
import os

import numpy as np

from multipath_boat import wrap_180, NAV_PARAMS

DT = 0.05                                    # Control loop period (20 Hz), the same as multipath_boat.py
GPS_LATENCY_SEC = 0.15                       # Fix time -> logged row, measured from gps_utc (median 149 ms)
REF_LAT, REF_LON = 35.07310917, 129.0829184  # Center path waypoint #1, the navigator's ENU origin

# Piecewise-linear knots (us). Values at these knots are fitted parameters.
SERVO_KNOTS = [1030.0, 1250.0, 1750.0, 2130.0]
ESC_KNOTS = [1650.0, 1800.0, 1830.0, 1890.0, 1950.0, 1990.0]
ESC_NEUTRAL_US = 1800.0
ESC_FWD_SPAN_US = 190.0

PARAM_NAMES = [
    "kappa_1030", "kappa_1250", "kappa_1750", "kappa_2130",   # Turn curvature (1/m), + = right turn
    "T_yaw",                                                  # Yaw time constant (s)
    "U_1650", "U_1830", "U_1890", "U_1950", "U_1990",         # Steady surge speed (m/s); U(1800) = 0
    "T_acc", "T_dec", "c_turn"                                 # Surge time constants (s) and turn drag
]


class BoatModel:
    def __init__(self, params, servo_delay=2, esc_delay=2):
        self.p = dict(params)
        self.servo_delay = int(servo_delay)
        self.esc_delay = int(esc_delay)

    # ---------------- Parameter (de)serialization ----------------
    @classmethod
    def load(cls, path="boat_model_params.json"):
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        return cls(d["params"], d["servo_delay"], d["esc_delay"])

    def save(self, path, extra=None):
        d = {"params": self.p, "servo_delay": self.servo_delay, "esc_delay": self.esc_delay}
        if extra:
            d.update(extra)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(d, f, indent=2)

    # ---------------- Actuator maps ----------------
    def kappa(self, servo_us):
        vals = [self.p["kappa_1030"], self.p["kappa_1250"], self.p["kappa_1750"], self.p["kappa_2130"]]
        return np.interp(servo_us, SERVO_KNOTS, vals)

    def steady_speed(self, esc_us):
        vals = [self.p["U_1650"], 0.0, self.p["U_1830"], self.p["U_1890"], self.p["U_1950"], self.p["U_1990"]]
        return np.interp(esc_us, ESC_KNOTS, vals)

    # ---------------- Dynamics ----------------
    def step(self, state, servo_us, esc_us, drift=(0.0, 0.0), speed_scale=1.0, yaw_bias=0.0, dt=DT):
        """
        Advance one control cycle. Works on scalars or numpy arrays (vectorized over many boats).
        state = (x, y, psi, u, r); servo_us/esc_us are the commands already delayed by the caller.
        speed_scale multiplies the throttle map (battery charge differs between sessions).
        yaw_bias is the session's throttle-dependent turn curvature (1/m) with the rudder centred.
        """
        x, y, psi, u, r = state
        p = self.p
        thrust_n = np.clip((esc_us - ESC_NEUTRAL_US) / ESC_FWD_SPAN_US, 0.0, 1.0)

        U = speed_scale * self.steady_speed(esc_us)
        T = np.where(U > u, p["T_acc"], p["T_dec"])
        du = (U - u) / T - p["c_turn"] * np.abs(r) * u

        # yaw_speed_gain / yaw_thrust_gain: fixed at 0 since 2026-11-16 (unidentifiable next to yaw_bias)
        r_ss = (self.kappa(servo_us) * (u * (1.0 + p.get("yaw_speed_gain", 0.0) * np.abs(u))
                                        + p.get("yaw_thrust_gain", 0.0) * thrust_n)
                + yaw_bias * thrust_n * u)
        dr = (r_ss - r) / p["T_yaw"]

        x = x + (u * np.sin(psi) + drift[0]) * dt
        y = y + (u * np.cos(psi) + drift[1]) * dt
        psi = psi + r * dt
        u = u + du * dt
        r = r + dr * dt
        return (x, y, psi, u, r)

    def rollout(self, state0, servo_seq, esc_seq, drift=(0.0, 0.0), speed_scale=1.0, yaw_bias=0.0):
        """
        Open-loop simulation with command sequences of shape (N, H) (or (H,)).
        servo_seq/esc_seq must already include the actuator delay (see delayed_commands()).
        Returns positions of shape (N, H + 1, 2) and final state.
        """
        servo_seq = np.atleast_2d(servo_seq)
        esc_seq = np.atleast_2d(esc_seq)
        state = tuple(np.atleast_1d(np.asarray(s, dtype=float)) for s in state0)
        n, h = servo_seq.shape
        out = np.empty((n, h + 1, 2))
        out[:, 0, 0], out[:, 0, 1] = state[0], state[1]
        for k in range(h):
            state = self.step(state, servo_seq[:, k], esc_seq[:, k], drift, speed_scale, yaw_bias)
            out[:, k + 1, 0], out[:, k + 1, 1] = state[0], state[1]
        return out, state


def delayed_commands(series, start_idx, horizon, delay):
    """Slice commands [start - delay, start - delay + horizon) from a full log series, clamped at index 0."""
    idx = np.arange(start_idx, start_idx + horizon) - delay
    return series[np.clip(idx, 0, len(series) - 1)]


# =====================================================================
# Telemetry log loading and state estimation
# =====================================================================

class LogRun:
    """
    One telemetry CSV resampled onto the 20 Hz control grid.
    - x_gps/y_gps: raw GPS position at its measurement time (row time - GPS_LATENCY_SEC), interpolated (m)
    - x_s/y_s, speed, course, yaw_rate: estimates for initializing simulations; speed/course come from the
      receiver's Doppler (gps_sog_mps/gps_cog_deg) when logged, else from smoothed positions
    - fix_quality: GGA fix quality per row
    """

    def __init__(self, path):
        from scipy.signal import savgol_filter

        self.path = path
        self.name = os.path.basename(path)
        with open(path, "r", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        if not rows:
            raise ValueError(f"Empty log: {path}")

        t = np.array([float(r["timestamp"]) for r in rows])
        self.t = t - t[0]
        self.servo = np.array([float(r["servo_us"]) for r in rows])
        self.esc = np.array([float(r["esc_us"]) for r in rows])
        self.mode = np.array([r["mode"] for r in rows])
        self.fix_quality = np.array([int(r["fix_quality"]) for r in rows])
        lat = np.array([float(r["latitude"]) if r["latitude"] else np.nan for r in rows])
        lon = np.array([float(r["longitude"]) if r["longitude"] else np.nan for r in rows])

        # GPS fixes arrive at ~5 Hz; the log repeats the last fix between updates.
        new_fix = np.r_[False, (np.diff(lat) != 0) | (np.diff(lon) != 0)] & np.isfinite(lat)
        fix_idx = np.where(new_fix)[0]
        if len(fix_idx) < 10:
            raise ValueError(f"Too few GPS fixes in {path}")
        ex, ny = self._enu(lat[fix_idx], lon[fix_idx])
        self.fix_t = self.t[fix_idx] - GPS_LATENCY_SEC
        self.fix_x, self.fix_y = ex, ny
        self.x_gps = np.interp(self.t, self.fix_t, ex)
        self.y_gps = np.interp(self.t, self.fix_t, ny)

        w = 21  # 1.0 s smoothing window
        self.x_s = savgol_filter(self.x_gps, w, 2)
        self.y_s = savgol_filter(self.y_gps, w, 2)
        vx = savgol_filter(self.x_gps, w, 2, deriv=1, delta=DT)
        vy = savgol_filter(self.y_gps, w, 2, deriv=1, delta=DT)
        self.speed = np.hypot(vx, vy)
        self.course = np.arctan2(vx, vy)
        self.has_doppler = "gps_sog_mps" in rows[0]
        if self.has_doppler:
            sog = np.array([float(rows[i]["gps_sog_mps"] or "nan") for i in fix_idx])
            cog = np.radians([float(rows[i]["gps_cog_deg"] or "nan") for i in fix_idx])
            ok = np.isfinite(sog)
            self.speed = np.interp(self.t, self.fix_t[ok], sog[ok])
            ok = np.isfinite(cog) & (sog > 0.3)
            if ok.sum() > 10:
                cog_t = np.interp(self.t, self.fix_t[ok], np.unwrap(cog[ok]))
                moving = self.speed > 0.3
                self.course = np.where(moving, cog_t, self.course)
        self.yaw_rate = savgol_filter(np.unwrap(self.course), 31, 2, deriv=1, delta=DT)

    @staticmethod
    def _enu(lat, lon):
        dlat = np.radians(lat - REF_LAT)
        dlon = np.radians(lon - REF_LON)
        return dlon * 6_378_137.0 * math.cos(math.radians(REF_LAT)), dlat * 6_378_137.0

    def index_at(self, t_sec):
        return int(np.searchsorted(self.t, t_sec))

    def state_at(self, i):
        """Smoothed (x, y, psi, u, r) estimate at grid index i; only meaningful when moving forward."""
        return (self.x_s[i], self.y_s[i], self.course[i], self.speed[i], self.yaw_rate[i])


# =====================================================================
# GPS emulator: reproduces what ZEDF9PReceiver gives the navigator
# =====================================================================

class GPSEmulator:
    """
    Emulates the ZED-F9P at 5 Hz: Gaussian position noise + optional slowly wandering bias,
    and the same movement-based heading estimator as ZEDF9PReceiver._update_movement_heading.
    """

    def __init__(self, rate_hz=5.0, noise_m=0.02, bias_walk_m=0.0, seed=0, vel_noise_mps=0.025, heading_source=None):
        self.period = 1.0 / rate_hz
        self.noise_m = noise_m
        self.bias_walk_m = bias_walk_m
        self.vel_noise = vel_noise_mps        # Doppler velocity noise per axis (2026-11-16 log: ~0.035 m/s fix-to-fix)
        self.heading_source = heading_source or NAV_PARAMS.get("heading_source", "blend")
        self.rng = np.random.default_rng(seed)
        self.bias = np.zeros(2)
        self.next_t = 0.0
        self.x = self.y = None
        self.sog = 0.0
        self.cog = None
        self._anchor = None
        self.heading_deg = None
        self._prev_fix = None

    def reset_heading_anchor(self):
        self._anchor = None if self.x is None else (self.x, self.y)
        self.heading_deg = None

    def update(self, t, true_x, true_y, true_vx, true_vy):
        """Call every control cycle; returns True when a new fix was produced."""
        if t + 1e-9 < self.next_t:
            return False
        self.next_t += self.period
        if self.bias_walk_m > 0:
            self.bias += self.rng.normal(0.0, self.bias_walk_m * math.sqrt(self.period), 2)
        self.x = true_x + self.bias[0] + self.rng.normal(0.0, self.noise_m)
        self.y = true_y + self.bias[1] + self.rng.normal(0.0, self.noise_m)

        # RMC speed/course from the receiver's own (Doppler) velocity: low noise.
        vx = true_vx + self.rng.normal(0.0, self.vel_noise)
        vy = true_vy + self.rng.normal(0.0, self.vel_noise)
        self.sog = math.hypot(vx, vy)
        self.cog = (math.degrees(math.atan2(vx, vy)) + 360.0) % 360.0

        if self.heading_source == "cog" and self._anchor is not None and self.sog >= NAV_PARAMS["sog_cog_blend_mps"]:
            self.heading_deg = self.cog
            self._anchor = (self.x, self.y)
            return True

        if self._anchor is None:
            self._anchor = (self.x, self.y)
            return True
        moved = math.hypot(self.x - self._anchor[0], self.y - self._anchor[1])
        if moved >= NAV_PARAMS["min_heading_move_m"]:
            vec = (math.degrees(math.atan2(self.x - self._anchor[0], self.y - self._anchor[1])) + 360.0) % 360.0
            self._anchor = (self.x, self.y)
            if self.cog is not None and self.sog >= NAV_PARAMS["sog_cog_blend_mps"]:
                self.heading_deg = (vec + 0.5 * wrap_180(self.cog - vec) + 360.0) % 360.0
            else:
                self.heading_deg = vec
        return True

    def latlon(self):
        lat = REF_LAT + math.degrees(self.y / 6_378_137.0)
        lon = REF_LON + math.degrees(self.x / (6_378_137.0 * math.cos(math.radians(REF_LAT))))
        return lat, lon
