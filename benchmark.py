#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
benchmark.py
Compare AUTO control methods (classical, tuned, RL, ...) for straight-line race speed on short, validated
windows of the fitted boat simulator, with paired statistics and robustness to wind/current/trim.

[Why short windows]
The simulator is only trusted where it has been checked against held-out real data. `window` measures, on
held-out logs, how well the simulator predicts the quantity we score (distance made good along the course)
at each horizon, and recommends the longest horizon where that error stays small. Every episode then starts
from a realistic state and is scored only over that window, so model error cannot accumulate.

[Score]
progress_m = distance made good along the course centreline in the window (more = faster).
A boat more than CORRIDOR_M from the centreline counts as hitting the bank: its progress stops there and
CRASH_PENALTY_M is subtracted. Two episode kinds:
- start : from rest on the start line (race start, AUTO entry), scored from t = 0
- cruise: already moving somewhere along the course with a random offset/heading error
Estimated race time = start window + remaining distance / cruise speed made good.

[Statistics]
- Common random numbers: every controller sees exactly the same scenarios (state, wind, current, rudder
  trim, battery, gusts, GPS noise), so differences are paired.
- 95% bootstrap confidence intervals of paired differences against the best controller; win rate.
- Robustness: mean and CVaR10 (mean of the worst 10% of episodes), crash rate.
- Model uncertainty: the same comparison under every fitted model in MODEL_ENSEMBLE. A ranking is only
  reported as defensible if the paired 95% CI excludes zero under every model.
- Tuning (cem) uses TUNE scenarios; compare uses separate TEST scenarios, so tuned controllers are not
  scored on the situations they were tuned on.

Usage:
    python3 benchmark.py window                     # validate and choose the window length
    python3 benchmark.py compare --n 300            # compare controllers (built-ins + tuned, if any)
    python3 benchmark.py cem --iters 12             # tune a full-throttle PI pursuit controller (CEM)

Add a controller: subclass Controller (reset/act) and register it in CONTROLLERS, or use RaceEnv (a
gym-style reset/step environment with the same scenarios and score) to train an RL policy.
"""

import argparse
import json
import math
import os

import numpy as np

import multipath_boat as mb
from boat_model import BoatModel, GPSEmulator, DT, GPS_LATENCY_SEC

MODEL_ENSEMBLE = ["boat_model_params.json", "boat_model_params_holdout_run1.json", "boat_model_params_v1.json"]
WINDOW_SEC = 8.0             # Scored window: longest validated by `benchmark.py window` (8-9 s on 2026-11-16 data)
WINDOW_TOL_MEDIAN = 0.05     # Max median relative error of predicted progress on held-out data
WINDOW_TOL_P90 = 0.15        # Max 90th-percentile relative error
MODEL_REL_ERR = 0.025        # Validated median relative progress error at the 8 s window (held-out data)
CORRIDOR_M = 3.0             # Bank distance from the centreline (lanes are at +-1.5 m); an assumption
CRASH_PENALTY_M = 10.0
START_FRACTION = 0.25        # Share of start-from-rest episodes
TUNED_FILE = "benchmark_tuned.json"
OUT_DIR = "sim_results"


# =====================================================================
# Course geometry (straight 92 m centreline, lanes at -1.5 / 0 / +1.5 m)
# =====================================================================

class Course:
    def __init__(self):
        paths = {lane: mb.load_waypoints_csv(p) for lane, p in mb.PATH_FILES.items()}
        ref = paths["center"][0]
        self.lines = {lane: np.array([mb.latlon_to_enu(lat, lon, ref[1], ref[2]) for _, lat, lon in pts])
                      for lane, pts in paths.items()}
        c = self.lines["center"]
        seg = np.diff(c, axis=0)
        self.seg_len = np.hypot(seg[:, 0], seg[:, 1])
        self.tan = seg / self.seg_len[:, None]
        self.normal = np.stack([self.tan[:, 1], -self.tan[:, 0]], axis=1)   # Right-hand normal
        self.cum = np.r_[0.0, np.cumsum(self.seg_len)]
        self.a = c[:-1]
        self.length = float(self.cum[-1])
        self.lane_offset = {lane: float(np.mean(np.sum((self.lines[lane][:-1] - self.a) * self.normal, axis=1)))
                            for lane in self.lines}

    def project(self, x, y):
        """Return (s along centreline, lateral offset + right, segment bearing rad) of a point."""
        d = np.stack([x - self.a[:, 0], y - self.a[:, 1]], axis=1)
        along = np.clip(np.sum(d * self.tan, axis=1), 0.0, self.seg_len)
        foot = self.a + along[:, None] * self.tan
        dist = np.hypot(x - foot[:, 0], y - foot[:, 1])
        i = int(np.argmin(dist))
        # Extend beyond the ends so progress keeps counting past the start/finish lines.
        if i == 0 or i == len(self.seg_len) - 1:
            along_i = float(np.dot(d[i], self.tan[i]))
        else:
            along_i = float(along[i])
        lateral = float(np.dot(d[i], self.normal[i]))
        return float(self.cum[i] + along_i), lateral, math.atan2(self.tan[i, 0], self.tan[i, 1])

    def point(self, s, lateral=0.0):
        i = int(np.clip(np.searchsorted(self.cum, s) - 1, 0, len(self.seg_len) - 1))
        p = self.a[i] + (s - self.cum[i]) * self.tan[i] + lateral * self.normal[i]
        return float(p[0]), float(p[1])


# =====================================================================
# Scenarios: disturbances spanning everything fitted from the logs
# =====================================================================

def disturbance_ranges(meta):
    yb = list(meta.get("yaw_bias_by_session", {}).values()) or [0.0]
    sc = list(meta.get("speed_scale_by_session", {}).values()) or [1.0]
    dr = np.array(list(meta.get("drift_by_session", {}).values()) or [[0.0, 0.0]])
    return {"yaw_bias": (min(yb) - 0.02, max(yb) + 0.02), "speed_scale": (min(sc), max(sc)), "drift": dr}


def sample_scenarios(n, seed, course, meta):
    rng = np.random.default_rng(seed)
    rngs = disturbance_ranges(meta)
    out = []
    for k in range(n):
        kind = "start" if rng.random() < START_FRACTION else "cruise"
        lane = rng.choice(["left", "center", "right"])
        off = course.lane_offset[lane]
        if kind == "start":
            s0, lat0, herr, u0, r0 = 0.5, off + rng.normal(0, 0.3), rng.normal(0, 15.0), 0.0, 0.0
        else:
            s0 = rng.uniform(5.0, course.length - 25.0)
            lat0, herr = off + rng.normal(0, 0.4), rng.normal(0, 8.0)
            u0, r0 = rng.uniform(0.9, 1.5), math.radians(rng.normal(0, 3.0))
        x0, y0 = course.point(s0, lat0)
        _, _, bearing = course.project(x0, y0)
        dgps = rng.random() < 0.2
        out.append({
            "id": k, "kind": kind, "lane": str(lane), "target_lateral": off, "dgps": bool(dgps),
            "state0": (x0, y0, bearing + math.radians(herr), u0, r0),
            "yaw_bias": rng.uniform(*rngs["yaw_bias"]),
            "speed_scale": rng.uniform(*rngs["speed_scale"]),
            "drift": tuple(rngs["drift"][rng.integers(len(rngs["drift"]))] + rng.normal(0, 0.05, 2)),
            "gust_deg": rng.uniform(0.0, 6.0),
            "gps_noise": 0.05 if dgps else 0.02, "gps_bias_walk": 0.03 if dgps else 0.0,
            "seed": int(rng.integers(1 << 31)),
        })
    return out


# =====================================================================
# Episode: same plant, actuator delay, GPS (5 Hz, 0.15 s latency) and heading estimate as the boat
# =====================================================================

class Episode:
    def __init__(self, sc, model, course, cal, window_sec=None, heading_source=None):
        self.sc, self.model, self.course = sc, model, course
        self.pwm = cal["pwm"]
        self.n_steps = int(round((window_sec or WINDOW_SEC) / DT))
        self.rng = np.random.default_rng(sc["seed"])
        self.state = tuple(sc["state0"])
        x, y, psi, u, _ = self.state
        vx, vy = u * math.sin(psi) + sc["drift"][0], u * math.cos(psi) + sc["drift"][1]
        self.gps = GPSEmulator(noise_m=sc["gps_noise"], bias_walk_m=sc["gps_bias_walk"], seed=sc["seed"],
                               heading_source=heading_source, fix_quality=2 if sc.get("dgps") else 4)
        self.gps.update(0.0, x, y, vx, vy)
        self.gps.reset_heading_anchor()     # As on AUTO entry: heading unknown until 0.25 m of movement
        self.gps_hist = [(x, y, vx, vy)] * (int(round(GPS_LATENCY_SEC / DT)) + 1)
        self.servo_hist = None
        self.esc_hist = None
        self.gust = 0.0
        self.t = 0.0
        self.k = 0
        self.s0 = course.project(x, y)[0]
        self.crashed = False
        self.steer_prev = 0.0
        self.steer_activity = 0.0
        self.lat_abs = []

    def obs(self):
        g = self.gps
        lat, lon = g.latlon()
        cog = g.cog if (g.cog is not None and g.sog >= 0.3) else None
        return {"t": self.t, "x": g.x, "y": g.y, "lat": lat, "lon": lon, "heading_deg": g.heading_deg,
                "sog": g.sog, "cog_deg": cog, "course": self.course, "target_lateral": self.sc["target_lateral"],
                "gps": g}

    def step(self, steer_norm, throttle):
        """steer_norm in [-1, 1] (+ = right), throttle in [0, 1] (0 = ESC neutral, 1 = esc_fwd_max_us)."""
        p = self.pwm
        steer_norm = max(-1.0, min(1.0, steer_norm))
        throttle = max(0.0, min(1.0, throttle))
        servo = mb.map_norm_to_pwm(steer_norm, p["servo_center_us"], p["servo_left_us"], p["servo_right_us"])
        esc = p["esc_neutral_us"] + throttle * (p["esc_fwd_max_us"] - p["esc_neutral_us"])
        if self.servo_hist is None:
            self.servo_hist = [servo] * (self.model.servo_delay + 1)
            self.esc_hist = [esc] * (self.model.esc_delay + 1)
        self.servo_hist.append(servo)
        self.esc_hist.append(esc)
        sc = self.sc
        self.state = self.model.step(self.state, self.servo_hist.pop(0), self.esc_hist.pop(0), drift=sc["drift"],
                                     speed_scale=sc["speed_scale"], yaw_bias=sc["yaw_bias"])
        tau = 2.0
        sig = math.radians(sc["gust_deg"])
        self.gust += -self.gust / tau * DT + sig * math.sqrt(2.0 * DT / tau) * self.rng.standard_normal()
        x, y, psi, u, r = self.state
        self.state = (x, y, psi + self.gust * DT, u, r)
        self.t += DT
        self.k += 1
        vx, vy = u * math.sin(psi) + sc["drift"][0], u * math.cos(psi) + sc["drift"][1]
        self.gps_hist.append((x, y, vx, vy))
        self.gps.update(self.t, *self.gps_hist.pop(0))
        s, lateral, _ = self.course.project(x, y)
        self.lat_abs.append(abs(lateral - sc["target_lateral"]))
        self.steer_activity += abs(steer_norm - self.steer_prev)
        self.steer_prev = steer_norm
        if abs(lateral) > CORRIDOR_M:
            self.crashed = True
        return s - self.s0, self.crashed or self.k >= self.n_steps

    def result(self):
        s = self.course.project(self.state[0], self.state[1])[0]
        progress = s - self.s0 - (CRASH_PENALTY_M if self.crashed else 0.0)
        return {"progress_m": progress, "crashed": self.crashed, "mean_abs_lat_m": float(np.mean(self.lat_abs)),
                "steer_activity": self.steer_activity, "kind": self.sc["kind"]}


def run_episode(controller, sc, model, course, cal, window_sec=None):
    ep = Episode(sc, model, course, cal, window_sec, getattr(controller, "heading_source", None))
    controller.reset(ep.obs(), cal)
    done = False
    while not done:
        steer, thr = controller.act(ep.obs())
        _, done = ep.step(steer, thr)
    return ep.result()


# =====================================================================
# Controllers
# =====================================================================

class Controller:
    """Interface: reset(obs, cal) at AUTO entry; act(obs) -> (steer_norm [-1,1], throttle [0,1]) at 20 Hz.
    obs has only what the boat has: GPS x/y (ENU m, 5 Hz, 0.15 s old), lat/lon, the boat's movement-based
    heading_deg (None until 0.25 m moved), Doppler sog/cog_deg, the Course geometry and the target lane."""
    name = "base"

    def reset(self, obs, cal):
        pass

    def act(self, obs):
        raise NotImplementedError


class PursuitController(Controller):
    """Pure pursuit on a straight line (+ optional integral on heading error, Doppler course, adaptive throttle)."""

    def __init__(self, name, throttle="full", cruise=0.6, lookahead_m=3.5, max_steer_deg=35.0, slew=0.18,
                 ki=0.0, use_cog=False, cte_gate=0.30, herr_gate=11.0):
        self.name = name
        self.p = dict(throttle=throttle, cruise=cruise, lookahead_m=lookahead_m, max_steer_deg=max_steer_deg,
                      slew=slew, ki=ki, use_cog=use_cog, cte_gate=cte_gate, herr_gate=herr_gate)

    def reset(self, obs, cal):
        self.prev = 0.0
        self.integ = 0.0

    def act(self, obs):
        p = self.p
        heading = obs["heading_deg"]
        if p["use_cog"] and obs["cog_deg"] is not None and obs["sog"] >= 0.45:
            heading = obs["cog_deg"]
        course = obs["course"]
        s, lateral, _ = course.project(obs["x"], obs["y"])
        tx, ty = course.point(s + p["lookahead_m"], obs["target_lateral"])
        bearing = math.degrees(math.atan2(tx - obs["x"], ty - obs["y"]))
        if heading is None:
            raw, herr = 0.0, None
        else:
            herr = mb.wrap_180(bearing - heading)
            if p["ki"] > 0:
                self.integ = max(-p["max_steer_deg"] / p["ki"],
                                 min(p["max_steer_deg"] / p["ki"], self.integ + herr * DT))
            raw = (herr + p["ki"] * self.integ) / p["max_steer_deg"]
        raw = max(-1.0, min(1.0, raw))
        steer = self.prev + max(-p["slew"], min(p["slew"], raw - self.prev))
        self.prev = steer
        if p["throttle"] == "full":
            thr = 1.0
        elif p["throttle"] == "adaptive":
            ok = herr is not None and abs(lateral - obs["target_lateral"]) <= p["cte_gate"] and abs(herr) <= p["herr_gate"]
            thr = 1.0 if ok else p["cruise"]
        else:
            thr = p["cruise"]
        return steer, thr


class NavigatorController(Controller):
    """The boat's real MultipathNavigator and AUTO strategy, called as the main loop does (simulate_boat.AutoDriver,
    including the GPS-course heading selector). strategy/options override calibration.json's autonomous section;
    cruise_ratio overrides pwm.auto_throttle_ratio; heading_source ("blend"/"cog") selects the legacy receiver
    heading estimate (NAV_PARAMS["heading_source"]) emulated for strategies 01/02."""

    def __init__(self, name="boat_code", strategy=None, options=None, heading_source=None, cruise_ratio=None):
        self.name = name
        self.strategy, self.options, self.heading_source = strategy, options or {}, heading_source
        self.cruise_ratio = cruise_ratio

    def reset(self, obs, cal):
        from simulate_boat import AutoDriver, make_navigator
        paths = {lane: mb.load_waypoints_csv(p) for lane, p in mb.PATH_FILES.items()}
        if self.strategy is not None:
            cal = dict(cal, autonomous={"strategy": self.strategy, self.strategy: dict(self.options)})
        self.nav = make_navigator(mb.MultipathNavigator, paths, cal)
        self.driver = AutoDriver(self.nav)
        self.driver.enter(obs["gps"], obs["t"], moving=obs["sog"] >= 0.45)
        self.cruise = cal["pwm"]["auto_throttle_ratio"] if self.cruise_ratio is None else self.cruise_ratio

    def act(self, obs):
        info = self.driver.update(obs["gps"], obs["t"])
        if info["finished"]:
            return 0.0, 0.0
        blend = max(0.0, min(1.0, info.get("throttle_blend", 0.0)))
        return info["steer_norm"], self.cruise + blend * (1.0 - self.cruise)


def builtin_controllers():
    ctrls = [
        NavigatorController("boat_code configured (calibration.json)"),
        NavigatorController("01_constant_cruise @0.6", "01_constant_cruise", cruise_ratio=0.6),
        NavigatorController("02_adaptive_throttle @0.6 (run A)", "02_adaptive_throttle", cruise_ratio=0.6),
        NavigatorController("02_adaptive_throttle @1.0 (run B)", "02_adaptive_throttle", cruise_ratio=1.0),
        NavigatorController("03_gps_course_progressive @1.0", "03_gps_course_progressive", cruise_ratio=1.0),
        NavigatorController("04_predictive_recovery @1.0", "04_predictive_recovery", cruise_ratio=1.0),
        NavigatorController("05_straight_line", "05_straight_line"),
        NavigatorController("05_straight_line no integral", "05_straight_line", {"course_integral_gain": 0.0}),
        PursuitController("cruise_60 (constant 60%)", throttle="cruise"),
        PursuitController("adaptive (full if |CTE|<=0.3 & |herr|<=11)", throttle="adaptive"),
        PursuitController("full (always 100%)", throttle="full"),
        PursuitController("full_pi (100% + integral)", throttle="full", ki=0.4),
        PursuitController("full_pi_cog (100% + integral + Doppler course)", throttle="full", ki=0.4, use_cog=True),
    ]
    if os.path.exists(TUNED_FILE):
        with open(TUNED_FILE, "r", encoding="utf-8") as f:
            tuned = json.load(f)
        ctrls.append(PursuitController("cem_tuned (" + ", ".join(f"{k}={v:.2f}" for k, v in tuned.items()
                                                                     if isinstance(v, float)) + ")",
                                       throttle="full", use_cog=True, **tuned))
    return ctrls


# =====================================================================
# RL-style environment (no external dependencies; wrap with gymnasium if wanted)
# =====================================================================

class RaceEnv:
    """reset(seed) -> obs vector; step([steer_norm, throttle]) -> (obs, reward, done, info).
    Reward = progress along the course in this 0.05 s step (m); -CRASH_PENALTY_M on hitting the bank.
    Observation = [lateral error m, heading error deg/90 (0 if unknown), sog/2, prev steer, prev throttle]."""

    def __init__(self, model_file=MODEL_ENSEMBLE[0], window_sec=None, seed=0):
        self.model = BoatModel.load(model_file)
        with open(model_file, "r", encoding="utf-8") as f:
            self.meta = json.load(f)
        self.course = Course()
        self.cal = mb.load_calibration(mb.DEFAULT_CALIBRATION_FILE)
        self.window_sec = window_sec
        self.rng = np.random.default_rng(seed)

    def _vec(self):
        o = self.ep.obs()
        s, lateral, path = self.course.project(o["x"], o["y"])
        h = o["cog_deg"] if o["cog_deg"] is not None else o["heading_deg"]
        herr = 0.0 if h is None else mb.wrap_180(math.degrees(path) - h) / 90.0
        return np.array([lateral - o["target_lateral"], herr, o["sog"] / 2.0, self.prev[0], self.prev[1]])

    def reset(self, seed=None):
        sc = sample_scenarios(1, int(self.rng.integers(1 << 31)) if seed is None else seed, self.course, self.meta)[0]
        self.ep = Episode(sc, self.model, self.course, self.cal, self.window_sec)
        self.prev = (0.0, 0.0)
        self.last_progress = 0.0
        return self._vec()

    def step(self, action):
        steer, thr = float(action[0]), float(action[1])
        progress, done = self.ep.step(steer, thr)
        reward = progress - self.last_progress - (CRASH_PENALTY_M if self.ep.crashed else 0.0)
        self.last_progress = progress
        self.prev = (steer, thr)
        return self._vec(), reward, done, {"crashed": self.ep.crashed}


# =====================================================================
# Window validation on held-out real data
# =====================================================================

def cmd_window(args):
    import fit_boat_model as F
    F.HORIZON_SEC = 20.0
    F.EVAL_HORIZONS = [2.0, 5.0, 10.0, 20.0]
    with open(MODEL_ENSEMBLE[0], "r", encoding="utf-8") as f:
        meta = json.load(f)
    F.AUTO_HOLDOUT = meta.get("auto_holdout", F.AUTO_HOLDOUT)
    model = BoatModel.load(MODEL_ENSEMBLE[0])
    runs, sessions, win, blocks, h, marks = F.build_windows()
    env = (np.array([meta["drift_by_session"][s] for s in sessions]),
           np.array([meta["speed_scale_by_session"][s] for s in sessions]),
           np.array([meta["yaw_bias_by_session"][s] for s in sessions]))
    course = Course()
    wins = win["test"] + win["auto_test"]          # Held-out data only
    st0, servo, esc, sess, target = F.stack(runs, wins, h, model.servo_delay, model.esc_delay)
    pos = F.simulate_windows(model, env, st0, servo, esc, sess)
    s_start = np.array([course.project(x, y)[0] for x, y in zip(st0[0], st0[1])])

    print(f"Held-out windows: {len(wins)} (TEST blocks + AUTO run never used for fitting)")
    print("Prediction error of distance made good along the course, on straight-running windows")
    print(f"  {'window':>7s} {'n':>4s} {'real progress':>14s} {'median |err|':>13s} {'p90 |err|':>10s} {'median rel':>11s} {'p90 rel':>8s}")
    best = None
    for hz in np.arange(2.0, 20.01, 1.0):
        k = int(round(hz / DT))
        j = int(np.argmin(np.abs(marks - k)))
        real_s = np.array([course.project(x, y)[0] for x, y in target[:, j, :]]) - s_start
        sim_s = np.array([course.project(x, y)[0] for x, y in pos[:, marks[j], :]]) - s_start
        path = np.array([np.sum(np.hypot(*np.diff(np.r_[[[a, b]], target[i, :j + 1, :]], axis=0).T))
                         for i, (a, b) in enumerate(zip(st0[0], st0[1]))])
        straight = (np.abs(real_s) >= 0.7 * path) & (path > 0.5 * hz)
        err = np.abs(sim_s - real_s)[straight]
        rel = err / np.maximum(np.abs(real_s[straight]), 1e-6)
        if len(err) < 10:
            continue
        ok = np.median(rel) <= WINDOW_TOL_MEDIAN and np.percentile(rel, 90) <= WINDOW_TOL_P90
        if ok:
            best = hz
        print(f"  {hz:6.0f}s {len(err):4d} {np.median(np.abs(real_s[straight])):12.1f} m {np.median(err):11.2f} m "
              f"{np.percentile(err, 90):8.2f} m {100 * np.median(rel):9.1f}% {100 * np.percentile(rel, 90):6.1f}%"
              f"{'  ok' if ok else ''}")
    print(f"\nLongest window meeting median <= {100 * WINDOW_TOL_MEDIAN:.0f}% and p90 <= {100 * WINDOW_TOL_P90:.0f}%: "
          f"{best} s  (WINDOW_SEC is {WINDOW_SEC:.0f} s)")


# =====================================================================
# Comparison
# =====================================================================

def bootstrap_ci(x, rng, n_boot=2000):
    idx = rng.integers(0, len(x), (n_boot, len(x)))
    m = x[idx].mean(axis=1)
    return float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5))


def evaluate(controllers, scenarios, model_file, course, cal):
    model = BoatModel.load(model_file)
    res = {}
    for c in controllers:
        res[c.name] = [run_episode(c, sc, model, course, cal) for sc in scenarios]
    return res


def race_time(res_list, course, window_sec):
    start = np.array([r["progress_m"] for r in res_list if r["kind"] == "start"])
    cruise = np.array([r["progress_m"] for r in res_list if r["kind"] == "cruise"])
    v = cruise.mean() / window_sec
    return window_sec + (course.length - start.mean()) / v


def cmd_compare(args):
    course = Course()
    cal = mb.load_calibration(mb.DEFAULT_CALIBRATION_FILE)
    with open(MODEL_ENSEMBLE[0], "r", encoding="utf-8") as f:
        meta = json.load(f)
    scenarios = sample_scenarios(args.n, args.seed + 1_000_000, course, meta)   # TEST seeds (tuning uses others)
    controllers = builtin_controllers()
    rng = np.random.default_rng(0)
    models = MODEL_ENSEMBLE if args.ensemble else MODEL_ENSEMBLE[:1]
    print(f"[COMPARE] {len(scenarios)} paired TEST scenarios ({sum(s['kind'] == 'start' for s in scenarios)} start, "
          f"{sum(s['kind'] == 'cruise' for s in scenarios)} cruise), window {WINDOW_SEC:.0f} s, corridor +-{CORRIDOR_M} m")
    rngs = disturbance_ranges(meta)
    print(f"          rudder trim yaw_bias {rngs['yaw_bias'][0]:+.2f}..{rngs['yaw_bias'][1]:+.2f} 1/m, battery scale "
          f"{rngs['speed_scale'][0]:.2f}..{rngs['speed_scale'][1]:.2f}, current/wind drift from {len(rngs['drift'])} logs "
          f"+-0.05 m/s, gusts 0-6 deg/s, 20% DGPS")
    summary = {}
    for mf in models:
        res = evaluate(controllers, scenarios, mf, course, cal)
        prog = {k: np.array([r["progress_m"] for r in v]) for k, v in res.items()}
        best = max(prog, key=lambda k: prog[k].mean())
        print(f"\n=== model {mf} ===")
        print(f"  {'controller':48s} {'progress':>9s} {'CVaR10':>7s} {'crash':>6s} {'|lat|':>6s} {'race est':>9s} "
              f"{'diff vs best [95% CI]':>24s} {'win%':>5s}")
        for name in sorted(prog, key=lambda k: -prog[k].mean()):
            p = prog[name]
            worst = np.sort(p)[:max(1, len(p) // 10)].mean()
            crash = np.mean([r["crashed"] for r in res[name]])
            lat = np.mean([r["mean_abs_lat_m"] for r in res[name]])
            t_race = race_time(res[name], course, WINDOW_SEC)
            separable = None
            if name == best:
                diff_txt, win = "best", ""
            else:
                d = p - prog[best]
                lo, hi = bootstrap_ci(d, rng)
                # Defensibly worse: CI excludes 0 AND the gap exceeds the simulator's validated error.
                separable = bool(hi < 0 and -d.mean() > MODEL_REL_ERR * prog[best].mean())
                diff_txt = f"{d.mean():+.2f} [{lo:+.2f},{hi:+.2f}]{'*' if separable else ' '}"
                win = f"{100 * np.mean(d > 0):4.0f}%"
            print(f"  {name[:48]:48s} {p.mean():8.2f}m {worst:6.2f}m {100 * crash:5.1f}% {lat:5.2f}m {t_race:8.1f}s "
                  f"{diff_txt:>24s} {win:>5s}")
            summary.setdefault(name, []).append({"model": mf, "mean": float(p.mean()), "cvar10": float(worst),
                                                 "crash": float(crash), "race_s": float(t_race),
                                                 "best": name == best, "separable_from_best": separable})
        print(f"  * = defensibly behind the best: 95% CI excludes 0 and gap > {100 * MODEL_REL_ERR:.1f}% "
              f"validated simulator error ({MODEL_REL_ERR * prog[best].mean():.2f} m)")
    if len(models) > 1:
        print("\n=== ranking agreement across models (by mean progress) ===")
        ranks = {n: [sorted(summary, key=lambda k: -summary[k][i]["mean"]).index(n) + 1 for i in range(len(models))]
                 for n in summary}
        for n in sorted(ranks, key=lambda k: sum(ranks[k])):
            verdict = ("top under every model" if all(e["best"] for e in summary[n]) else
                       "defensibly behind under every model" if all(e["separable_from_best"] for e in summary[n]) else
                       "not separable from the best")
            print(f"  {n[:48]:48s} ranks {ranks[n]}  race est " +
                  " / ".join(f"{e['race_s']:.1f}s" for e in summary[n]) + f"  -> {verdict}")
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, "benchmark_summary.json"), "w", encoding="utf-8") as f:
        json.dump({"window_sec": WINDOW_SEC, "n": len(scenarios), "summary": summary}, f, indent=2)


# =====================================================================
# Cross-entropy method tuning (derivative-free policy search) on TUNE scenarios
# =====================================================================

def cmd_cem(args):
    course = Course()
    cal = mb.load_calibration(mb.DEFAULT_CALIBRATION_FILE)
    with open(MODEL_ENSEMBLE[0], "r", encoding="utf-8") as f:
        meta = json.load(f)
    scenarios = sample_scenarios(args.n, args.seed, course, meta)                 # TUNE seeds
    model = BoatModel.load(MODEL_ENSEMBLE[0])
    names = ["lookahead_m", "max_steer_deg", "slew", "ki"]
    lo = np.array([1.5, 15.0, 0.05, 0.0])
    hi = np.array([8.0, 70.0, 0.50, 1.5])
    mu = np.array([3.5, 35.0, 0.18, 0.4])
    sd = (hi - lo) / 4
    rng = np.random.default_rng(args.seed)

    def score(x):
        c = PursuitController("cand", throttle="full", use_cog=True, **dict(zip(names, map(float, x))))
        p = np.array([run_episode(c, sc, model, course, cal)["progress_m"] for sc in scenarios])
        return 0.5 * p.mean() + 0.5 * np.sort(p)[:max(1, len(p) // 10)].mean()     # Speed + robustness (CVaR10)

    best = (score(mu), mu.copy())
    print(f"[CEM] {len(scenarios)} TUNE scenarios, objective = 0.5*mean + 0.5*CVaR10 progress; start {best[0]:.3f}")
    for it in range(args.iters):
        X = np.clip(rng.normal(mu, sd, (args.pop, len(mu))), lo, hi)
        S = np.array([score(x) for x in X])
        elite = X[np.argsort(-S)[:max(2, args.pop // 4)]]
        mu, sd = elite.mean(axis=0), np.maximum(elite.std(axis=0), (hi - lo) * 0.02)
        if S.max() > best[0]:
            best = (float(S.max()), X[int(np.argmax(S))].copy())
        print(f"  iter {it + 1:2d}: best {S.max():.3f}  mean {S.mean():.3f}  mu " +
              ", ".join(f"{n}={v:.2f}" for n, v in zip(names, mu)))
    tuned = {n: float(v) for n, v in zip(names, best[1])}
    with open(TUNED_FILE, "w", encoding="utf-8") as f:
        json.dump(tuned, f, indent=2)
    print(f"[SAVED] {TUNED_FILE}: {tuned}  (score {best[0]:.3f}); evaluate with `compare` on TEST scenarios")


def main():
    global WINDOW_SEC
    parser = argparse.ArgumentParser(description="Benchmark AUTO controllers on validated simulator windows")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("window", help="Validate the window length on held-out real data")
    p = sub.add_parser("compare", help="Paired comparison of controllers on TEST scenarios")
    p.add_argument("--n", type=int, default=300)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--window", type=float, default=WINDOW_SEC)
    p.add_argument("--no-ensemble", dest="ensemble", action="store_false")
    p = sub.add_parser("cem", help="Tune a full-throttle pursuit/PI controller by cross-entropy method")
    p.add_argument("--n", type=int, default=60)
    p.add_argument("--iters", type=int, default=10)
    p.add_argument("--pop", type=int, default=16)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--window", type=float, default=WINDOW_SEC)
    args = parser.parse_args()
    if getattr(args, "window", None):
        WINDOW_SEC = args.window
    {"window": cmd_window, "compare": cmd_compare, "cem": cmd_cem}[args.cmd](args)


if __name__ == "__main__":
    main()
