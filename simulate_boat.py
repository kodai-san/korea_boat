#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simulate_boat.py
Haesucheon boat simulator built on the fitted dynamics model (boat_model_params.json).

[Modes]
1. replay: feed the servo/ESC pulses from a real log into the model, open-loop, and compare with the real GPS track.
     python3 simulate_boat.py replay logs/multipath_log_20261116_143810.csv --t0 60 --t1 120

2. auto: closed-loop run of the real AUTO controller (MultipathNavigator in multipath_boat.py, or any class with
   the same update() interface) against the simulated boat, emulated 5 Hz GPS, current and wind gusts.
     python3 simulate_boat.py auto
     python3 simulate_boat.py auto --runs 20 --gust 8 --start-offset 1.0
     python3 simulate_boat.py auto --controller my_nav:MyNavigator --throttle-ratio 0.7

   The AUTO strategy (steering/throttle law) is read from calibration.json "autonomous", exactly like the boat;
   add a strategy class in auto_strategies.py, or edit MultipathNavigator / NAV_PARAMS, and rerun.
   Each run writes a CSV in the same format as the real telemetry logs to sim_results/.

3. validate-auto: restart the simulator at the moment a real AUTO run started (same position, heading, speed,
   current and battery estimate) and run the controller closed-loop; compare with what the real boat did.
     python3 simulate_boat.py validate-auto logs/multipath_log_20261116_172709.csv --t0 48.2 --t1 112.6
"""

import argparse
import csv
import importlib
import json
import math
import os

import numpy as np

import multipath_boat as mb
from boat_model import BoatModel, LogRun, GPSEmulator, DT, GPS_LATENCY_SEC, delayed_commands
from gps_heading import MotionHeadingSelector

PARAMS_FILE = "boat_model_params.json"
OUT_DIR = "sim_results"
GUST_TAU_SEC = 2.0   # Correlation time of the yaw gust disturbance (s)


def load_model_and_env():
    with open(PARAMS_FILE, "r", encoding="utf-8") as f:
        d = json.load(f)
    return BoatModel.load(PARAMS_FILE), d


def session_env(meta, session, args=None):
    """Current, battery speed scale and rudder-centred turning bias fitted for a log (overridable by CLI)."""
    drift = meta["drift_by_session"].get(session, [0.0, 0.0])
    scale = meta["speed_scale_by_session"].get(session, 1.0)
    bias = meta.get("yaw_bias_by_session", {}).get(session, 0.0)
    if args is not None:
        drift = args.drift if args.drift is not None else drift
        scale = args.speed_scale if args.speed_scale is not None else scale
        bias = args.yaw_bias if args.yaw_bias is not None else bias
    return drift, scale, bias


# =====================================================================
# Replay mode
# =====================================================================

def run_replay(args):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    model, meta = load_model_and_env()
    run = LogRun(args.log)
    drift, scale, bias = session_env(meta, os.path.basename(args.log)[len("multipath_log_"):-len(".csv")], args)

    i0 = run.index_at(args.t0)
    i1 = run.index_at(args.t1) if args.t1 is not None else len(run.t) - 1
    x0, y0, psi0, u0, r0 = run.state_at(i0)
    if u0 < 0.3:
        print(f"[NOTE] Boat nearly stationary at t={args.t0}s ({u0:.2f} m/s): heading taken from the next movement.")
        j = i0
        while j < i1 and math.hypot(run.x_s[j] - x0, run.y_s[j] - y0) < 0.5:
            j += 1
        psi0 = math.atan2(run.x_s[j] - x0, run.y_s[j] - y0)
        u0, r0 = 0.0, 0.0

    h = i1 - i0
    servo = delayed_commands(run.servo, i0, h, model.servo_delay)
    esc = delayed_commands(run.esc, i0, h, model.esc_delay)
    pos, _ = model.rollout((x0, y0, psi0, u0, r0), servo, esc, drift=tuple(drift), speed_scale=scale, yaw_bias=bias)
    pos = pos[0]
    real = np.stack([run.x_gps[i0:i1 + 1], run.y_gps[i0:i1 + 1]], axis=1)
    err = np.hypot(*(pos - real).T)
    tt = run.t[i0:i1 + 1]

    print(f"[REPLAY] {run.name} t={run.t[i0]:.1f}-{run.t[i1]:.1f}s  drift={drift} speed_scale={scale:.2f}")
    for k in range(0, h + 1, int(round(5.0 / DT))):
        print(f"  t+{k * DT:5.1f}s  position error {err[k]:5.2f} m")
    print(f"  end      position error {err[-1]:5.2f} m  (path length {np.sum(np.hypot(*np.diff(real, axis=0).T)):.1f} m)")

    os.makedirs(OUT_DIR, exist_ok=True)
    fig = plt.figure(figsize=(14, 7))
    ax = fig.add_subplot(1, 2, 1)
    ax.plot(real[:, 0], real[:, 1], "k-", label="real (GPS)")
    ax.plot(pos[:, 0], pos[:, 1], "r--", label="simulated")
    for k in range(0, h + 1, int(round(5.0 / DT))):
        ax.plot([real[k, 0], pos[k, 0]], [real[k, 1], pos[k, 1]], ":", c="gray", lw=0.8)
    ax.set_aspect("equal", "datalim")
    ax.legend()
    ax.grid(alpha=0.3)
    ax.set_title(f"{run.name}  t={run.t[i0]:.0f}-{run.t[i1]:.0f}s (gray = 5 s ticks)")
    a2 = fig.add_subplot(3, 2, 2)
    a2.plot(tt, run.servo[i0:i1 + 1])
    a2.set_ylabel("servo us")
    a3 = fig.add_subplot(3, 2, 4, sharex=a2)
    a3.plot(tt, run.esc[i0:i1 + 1])
    a3.set_ylabel("esc us")
    a4 = fig.add_subplot(3, 2, 6, sharex=a2)
    a4.plot(tt, err)
    a4.set_ylabel("position error m")
    a4.set_xlabel("log time (s)")
    for a in (a2, a3, a4):
        a.grid(alpha=0.3)
    fig.tight_layout()
    out = os.path.join(OUT_DIR, f"replay_{run.name[14:29]}_{int(args.t0)}-{int(run.t[i1])}.png")
    fig.savefig(out, dpi=90)
    print(f"[SAVED] {out}")


# =====================================================================
# Closed-loop AUTO mode
# =====================================================================

def add_strategy_args(p):
    p.add_argument("--strategy", default=None, help="Override calibration.json autonomous.strategy")
    p.add_argument("--strategy-option", action="append", default=[], metavar="KEY=VALUE",
                   help="Strategy option override, e.g. heading_integral_gain=0.3 (repeatable)")
    p.add_argument("--heading-source", choices=["blend", "cog"], default=None,
                   help="Receiver heading estimate (NAV_PARAMS heading_source) to emulate")
    p.add_argument("--nav-param", action="append", default=[], metavar="KEY=VALUE",
                   help="NAV_PARAMS override, e.g. shift_cte_m=99 to hold the start lane (repeatable)")


def apply_strategy_args(cal, args):
    if getattr(args, "heading_source", None):
        mb.NAV_PARAMS["heading_source"] = args.heading_source
    for kv in getattr(args, "nav_param", []):
        k, v = kv.split("=", 1)
        mb.NAV_PARAMS[k] = float(v)
    if getattr(args, "strategy", None) or getattr(args, "strategy_option", None):
        auto = dict(cal.get("autonomous", {}))
        name = args.strategy or auto.get("strategy", "adaptive_throttle")
        opts = dict(auto.get(name, {}))
        for kv in args.strategy_option:
            k, v = kv.split("=", 1)
            opts[k] = float(v)
        cal = dict(cal, autonomous={"strategy": name, name: opts})
    return cal


def make_navigator(nav_cls, paths, cal):
    """Build the navigator with the AUTO strategy configured in calibration.json, as the boat does.
    Older multipath_boat.py versions (before auto_strategies.py) have the throttle law built in."""
    if not hasattr(mb, "create_auto_strategy"):
        return nav_cls(paths, mb.NAV_PARAMS)
    params = (mb.resolve_navigation_params(cal.get("navigation", {})) if hasattr(mb, "resolve_navigation_params")
              else mb.NAV_PARAMS)
    strategy = mb.create_auto_strategy(cal.get("autonomous", {}), params)
    try:
        return nav_cls(paths, params, strategy=strategy)
    except TypeError:
        return nav_cls(paths, params)


class AutoDriver:
    """Call the navigator the way multipath_boat.py's main loop does in AUTO, including the GPS-course
    heading selector (MotionHeadingSelector) used by strategies with uses_gps_course."""

    def __init__(self, nav):
        self.nav = nav
        strategy = getattr(nav, "strategy", None)
        self.selector = (MotionHeadingSelector(strategy.options)
                         if getattr(strategy, "uses_gps_course", False) else None)
        self.heading_info = None

    def enter(self, gps, t, moving=False):
        """AUTO entry. moving=True emulates recent forward driving in MANUAL (the course is reused)."""
        snap = gps.snapshot(t)
        if self.selector is not None:
            if moving:
                self.selector.observe_manual(snap, 1.0, t - 1.0)
                self.selector.observe_manual(snap, 1.0, t)
            snap = dict(snap, heading_deg=None, position_heading_deg=None, position_heading_age_sec=None,
                        position_heading_time_mono=None, position_speed_mps=None)
            self.selector.enter_auto(snap, t)
        return self.nav.enter_auto_mode(snap["lat"], snap["lon"], now_mono=t)

    def update(self, gps, t):
        snap = gps.snapshot(t)
        if self.selector is not None:
            info = self.selector.select(snap, t)
        else:
            info = {"heading_deg": snap["heading_deg"], "source": "legacy_blend", "age_sec": None,
                    "confident": snap["heading_deg"] is not None, "state": "legacy"}
        self.heading_info = info
        try:
            return self.nav.update(snap["lat"], snap["lon"], info["heading_deg"], now_mono=t,
                                   heading_confident=info["confident"], motion=info)
        except TypeError:       # Custom navigators with the original update() signature
            return self.nav.update(snap["lat"], snap["lon"], info["heading_deg"], now_mono=t)


def load_controller_class(spec):
    if spec is None:
        return mb.MultipathNavigator
    module_name, class_name = spec.split(":")
    return getattr(importlib.import_module(module_name), class_name)


def simulate_auto_once(args, model, env, cal, nav_cls, seed, log_path=None, init_state=None):
    drift, scale, bias = env
    paths = {lane: mb.load_waypoints_csv(p) for lane, p in mb.PATH_FILES.items()}
    nav = make_navigator(nav_cls, paths, cal)
    pwm = cal["pwm"]
    ratio = args.throttle_ratio if args.throttle_ratio is not None else pwm["auto_throttle_ratio"]
    cruise_us = int(round(pwm["esc_neutral_us"] + ratio * (pwm["esc_fwd_max_us"] - pwm["esc_neutral_us"])))
    rng = np.random.default_rng(seed)

    # Initial pose: start of the chosen lane, pointing along the first segment, at rest.
    lane_pts = nav.enu_paths[args.start_lane]
    (ax_, ay_), (bx_, by_) = lane_pts[0], lane_pts[1]
    seg_psi = math.atan2(bx_ - ax_, by_ - ay_)
    off = args.start_offset  # + = to the right of the path
    x = ax_ + off * math.cos(seg_psi)
    y = ay_ - off * math.sin(seg_psi)
    psi = seg_psi + math.radians(args.start_heading)
    state = (x, y, psi, 0.0, 0.0) if init_state is None else tuple(init_state)
    x, y = state[0], state[1]

    gps = GPSEmulator(noise_m=args.gps_noise, bias_walk_m=args.gps_bias_walk, seed=seed,
                      heading_source=getattr(args, "heading_source", None))
    gps.update(0.0, x, y, 0.0, 0.0)
    gps.reset_heading_anchor()
    driver = AutoDriver(nav)
    driver.enter(gps, 0.0, moving=state[3] >= 0.45)

    gust = 0.0
    gust_sigma = math.radians(args.gust)
    servo_hist = [pwm["servo_center_us"]] * (model.servo_delay + 1)
    esc_hist = [pwm["esc_neutral_us"]] * (model.esc_delay + 1)
    # The controller sees each fix ~GPS_LATENCY_SEC after it was measured (gps_utc vs log time).
    v0 = (state[3] * math.sin(state[2]) + drift[0], state[3] * math.cos(state[2]) + drift[1])
    gps_hist = [(x, y) + v0] * (int(round(GPS_LATENCY_SEC / DT)) + 1)

    rows = []
    nav_info = None
    t = 0.0
    finished = False
    finish_t = None          # When AUTO declared the course complete (motor to neutral, MANUAL)
    line_t = None            # When the true boat crossed the finish line (race time), coasting included
    c_pts = nav.enu_paths["center"]
    fin_ux, fin_uy = c_pts[-1][0] - c_pts[-2][0], c_pts[-1][1] - c_pts[-2][1]
    fin_norm = math.hypot(fin_ux, fin_uy)
    fin_ux, fin_uy = fin_ux / fin_norm, fin_uy / fin_norm
    n_steps = int(args.max_time / DT)
    for _ in range(n_steps):
        # Controller sees the latest GPS fix (5 Hz), like the real main loop at 20 Hz.
        lat, lon = gps.latlon()
        if finish_t is None:
            nav_info = driver.update(gps, t)
            if nav_info["finished"]:
                finished, finish_t = True, t
        if finished:
            servo_us, esc_us = pwm["servo_center_us"], pwm["esc_neutral_us"]
        else:
            servo_us = mb.map_norm_to_pwm(nav_info["steer_norm"], pwm["servo_center_us"], pwm["servo_left_us"], pwm["servo_right_us"])
            blend = max(0.0, min(1.0, nav_info.get("throttle_blend", 0.0)))
            esc_us = int(round(cruise_us + blend * (pwm["esc_fwd_max_us"] - cruise_us)))

        if finish_t is not None and t > finish_t:
            if line_t is not None or t > finish_t + 15.0:
                break
        else:
            rows.append([f"{t:.3f}", "AUTO", nav_info["active_lane"], nav_info["wp_number"], 4,
                         f"{lat:.8f}", f"{lon:.8f}", f"{nav_info['cte_right_m']:.3f}",
                         (f"{driver.heading_info['heading_deg']:.1f}"
                          if driver.heading_info["heading_deg"] is not None else ""),
                         f"{nav_info['target_bearing_deg']:.1f}", servo_us, esc_us,
                         f"{state[0]:.3f}", f"{state[1]:.3f}", int(nav_info["shifted_to"] is not None),
                         int(nav_info["boundary_guard"])])

        # Actuator delay, plant step, gust disturbance.
        servo_hist.append(servo_us)
        esc_hist.append(esc_us)
        state = model.step(state, servo_hist.pop(0), esc_hist.pop(0), drift=tuple(drift), speed_scale=scale,
                           yaw_bias=bias)
        gust += -gust / GUST_TAU_SEC * DT + gust_sigma * math.sqrt(2.0 * DT / GUST_TAU_SEC) * rng.standard_normal()
        state = (state[0], state[1], state[2] + gust * DT, state[3], state[4])
        t += DT
        if line_t is None and (state[0] - c_pts[-1][0]) * fin_ux + (state[1] - c_pts[-1][1]) * fin_uy >= 0.0:
            line_t = t

        vx = state[3] * math.sin(state[2]) + drift[0]
        vy = state[3] * math.cos(state[2]) + drift[1]
        gps_hist.append((state[0], state[1], vx, vy))
        gps.update(t, *gps_hist.pop(0))

    if log_path:
        with open(log_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "mode", "lane", "wp_no", "fix_quality", "latitude", "longitude",
                        "cte_right_m", "boat_heading_deg", "target_bearing_deg", "servo_us", "esc_us",
                        "true_x_m", "true_y_m", "lane_shift", "boundary_guard"])
            w.writerows(rows)

    arr = np.array([[float(r[12]), float(r[13]), float(r[7]), float(r[10])] for r in rows])
    lanes = [r[2] for r in rows]
    return {
        "finished": finished and line_t is not None,
        "time_s": line_t if line_t is not None else t,     # Finish-line crossing (race time)
        "auto_end_s": finish_t,
        "true_xy": arr[:, :2],
        "cte": arr[:, 2],
        "servo": arr[:, 3],
        "lanes": lanes,
        "lane_shifts": sum(r[14] for r in rows),
        "guard_frac": float(np.mean([r[15] for r in rows])),
        "outside_lanes_m": outside_corridor(nav, arr[:, :2]),
        "max_abs_lateral_m": max_abs_lateral(nav, arr[:, :2]),
        "esc": np.array([float(r[11]) for r in rows]),
    }


def max_abs_lateral(nav, xy):
    """Max distance (m) of the boat from the centerline (the +-2.5 m band of 05_straight_line)."""
    return max(abs(nav._closest_segment_metrics("center", x, y)[2]) for x, y in xy[::4])


def outside_corridor(nav, xy):
    """Max distance (m) beyond the outer lanes (left/right path lines): a proxy for hitting the wall."""
    worst = 0.0
    for x, y in xy[::4]:
        _, _, cte_l, _ = nav._closest_segment_metrics("left", x, y)
        _, _, cte_r, _ = nav._closest_segment_metrics("right", x, y)
        worst = max(worst, -cte_l, cte_r)
    return worst


def run_auto(args):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    model, meta = load_model_and_env()
    env = session_env(meta, args.session, args)
    drift, scale, bias = env
    cal = apply_strategy_args(mb.load_calibration(args.config), args)
    nav_cls = load_controller_class(args.controller)
    os.makedirs(OUT_DIR, exist_ok=True)

    print(f"[SIM] controller={nav_cls.__module__}.{nav_cls.__name__} runs={args.runs} drift={drift} "
          f"speed_scale={scale:.2f} yaw_bias={bias:+.3f} gust={args.gust}deg/s gps_noise={args.gps_noise}m start={args.start_lane}"
          f"{args.start_offset:+.1f}m/{args.start_heading:+.0f}deg")
    results = []
    for k in range(args.runs):
        log_path = os.path.join(OUT_DIR, f"sim_auto_run{k:02d}.csv") if k < 5 else None
        res = simulate_auto_once(args, model, env, cal, nav_cls, seed=args.seed + k, log_path=log_path)
        results.append(res)
        print(f"  run {k:02d}: {'finished' if res['finished'] else 'TIMEOUT '} in {res['time_s']:6.1f}s | "
              f"mean|CTE| {np.mean(np.abs(res['cte'])):.2f} m | max|CTE| {np.max(np.abs(res['cte'])):.2f} m | "
              f"lane shifts {res['lane_shifts']} | wall guard {100 * res['guard_frac']:.0f}% | "
              f"beyond outer lanes {res['outside_lanes_m']:.2f} m | max|lateral| {res['max_abs_lateral_m']:.2f} m")

    fin = [r for r in results if r["finished"]]
    print(f"\n[SUMMARY] finished {len(fin)}/{len(results)} | "
          f"time {np.mean([r['time_s'] for r in fin]) if fin else float('nan'):.1f}s | "
          f"mean|CTE| {np.mean([np.mean(np.abs(r['cte'])) for r in results]):.2f} m | "
          f"lane shifts/run {np.mean([r['lane_shifts'] for r in results]):.1f} | "
          f"worst excursion beyond outer lanes {max(r['outside_lanes_m'] for r in results):.2f} m | "
          f"worst |lateral| from centerline {max(r['max_abs_lateral_m'] for r in results):.2f} m")

    paths = {lane: mb.load_waypoints_csv(p) for lane, p in mb.PATH_FILES.items()}
    nav = make_navigator(nav_cls, paths, cal)
    fig = plt.figure(figsize=(16, 8))
    ax = fig.add_subplot(1, 2, 1)
    for lane, c in zip(("left", "center", "right"), ("tab:blue", "tab:orange", "tab:green")):
        p = np.array(nav.enu_paths[lane])
        ax.plot(p[:, 0], p[:, 1], "-", c=c, lw=1, label=f"{lane} path")
    for r in results:
        ax.plot(r["true_xy"][:, 0], r["true_xy"][:, 1], "k-", lw=0.8, alpha=0.6)
    ax.set_aspect("equal", "datalim")
    ax.legend()
    ax.grid(alpha=0.3)
    ax.set_title(f"Closed-loop AUTO, {len(results)} runs (black = true boat track)")
    r0 = results[0]
    tt = np.arange(len(r0["cte"])) * DT
    a2 = fig.add_subplot(3, 2, 2)
    a2.plot(tt, r0["cte"])
    a2.set_ylabel("CTE to active lane (m)")
    a3 = fig.add_subplot(3, 2, 4, sharex=a2)
    lane_names = ("left", "center", "right", "line")
    a3.plot(tt, [lane_names.index(l) if l in lane_names else -1 for l in r0["lanes"]])
    a3.set_yticks([0, 1, 2, 3], lane_names)
    a3.set_ylabel("active lane")
    a4 = fig.add_subplot(3, 2, 6, sharex=a2)
    a4.plot(tt, r0["servo"])
    a4.set_ylabel("servo us")
    a4.set_xlabel("time (s), run 00")
    for a in (a2, a3, a4):
        a.grid(alpha=0.3)
    fig.tight_layout()
    out = os.path.join(OUT_DIR, "sim_auto.png")
    fig.savefig(out, dpi=90)
    print(f"[SAVED] {out} and per-run CSV logs in {OUT_DIR}/")


def run_validate_auto(args):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    model, meta = load_model_and_env()
    run = LogRun(args.log)
    session = os.path.basename(args.log)[len("multipath_log_"):-len(".csv")]
    env = session_env(meta, session, args)
    drift, scale, bias = env
    cal = apply_strategy_args(mb.load_calibration(args.config), args)
    nav_cls = load_controller_class(args.controller)
    i0, i1 = run.index_at(args.t0), run.index_at(args.t1)
    x0, y0, psi0, u0, r0 = run.state_at(i0)
    if u0 < 0.3:  # Heading is unobservable when slow: take it from the next 0.5 m of real movement
        j = i0
        while j < i1 and math.hypot(run.x_s[j] - x0, run.y_s[j] - y0) < 0.5:
            j += 1
        psi0 = math.atan2(run.x_s[j] - x0, run.y_s[j] - y0)
    args.start_lane, args.start_offset, args.start_heading = "center", 0.0, 0.0
    args.max_time = (run.t[i1] - run.t[i0]) + 30.0

    with open(args.log, "r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))[i0:i1 + 1]
    real_cte = np.array([float(r["cte_right_m"]) if r["cte_right_m"] else np.nan for r in rows])
    real_lanes = [r["lane"] for r in rows]
    real_tt = run.t[i0:i1 + 1] - run.t[i0]
    real_xy = np.stack([run.x_gps[i0:i1 + 1], run.y_gps[i0:i1 + 1]], axis=1)

    print(f"[VALIDATE] {run.name} t={run.t[i0]:.1f}-{run.t[i1]:.1f}s session={session} "
          f"drift=({drift[0]:+.3f}, {drift[1]:+.3f}) speed_scale={scale:.2f} yaw_bias={bias:+.3f} "
          f"start speed {u0:.2f} m/s heading {math.degrees(psi0) % 360:.0f} deg")
    res = [simulate_auto_once(args, model, env, cal, nav_cls, seed=args.seed + k,
                              init_state=(x0, y0, psi0, u0, 0.0)) for k in range(args.runs)]

    def lanes_of(seq):
        out = []
        for lane in seq:
            if lane not in ("-", "") and (not out or out[-1] != lane):
                out.append(lane)
        return out

    settle = real_tt >= 15.0
    print(f"  {'':8s} {'duration':>9s} {'mean|CTE|':>10s} {'after 15s':>10s} {'max|CTE|':>9s} {'full thr':>9s}  lanes")
    print(f"  {'REAL':8s} {real_tt[-1]:8.1f}s {np.nanmean(np.abs(real_cte)):9.2f}m "
          f"{np.nanmean(np.abs(real_cte[settle])):9.2f}m {np.nanmax(np.abs(real_cte)):8.2f}m "
          f"{100 * np.mean(run.esc[i0:i1 + 1] >= 1985):8.0f}%  {lanes_of(real_lanes)}")
    for k, r in enumerate(res):
        tt = np.arange(len(r["cte"])) * DT
        print(f"  {'sim ' + str(k):8s} {r['time_s']:8.1f}s {np.mean(np.abs(r['cte'])):9.2f}m "
              f"{np.mean(np.abs(r['cte'][tt >= 15.0])):9.2f}m {np.max(np.abs(r['cte'])):8.2f}m "
              f"{100 * np.mean(r['esc'] >= 1985):8.0f}%  {lanes_of(r['lanes'])}"
              f"{'' if r['finished'] else ' (not finished)'}")

    os.makedirs(OUT_DIR, exist_ok=True)
    paths = {lane: mb.load_waypoints_csv(p) for lane, p in mb.PATH_FILES.items()}
    nav = make_navigator(nav_cls, paths, cal)
    fig = plt.figure(figsize=(16, 8))
    ax = fig.add_subplot(1, 2, 1)
    for lane, c in zip(("left", "center", "right"), ("tab:blue", "tab:orange", "tab:green")):
        p = np.array(nav.enu_paths[lane])
        ax.plot(p[:, 0], p[:, 1], "-", c=c, lw=1, label=f"{lane} path")
    for k, r in enumerate(res):
        ax.plot(r["true_xy"][:, 0], r["true_xy"][:, 1], "r-", lw=0.7, alpha=0.5, label="simulated" if k == 0 else None)
    ax.plot(real_xy[:, 0], real_xy[:, 1], "k-", lw=2, label="real")
    ax.set_aspect("equal", "datalim")
    ax.legend()
    ax.grid(alpha=0.3)
    ax.set_title(f"Closed-loop: {run.name} AUTO t={run.t[i0]:.0f}-{run.t[i1]:.0f}s")
    a2 = fig.add_subplot(2, 2, 2)
    for r in res:
        a2.plot(np.arange(len(r["cte"])) * DT, r["cte"], "r-", lw=0.7, alpha=0.5)
    a2.plot(real_tt, real_cte, "k-", lw=2)
    a2.set_ylabel("CTE to active lane (m)")
    a2.grid(alpha=0.3)
    a3 = fig.add_subplot(2, 2, 4, sharex=a2)
    for r in res:
        a3.plot(np.arange(len(r["esc"])) * DT, r["esc"], "r-", lw=0.7, alpha=0.5)
    a3.plot(real_tt, run.esc[i0:i1 + 1], "k-", lw=2)
    a3.set_ylabel("ESC us")
    a3.set_xlabel("time since AUTO entry (s)")
    a3.grid(alpha=0.3)
    fig.tight_layout()
    out = os.path.join(OUT_DIR, f"validate_auto_{session}.png")
    fig.savefig(out, dpi=80)
    print(f"[SAVED] {out}")


def main():
    parser = argparse.ArgumentParser(description="Haesucheon boat simulator (fitted from telemetry)")
    sub = parser.add_subparsers(dest="cmd", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--session", default="20261116_172709",
                        help="Use the current/battery estimate fitted for this log (time stamp of its file name)")
    common.add_argument("--drift", type=float, nargs=2, default=None, metavar=("EAST", "NORTH"), help="Water current override (m/s)")
    common.add_argument("--speed-scale", type=float, default=None, help="Throttle map multiplier override (battery)")
    common.add_argument("--yaw-bias", type=float, default=None,
                        help="Rudder-centred turn curvature at full throttle (1/m, + = right) override; "
                             "varies between logs, so test controllers at several values")

    p_rep = sub.add_parser("replay", parents=[common], help="Replay logged commands open-loop and compare to GPS")
    p_rep.add_argument("log")
    p_rep.add_argument("--t0", type=float, default=0.0)
    p_rep.add_argument("--t1", type=float, default=None)

    p_auto = sub.add_parser("auto", parents=[common], help="Closed-loop AUTO controller simulation")
    p_auto.add_argument("--controller", default=None, help="module:Class with MultipathNavigator's interface")
    p_auto.add_argument("--config", default=mb.DEFAULT_CALIBRATION_FILE)
    p_auto.add_argument("--throttle-ratio", type=float, default=None)
    p_auto.add_argument("--runs", type=int, default=5)
    p_auto.add_argument("--seed", type=int, default=0)
    p_auto.add_argument("--gust", type=float, default=3.0,
                        help="Yaw gust std (deg/s); 3 matched the real 17:27 AUTO run best, 6 is a pessimistic case")
    p_auto.add_argument("--gps-noise", type=float, default=0.02, help="GPS position noise std (m); ~0.02 RTK fix")
    p_auto.add_argument("--gps-bias-walk", type=float, default=0.0, help="GPS bias random walk (m/sqrt(s)), e.g. 0.05 for DGPS")
    p_auto.add_argument("--start-lane", default="center", choices=["left", "center", "right"])
    p_auto.add_argument("--start-offset", type=float, default=0.0, help="Lateral start offset, + = right of the lane (m)")
    p_auto.add_argument("--start-heading", type=float, default=0.0, help="Start heading offset from the path (deg)")
    p_auto.add_argument("--max-time", type=float, default=300.0)
    add_strategy_args(p_auto)

    p_val = sub.add_parser("validate-auto", parents=[common], help="Closed-loop replay of a real AUTO run")
    p_val.add_argument("log")
    p_val.add_argument("--t0", type=float, required=True, help="AUTO entry time in the log (s)")
    p_val.add_argument("--t1", type=float, required=True, help="AUTO end time in the log (s)")
    p_val.add_argument("--controller", default=None)
    p_val.add_argument("--config", default=mb.DEFAULT_CALIBRATION_FILE)
    p_val.add_argument("--throttle-ratio", type=float, default=None)
    p_val.add_argument("--runs", type=int, default=10)
    p_val.add_argument("--seed", type=int, default=0)
    p_val.add_argument("--gust", type=float, default=3.0)
    p_val.add_argument("--gps-noise", type=float, default=0.02)
    p_val.add_argument("--gps-bias-walk", type=float, default=0.0)
    add_strategy_args(p_val)

    args = parser.parse_args()
    if args.cmd == "replay":
        run_replay(args)
    elif args.cmd == "validate-auto":
        run_validate_auto(args)
    else:
        run_auto(args)


if __name__ == "__main__":
    main()
