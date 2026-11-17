#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
optimise_lanes.py
Choose the position and spacing of the three lanes (left / center / right) between the measured banks,
for robustness (staying clear of the banks) and speed (time from AUTO entry to the finish).

It runs the REAL controller code in a closed loop: MultipathNavigator and ZEDF9PReceiver from
multipath_boat.py, MotionHeadingSelector from gps_heading.py and the AUTO strategy from auto_strategies.py,
with the strategy, throttle ratio and finish margin from calibration.json. Only the boat and the GPS are
modelled.

Boat model (fitted from the 2026-11-16 and 2026-11-17 logs, RTK fix):
  yaw      r' = (K * rudder + bias + gust - r) / T      K = 36 deg/s at full rudder and 1.3 m/s, scaling with
           the flow over the rudder (half hull speed, half propeller wash); T = 0.28 s. Field fit at
           1.64 m/s: K = 46 deg/s.
  bias     steady turning push that changes from run to run: fitted values -7 to +12.5 deg/s
           (mean +2.7, s.d. 6.5; + = turns right).
  speed    from the ESC pulse: 1914 us -> 1.18 m/s, 1990 us -> 1.55 m/s in still water (17 Nov), reduced
           by rudder drag; reaches speed with a 0.3 s time constant (AUTO entry: 0.2 -> 1.6 m/s in ~1 s).
  skid     the direction of travel follows the bow with a 0.3 s lag, so the hull keeps sliding outward
           while it turns back at the start.
  drift    velocity = hull speed along its travel direction + a steady sideways drift + 0.09 m/s push
           along the course.
  check    --validate re-runs seven logged AUTO entries. Start excursions: 1.95 m vs 2.14 m logged (run C),
           2.10 vs 2.65 (16 Nov 10:43), 1.67 vs 1.61 to the left (16 Nov 17:27); steady offset from the
           lane within 0.2 m; time to the finish within 1.5 s. The largest excursions read 10-20% low,
           which SAFE_CLEARANCE_M allows for.
  GPS      5 Hz RTK fixes, 1.5 cm noise, 80 ms latency; RMC course noise 1.5 deg, speed noise 0.03 m/s.

AUTO entry, as logged: the entry gesture (neutral throttle, right rudder for 2 s) leaves the boat at about
0.2 m/s pointing anywhere from -52 to +56 deg off the course (mean +10, s.d. 37). AUTO then applies
throttle before it has a heading, so the boat first runs off in that direction. That start excursion
(up to 2.65 m in the logs) is the main bank risk; later the boat stays within about 0.7 m of its lane.

Method:
  1. The controller never sees the banks, so its track relative to the lanes does not depend on where the
     lanes sit in the channel. Each lane spacing is therefore simulated once (every start lane, the same
     random scenarios), recording how far left and right of the center lane the boat went.
  2. Sliding the three lanes sideways is then arithmetic: the clearance to each bank for every run.
  3. Robustness = share of runs that come within SAFE_CLEARANCE_M (0.75 m) of a bank (antenna position;
     the bank lines are antenna positions at bow contact). Speed = mean time from AUTO entry to the finish.
  4. Pick the layout with the fewest unsafe runs; among layouts within 0.5 percentage points of the best,
     the fastest; then the one nearest the present 1.5 m spacing.

Result for the 17 Nov banks (-4.20 / +5.11 m), strategy 04_predictive_recovery at throttle ratio 1.0,
300 scenarios per start lane:
  - Lane layout does not change the time (56.8 to 57.0 s for every layout): AUTO is at full throttle on
    a straight course.
  - Lanes as in left/center/right.csv (1.5 m apart, center on 0.00): 3.0% of runs within 0.75 m of a bank
    (start on the left lane 3.0%, center 0.3%, right 5.7%).
  - The same lanes centred between the banks (+0.46 m) are worse: 3.8% (right lane 10.3%), because the
    boat more often launches to the right.
  - Chosen: 1.0 m apart at -1.20 / -0.20 / +0.80 m: 0.9% (left 1.7%, center 0.3%, right 0.7%).
  - Starting on the center lane is the largest gain with any layout (0.3%).

Outputs:
  course_lanes_optimised.json    chosen layout, comparison with the present lanes, per-lane results
  lanes_optimised/{left,center,right}.csv   waypoints in the format multipath_boat.py reads
The boat's own left.csv / center.csv / right.csv are NOT changed.

Usage (from the repo root, after tools/map_banks.py has written course_banks.json):
  python tools/optimise_lanes.py --validate        # model vs the field runs
  python tools/optimise_lanes.py                   # optimise (about 10 min on 14 cores)
  python tools/optimise_lanes.py --runs 60         # quicker, rougher
"""

import argparse
import concurrent.futures
import csv
import datetime
import json
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

import multipath_boat as mb                      # noqa: E402
from auto_strategies import create_auto_strategy  # noqa: E402
from gps_heading import MotionHeadingSelector     # noqa: E402
import map_banks as course                        # noqa: E402  (course frame: to_course / from_course)

# ---- boat model
SPEED_CURVE = [(1800, 0.0), (1830, 0.25), (1850, 0.85), (1890, 1.08), (1914, 1.175), (1990, 1.55)]
K_REF, K_REF_SPEED, T_YAW = 36.0, 1.3, 0.28
TAU_SERVO, TAU_SURGE, DRAG = 0.08, 0.3, 0.25
TAU_SLIP = 0.3                        # s: the direction of travel follows the bow with this lag (hull skid)
ALONG_PUSH = 0.09
GPS_LATENCY, GPS_POS_NOISE, GPS_COG_NOISE, GPS_SOG_NOISE = 0.08, 0.015, 1.5, 0.03

# ---- scenario distributions (from the logs; see module docstring)
LAUNCH_DEG = (10.0, 37.0, 75.0)       # mean, s.d., clip: direction the boat points at AUTO entry
BIAS_DPS = (2.7, 6.5, -8.0, 13.0)     # mean, s.d., min, max
DRIFT_MPS = (0.0, 0.05, 0.12)         # sideways drift: mean, s.d., clip
START_ERR_M = (0.25, 0.6)             # how far off the chosen lane AUTO is engaged: s.d., clip
START_ALONG_M = (1.0, 8.0)

SAFE_CLEARANCE_M = 0.75               # 0.25 m half-beam + 0.5 m for the model reading large excursions 10-20% low
# Lane spacings tried. 1.0 m is the lower limit: the navigator changes lane at shift_cte_m (0.70 m) off the
# lane, and at 1.0 m spacing that puts the boat 0.30 m from the next lane, the edge of the full-throttle
# band (full_throttle_cte_m). Closer lanes would make a lane change overshoot the lane it moves to.
SPACINGS = (1.0, 1.25, 1.5, 1.75, 2.0)
TIE_PCT = 0.5


class SimClock:
    """Stands in for the `time` module inside multipath_boat so its timestamps follow simulated time."""
    now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, _):
        pass


CLOCK = SimClock()
mb.time = CLOCK


def speed_from_esc(us):
    if us <= SPEED_CURVE[0][0]:
        return 0.0
    for (a, va), (b, vb) in zip(SPEED_CURVE, SPEED_CURVE[1:]):
        if us <= b:
            return va + (vb - va) * (us - a) / (b - a)
    return SPEED_CURVE[-1][1]


def make_paths(spacing, center_off=0.0):
    """Three straight lanes parallel to the course, 65 waypoints each, in multipath_boat's format."""
    n = len(course.LANES["center"])
    step = course.COURSE_LEN / (n - 1)
    return {name: [(i + 1,) + tuple(course.from_course(i * step, center_off + k * spacing)) for i in range(n)]
            for name, k in (("left", -1), ("center", 0), ("right", 1))}


def load_config():
    cal = mb.load_calibration(os.path.join(ROOT, "calibration.json"))
    return dict(auto=cal.get("autonomous", {}), ratio=cal["pwm"]["auto_throttle_ratio"],
                nav=mb.resolve_navigation_params(cal.get("navigation", {})),
                esc_neutral=cal["pwm"]["esc_neutral_us"], esc_full=cal["pwm"]["esc_fwd_max_us"])


def feed_epoch(rx, lat, lon, sog, cog):
    """One GPS epoch, applied the way ZEDF9PReceiver._read_loop applies an RMC then a GGA sentence."""
    stamp = datetime.datetime(2026, 11, 17, 0, 30, 0) + datetime.timedelta(seconds=CLOCK.now)
    utc = stamp.strftime("%H%M%S.") + f"{stamp.microsecond // 10000:02d}"
    rmc = ["$GNRMC", utc, "A", "", "", "", "", f"{sog / 0.514444:.3f}",
           "" if cog is None else f"{cog:.2f}", stamp.strftime("%d%m%y"), "", "", "R"]
    rx._update_course_observation(rmc)
    rx.sog_mps, rx.cog_deg = sog, cog
    rx.fix_quality, rx.num_sats, rx.hdop, rx.gps_utc = 4, 12, 0.7, utc
    rx._update_position_observation(lat, lon, utc, 4)
    rx.lat, rx.lon = lat, lon
    rx.last_fix_time = CLOCK.now
    rx._update_movement_heading(lat, lon, rx.sog_mps, rx.cog_deg)


def simulate(paths, cfg, scen, seed, trace=False):
    """One AUTO run from entry to the finish. Offsets are in course coordinates (m right of the course
    origin line). scen: along, off, launch_deg, u0, r0, bias, gust, drift."""
    rnd = random.Random(seed)
    CLOCK.now = 0.0
    params = dict(cfg["nav"])
    strategy = create_auto_strategy(cfg["auto"], params)
    nav = mb.MultipathNavigator(paths, params, strategy=strategy)
    selector = MotionHeadingSelector(strategy.options) if strategy.uses_gps_course else None
    rx = mb.ZEDF9PReceiver("sim")                       # the serial port is never opened
    cruise_us = cfg["esc_neutral"] + cfg["ratio"] * (cfg["esc_full"] - cfg["esc_neutral"])

    brg = math.radians(course.COURSE_BRG)
    tx, ty = math.sin(brg), math.cos(brg)               # along the course (east, north)
    nx, ny = ty, -tx                                    # to the right of the course
    x = scen["along"] * tx + scen["off"] * nx
    y = scen["along"] * ty + scen["off"] * ny
    psi = course.COURSE_BRG + scen["launch_deg"]
    u, r = scen["u0"], scen["r0"]
    chi = psi                                           # direction the hull is travelling
    dvx = scen["drift"] * nx + ALONG_PUSH * tx
    dvy = scen["drift"] * ny + ALONG_PUSH * ty
    delta, gust, steer_cmd, esc_us = 0.0, 0.0, 0.0, cfg["esc_neutral"]

    def fix_at(px, py, vx, vy):
        lat, lon = course.latlon(px + rnd.gauss(0, GPS_POS_NOISE), py + rnd.gauss(0, GPS_POS_NOISE))
        sog = max(0.0, math.hypot(vx, vy) + rnd.gauss(0, GPS_SOG_NOISE))
        cog = (math.degrees(math.atan2(vx, vy)) + rnd.gauss(0, GPS_COG_NOISE)) % 360.0 if sog >= 0.1 else None
        feed_epoch(rx, lat, lon, sog, cog)

    # Before entry: the boat coasts at u0 with the throttle at neutral (MANUAL), so the receiver and the
    # heading selector hold the same kind of history they have on the water.
    vx0 = u * math.sin(math.radians(psi)) + dvx
    vy0 = u * math.cos(math.radians(psi)) + dvy
    for k in range(-6, 1):
        CLOCK.now = 1.2 + k * 0.2
        back = (k * 0.2) - GPS_LATENCY
        fix_at(x + vx0 * back, y + vy0 * back, vx0, vy0)
        if selector is not None:
            selector.observe_manual(rx.get_snapshot(), 0.0, CLOCK.now)
    t0 = CLOCK.now

    # AUTO entry, as in multipath_boat.main()
    snap = rx.get_snapshot()
    rx.reset_heading_anchor()
    if selector is not None:
        snap = dict(snap, heading_deg=None, position_heading_deg=None, position_heading_age_sec=None,
                    position_heading_time_mono=None, position_speed_mps=None)
        selector.enter_auto(snap, t0)
    start_lane, _, _ = nav.enter_auto_mode(snap["lat"], snap["lon"], now_mono=t0)
    esc_us = cruise_us

    dt, t = 0.01, t0
    hist = [(t, x, y, vx0, vy0)]
    next_fix, next_ctrl = t0 + 0.2, t0 + 0.05
    lo = hi = (x * nx + y * ny)
    lo8, hi8 = lo, hi
    shifts, finished, lane = 0, False, start_lane
    cte_sum = cte_n = 0
    rows = []
    while t - t0 < 150.0:
        # ---- boat
        delta += (steer_cmd - delta) * dt / TAU_SERVO
        flow = (0.5 * u + 0.5 * speed_from_esc(esc_us)) / K_REF_SPEED
        gust += (-gust / 1.5) * dt + scen["gust"] * math.sqrt(2 * dt / 1.5) * rnd.gauss(0, 1)
        r += (K_REF * flow * delta + scen["bias"] + gust - r) * dt / T_YAW
        psi += r * dt
        u += (speed_from_esc(esc_us) * (1.0 - DRAG * delta * delta) - u) * dt / TAU_SURGE
        chi = psi if TAU_SLIP <= 0 else chi + (psi - chi) * dt / TAU_SLIP
        vx = u * math.sin(math.radians(chi)) + dvx
        vy = u * math.cos(math.radians(chi)) + dvy
        x += vx * dt
        y += vy * dt
        t += dt
        CLOCK.now = t
        hist.append((t, x, y, vx, vy))
        if len(hist) > 40:
            hist.pop(0)
        off = x * nx + y * ny
        lo, hi = min(lo, off), max(hi, off)
        if t - t0 <= 8.0:
            lo8, hi8 = min(lo8, off), max(hi8, off)

        # ---- GPS epoch arrives
        if t >= next_fix:
            next_fix += 0.2
            past = min(hist, key=lambda h: abs(h[0] - (t - GPS_LATENCY)))
            fix_at(past[1], past[2], past[3], past[4])

        # ---- 20 Hz control loop, as in multipath_boat.main()
        if t >= next_ctrl:
            next_ctrl += 0.05
            snap = rx.get_snapshot()
            heading = {"heading_deg": snap["heading_deg"], "confident": snap["heading_deg"] is not None}
            if selector is not None:
                heading = selector.select(snap, t)
            info = nav.update(snap["lat"], snap["lon"], heading["heading_deg"], now_mono=t,
                              heading_confident=heading["confident"], motion=heading)
            if info["finished"]:
                finished = True
                break
            if info["shifted_to"]:
                shifts += 1
            lane = info["active_lane"]
            steer_cmd = info["steer_norm"]
            blend = max(0.0, min(1.0, info.get("throttle_blend", 0.0)))
            esc_us = int(round(cruise_us + blend * (cfg["esc_full"] - cruise_us)))
            if t - t0 > 10.0:
                cte_sum += info["cte_right_m"]
                cte_n += 1
            if trace:
                rows.append((round(t - t0, 2), round(x * tx + y * ty, 2), round(off, 2), lane,
                             round(info["cte_right_m"], 3), round(steer_cmd, 3), esc_us, round(u, 2)))
    out = dict(finished=finished, time=round(t - t0, 2), lo=round(lo, 3), hi=round(hi, 3),
               lo8=round(lo8, 3), hi8=round(hi8, 3), shifts=shifts, start_lane=start_lane, end_lane=lane,
               mean_cte=round(cte_sum / cte_n, 3) if cte_n else None)
    if trace:
        out["rows"] = rows
    return out


def draw_scenario(seed):
    """Random AUTO entry and disturbance. The same seed gives the same scenario for every layout."""
    rnd = random.Random(1000 + seed)
    clip = lambda v, lo, hi: max(lo, min(hi, v))
    return dict(
        along=rnd.uniform(*START_ALONG_M),
        err=clip(rnd.gauss(0, START_ERR_M[0]), -START_ERR_M[1], START_ERR_M[1]),
        launch_deg=clip(rnd.gauss(LAUNCH_DEG[0], LAUNCH_DEG[1]), -LAUNCH_DEG[2], LAUNCH_DEG[2]),
        u0=rnd.uniform(0.1, 0.3), r0=rnd.uniform(0.0, 20.0),
        bias=clip(rnd.gauss(BIAS_DPS[0], BIAS_DPS[1]), BIAS_DPS[2], BIAS_DPS[3]),
        gust=rnd.uniform(1.0, 5.0),
        drift=clip(rnd.gauss(DRIFT_MPS[0], DRIFT_MPS[1]), -DRIFT_MPS[2], DRIFT_MPS[2]),
    )


def _job(args):
    spacing, lane_k, seed, cfg = args
    scen = draw_scenario(seed)
    scen["off"] = lane_k * spacing + scen["err"]
    res = simulate(make_paths(spacing), cfg, scen, seed)
    return spacing, lane_k, seed, res


def validate(cfg):
    """Re-run logged AUTO entries with the strategy and throttle they used; compare the start excursion,
    the steady offset from the lane and the time to the finish."""
    cases = [
        # name, strategy, throttle ratio, along, off, launch deg, bias; field: (right, left, steady cte, time)
        ("17 Nov run C  right lane, +56 deg", "03_gps_course_progressive", 1.0, 5.1, 1.28, 56.0, -3.8, (2.14, 0.0, -0.26, 53.6)),
        ("17 Nov run D  left lane,  +23 deg", "04_predictive_recovery", 1.0, 5.2, -1.59, 23.0, 7.5, (0.55, 0.0, 0.2, 52.6)),
        ("17 Nov run B  left lane,  +12 deg", "02_adaptive_throttle", 1.0, 31.6, -1.39, 12.0, -1.9, (0.39, -0.19, -0.1, 36.3)),
        ("17 Nov 09:36  left lane,  -33 deg", "02_adaptive_throttle", 0.6, 8.5, -1.49, -33.0, 0.0, (0.0, -0.49, -0.17, None)),
        ("16 Nov 10:43  right lane, +55 deg", "01_constant_cruise", 90.0 / 190.0, 3.8, 1.44, 55.0, 5.0, (2.65, 0.0, 0.62, None)),
        ("16 Nov 17:21  center,     +38 deg", "01_constant_cruise", 0.6, 20.3, 0.27, 38.0, 5.0, (1.63, 0.0, None, None)),
        ("16 Nov 17:27  right lane, -52 deg", "02_adaptive_throttle", 0.6, 4.7, 1.44, -52.0, -3.5, (0.0, -1.61, -0.2, None)),
    ]
    paths = {name: mb.load_waypoints_csv(os.path.join(ROOT, fn)) for name, fn in mb.PATH_FILES.items()}
    print("Model check against logged AUTO runs (median of 9 seeds; excursion = from the start point, first 8 s)")
    print(f"{'case':36s} {'':5s} {'right':>6s} {'left':>6s} {'steady cte':>10s} {'time':>7s}")
    for name, strat, ratio, along, off, launch, bias, field in cases:
        c = dict(cfg, auto=dict(cfg["auto"], strategy=strat), ratio=ratio, nav=dict(cfg["nav"], finish_margin_m=0.0))
        res = [simulate(paths, c, dict(along=along, off=off, launch_deg=launch, u0=0.2, r0=10.0, bias=bias,
                                       gust=2.0, drift=0.0), s) for s in range(9)]
        med = lambda f: sorted(f(q) for q in res)[len(res) // 2]
        fc = "" if field[2] is None else f"{field[2]:+.2f}"
        ft = "" if field[3] is None else f"{field[3]:.1f}s"
        print(f"{name:36s} {'field':5s} {field[0]:+6.2f} {field[1]:+6.2f} {fc:>10s} {ft:>7s}")
        print(f"{'':36s} {'model':5s} {med(lambda q: q['hi8'] - off):+6.2f} {med(lambda q: q['lo8'] - off):+6.2f} "
              f"{med(lambda q: q['mean_cte'] or 0):+10.2f} {med(lambda q: q['time']):6.1f}s")


def pct(values, p):
    v = sorted(values)
    return v[min(len(v) - 1, max(0, int(round(p / 100.0 * (len(v) - 1)))))]


def evaluate(results, spacing, center, left_bank, right_bank):
    """Bank clearance statistics for one layout: lanes at center - spacing, center, center + spacing."""
    per_lane, pooled = {}, []
    for k, name in ((-1, "left"), (0, "center"), (1, "right")):
        runs = results[(spacing, k)]
        clear = [min(center + q["lo"] - left_bank, right_bank - center - q["hi"]) for q in runs]
        pooled += clear
        per_lane[name] = dict(
            unsafe_pct=round(100.0 * sum(1 for c in clear if c < SAFE_CLEARANCE_M) / len(clear), 1),
            hit_pct=round(100.0 * sum(1 for c in clear if c <= 0) / len(clear), 1),
            clearance_p1_m=round(pct(clear, 1), 2), clearance_p10_m=round(pct(clear, 10), 2),
            clearance_median_m=round(pct(clear, 50), 2),
            mean_time_s=round(sum(q["time"] for q in runs) / len(runs), 2),
            p90_time_s=round(pct([q["time"] for q in runs], 90), 2),
            lane_changes_per_run=round(sum(q["shifts"] for q in runs) / len(runs), 2),
            did_not_finish_pct=round(100.0 * sum(1 for q in runs if not q["finished"]) / len(runs), 1))
    return dict(
        spacing_m=spacing, center_off_m=round(center, 2),
        unsafe_pct=round(100.0 * sum(1 for c in pooled if c < SAFE_CLEARANCE_M) / len(pooled), 2),
        hit_pct=round(100.0 * sum(1 for c in pooled if c <= 0) / len(pooled), 2),
        clearance_p1_m=round(pct(pooled, 1), 2),
        mean_time_s=round(sum(v["mean_time_s"] for v in per_lane.values()) / 3.0, 2),
        lanes=per_lane)


def main():
    ap = argparse.ArgumentParser(description="Optimise lane position and spacing between the measured banks")
    ap.add_argument("--banks", default=os.path.join(ROOT, "course_banks.json"))
    ap.add_argument("--runs", type=int, default=300, help="Random scenarios per start lane and spacing")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--validate", action="store_true", help="Compare the model with logged AUTO runs and exit")
    ap.add_argument("--json-out", default=os.path.join(ROOT, "course_lanes_optimised.json"))
    ap.add_argument("--lanes-out", default=os.path.join(ROOT, "lanes_optimised"))
    args = ap.parse_args()

    cfg = load_config()
    if args.validate:
        validate(cfg)
        return
    with open(args.banks, encoding="utf-8") as f:
        banks = json.load(f)
    lb, rb = banks["left_bank"]["off_m"], banks["right_bank"]["off_m"]
    now_off = banks["lanes_off_m"]
    print(f"Banks {lb:+.2f} / {rb:+.2f} m, strategy {cfg['auto'].get('strategy')}, throttle ratio {cfg['ratio']}, "
          f"finish margin {cfg['nav']['finish_margin_m']} m; {args.runs} scenarios x 3 start lanes x {len(SPACINGS)} spacings")

    jobs = [(s, k, seed, cfg) for s in SPACINGS for k in (-1, 0, 1) for seed in range(args.runs)]
    results = {}
    with concurrent.futures.ProcessPoolExecutor(max_workers=args.workers) as pool:
        for n, (s, k, seed, res) in enumerate(pool.map(_job, jobs, chunksize=8), 1):
            results.setdefault((s, k), []).append(res)
            if n % 500 == 0:
                print(f"  {n}/{len(jobs)} runs")

    # The present lanes: nearest simulated spacing, at their present position.
    now_spacing = min(SPACINGS, key=lambda s: abs(s - (now_off["right"] - now_off["center"])))
    present = evaluate(results, now_spacing, now_off["center"], lb, rb)
    centred = evaluate(results, now_spacing, (lb + rb) / 2.0, lb, rb)

    table = []
    for s in SPACINGS:
        best = None
        c = lb + s
        while c <= rb - s + 1e-9:
            e = evaluate(results, s, c, lb, rb)
            key = (e["unsafe_pct"], -e["clearance_p1_m"])
            if best is None or key < best[0]:
                best = (key, e)
            c += 0.05
        table.append(best[1])
    floor = min(e["unsafe_pct"] for e in table)
    near = [e for e in table if e["unsafe_pct"] <= floor + TIE_PCT]
    fastest = min(e["mean_time_s"] for e in near)
    chosen = min((e for e in near if e["mean_time_s"] <= fastest + 0.1), key=lambda e: abs(e["spacing_m"] - 1.5))

    lanes = {n: round(chosen["center_off_m"] + k * chosen["spacing_m"], 3) for n, k in (("left", -1), ("center", 0), ("right", 1))}
    best_lane = min(chosen["lanes"], key=lambda n: (chosen["lanes"][n]["unsafe_pct"], chosen["lanes"][n]["mean_time_s"]))
    paths = make_paths(chosen["spacing_m"], chosen["center_off_m"])
    os.makedirs(args.lanes_out, exist_ok=True)
    for name, pts in paths.items():
        with open(os.path.join(args.lanes_out, f"{name}.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["waypoint_number", "latitude", "longitude"])
            for wp, lat, lon in pts:
                w.writerow([wp, f"{lat:.8f}", f"{lon:.8f}"])

    out = dict(
        chosen=dict(chosen, lanes_off_m=lanes, start_lane=best_lane,
                    room_to_bank_m=dict(left=round(lanes["left"] - lb, 2), right=round(rb - lanes["right"], 2))),
        present=dict(present, lanes_off_m=now_off), centred=centred, by_spacing=table,
        banks=dict(left=lb, right=rb), safe_clearance_m=SAFE_CLEARANCE_M, runs_per_lane=args.runs,
        settings=dict(strategy=cfg["auto"].get("strategy"), throttle_ratio=cfg["ratio"],
                      finish_margin_m=cfg["nav"]["finish_margin_m"], shift_cte_m=cfg["nav"]["shift_cte_m"]),
        scenario=dict(launch_deg=LAUNCH_DEG, bias_dps=BIAS_DPS, drift_mps=DRIFT_MPS, start_err_m=START_ERR_M,
                      start_along_m=START_ALONG_M),
        waypoints_latlon={n: [[round(lat, 8), round(lon, 8)] for _, lat, lon in pts] for n, pts in paths.items()},
        built=datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
        note="Simulated with the real navigator and strategy code; boat and GPS are models fitted from the logs. "
             "The boat's left/center/right.csv are not changed.",
    )
    with open(args.json_out, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)

    print(f"\n{'spacing':>7s} {'center':>7s} {'unsafe%':>8s} {'hit%':>6s} {'p1 clear':>9s} {'time':>7s}   per start lane: unsafe% / time / lane changes")
    for e in [present, centred] + table:
        tag = "present" if e is present else "centred" if e is centred else ("  <-- chosen" if e is chosen else "")
        lanes_txt = "  ".join(f"{n[0].upper()} {v['unsafe_pct']:4.1f}% {v['mean_time_s']:5.1f}s {v['lane_changes_per_run']:.2f}"
                              for n, v in e["lanes"].items())
        print(f"{e['spacing_m']:7.2f} {e['center_off_m']:+7.2f} {e['unsafe_pct']:8.2f} {e['hit_pct']:6.2f} "
              f"{e['clearance_p1_m']:9.2f} {e['mean_time_s']:6.2f}s   {lanes_txt}   {tag}")
    print(f"\nChosen: lanes at {lanes['left']:+.2f} / {lanes['center']:+.2f} / {lanes['right']:+.2f} m "
          f"(spacing {chosen['spacing_m']} m); best start lane: {best_lane}")
    print(f"Wrote {args.json_out} and {args.lanes_out}{os.sep}{{left,center,right}}.csv")


if __name__ == "__main__":
    main()
