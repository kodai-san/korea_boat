#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fit_boat_model.py
Fit the boat dynamics model (boat_model.py) to manual-driving telemetry and evaluate it on held-out data.

[Method]
1. Use only segments where the boat is actually driven on the water (no carrying, no stuck-on-the-bank,
   no fading battery). Windows that span a GPS fix-quality change (RTK <-> DGPS position jump) are dropped.
2. Chop each segment into 30 s blocks; every 3rd block is TEST, the rest TRAIN (blocked split, so test
   data is never adjacent-in-time to a training sample inside the same block -> no leakage via autocorrelation).
3. Inside each block, start a 10 s simulation window every 1 s from the smoothed measured state,
   replay the logged servo/ESC pulses open-loop, and compare simulated vs GPS positions every 0.5 s.
4. Fit all parameters (plus per session: water-current drift, battery speed scale, and the rudder-centred
   turning bias, which changes between logs) with robust least squares on TRAIN windows,
   grid-searching the actuator delays.
5. Report position error at 2 / 5 / 10 / 20 s horizons on TEST windows against two naive baselines:
   constant velocity (CV) and constant turn rate + velocity (CTRV).
6. AUTO data (small steering corrections at cruise/full throttle, the regime the controller lives in):
   every AUTO segment goes into TRAIN except the AUTO run of the --auto-holdout log, which is AUTO_TEST
   (never fitted; its logged commands are replayed the same way). Run once per held-out run to score each.

Usage: python3 fit_boat_model.py [--horizon 20]   -> writes boat_model_params.json and sim_results/*.png
"""

import argparse
import os

import numpy as np
from scipy.optimize import least_squares

from boat_model import BoatModel, LogRun, PARAM_NAMES, DT, delayed_commands

# (log file, start s, end s, kind). Times trimmed to on-water driving after inspecting each log.
# kind "manual" -> blocked TRAIN/TEST split; kind "auto" -> TRAIN, or AUTO_TEST for the --auto-holdout log.
# Each log is its own session (water current + battery speed scale fitted per log).
SEGMENTS = [
    ("logs/multipath_log_20261116_104352.csv", 0.0, 54.5, "manual"),
    ("logs/multipath_log_20261116_104352.csv", 54.8, 82.6, "auto"),        # Short AUTO run (1890 us), RTK fix
    ("logs/multipath_log_20261116_104352.csv", 890.0, 1060.0, "manual"),
    ("logs/multipath_log_20261116_143810.csv", 30.0, 320.0, "manual"),     # Battery faded after ~320 s
    ("logs/multipath_log_20261116_171303.csv", 35.0, 455.0, "manual"),     # First log with Doppler columns
    ("logs/multipath_log_20261116_172140.csv", 66.5, 225.0, "manual"),
    ("logs/multipath_log_20261116_172709.csv", 0.0, 48.0, "manual"),
    ("logs/multipath_log_20261116_172709.csv", 113.0, 228.0, "manual"),    # Carried along the bank afterwards
    ("logs/multipath_log_20261116_172140.csv", 18.8, 66.3, "auto"),        # AUTO run 1 (linear throttle blend)
    ("logs/multipath_log_20261116_172709.csv", 48.2, 112.6, "auto"),       # AUTO run 2 (binary full throttle)
]

BLOCK_SEC = 30.0
TEST_EVERY = 3
HORIZON_SEC = 20.0
STRIDE_SEC = 1.0
SAMPLE_SEC = 0.5
EVAL_HORIZONS = [2.0, 5.0, 10.0]
MIN_START_SPEED = 0.4
OUT_DIR = "sim_results"
PARAMS_FILE = "boat_model_params.json"
AUTO_HOLDOUT = "20261116_172709"   # Session whose AUTO run is held out (keeps validate-auto on run 2 honest)

INIT = {
    "kappa_1030": 0.5, "kappa_1250": 0.45, "kappa_1750": 0.1, "kappa_2130": -0.25,
    "T_yaw": 0.5,
    "U_1650": -0.4, "U_1830": 0.65, "U_1890": 1.15, "U_1950": 1.3, "U_1990": 1.45,
    "T_acc": 1.5, "T_dec": 2.5, "c_turn": 0.2,
}
BOUNDS = {
    "kappa_1030": (-2, 2), "kappa_1250": (-2, 2), "kappa_1750": (-2, 2), "kappa_2130": (-2, 2),
    "T_yaw": (0.05, 5),
    "U_1650": (-2, 0), "U_1830": (0, 4), "U_1890": (0, 4), "U_1950": (0, 4), "U_1990": (0, 4),
    "T_acc": (0.1, 10), "T_dec": (0.1, 15), "c_turn": (0, 1),
}
DRIFT_BOUND = 0.5
REF_SESSION = "20261116_171303"   # Speed scale fixed at 1.0 for this log (first log with Doppler speed)
SCALE_BOUNDS = (0.5, 2.0)
YAW_BIAS_BOUND = 0.5


def build_windows():
    """Return a dict of window arrays for TRAIN and TEST, plus run objects and block list."""
    runs = {}
    sessions = sorted({session_of(p) for (p, _, _, _) in SEGMENTS})
    h = int(round(HORIZON_SEC / DT))
    marks = np.arange(int(SAMPLE_SEC / DT), h + 1, int(SAMPLE_SEC / DT))
    win = {"train": [], "test": [], "auto_test": []}
    blocks = []

    def add_windows(run, path, split, bs, be):
        s = bs
        while s + HORIZON_SEC <= be + 1e-6:
            i = run.index_at(s)
            fwd = np.all(run.esc[max(0, i - 40):i + 1] >= 1800)
            fq = run.fix_quality[i:i + h + 1]
            same_fix = len(fq) == h + 1 and fq.min() > 0 and fq.min() == fq.max()
            if run.speed[i] >= MIN_START_SPEED and fwd and same_fix:
                win[split].append({
                    "path": path, "i": i, "session": sessions.index(session_of(path)),
                    "target": np.stack([run.x_gps[i + marks], run.y_gps[i + marks]], axis=1),
                })
            s += STRIDE_SEC

    for path, t0, t1, kind in SEGMENTS:
        run = runs.setdefault(path, LogRun(path))
        if kind == "auto":
            split = "auto_test" if session_of(path) == AUTO_HOLDOUT else "train"
            blocks.append((path, t0, t1, session_of(path), "auto_test" if split == "auto_test" else "auto_train"))
            add_windows(run, path, split, t0, t1)
            continue
        n_blocks = int((t1 - t0) // BLOCK_SEC)
        for b in range(n_blocks):
            split = "test" if b % TEST_EVERY == TEST_EVERY - 1 else "train"
            bs, be = t0 + b * BLOCK_SEC, t0 + (b + 1) * BLOCK_SEC
            blocks.append((path, bs, be, session_of(path), split))
            add_windows(run, path, split, bs, be)
    return runs, sessions, win, blocks, h, marks


def session_of(path):
    """Session name = log time stamp, e.g. logs/multipath_log_20261116_171303.csv -> 20261116_171303."""
    return os.path.basename(path)[len("multipath_log_"):-len(".csv")]


def stack(runs, wins, h, servo_delay, esc_delay):
    st0 = np.array([runs[w["path"]].state_at(w["i"]) for w in wins]).T
    servo = np.array([delayed_commands(runs[w["path"]].servo, w["i"], h, servo_delay) for w in wins])
    esc = np.array([delayed_commands(runs[w["path"]].esc, w["i"], h, esc_delay) for w in wins])
    sess = np.array([w["session"] for w in wins])
    target = np.array([w["target"] for w in wins])
    return st0, servo, esc, sess, target


def unpack(theta, sessions):
    """theta = [model params, (drift_x, drift_y) per session, speed scale per non-reference session,
    yaw bias per session]. Returns params and env = (drift, scale, yaw_bias), each indexed by session."""
    n_p, n_s = len(PARAM_NAMES), len(sessions)
    params = dict(zip(PARAM_NAMES, theta[:n_p]))
    drift = theta[n_p:n_p + 2 * n_s].reshape(n_s, 2)
    scale = np.ones(n_s)
    scale[[i for i, s in enumerate(sessions) if s != REF_SESSION]] = theta[n_p + 2 * n_s:n_p + 3 * n_s - 1]
    yaw_bias = theta[n_p + 3 * n_s - 1:]
    return params, (drift, scale, yaw_bias)


def simulate_windows(model, env, st0, servo, esc, sess):
    drift, scale, yaw_bias = env
    pos, _ = model.rollout(st0, servo, esc, drift=(drift[sess, 0], drift[sess, 1]),
                           speed_scale=scale[sess], yaw_bias=yaw_bias[sess])
    return pos


def fit(runs, sessions, train, h, marks, servo_delay, esc_delay, theta0=None):
    st0, servo, esc, sess, target = stack(runs, train, h, servo_delay, esc_delay)
    n_s = len(sessions)
    if theta0 is None:
        theta0 = np.r_[[INIT[k] for k in PARAM_NAMES], np.zeros(2 * n_s), np.ones(n_s - 1), np.zeros(n_s)]
    lb = np.r_[[BOUNDS[k][0] for k in PARAM_NAMES], -DRIFT_BOUND * np.ones(2 * n_s),
               SCALE_BOUNDS[0] * np.ones(n_s - 1), -YAW_BIAS_BOUND * np.ones(n_s)]
    ub = np.r_[[BOUNDS[k][1] for k in PARAM_NAMES], DRIFT_BOUND * np.ones(2 * n_s),
               SCALE_BOUNDS[1] * np.ones(n_s - 1), YAW_BIAS_BOUND * np.ones(n_s)]

    def resid(theta):
        params, env = unpack(theta, sessions)
        pos = simulate_windows(BoatModel(params, servo_delay, esc_delay), env, st0, servo, esc, sess)
        return (pos[:, marks, :] - target).ravel()

    res = least_squares(resid, np.clip(theta0, lb + 1e-9, ub - 1e-9), bounds=(lb, ub),
                        loss="soft_l1", f_scale=1.0, x_scale="jac", max_nfev=300)
    rms = float(np.sqrt(np.mean(res.fun ** 2)))
    return res.x, rms


def horizon_errors(pos_pred, target, marks):
    """Position error (m) at each EVAL_HORIZON for every window."""
    out = {}
    for hz in EVAL_HORIZONS:
        k = int(np.where(marks == int(round(hz / DT)))[0][0])
        out[hz] = np.hypot(pos_pred[:, marks[k], 0] - target[:, k, 0], pos_pred[:, marks[k], 1] - target[:, k, 1])
    return out


def baselines(st0, h):
    """Constant-velocity and constant-turn-rate rollouts from the same initial state."""
    x0, y0, psi0, u0, r0 = st0
    tt = np.arange(h + 1) * DT
    psi_cv = psi0[:, None] + 0 * tt
    psi_ct = psi0[:, None] + r0[:, None] * tt
    res = {}
    for name, psi in (("CV", psi_cv), ("CTRV", psi_ct)):
        dx = np.cumsum(np.c_[np.zeros(len(x0)), u0[:, None] * np.sin(psi[:, :-1]) * DT], axis=1)
        dy = np.cumsum(np.c_[np.zeros(len(x0)), u0[:, None] * np.cos(psi[:, :-1]) * DT], axis=1)
        res[name] = np.stack([x0[:, None] + dx, y0[:, None] + dy], axis=2)
    return res


def evaluate(model, env, runs, wins, h, marks):
    st0, servo, esc, sess, target = stack(runs, wins, h, model.servo_delay, model.esc_delay)
    preds = {"model": simulate_windows(model, env, st0, servo, esc, sess)}
    preds.update(baselines(st0, h))
    table = {}
    for name, pos in preds.items():
        errs = horizon_errors(pos, target, marks)
        table[name] = {f"{hz:g}s": {"median_m": float(np.median(e)), "p90_m": float(np.percentile(e, 90))}
                       for hz, e in errs.items()}
    return table


def print_table(title, table, n):
    print(f"\n{title}  ({n} windows)  position error median / p90 (m)")
    print("  " + "horizon".ljust(10) + "".join(name.rjust(18) for name in table))
    for hz in EVAL_HORIZONS:
        key = f"{hz:g}s"
        print("  " + key.ljust(10) + "".join(
            f"{table[name][key]['median_m']:7.2f} / {table[name][key]['p90_m']:5.2f}".rjust(18) for name in table))


def plot_results(model, env, sessions, runs, blocks):
    drift, scale, yaw_bias = env
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(OUT_DIR, exist_ok=True)

    # 1) Open-loop replay of whole held-out blocks (TEST) and whole AUTO runs (AUTO_TEST).
    def plot_replays(sel, fname, title):
        ncol = 3
        nrow = max(1, int(np.ceil(len(sel) / ncol)))
        fig, axes = plt.subplots(nrow, ncol, figsize=(5 * ncol, 4.5 * nrow), squeeze=False)
        for ax, (path, bs, be, session, _) in zip(np.ravel(axes), sel):
            run = runs[path]
            i0, i1 = run.index_at(bs), run.index_at(be)
            while i0 < i1 and run.speed[i0] < MIN_START_SPEED:
                i0 += 1
            if i1 - i0 < 40:
                ax.set_visible(False)
                continue
            h = i1 - i0
            servo = delayed_commands(run.servo, i0, h, model.servo_delay)
            esc = delayed_commands(run.esc, i0, h, model.esc_delay)
            si = sessions.index(session)
            pos, _ = model.rollout(run.state_at(i0), servo, esc, drift=tuple(drift[si]), speed_scale=scale[si],
                                   yaw_bias=yaw_bias[si])
            ax.plot(run.x_gps[i0:i1 + 1], run.y_gps[i0:i1 + 1], "k-", lw=1.5, label="real (GPS)")
            ax.plot(pos[0, :, 0], pos[0, :, 1], "r--", lw=1.5, label="simulated")
            for k in range(0, h + 1, 100):
                ax.plot([run.x_gps[i0 + k], pos[0, k, 0]], [run.y_gps[i0 + k], pos[0, k, 1]], ":", c="gray", lw=0.8)
            err = np.hypot(pos[0, -1, 0] - run.x_gps[i1], pos[0, -1, 1] - run.y_gps[i1])
            fq = sorted(set(run.fix_quality[i0:i1 + 1]))
            ax.set_title(f"{session} t={run.t[i0]:.0f}-{be:.0f}s fix {[int(q) for q in fq]}\n"
                         f"end error {err:.1f} m over {h * DT:.0f} s", fontsize=9)
            ax.set_aspect("equal", "datalim")
            ax.grid(alpha=0.3)
        np.ravel(axes)[0].legend(fontsize=8)
        for ax in np.ravel(axes)[len(sel):]:
            ax.set_visible(False)
        fig.suptitle(title)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT_DIR, fname), dpi=80)
        plt.close(fig)

    plot_replays([b for b in blocks if b[4] == "test"], "test_block_replays.png",
                 "TEST blocks: logged servo/ESC replayed open-loop (gray dotted = 5 s ticks)")
    plot_replays([b for b in blocks if b[4] in ("auto_test", "auto_train")], "auto_replays.png",
                 f"AUTO runs, logged servo/ESC replayed open-loop ({AUTO_HOLDOUT} held out, others in TRAIN)")

    # 2) Fitted actuator maps.
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.5))
    s = np.linspace(1030, 2130, 200)
    a1.plot(s, model.kappa(s), "r-")
    a1.axhline(0, c="k", lw=0.5)
    a1.axvline(1750, c="gray", ls=":", label="calibrated centre 1750")
    a1.set_xlabel("servo pulse (us)")
    a1.set_ylabel("turn curvature (1/m), + = right")
    a1.set_title("Steering map (at 1 m/s: yaw rate deg/s = curvature x 57)")
    a1.legend()
    e = np.linspace(1650, 1990, 200)
    a2.plot(e, model.steady_speed(e), "b-")
    a2.axhline(0, c="k", lw=0.5)
    a2.set_xlabel("ESC pulse (us)")
    a2.set_ylabel("steady surge speed (m/s)")
    a2.set_title("Throttle map")
    for ax in (a1, a2):
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "actuator_maps.png"), dpi=90)
    plt.close(fig)


def main():
    global HORIZON_SEC, EVAL_HORIZONS, AUTO_HOLDOUT, PARAMS_FILE
    parser = argparse.ArgumentParser(description="Fit the boat dynamics model to telemetry logs")
    parser.add_argument("--horizon", type=float, default=HORIZON_SEC, help="Simulation window length used for fitting (s)")
    parser.add_argument("--auto-holdout", default=AUTO_HOLDOUT, help="Session whose AUTO run is held out as AUTO_TEST")
    parser.add_argument("--out", default=PARAMS_FILE, help="Output parameter file")
    args = parser.parse_args()
    HORIZON_SEC = args.horizon
    AUTO_HOLDOUT = args.auto_holdout
    PARAMS_FILE = args.out
    EVAL_HORIZONS = sorted({2.0, 5.0, 10.0, HORIZON_SEC})

    runs, sessions, win, blocks, h, marks = build_windows()
    print(f"[DATA] {len(win['train'])} TRAIN, {len(win['test'])} TEST, {len(win['auto_test'])} AUTO_TEST windows; "
          f"sessions: {sessions}")

    best = None
    theta = None
    for servo_delay in (0, 1, 2, 3, 4):
        for esc_delay in (0, 2):
            theta, rms = fit(runs, sessions, win["train"], h, marks, servo_delay, esc_delay, theta0=theta)
            print(f"[FIT] servo_delay={servo_delay * DT:.2f}s esc_delay={esc_delay * DT:.2f}s -> train RMS {rms:.3f} m")
            if best is None or rms < best[0]:
                best = (rms, servo_delay, esc_delay, theta.copy())

    rms, servo_delay, esc_delay, theta = best
    params, env = unpack(theta, sessions)
    drift, scale, yaw_bias = env
    model = BoatModel(params, servo_delay, esc_delay)
    print(f"\n[BEST] servo delay {servo_delay * DT:.2f}s, ESC delay {esc_delay * DT:.2f}s, train RMS {rms:.3f} m")
    for k in PARAM_NAMES:
        print(f"  {k:16s} = {params[k]:+.4f}")
    for s, d, k, b in zip(sessions, drift, scale, yaw_bias):
        print(f"  [{s}] drift = ({d[0]:+.3f}, {d[1]:+.3f}) m/s (east, north), speed scale = {k:.3f}, "
              f"yaw bias = {b:+.3f} 1/m (= {np.degrees(b * 1.3):+.1f} deg/s centred at full throttle, 1.3 m/s)")

    metrics = {}
    for split in ("train", "test", "auto_test"):
        metrics[split] = evaluate(model, env, runs, win[split], h, marks)
        print_table(split.upper(), metrics[split], len(win[split]))

    print("\nTEST windows per log: model position error median (m)")
    metrics["test_by_session"] = {}
    for si, sname in enumerate(sessions):
        sub = [w for w in win["test"] if w["session"] == si]
        if not sub:
            continue
        tab = evaluate(model, env, runs, sub, h, marks)["model"]
        metrics["test_by_session"][sname] = tab
        print(f"  {sname}  n={len(sub):3d}  " + "  ".join(f"{k}: {v['median_m']:.2f}" for k, v in tab.items()))

    model.save(PARAMS_FILE, extra={
        "drift_by_session": {s: list(map(float, d)) for s, d in zip(sessions, drift)},
        "speed_scale_by_session": {s: float(k) for s, k in zip(sessions, scale)},
        "yaw_bias_by_session": {s: float(b) for s, b in zip(sessions, yaw_bias)},
        "horizon_sec": HORIZON_SEC,
        "segments": SEGMENTS,
        "auto_holdout": AUTO_HOLDOUT,
        "metrics": metrics,
    })
    print(f"\n[SAVED] {PARAMS_FILE}")
    plot_results(model, env, sessions, runs, blocks)
    print(f"[SAVED] {OUT_DIR}/test_block_replays.png, auto_replays.png, actuator_maps.png")


if __name__ == "__main__":
    main()
