#!/usr/bin/env python3
"""Replay recorded GPS observations through mode 06 without hardware or dynamics.

The recorded positions and MANUAL/AUTO transitions are inputs, even when the
replayed commands differ from the historical commands. This checks the guard's
responses to real observations; it cannot predict the boat's counterfactual
path, race time, or whether a physical bank impact would have been prevented.

Example:
    python3 replay_gps_guard.py logs/multipath_log_20261117_151246.csv
"""

import argparse
import contextlib
import csv
import io
import json
import math
from collections import defaultdict
from pathlib import Path

import multipath_boat as boat
from auto_strategies import create_auto_strategy
from gps_guard import GpsAwareController


ROOT = Path(__file__).resolve().parent


def _number(value):
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _flag(value):
    return str(value).lower() in ("1", "true")


def snapshot_from_row(row, now):
    """Reconstruct the receiver snapshot using recorded observation ages.

    The host logging timestamp does not replace either GGA/RMC source UTC or
    observation age. Missing accuracy fields stay unknown for historical logs.
    """
    mapping = {
        "lat": "latitude", "lon": "longitude", "age_sec": "gps_age_sec",
        "num_sats": "num_sats", "hdop": "hdop", "sog_mps": "gps_sog_mps",
        "cog_deg": "gps_cog_deg", "diff_age_sec": "gps_diff_age_sec",
        "alt_m": "gps_alt_m", "heading_deg": "legacy_heading_deg",
        "course_deg": "gps_course_deg", "course_speed_mps": "gps_course_speed_mps",
        "course_age_sec": "gps_course_age_sec", "course_gga_skew_sec": "gps_course_gga_skew_sec",
        "position_heading_deg": "position_heading_deg",
        "position_heading_age_sec": "position_heading_age_sec",
        "position_speed_mps": "position_speed_mps", "hacc_m": "hacc_m",
        "hacc_age_sec": "hacc_age_sec", "hacc_gga_skew_sec": "hacc_gga_skew_sec",
    }
    snap = {key: _number(row.get(column)) for key, column in mapping.items()}
    quality = _number(row.get("fix_quality"))
    snap.update(
        valid=_flag(row.get("gps_valid")), fix_quality=int(quality) if quality is not None else 0,
        course_valid=_flag(row.get("gps_course_valid")), gps_utc=row.get("gps_utc", ""),
        course_utc=row.get("gps_course_utc", ""), hacc_source=row.get("hacc_source", ""),
    )
    for time_key, age_key in (("course_time_mono", "course_age_sec"),
                              ("position_heading_time_mono", "position_heading_age_sec")):
        age = snap[age_key]
        snap[time_key] = now - age if age is not None and age >= 0 else None
    return snap


def _controller(config, paths):
    nav_params = boat.resolve_navigation_params(config.get("navigation", {}))
    autonomous = dict(config.get("autonomous", {}), strategy="06_gps_aware", use_04_control=False)
    strategy = create_auto_strategy(autonomous, nav_params)
    navigator = boat.MultipathNavigator(paths, nav_params, strategy=strategy)
    return GpsAwareController(navigator)


def replay_rows(rows, config, paths, source="observations"):
    """Return aggregate commands, transitions, and rejected jumps for CSV rows."""
    controller = _controller(config, paths)
    durations = defaultdict(float)
    reasons = defaultdict(float)
    transitions, jumps, samples = [], [], []
    seen_jump_epochs = set()
    previous_mode, previous_status = "MANUAL", None
    finished = False
    auto_entries = 0
    no_heading_full = 0.0
    recorded_no_heading_full = 0.0
    unobserved_gap = 0.0
    timestamps = [_number(row.get("timestamp")) for row in rows]
    if any(value is None for value in timestamps):
        raise ValueError("Every replay row needs a finite timestamp")
    if any(b < a for a, b in zip(timestamps, timestamps[1:])):
        raise ValueError("Replay timestamps must be nondecreasing")
    start = timestamps[0] if timestamps else 0.0
    cycle = 1.0 / float(boat.CONTROL_HZ) if hasattr(boat, "CONTROL_HZ") else .05
    for index, row in enumerate(rows):
        now = 100.0 + timestamps[index] - start
        elapsed = timestamps[index + 1] - timestamps[index] if index + 1 < len(rows) else cycle
        # Do not pretend the controller was observed across a missing log segment.
        dt = max(0.0, min(.25, elapsed))
        mode = row.get("mode", "MANUAL")
        snap = snapshot_from_row(row, now)
        if mode != "AUTO":
            if previous_mode == "AUTO":
                controller.reset()
            controller.observe_manual(snap, _number(row.get("rc_throttle_norm")) or 0.0, now)
            previous_mode, previous_status, finished = mode, None, False
            continue
        unobserved_gap += max(0.0, elapsed - dt)
        if previous_mode != "AUTO":
            # The real AUTO entry also discards the displacement-heading anchor.
            snap = dict(snap, position_heading_deg=None, position_heading_age_sec=None,
                        position_heading_time_mono=None, position_speed_mps=None)
            command = controller.enter_auto(snap, now)
            auto_entries += 1
        elif finished:
            command = dict(finished=True, steer_norm=0.0, throttle_norm=0.0,
                           gps_guard_state="FINISHED", gps_guard_reason="accepted_finish")
        else:
            command = controller.update(snap, now)
        finished = finished or bool(command.get("finished", False))
        state = command.get("gps_guard_state", "UNKNOWN")
        reason = command.get("gps_guard_reason", "")
        throttle = float(command.get("throttle_norm", 0.0))
        heading = controller.heading_info.get("heading_deg")
        sample = dict(line=index + 2, timestamp=timestamps[index], seconds_from_start=now - 100.0,
                      gps_utc=snap["gps_utc"], fix_quality=snap["fix_quality"], state=state,
                      reason=reason, throttle_norm=throttle, steer_norm=command.get("steer_norm", 0.0),
                      heading_deg=heading, finished=bool(command.get("finished", False)),
                      innovation_m=command.get("gps_guard_innovation_m"),
                      accepted_latitude=command.get("gps_guard_accepted_latitude"),
                      accepted_longitude=command.get("gps_guard_accepted_longitude"))
        samples.append(sample)
        durations[state] += dt
        reasons[reason] += dt
        if heading is None and throttle >= 1.0 - 1e-9:
            no_heading_full += dt
        original_esc = _number(row.get("esc_us"))
        if not row.get("boat_heading_deg") and original_esc is not None and original_esc >= config["pwm"]["esc_fwd_max_us"]:
            recorded_no_heading_full += dt
        if (state, reason) != previous_status:
            transitions.append(dict(sample))
        if "jump" in str(reason).lower():
            epoch = (auto_entries, snap["gps_utc"])
            if epoch not in seen_jump_epochs:
                seen_jump_epochs.add(epoch)
                jumps.append(dict(sample))
        previous_mode, previous_status = mode, (state, reason)
    return dict(
        source=str(source), strategy="06_gps_aware", rows=len(rows), auto_entries=auto_entries,
        observed_auto_sec=round(sum(durations.values()), 6),
        unobserved_auto_gap_sec=round(unobserved_gap, 6),
        state_seconds={key: round(value, 6) for key, value in sorted(durations.items())},
        reason_seconds={key: round(value, 6) for key, value in sorted(reasons.items())},
        full_throttle_without_heading_sec=round(no_heading_full, 6),
        recorded_full_throttle_without_heading_sec=round(recorded_no_heading_full, 6),
        rejected_jump_count=len(jumps), rejected_jumps=jumps, transitions=transitions,
        samples=samples,
        limitation="Recorded positions are fixed inputs. This replay evaluates commands, not a counterfactual physical path.",
    )


def replay_log(log_path, config=None, path_root=ROOT):
    config = boat.load_calibration(ROOT / "calibration.json") if config is None else config
    paths = {lane: boat.load_waypoints_csv(Path(path_root) / filename) for lane, filename in boat.PATH_FILES.items()}
    with Path(log_path).open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    # Keep the CLI JSON valid if the existing navigator writes diagnostic text.
    with contextlib.redirect_stdout(io.StringIO()):
        return replay_rows(rows, config, paths, source=log_path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logs", nargs="+", type=Path, help="Recorded telemetry CSV files")
    parser.add_argument("--config", type=Path, default=ROOT / "calibration.json")
    parser.add_argument("--json-output", type=Path, help="Save summary JSON to a new file (existing files are not overwritten)")
    args = parser.parse_args(argv)
    config = boat.load_calibration(args.config)
    reports = []
    for path in args.logs:
        report = replay_log(path, config)
        report.pop("samples")  # Per-cycle data remains available to Python callers.
        reports.append(report)
    text = json.dumps(reports, indent=2, allow_nan=False)
    if args.json_output is not None:
        with args.json_output.open("x", encoding="utf-8") as handle:
            handle.write(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
