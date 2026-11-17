#!/usr/bin/env python3
"""Summarise a 05 test-square run from a real telemetry log (no simulator).

    python3 analyse_square.py logs/multipath_log_YYYYMMDD_HHMMSS.csv

For every leg it reports where the boat actually was relative to the centerline (d, + = right) and how far
it was from the line it was told to follow. The legs along the course ("left line", "right line") show
whether the code's +-bank and +-line boundaries sit where the team sees them on the water.
"""

import csv
import math
import re
import sys

import multipath_boat as mb
from path_guidance import StraightLineGuidance

LEG = re.compile(r"square leg (\d+): \+-([0-9.]+) m (.+)")


def course_frame():
    paths = mb.load_waypoints_csv(mb.PATH_FILES["center"])
    ref_lat, ref_lon = paths[0][1], paths[0][2]
    pts = [mb.latlon_to_enu(lat, lon, ref_lat, ref_lon) for _, lat, lon in paths]
    return StraightLineGuidance(pts[0], pts[-1], 10.0, 0.0), ref_lat, ref_lon


def main(path):
    frame, ref_lat, ref_lon = course_frame()
    legs = {}
    for row in csv.DictReader(open(path, newline="", encoding="utf-8")):
        m = LEG.match(row.get("prediction_status") or "")
        if not m or not row["latitude"]:
            continue
        x, y = mb.latlon_to_enu(float(row["latitude"]), float(row["longitude"]), ref_lat, ref_lon)
        s, d = frame.frame(x, y)
        key = (int(m.group(1)), float(m.group(2)), m.group(3))
        legs.setdefault(key, []).append((s, d, float(row["cte_right_m"]), row["fix_quality"]))
    if not legs:
        print("No test-square rows found (prediction_status 'square leg ...'). Was the run made with --test-square?")
        return 1

    print(f"{'leg':>3}  {'boundary':>9}  {'side':<11} {'rows':>5}  {'RTK%':>4}  {'d mean':>7}  {'d min..max':>14}"
          f"  {'|err| mean':>10}  {'|err| max':>9}")
    for (n, h, side), rows in sorted(legs.items()):
        d = [r[1] for r in rows]
        err = [abs(r[2]) for r in rows]
        rtk = 100.0 * sum(r[3] == "4" for r in rows) / len(rows)
        print(f"{n:>3}  {'+-%.2f m' % h:>9}  {side:<11} {len(rows):>5}  {rtk:>4.0f}  {sum(d) / len(d):>+7.2f}"
              f"  {min(d):>+6.2f}..{max(d):>+6.2f}  {sum(err) / len(err):>10.2f}  {max(err):>9.2f}")
    print("\nAlong-course legs (the boundaries themselves): error measured after the boat has settled "
          "(skipping the first 3 m of each leg).")
    for (n, h, side), rows in sorted(legs.items()):
        if side not in ("left line", "right line"):
            continue
        start = rows[0][0]
        settled = [r for r in rows if abs(r[0] - start) >= 3.0]
        if not settled:
            continue
        target = -h if side == "left line" else h
        err = [r[1] - target for r in settled]
        print(f"  {side:<10} target d {target:+.2f} m: actual mean {target + sum(err) / len(err):+.2f} m, "
              f"bias {sum(err) / len(err):+.2f} m, rms error {math.sqrt(sum(e * e for e in err) / len(err)):.2f} m")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1]))
