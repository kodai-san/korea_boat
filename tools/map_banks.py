#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
map_banks.py
Estimate the banks of the Haesucheon course from every telemetry log and draw a map of how the boat travels.

The course is a straight line, so the banks are modelled as two straight lines parallel to it. Each bank
is placed on the places the boat hit it; the width between them comes from the data. The on-site width
estimate (about 11 m, approximate; --channel-width) is only used for a side with no bank hits.

Method:
  1. Read every logs/multipath_log_*.csv and keep RTK-fix positions only (fix_quality 4, ~2 cm).
  2. Express each position in course coordinates: "along" = metres from center-lane WP1 toward WP65,
     "off" = metres right (+) or left (-) of the center lane.
  3. Classify every position, because the logs also record the boat being carried to and from the water
     (MANUAL mode keeps logging while people walk it along the bank or lift it out):
       powered  motor on within the last POWERED_SEC, or coasting up to COAST_SEC after that   -> on water
       carried  moving faster than CARRY_SPEED with the motor at neutral for longer than COAST_SEC -> on land
       idle     not moving, motor at neutral (could be on land or drifting)                    -> not used
     Only powered positions are used for the banks.
  4. Use the course itself (WP1..WP65) only. Before WP1 is the launch area, where the motor often runs
     while people hold, lower or lift the boat. Find the outermost powered position on each side (a point counts only if MIN_SUPPORT positions lie
     within SUPPORT_RADIUS of it, which ignores single GPS glitches). This is where the water definitely
     reaches; the gap between the two sides is the narrowest the channel can be.
  5. Look for bank hits: under power the boat heads out toward one side, stops at its furthest point,
     reverse is used, and it moves back off within CRASH_BACK_SEC (it may sit stuck first). Nowhere
     nearby may the boat have been further out (that rules out reverse turns in open water). On each side
     the hits that agree within CRASH_GROUP_M form the bank; the rest are not used.
     Also look for long contacts: the boat stood still for at least CONTACT_SEC with the motor pushing.
  6. Place each bank on the median of its largest group of agreeing hits (at least two). A side without
     hits goes CHANNEL_WIDTH from the other; with no hits at all, line a bank up with a contact, otherwise
     centre a CHANNEL_WIDTH channel on the powered positions.
     Powered positions beyond a bank placed on hits are reported as anomalies (e.g. the boat pushed onto
     the bank or lifted out) instead of widening or shifting the channel.

The GPS antenna is on the boat's centre line, so the hull reaches about half a beam further out than the
logged position (boat_half_width_m, stored but not added to the offsets).

Outputs:
  course_banks.json   bank lines in course and lat/lon coordinates, evidence, per-station envelope
  bank_map.html       interactive map (open in a browser)

Lanes on the map: the lanes in left/center/right.csv are drawn faint. Beside them the map draws either the
same lanes centred between the banks or, once tools/optimise_lanes.py has been run for these banks, the
optimised lanes from course_lanes_optimised.json. The boat's lane CSV files are never changed.

Usage (from the repo root):
  python tools/map_banks.py
  python tools/map_banks.py --channel-width 11.0 --html-out somewhere.html
"""

import argparse
import csv
import datetime
import glob
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
R_EARTH = 6_378_137.0

CHANNEL_WIDTH = 11.0                         # rough on-site estimate; only used for a side with no bank hits
STRAIGHT_FROM, STRAIGHT_TO = -120.0, 110.0   # along-course range where the banks are drawn (m); estimated on WP1..WP65
BIN_M = 2.0                                  # station spacing for the envelope (m)
MIN_SUPPORT, SUPPORT_RADIUS = 3, 0.25        # glitch filter for outermost points
ESC_NEUTRAL, ESC_DEADBAND = 1800, 25         # motor counts as "on" outside neutral +/- deadband (us)
POWERED_SEC, COAST_SEC = 1.0, 6.0
CARRY_SPEED = 0.45                           # m/s; faster than drift, slower than nothing else on land
CONTACT_SEC, CONTACT_SPREAD = 10.0, 0.3
# Bank hit: heading out under power, the boat stops at its furthest point, reverse is used, and it later
# moves back off (or sits stuck first).
CRASH_MIN_OFF = 3.0                          # only beyond this offset (clear of the lanes at +/-1.5 m)
CRASH_RUN_IN_M, CRASH_ANGLE_DEG = 0.20, 20.0 # in the 3 s before: at least this far toward the side, this steep
CRASH_BACK_M, CRASH_BACK_SEC = 0.15, 20.0    # then back off at least this far within this time
CRASH_GROUP_M = 0.5                          # a bank is the largest group of hits within this spread
CRASH_NEAR_ALONG, CRASH_BEYOND_M = 6.0, 0.25 # a hit is the furthest out within +/- this along (else a turn)
BOAT_HALF_WIDTH_M = 0.25                     # assumed; measure the hull and update


def load_lane(name):
    with open(os.path.join(ROOT, f"{name}.csv"), newline="") as f:
        return [(float(r["latitude"]), float(r["longitude"])) for r in csv.DictReader(f)]


LANES = {n: load_lane(n) for n in ("left", "center", "right")}
REF_LAT, REF_LON = LANES["center"][0]


def enu(lat, lon):
    return (math.radians(lon - REF_LON) * R_EARTH * math.cos(math.radians(REF_LAT)),
            math.radians(lat - REF_LAT) * R_EARTH)


def latlon(x, y):
    return (REF_LAT + math.degrees(y / R_EARTH),
            REF_LON + math.degrees(x / (R_EARTH * math.cos(math.radians(REF_LAT)))))


_ex, _ey = enu(*LANES["center"][-1])
COURSE_LEN = math.hypot(_ex, _ey)
TX, TY = _ex / COURSE_LEN, _ey / COURSE_LEN
NX, NY = TY, -TX
COURSE_BRG = math.degrees(math.atan2(TX, TY)) % 360.0


def to_course(lat, lon):
    x, y = enu(lat, lon)
    return x * TX + y * TY, x * NX + y * NY


def from_course(along, off):
    return latlon(along * TX + off * NX, along * TY + off * NY)


LANE_OFF = {n: round(to_course(*LANES[n][0])[1], 3) for n in LANES}


def motor_on(esc_us):
    return abs(esc_us - ESC_NEUTRAL) > ESC_DEADBAND


def classify(pts):
    """Label each fix powered / carried / idle (see module docstring)."""
    last_on = -1e9
    labels = []
    for i, p in enumerate(pts):
        if motor_on(p["esc"]):
            last_on = p["t"]
        a, b = pts[max(0, i - 5)], pts[min(len(pts) - 1, i + 5)]
        speed = math.hypot(b["a"] - a["a"], b["o"] - a["o"]) / max(0.1, b["t"] - a["t"])
        since = p["t"] - last_on
        if since <= COAST_SEC:
            labels.append("powered")
        elif speed > CARRY_SPEED:
            labels.append("carried")
        else:
            labels.append("idle")
    return labels


def load_sessions(log_dir):
    sessions = []
    for path in sorted(glob.glob(os.path.join(log_dir, "multipath_log_*.csv"))):
        with open(path, newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        pts, prev = [], None
        for x in rows:
            if x.get("fix_quality") != "4" or not x.get("latitude"):
                continue
            p = (x["latitude"], x["longitude"])
            if p == prev:
                continue
            prev = p
            a, o = to_course(float(p[0]), float(p[1]))
            pts.append(dict(t=float(x["timestamp"]), a=a, o=o, esc=int(x["esc_us"]),
                            auto=1 if x["mode"] == "AUTO" else 0))
        if not pts:
            continue
        for p, lab in zip(pts, classify(pts)):
            p["cls"] = lab
        sid = os.path.basename(path)[len("multipath_log_"):-4]
        start = datetime.datetime.strptime(sid, "%Y%m%d_%H%M%S")      # local (Korean) time from the file name
        sessions.append(dict(id=sid, start=start, t0=float(rows[0]["timestamp"]), pts=pts))
    return sessions


def clock(session, t):
    return (session["start"] + datetime.timedelta(seconds=t - session["t0"])).strftime("%H:%M:%S")


def in_stretch(p):
    return STRAIGHT_FROM <= p["a"] <= STRAIGHT_TO


def in_course(p):
    """Banks are estimated from the course itself (WP1..WP65). Before WP1 is the launch area, where the
    motor often runs while people hold, lower or lift the boat, so 'powered' there does not mean afloat."""
    return 0.0 <= p["a"] <= COURSE_LEN


def outermost(sessions, side):
    """Outermost supported powered position on one side (side=-1 left, +1 right)."""
    cand = [(side * p["o"], s, i) for s in sessions for i, p in enumerate(s["pts"])
            if p["cls"] == "powered" and in_course(p)]
    cand.sort(key=lambda c: c[0], reverse=True)
    for _, s, i in cand:
        p = s["pts"][i]
        support = sum(1 for q in s["pts"] if q["cls"] == "powered"
                      and math.hypot(q["a"] - p["a"], q["o"] - p["o"]) <= SUPPORT_RADIUS)
        if support >= MIN_SUPPORT:
            return dict(off=round(p["o"], 2), along=round(p["a"], 1), session=s["id"],
                        time=clock(s, p["t"]), auto=bool(p["auto"]))
    return None


def contacts(sessions):
    """Places where the boat stood still for CONTACT_SEC while the motor was pushing: pressed on a bank."""
    found = []
    for s in sessions:
        pts, i = s["pts"], 0
        while i < len(pts):
            p = pts[i]
            j = i
            while j + 1 < len(pts) and math.hypot(pts[j + 1]["a"] - p["a"], pts[j + 1]["o"] - p["o"]) < CONTACT_SPREAD:
                j += 1
            stay = pts[j]["t"] - p["t"]
            if stay >= CONTACT_SEC and in_course(p):
                block = pts[i:j + 1]
                pushing = sum(1 for q in block if motor_on(q["esc"])) / len(block)
                # must arrive and leave under power too, so it was on the water the whole time
                arrived = any(q["cls"] == "powered" for q in pts[max(0, i - 25):i])
                left_ = any(q["cls"] == "powered" for q in pts[j + 1:j + 26])
                if pushing >= 0.5 and arrived and left_:
                    o = sorted(q["o"] for q in block)
                    found.append(dict(off=round(o[len(o) // 2], 2), along=round(p["a"], 1), session=s["id"],
                                      time=clock(s, p["t"]), seconds=round(stay), pushing_pct=round(100 * pushing)))
            i = j + 1
    return found


def crashes(sessions):
    """Bank hits. The boat heads out toward one side under power, stops at the furthest point it reaches
    in a +/-3 s window, reverse is used within 3 s of that, and it then moves back off (possibly after
    sitting stuck). The furthest point is where the antenna was when the bow touched the bank."""
    rev = lambda p: p["esc"] < ESC_NEUTRAL - ESC_DEADBAND
    found = []
    for s in sessions:
        pts, last = s["pts"], -1e9
        for m, q in enumerate(pts):
            if q["cls"] != "powered" or not in_course(q) or abs(q["o"]) < CRASH_MIN_OFF or q["t"] - last < 5.0:
                continue
            side = 1 if q["o"] > 0 else -1
            win = pts[max(0, m - 60):m + 60]
            if any(side * p["o"] > side * q["o"] for p in win if abs(p["t"] - q["t"]) <= 3.0):
                continue                                   # not the furthest point
            before = [p for p in pts[max(0, m - 40):m] if q["t"] - p["t"] <= 3.0]
            after = [p for p in pts[m + 1:m + 300] if p["t"] - q["t"] <= CRASH_BACK_SEC]
            if len(before) < 4 or not after:
                continue
            b = before[0]
            run_in = side * (q["o"] - b["o"])
            ang = math.degrees(math.atan2(run_in, abs(q["a"] - b["a"])))
            back = max(side * (q["o"] - p["o"]) for p in after)
            reversed_ = any(rev(p) for p in win if -1.0 <= p["t"] - q["t"] <= 3.0)
            if run_in >= CRASH_RUN_IN_M and ang >= CRASH_ANGLE_DEG and back >= CRASH_BACK_M and reversed_:
                peak = max((side * (y["o"] - x["o"]) / (y["t"] - x["t"]) for x in before for y in before + [q]
                            if 0.4 <= y["t"] - x["t"] <= 0.6), default=0.0)
                stuck = next((p["t"] - q["t"] for p in after if side * (q["o"] - p["o"]) >= CRASH_BACK_M), 0.0)
                found.append(dict(side="left" if side < 0 else "right", off=round(q["o"], 2), along=round(q["a"], 1),
                                  session=s["id"], time=clock(s, q["t"]), auto=bool(q["auto"]),
                                  approach_deg=round(ang), speed_in_mps=round(peak, 2), backed_out_m=round(back, 2),
                                  stuck_s=round(stuck, 1)))
                last = q["t"]
    # A reverse turn in open water looks the same, but there the boat has been further out nearby on
    # other occasions. A bank hit is the furthest out under power within CRASH_NEAR_ALONG along.
    powered = [(p["a"], p["o"]) for s in sessions for p in s["pts"] if p["cls"] == "powered" and in_course(p)]
    for c in found:
        sd = -1 if c["side"] == "left" else 1
        c["further_nearby"] = sum(1 for a, o in powered if abs(a - c["along"]) <= CRASH_NEAR_ALONG
                                  and sd * (o - c["off"]) > CRASH_BEYOND_M)
    found = [c for c in found if c["further_nearby"] < MIN_SUPPORT]
    # The bank on each side is the largest group of hits within CRASH_GROUP_M (outermost group on a tie).
    # Hits outside it are not used: inside it they are probably turns, beyond it anomalies.
    for side, sd in (("left", -1), ("right", 1)):
        hits = sorted((c for c in found if c["side"] == side), key=lambda c: -sd * c["off"])
        best = []
        for h in hits:
            g = [c for c in hits if 0 <= sd * (h["off"] - c["off"]) <= CRASH_GROUP_M]
            if len(g) > len(best):
                best = g
        for c in hits:
            c["anomaly"] = not (len(best) >= 2 and c in best)
            c["why_not_used"] = "" if not c["anomaly"] else (
                "only hit on this side" if len(best) < 2 else
                "beyond the other hits" if sd * c["off"] > sd * best[0]["off"] else "inside the other hits (probably a turn)")
    return found


def envelope(sessions):
    bins = {}
    for s in sessions:
        for p in s["pts"]:
            if p["cls"] == "powered" and in_course(p):
                bins.setdefault(math.floor(p["a"] / BIN_M), []).append(p["o"])
    out = []
    for b in sorted(bins):
        v = sorted(bins[b])
        if len(v) < MIN_SUPPORT:
            continue
        ok = lambda x: sum(1 for w in v if abs(w - x) <= SUPPORT_RADIUS) >= MIN_SUPPORT
        lo = next((x for x in v if ok(x)), v[0])
        hi = next((x for x in reversed(v) if ok(x)), v[-1])
        out.append(dict(along=round((b + 0.5) * BIN_M, 1), left=round(lo, 2), right=round(hi, 2), n=len(v)))
    return out


def bank_path_check(sessions, a_from=-115.0, a_to=-25.0):
    """The boat was carried along a path beside the right bank before the course. A path built along a
    straight bank runs parallel to it, so its angle to the course is a check on 'banks parallel to the
    course'. Returns its angle (deg), scatter (m) and mean offset."""
    bins = {}
    for s in sessions:
        for p in s["pts"]:
            if p["cls"] == "carried" and a_from <= p["a"] <= a_to:
                bins.setdefault(math.floor(p["a"] / BIN_M), []).append(p["o"])
    pts = [((b + 0.5) * BIN_M, sorted(v)[len(v) // 2]) for b, v in bins.items() if len(v) >= MIN_SUPPORT]
    if len(pts) < 5:
        return None
    n = len(pts)
    ma, mo = sum(a for a, _ in pts) / n, sum(o for _, o in pts) / n
    slope = sum((a - ma) * (o - mo) for a, o in pts) / sum((a - ma) ** 2 for a, _ in pts)
    resid = (sum((o - (mo + slope * (a - ma))) ** 2 for a, o in pts) / n) ** 0.5
    return dict(along_range_m=[a_from, a_to], stations=n, angle_deg=round(math.degrees(math.atan(slope)), 2),
                scatter_m=round(resid, 2), mean_off_m=round(mo, 2), drift_over_course_m=round(slope * COURSE_LEN, 2))


def place_banks(width, env_left, env_right, contact_list, hit_list, sessions):
    """Place each bank on the median of its group of bank hits (the antenna position at the hit). If only
    one side has hits, put the other bank `width` away; with no hits, fall back to a long contact, then to
    centring a `width` channel on the powered positions."""
    notes = []
    good = {sd: [c["off"] for c in hit_list if c["side"] == sd and not c["anomaly"]] for sd in ("left", "right")}
    med = lambda v: (sorted(v)[len(v) // 2] if len(v) % 2 else sum(sorted(v)[len(v) // 2 - 1:len(v) // 2 + 1]) / 2)
    left_c = [c for c in contact_list if c["off"] <= env_left["off"] + 0.3]
    right_c = [c for c in contact_list if c["off"] >= env_right["off"] - 0.3]
    fixed, given, width_range = set(), width, None
    if good["left"] and good["right"]:
        left, right = med(good["left"]), med(good["right"]); fixed = {"left", "right"}
        width = right - left
        width_range = [round(min(good["right"]) - max(good["left"]), 2), round(max(good["right"]) - min(good["left"]), 2)]
        how = (f"both banks on bank hits (left: median of {len(good['left'])}, right: median of {len(good['right'])}); "
               f"width {width:.2f} m from the data")
        notes.append(f"Measured width {width:.2f} m between the antenna positions at the hits "
                     f"({width_range[0]:.2f} to {width_range[1]:.2f} m from the spread of the hits); "
                     f"the on-site estimate was about {given:g} m.")
    elif good["left"]:
        left = med(good["left"]); right = left + width; fixed = {"left"}
        how = f"left bank on the median of {len(good['left'])} bank hits; right bank one channel width ({width:g} m) away"
    elif good["right"]:
        right = med(good["right"]); left = right - width; fixed = {"right"}
        how = f"right bank on the median of {len(good['right'])} bank hits; left bank one channel width ({width:g} m) away"
    elif left_c:
        left = min(c["off"] for c in left_c); right = left + width
        how = "left bank on the contact; right bank one channel width away"
    elif right_c:
        right = max(c["off"] for c in right_c); left = right - width
        how = "right bank on the contact; left bank one channel width away"
    else:
        mid = (env_left["off"] + env_right["off"]) / 2.0
        left, right = mid - width / 2.0, mid + width / 2.0
        how = "no bank hit or contact; channel centred on the powered positions"

    # Powered positions beyond a bank that was placed on hits are anomalies (GPS, or the boat lifted or
    # pushed onto the bank), not water. Elsewhere, keep every powered position inside the channel.
    # On a side placed on hits the limit is the outermost hit used (the hits scatter around the median).
    lim_l = min([left] + good["left"]) - 0.1 if "left" in fixed else left - 0.1
    lim_r = max([right] + good["right"]) + 0.1 if "right" in fixed else right + 0.1
    beyond = {}
    for s in sessions:
        for p in s["pts"]:
            if p["cls"] != "powered" or not in_course(p):
                continue
            sd = "left" if p["o"] < lim_l else "right" if p["o"] > lim_r else None
            if sd:
                e = beyond.setdefault((sd, s["id"]), dict(side=sd, session=s["id"], points=0, worst=p["o"],
                                                         along=round(p["a"], 1), first=clock(s, p["t"])))
                e["points"] += 1
                e["last"] = clock(s, p["t"])
                if abs(p["o"]) > abs(e["worst"]):
                    e["worst"], e["along"] = p["o"], round(p["a"], 1)
    anomalies = []
    for e in beyond.values():
        e["worst"] = round(e["worst"], 2)
        if e["side"] in fixed:
            anomalies.append(e)
    for c in contact_list:
        c["anomaly"] = ("left" in fixed and c["off"] < left - CRASH_GROUP_M) or \
                       ("right" in fixed and c["off"] > right + CRASH_GROUP_M)
    if "right" not in fixed and right < env_right["off"]:
        shift = env_right["off"] - right
        if fixed:
            notes.append(f"The boat has been {shift:.2f} m beyond the right bank under power; the channel may be wider than {width} m.")
        else:
            left, right = left + shift, right + shift
            if shift > 0.05:
                notes.append(f"Shifted {shift:.2f} m right so the furthest-right powered position stays inside the channel.")
    if "left" not in fixed and left > env_left["off"]:
        shift = left - env_left["off"]
        if fixed:
            notes.append(f"The boat has been {shift:.2f} m beyond the left bank under power; the channel may be wider than {width} m.")
        else:
            left, right = left - shift, right - shift
            if shift > 0.05:
                notes.append(f"Shifted {shift:.2f} m left so the furthest-left powered position stays inside the channel.")
    for e in sorted(anomalies, key=lambda e: e["session"]):
        notes.append(f"Left out as an anomaly: {e['points']} powered positions beyond the {e['side']} bank in "
                     f"{e['session']} ({e['first']} to {e['last']}), furthest {e['worst']:+.2f} m at {e['along']} m along.")
    # inner edge of the water actually used, without anomalies
    in_l = min((p["o"] for s in sessions for p in s["pts"] if p["cls"] == "powered" and in_course(p)
                and p["o"] >= lim_l), default=env_left["off"])
    in_r = max((p["o"] for s in sessions for p in s["pts"] if p["cls"] == "powered" and in_course(p)
                and p["o"] <= lim_r), default=env_right["off"])
    return dict(left=round(left, 2), right=round(right, 2), width=round(width, 2), width_range=width_range,
                width_source="measured from bank hits" if len(fixed) == 2 else "given (--channel-width)",
                min_width_from_data=round(in_r - in_l, 2), how=how, notes=notes, fixed=sorted(fixed),
                anomalies=sorted(anomalies, key=lambda e: e["session"]),
                room_left_m=round(in_l - left, 2), room_right_m=round(right - in_r, 2))


def find_runs(log_dir, left_bank, right_bank, min_sec=30.0):
    """AUTO runs of at least min_sec from the most recent day of logs, labelled A, B, C... in time order.
    Keeps every fix (RTK or not) so the whole run can be drawn; bank distances use RTK-fix points only."""
    paths = sorted(glob.glob(os.path.join(log_dir, "multipath_log_*.csv")))
    if not paths:
        return []
    day = os.path.basename(paths[-1])[len("multipath_log_"):][:8]
    runs = []
    for path in paths:
        sid = os.path.basename(path)[len("multipath_log_"):-4]
        if not sid.startswith(day):
            continue
        with open(path, newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        if not rows:
            continue
        start, t0 = datetime.datetime.strptime(sid, "%Y%m%d_%H%M%S"), float(rows[0]["timestamp"])
        i = 0
        while i < len(rows):
            if rows[i]["mode"] != "AUTO":
                i += 1
                continue
            j = i
            while j < len(rows) and rows[j]["mode"] == "AUTO":
                j += 1
            seg = rows[i:j]
            ta, tb = float(seg[0]["timestamp"]), float(seg[-1]["timestamp"])
            if tb - ta >= min_sec:
                pts, prev = [], None
                for x in seg:
                    if not x.get("latitude") or x.get("fix_quality") in ("", "0"):
                        continue
                    p = (x["latitude"], x["longitude"])
                    if p == prev:
                        continue
                    prev = p
                    a, o = to_course(float(p[0]), float(p[1]))
                    pts.append([round(float(x["timestamp"]) - ta, 2), round(a, 2), round(o, 2), int(x["fix_quality"]),
                                int(x["esc_us"]), int(x["servo_us"])])
                rtk = [p for p in pts if p[3] == 4]
                dist = sum(math.hypot(q[1] - p[1], q[2] - p[2]) for p, q in zip(rtk, rtk[1:]) if math.hypot(q[1] - p[1], q[2] - p[2]) < 3)
                wps = [int(x["wp_no"]) for x in seg if x.get("wp_no") not in ("", "0")]
                ctes = [abs(float(x["cte_right_m"])) for x in seg if x.get("cte_right_m")]
                nxt = rows[j] if j < len(rows) else None
                end = ("log ended" if nxt is None else
                       "stopped by the driver (full reverse)" if int(nxt["esc_us"]) < ESC_NEUTRAL - 50 else
                       "finished at WP65" if wps and max(wps) >= 65 else "returned to MANUAL")
                near_l = min(rtk, key=lambda p: p[2] - left_bank) if rtk else None
                near_r = min(rtk, key=lambda p: right_bank - p[2]) if rtk else None
                fq = {}
                for x in seg:
                    fq[x["fix_quality"]] = fq.get(x["fix_quality"], 0) + 1
                runs.append(dict(
                    session=sid, start=(start + datetime.timedelta(seconds=ta - t0)).strftime("%H:%M:%S"),
                    duration_s=round(tb - ta, 1), strategy=seg[0].get("auto_strategy", "") or "not logged",
                    wp=[min(wps), max(wps)] if wps else None, end=end,
                    speed_mps=round(dist / max(0.1, rtk[-1][0] - rtk[0][0]), 2) if len(rtk) > 1 else None,
                    max_cte_m=round(max(ctes), 2) if ctes else None,
                    full_throttle_pct=round(100.0 * sum(1 for x in seg if int(x["esc_us"]) >= 1985) / len(seg)),
                    rtk_fix_pct=round(100.0 * fq.get("4", 0) / len(seg)),
                    min_to_left_m=round(near_l[2] - left_bank, 2) if near_l else None,
                    min_to_left_at=near_l[1] if near_l else None,
                    min_to_right_m=round(right_bank - near_r[2], 2) if near_r else None,
                    min_to_right_at=near_r[1] if near_r else None,
                    pts=pts,
                ))
            i = j
    for k, r in enumerate(runs):
        r["label"] = chr(ord("A") + k)
    return runs


def downsample(pts, min_step=0.35):
    out, last = [], None
    code = {"powered": 0, "carried": 1, "idle": 2}
    for p in pts:
        key = (p["cls"], p["auto"])
        if last is None or math.hypot(p["a"] - last[0], p["o"] - last[1]) >= min_step or key != last[2]:
            out.append([round(p["a"], 2), round(p["o"], 2), p["auto"], code[p["cls"]]])
            last = (p["a"], p["o"], key)
    return out


def main():
    ap = argparse.ArgumentParser(description="Estimate the course banks from telemetry logs")
    ap.add_argument("--logs", default=os.path.join(ROOT, "logs"))
    ap.add_argument("--channel-width", type=float, default=CHANNEL_WIDTH, help="Bank-to-bank width in metres, used only for a side with no bank hits")
    ap.add_argument("--min-run-sec", type=float, default=30.0, help="Shortest AUTO segment shown as a run")
    ap.add_argument("--json-out", default=os.path.join(ROOT, "course_banks.json"))
    ap.add_argument("--html-out", default=os.path.join(ROOT, "bank_map.html"))
    args = ap.parse_args()

    sessions = load_sessions(args.logs)
    counts = {c: sum(1 for s in sessions for p in s["pts"] if p["cls"] == c) for c in ("powered", "carried", "idle")}
    env_l, env_r = outermost(sessions, -1), outermost(sessions, +1)
    cts = contacts(sessions)
    hits = crashes(sessions)
    placed = place_banks(args.channel_width, env_l, env_r, cts, hits, sessions)
    env = envelope(sessions)
    # Lanes re-centred between the banks, keeping their spacing. The boat still follows left/center/right.csv;
    # these waypoints are only written to the JSON.
    lane_shift = round((placed["left"] + placed["right"]) / 2.0 - LANE_OFF["center"], 3)
    lanes_adj = dict(
        source="centred",
        shift_m=lane_shift,
        spacing_m=round(LANE_OFF["right"] - LANE_OFF["center"], 2),
        off_m={n: round(o + lane_shift, 3) for n, o in LANE_OFF.items()},
        room_to_bank_m=dict(left=round(LANE_OFF["left"] + lane_shift - placed["left"], 2),
                            right=round(placed["right"] - LANE_OFF["right"] - lane_shift, 2)),
        waypoints_latlon={n: [[round(v, 8) for v in from_course(a, o + lane_shift)]
                              for a, o in (to_course(*p) for p in LANES[n])] for n in LANES},
        note="center lane on the channel centre line, lane spacing unchanged; the boat's lane CSV files are not changed",
    )
    # If tools/optimise_lanes.py has been run for these banks, draw its lanes instead of the centred ones.
    opt_path = os.path.join(ROOT, "course_lanes_optimised.json")
    if os.path.exists(opt_path):
        with open(opt_path, encoding="utf-8") as f:
            opt = json.load(f)
        if abs(opt["banks"]["left"] - placed["left"]) < 0.011 and abs(opt["banks"]["right"] - placed["right"]) < 0.011:
            ch, pr = opt["chosen"], opt["present"]
            lanes_adj = dict(
                source="optimised", shift_m=round(ch["center_off_m"] - LANE_OFF["center"], 3),
                spacing_m=ch["spacing_m"], off_m=ch["lanes_off_m"], room_to_bank_m=ch["room_to_bank_m"],
                start_lane=ch["start_lane"], safe_clearance_m=opt["safe_clearance_m"],
                runs_per_lane=opt["runs_per_lane"], settings=opt["settings"],
                chosen={k: ch[k] for k in ("unsafe_pct", "hit_pct", "clearance_p1_m", "mean_time_s", "lanes")},
                present={k: pr[k] for k in ("unsafe_pct", "hit_pct", "clearance_p1_m", "mean_time_s", "lanes")},
                waypoints_latlon=opt["waypoints_latlon"],
                note="from tools/optimise_lanes.py (course_lanes_optimised.json, lanes_optimised/*.csv); "
                     "the boat's lane CSV files are not changed",
            )
        else:
            print("Note: course_lanes_optimised.json was made for different banks; run tools/optimise_lanes.py again.")
    lanes_adj["changed"] = any(abs(lanes_adj["off_m"][n] - LANE_OFF[n]) > 0.01 for n in LANE_OFF)
    banks = dict(
        model=f"straight lines parallel to the course, {placed['width']} m apart",
        course_frame=dict(origin_lat=REF_LAT, origin_lon=REF_LON, bearing_deg=round(COURSE_BRG, 2),
                          length_m=round(COURSE_LEN, 2), waypoints=len(LANES["center"]),
                          along="metres from center-lane WP1 toward WP65",
                          off="metres right (+) / left (-) of the center lane"),
        lanes_off_m=LANE_OFF,
        lanes_adjusted=lanes_adj,
        valid_along_m=[STRAIGHT_FROM, STRAIGHT_TO],
        channel_width_m=placed["width"],
        channel_width_given_m=args.channel_width,
        channel_width_source=placed["width_source"],
        channel_width_range_m=placed["width_range"],
        min_width_from_data_m=placed["min_width_from_data"],
        left_bank=dict(off_m=placed["left"], room_m=placed["room_left_m"]),
        right_bank=dict(off_m=placed["right"], room_m=placed["room_right_m"]),
        placement=placed["how"],
        notes=placed["notes"],
        outermost_powered=dict(left=env_l, right=env_r),
        contacts=cts,
        bank_hits=hits,
        anomalies=placed["anomalies"],
        bank_path_check=bank_path_check(sessions),
        boat_half_width_m=BOAT_HALF_WIDTH_M,
        position_classes=counts,
        bank_lines_latlon={
            side: [[round(v, 8) for v in from_course(a, off)] for a in (STRAIGHT_FROM, STRAIGHT_TO)]
            for side, off in (("left", placed["left"]), ("right", placed["right"]))
        },
        distance_to_bank="left: off - left_bank.off_m ; right: right_bank.off_m - off (minus boat_half_width_m for the hull)",
        sources=dict(logs=len(sessions), rtk_fix_points=sum(len(s["pts"]) for s in sessions),
                     built=datetime.datetime.now().strftime("%Y-%m-%d %H:%M")),
        envelope=env,
    )
    with open(args.json_out, "w", encoding="utf-8") as f:
        json.dump(banks, f, indent=2)

    runs = find_runs(args.logs, placed["left"], placed["right"], args.min_run_sec)
    payload = dict(banks=banks, tracks=[dict(id=s["id"], pts=downsample(s["pts"])) for s in sessions], runs=runs)
    with open(os.path.join(HERE, "bank_map_template.html"), encoding="utf-8") as f:
        html = f.read()
    html = html.replace("/*__DATA__*/null", json.dumps(payload, separators=(",", ":")).replace("</", "<\\/"))
    with open(args.html_out, "w", encoding="utf-8") as f:
        f.write(html)

    print(f"{len(sessions)} logs with RTK fix: {counts['powered']} powered, {counts['carried']} carried, {counts['idle']} idle positions")
    print(f"Outermost powered: left {env_l['off']:+.2f} m ({env_l['session']} {env_l['time']}), "
          f"right {env_r['off']:+.2f} m ({env_r['session']} {env_r['time']}, {'AUTO' if env_r['auto'] else 'MANUAL'})")
    for c in hits:
        print(f"Bank hit {c['side']}: {c['off']:+.2f} m at {c['along']} m along, in at {c['approach_deg']} deg "
              f"{c['speed_in_mps']} m/s, backed out {c['backed_out_m']} m ({c['session']} {c['time']})"
              + ("  [anomaly]" if c["anomaly"] else ""))
    for c in cts:
        print(f"Contact{' [anomaly]' if c.get('anomaly') else ''}: {c['off']:+.2f} m at {c['along']} m along, {c['seconds']} s, motor pushing {c['pushing_pct']}% ({c['session']} {c['time']})")
    wr = placed["width_range"]
    print(f"Banks: left {placed['left']:+.2f} m, right {placed['right']:+.2f} m ({placed['width']} m wide; "
          + (f"{wr[0]}-{wr[1]} m from the spread of the hits" if wr else
             f"data needs at least {placed['min_width_from_data']} m") + f"). {placed['how']}.")
    if not wr:
        print(f"Room beyond the outermost powered position: left {placed['room_left_m']} m, right {placed['room_right_m']} m")
    print(f"Lanes drawn ({lanes_adj['source']}, spacing {lanes_adj['spacing_m']} m): left {lanes_adj['off_m']['left']:+.2f}, center {lanes_adj['off_m']['center']:+.2f}, "
          f"right {lanes_adj['off_m']['right']:+.2f} m; {lanes_adj['room_to_bank_m']['left']} m and "
          f"{lanes_adj['room_to_bank_m']['right']} m to the banks")
    for n in placed["notes"]:
        print("Note:", n)
    print(f"Bank path check: {banks['bank_path_check']}")
    for r in runs:
        print(f"Run {r['label']}: {r['session']} {r['start']} {r['duration_s']} s, WP {r['wp']}, {r['strategy']}, "
              f"{r['end']}; closest to left bank {r['min_to_left_m']} m, right bank {r['min_to_right_m']} m")
    print(f"Wrote {args.json_out} and {args.html_out}")


if __name__ == "__main__":
    main()
