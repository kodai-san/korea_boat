#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
phone_imu.py - use an iPhone as the boat's gyro (yaw rate) sensor.  Python 3.6+, standard library only
(the offline 'calibrate-gps' mode additionally needs numpy).

Sources
  --source phyphox     phyphox "Gyroscope (rotation rate)" experiment with "Allow remote access".
                       The Jetson polls http://<phone>:8080 (--url). USB cable + iPhone Personal Hotspot:
                       http://172.20.10.1:8080 (default)
  --source sensorlog   SensorLog app pushes JSON/CSV over UDP to this Jetson (--port 5555)

Calibration (needed: the phone is NOT mounted flat, so the boat's yaw axis is a mix of phone x/y/z)
  The boat's yaw rate is the phone's 3-axis rotation rate projected on the boat's vertical axis:
        yaw_rate_right = -( up . (w - bias) )        (+ = turning right / clockwise seen from above)
  'up' (unit vector in the phone frame) and 'bias' are stored in imu_calibration.json.
  Re-calibrate whenever the phone is re-mounted.

  python3 phone_imu.py calibrate --source phyphox
        live, ~1 min: hold the boat still 10 s, then turn it slowly RIGHT by 90-180 deg (and optionally LEFT)
  python3 phone_imu.py calibrate-gps --imu-file "Gyroscope ... .xlsx" --boat-log logs/multipath_log_X.csv
        offline (laptop OK): fits 'up' so that gyro heading changes match RTK GPS course changes
        --imu-file accepts a phyphox .xlsx export or a phone_imu log CSV. --valid-from/--valid-to are
        seconds from the start of the IMU file.

Other modes
  discover   show source, fields/buffers and sample rate
  test       live calibrated yaw rate (turn RIGHT -> must be POSITIVE)
  log        write logs/phone_imu_<time>.csv until Ctrl+C (raw gx,gy,gz + calibrated yaw rate)

Log timestamps are the Jetson's time.time(), the same clock as multipath_boat.py's 'timestamp' column.

Import:  from phone_imu import make_imu; imu = make_imu("phyphox"); imu.start(); imu.get_latest()
"""

import argparse
import csv
import datetime
import json
import math
import os
import re
import socket
import sys
import threading
import time
import urllib.request
import zipfile
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CAL_FILE = os.path.join(HERE, "imu_calibration.json")

# field / buffer names (first match wins; case-insensitive fallback)
SENSORLOG_KEYS = {
    "x": ["motionRotationRateX", "gyroRotationX", "rotationRateX"],
    "y": ["motionRotationRateY", "gyroRotationY", "rotationRateY"],
    "z": ["motionRotationRateZ", "gyroRotationZ", "rotationRateZ"],
    "heading": ["motionHeading", "locationTrueHeading", "locationMagneticHeading"],
}
PHYPHOX_BUFFERS = {
    "x": ["gyrX", "gyroX", "gx"],
    "y": ["gyrY", "gyroY", "gy"],
    "z": ["gyrZ", "gyroZ", "gz"],
    "t": ["gyr_time", "gyrT", "gyro_time", "t", "time"],
}


# ----------------------------------------------------------------------------------------------
# small vector helpers (no numpy on the live path)
# ----------------------------------------------------------------------------------------------
def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a):
    return math.sqrt(_dot(a, a))


def _to_float(v):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    if math.isnan(f) or math.isinf(f):
        return None
    return f


def _pick(d, candidates, override=None):
    if override:
        return override if override in d else None
    for k in candidates:
        if k in d:
            return k
    lower = {k.lower(): k for k in d}
    for k in candidates:
        if k.lower() in lower:
            return lower[k.lower()]
    return None


# ----------------------------------------------------------------------------------------------
# calibration
# ----------------------------------------------------------------------------------------------
class Calibration(object):
    """yaw_rate_right [rad/s] = -(up . (w - bias)); w = phone rotation rate [rad/s] in the phone frame."""

    def __init__(self, up=(0.0, 0.0, 1.0), bias=(0.0, 0.0, 0.0), scale=1.0, info=None, calibrated=False):
        n = _norm(up)
        self.up = [c / n for c in up]
        self.bias = list(bias)
        self.scale = float(scale)
        self.info = info or {}
        self.calibrated = calibrated

    def yaw_rate_rad(self, w):
        return -self.scale * _dot(self.up, [w[0] - self.bias[0], w[1] - self.bias[1], w[2] - self.bias[2]])

    def describe(self):
        tilt = [math.degrees(math.acos(max(-1.0, min(1.0, abs(c))))) for c in self.up]
        dominant = "xyz"[max(range(3), key=lambda i: abs(self.up[i]))]
        return ("up=[{:+.3f} {:+.3f} {:+.3f}] (boat vertical is closest to phone {} axis, {:.0f} deg off), "
                "bias=[{:+.4f} {:+.4f} {:+.4f}] rad/s, scale={:.3f}{}".format(
                    self.up[0], self.up[1], self.up[2], dominant, min(tilt),
                    self.bias[0], self.bias[1], self.bias[2], self.scale,
                    "" if self.calibrated else "  [UNCALIBRATED: assumes phone flat, screen up]"))

    @classmethod
    def load(cls, path):
        if not path or not os.path.exists(path):
            return cls()
        with open(path) as f:
            d = json.load(f)
        return cls(d["up"], d.get("bias", [0, 0, 0]), d.get("scale", 1.0), d, calibrated=True)

    def save(self, path, **info):
        d = {"up": [round(c, 5) for c in self.up], "bias": [round(c, 6) for c in self.bias],
             "scale": round(self.scale, 4),
             "formula": "yaw_rate_right = -scale * (up . (w - bias)), w = phone rotation rate in rad/s",
             "created": datetime.datetime.now().isoformat(timespec="seconds")}
        d.update(info)
        with open(path, "w") as f:
            json.dump(d, f, indent=2)
        print("Saved calibration to {}".format(path))


# ----------------------------------------------------------------------------------------------
# live sources
# ----------------------------------------------------------------------------------------------
class _IMUBase(object):
    def __init__(self, calibration=None, lpf_hz=3.0):
        self.cal = calibration or Calibration()
        self.lpf_hz = lpf_hz
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.sender = None
        self.format = None
        self.n_samples = 0
        self.last_rx = 0.0
        self.w = None                 # latest raw rotation rate (rad/s, phone frame)
        self.yaw_rate_dps = None      # calibrated, + = right
        self.yaw_rate_filt_dps = None
        self.yaw_int_deg = 0.0        # integrated calibrated yaw (relative heading, drifts slowly)
        self.heading_deg = None       # compass-style heading if the source provides one
        self.latest_raw = None
        self.used_keys = {}
        self.last_error = None
        self.callbacks = []           # fn(sample_dict) called for every sample (from the receiver thread)
        self._prev_t = None

    def _ingest(self, t_jetson, w, heading=None, raw=None):
        yr = math.degrees(self.cal.yaw_rate_rad(w))
        with self.lock:
            dt = 0.0 if self._prev_t is None else max(0.0, min(0.2, t_jetson - self._prev_t))
            self._prev_t = t_jetson
            if self.yaw_rate_filt_dps is None or dt == 0.0:
                self.yaw_rate_filt_dps = yr
            else:
                a = dt / (dt + 1.0 / (2.0 * math.pi * self.lpf_hz))
                self.yaw_rate_filt_dps += a * (yr - self.yaw_rate_filt_dps)
            self.yaw_int_deg += yr * dt
            self.w = w
            self.yaw_rate_dps = yr
            if heading is not None:
                self.heading_deg = heading
            self.latest_raw = raw
            self.n_samples += 1
            self.last_rx = t_jetson
            sample = self._snapshot_locked(t_jetson)
        for cb in self.callbacks:
            cb(sample)

    def _snapshot_locked(self, now):
        age = now - self.last_rx if self.last_rx > 0 else 999.0
        return {
            "valid": age < 0.3 and self.yaw_rate_dps is not None,
            "recv_time": self.last_rx,
            "age_s": age,
            "w": self.w,
            "yaw_rate_dps": self.yaw_rate_dps,
            "yaw_rate_filt_dps": self.yaw_rate_filt_dps,
            "yaw_int_deg": self.yaw_int_deg,
            "heading_deg": self.heading_deg,
            "n_samples": self.n_samples,
            "sender": self.sender,
        }

    def get_latest(self):
        with self.lock:
            return self._snapshot_locked(time.time())

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2.0)


class SensorLogIMU(_IMUBase):
    """SensorLog pushes JSON objects (or CSV with a header line) over UDP."""

    def __init__(self, port=5555, calibration=None, lpf_hz=3.0, bind_ip="0.0.0.0"):
        _IMUBase.__init__(self, calibration, lpf_hz)
        self.port = port
        self.bind_ip = bind_ip
        self.sock = None
        self.csv_header = None

    def describe_source(self):
        return "UDP port {}".format(self.port)

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind_ip, self.port))
        self.sock.settimeout(0.5)
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _parse(self, payload):
        text = payload.decode("utf-8", errors="ignore").strip()
        if not text:
            return []
        if text[0] in "{[":
            self.format = "json"
            try:
                obj = json.loads(text)
                return [o for o in (obj if isinstance(obj, list) else [obj]) if isinstance(o, dict)]
            except ValueError:
                out = []
                for line in text.splitlines():
                    try:
                        o = json.loads(line.strip().rstrip(","))
                        if isinstance(o, dict):
                            out.append(o)
                    except ValueError:
                        pass
                return out
        self.format = "csv"
        out = []
        for line in text.splitlines():
            fields = [f.strip() for f in line.split(",")]
            if sum(1 for f in fields if _to_float(f) is not None) < len(fields) / 2.0:
                self.csv_header = fields
            elif self.csv_header and len(self.csv_header) == len(fields):
                out.append(dict(zip(self.csv_header, fields)))
        return out

    def _loop(self):
        while self.running:
            try:
                payload, addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            now = time.time()
            self.sender = addr[0]
            for d in self._parse(payload):
                keys = {a: _pick(d, SENSORLOG_KEYS[a]) for a in ("x", "y", "z", "heading")}
                self.used_keys = keys
                w = [_to_float(d.get(keys[a])) if keys[a] else None for a in ("x", "y", "z")]
                if any(c is None for c in w):
                    self.last_error = "rotation-rate x/y/z not all present (fields: {})".format(sorted(d)[:12])
                    continue
                hdg = _to_float(d.get(keys["heading"])) if keys["heading"] else None
                self._ingest(now, w, hdg if (hdg is not None and hdg >= 0) else None, d)

    def stop(self):
        _IMUBase.stop(self)
        if self.sock is not None:
            self.sock.close()


class PhyphoxIMU(_IMUBase):
    """Polls the phyphox remote-access server (/config, /get?buf=<t>|timebuf, /control?cmd=start)."""

    def __init__(self, url="http://172.20.10.1:8080", calibration=None, lpf_hz=3.0, poll_hz=25.0,
                 autostart=True, buffers=None):
        _IMUBase.__init__(self, calibration, lpf_hz)
        if not url.startswith("http"):
            url = "http://" + url
        self.url = url.rstrip("/")
        self.poll_hz = poll_hz
        self.autostart = autostart
        self.buf_override = buffers or {}
        self.buffers = []
        self.bufmap = {}
        self.measuring = None
        self.format = "phyphox-http"

    def describe_source(self):
        return "phyphox {} (buffers {})".format(self.url, self.bufmap)

    def _http_json(self, path, timeout=1.0):
        resp = urllib.request.urlopen(self.url + path, timeout=timeout)
        try:
            return json.loads(resp.read().decode("utf-8"))
        finally:
            resp.close()

    def start(self):
        cfg = self._http_json("/config", timeout=3.0)
        self.buffers = [b.get("name") for b in cfg.get("buffers", []) if isinstance(b, dict)]
        bufdict = {b: True for b in self.buffers}
        for a in ("x", "y", "z", "t"):
            self.bufmap[a] = _pick(bufdict, PHYPHOX_BUFFERS[a], self.buf_override.get(a))
        missing = [a for a in ("x", "y", "z", "t") if not self.bufmap[a]]
        if missing:
            raise RuntimeError("phyphox buffers for {} not found. Available: {}. Open the 'Gyroscope (rotation "
                               "rate)' experiment, or pass --buffers x=..,y=..,z=..,t=..".format(missing, self.buffers))
        self.sender = self.url
        self.used_keys = dict(self.bufmap)
        if self.autostart:
            self._http_json("/control?cmd=start", timeout=2.0)
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        b = self.bufmap
        last_t = None
        period = 1.0 / self.poll_hz
        while self.running:
            t_start = time.time()
            try:
                if last_t is None:
                    q = "/get?{}&{}&{}&{}".format(b["t"], b["x"], b["y"], b["z"])
                else:
                    q = "/get?{t}={v!r}&{x}={v!r}|{t}&{y}={v!r}|{t}&{z}={v!r}|{t}".format(
                        t=b["t"], x=b["x"], y=b["y"], z=b["z"], v=last_t)
                data = self._http_json(q, timeout=1.0)
                now = time.time()
                buf = data.get("buffer", {})
                cols = [buf.get(b[a], {}).get("buffer", []) or [] for a in ("t", "x", "y", "z")]
                self.measuring = data.get("status", {}).get("measuring")
                rows = [r for r in zip(*cols) if None not in r and (last_t is None or r[0] > last_t)]
                if rows:
                    t_new = rows[-1][0]
                    for t, x, y, z in rows:
                        self._ingest(now - (t_new - t), [x, y, z], raw={"t_phone": t})
                    last_t = t_new
                self.last_error = None
            except Exception as exc:
                self.last_error = str(exc)
                time.sleep(0.5)
            dt = time.time() - t_start
            if dt < period:
                time.sleep(period - dt)


def make_imu(source="phyphox", url="http://172.20.10.1:8080", port=5555, cal_file=DEFAULT_CAL_FILE,
             lpf_hz=3.0, autostart=True, buffers=None):
    cal = Calibration.load(cal_file)
    if source == "phyphox":
        return PhyphoxIMU(url, cal, lpf_hz, autostart=autostart, buffers=buffers)
    return SensorLogIMU(port, cal, lpf_hz)


# ----------------------------------------------------------------------------------------------
# live modes
# ----------------------------------------------------------------------------------------------
def wait_for_first(imu, timeout_s=30.0):
    print("Waiting for data from {} ...".format(imu.describe_source()))
    t0 = time.time()
    while imu.get_latest()["n_samples"] == 0:
        if time.time() - t0 > timeout_s:
            print("No samples in {:.0f} s (last error: {}). Is the app open, in the foreground, and "
                  "measuring / streaming?".format(timeout_s, imu.last_error))
            return False
        time.sleep(0.1)
    return True


def measure_rate(imu, secs=2.0):
    n0 = imu.get_latest()["n_samples"]
    time.sleep(secs)
    return (imu.get_latest()["n_samples"] - n0) / secs


def cmd_discover(imu):
    if not wait_for_first(imu):
        return
    hz = measure_rate(imu, 3.0)
    s = imu.get_latest()
    print("\nSource : {}\nFormat : {}\nRate   : {:.1f} Hz".format(imu.describe_source(), imu.format, hz))
    print("Fields : {}".format(imu.used_keys))
    if isinstance(imu, PhyphoxIMU):
        print("All phyphox buffers: {}".format(imu.buffers))
    elif imu.latest_raw:
        print("All SensorLog fields: {}".format(sorted(imu.latest_raw)))
    print("Latest raw w (rad/s): {}".format(["{:+.4f}".format(c) for c in s["w"]]))
    print("Calibration: {}".format(imu.cal.describe()))


def cmd_test(imu):
    if not wait_for_first(imu):
        return
    print("Calibration: {}".format(imu.cal.describe()))
    print("Turn the boat RIGHT (clockwise seen from above): yaw rate must go POSITIVE and 'turned' must "
          "count up. Ctrl+C to stop.\n")
    n_prev, t_prev, hz = imu.get_latest()["n_samples"], time.time(), 0.0
    while True:
        time.sleep(0.1)
        s = imu.get_latest()
        now = time.time()
        if now - t_prev >= 1.0:
            hz = (s["n_samples"] - n_prev) / (now - t_prev)
            n_prev, t_prev = s["n_samples"], now
        r = s["yaw_rate_filt_dps"] or 0.0
        n = int(max(-20, min(20, r / 5.0)))
        bar = (" " * (20 + min(0, n)) + ("<" * -n if n < 0 else "|") + (">" * n if n > 0 else "")).ljust(41)
        sys.stdout.write("\ryaw rate {:+7.1f} deg/s [{}] turned {:+7.1f} deg  {:5.1f} Hz  age {:4.0f} ms {}".format(
            r, bar, s["yaw_int_deg"], hz, s["age_s"] * 1000.0, "" if s["valid"] else "STALE"))
        sys.stdout.flush()


def cmd_log(imu, out_path):
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    f = open(out_path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["timestamp", "gx_rad_s", "gy_rad_s", "gz_rad_s", "yaw_rate_dps", "yaw_rate_filt_dps",
                "yaw_int_deg", "heading_deg"])
    lock = threading.Lock()
    count = [0]

    def on_sample(s):
        with lock:
            if f.closed:
                return
            w.writerow(["{:.3f}".format(s["recv_time"])] + ["{:.6f}".format(c) for c in s["w"]] +
                       ["{:.3f}".format(s["yaw_rate_dps"]), "{:.3f}".format(s["yaw_rate_filt_dps"]),
                        "{:.2f}".format(s["yaw_int_deg"]),
                        "" if s["heading_deg"] is None else "{:.2f}".format(s["heading_deg"])])
            count[0] += 1
            if count[0] % 200 == 0:
                f.flush()

    imu.callbacks.append(on_sample)
    print("Logging to {}\nCalibration: {}".format(out_path, imu.cal.describe()))
    if not imu.cal.calibrated:
        print("WARNING: no calibration file -> yaw rate assumes the phone is flat. Raw gx,gy,gz are logged, "
              "so the run can still be calibrated afterwards with calibrate-gps.")
    if not wait_for_first(imu, timeout_s=1e9):
        return
    try:
        while True:
            time.sleep(1.0)
            s = imu.get_latest()
            sys.stdout.write("\r{} samples  yaw rate {:+6.1f} deg/s  {}   ".format(
                count[0], s["yaw_rate_filt_dps"] or 0.0, "OK" if s["valid"] else "STALE"))
            sys.stdout.flush()
    finally:
        with lock:
            f.close()
        print("\nSaved {} samples to {}".format(count[0], out_path))


def _collect(imu, prompt_start, prompt_stop=None, duration=None):
    """Collect raw samples (t, w) between two Enter presses, or for a fixed duration."""
    buf = []
    lock = threading.Lock()

    def cb(s):
        with lock:
            buf.append((s["recv_time"], list(s["w"])))

    input(prompt_start)
    imu.callbacks.append(cb)
    if duration is not None:
        t_end = time.time() + duration
        while time.time() < t_end:
            sys.stdout.write("\r  {:4.1f} s left ".format(t_end - time.time()))
            sys.stdout.flush()
            time.sleep(0.2)
        print("")
    else:
        input(prompt_stop)
    imu.callbacks.remove(cb)
    with lock:
        return list(buf)


def _integrate(samples, bias):
    R = [0.0, 0.0, 0.0]
    for (t0, w0), (t1, w1) in zip(samples[:-1], samples[1:]):
        dt = t1 - t0
        if 0 < dt < 0.2:
            for i in range(3):
                R[i] += 0.5 * ((w0[i] - bias[i]) + (w1[i] - bias[i])) * dt
    return R


def cmd_calibrate(imu, cal_file):
    if not wait_for_first(imu):
        return
    print("\n=== Phone IMU mounting calibration ===")
    print("Mount the phone in its final position first. Re-run this whenever the phone is re-mounted.\n")
    still = _collect(imu, "Step 1/3: keep the boat completely STILL (on its stand or on calm water). "
                          "Press Enter to start 10 s ...", duration=10.0)
    if len(still) < 50:
        print("Too few samples ({}). Aborting.".format(len(still)))
        return
    n = float(len(still))
    bias = [sum(w[i] for _, w in still) / n for i in range(3)]
    noise = max(math.sqrt(sum((w[i] - bias[i]) ** 2 for _, w in still) / n) for i in range(3))
    print("  bias = [{:+.4f} {:+.4f} {:+.4f}] rad/s ({:.2f} deg/s max), noise {:.2f} deg/s".format(
        bias[0], bias[1], bias[2], math.degrees(max(abs(c) for c in bias)), math.degrees(noise)))
    if math.degrees(noise) > 1.0:
        print("  WARNING: the boat was not still (noise > 1 deg/s). Bias set to 0 instead.")
        bias = [0.0, 0.0, 0.0]

    right = _collect(imu, "\nStep 2/3: press Enter, then turn the boat slowly to the RIGHT (clockwise seen from "
                          "above) by 90-180 deg, keeping it level ...", "  ... press Enter when the turn is done ")
    R = _integrate(right, bias)
    ang = math.degrees(_norm(R))
    if ang < 45:
        print("  Only {:.0f} deg of rotation measured; need at least 45 deg. Aborting.".format(ang))
        return
    up = [-c / _norm(R) for c in R]          # right turn = negative rotation about 'up'
    print("  measured rotation {:.0f} deg about phone-frame axis [{:+.3f} {:+.3f} {:+.3f}]".format(ang, *[-c for c in up]))

    left = _collect(imu, "\nStep 3/3 (check, optional): press Enter, turn LEFT by 90-180 deg, press Enter again. "
                         "(Press Enter twice quickly to skip) ...", "  ... press Enter when done ")
    info = {"method": "live turn-in-place", "right_turn_deg": round(ang, 1)}
    if len(left) > 20:
        L = _integrate(left, bias)
        if _norm(L) > math.radians(30):
            up_l = [c / _norm(L) for c in L]
            disagree = math.degrees(math.acos(max(-1.0, min(1.0, _dot(up, up_l)))))
            print("  left-turn axis differs from right-turn axis by {:.1f} deg".format(disagree))
            info["left_right_axis_disagreement_deg"] = round(disagree, 1)
            if disagree > 10:
                print("  WARNING: > 10 deg disagreement - the boat probably rolled/pitched a lot during the turns. "
                      "Repeat slower, or use calibrate-gps.")
            s = [up[i] + up_l[i] for i in range(3)]
            up = [c / _norm(s) for c in s]
    cal = Calibration(up, bias, 1.0, calibrated=True)
    print("\n" + cal.describe())
    cal.save(cal_file, **info)
    imu.cal = cal
    print("Now run 'test' and turn right: the yaw rate must be positive.")


# ----------------------------------------------------------------------------------------------
# offline calibration against RTK GPS (numpy)
# ----------------------------------------------------------------------------------------------
def load_phyphox_xlsx(path):
    """Returns (epoch_times, wx, wy, wz) from a phyphox 'Gyroscope' .xlsx export (stdlib xlsx reader)."""
    ns = "{http://schemas.openxmlformats.org/spreadsheetml/2006/main}"
    z = zipfile.ZipFile(path)
    rows = []
    with z.open("xl/worksheets/sheet1.xml") as f:
        for _, el in ET.iterparse(f):
            if el.tag == ns + "row":
                vals = [c.find(ns + "v") for c in el.findall(ns + "c")]
                if len(vals) >= 4 and all(v is not None for v in vals[:4]):
                    rows.append([float(v.text) for v in vals[:4]])
                el.clear()
    meta = z.read("xl/worksheets/sheet3.xml").decode("utf-8", errors="ignore")
    starts = re.findall(r"(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+) UTC([+-]\d{2}):(\d{2})", meta)
    if not starts:
        raise ValueError("no START time found in the 'Metadata Time' sheet")
    if len(starts) > 2:
        print("WARNING: the phyphox recording was paused/resumed; times after the first pause may be off.")
    stamp, hh, mm = starts[0]
    tz = datetime.timezone(datetime.timedelta(hours=int(hh), minutes=int(mm) * (1 if int(hh) >= 0 else -1)))
    t0 = datetime.datetime.strptime(stamp, "%Y-%m-%d %H:%M:%S.%f").replace(tzinfo=tz).timestamp()
    return [t0 + r[0] for r in rows], [r[1] for r in rows], [r[2] for r in rows], [r[3] for r in rows]


def load_phone_imu_csv(path):
    t, x, y, z = [], [], [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            t.append(float(r["timestamp"]))
            x.append(float(r["gx_rad_s"]))
            y.append(float(r["gy_rad_s"]))
            z.append(float(r["gz_rad_s"]))
    return t, x, y, z


def cmd_calibrate_gps(args):
    import numpy as np
    wrap = lambda a: (a + 180.0) % 360.0 - 180.0

    if args.imu_file.lower().endswith(".xlsx"):
        t, x, y, z = load_phyphox_xlsx(args.imu_file)
    else:
        t, x, y, z = load_phone_imu_csv(args.imu_file)
    t = np.array(t)
    G = np.degrees(np.c_[x, y, z])
    t_from = t[0] + (args.valid_from or 0.0)
    t_to = t[0] + args.valid_to if args.valid_to else t[-1]
    print("IMU: {} samples, {:.0f} s, using {:.0f}-{:.0f} s".format(len(t), t[-1] - t[0], t_from - t[0], t_to - t[0]))

    fx, prev = [], None
    with open(args.boat_log) as f:
        for r in csv.DictReader(f):
            if r.get("latitude") and (r["latitude"], r["longitude"]) != prev:
                prev = (r["latitude"], r["longitude"])
                fx.append((float(r["timestamp"]), float(r["latitude"]), float(r["longitude"]),
                           int(r["fix_quality"] or 0)))
    fx = np.array(fx)
    tf = fx[:, 0]
    R = 6378137.0
    ex = np.radians(fx[:, 2] - fx[0, 2]) * R * math.cos(math.radians(fx[0, 1]))
    ny = np.radians(fx[:, 1] - fx[0, 1]) * R
    lo = np.searchsorted(tf, tf - 0.4)
    hi = np.clip(np.searchsorted(tf, tf + 0.4, side="right") - 1, 0, len(tf) - 1)
    dtt = tf[hi] - tf[lo]
    ok = dtt > 0.3
    vx = np.where(ok, (ex[hi] - ex[lo]) / np.where(ok, dtt, 1), 0)
    vy = np.where(ok, (ny[hi] - ny[lo]) / np.where(ok, dtt, 1), 0)
    course = np.degrees(np.arctan2(vx, vy))
    good = ok & (np.hypot(vx, vy) > args.min_speed) & (fx[:, 3] >= 4) & (tf > t_from) & (tf < t_to)
    print("Boat log: {} GPS fixes, {} usable (RTK float/fix, > {:.1f} m/s, inside the IMU window)".format(
        len(tf), int(good.sum()), args.min_speed))
    if good.sum() < 100:
        print("Not enough usable GPS data (need RTK and the boat moving). Check the time window.")
        return

    cum = np.vstack([np.concatenate([[0.0], np.cumsum(0.5 * (G[1:, i] + G[:-1, i]) * np.diff(t))])
                     for i in range(3)]).T

    def pairs(W):
        ia = np.where(good)[0]
        j = np.searchsorted(tf, tf[ia] + W)
        keep = j < len(tf)
        ia, j = ia[keep], j[keep]
        keep = good[j] & (np.abs(tf[j] - tf[ia] - W) < 0.3)
        return ia[keep], j[keep]

    def windows(W, lag):
        a, b = pairs(W)
        I = np.vstack([np.interp(tf[b] - lag, t, cum[:, i]) - np.interp(tf[a] - lag, t, cum[:, i])
                       for i in range(3)]).T
        return I, wrap(course[b] - course[a])

    def fit(I, d):
        # d = I . v (v = -scale*up), robust (Huber) least squares
        wts = np.ones(len(d))
        for _ in range(10):
            v = np.linalg.lstsq(I * wts[:, None], d * wts, rcond=None)[0]
            r = d - I @ v
            wts = np.sqrt(np.where(np.abs(r) < 5.0, 1.0, 5.0 / np.maximum(np.abs(r), 1e-9)))
        return v, np.sqrt(np.mean((d - I @ v) ** 2))

    best = None
    for lag in np.arange(-2.0, 2.01, 0.1):
        I, d = windows(3.0, lag)
        keep = np.abs(d) < 120
        v, rms = fit(I[keep], d[keep])
        if best is None or rms < best[0]:
            best = (rms, lag, v)
    rms, lag, v = best
    scale = float(np.linalg.norm(v))
    up = -v / scale
    print("Phone clock vs Jetson clock offset: {:+.1f} s (GPS course lags gyro)".format(lag))
    for W in (3.0, 10.0, 20.0):
        I, d = windows(W, lag)
        keep = np.abs(d) < 150
        e = np.abs(wrap(-(I[keep] @ up) - d[keep]))
        print("  {:2.0f}-s heading change, gyro vs GPS: median error {:.1f} deg, 90% within {:.1f} deg (n={})".format(
            W, np.median(e), np.percentile(e, 90), int(keep.sum())))
    if abs(scale - 1.0) > 0.1:
        print("  NOTE: fitted scale {:.2f} is far from 1; GPS course may differ from heading (sideslip) - "
              "scale not applied.".format(scale))
    cal = Calibration(list(up), [0.0, 0.0, 0.0], 1.0, calibrated=True)
    print("\n" + cal.describe())
    cal.save(args.cal_file, method="offline fit vs RTK GPS course", imu_file=os.path.basename(args.imu_file),
             boat_log=os.path.basename(args.boat_log), fitted_scale=round(scale, 3), clock_offset_s=round(float(lag), 2),
             heading_change_rms_3s_deg=round(float(rms), 2))


# ----------------------------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="iPhone gyro for the multipath boat")
    ap.add_argument("mode", choices=["discover", "test", "log", "calibrate", "calibrate-gps"])
    ap.add_argument("--source", choices=["phyphox", "sensorlog"], default="phyphox")
    ap.add_argument("--url", default="http://172.20.10.1:8080", help="phyphox address")
    ap.add_argument("--port", type=int, default=5555, help="SensorLog UDP port")
    ap.add_argument("--cal-file", default=DEFAULT_CAL_FILE)
    ap.add_argument("--lpf-hz", type=float, default=3.0, help="low-pass cutoff for yaw_rate_filt_dps")
    ap.add_argument("--buffers", default=None, help="phyphox buffer override, e.g. x=gyrX,y=gyrY,z=gyrZ,t=gyr_time")
    ap.add_argument("--no-autostart", action="store_true", help="do not press play in phyphox remotely")
    ap.add_argument("--out", default=None, help="log file (default logs/phone_imu_<time>.csv)")
    ap.add_argument("--imu-file", help="calibrate-gps: phyphox .xlsx export or phone_imu log .csv")
    ap.add_argument("--boat-log", help="calibrate-gps: multipath_log_*.csv recorded at the same time")
    ap.add_argument("--valid-from", type=float, default=0.0, help="calibrate-gps: seconds into the IMU file")
    ap.add_argument("--valid-to", type=float, default=None, help="calibrate-gps: seconds into the IMU file")
    ap.add_argument("--min-speed", type=float, default=0.6, help="calibrate-gps: minimum GPS speed (m/s)")
    args = ap.parse_args()

    if args.mode == "calibrate-gps":
        if not args.imu_file or not args.boat_log:
            ap.error("calibrate-gps needs --imu-file and --boat-log")
        cmd_calibrate_gps(args)
        return

    buffers = dict(kv.split("=", 1) for kv in args.buffers.split(",")) if args.buffers else None
    imu = make_imu(args.source, args.url, args.port, args.cal_file, args.lpf_hz, not args.no_autostart, buffers)
    try:
        imu.start()
    except Exception as exc:
        print("Could not start {} source: {}".format(args.source, exc))
        sys.exit(1)
    try:
        if args.mode == "discover":
            cmd_discover(imu)
        elif args.mode == "test":
            cmd_test(imu)
        elif args.mode == "calibrate":
            cmd_calibrate(imu, args.cal_file)
        else:
            out = args.out or os.path.join(
                "logs", "phone_imu_{}.csv".format(datetime.datetime.now().strftime("%Y%m%d_%H%M%S")))
            cmd_log(imu, out)
    except KeyboardInterrupt:
        print("")
    finally:
        imu.stop()


if __name__ == "__main__":
    main()
