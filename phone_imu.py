#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
phone_imu.py - receive iPhone IMU data streamed by the SensorLog app over UDP (Python 3.6+)

Sources
  --source sensorlog   (default) SensorLog app pushes UDP to this Jetson
  --source phyphox     phyphox app "Gyroscope (rotation rate)" experiment with "Allow remote access";
                       the Jetson polls http://<phone-ip>:8080 (--url). Over a USB cable with the
                       iPhone's Personal Hotspot on, the phone is http://172.20.10.1:8080

Modes
  python3 phone_imu.py discover            # show sender, format and every field name/value received
  python3 phone_imu.py test                # live yaw rate / heading display (check the sign!)
  python3 phone_imu.py log                 # write logs/phone_imu_YYYYmmdd_HHMMSS.csv until Ctrl+C

Common options
  --port 5555          UDP port SensorLog sends to
  --sign -1            multiplier so that a RIGHT (clockwise from above) turn is POSITIVE.
                       Phone lying flat, screen up -> CoreMotion z points up, CCW positive -> use -1.
  --rate-key / --yaw-key / --heading-key   override field names if discover shows different ones

Timestamps in the log are the Jetson's time.time() at reception, the same clock multipath_boat.py
uses for its 'timestamp' column, so the two CSVs can be merged by time afterwards.

Can also be imported:  from phone_imu import PhoneIMU; imu = PhoneIMU(5555); imu.start(); imu.get_latest()
"""

import argparse
import csv
import datetime
import json
import math
import os
import socket
import sys
import threading
import time
import urllib.request

RATE_KEYS = ["motionRotationRateZ", "gyroRotationZ", "RotationRateZ", "rotationRateZ"]
YAW_KEYS = ["motionYaw", "yaw"]
HEADING_KEYS = ["motionHeading", "locationTrueHeading", "locationMagneticHeading", "heading"]
TIME_KEYS = ["motionTimestamp_sinceReboot", "loggingTime", "loggingSample"]


def _to_float(v):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    if math.isnan(f) or math.isinf(f):
        return None
    return f


def _pick(d, preferred, override=None):
    """Return (key, value) for the first matching key (exact, then case-insensitive)."""
    if override:
        return (override, d.get(override)) if override in d else (None, None)
    for k in preferred:
        if k in d:
            return k, d[k]
    lower = {k.lower(): k for k in d}
    for k in preferred:
        if k.lower() in lower:
            real = lower[k.lower()]
            return real, d[real]
    return None, None


class SensorLogParser(object):
    """Turns one UDP datagram into a list of dicts. Handles JSON objects and CSV with a header line."""

    def __init__(self):
        self.csv_header = None
        self.format = None

    def parse(self, payload):
        text = payload.decode("utf-8", errors="ignore").strip()
        if not text:
            return []
        out = []
        if text.startswith("{") or text.startswith("["):
            self.format = "json"
            try:                                     # one (possibly pretty-printed) object or array
                obj = json.loads(text)
                if isinstance(obj, list):
                    return [o for o in obj if isinstance(o, dict)]
                return [obj] if isinstance(obj, dict) else []
            except ValueError:
                pass
            for chunk in text.splitlines():          # several objects, one per line
                chunk = chunk.strip().rstrip(",")
                if not chunk:
                    continue
                try:
                    obj = json.loads(chunk)
                except ValueError:
                    continue
                if isinstance(obj, list):
                    out.extend(o for o in obj if isinstance(o, dict))
                elif isinstance(obj, dict):
                    out.append(obj)
            return out
        self.format = "csv"
        for line in text.splitlines():
            fields = [f.strip() for f in line.split(",")]
            numeric = sum(1 for f in fields if _to_float(f) is not None)
            if numeric < len(fields) / 2.0:          # mostly text -> header line
                self.csv_header = fields
                continue
            if self.csv_header and len(self.csv_header) == len(fields):
                out.append(dict(zip(self.csv_header, fields)))
        return out


class PhoneIMU(object):
    def __init__(self, port=5555, sign=-1.0, rate_key=None, yaw_key=None, heading_key=None,
                 bind_ip="0.0.0.0"):
        self.port = port
        self.sign = float(sign)
        self.keys_override = (rate_key, yaw_key, heading_key)
        self.bind_ip = bind_ip
        self.parser = SensorLogParser()
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.sock = None
        self.sender = None
        self.n_samples = 0
        self.last_rx = 0.0
        self.latest_raw = None
        self.yaw_rate_dps = None     # + = turning right (clockwise seen from above)
        self.yaw_deg = None          # CoreMotion attitude yaw converted to degrees, sign-corrected
        self.heading_deg = None      # 0..360 compass-style heading if the app provides it
        self.used_keys = (None, None, None)
        self.callbacks = []

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind_ip, self.port))
        self.sock.settimeout(0.5)
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            try:
                payload, addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            now = time.time()
            for d in self.parser.parse(payload):
                self._handle(d, addr, now)

    def _handle(self, d, addr, now):
        rk, rv = _pick(d, RATE_KEYS, self.keys_override[0])
        yk, yv = _pick(d, YAW_KEYS, self.keys_override[1])
        hk, hv = _pick(d, HEADING_KEYS, self.keys_override[2])
        rate = _to_float(rv)
        yaw = _to_float(yv)
        hdg = _to_float(hv)
        with self.lock:
            self.sender = addr[0]
            self.n_samples += 1
            self.last_rx = now
            self.latest_raw = d
            self.used_keys = (rk, yk, hk)
            # CoreMotion reports rotation rate in rad/s and attitude in radians
            self.yaw_rate_dps = None if rate is None else self.sign * math.degrees(rate)
            self.yaw_deg = None if yaw is None else self.sign * math.degrees(yaw)
            self.heading_deg = None if (hdg is None or hdg < 0) else hdg % 360.0
            sample = self._snapshot_locked(now)
        for cb in self.callbacks:
            cb(sample, d)

    def _snapshot_locked(self, now):
        age = now - self.last_rx if self.last_rx > 0 else 999.0
        return {
            "valid": age < 0.3 and self.yaw_rate_dps is not None,
            "recv_time": self.last_rx,
            "age_s": age,
            "yaw_rate_dps": self.yaw_rate_dps,
            "yaw_deg": self.yaw_deg,
            "heading_deg": self.heading_deg,
            "sender": self.sender,
            "n_samples": self.n_samples,
        }

    def get_latest(self):
        with self.lock:
            return self._snapshot_locked(time.time())

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if self.sock is not None:
            self.sock.close()


class PhyphoxIMU(PhoneIMU):
    """
    Polls the phyphox app's remote-access web server (http://<phone>:8080) for new gyroscope samples.
    Use the phyphox "Gyroscope (rotation rate)" experiment with "Allow remote access" enabled.
    API: /config lists buffers, /get?buf=<t>|timebuf returns values newer than t, /control?cmd=start.
    """

    Z_CANDIDATES = ["gyrZ", "gyroZ", "gz", "z"]
    T_CANDIDATES = ["gyr_time", "gyrT", "gyro_time", "t", "time"]

    def __init__(self, url, sign=-1.0, z_buf=None, t_buf=None, poll_hz=25.0, autostart=True):
        PhoneIMU.__init__(self, port=0, sign=sign)
        if not url.startswith("http"):
            url = "http://" + url
        self.url = url.rstrip("/")
        self.z_buf = z_buf
        self.t_buf = t_buf
        self.poll_hz = poll_hz
        self.autostart = autostart
        self.buffers = []
        self.measuring = None
        self.last_error = None
        self.parser.format = "phyphox-http"

    def _http_json(self, path, timeout=1.0):
        req = urllib.request.urlopen(self.url + path, timeout=timeout)
        try:
            return json.loads(req.read().decode("utf-8"))
        finally:
            req.close()

    def _choose(self, wanted, candidates, must_contain):
        if wanted:
            return wanted if wanted in self.buffers else None
        for c in candidates:
            if c in self.buffers:
                return c
        for b in self.buffers:
            if must_contain(b.lower()):
                return b
        return None

    def start(self):
        cfg = self._http_json("/config", timeout=3.0)
        self.buffers = [b.get("name") for b in cfg.get("buffers", []) if isinstance(b, dict)]
        self.z_buf = self._choose(self.z_buf, self.Z_CANDIDATES, lambda b: b.endswith("z"))
        self.t_buf = self._choose(self.t_buf, self.T_CANDIDATES, lambda b: "time" in b or b.endswith("_t"))
        if not self.z_buf or not self.t_buf:
            raise RuntimeError("phyphox buffers not found (z={}, time={}). Available: {}. "
                               "Open the 'Gyroscope (rotation rate)' experiment, or pass --z-buffer/--t-buffer."
                               .format(self.z_buf, self.t_buf, self.buffers))
        self.keys_override = (self.z_buf, None, None)
        self.sender = self.url
        if self.autostart:
            self._http_json("/control?cmd=start", timeout=2.0)
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        tb, zb = self.t_buf, self.z_buf
        last_t = None
        period = 1.0 / self.poll_hz
        while self.running:
            t_start = time.time()
            try:
                if last_t is None:   # no value -> only the latest sample
                    q = "/get?{}&{}".format(tb, zb)
                else:                # values newer than last_t
                    q = "/get?{t}={v!r}&{z}={v!r}|{t}".format(t=tb, z=zb, v=last_t)
                data = self._http_json(q, timeout=1.0)
                now = time.time()
                buf = data.get("buffer", {})
                ts = buf.get(tb, {}).get("buffer", []) or []
                zs = buf.get(zb, {}).get("buffer", []) or []
                self.measuring = data.get("status", {}).get("measuring")
                pairs = [(t, z) for t, z in zip(ts, zs)
                         if t is not None and z is not None and (last_t is None or t > last_t)]
                if pairs:
                    t_newest = pairs[-1][0]
                    for t, z in pairs:
                        # phone time -> Jetson time: newest sample ~= now
                        self._handle({tb: t, zb: z}, (self.url, 0), now - (t_newest - t))
                    last_t = t_newest
                self.last_error = None
            except Exception as exc:  # network hiccup, app closed, etc.
                self.last_error = str(exc)
                time.sleep(0.5)
            dt = time.time() - t_start
            if dt < period:
                time.sleep(period - dt)

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2.0)


def local_ips():
    ips = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("172.20.10.1", 9))     # iPhone hotspot gateway; no packet is actually sent
        ips.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for ip in socket.gethostbyname_ex(socket.gethostname())[2]:
            if ip not in ips and not ip.startswith("127."):
                ips.append(ip)
    except OSError:
        pass
    return ips


def wait_for_first(imu, timeout_s=30.0):
    if isinstance(imu, PhyphoxIMU):
        print("Polling phyphox at {} (buffers: time={}, z={}) ...".format(imu.url, imu.t_buf, imu.z_buf))
    else:
        print("Listening on UDP port {} (this Jetson: {}) ...".format(imu.port, ", ".join(local_ips()) or "unknown IP"))
    t0 = time.time()
    while imu.get_latest()["n_samples"] == 0:
        if time.time() - t0 > timeout_s:
            if isinstance(imu, PhyphoxIMU):
                print("\nNo samples in {:.0f} s. measuring={} last_error={}. Press the play button in phyphox "
                      "and keep the app in the foreground.".format(timeout_s, imu.measuring, imu.last_error))
            else:
                print("\nNothing received in {:.0f} s. Check: same network, IP/port in SensorLog, "
                      "streaming switched ON, app in the foreground.".format(timeout_s))
            return False
        time.sleep(0.1)
    return True


def cmd_discover(imu):
    if not wait_for_first(imu):
        return
    time.sleep(2.0)
    s = imu.get_latest()
    with imu.lock:
        raw = dict(imu.latest_raw or {})
        keys = imu.used_keys
    print("\nSender: {}   format: {}   samples in ~2 s: {}".format(s["sender"], imu.parser.format, s["n_samples"]))
    if isinstance(imu, PhyphoxIMU):
        print("phyphox buffers available: {}".format(imu.buffers))
    print("\nAll fields in the latest sample:")
    for k in sorted(raw):
        print("  {:45s} {}".format(k, raw[k]))
    print("\nFields that will be used:")
    print("  rotation rate z : {}".format(keys[0] or "NOT FOUND -> enable CoreMotion / gyro, or pass --rate-key"))
    print("  yaw             : {}".format(keys[1] or "not found (optional)"))
    print("  heading         : {}".format(keys[2] or "not found (optional)"))
    n0 = s["n_samples"]
    time.sleep(3.0)
    print("\nMeasured sample rate: {:.1f} Hz".format((imu.get_latest()["n_samples"] - n0) / 3.0))


def cmd_test(imu):
    if not wait_for_first(imu):
        return
    print("\nRotate the boat (or phone) clockwise seen from above = RIGHT turn. yaw rate must go POSITIVE.")
    print("If it goes negative, re-run with --sign {}. Ctrl+C to stop.\n".format(-imu.sign))
    n_prev, t_prev = imu.get_latest()["n_samples"], time.time()
    hz = 0.0
    while True:
        time.sleep(0.1)
        s = imu.get_latest()
        now = time.time()
        if now - t_prev >= 1.0:
            hz = (s["n_samples"] - n_prev) / (now - t_prev)
            n_prev, t_prev = s["n_samples"], now
        r = s["yaw_rate_dps"]
        bar_n = 0 if r is None else int(max(-20, min(20, r / 5.0)))
        bar = (" " * (20 + min(0, bar_n)) + ("<" * -bar_n if bar_n < 0 else "|") +
               (">" * bar_n if bar_n > 0 else "")).ljust(41)
        sys.stdout.write("\rrate {:>7} deg/s [{}]  yaw {:>7}  heading {:>6}  {:5.1f} Hz  age {:4.0f} ms {}".format(
            "--" if r is None else "{:+.1f}".format(r), bar,
            "--" if s["yaw_deg"] is None else "{:+.1f}".format(s["yaw_deg"]),
            "--" if s["heading_deg"] is None else "{:.1f}".format(s["heading_deg"]),
            hz, s["age_s"] * 1000.0, "" if s["valid"] else "STALE"))
        sys.stdout.flush()


def cmd_log(imu, out_path):
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    f = open(out_path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["timestamp", "yaw_rate_dps", "yaw_deg", "heading_deg", "sender", "raw_json"])
    lock = threading.Lock()
    count = [0]

    def on_sample(sample, raw):
        with lock:
            if f.closed:
                return
            w.writerow([
                "{:.3f}".format(sample["recv_time"]),
                "" if sample["yaw_rate_dps"] is None else "{:.3f}".format(sample["yaw_rate_dps"]),
                "" if sample["yaw_deg"] is None else "{:.2f}".format(sample["yaw_deg"]),
                "" if sample["heading_deg"] is None else "{:.2f}".format(sample["heading_deg"]),
                sample["sender"],
                json.dumps(raw, separators=(",", ":")),
            ])
            count[0] += 1
            if count[0] % 100 == 0:
                f.flush()

    imu.callbacks.append(on_sample)
    print("Logging to {} (Ctrl+C to stop)".format(out_path))
    if not wait_for_first(imu, timeout_s=1e9):
        return
    try:
        while True:
            time.sleep(1.0)
            s = imu.get_latest()
            sys.stdout.write("\r{} samples  last rate {}  {}   ".format(
                count[0], "--" if s["yaw_rate_dps"] is None else "{:+.1f} deg/s".format(s["yaw_rate_dps"]),
                "OK" if s["valid"] else "STALE (phone locked? app in background?)"))
            sys.stdout.flush()
    finally:
        with lock:
            f.close()
        print("\nSaved {} samples to {}".format(count[0], out_path))


def main():
    ap = argparse.ArgumentParser(description="Receive iPhone SensorLog IMU stream over UDP")
    ap.add_argument("mode", choices=["discover", "test", "log"])
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--sign", type=float, default=-1.0)
    ap.add_argument("--rate-key", default=None)
    ap.add_argument("--yaw-key", default=None)
    ap.add_argument("--heading-key", default=None)
    ap.add_argument("--out", default=None, help="log file (default logs/phone_imu_<time>.csv)")
    ap.add_argument("--source", choices=["sensorlog", "phyphox"], default="sensorlog")
    ap.add_argument("--url", default="http://172.20.10.1:8080",
                    help="phyphox remote-access address (USB/iPhone hotspot: http://172.20.10.1:8080)")
    ap.add_argument("--z-buffer", default=None, help="phyphox buffer with rotation rate z (default: auto)")
    ap.add_argument("--t-buffer", default=None, help="phyphox buffer with time (default: auto)")
    ap.add_argument("--no-autostart", action="store_true", help="do not press 'play' in phyphox remotely")
    args = ap.parse_args()

    if args.source == "phyphox":
        imu = PhyphoxIMU(args.url, args.sign, args.z_buffer, args.t_buffer, autostart=not args.no_autostart)
    else:
        imu = PhoneIMU(args.port, args.sign, args.rate_key, args.yaw_key, args.heading_key)
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
