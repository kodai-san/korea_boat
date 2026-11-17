#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
balloon_vision_test.py - bench test for boat.py's balloon (pink target) camera detection.

Runs ONLY the camera, the detector and a web page. It never touches the PCA9685, the RC
Arduino or the GPS, so nothing on the boat can move. Run it on the Jetson (Python 3.6+,
JetPack OpenCV). Stop boat.py first: both programs need the same camera.

    python3 balloon_vision_test.py                    # auto camera: CSI -> gst-launch -> USB
    python3 balloon_vision_test.py --device csi0      # csi0, csi1, /dev/video1, 1, or a video file
    python3 balloon_vision_test.py --port 8091 --no-log

Then open http://<jetson-ip>:8091 in the laptop browser.

The detector (HSV mask -> morphology -> contour filters -> best circle), the angle/distance
maths and the BALLOON-mode decisions are copied from boat.py with the same constants, and
the colour/target settings are read from pid_config.json, so the page shows what BALLOON
mode would see and do. The "would command" PWM values are computed only for display;
nothing is sent anywhere. "Save to pid_config.json" writes only the colour/target keys and
keeps every other key in that file. Detections are logged to
logs/vision_test_<date>_<time>.csv unless --no-log is given.
"""

import argparse
import csv
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

try:
    import cv2
    import numpy as np
except ImportError:
    sys.exit("OpenCV (cv2) and numpy are required (JetPack provides them: python3 -c 'import cv2').")

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PID_CONFIG_FILE = os.path.join(BASE_DIR, "pid_config.json")

# ---- Constants copied from boat.py: keep in sync when boat.py changes ----
CAM_SENSOR_WIDTH = 1280
CAM_SENSOR_HEIGHT = 720
CAM_WIDTH = 640
CAM_HEIGHT = 360
CAM_FPS = 30
CAM_HFOV_DEG = 62.2
CAM_RECONNECT_SEC = 2.0

BALLOON_STEER_GAIN = 18.0
BALLOON_STEER_KD = 3.5
BALLOON_MIN_AREA = 110
BALLOON_CHARGE_ANGLE_DEG = 18.0
BALLOON_MEMORY_SEC = 1.0
BALLOON_POP_FOLLOW_SEC = 1.2
BALLOON_SEARCH_TIMEOUT_SEC = 25.0
BALLOON_SLOW_DIST_M = 8.0
BALLOON_FOCAL_PX = 520.0

CENTER = 4500
max_LEFT = 5500
max_RIGHT = 3000
STOP = 4443
DEFAULT_MAX_GO = 4800
MAX_GO_PRESETS = (4700, 4800, 5000, 5100, 5200)
STEER_SIGN = 1

HSV_PRESETS = {
    "default": {"lower1": (150, 100, 80), "upper1": (179, 255, 255),
                "lower2": (0, 115, 100), "upper2": (4, 255, 255)},
    "bright": {"lower1": (152, 120, 100), "upper1": (179, 255, 255),
               "lower2": (0, 130, 120), "upper2": (3, 255, 255)},
    "cloudy": {"lower1": (146, 80, 65), "upper1": (179, 255, 255),
               "lower2": (0, 95, 80), "upper2": (5, 255, 255)},
}

# Keys this tool may write into pid_config.json (everything else is preserved).
VISION_KEYS = (
    "balloonActionMode", "balloonRealDiamM", "balloonStopDistM", "balloonStopAreaPct",
    "customColorEnable", "targetHue", "targetHueTol", "targetSatMin", "targetValMin",
    "targetColorHex", "camFlip",
)


def clamp(v, lo, hi):
    if lo > hi:
        lo, hi = hi, lo
    return max(lo, min(hi, v))


def lerp(a, b, t):
    t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
    return int(a + (b - a) * t)


def hsv_to_hex(h_cv, s_cv=220, v_cv=240):
    h_deg = (float(h_cv) * 2.0) % 360.0
    s = clamp(float(s_cv) / 255.0, 0.0, 1.0)
    v = clamp(float(v_cv) / 255.0, 0.0, 1.0)
    c = v * s
    x = c * (1.0 - abs(((h_deg / 60.0) % 2.0) - 1.0))
    m = v - c
    if h_deg < 60:
        r1, g1, b1 = c, x, 0.0
    elif h_deg < 120:
        r1, g1, b1 = x, c, 0.0
    elif h_deg < 180:
        r1, g1, b1 = 0.0, c, x
    elif h_deg < 240:
        r1, g1, b1 = 0.0, x, c
    elif h_deg < 300:
        r1, g1, b1 = x, 0.0, c
    else:
        r1, g1, b1 = c, 0.0, x
    r = int(round((r1 + m) * 255))
    g = int(round((g1 + m) * 255))
    b = int(round((b1 + m) * 255))
    return "#{:02x}{:02x}{:02x}".format(clamp(r, 0, 255), clamp(g, 0, 255), clamp(b, 0, 255))


# ---------------------------------------------------------------------------
# Settings (same names, defaults and limits as boat.py / pid_config.json)
# ---------------------------------------------------------------------------
settings_lock = threading.Lock()
settings = {
    "customColorEnable": True,
    "targetHue": 166,
    "targetHueTol": 10,
    "targetSatMin": 105,
    "targetValMin": 80,
    "targetColorHex": "#ff2a85",
    "hsvPreset": "default",
    "balloonActionMode": "APPROACH_STOP",
    "balloonRealDiamM": 3.50,
    "balloonStopDistM": 3.50,
    "balloonStopAreaPct": 30.0,
    "focalPx": BALLOON_FOCAL_PX,
    "flip": False,
    "showMask": False,
    "maxGo": DEFAULT_MAX_GO,
}
config_status = {"loaded": False, "msg": "pid_config.json not found: using boat.py defaults"}


def apply_settings(values):
    """Validate and apply settings with boat.py's limits. Returns the list of applied keys."""
    applied = []
    with settings_lock:
        for key, val in values.items():
            try:
                if key == "balloonActionMode" and val in ("APPROACH_STOP", "RAM_POP"):
                    settings[key] = val
                elif key == "balloonRealDiamM":
                    settings[key] = round(clamp(float(val), 0.2, 8.0), 2)
                elif key == "balloonStopDistM":
                    settings[key] = round(clamp(float(val), 0.8, 15.0), 2)
                elif key == "balloonStopAreaPct":
                    settings[key] = round(clamp(float(val), 5.0, 80.0), 1)
                elif key == "customColorEnable":
                    settings[key] = bool(val)
                elif key == "targetHue":
                    settings[key] = int(clamp(int(float(val)), 0, 179))
                elif key == "targetHueTol":
                    settings[key] = int(clamp(int(float(val)), 3, 40))
                elif key in ("targetSatMin", "targetValMin"):
                    settings[key] = int(clamp(int(float(val)), 25, 245))
                elif key == "targetColorHex" and isinstance(val, str) and val.startswith("#"):
                    settings[key] = val[:7]
                elif key == "hsvPreset" and val in HSV_PRESETS:
                    settings[key] = val
                elif key == "focalPx":
                    settings[key] = round(clamp(float(val), 50.0, 5000.0), 1)
                elif key in ("flip", "showMask"):
                    settings[key] = bool(val)
                elif key == "camFlip":
                    settings["flip"] = int(float(val)) == 180
                elif key == "maxGo":
                    mg = int(float(val))
                    settings[key] = mg if (mg in MAX_GO_PRESETS and mg < 5200) else DEFAULT_MAX_GO
                else:
                    continue
                applied.append(key)
            except (TypeError, ValueError):
                continue
    if any(k in applied for k in ("targetHue", "targetSatMin", "targetValMin")) and "targetColorHex" not in applied:
        with settings_lock:
            settings["targetColorHex"] = hsv_to_hex(
                settings["targetHue"], max(140, settings["targetSatMin"]), max(160, settings["targetValMin"]))
    return applied


def settings_snapshot():
    with settings_lock:
        return dict(settings)


def load_pid_config():
    if not os.path.exists(PID_CONFIG_FILE):
        return
    try:
        with open(PID_CONFIG_FILE, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        apply_settings({k: cfg[k] for k in VISION_KEYS + ("maxGo",) if k in cfg})
        config_status["loaded"] = True
        config_status["msg"] = "Loaded settings from pid_config.json"
    except Exception as e:
        config_status["msg"] = "pid_config.json could not be read ({}): using defaults".format(e)


def save_pid_config():
    """Merge the vision keys into pid_config.json, keeping all other keys."""
    cfg = {}
    if os.path.exists(PID_CONFIG_FILE):
        try:
            with open(PID_CONFIG_FILE, "r", encoding="utf-8") as f:
                cfg = json.load(f)
        except Exception as e:
            return False, "Not saved: existing pid_config.json is unreadable ({})".format(e)
    s = settings_snapshot()
    for key in VISION_KEYS:
        cfg[key] = (180 if s["flip"] else 0) if key == "camFlip" else s[key]
    tmp = PID_CONFIG_FILE + ".tmp"
    try:
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2, ensure_ascii=False)
        os.replace(tmp, PID_CONFIG_FILE)
    except Exception as e:
        return False, "Save failed ({})".format(e)
    return True, "Saved colour/target settings to pid_config.json (boat.py reads it at start-up)"


# ---------------------------------------------------------------------------
# Detection (copied from boat.py: build_target_hsv_mask / detect_pink_balloon)
# ---------------------------------------------------------------------------
def build_target_hsv_mask(hsv_img, s):
    if not s["customColorEnable"]:
        preset = HSV_PRESETS.get(s["hsvPreset"], HSV_PRESETS["default"])
        m1 = cv2.inRange(hsv_img, np.array(preset["lower1"], dtype=np.uint8), np.array(preset["upper1"], dtype=np.uint8))
        m2 = cv2.inRange(hsv_img, np.array(preset["lower2"], dtype=np.uint8), np.array(preset["upper2"], dtype=np.uint8))
        return cv2.bitwise_or(m1, m2)

    h_center = int(clamp(s["targetHue"], 0, 179))
    h_tol = int(clamp(s["targetHueTol"], 3, 40))
    s_min = int(clamp(s["targetSatMin"], 25, 245))
    v_min = int(clamp(s["targetValMin"], 25, 245))
    low_h = h_center - h_tol
    high_h = h_center + h_tol
    if low_h < 0:
        m1 = cv2.inRange(hsv_img, np.array([180 + low_h, s_min, v_min], dtype=np.uint8), np.array([179, 255, 255], dtype=np.uint8))
        m2 = cv2.inRange(hsv_img, np.array([0, s_min, v_min], dtype=np.uint8), np.array([min(179, high_h), 255, 255], dtype=np.uint8))
        return cv2.bitwise_or(m1, m2)
    elif high_h > 179:
        m1 = cv2.inRange(hsv_img, np.array([max(0, low_h), s_min, v_min], dtype=np.uint8), np.array([179, 255, 255], dtype=np.uint8))
        m2 = cv2.inRange(hsv_img, np.array([0, s_min, v_min], dtype=np.uint8), np.array([high_h - 180, 255, 255], dtype=np.uint8))
        return cv2.bitwise_or(m1, m2)
    return cv2.inRange(hsv_img, np.array([low_h, s_min, v_min], dtype=np.uint8), np.array([high_h, 255, 255], dtype=np.uint8))


def detect_pink_balloon(frame, s):
    """Same filters and maths as boat.py. Returns (detected, nx, ny, radius, area_pct, dist_m, angle_err, n_candidates, vis)."""
    h, w = frame.shape[:2]
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = build_target_hsv_mask(hsv, s)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    cnt_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = cnt_info[0] if len(cnt_info) == 2 else cnt_info[1]

    best_cnt = None
    best_score = 0.0
    best_area = 0.0
    best_circle = (0.0, 0.0, 0.0)
    rejected = []  # outlines that passed the area check but failed a shape filter (drawn in grey)

    for cnt in contours:
        raw_area = cv2.contourArea(cnt)
        if raw_area < BALLOON_MIN_AREA:
            continue
        hull = cv2.convexHull(cnt)
        hull_area = max(raw_area, cv2.contourArea(hull))
        x, y, bw, bh = cv2.boundingRect(hull)
        if bh <= 0 or bw <= 0:
            continue
        aspect = float(bw) / float(bh)
        if not (0.28 <= aspect <= 2.8):
            rejected.append(hull)
            continue
        perim = cv2.arcLength(hull, True)
        if perim <= 0:
            continue
        circularity = (4.0 * math.pi * hull_area) / (perim * perim)
        if circularity < 0.24:
            rejected.append(hull)
            continue
        (cx, cy), radius = cv2.minEnclosingCircle(hull)
        if radius < 5.0:
            continue
        circle_area = math.pi * (radius ** 2)
        fill_ratio = hull_area / max(1.0, circle_area)
        if fill_ratio < 0.26:
            rejected.append(hull)
            continue
        score = hull_area * (0.5 + 0.5 * min(1.0, circularity))
        if score > best_score:
            best_score = score
            best_cnt = hull
            best_area = hull_area
            best_circle = (float(cx), float(cy), float(radius))

    if s["showMask"]:
        vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        vis[:, :, 1] = vis[:, :, 1] // 3
    else:
        vis = frame.copy()
        pip_w, pip_h = w // 4, h // 4
        mask_small = cv2.resize(mask, (pip_w, pip_h))
        mask_color = np.zeros((pip_h, pip_w, 3), dtype=np.uint8)
        mask_color[:, :, 0] = mask_small // 2
        mask_color[:, :, 2] = mask_small
        vis[8:8 + pip_h, w - pip_w - 8:w - 8] = mask_color
        cv2.rectangle(vis, (w - pip_w - 8, 8), (w - 8, 8 + pip_h), (147, 20, 255), 1)
        cv2.putText(vis, "HSV MASK", (w - pip_w - 2, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 200, 255), 1)

    if rejected:
        cv2.drawContours(vis, rejected, -1, (140, 140, 140), 1)

    mid_x, mid_y = w // 2, h // 2
    cv2.line(vis, (mid_x, 0), (mid_x, h), (180, 180, 180), 1)
    cv2.line(vis, (0, mid_y), (w, mid_y), (180, 180, 180), 1)
    # +-18 deg "charge" cone, as used by boat.py's CHARGE state
    cone_px = int(round((BALLOON_CHARGE_ANGLE_DEG / (CAM_HFOV_DEG / 2.0)) * (w / 2.0)))
    for xx in (mid_x - cone_px, mid_x + cone_px):
        cv2.line(vis, (xx, h - 40), (xx, h), (0, 200, 120), 1)

    if best_cnt is not None:
        cx, cy, radius = best_circle
        norm_x = (cx - (w / 2.0)) / (w / 2.0)
        norm_y = (cy - (h / 2.0)) / (h / 2.0)
        area_pct = (best_area / float(w * h)) * 100.0
        angle_err = norm_x * (CAM_HFOV_DEG / 2.0)
        est_dist_m = (s["balloonRealDiamM"] * s["focalPx"]) / max(4.0, 2.0 * radius)
        est_dist_m = clamp(est_dist_m, 0.05, 60.0)

        icx, icy, ir = int(round(cx)), int(round(cy)), int(round(radius))
        cv2.circle(vis, (icx, icy), ir, (147, 20, 255), 3)
        cv2.circle(vis, (icx, icy), 4, (0, 255, 255), -1)
        cv2.line(vis, (mid_x, h - 20), (icx, icy), (0, 255, 157), 2)
        label = "TARGET {:.2f}m ({:+.1f}deg)".format(est_dist_m, angle_err)
        cv2.putText(vis, label, (max(10, icx - 90), max(25, icy - ir - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        return True, norm_x, norm_y, radius, area_pct, est_dist_m, angle_err, len(rejected) + 1, vis

    cv2.putText(vis, "SCANNING TARGET (H:{} S>={})...".format(s["targetHue"], s["targetSatMin"]), (14, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 212, 255), 2)
    return False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, len(rejected), vis


# ---------------------------------------------------------------------------
# What BALLOON mode would command (display only: copied from boat.py main loop)
# ---------------------------------------------------------------------------
def balloon_steer(cam_angle_err, d_angle_err=0.0):
    pd_term = (cam_angle_err * BALLOON_STEER_GAIN) + (d_angle_err * BALLOON_STEER_KD)
    raw = CENTER - int(STEER_SIGN * pd_term)
    return clamp(raw, max_LEFT, max_RIGHT)


def balloon_drive(cam_angle_err, area_pct, dist_m, s):
    max_go = s["maxGo"]
    a = abs(cam_angle_err)
    if s["balloonActionMode"] == "RAM_POP":
        if a <= BALLOON_CHARGE_ANGLE_DEG or area_pct >= 2.0:
            return max_go
        t = 0.55 if a >= 32.0 else 0.55 + 0.45 * (32.0 - a) / (32.0 - BALLOON_CHARGE_ANGLE_DEG)
        return lerp(STOP, max_go, t)

    stop_d = s["balloonStopDistM"]
    slow_start_d = max(stop_d + 1.5, BALLOON_SLOW_DIST_M)
    if dist_m <= stop_d or area_pct >= s["balloonStopAreaPct"]:
        return STOP
    elif dist_m < slow_start_d:
        dist_ratio = (dist_m - stop_d) / max(0.5, slow_start_d - stop_d)
        t_dist = 0.35 + 0.65 * clamp(dist_ratio, 0.0, 1.0)
    else:
        t_dist = 1.0
    if a >= 32.0:
        t_ang = 0.55
    elif a <= BALLOON_CHARGE_ANGLE_DEG:
        t_ang = 1.0
    else:
        t_ang = 0.55 + 0.45 * (32.0 - a) / (32.0 - BALLOON_CHARGE_ANGLE_DEG)
    return lerp(STOP, max_go, min(t_dist, t_ang))


class WouldCommand(object):
    """Replays boat.py's BALLOON-mode decisions without commanding anything.
    Unlike boat.py it keeps running after FINISH/TIMEOUT so testing can continue."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.arrive_count = 0
        self.last_seen_t = 0.0
        self.last_angle_err = 0.0
        self.start_t = time.time()

    def update(self, now, det, ang, d_ang, area, dist, s):
        mode = s["balloonActionMode"]
        max_go = s["maxGo"]
        if det:
            self.last_seen_t = now
            self.last_angle_err = ang
            if mode == "APPROACH_STOP" and (dist <= s["balloonStopDistM"] or area >= s["balloonStopAreaPct"]):
                self.arrive_count += 1
            else:
                self.arrive_count = 0
            if mode == "APPROACH_STOP" and self.arrive_count >= 5:
                return "FINISH", CENTER, STOP, "Target reached: boat.py would stop and switch to MANUAL"
            state = "CHARGE" if (abs(ang) <= BALLOON_CHARGE_ANGLE_DEG or area >= 2.0) else "TRACKING"
            return state, balloon_steer(ang, d_ang), balloon_drive(ang, area, dist, s), ""

        self.arrive_count = 0
        since_seen = (now - self.last_seen_t) if self.last_seen_t > 0.0 else 999.0
        if mode == "RAM_POP" and since_seen <= BALLOON_POP_FOLLOW_SEC and abs(self.last_angle_err) <= BALLOON_CHARGE_ANGLE_DEG:
            return "CHARGE", balloon_steer(self.last_angle_err * 0.5, 0.0), max_go, "Target lost: keeps ramming"
        if since_seen <= BALLOON_MEMORY_SEC:
            return ("TRACKING", balloon_steer(self.last_angle_err * 0.8, 0.0), lerp(STOP, max_go, 0.65),
                    "Target lost {:.1f}s: holding last direction".format(since_seen))
        ref_t = self.last_seen_t if self.last_seen_t > 0.0 else self.start_t
        if (now - ref_t) >= BALLOON_SEARCH_TIMEOUT_SEC:
            return "TIMEOUT", CENTER, STOP, "Not seen for {:.0f}s: boat.py would stop and switch to MANUAL".format(BALLOON_SEARCH_TIMEOUT_SEC)
        scan_dir = 1 if self.last_angle_err >= 0 else -1
        steer = clamp(CENTER - int(STEER_SIGN * scan_dir * 220), max_LEFT, max_RIGHT)
        return ("SEARCHING", steer, lerp(STOP, max_go, 0.30),
                "Turning to search, timeout in {:.0f}s".format(BALLOON_SEARCH_TIMEOUT_SEC - (now - ref_t)))


# ---------------------------------------------------------------------------
# Camera opening (same order as boat.py; flip is done in software for every source)
# ---------------------------------------------------------------------------
def build_csi_gstreamer_pipeline(sensor_id=0, sensor_w=CAM_SENSOR_WIDTH, sensor_h=CAM_SENSOR_HEIGHT):
    return (
        "nvarguscamerasrc sensor-id={sid} ! "
        "video/x-raw(memory:NVMM), width=(int){sw}, height=(int){sh}, format=(string)NV12, framerate=(fraction){fps}/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, width=(int){ow}, height=(int){oh}, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=1 max-buffers=1"
    ).format(sid=sensor_id, sw=sensor_w, sh=sensor_h, fps=CAM_FPS, ow=CAM_WIDTH, oh=CAM_HEIGHT)


class GstPipeCapture(object):
    def __init__(self, sensor_id=0):
        self.width, self.height = CAM_WIDTH, CAM_HEIGHT
        self.frame_bytes = self.width * self.height * 3
        cmd = [
            "gst-launch-1.0", "-q", "nvarguscamerasrc", "sensor-id={}".format(sensor_id),
            "!", "video/x-raw(memory:NVMM),width={},height={},format=NV12,framerate={}/1".format(
                CAM_SENSOR_WIDTH, CAM_SENSOR_HEIGHT, CAM_FPS),
            "!", "nvvidconv", "flip-method=0",
            "!", "video/x-raw,width={},height={},format=BGRx".format(self.width, self.height),
            "!", "videoconvert", "!", "video/x-raw,format=BGR", "!", "fdsink", "fd=1",
        ]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                     bufsize=self.frame_bytes * 2)

    def read(self):
        if self.proc is None or self.proc.poll() is not None:
            return False, None
        raw = self.proc.stdout.read(self.frame_bytes)
        if not raw or len(raw) != self.frame_bytes:
            return False, None
        return True, np.frombuffer(raw, dtype=np.uint8).reshape((self.height, self.width, 3)).copy()

    def release(self):
        if self.proc is not None:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=1.0)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass
            self.proc = None


class VideoFileCapture(object):
    """Plays a recorded video in a loop at its own frame rate (for repeatable tests)."""

    def __init__(self, path):
        self.cap = cv2.VideoCapture(path)
        fps = self.cap.get(cv2.CAP_PROP_FPS) if self.cap.isOpened() else 0
        self.dt = 1.0 / fps if fps and 1.0 <= fps <= 120.0 else 1.0 / CAM_FPS
        self.next_t = time.time()

    def read(self):
        delay = self.next_t - time.time()
        if delay > 0:
            time.sleep(delay)
        self.next_t = max(self.next_t, time.time()) + self.dt
        ok, frame = self.cap.read()
        if not ok:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        if ok and frame is not None and (frame.shape[1], frame.shape[0]) != (CAM_WIDTH, CAM_HEIGHT):
            frame = cv2.resize(frame, (CAM_WIDTH, CAM_HEIGHT))
        return ok, frame

    def release(self):
        self.cap.release()


def _first_frame_ok(cap):
    ret, frame = cap.read()
    return ret and frame is not None and frame.size > 0 and len(frame.shape) == 3 and frame.shape[2] == 3


def _open_csi(sid):
    for sw, sh in ((1280, 720), (1640, 1232)):
        try:
            cap = cv2.VideoCapture(build_csi_gstreamer_pipeline(sid, sw, sh), cv2.CAP_GSTREAMER)
            if cap is not None and cap.isOpened() and _first_frame_ok(cap):
                return cap, "CSI camera {} via OpenCV GStreamer ({}x{} -> {}x{})".format(sid, sw, sh, CAM_WIDTH, CAM_HEIGHT)
            if cap is not None:
                cap.release()
        except Exception:
            pass
    if shutil.which("gst-launch-1.0"):
        try:
            gcap = GstPipeCapture(sid)
            if _first_frame_ok(gcap):
                return gcap, "CSI camera {} via gst-launch pipe".format(sid)
            gcap.release()
        except Exception:
            pass
    return None, None


def _open_v4l2(dev):
    for api_pref in (getattr(cv2, "CAP_V4L2", 0), 0):
        try:
            cap = cv2.VideoCapture(dev, api_pref) if api_pref else cv2.VideoCapture(dev)
            if cap is not None and cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
                cap.set(cv2.CAP_PROP_FPS, CAM_FPS)
                if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if _first_frame_ok(cap):
                    return cap, "USB/V4L2 camera {}".format(dev)
            if cap is not None:
                cap.release()
        except Exception:
            pass
    return None, None


def open_camera(device):
    """device: auto | csi0 | csi1 | /dev/videoN | N | path to a video file."""
    if device == "auto":
        for sid in (0, 1):
            cap, desc = _open_csi(sid)
            if cap is not None:
                return cap, desc
        for idx in (0, 1, 2):
            cap, desc = _open_v4l2(idx)
            if cap is not None:
                return cap, desc
        return None, None
    if device in ("csi0", "csi1"):
        return _open_csi(int(device[-1]))
    if device.isdigit():
        return _open_v4l2(int(device))
    if device.startswith("/dev/video"):
        return _open_v4l2(device)
    if os.path.isfile(device):
        cap = VideoFileCapture(device)
        if cap.cap.isOpened():
            return cap, "Video file {}".format(os.path.basename(device))
    return None, None


# ---------------------------------------------------------------------------
# Camera + detection thread
# ---------------------------------------------------------------------------
state_lock = threading.Lock()
state = {
    "cameraOk": False, "cameraDesc": "", "fps": 0.0, "procMs": 0.0,
    "detected": False, "normX": 0.0, "normY": 0.0, "radiusPx": 0.0, "areaPct": 0.0,
    "distM": 0.0, "angleErr": 0.0, "dAngleErr": 0.0, "candidates": 0,
    "detectRate": 0.0, "wouldState": "IDLE", "wouldSteer": CENTER, "wouldDrive": STOP, "wouldNote": "",
    "msg": "",
}
frame_lock = threading.Lock()
latest_bgr = None
latest_jpeg = None
latest_jpeg_seq = 0
last_viewer_t = 0.0
stop_event = threading.Event()
would = WouldCommand()
would_lock = threading.Lock()


class TelemetryLog(object):
    FIELDS = ["time", "detected", "angle_err_deg", "d_angle_err_dps", "dist_m", "area_pct", "radius_px",
              "norm_x", "norm_y", "candidates", "would_state", "would_steer_pwm", "would_drive_pwm",
              "target_hue", "hue_tol", "sat_min", "val_min", "real_diam_m", "focal_px", "fps"]

    def __init__(self, enabled):
        self.f = None
        self.path = None
        if not enabled:
            return
        log_dir = os.path.join(BASE_DIR, "logs")
        try:
            os.makedirs(log_dir, exist_ok=True)
            self.path = os.path.join(log_dir, "vision_test_{}.csv".format(datetime.now().strftime("%Y%m%d_%H%M%S")))
            self.f = open(self.path, "w", newline="", encoding="utf-8")
            self.w = csv.writer(self.f)
            self.w.writerow(self.FIELDS)
            self.last_flush = time.time()
        except Exception as e:
            print("[LOG] could not open log: {}".format(e))
            self.f = None

    def write(self, row):
        if self.f is None:
            return
        self.w.writerow(row)
        now = time.time()
        if now - self.last_flush >= 1.0:
            self.f.flush()
            self.last_flush = now

    def close(self):
        if self.f is not None:
            self.f.close()
            self.f = None


def camera_worker(device, tlog):
    global latest_bgr, latest_jpeg, latest_jpeg_seq
    cap = None
    last_try = 0.0
    fps_cnt, fps_t, cur_fps = 0, time.time(), 0.0
    det_hist = []
    prev_ang, prev_t = 0.0, time.time()
    last_jpeg_t = 0.0

    while not stop_event.is_set():
        now = time.time()
        if cap is None:
            if now - last_try < CAM_RECONNECT_SEC:
                time.sleep(0.1)
                continue
            last_try = now
            cap, desc = open_camera(device)
            with state_lock:
                state["cameraOk"] = cap is not None
                state["cameraDesc"] = desc or ""
                state["msg"] = ("Camera: " + desc) if cap is not None else (
                    "No camera found for '{}'. Is boat.py still running? Retrying...".format(device))
            print("[CAMERA] " + state["msg"], flush=True)
            continue

        ret, frame = cap.read()
        if not ret or frame is None or frame.size == 0:
            print("[CAMERA] Frame read failed: reconnecting", flush=True)
            try:
                cap.release()
            except Exception:
                pass
            cap = None
            with state_lock:
                state["cameraOk"] = False
                state["detected"] = False
                state["msg"] = "Camera frames stopped: reconnecting"
            continue

        s = settings_snapshot()
        if s["flip"]:
            frame = cv2.flip(frame, -1)
        with frame_lock:
            latest_bgr = frame

        t0 = time.time()
        try:
            det, nx, ny, rad, area, dist, ang, ncand, vis = detect_pink_balloon(frame, s)
        except Exception as e:
            print("[DETECT] error: {}".format(e), flush=True)
            time.sleep(0.2)
            continue
        proc_ms = (time.time() - t0) * 1000.0
        now = time.time()

        dt_e = max(0.01, now - prev_t)
        d_ang = (ang - prev_ang) / dt_e if det else 0.0
        prev_ang = ang if det else 0.0
        prev_t = now

        with would_lock:
            w_state, w_steer, w_drive, w_note = would.update(now, det, ang, d_ang, area, dist, s)

        fps_cnt += 1
        if now - fps_t >= 1.0:
            cur_fps = round(fps_cnt / (now - fps_t), 1)
            fps_cnt, fps_t = 0, now
        det_hist.append((now, det))
        while det_hist and now - det_hist[0][0] > 3.0:
            det_hist.pop(0)
        det_rate = 100.0 * sum(1 for _, d in det_hist if d) / max(1, len(det_hist))

        # Encode JPEG at ~15 fps while someone is watching, 1 fps otherwise.
        min_dt = 0.066 if (now - last_viewer_t) < 4.0 else 1.0
        if now - last_jpeg_t >= min_dt:
            ok, buf = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 72])
            if ok:
                with frame_lock:
                    latest_jpeg = buf.tobytes()
                    latest_jpeg_seq += 1
                last_jpeg_t = now

        with state_lock:
            state.update({
                "cameraOk": True, "fps": cur_fps, "procMs": round(proc_ms, 1),
                "detected": det, "normX": round(nx, 3), "normY": round(ny, 3), "radiusPx": round(rad, 1),
                "areaPct": round(area, 2), "distM": round(dist, 2), "angleErr": round(ang, 1),
                "dAngleErr": round(d_ang, 1), "candidates": ncand, "detectRate": round(det_rate, 0),
                "wouldState": w_state, "wouldSteer": int(w_steer), "wouldDrive": int(w_drive), "wouldNote": w_note,
            })

        tlog.write([datetime.now().strftime("%H:%M:%S.%f")[:-3], int(det), round(ang, 2), round(d_ang, 2),
                    round(dist, 3), round(area, 3), round(rad, 1), round(nx, 4), round(ny, 4), ncand,
                    w_state, int(w_steer), int(w_drive), s["targetHue"], s["targetHueTol"], s["targetSatMin"],
                    s["targetValMin"], s["balloonRealDiamM"], s["focalPx"], cur_fps])

    if cap is not None:
        try:
            cap.release()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Commands from the page
# ---------------------------------------------------------------------------
def pick_color(u_norm, v_norm):
    """Same sampling as boat.py's sample_color_from_latest_frame (11x11 patch, circular hue mean)."""
    with frame_lock:
        frame = None if latest_bgr is None else latest_bgr.copy()
    if frame is None:
        return False, "No camera frame yet"
    h, w = frame.shape[:2]
    cx = int(clamp(round(float(u_norm) * w), 0, w - 1))
    cy = int(clamp(round(float(v_norm) * h), 0, h - 1))
    r = 5
    patch = cv2.cvtColor(frame[max(0, cy - r):min(h, cy + r + 1), max(0, cx - r):min(w, cx + r + 1)], cv2.COLOR_BGR2HSV)
    med_s = int(np.median(patch[:, :, 1]))
    med_v = int(np.median(patch[:, :, 2]))
    angles = patch[:, :, 0].astype(np.float32) * (2.0 * math.pi / 180.0)
    mean_ang = math.atan2(float(np.mean(np.sin(angles))), float(np.mean(np.cos(angles))))
    if mean_ang < 0:
        mean_ang += 2.0 * math.pi
    med_h = int(round(mean_ang * 180.0 / (2.0 * math.pi))) % 180
    sat_min = int(clamp(med_s - 45, 55, 225))
    val_min = int(clamp(med_v - 55, 45, 225))
    apply_settings({"customColorEnable": True, "targetHue": med_h, "targetSatMin": sat_min, "targetValMin": val_min,
                    "targetColorHex": hsv_to_hex(med_h, max(140, med_s), max(160, med_v))})
    return True, "Picked H:{} (pixel S:{} V:{}) -> S>={} V>={}".format(med_h, med_s, med_v, sat_min, val_min)


def calibrate_focal(actual_dist_m):
    """focal_px = distance * pixel_diameter / real_diameter, from the current detection."""
    with state_lock:
        det, rad = state["detected"], state["radiusPx"]
    if not det or rad < 5:
        return False, "No target detected: hold the balloon in view first"
    d = float(actual_dist_m)
    if not (0.1 <= d <= 60.0):
        return False, "Distance must be between 0.1 and 60 m"
    s = settings_snapshot()
    focal = d * 2.0 * rad / s["balloonRealDiamM"]
    apply_settings({"focalPx": focal})
    return True, ("Focal length {:.0f} px (boat.py uses {:.0f}). This tool now uses it; to use it on the boat, "
                  "set BALLOON_FOCAL_PX in boat.py".format(focal, BALLOON_FOCAL_PX))


def handle_cmd(cmd, val):
    if cmd == "set" and isinstance(val, dict):
        applied = apply_settings(val)
        return True, "Updated " + ", ".join(applied) if applied else "Nothing changed"
    if cmd == "pick" and isinstance(val, dict):
        return pick_color(val.get("u", 0.5), val.get("v", 0.5))
    if cmd == "calibrate_focal":
        try:
            return calibrate_focal(val)
        except (TypeError, ValueError):
            return False, "Enter a distance in metres"
    if cmd == "reset_would":
        with would_lock:
            would.reset()
        return True, "Restarted the BALLOON-mode simulation (as if just entering BALLOON mode)"
    if cmd == "save":
        return save_pid_config()
    if cmd == "reload":
        load_pid_config()
        return True, config_status["msg"]
    if cmd == "snapshot":
        with frame_lock:
            jpg = latest_jpeg
        if not jpg:
            return False, "No frame yet"
        snap_dir = os.path.join(BASE_DIR, "vision_snapshots")
        os.makedirs(snap_dir, exist_ok=True)
        path = os.path.join(snap_dir, "snap_{}.jpg".format(datetime.now().strftime("%Y%m%d_%H%M%S")))
        with open(path, "wb") as f:
            f.write(jpg)
        return True, "Saved " + os.path.relpath(path, BASE_DIR)
    return False, "Unknown command"


# ---------------------------------------------------------------------------
# Web server
# ---------------------------------------------------------------------------
class ThreadingServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def address_string(self):
        return self.client_address[0]

    def log_message(self, fmt, *args):
        pass

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        global last_viewer_t
        try:
            if self.path == "/" or self.path.startswith("/?"):
                self._send(200, "text/html; charset=utf-8", PAGE_HTML.encode("utf-8"))
            elif self.path.startswith("/api/state"):
                with state_lock:
                    payload = dict(state)
                payload["settings"] = settings_snapshot()
                payload["configMsg"] = config_status["msg"]
                payload["constants"] = {
                    "CENTER": CENTER, "max_LEFT": max_LEFT, "max_RIGHT": max_RIGHT, "STOP": STOP,
                    "hfov": CAM_HFOV_DEG, "chargeDeg": BALLOON_CHARGE_ANGLE_DEG, "boatFocal": BALLOON_FOCAL_PX,
                }
                self._send(200, "application/json", json.dumps(payload).encode("utf-8"))
            elif self.path.startswith("/stream"):
                self.close_connection = True
                self.send_response(200)
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Connection", "close")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                last_seq = -1
                while not stop_event.is_set():
                    last_viewer_t = time.time()
                    with frame_lock:
                        jpg, seq = latest_jpeg, latest_jpeg_seq
                    if jpg and seq != last_seq:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write("Content-Length: {}\r\n\r\n".format(len(jpg)).encode("ascii"))
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                        last_seq = seq
                    time.sleep(0.03)
            elif self.path.startswith("/favicon"):
                self._send(204, "image/x-icon", b"")
            else:
                self._send(404, "text/plain", b"not found")
        except (BrokenPipeError, ConnectionResetError, OSError):
            self.close_connection = True

    def do_POST(self):
        try:
            if not self.path.startswith("/api/cmd"):
                self._send(404, "text/plain", b"not found")
                return
            length = int(self.headers.get("Content-Length", 0))
            data = json.loads(self.rfile.read(length).decode("utf-8") or "{}") if length else {}
            ok, msg = handle_cmd(data.get("cmd"), data.get("value"))
            if msg:
                print("[WEB] " + msg, flush=True)
            self._send(200, "application/json", json.dumps({"ok": ok, "msg": msg}).encode("utf-8"))
        except Exception as e:
            self._send(400, "application/json", json.dumps({"ok": False, "msg": str(e)}).encode("utf-8"))


def local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.5)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


PAGE_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Balloon Vision Test</title>
<style>
:root{--bg:#f4f5f7;--panel:#fff;--ink:#1b1f24;--mute:#5f6b7a;--line:#d9dde3;--accent:#d61f69;--ok:#1a7f4b;--warn:#a15c00;--bad:#c62828;--chip:#eef0f3}
@media (prefers-color-scheme:dark){:root{--bg:#121417;--panel:#1b1e23;--ink:#e8eaed;--mute:#9aa4b1;--line:#2c3138;--accent:#ff4f93;--ok:#4cc38a;--warn:#f0a440;--bad:#ff6b6b;--chip:#252a31}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
header{display:flex;flex-wrap:wrap;gap:8px 16px;align-items:center;padding:10px 16px;border-bottom:1px solid var(--line);background:var(--panel)}
header h1{font-size:16px;margin:0}
.badge{padding:2px 8px;border-radius:99px;font-size:12px;background:var(--chip);color:var(--mute)}
.badge.ok{color:var(--ok)} .badge.bad{color:var(--bad)}
.safe{margin-left:auto;font-size:12px;color:var(--mute)}
main{display:grid;grid-template-columns:minmax(0,1fr) 360px;gap:16px;padding:16px;max-width:1400px;margin:0 auto}
@media (max-width:900px){main{grid-template-columns:1fr}}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:12px 14px;margin-bottom:14px}
.panel h2{font-size:13px;text-transform:uppercase;letter-spacing:.04em;color:var(--mute);margin:0 0 10px}
.video{position:relative;background:#000;border-radius:10px;overflow:hidden;line-height:0}
.video img{width:100%;height:auto;cursor:crosshair;display:block}
.hint{font-size:12px;color:var(--mute);margin-top:6px}
.big{display:flex;align-items:center;gap:10px;font-size:22px;font-weight:600}
.dot{width:14px;height:14px;border-radius:50%;background:var(--line)}
.dot.on{background:var(--ok)}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:6px 14px;margin-top:10px}
.kv span{display:block;font-size:11px;color:var(--mute)} .kv b{font-variant-numeric:tabular-nums;font-size:15px}
.bar{position:relative;height:22px;background:var(--chip);border-radius:6px;margin:8px 0 2px}
.bar .mid{position:absolute;left:50%;top:0;bottom:0;width:1px;background:var(--mute)}
.bar .cone{position:absolute;top:0;bottom:0;background:rgba(26,127,75,.18)}
.bar .mark{position:absolute;top:2px;bottom:2px;width:6px;margin-left:-3px;border-radius:3px;background:var(--accent)}
.scale{display:flex;justify-content:space-between;font-size:11px;color:var(--mute)}
.state{font-weight:700;font-size:18px}
.state.FINISH,.state.TIMEOUT{color:var(--bad)} .state.CHARGE{color:var(--ok)} .state.SEARCHING{color:var(--warn)}
label.row{display:grid;grid-template-columns:96px 1fr 44px;align-items:center;gap:8px;margin:6px 0}
label.row output{text-align:right;font-variant-numeric:tabular-nums}
input[type=range]{width:100%}
input[type=number],select{width:100%;padding:5px 6px;border:1px solid var(--line);border-radius:6px;background:var(--bg);color:var(--ink)}
.btns{display:flex;flex-wrap:wrap;gap:8px;margin-top:8px}
button{padding:7px 12px;border:1px solid var(--line);border-radius:7px;background:var(--chip);color:var(--ink);cursor:pointer;font:inherit}
button.primary{background:var(--accent);border-color:var(--accent);color:#fff}
.swatch{display:inline-block;width:18px;height:18px;border-radius:4px;vertical-align:middle;border:1px solid var(--line)}
#msg{font-size:12px;color:var(--mute);min-height:1.4em;margin-top:8px;word-break:break-word}
.inline{display:flex;gap:8px;align-items:center}
</style>
</head>
<body>
<header>
  <h1>Balloon vision test</h1>
  <span id="cam" class="badge">camera ...</span>
  <span id="fps" class="badge">-- fps</span>
  <span class="safe">Camera only: no motor, RC or GPS connection</span>
</header>
<main>
  <section>
    <div class="video"><img id="view" src="/stream" alt="Camera view with detection overlay"></div>
    <div class="hint">Click the balloon in the image to set the target colour. Grey outlines are pink blobs rejected by the shape filters; green ticks at the bottom mark the &plusmn;18&deg; CHARGE cone.</div>
    <div class="panel" style="margin-top:14px">
      <h2>What BALLOON mode would command (not sent)</h2>
      <div class="inline"><span id="wstate" class="state">IDLE</span><span id="wnote" class="hint" style="margin:0"></span></div>
      <div class="grid">
        <div class="kv"><span>Steer PWM</span><b id="wsteer">-</b></div>
        <div class="kv"><span>Drive PWM (before slew limit)</span><b id="wdrive">-</b></div>
      </div>
      <div class="btns"><button id="resetWould">Restart as if entering BALLOON mode</button></div>
    </div>
  </section>
  <aside>
    <div class="panel">
      <h2>Detection</h2>
      <div class="big"><span id="ddot" class="dot"></span><span id="dtext">No target</span></div>
      <div class="bar"><div id="cone" class="cone"></div><div class="mid"></div><div id="mark" class="mark" hidden></div></div>
      <div class="scale"><span>left</span><span id="ang">0.0&deg;</span><span>right</span></div>
      <div class="grid">
        <div class="kv"><span>Distance (est.)</span><b id="dist">-</b></div>
        <div class="kv"><span>Area of frame</span><b id="area">-</b></div>
        <div class="kv"><span>Radius</span><b id="rad">-</b></div>
        <div class="kv"><span>Angle rate</span><b id="dang">-</b></div>
        <div class="kv"><span>Detected (last 3 s)</span><b id="rate">-</b></div>
        <div class="kv"><span>Processing</span><b id="proc">-</b></div>
      </div>
    </div>
    <div class="panel">
      <h2>Target colour <span id="swatch" class="swatch"></span></h2>
      <label class="row">Mode
        <select id="colorMode" style="grid-column:span 2">
          <option value="custom">Custom (H/S/V below)</option>
          <option value="default">Preset: default</option>
          <option value="bright">Preset: bright sun</option>
          <option value="cloudy">Preset: cloudy</option>
        </select></label>
      <label class="row">Hue <input id="targetHue" type="range" min="0" max="179"><output></output></label>
      <label class="row">Hue &plusmn; <input id="targetHueTol" type="range" min="3" max="40"><output></output></label>
      <label class="row">Sat min <input id="targetSatMin" type="range" min="25" max="245"><output></output></label>
      <label class="row">Value min <input id="targetValMin" type="range" min="25" max="245"><output></output></label>
      <div class="btns">
        <button id="maskBtn">Show mask</button>
        <button id="flipBtn">Flip 180&deg;</button>
        <button id="snapBtn">Save snapshot</button>
      </div>
    </div>
    <div class="panel">
      <h2>Target and stop</h2>
      <label class="row">Action
        <select id="balloonActionMode" style="grid-column:span 2">
          <option value="APPROACH_STOP">APPROACH_STOP</option><option value="RAM_POP">RAM_POP</option>
        </select></label>
      <label class="row">Diameter m <input id="balloonRealDiamM" type="number" min="0.2" max="8" step="0.05" style="grid-column:span 2"></label>
      <label class="row">Stop dist m <input id="balloonStopDistM" type="number" min="0.8" max="15" step="0.1" style="grid-column:span 2"></label>
      <label class="row">Stop area % <input id="balloonStopAreaPct" type="number" min="5" max="80" step="1" style="grid-column:span 2"></label>
      <div class="hint">Distance check: put the balloon at a measured distance, enter it, and press Calibrate. It computes the camera focal length (boat.py assumes <span id="boatFocal">520</span> px; now using <b id="focal">-</b> px).</div>
      <div class="inline" style="margin-top:6px"><input id="trueDist" type="number" min="0.1" max="60" step="0.1" placeholder="Measured distance, m"><button id="calBtn">Calibrate</button></div>
    </div>
    <div class="panel">
      <h2>Settings file</h2>
      <div class="btns">
        <button id="saveBtn" class="primary">Save to pid_config.json</button>
        <button id="reloadBtn">Reload</button>
      </div>
      <div id="msg"></div>
    </div>
  </aside>
</main>
<script>
const $ = id => document.getElementById(id);
let S = null, editing = null, editTimer = null;

async function cmd(name, value) {
  try {
    const r = await fetch('/api/cmd', {method: 'POST', headers: {'Content-Type': 'application/json'},
                                       body: JSON.stringify({cmd: name, value: value})});
    const j = await r.json();
    $('msg').textContent = j.msg || '';
    return j;
  } catch (e) { $('msg').textContent = 'Request failed: ' + e; }
}
function holdEdit(id) { editing = id; clearTimeout(editTimer); editTimer = setTimeout(() => editing = null, 1500); }

['targetHue', 'targetHueTol', 'targetSatMin', 'targetValMin'].forEach(id => {
  const el = $(id);
  el.addEventListener('input', () => { holdEdit(id); el.nextElementSibling.textContent = el.value; });
  el.addEventListener('change', () => { holdEdit(id); cmd('set', {[id]: Number(el.value), customColorEnable: true}); });
});
['balloonRealDiamM', 'balloonStopDistM', 'balloonStopAreaPct'].forEach(id => {
  $(id).addEventListener('focus', () => holdEdit(id));
  $(id).addEventListener('input', () => holdEdit(id));
  $(id).addEventListener('change', () => { holdEdit(id); cmd('set', {[id]: Number($(id).value)}); });
});
$('balloonActionMode').addEventListener('change', e => cmd('set', {balloonActionMode: e.target.value}));
$('colorMode').addEventListener('change', e => {
  const v = e.target.value;
  cmd('set', v === 'custom' ? {customColorEnable: true} : {customColorEnable: false, hsvPreset: v});
});
$('maskBtn').onclick = () => cmd('set', {showMask: !(S && S.settings.showMask)});
$('flipBtn').onclick = () => cmd('set', {flip: !(S && S.settings.flip)});
$('snapBtn').onclick = () => cmd('snapshot');
$('saveBtn').onclick = () => cmd('save');
$('reloadBtn').onclick = () => cmd('reload');
$('resetWould').onclick = () => cmd('reset_would');
$('calBtn').onclick = () => cmd('calibrate_focal', Number($('trueDist').value));
$('view').addEventListener('click', e => {
  const r = e.target.getBoundingClientRect();
  cmd('pick', {u: (e.clientX - r.left) / r.width, v: (e.clientY - r.top) / r.height});
});
$('view').addEventListener('error', () => setTimeout(() => { $('view').src = '/stream?t=' + Date.now(); }, 1500));

function fmt(v, d, unit) { return (v === null || v === undefined) ? '-' : Number(v).toFixed(d) + (unit || ''); }

function render(s) {
  S = s;
  const st = s.settings, c = s.constants;
  $('cam').textContent = s.cameraOk ? s.cameraDesc : (s.msg || 'no camera');
  $('cam').className = 'badge ' + (s.cameraOk ? 'ok' : 'bad');
  $('fps').textContent = fmt(s.fps, 1, ' fps');
  $('ddot').className = 'dot' + (s.detected ? ' on' : '');
  $('dtext').textContent = s.detected ? 'Target detected' : 'No target';
  const half = c.hfov / 2;
  $('cone').style.left = (50 - 50 * c.chargeDeg / half) + '%';
  $('cone').style.width = (100 * c.chargeDeg / half) + '%';
  $('mark').hidden = !s.detected;
  $('mark').style.left = (50 + 50 * Math.max(-1, Math.min(1, s.normX))) + '%';
  $('ang').textContent = s.detected ? (s.angleErr > 0 ? '+' : '') + s.angleErr.toFixed(1) + '°' : '-';
  $('dist').textContent = s.detected ? fmt(s.distM, 2, ' m') : '-';
  $('area').textContent = s.detected ? fmt(s.areaPct, 2, ' %') : '-';
  $('rad').textContent = s.detected ? fmt(s.radiusPx, 0, ' px') : '-';
  $('dang').textContent = s.detected ? fmt(s.dAngleErr, 1, ' °/s') : '-';
  $('rate').textContent = fmt(s.detectRate, 0, ' %');
  $('proc').textContent = fmt(s.procMs, 1, ' ms');

  $('wstate').textContent = s.wouldState; $('wstate').className = 'state ' + s.wouldState;
  $('wnote').textContent = s.wouldNote;
  const ds = s.wouldSteer - c.CENTER;
  $('wsteer').textContent = s.wouldSteer + (ds === 0 ? ' (centre)' : (ds > 0 ? ' (left ' : ' (right ') + Math.abs(ds) + ')');
  $('wdrive').textContent = s.wouldDrive + (s.wouldDrive <= c.STOP ? ' (stop)' : ' (+' + (s.wouldDrive - c.STOP) + ' over stop)');

  ['targetHue', 'targetHueTol', 'targetSatMin', 'targetValMin'].forEach(id => {
    if (editing !== id) { $(id).value = st[id]; $(id).nextElementSibling.textContent = st[id]; }
  });
  ['balloonRealDiamM', 'balloonStopDistM', 'balloonStopAreaPct'].forEach(id => {
    if (editing !== id && document.activeElement !== $(id)) $(id).value = st[id];
  });
  $('balloonActionMode').value = st.balloonActionMode;
  if (document.activeElement !== $('colorMode')) $('colorMode').value = st.customColorEnable ? 'custom' : st.hsvPreset;
  $('swatch').style.background = st.targetColorHex;
  $('maskBtn').textContent = st.showMask ? 'Show camera' : 'Show mask';
  $('flipBtn').textContent = st.flip ? 'Unflip' : 'Flip 180°';
  $('focal').textContent = Math.round(st.focalPx);
  $('boatFocal').textContent = Math.round(c.boatFocal);
  if (!$('msg').textContent) $('msg').textContent = s.configMsg;
}

async function poll() {
  try { const r = await fetch('/api/state', {cache: 'no-store'}); render(await r.json()); }
  catch (e) { $('cam').textContent = 'connection lost'; $('cam').className = 'badge bad'; }
  setTimeout(poll, 200);
}
poll();
</script>
</body>
</html>
"""


def main():
    ap = argparse.ArgumentParser(description="Camera-only test of boat.py's balloon detection (no motors, RC or GPS).")
    ap.add_argument("--device", default="auto", help="auto, csi0, csi1, /dev/videoN, N, or a video file path")
    ap.add_argument("--port", type=int, default=8091, help="web port (boat.py uses 8080)")
    ap.add_argument("--no-log", action="store_true", help="do not write logs/vision_test_*.csv")
    args = ap.parse_args()

    load_pid_config()
    print("[CONFIG] " + config_status["msg"], flush=True)
    tlog = TelemetryLog(not args.no_log)
    if tlog.path:
        print("[LOG] " + os.path.relpath(tlog.path, BASE_DIR), flush=True)

    worker = threading.Thread(target=camera_worker, args=(args.device, tlog), daemon=True)
    worker.start()

    try:
        server = ThreadingServer(("0.0.0.0", args.port), Handler)
    except OSError as e:
        stop_event.set()
        sys.exit("Cannot open port {} ({}). Try --port {}".format(args.port, e, args.port + 1))
    print("[WEB] Open http://{}:{} in a browser (Ctrl+C to quit)".format(local_ip(), args.port), flush=True)
    srv_thread = threading.Thread(target=server.serve_forever, daemon=True)
    srv_thread.start()
    try:
        while worker.is_alive():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopping...", flush=True)
    finally:
        stop_event.set()
        server.shutdown()
        worker.join(timeout=3.0)
        tlog.close()


if __name__ == "__main__":
    main()
