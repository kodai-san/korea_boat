#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
multipath_boat.py
Korea Maritime & Ocean University autonomous boat on Haesucheon:
main controller for multipath navigation (Left/Center/Right).

[Core behavior]
1. MANUAL mode: follow the transmitter's steering and throttle signals (Spektrum SR215 -> Arduino).
2. MANUAL -> AUTO: hold neutral throttle and right steering for at least 2 seconds.
   - On entry, select the nearest path (left / center / right) and a waypoint ahead of the boat.
   - Standard GPS/DGPS permits entry with a warning; RTK Fix/Float is not required (only fix quality 0 blocks entry).
3. AUTO mode:
   - Ignore transmitter steering (CH1); monitor throttle (CH2) for full reverse to return immediately to MANUAL.
   - Stay on the current path when there is no disturbance from wind or waves.
   - If cross-track error (CTE) and directional drift persist, move to an adjacent path with the disturbance.
   - If pushed toward the outer wall on the left/right path, block path changes and strengthen recovery steering.
   - After crossing the finish line at the final waypoint (#65), or entering its radius, stop in neutral and return to MANUAL.
4. National Geographic Information Institute (NGII) RTK corrections (NTRIP):
   a built-in background client sends $GNGGA sentences and injects RTCM3 corrections.
"""

import argparse
import base64
import csv
import datetime
import json
import math
import os
import signal
import socket
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None


# =====================================================================
# [User settings] Adjust these parameters to suit conditions at the site.
# =====================================================================

# 1) National Geographic Information Institute (NGII) network RTK (NTRIP) account settings
NTRIP_CONFIG = {
    "enabled": True,
    "host": "rts2.ngii.go.kr",        # VRS: rts1.ngii.go.kr / FKP: rts2.ngii.go.kr
    "port": 2101,
    "mountpoint": "VRS-RTCM32",       # Team mountpoint (e.g., VRS-RTCM31, FKP-RTCM31)
    "user": "yeseong5226",           # NGII platform user ID
    "password": "ngii",               # Password (default: ngii, or your configured password)
    "gga_interval_sec": 5.0           # Interval for sending the current position ($GNGGA) to the VRS server (s)
}

# 2) Path CSV files (relative to the current directory)
PATH_FILES = {
    "left": "left.csv",
    "center": "center.csv",
    "right": "right.csv"
}

# 3) Multipath navigation and autonomous control parameters
NAV_PARAMS = {
    "lookahead_dist_m": 3.5,          # Default lookahead distance (m), about 2.5 times the 1.44 m waypoint spacing
    "boundary_lookahead_m": 2.4,      # Shorter recovery lookahead when pushed toward a wall on an outer path (m)
    "wp_reach_radius_m": 1.8,         # Waypoint arrival radius (m)
    "finish_radius_m": 1.8,           # Arrival radius at the final waypoint (#65) (m)

    # Disturbance detection and path change thresholds (based on 1.50 m lane spacing)
    "shift_cte_m": 0.70,              # Detect a potential disturbance at 0.70 m or more to either side of the path
    "shift_hold_sec": 1.2,            # Move to an adjacent path after a disturbance persists for 1.2 seconds
    "shift_cooldown_sec": 3.0,        # Settling time after a path change to prevent repeated switching (s)
    "drift_oppose_tol_deg": 5.0,      # Defer a lane change if already heading back toward the path by at least 5 degrees

    # Steering control gains and GPS heading filter
    "max_steer_angle_deg": 35.0,      # Apply full servo deflection (100%) at a 35-degree heading error
    "boundary_gain_boost": 1.45,      # Recovery steering gain multiplier when drifting outside an outer path toward a wall
    "servo_slew_per_cycle": 0.18,     # Maximum steering change per cycle to prevent abrupt servo motion (at 20 Hz)
    "min_heading_move_m": 0.25,       # Minimum movement to update GPS heading, filtering stationary noise (m)
    "sog_cog_blend_mps": 0.45         # Minimum speed at which to trust $GNRMC course over ground (m/s)
}

DEFAULT_CALIBRATION_FILE = "calibration.json"
CONTROL_LOOP_HZ = 20.0                # Main control loop frequency (20 Hz = 50 ms)

# =====================================================================


EARTH_RADIUS_M = 6_378_137.0

FIX_QUALITY_NAMES = {
    0: "No Fix",
    1: "Standard GPS (SPS)",
    2: "DGPS (Differential)",
    4: "RTK FIX (Precise)",
    5: "RTK FLOAT"
}


def wrap_180(angle_deg):
    """Normalize an angle to the range -180 to +180 degrees."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def latlon_to_enu(lat, lon, ref_lat, ref_lon):
    """Convert latitude/longitude to east (x, m) and north (y, m) relative to a reference."""
    dlat = math.radians(lat - ref_lat)
    dlon = math.radians(lon - ref_lon)
    east_m = dlon * EARTH_RADIUS_M * math.cos(math.radians(ref_lat))
    north_m = dlat * EARTH_RADIUS_M
    return east_m, north_m


def bearing_deg_between(lat1, lon1, lat2, lon2):
    """Return the bearing between two coordinates (0-360 degrees clockwise from true north)."""
    dx, dy = latlon_to_enu(lat2, lon2, lat1, lon1)
    return (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0


def distance_m_between(lat1, lon1, lat2, lon2):
    dx, dy = latlon_to_enu(lat2, lon2, lat1, lon1)
    return math.hypot(dx, dy)


def normalize_axis(pulse_us, neutral_us, neg_limit_us, pos_limit_us, deadband_us):
    """Normalize transmitter pulses to -1.0 through +1.0, regardless of channel reversal."""
    diff = pulse_us - neutral_us
    if abs(diff) <= deadband_us:
        return 0.0
    pos_span = pos_limit_us - neutral_us
    neg_span = neg_limit_us - neutral_us
    if pos_span == 0 or neg_span == 0:
        return 0.0
    if (diff > 0 and pos_span > 0) or (diff < 0 and pos_span < 0):
        eff = abs(diff) - deadband_us
        max_eff = max(1, abs(pos_span) - deadband_us)
        return min(1.0, max(0.0, eff / max_eff))
    eff = abs(diff) - deadband_us
    max_eff = max(1, abs(neg_span) - deadband_us)
    return -min(1.0, max(0.0, eff / max_eff))


def map_norm_to_pwm(norm_val, center_us, neg_pwm_us, pos_pwm_us):
    """Convert a normalized value (-1.0 through +1.0) to servo/ESC PWM (us)."""
    norm_val = max(-1.0, min(1.0, norm_val))
    if norm_val >= 0.0:
        return int(round(center_us + norm_val * (pos_pwm_us - center_us)))
    return int(round(center_us + (-norm_val) * (neg_pwm_us - center_us)))


class PCA9685Driver:
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06
    OSC_CLOCK_HZ = 25_000_000.0

    def __init__(self, bus_num=0, address=0x40, freq_hz=50):
        if SMBus is None:
            raise RuntimeError("The smbus2 package is missing ('pip3 install smbus2').")
        self.address = address
        self.bus = SMBus(bus_num)
        self.actual_freq_hz = float(freq_hz)
        self._init_device(freq_hz)

    def _init_device(self, freq_hz):
        self.bus.write_byte_data(self.address, self.MODE1, 0x00)
        time.sleep(0.005)
        prescale = int(round(self.OSC_CLOCK_HZ / (4096.0 * float(freq_hz))) - 1)
        prescale = max(3, min(255, prescale))
        self.actual_freq_hz = self.OSC_CLOCK_HZ / (4096.0 * (prescale + 1))

        old_mode = self.bus.read_byte_data(self.address, self.MODE1)
        self.bus.write_byte_data(self.address, self.MODE1, (old_mode & 0x7F) | 0x10)
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode | 0xA0)

    def set_pwm_us(self, channel, pulse_us):
        period_us = 1_000_000.0 / self.actual_freq_hz
        ticks = int(round((float(pulse_us) / period_us) * 4096.0))
        ticks = max(0, min(4095, ticks))
        reg = self.LED0_ON_L + 4 * int(channel)
        self.bus.write_i2c_block_data(
            self.address, reg, [0x00, 0x00, ticks & 0xFF, (ticks >> 8) & 0x0F]
        )

    def close(self):
        if self.bus is not None:
            self.bus.close()
            self.bus = None


def detect_serial_ports(arduino_pref="auto", gps_pref="auto", force_manual=False):
    """
    Match ports by manufacturer/VID (u-blox 0x1546 versus Arduino) first,
    even if a change in USB connection order swaps ttyACM0 and ttyACM1.
    """
    if force_manual:
        return arduino_pref, gps_pref

    ard_port = None
    gps_port = None

    if list_ports is not None:
        ports = list(list_ports.comports())
        port_map = {}
        for p in ports:
            desc = f"{p.manufacturer or ''} {p.description or ''} {p.hwid or ''}".lower()
            is_ublox = (p.vid == 0x1546) or ("u-blox" in desc) or ("ublox" in desc)
            is_ard = (not is_ublox) and (
                p.vid in (0x2341, 0x2A03, 0x1A86, 0x10C4, 0x0403)
                or "arduino" in desc
                or "ttyacm" in p.device.lower()
                or "ttyusb" in p.device.lower()
            )
            port_map[p.device] = (is_ublox, is_ard)

        # Priority 1: Assign a port positively identified as u-blox to GPS.
        for dev, (is_ublox, _) in port_map.items():
            if is_ublox:
                gps_port = dev
                break

        # Priority 2: Prefer the Arduino port from calibration.json if it is still a non-GPS port.
        if arduino_pref and arduino_pref != "auto" and arduino_pref in port_map:
            is_ublox, _ = port_map[arduino_pref]
            if not is_ublox and arduino_pref != gps_port:
                ard_port = arduino_pref

        # Priority 3: Automatically select one of the remaining Arduino candidates.
        if ard_port is None:
            for dev, (is_ublox, is_ard) in port_map.items():
                if is_ard and dev != gps_port:
                    ard_port = dev
                    break

        # Fall back to the configured GPS port for boards not identified by VID.
        if gps_port is None and gps_pref and gps_pref != "auto" and os.path.exists(gps_pref) and gps_pref != ard_port:
            gps_port = gps_pref

    if ard_port is None and arduino_pref and arduino_pref != "auto" and os.path.exists(arduino_pref):
        ard_port = arduino_pref
    if gps_port is None and gps_pref and gps_pref != "auto" and os.path.exists(gps_pref):
        gps_port = gps_pref

    return ard_port, gps_port


class ArduinoRCReader:
    """Asynchronous receiver thread for Arduino serial data ("CH1,CH2\n")."""

    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
        self.ch1_us = 0
        self.ch2_us = 0
        self.last_rx_time = 0.0

    def start(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        print(f"[RC] Connecting to Arduino ({self.port}, {self.baud} bps)... waiting 2 seconds")
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                parts = raw.decode("ascii", errors="ignore").strip().split(",")
                if len(parts) != 2:
                    continue
                c1, c2 = int(parts[0]), int(parts[1])
                with self.lock:
                    self.ch1_us = c1
                    self.ch2_us = c2
                    self.last_rx_time = time.monotonic()
            except Exception:
                time.sleep(0.02)

    def get_latest(self):
        with self.lock:
            age = time.monotonic() - self.last_rx_time if self.last_rx_time > 0 else 999.0
            if age > 0.5:
                return 0, 0, False
            alive = (800 <= self.ch1_us <= 2200) and (800 <= self.ch2_us <= 2200)
            return self.ch1_us, self.ch2_us, alive

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=0.5)
        if self.ser is not None and self.ser.is_open:
            self.ser.close()


def parse_nmea_latlon(coord_str, hemi):
    """Parse high-precision NMEA ddmm.mmmmmmm (latitude) / dddmm.mmmmmmm (longitude)."""
    if not coord_str or not hemi or "." not in coord_str:
        return None
    dot_idx = coord_str.index(".")
    deg_len = dot_idx - 2
    if deg_len <= 0:
        return None
    deg = float(coord_str[:deg_len])
    minutes = float(coord_str[deg_len:])
    val = deg + (minutes / 60.0)
    if hemi in ("S", "W"):
        val = -val
    return val


def verify_nmea_checksum(sentence):
    """Validate an NMEA 0183 checksum (*XX)."""
    if not sentence.startswith("$") or "*" not in sentence:
        return False
    body, chk_hex = sentence[1:].rsplit("*", 1)
    if len(chk_hex) < 2:
        return False
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(chk_hex[:2], 16)
    except ValueError:
        return False


class ZEDF9PReceiver:
    """Bidirectional ZED-F9P USB communication: receive NMEA ($GxGGA, $GxRMC) and inject RTCM3 corrections."""

    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.write_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.running = False
        self.thread = None

        self.lat = None
        self.lon = None
        self.fix_quality = 0
        self.num_sats = 0
        self.hdop = 99.9
        self.sog_mps = 0.0
        self.cog_deg = None
        self.latest_gga_raw = None
        self.last_fix_time = 0.0

        # Anchor position for estimating boat heading from the movement vector.
        self._anchor_lat = None
        self._anchor_lon = None
        self.heading_deg = None

    def start(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        print(f"[GPS] Connected to ZED-F9P ({self.port}, {self.baud} bps)")

    def write_rtcm(self, data_bytes):
        if not data_bytes or self.ser is None or not self.ser.is_open:
            return
        with self.write_lock:
            try:
                self.ser.write(data_bytes)
            except Exception:
                pass

    def _update_movement_heading(self, new_lat, new_lon, sog_mps, rmc_cog):
        """Combine a minimum movement threshold with RMC course over ground for a stable boat heading."""
        if self._anchor_lat is None:
            self._anchor_lat = new_lat
            self._anchor_lon = new_lon
            return

        moved_m = distance_m_between(self._anchor_lat, self._anchor_lon, new_lat, new_lon)
        if moved_m >= NAV_PARAMS["min_heading_move_m"]:
            vec_heading = bearing_deg_between(self._anchor_lat, self._anchor_lon, new_lat, new_lon)
            self._anchor_lat = new_lat
            self._anchor_lon = new_lon

            if rmc_cog is not None and sog_mps >= NAV_PARAMS["sog_cog_blend_mps"]:
                diff = wrap_180(rmc_cog - vec_heading)
                self.heading_deg = (vec_heading + 0.5 * diff + 360.0) % 360.0
            else:
                self.heading_deg = vec_heading

    def _read_loop(self):
        while self.running:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="ignore").strip()
                if not line.startswith("$") or not verify_nmea_checksum(line):
                    continue

                fields = line.split("*")[0].split(",")
                talker_msg = fields[0]

                if talker_msg.endswith("GGA") and len(fields) >= 10:
                    qual = int(fields[6]) if fields[6].isdigit() else 0
                    sats = int(fields[7]) if fields[7].isdigit() else 0
                    hdop = float(fields[8]) if fields[8] else 99.9
                    lat = parse_nmea_latlon(fields[2], fields[3]) if qual > 0 else None
                    lon = parse_nmea_latlon(fields[4], fields[5]) if qual > 0 else None

                    with self.state_lock:
                        self.fix_quality = qual
                        self.num_sats = sats
                        self.hdop = hdop
                        if qual > 0 and lat is not None and lon is not None:
                            self.lat = lat
                            self.lon = lon
                            self.latest_gga_raw = line
                            self.last_fix_time = time.monotonic()
                            self._update_movement_heading(lat, lon, self.sog_mps, self.cog_deg)

                elif talker_msg.endswith("RMC") and len(fields) >= 9:
                    status = fields[2]
                    if status == "A":
                        knots = float(fields[7]) if fields[7] else 0.0
                        sog = knots * 0.514444
                        cog = float(fields[8]) if fields[8] else None
                        with self.state_lock:
                            self.sog_mps = sog
                            self.cog_deg = cog
            except Exception:
                time.sleep(0.02)

    def reset_heading_anchor(self):
        with self.state_lock:
            self._anchor_lat = self.lat
            self._anchor_lon = self.lon
            self.heading_deg = None

    def get_snapshot(self):
        with self.state_lock:
            age = time.monotonic() - self.last_fix_time if self.last_fix_time > 0 else 999.0
            valid = (self.fix_quality > 0) and (self.lat is not None) and (age <= 2.5)
            return {
                "valid": valid,
                "lat": self.lat,
                "lon": self.lon,
                "fix_quality": self.fix_quality,
                "num_sats": self.num_sats,
                "hdop": self.hdop,
                "sog_mps": self.sog_mps,
                "heading_deg": self.heading_deg,
                "gga_raw": self.latest_gga_raw,
                "age_sec": age
            }

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=0.5)
        if self.ser is not None and self.ser.is_open:
            self.ser.close()


class NGII_NtripClient:
    """Receive RTCM3 corrections from the NGII NTRIP server and inject them into the ZED-F9P."""

    def __init__(self, ntrip_cfg, gps_receiver):
        self.cfg = ntrip_cfg
        self.gps = gps_receiver
        self.running = False
        self.thread = None
        self.connected = False
        self.rtcm_bytes_rx = 0

    def start(self):
        if not self.cfg.get("enabled", True):
            print("[NTRIP] Disabled in the configuration.")
            return
        user = str(self.cfg.get("user", "")).strip()
        if not user or user == "YOUR_NGII_ID":
            print("[NTRIP WARNING] No NGII user ID is set in NTRIP_CONFIG. Continuing with standard GPS/DGPS.")
            return
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        host = self.cfg["host"].strip()
        port = int(self.cfg["port"])
        mp = self.cfg["mountpoint"].strip().lstrip("/")
        auth_raw = f"{self.cfg['user']}:{self.cfg['password']}".encode("utf-8")
        auth_b64 = base64.b64encode(auth_raw).decode("ascii")
        gga_interval = float(self.cfg.get("gga_interval_sec", 5.0))

        while self.running:
            sock = None
            try:
                sock = socket.create_connection((host, port), timeout=8.0)
                req = (
                    f"GET /{mp} HTTP/1.0\r\n"
                    f"Host: {host}:{port}\r\n"
                    f"User-Agent: NTRIP KMOU-Boat/1.0\r\n"
                    f"Accept: */*\r\n"
                    f"Authorization: Basic {auth_b64}\r\n"
                    f"Connection: close\r\n\r\n"
                )
                sock.sendall(req.encode("ascii"))

                header = b""
                while self.running and b"\r\n\r\n" not in header and b"\n\n" not in header:
                    chunk = sock.recv(256)
                    if not chunk:
                        break
                    header += chunk
                    if b"ICY 200 OK" in header or b"200 OK" in header:
                        break

                if b"200 OK" not in header:
                    msg = header.decode("ascii", errors="ignore").strip().splitlines()
                    first_line = msg[0] if msg else "No response"
                    print(f"[NTRIP ERROR] Check server credentials and mountpoint: {first_line}")
                    time.sleep(5.0)
                    continue

                self.connected = True
                print(f"[NTRIP] Connected to NGII ({host}:{port}/{mp})! Starting RTCM3 reception")
                sock.settimeout(2.0)
                last_gga_sent = 0.0

                while self.running:
                    now = time.monotonic()
                    snap = self.gps.get_snapshot()
                    if snap["gga_raw"] and (now - last_gga_sent >= gga_interval):
                        sock.sendall((snap["gga_raw"] + "\r\n").encode("ascii"))
                        last_gga_sent = now

                    try:
                        data = sock.recv(2048)
                        if not data:
                            raise ConnectionError("NTRIP stream closed")
                        self.rtcm_bytes_rx += len(data)
                        self.gps.write_rtcm(data)
                    except socket.timeout:
                        continue

            except Exception as exc:
                if self.connected:
                    print(f"[NTRIP RECONNECT PENDING] Connection lost: {exc}")
                self.connected = False
                time.sleep(5.0)
            finally:
                self.connected = False
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)


def load_waypoints_csv(filepath):
    """Read a CSV (waypoint_number,latitude,longitude) and return [(wp_no, lat, lon), ...]."""
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Path file not found: {filepath}")
    pts = []
    with open(filepath, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        for row in reader:
            wp_no = int(row["waypoint_number"])
            lat = float(row["latitude"])
            lon = float(row["longitude"])
            pts.append((wp_no, lat, lon))
    if len(pts) < 2:
        raise ValueError(f"Path file must contain at least 2 waypoints: {filepath}")
    return pts


class MultipathNavigator:
    """
    Three-lane (left / center / right) multipath navigation engine for Haesucheon.
    - Keep the current path when there is no disturbance.
    - When wind or waves cause drift, switch to an adjacent path in the direction of the drift.
    - When pushed toward a wall on an outer path, strengthen recovery steering to prevent a collision.
    """

    LANE_ORDER = ["left", "center", "right"]

    def __init__(self, paths_dict, params):
        self.paths = paths_dict
        self.params = params
        self.ref_lat = paths_dict["center"][0][1]
        self.ref_lon = paths_dict["center"][0][2]

        # Precompute ENU coordinates for fast vector calculations.
        self.enu_paths = {}
        for name, wps in self.paths.items():
            self.enu_paths[name] = [latlon_to_enu(lat, lon, self.ref_lat, self.ref_lon) for (_, lat, lon) in wps]

        self.active_lane = "center"
        self.target_idx = 1
        self.disturb_dir = 0             # -1: leftward drift, 0: no drift, +1: rightward drift
        self.disturb_start_t = None
        self.last_shift_t = -999.0
        self.prev_steer_norm = 0.0

    def _closest_segment_metrics(self, lane_name, boat_x, boat_y, start_idx=0):
        """
        Find the closest segment (i -> i+1) on the given path and return its index,
        distance (best_dist_m), signed cross-track error (cte_right_m), and bearing.
        - cte_right_m > 0: drift to the right of the path
        - cte_right_m < 0: drift to the left of the path
        """
        pts = self.enu_paths[lane_name]
        best_dist_m = float("inf")
        best_cte_right = 0.0
        best_seg_idx = 0
        best_seg_bearing = 0.0

        search_from = max(0, start_idx - 2)
        for i in range(search_from, len(pts) - 1):
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            vx, vy = bx - ax, by - ay
            seg_len = math.hypot(vx, vy)
            if seg_len < 1e-6:
                continue

            tx, ty = vx / seg_len, vy / seg_len
            px, py = boat_x - ax, boat_y - ay
            proj = max(0.0, min(seg_len, px * tx + py * ty))
            cx, cy = ax + proj * tx, ay + proj * ty
            dist = math.hypot(boat_x - cx, boat_y - cy)

            if dist < best_dist_m:
                best_dist_m = dist
                # Right-hand perpendicular unit vector for the travel vector (tx, ty): n_right = (ty, -tx).
                nx_r, ny_r = ty, -tx
                best_cte_right = (boat_x - cx) * nx_r + (boat_y - cy) * ny_r
                best_seg_idx = i
                best_seg_bearing = (math.degrees(math.atan2(tx, ty)) + 360.0) % 360.0

        return best_seg_idx, best_dist_m, best_cte_right, best_seg_bearing

    def enter_auto_mode(self, boat_lat, boat_lon, now_mono=None):
        """On AUTO entry, select the nearest path and a target waypoint ahead of the boat."""
        if now_mono is None:
            now_mono = time.monotonic()

        bx, by = latlon_to_enu(boat_lat, boat_lon, self.ref_lat, self.ref_lon)

        best_lane = "center"
        best_dist = float("inf")
        best_seg = 0

        for lane in self.LANE_ORDER:
            seg_idx, dist_m, _, _ = self._closest_segment_metrics(lane, bx, by, start_idx=0)
            if dist_m < best_dist:
                best_dist = dist_m
                best_lane = lane
                best_seg = seg_idx

        self.active_lane = best_lane
        pts = self.enu_paths[best_lane]
        idx = min(len(pts) - 1, best_seg + 1)
        while idx < len(pts) - 1:
            if math.hypot(pts[idx][0] - bx, pts[idx][1] - by) <= self.params["wp_reach_radius_m"]:
                idx += 1
            else:
                break

        self.target_idx = idx
        self.disturb_dir = 0
        self.disturb_start_t = None
        self.last_shift_t = now_mono
        self.prev_steer_norm = 0.0
        return self.active_lane, self.paths[self.active_lane][self.target_idx][0], best_dist

    def _check_finish_passed(self, lane_name, boat_x, boat_y):
        """Check arrival within the final waypoint (#65) radius or across its perpendicular finish line (dot product >= 0)."""
        pts = self.enu_paths[lane_name]
        p_prev = pts[-2]
        p_last = pts[-1]

        dist_to_last = math.hypot(boat_x - p_last[0], boat_y - p_last[1])
        if dist_to_last <= self.params["finish_radius_m"]:
            return True

        # In the final 25% of the path, use a dot product to check whether the finish line was crossed.
        if self.target_idx >= int(len(pts) * 0.75):
            ux = p_last[0] - p_prev[0]
            uy = p_last[1] - p_prev[1]
            ulen = math.hypot(ux, uy)
            if ulen > 1e-6:
                ux /= ulen
                uy /= ulen
                along_past_end = (boat_x - p_last[0]) * ux + (boat_y - p_last[1]) * uy
                lateral_dist = abs((boat_x - p_last[0]) * uy - (boat_y - p_last[1]) * ux)
                if along_past_end >= 0.0 and lateral_dist <= 10.0:
                    return True
        return False

    def _select_lookahead_point(self, lane_name, from_idx, boat_x, boat_y, lookahead_m):
        pts = self.enu_paths[lane_name]
        idx = from_idx
        while idx < len(pts) - 1:
            d = math.hypot(pts[idx][0] - boat_x, pts[idx][1] - boat_y)
            if d >= lookahead_m:
                break
            idx += 1
        return idx, pts[idx]

    def update(self, boat_lat, boat_lon, boat_heading_deg, now_mono=None):
        """
        Call once per control cycle.
        Return a dict: finished, active_lane, shifted_to, wp_number, cte_right_m,
        steer_norm (-1 through +1), target_bearing_deg, boundary_guard.
        """
        if now_mono is None:
            now_mono = time.monotonic()

        bx, by = latlon_to_enu(boat_lat, boat_lon, self.ref_lat, self.ref_lon)

        # 1) Calculate the closest segment and CTE on the current path.
        seg_idx, _, cte_right_m, path_bearing_deg = self._closest_segment_metrics(
            self.active_lane, bx, by, start_idx=self.target_idx
        )
        if seg_idx + 1 > self.target_idx:
            self.target_idx = min(len(self.paths[self.active_lane]) - 1, seg_idx + 1)

        # Advance to the next waypoint when within the current target's arrival radius.
        cur_pts = self.enu_paths[self.active_lane]
        while self.target_idx < len(cur_pts) - 1:
            tx_pt, ty_pt = cur_pts[self.target_idx]
            if math.hypot(tx_pt - bx, ty_pt - by) <= self.params["wp_reach_radius_m"]:
                self.target_idx += 1
            else:
                break

        # 2) Check for completion at the final waypoint (#65).
        if self._check_finish_passed(self.active_lane, bx, by):
            return {
                "finished": True,
                "active_lane": self.active_lane,
                "shifted_to": None,
                "wp_number": self.paths[self.active_lane][-1][0],
                "cte_right_m": cte_right_m,
                "steer_norm": 0.0,
                "target_bearing_deg": path_bearing_deg,
                "boundary_guard": False
            }

        # 3) Detect disturbances and decide whether to switch paths.
        lane_idx = self.LANE_ORDER.index(self.active_lane)
        drift_deg = wrap_180(boat_heading_deg - path_bearing_deg) if boat_heading_deg is not None else 0.0
        shifted_to = None
        boundary_guard = False

        # Detect rightward drift (cte_right_m >= +shift_cte_m).
        if cte_right_m >= self.params["shift_cte_m"]:
            if lane_idx < 2:  # A left -> center or center -> right shift is possible.
                not_recovering = (drift_deg >= -self.params["drift_oppose_tol_deg"])
                if not_recovering and (now_mono - self.last_shift_t >= self.params["shift_cooldown_sec"]):
                    if self.disturb_dir != 1:
                        self.disturb_dir = 1
                        self.disturb_start_t = now_mono
                    elif (now_mono - self.disturb_start_t) >= self.params["shift_hold_sec"]:
                        self.active_lane = self.LANE_ORDER[lane_idx + 1]
                        shifted_to = self.active_lane
                else:
                    self.disturb_dir = 0
                    self.disturb_start_t = None
            else:
                # Already on the rightmost path and drifting farther right: protect against the right wall.
                boundary_guard = True
                self.disturb_dir = 0
                self.disturb_start_t = None

        # Detect leftward drift (cte_right_m <= -shift_cte_m).
        elif cte_right_m <= -self.params["shift_cte_m"]:
            if lane_idx > 0:  # A right -> center or center -> left shift is possible.
                not_recovering = (drift_deg <= self.params["drift_oppose_tol_deg"])
                if not_recovering and (now_mono - self.last_shift_t >= self.params["shift_cooldown_sec"]):
                    if self.disturb_dir != -1:
                        self.disturb_dir = -1
                        self.disturb_start_t = now_mono
                    elif (now_mono - self.disturb_start_t) >= self.params["shift_hold_sec"]:
                        self.active_lane = self.LANE_ORDER[lane_idx - 1]
                        shifted_to = self.active_lane
                else:
                    self.disturb_dir = 0
                    self.disturb_start_t = None
            else:
                # Already on the leftmost path and drifting farther left: protect against the left wall.
                boundary_guard = True
                self.disturb_dir = 0
                self.disturb_start_t = None
        else:
            self.disturb_dir = 0
            self.disturb_start_t = None

        # After a path change, immediately recalculate the closest index and CTE for the new path.
        if shifted_to is not None:
            self.last_shift_t = now_mono
            self.disturb_dir = 0
            self.disturb_start_t = None
            new_seg, _, cte_right_m, path_bearing_deg = self._closest_segment_metrics(
                self.active_lane, bx, by, start_idx=max(0, self.target_idx - 2)
            )
            self.target_idx = min(len(self.paths[self.active_lane]) - 1, new_seg + 1)

        # 4) Calculate the lookahead target bearing and servo steering command.
        lh_dist = self.params["boundary_lookahead_m"] if boundary_guard else self.params["lookahead_dist_m"]
        lh_idx, (lx, ly) = self._select_lookahead_point(
            self.active_lane, self.target_idx, bx, by, lh_dist
        )
        target_bearing_deg = (math.degrees(math.atan2(lx - bx, ly - by)) + 360.0) % 360.0

        # Until at least 25 cm of movement provides a GPS heading, keep steering straight (0.0) as the boat accelerates.
        if boat_heading_deg is None:
            raw_steer = 0.0
        else:
            heading_err = wrap_180(target_bearing_deg - boat_heading_deg)
            gain = 1.0 / self.params["max_steer_angle_deg"]
            if boundary_guard:
                gain *= self.params["boundary_gain_boost"]
            raw_steer = max(-1.0, min(1.0, heading_err * gain))

        # Limit the slew rate to smooth abrupt steering changes.
        max_step = self.params["servo_slew_per_cycle"]
        delta = max(-max_step, min(max_step, raw_steer - self.prev_steer_norm))
        steer_norm = max(-1.0, min(1.0, self.prev_steer_norm + delta))
        self.prev_steer_norm = steer_norm

        return {
            "finished": False,
            "active_lane": self.active_lane,
            "shifted_to": shifted_to,
            "wp_number": self.paths[self.active_lane][lh_idx][0],
            "cte_right_m": cte_right_m,
            "steer_norm": steer_norm,
            "target_bearing_deg": target_bearing_deg,
            "boundary_guard": boundary_guard
        }


def load_calibration(path):
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"Calibration file '{path}' not found. Run 'python3 calibrate_tool.py' to complete calibration first."
        )
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def open_telemetry_log():
    """Create a telemetry CSV for QGIS/analysis; allow operation to continue if the filesystem is read-only."""
    try:
        os.makedirs("logs", exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join("logs", f"multipath_log_{stamp}.csv")
        f = open(filepath, "w", newline="", encoding="utf-8")
        writer = csv.writer(f)
        writer.writerow([
            "timestamp", "mode", "lane", "wp_no", "fix_quality",
            "latitude", "longitude", "cte_right_m", "boat_heading_deg",
            "target_bearing_deg", "servo_us", "esc_us",
            "rc_ch1_us", "rc_ch2_us", "rc_steer_norm", "rc_throttle_norm",
            "num_sats", "hdop"
        ])
        print(f"[LOG] Started recording telemetry: {filepath}")
        return f, writer
    except OSError as exc:
        print(f"[LOG WARNING] Cannot write file ({exc}) -> continuing without telemetry logging.")
        return None, None


def main():
    parser = argparse.ArgumentParser(description="Haesucheon multipath autonomous boat controller")
    parser.add_argument("--config", default=DEFAULT_CALIBRATION_FILE, help="Path to calibration.json")
    parser.add_argument("--arduino-port", default=None, help="Manually override the Arduino port")
    parser.add_argument("--gps-port", default=None, help="Manually override the ZED-F9P GPS port")
    parser.add_argument("--throttle-ratio", type=float, default=None, help="Override the AUTO throttle ratio (e.g., 0.60, 0.70)")
    args = parser.parse_args()

    if serial is None or SMBus is None:
        print("[ERROR] Required packages (pyserial, smbus2) are not installed.")
        sys.exit(1)

    cal = load_calibration(args.config)
    hw = cal["hardware"]
    rc_cfg = cal["rc"]
    pwm_cfg = cal["pwm"]
    ms_cfg = cal["mode_switch"]

    if args.throttle_ratio is not None:
        pwm_cfg["auto_throttle_ratio"] = max(0.20, min(1.00, args.throttle_ratio))

    # 1) Load the three Haesucheon path CSV files.
    paths = {}
    for lane_name, csv_path in PATH_FILES.items():
        paths[lane_name] = load_waypoints_csv(csv_path)
    print(
        f"[PATHS] Loaded CSV files (Left: {len(paths['left'])} waypoints, "
        f"Center: {len(paths['center'])} waypoints, Right: {len(paths['right'])} waypoints)"
    )

    navigator = MultipathNavigator(paths, NAV_PARAMS)

    # 2) Initialize the PCA9685 and safely arm the ESC for 2 seconds.
    pca = PCA9685Driver(
        bus_num=hw["i2c_bus"],
        address=hw["pca9685_addr"],
        freq_hz=hw["pca9685_freq_hz"]
    )
    servo_ch = hw["servo_channel"]
    esc_ch = hw["esc_channel"]

    print(f"[PWM] Initializing PCA9685 and arming ESC at neutral ({pwm_cfg['esc_neutral_us']} us) for 2 seconds...")
    pca.set_pwm_us(servo_ch, pwm_cfg["servo_center_us"])
    pca.set_pwm_us(esc_ch, pwm_cfg["esc_neutral_us"])
    time.sleep(2.0)

    # 3) Connect the Arduino (RC) and ZED-F9P (GPS) serial ports.
    ard_pref = args.arduino_port or hw.get("arduino_port", "auto")
    gps_pref = args.gps_port or hw.get("gps_port", "auto")
    force_manual = (args.arduino_port is not None) and (args.gps_port is not None)
    ard_port, gps_port = detect_serial_ports(ard_pref, gps_pref, force_manual=force_manual)

    if not ard_port:
        pca.close()
        raise RuntimeError("Arduino serial port not found. Check the USB connection.")
    if not gps_port:
        pca.close()
        raise RuntimeError("ZED-F9P GPS serial port not found. Check the USB connection.")

    rc = ArduinoRCReader(ard_port, hw.get("arduino_baud", 115200))
    gps = ZEDF9PReceiver(gps_port, hw.get("gps_baud", 115200))
    ntrip = NGII_NtripClient(NTRIP_CONFIG, gps)
    log_file, log_writer = open_telemetry_log()

    running = True

    def shutdown_handler(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    rc.start()
    gps.start()
    ntrip.start()

    mode = "MANUAL"
    right_steer_hold_start = None
    last_no_gps_warn_t = 0.0
    last_non_rtk_warn_t = 0.0

    # Calculate the constant forward cruise PWM for AUTO mode.
    auto_cruise_us = int(round(
        pwm_cfg["esc_neutral_us"]
        + pwm_cfg["auto_throttle_ratio"] * (pwm_cfg["esc_fwd_max_us"] - pwm_cfg["esc_neutral_us"])
    ))

    print("\n=====================================================================")
    print("  Haesucheon multipath autonomous navigation started (initial mode: MANUAL)")
    print(f"  - AUTO throttle  : {int(pwm_cfg['auto_throttle_ratio']*100)}% ({auto_cruise_us} us)")
    print("  - MANUAL -> AUTO : Hold neutral throttle + right steering for 2.0 seconds (then transmitter steering is ignored)")
    print("  - AUTO -> MANUAL : Apply full reverse throttle to return to MANUAL immediately")
    print("  - Safe shutdown  : Ctrl + C (immediate neutral stop)")
    print("=====================================================================\n")

    period_sec = 1.0 / CONTROL_LOOP_HZ

    try:
        while running:
            loop_t0 = time.monotonic()

            ch1_us, ch2_us, rc_alive = rc.get_latest()
            gps_snap = gps.get_snapshot()

            # [Safety] Stop in neutral immediately if the Arduino serial connection or wiring is lost.
            if not rc_alive:
                servo_us = pwm_cfg["servo_center_us"]
                esc_us = pwm_cfg["esc_neutral_us"]
                pca.set_pwm_us(servo_ch, servo_us)
                pca.set_pwm_us(esc_ch, esc_us)
                right_steer_hold_start = None
                if mode == "AUTO":
                    mode = "MANUAL"
                    print("\n[FAILSAFE] RC receiver signal lost -> stopping in neutral and switching to MANUAL")
                sys.stdout.write("\r[WAITING] Checking for an RC receiver signal...                                            ")
                sys.stdout.flush()
                time.sleep(period_sec)
                continue

            st_norm = normalize_axis(
                ch1_us,
                rc_cfg["steer_neutral_us"],
                rc_cfg["steer_left_us"],
                rc_cfg["steer_right_us"],
                rc_cfg["steer_deadband_us"]
            )
            th_norm = normalize_axis(
                ch2_us,
                rc_cfg["throttle_neutral_us"],
                rc_cfg["throttle_rev_us"],
                rc_cfg["throttle_fwd_us"],
                rc_cfg["throttle_deadband_us"]
            )

            nav_info = None

            if mode == "MANUAL":
                servo_us = map_norm_to_pwm(
                    st_norm,
                    pwm_cfg["servo_center_us"],
                    pwm_cfg["servo_left_us"],
                    pwm_cfg["servo_right_us"]
                )
                esc_us = map_norm_to_pwm(
                    th_norm,
                    pwm_cfg["esc_neutral_us"],
                    pwm_cfg["esc_rev_max_us"],
                    pwm_cfg["esc_fwd_max_us"]
                )

                # Detect MANUAL -> AUTO transition (neutral throttle + right steering held for 2 seconds).
                if th_norm == 0.0 and st_norm >= ms_cfg["right_steer_ratio_threshold"]:
                    if right_steer_hold_start is None:
                        right_steer_hold_start = loop_t0
                    elif (loop_t0 - right_steer_hold_start) >= ms_cfg["right_steer_hold_sec"]:
                        right_steer_hold_start = None
                        if not gps_snap["valid"]:
                            if loop_t0 - last_no_gps_warn_t > 2.0:
                                print("\n[AUTO ENTRY DEFERRED] No valid GPS position received yet; remaining in MANUAL mode.")
                                last_no_gps_warn_t = loop_t0
                        else:
                            fq = gps_snap["fix_quality"]
                            fq_str = FIX_QUALITY_NAMES.get(fq, f"Code ({fq})")
                            if fq not in (4, 5):
                                print(f"\n[WARNING] RTK is not active ({fq_str}). Starting AUTO mode with the current position fix!")
                                last_non_rtk_warn_t = loop_t0

                            gps.reset_heading_anchor()
                            lane, wp_no, init_dist = navigator.enter_auto_mode(
                                gps_snap["lat"], gps_snap["lon"], now_mono=loop_t0
                            )
                            mode = "AUTO"
                            servo_us = pwm_cfg["servo_center_us"]
                            esc_us = auto_cruise_us
                            print(
                                f"\n[MODE CHANGE] >>> Entering AUTO mode! Initial nearest path: '{lane.upper()}' "
                                f"(target ahead: WP #{wp_no}, distance to path: {init_dist:.2f}m, fix: {fq_str})"
                            )
                else:
                    right_steer_hold_start = None

            else:
                # ==================== AUTO mode ====================
                # 1) Ignore transmitter steering (st_norm); monitor for full reverse throttle.
                if th_norm <= -ms_cfg["full_reverse_ratio_threshold"]:
                    mode = "MANUAL"
                    right_steer_hold_start = None
                    servo_us = pwm_cfg["servo_center_us"]
                    esc_us = map_norm_to_pwm(
                        th_norm,
                        pwm_cfg["esc_neutral_us"],
                        pwm_cfg["esc_rev_max_us"],
                        pwm_cfg["esc_fwd_max_us"]
                    )
                    print("\n[MANUAL RETURN] <<< Full reverse detected on the transmitter -> switching immediately to MANUAL mode!")

                elif not gps_snap["valid"]:
                    # Stop in neutral without abrupt steering during a temporary GPS outage.
                    servo_us = pwm_cfg["servo_center_us"]
                    esc_us = pwm_cfg["esc_neutral_us"]
                    if loop_t0 - last_no_gps_warn_t > 2.0:
                        print("\n[GPS WARNING] GPS reception delayed -> holding neutral until reception recovers.")
                        last_no_gps_warn_t = loop_t0

                else:
                    # If RTK drops to standard GPS/DGPS, continue navigating and warn every 10 seconds.
                    fq = gps_snap["fix_quality"]
                    if fq not in (4, 5) and (loop_t0 - last_non_rtk_warn_t > 10.0):
                        fq_str = FIX_QUALITY_NAMES.get(fq, f"Code ({fq})")
                        print(f"\n[POSITION WARNING] RTK is not active ({fq_str}) - continuing in AUTO mode.")
                        last_non_rtk_warn_t = loop_t0

                    nav_info = navigator.update(
                        gps_snap["lat"],
                        gps_snap["lon"],
                        gps_snap["heading_deg"],
                        now_mono=loop_t0
                    )

                    if nav_info["shifted_to"] is not None:
                        print(
                            f"\n[PATH CHANGE] Disturbance detected; switching path -> '{nav_info['shifted_to'].upper()}' "
                            f"(cross-track error: {nav_info['cte_right_m']:+.2f}m)"
                        )

                    if nav_info["finished"]:
                        mode = "MANUAL"
                        servo_us = pwm_cfg["servo_center_us"]
                        esc_us = pwm_cfg["esc_neutral_us"]
                        print(
                            f"\n[COURSE COMPLETE] Passed final WP #{nav_info['wp_number']}! "
                            f"Stopping the motor in neutral and returning to MANUAL mode."
                        )
                    else:
                        servo_us = map_norm_to_pwm(
                            nav_info["steer_norm"],
                            pwm_cfg["servo_center_us"],
                            pwm_cfg["servo_left_us"],
                            pwm_cfg["servo_right_us"]
                        )
                        esc_us = auto_cruise_us

            # 4) Write the servo and ESC hardware outputs.
            pca.set_pwm_us(servo_ch, servo_us)
            pca.set_pwm_us(esc_ch, esc_us)

            # 5) Display the live terminal status line and write CSV telemetry.
            fq_short = FIX_QUALITY_NAMES.get(gps_snap["fix_quality"], str(gps_snap["fix_quality"]))
            hdg_txt = f"{gps_snap['heading_deg']:5.1f}" if gps_snap["heading_deg"] is not None else " N/A "
            if mode == "AUTO" and nav_info is not None and not nav_info["finished"]:
                guard_tag = "[WALL GUARD]" if nav_info["boundary_guard"] else ""
                status_line = (
                    f"\r[AUTO:{nav_info['active_lane'].upper():6s}{guard_tag}] "
                    f"WP#{nav_info['wp_number']:02d} | CTE:{nav_info['cte_right_m']:+5.2f}m | "
                    f"Heading:{hdg_txt}deg->Target:{nav_info['target_bearing_deg']:5.1f}deg | "
                    f"GPS:{fq_short} | Servo:{servo_us:4d} ESC:{esc_us:4d}   "
                )
            else:
                status_line = (
                    f"\r[MANUAL] GPS:{fq_short}(Satellites:{gps_snap['num_sats']:02d}) | "
                    f"Steering:{st_norm:+.2f}({servo_us:4d}us) | Throttle:{th_norm:+.2f}({esc_us:4d}us)   "
                )
            sys.stdout.write(status_line)
            sys.stdout.flush()

            if log_writer is not None:
                try:
                    log_writer.writerow([
                        f"{time.time():.3f}",
                        mode,
                        nav_info["active_lane"] if nav_info else "-",
                        nav_info["wp_number"] if nav_info else 0,
                        gps_snap["fix_quality"],
                        f"{gps_snap['lat']:.8f}" if gps_snap["lat"] is not None else "",
                        f"{gps_snap['lon']:.8f}" if gps_snap["lon"] is not None else "",
                        f"{nav_info['cte_right_m']:.3f}" if nav_info else "",
                        f"{gps_snap['heading_deg']:.1f}" if gps_snap["heading_deg"] is not None else "",
                        f"{nav_info['target_bearing_deg']:.1f}" if nav_info else "",
                        servo_us,
                        esc_us,
                        ch1_us,
                        ch2_us,
                        st_norm,
                        th_norm,
                        gps_snap["num_sats"],
                        gps_snap["hdop"]
                    ])
                except OSError:
                    log_writer = None

            elapsed = time.monotonic() - loop_t0
            if elapsed < period_sec:
                time.sleep(period_sec - elapsed)

    finally:
        print("\n\n[SHUTDOWN] Returning servo and ESC to safe neutral and disconnecting devices...")
        try:
            pca.set_pwm_us(servo_ch, pwm_cfg["servo_center_us"])
            pca.set_pwm_us(esc_ch, pwm_cfg["esc_neutral_us"])
            pca.close()
        except Exception:
            pass
        ntrip.stop()
        gps.stop()
        rc.stop()
        if log_file is not None:
            try:
                log_file.close()
            except Exception:
                pass
        print("[SHUTDOWN COMPLETE] All outputs have been safely stopped.")


if __name__ == "__main__":
    main()
