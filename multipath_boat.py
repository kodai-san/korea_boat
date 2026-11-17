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
   - AUTO strategy comes from calibration.json: constant cruise or adaptive full throttle.
   - Stop in neutral and return to MANUAL at the finish, optionally continuing a configured distance beyond it.
4. National Geographic Information Institute (NGII) RTK corrections (NTRIP):
   a built-in background client sends $GNGGA sentences and injects RTCM3 corrections.
5. ArbaLabs Spresense USB output: a separate worker sends the latest GPS sample
   every 5 seconds at 921600 8N1. See SPRESENSE_USB.md for device selection.

[Session diagnostics]
- logs/multipath_log_<session>.csv: telemetry plus GPS validity/age, NTRIP connection
  state/attempts/response/errors, stream bytes received and their age, GGA sends,
  and bytes written to the GPS serial port / write errors. Original columns stay first.
- logs/multipath_events_<session>.log: UTC-timestamped connection attempts, server
  responses, errors, GPS fix/validity transitions, and diagnostic summaries every 10 s
  (also recorded while waiting for RC). Passwords and authorization headers are excluded.
- Counters accumulate across reconnects. Blank rtcm_rx_age_sec means no stream data
  has arrived; a growing age means data stopped, even if ntrip_connected remains 1.
  Received/written byte counts do not prove the receiver accepted the corrections;
  fix_quality 4/5 indicates an RTK solution. Telemetry is flushed once per second.
"""

import argparse
import base64
import csv
import datetime
import json
import logging
import math
import os
import signal
import socket
import sys
import threading
import time

from auto_strategies import (AUTO_STRATEGIES, apply_run_overrides, canonical_auto_strategy_name,
                             create_auto_strategy, resolve_recovery_config)
from gps_heading import MotionHeadingSelector
from path_guidance import StraightLineGuidance, interpolate_lookahead, square_corners
from recovery import StuckRecovery
from spresense_stream import (
    SpresenseGPSStreamer, reserved_spresense_ports, same_serial_port,
)

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
    # Valid NTRIP accounts: lim6246 or yeseong5226.
    "user": "lim6246",               # NGII RTS2 account ID
    "password": "ngii",               # Password (default: ngii, or your configured password)
    "gga_interval_sec": 5.0,          # Interval for sending the current position ($GNGGA) to the VRS server (s)
    # Reconnect when the stream stays open but sends nothing (2026-11-16: stalls up to 122 s while "connected"
    # cost RTK for 31% of a run). Corrections normally arrive every 1-2 s; the F9P holds RTK ~60 s on old data.
    "stall_timeout_sec": 8.0
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
    "finish_margin_m": 0.0,           # Continue this far beyond the finish; positive values require crossing the extended line (m)

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
    "sog_cog_blend_mps": 0.45,        # Minimum speed at which to trust $GNRMC course over ground (m/s)
    # "blend": 50/50 of RMC course and the 0.25 m movement bearing (original).
    # "cog": receiver Doppler course every fix above sog_cog_blend_mps, movement bearing below it.
    "heading_source": "blend"
}

DEFAULT_CALIBRATION_FILE = "calibration.json"
CONTROL_LOOP_HZ = 20.0                # Main control loop frequency (20 Hz = 50 ms)
SESSION_LOG = logging.getLogger("multipath_boat.session")
SESSION_LOG.addHandler(logging.NullHandler())
SESSION_LOG.propagate = False
GPS_NTRIP_FIELDS = (
    "gps_valid", "gps_age_sec", "ntrip_connected", "ntrip_status",
    "ntrip_connection_attempts", "ntrip_last_response", "ntrip_last_error",
    "rtcm_bytes_rx", "rtcm_rx_age_sec", "ntrip_gga_sent",
    "rtcm_bytes_written", "rtcm_write_errors", "rtcm_last_write_error",
    "gps_sog_mps", "gps_cog_deg", "gps_diff_age_sec", "gps_alt_m", "gps_utc"
)


def log_event(message, level=logging.INFO):
    """Keep important diagnostics in both the terminal and the session event log."""
    print(message)
    SESSION_LOG.log(level, message)

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


def detect_serial_ports(arduino_pref="auto", gps_pref="auto", force_manual=False,
                        excluded_ports=()):
    """
    Match ports by manufacturer/VID (u-blox 0x1546 versus Arduino) first,
    even if a change in USB connection order swaps ttyACM0 and ttyACM1.
    """
    def allowed(port):
        return not any(same_serial_port(port, reserved) for reserved in excluded_ports)

    if force_manual:
        if not allowed(arduino_pref) or not allowed(gps_pref):
            raise ValueError("Spresense USB output must use a different port from Arduino and GPS")
        return arduino_pref, gps_pref

    ard_port = None
    gps_port = None

    if list_ports is not None:
        ports = list(list_ports.comports())
        port_map = {}
        for p in ports:
            if not allowed(p.device):
                continue
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
        if gps_port is None and gps_pref and gps_pref != "auto" and allowed(gps_pref) and os.path.exists(gps_pref) and gps_pref != ard_port:
            gps_port = gps_pref

    if ard_port is None and arduino_pref and arduino_pref != "auto" and allowed(arduino_pref) and os.path.exists(arduino_pref):
        ard_port = arduino_pref
    if gps_port is None and gps_pref and gps_pref != "auto" and allowed(gps_pref) and os.path.exists(gps_pref):
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


def parse_float_or_none(text):
    try:
        return float(text) if text else None
    except ValueError:
        return None


def parse_nmea_time_seconds(text):
    """Return seconds since UTC midnight, or None for an invalid NMEA time."""
    try:
        if not isinstance(text, str):
            return None
        parts = text.split(".")
        if (len(parts) > 2 or len(parts[0]) != 6 or not parts[0].isdigit()
                or (len(parts) == 2 and not parts[1].isdigit())):
            return None
        hours, minutes, seconds = int(text[:2]), int(text[2:4]), float(text[4:])
        if hours >= 24 or minutes >= 60 or not math.isfinite(seconds) or not 0 <= seconds < 60:
            return None
        return hours * 3600 + minutes * 60 + seconds
    except (TypeError, ValueError):
        return None


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
        # Validated RMC observations are separate from the legacy blend inputs.
        self.course_deg = None
        self.course_speed_mps = None
        self.course_time_mono = None
        self.course_utc = ""
        self.course_valid = False
        self._course_time_seconds = None
        self._last_course_epoch = None
        self.alt_m = None
        self.diff_age_sec = None          # Age of the RTK/DGPS corrections in use (GGA field 13)
        self.gps_utc = ""                 # GPS UTC time of the latest GGA (hhmmss.ss)
        self.latest_gga_raw = None
        # Export observations separately: navigation retains last known coordinates
        # after a no-fix GGA, which must not be mixed into the new USB sample.
        self.latest_gps_sample = None
        self.last_fix_time = 0.0
        self.last_rx_mono = 0.0           # Last serial line of any kind (for link watchdogs)
        self.rtcm_bytes_written = 0
        self.rtcm_write_errors = 0
        self.last_rtcm_write_error = ""
        self._last_write_warning = None

        # Anchor position for estimating boat heading from the movement vector.
        self._anchor_lat = None
        self._anchor_lon = None
        self.heading_deg = None
        self._position_anchor_lat = None
        self._position_anchor_lon = None
        self._position_anchor_time_mono = None
        self._last_position_gga_seconds = None
        self.position_heading_deg = None
        self.position_heading_time_mono = None
        self.position_speed_mps = None

    def start(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        print(f"[GPS] Connected to ZED-F9P ({self.port}, {self.baud} bps)")

    def write_rtcm(self, data_bytes):
        if not data_bytes:
            return
        with self.write_lock:
            try:
                if self.ser is None or not self.ser.is_open:
                    raise OSError("GPS serial port is not open")
                written = self.ser.write(data_bytes)
                with self.state_lock:
                    self.rtcm_bytes_written += written
                if written != len(data_bytes):
                    raise OSError(f"Incomplete GPS serial write: {written}/{len(data_bytes)} bytes")
            except Exception as exc:
                with self.state_lock:
                    self.rtcm_write_errors += 1
                    self.last_rtcm_write_error = str(exc)
                now = time.monotonic()
                if self._last_write_warning is None or now - self._last_write_warning >= 10.0:
                    log_event(f"[GPS RTCM ERROR] {exc}", logging.WARNING)
                    self._last_write_warning = now

    def _update_movement_heading(self, new_lat, new_lon, sog_mps, rmc_cog):
        """Combine a minimum movement threshold with RMC course over ground for a stable boat heading."""
        if self._anchor_lat is None:
            self._anchor_lat = new_lat
            self._anchor_lon = new_lon
            return

        if (NAV_PARAMS.get("heading_source", "blend") == "cog" and rmc_cog is not None
                and sog_mps >= NAV_PARAMS["sog_cog_blend_mps"]):
            # Doppler course: low noise, no 0.25 m chord lag, updated every fix while moving.
            self.heading_deg = rmc_cog % 360.0
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

    def _clear_position_observation(self):
        self.position_heading_deg = None
        self.position_heading_time_mono = None
        self.position_speed_mps = None
        self._position_anchor_lat = None
        self._position_anchor_lon = None
        self._position_anchor_time_mono = None

    def _update_position_observation(self, lat, lon, utc, quality):
        """Pure displacement fallback from advancing RTK fixes, separate from legacy anchors."""
        seconds = parse_nmea_time_seconds(utc)
        if seconds is None:
            self._clear_position_observation()
            return
        delta = ((seconds - self._last_position_gga_seconds + 43200.0) % 86400.0 - 43200.0
                 if self._last_position_gga_seconds is not None else None)
        advancing = delta is None or delta > 0
        if quality not in (4, 5) or lat is None or lon is None:
            self._clear_position_observation()
            if advancing:
                self._last_position_gga_seconds = seconds
            return
        if not advancing:
            return
        self._last_position_gga_seconds = seconds
        now = time.monotonic()
        if self._position_anchor_lat is None:
            self._position_anchor_lat = lat
            self._position_anchor_lon = lon
            self._position_anchor_time_mono = now
            return
        moved = distance_m_between(self._position_anchor_lat, self._position_anchor_lon, lat, lon)
        if moved >= NAV_PARAMS["min_heading_move_m"]:
            elapsed = now - self._position_anchor_time_mono
            if elapsed > 0 and math.isfinite(moved):
                self.position_heading_deg = bearing_deg_between(
                    self._position_anchor_lat, self._position_anchor_lon, lat, lon)
                self.position_heading_time_mono = now
                self.position_speed_mps = moved / elapsed
            self._position_anchor_lat = lat
            self._position_anchor_lon = lon
            self._position_anchor_time_mono = now

    def _update_course_observation(self, fields):
        """Accept complete, advancing RMC observations. Caller holds state_lock."""
        try:
            seconds = parse_nmea_time_seconds(fields[1])
            date = datetime.datetime.strptime(fields[9], "%d%m%y")
            if seconds is None or len(fields[9]) != 6:
                raise ValueError("invalid RMC timestamp")
            epoch = date + datetime.timedelta(seconds=seconds)
            speed = float(fields[7]) * 0.514444
            # Low-speed receivers may leave COG blank: retain the valid speed
            # observation so the new mode can reject a held movement heading.
            course = float(fields[8]) if fields[8] else None
            if (fields[2] != "A" or (len(fields) > 12 and fields[12] == "N")
                    or not math.isfinite(speed) or speed < 0
                    or (course is not None and (not math.isfinite(course) or not 0 <= course < 360))):
                raise ValueError("invalid RMC motion observation")
        except (IndexError, TypeError, ValueError, OverflowError):
            self.course_deg = None
            self.course_speed_mps = None
            self.course_time_mono = None
            self.course_utc = ""
            self._course_time_seconds = None
            self.course_valid = False
            return

        # Replayed sentences must not turn an old course into a fresh estimate.
        if self._last_course_epoch is not None and epoch <= self._last_course_epoch:
            return
        self._last_course_epoch = epoch
        self.course_deg = course
        self.course_speed_mps = speed
        self.course_time_mono = time.monotonic()
        self.course_utc = epoch.isoformat(timespec="milliseconds") + "Z"
        self._course_time_seconds = seconds
        self.course_valid = course is not None

    def ingest_nmea_line(self, line):
        """Parse one NMEA sentence for the shared threaded GPS receiver.

        Malformed numeric fields raise ValueError; callers decide how to recover.
        """
        if not line.startswith("$") or not verify_nmea_checksum(line):
            return

        fields = line.split("*")[0].split(",")
        talker_msg = fields[0]

        if talker_msg.endswith("GGA") and len(fields) >= 10:
            qual = int(fields[6]) if fields[6].isdigit() else 0
            sats = int(fields[7]) if fields[7].isdigit() else 0
            hdop = float(fields[8]) if fields[8] else 99.9
            lat = parse_nmea_latlon(fields[2], fields[3]) if qual > 0 else None
            lon = parse_nmea_latlon(fields[4], fields[5]) if qual > 0 else None
            # Logging-only fields: never let a malformed value discard the position fix.
            alt = parse_float_or_none(fields[9])
            diff_age = parse_float_or_none(fields[13]) if len(fields) > 13 else None

            with self.state_lock:
                self.latest_gps_sample = {
                    "timestamp": fields[1],  # Original GGA UTC time, never USB send time.
                    "fix_quality": qual,
                    "latitude": lat,
                    "longitude": lon,
                    "num_sats": sats,
                    "hdop": hdop if fields[8] else None,
                }
                self.fix_quality = qual
                self.num_sats = sats
                self.hdop = hdop
                self.alt_m = alt
                self.diff_age_sec = diff_age
                self.gps_utc = fields[1]
                self._update_position_observation(lat, lon, fields[1], qual)
                if qual > 0 and lat is not None and lon is not None:
                    self.lat = lat
                    self.lon = lon
                    self.latest_gga_raw = line
                    self.last_fix_time = time.monotonic()
                    self._update_movement_heading(lat, lon, self.sog_mps, self.cog_deg)

        elif talker_msg.endswith("RMC"):
            with self.state_lock:
                self._update_course_observation(fields)
            if len(fields) >= 9 and fields[2] == "A":
                knots = float(fields[7]) if fields[7] else 0.0
                sog = knots * 0.514444
                cog = float(fields[8]) if fields[8] else None
                with self.state_lock:
                    self.sog_mps = sog
                    self.cog_deg = cog
                    # Apply the Doppler course as soon as it arrives, whatever the GGA/RMC order.
                    if (NAV_PARAMS.get("heading_source", "blend") == "cog" and cog is not None
                            and self._anchor_lat is not None and sog >= NAV_PARAMS["sog_cog_blend_mps"]):
                        self.heading_deg = cog % 360.0

    def _read_loop(self):
        while self.running:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                self.last_rx_mono = time.monotonic()
                self.ingest_nmea_line(raw.decode("ascii", errors="ignore").strip())
            except Exception:
                time.sleep(0.02)

    def reset_heading_anchor(self):
        with self.state_lock:
            self._anchor_lat = self.lat
            self._anchor_lon = self.lon
            self.heading_deg = None
            self._clear_position_observation()
            self._last_position_gga_seconds = parse_nmea_time_seconds(self.gps_utc)
            if self.fix_quality in (4, 5) and self._last_position_gga_seconds is not None:
                self._position_anchor_lat = self.lat
                self._position_anchor_lon = self.lon
                self._position_anchor_time_mono = time.monotonic()

    def get_latest_gps_sample(self):
        """Copy one complete GGA observation; callers do I/O after releasing the lock."""
        with self.state_lock:
            return dict(self.latest_gps_sample) if self.latest_gps_sample is not None else None

    def get_snapshot(self):
        with self.state_lock:
            now = time.monotonic()
            age = now - self.last_fix_time if self.last_fix_time > 0 else 999.0
            valid = (self.fix_quality > 0) and (self.lat is not None) and (age <= 2.5)
            gga_seconds = parse_nmea_time_seconds(self.gps_utc)
            course_skew = None
            if gga_seconds is not None and self._course_time_seconds is not None:
                course_skew = (self._course_time_seconds - gga_seconds + 43200.0) % 86400.0 - 43200.0
            return {
                "valid": valid,
                "lat": self.lat,
                "lon": self.lon,
                "fix_quality": self.fix_quality,
                "num_sats": self.num_sats,
                "hdop": self.hdop,
                "sog_mps": self.sog_mps,
                "cog_deg": self.cog_deg,
                "course_deg": self.course_deg,
                "course_speed_mps": self.course_speed_mps,
                "course_valid": self.course_valid,
                "course_time_mono": self.course_time_mono,
                "course_age_sec": now - self.course_time_mono if self.course_time_mono is not None else None,
                "course_utc": self.course_utc,
                "course_gga_skew_sec": course_skew,
                "alt_m": self.alt_m,
                "diff_age_sec": self.diff_age_sec,
                "gps_utc": self.gps_utc,
                "heading_deg": self.heading_deg,
                "position_heading_deg": self.position_heading_deg,
                "position_heading_time_mono": self.position_heading_time_mono,
                "position_heading_age_sec": (now - self.position_heading_time_mono
                                             if self.position_heading_time_mono is not None else None),
                "position_speed_mps": self.position_speed_mps,
                "gga_raw": self.latest_gga_raw,
                "age_sec": age,
                "rtcm_bytes_written": self.rtcm_bytes_written,
                "rtcm_write_errors": self.rtcm_write_errors,
                "last_rtcm_write_error": self.last_rtcm_write_error
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
        self.state_lock = threading.Lock()
        self.status = "not_started"
        self.connection_attempts = 0
        self.last_response = ""
        self.last_error = ""
        self.last_rtcm_rx_time = None
        self.gga_sent = 0

    def get_snapshot(self):
        with self.state_lock:
            return {
                "connected": self.connected,
                "status": self.status,
                "connection_attempts": self.connection_attempts,
                "last_response": self.last_response,
                "last_error": self.last_error,
                "rtcm_bytes_rx": self.rtcm_bytes_rx,
                "rtcm_rx_age_sec": (time.monotonic() - self.last_rtcm_rx_time
                                    if self.last_rtcm_rx_time is not None else None),
                "gga_sent": self.gga_sent
            }

    def start(self):
        if not self.cfg.get("enabled", True):
            self.status = "disabled"
            log_event("[NTRIP] Disabled in the configuration.")
            return
        user = str(self.cfg.get("user", "")).strip()
        if not user or user == "YOUR_NGII_ID":
            self.status = "missing_account"
            log_event("[NTRIP WARNING] No NGII user ID is set in NTRIP_CONFIG. Continuing with standard GPS/DGPS.", logging.WARNING)
            return
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _read_response(self, sock):
        """Read a status line and preserve correction bytes sharing its TCP packet."""
        response = b""
        while self.running:
            if len(response) > 16384:
                raise ConnectionError("NTRIP response header exceeded 16 KiB")
            if b"\n" in response:
                first, remainder = response.split(b"\n", 1)
                first_line = first.rstrip(b"\r").decode("ascii", errors="replace")
                parts = first_line.split()
                accepted = len(parts) >= 2 and parts[1] == "200" and (
                    parts[0].startswith("HTTP/") or parts[0] == "ICY"
                )
                if not accepted:
                    return first_line, False, b""
                # NTRIP v1 may send only an ICY status line, without HTTP headers.
                if parts[0] == "ICY":
                    return first_line, True, remainder[2:] if remainder.startswith(b"\r\n") else remainder
                for separator in (b"\r\n\r\n", b"\n\n"):
                    if separator in response:
                        return first_line, True, response.split(separator, 1)[1]
            chunk = sock.recv(256)
            if not chunk:
                raise ConnectionError("NTRIP connection closed before a complete response")
            response += chunk
        raise ConnectionError("NTRIP stopped while awaiting a response")

    def _run(self):
        host = self.cfg["host"].strip()
        port = int(self.cfg["port"])
        mp = self.cfg["mountpoint"].strip().lstrip("/")
        auth_raw = f"{self.cfg['user']}:{self.cfg['password']}".encode("utf-8")
        auth_b64 = base64.b64encode(auth_raw).decode("ascii")
        gga_interval = float(self.cfg.get("gga_interval_sec", 5.0))

        while self.running:
            sock = None
            phase = "tcp_connect"
            try:
                with self.state_lock:
                    self.connection_attempts += 1
                    attempt = self.connection_attempts
                    self.status = "connecting"
                    self.last_response = ""
                log_event(f"[NTRIP] Connection attempt {attempt}: {host}:{port}/{mp}")
                sock = socket.create_connection((host, port), timeout=8.0)
                with self.state_lock:
                    self.status = "sending_request"
                log_event("[NTRIP] TCP connected; sending mountpoint request")
                phase = "request_send"
                req = (
                    f"GET /{mp} HTTP/1.0\r\n"
                    f"Host: {host}:{port}\r\n"
                    f"User-Agent: NTRIP KMOU-Boat/1.0\r\n"
                    f"Accept: */*\r\n"
                    f"Authorization: Basic {auth_b64}\r\n"
                    f"Connection: close\r\n\r\n"
                )
                sock.sendall(req.encode("ascii"))
                phase = "response_wait"
                with self.state_lock:
                    self.status = "awaiting_response"

                # Only save the response status line, never the authorization request.
                first_line, accepted, pending_data = self._read_response(sock)
                with self.state_lock:
                    self.last_response = first_line
                if not accepted:
                    with self.state_lock:
                        self.status = "rejected"
                        self.last_error = first_line
                    log_event(f"[NTRIP ERROR] Check server credentials and mountpoint: {first_line}", logging.WARNING)
                    time.sleep(5.0)
                    continue

                with self.state_lock:
                    self.connected = True
                    self.status = "connected"
                    self.last_error = ""
                phase = "correction_stream"
                log_event(f"[NTRIP] Connected to NGII ({host}:{port}/{mp})! Response: {first_line}")
                sock.settimeout(2.0)
                last_gga_sent = 0.0
                first_data = True
                stall_timeout = float(self.cfg.get("stall_timeout_sec", 0.0))
                last_data_time = time.monotonic()

                while self.running:
                    now = time.monotonic()
                    if stall_timeout > 0 and now - last_data_time > stall_timeout:
                        phase = "stall_watchdog"
                        raise ConnectionError(f"no correction data for {now - last_data_time:.1f} s; reconnecting")
                    snap = self.gps.get_snapshot()
                    if snap["gga_raw"] and (now - last_gga_sent >= gga_interval):
                        sock.sendall((snap["gga_raw"] + "\r\n").encode("ascii"))
                        with self.state_lock:
                            self.gga_sent += 1
                        last_gga_sent = now

                    try:
                        if pending_data:
                            data, pending_data = pending_data, b""
                        else:
                            data = sock.recv(2048)
                        if not data:
                            raise ConnectionError("NTRIP stream closed")
                        last_data_time = time.monotonic()
                        with self.state_lock:
                            self.rtcm_bytes_rx += len(data)
                            self.last_rtcm_rx_time = last_data_time
                        if first_data:
                            log_event(f"[NTRIP] First correction-stream data received: {len(data)} bytes")
                            first_data = False
                        self.gps.write_rtcm(data)
                    except socket.timeout:
                        continue

            except Exception as exc:
                with self.state_lock:
                    self.connected = False
                    self.status = "error"
                    self.last_error = f"{phase}: {exc}"
                log_event(f"[NTRIP RECONNECT PENDING] phase={phase}, {type(exc).__name__}: {exc}", logging.WARNING)
                time.sleep(1.0 if phase == "stall_watchdog" else 5.0)
            finally:
                with self.state_lock:
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
        with self.state_lock:
            if self.status not in ("disabled", "missing_account"):
                self.status = "stopping" if self.thread and self.thread.is_alive() else "stopped"
        SESSION_LOG.info("[NTRIP STOP] %s", self.get_snapshot())


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


def validate_finish_margin(value):
    """Require an explicit finite, nonnegative distance in meters."""
    try:
        finite = math.isfinite(value)
    except (TypeError, OverflowError):
        finite = False
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not finite or value < 0):
        raise ValueError("navigation.finish_margin_m must be a finite number >= 0")
    return float(value)


def resolve_navigation_params(navigation_config, finish_margin_override=None):
    """Apply persisted navigation settings and an optional one-run override."""
    if not isinstance(navigation_config, dict):
        raise ValueError("navigation must be a JSON object")
    unknown = navigation_config.keys() - {"finish_margin_m"}
    if unknown:
        raise ValueError(f"Unknown navigation option(s): {', '.join(sorted(unknown))}")
    params = dict(NAV_PARAMS)
    params["finish_margin_m"] = validate_finish_margin(
        navigation_config.get("finish_margin_m", params["finish_margin_m"]))
    if finish_margin_override is not None:
        params["finish_margin_m"] = validate_finish_margin(finish_margin_override)
    return params


class MultipathNavigator:
    """
    Three-lane (left / center / right) multipath navigation engine for Haesucheon.
    - Keep the current path when there is no disturbance.
    - When wind or waves cause drift, switch to an adjacent path in the direction of the drift.
    - When pushed toward a wall on an outer path, strengthen recovery steering to prevent a collision.
    """

    LANE_ORDER = ["left", "center", "right"]

    def __init__(self, paths_dict, params, strategy=None):
        self.paths = paths_dict
        self.params = dict(params)
        self.finish_margin_m = validate_finish_margin(params.get("finish_margin_m", 0.0))
        self.params["finish_margin_m"] = self.finish_margin_m
        self.strategy = strategy if strategy is not None else create_auto_strategy({}, params)
        self.ref_lat = paths_dict["center"][0][1]
        self.ref_lon = paths_dict["center"][0][2]

        # Precompute ENU coordinates for fast vector calculations.
        self.enu_paths = {}
        self.finish_directions = {}
        for name, wps in self.paths.items():
            pts = [latlon_to_enu(lat, lon, self.ref_lat, self.ref_lon) for (_, lat, lon) in wps]
            if self.finish_margin_m > 0:
                # Extend the final straight segment for guidance and finish detection.
                # Keep the CSV coordinates and waypoint numbers unchanged.
                end_x, end_y = pts[-1]
                for prev_x, prev_y in reversed(pts[:-1]):
                    dx, dy = end_x - prev_x, end_y - prev_y
                    length = math.hypot(dx, dy)
                    if length > 1e-6:
                        ux, uy = dx / length, dy / length
                        self.finish_directions[name] = (ux, uy)
                        pts[-1] = (end_x + self.finish_margin_m * ux,
                                   end_y + self.finish_margin_m * uy)
                        break
                else:
                    raise ValueError(f"Cannot extend finish for '{name}': path has no nonzero segment")
            self.enu_paths[name] = pts

        # Straight-line strategies follow one line drawn at AUTO entry instead of the three lanes.
        # The course frame comes from the unextended centerline CSV; the banks are parallel to it.
        self.line_guidance = None
        if getattr(self.strategy, "uses_straight_line", False) is True:
            center = [latlon_to_enu(lat, lon, self.ref_lat, self.ref_lon) for (_, lat, lon) in paths_dict["center"]]
            self.line_guidance = StraightLineGuidance(
                center[0], center[-1], self.strategy.options["band_half_width_m"],
                self.strategy.options["line_max_offset_m"])

        self.active_lane = "center"
        # False keeps the boat on its lane (boat.py "fixed path"); wall guards still apply.
        self.allow_lane_shift = True
        self.target_idx = 1
        self.guidance_segment_idx = 0    # Continuous projection progress, independent of waypoint arrival radius.
        self.disturb_dir = 0             # -1: leftward drift, 0: no drift, +1: rightward drift
        self.disturb_start_t = None
        self.last_shift_t = -999.0

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

    def enter_auto_mode(self, boat_lat, boat_lon, now_mono=None, lane=None):
        """On AUTO entry, select the nearest path (or the given lane) and a target waypoint ahead of the boat."""
        if now_mono is None:
            now_mono = time.monotonic()
        if lane is not None and lane not in self.LANE_ORDER:
            raise ValueError(f"Unknown lane {lane!r}")

        bx, by = latlon_to_enu(boat_lat, boat_lon, self.ref_lat, self.ref_lon)

        if self.line_guidance is not None:
            line = self.line_guidance
            line.draw_line(bx, by)
            self.active_lane = "line"
            self.strategy.reset()
            opts = self.strategy.options
            if opts["test_square"]:
                s0 = line.frame(bx, by)[0] + opts["test_lead_m"]
                self.test_targets = [line.point(s, d) + (label,) for s, d, label in square_corners(
                    s0, (opts["band_half_width_m"], opts["line_max_offset_m"]), opts["test_leg_length_m"])]
                self.test_leg = 0
                self.test_leg_start = (bx, by)
                self.active_lane = "test"
            return self.active_lane, self._line_wp_number(line.frame(bx, by)[0]), abs(
                line.entry_offset_m - line.line_offset_m)

        best_lane = "center"
        best_dist = float("inf")
        best_seg = 0

        for lane in ([lane] if lane is not None else self.LANE_ORDER):
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
        self.guidance_segment_idx = best_seg
        self.disturb_dir = 0
        self.disturb_start_t = None
        self.last_shift_t = now_mono
        self.strategy.reset()
        return self.active_lane, self.paths[self.active_lane][self.target_idx][0], best_dist

    def _check_finish_passed(self, lane_name, boat_x, boat_y):
        """Check the extended finish line, or legacy radius arrival when margin is zero."""
        pts = self.enu_paths[lane_name]
        p_prev = pts[-2]
        p_last = pts[-1]

        dist_to_last = math.hypot(boat_x - p_last[0], boat_y - p_last[1])
        # A radius would consume small margins and could stop before the original finish.
        if self.finish_margin_m == 0 and dist_to_last <= self.params["finish_radius_m"]:
            return True

        # In the final 25% of the path, use a dot product to check whether the finish line was crossed.
        if self.target_idx >= int(len(pts) * 0.75):
            if self.finish_margin_m > 0:
                # Retain the original direction even for tiny extensions of a repeated endpoint.
                ux, uy = self.finish_directions[lane_name]
            else:
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

    def _fresh_selected_motion(self, motion, boat_heading_deg, now_mono):
        """Keep heading/speed paired and recheck age before prediction or speed guidance."""
        if not isinstance(motion, dict):
            return None
        if motion.get("source") not in ("gps_course", "position"):
            return None
        age_limit = {
            "gps_course": self.strategy.options["course_max_age_sec"],
            "position": self.strategy.options["position_heading_max_age_sec"],
        }.get(motion.get("source"))
        values = [motion.get(key) for key in ("speed_mps", "heading_deg", "age_sec", "observed_at_mono")]
        try:
            finite = all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                         for v in values + [boat_heading_deg])
        except OverflowError:
            finite = False
        if not finite:
            return None
        speed, heading, age, observed = values
        if age < 0:
            return None
        # The GPS reader may publish between loop_t0 and get_snapshot().
        age = max(age, now_mono - observed)
        if (speed < self.strategy.options["min_course_speed_mps"] or not 0 <= heading < 360
                or not 0 <= age <= age_limit or observed > now_mono + 1.0 / CONTROL_LOOP_HZ
                or abs(wrap_180(heading - boat_heading_deg)) > 1e-6):
            return None
        return {"speed_mps": speed, "course_deg": heading, "age_sec": age}

    def _prediction_context(self, bx, by, seg_idx, lookahead_idx, path_bearing_deg, motion):
        """Sample constant-velocity travel against an envelope around the outer routes.

        Outer paths are nominal routes, not surveyed banks. This is a conservative
        throttle constraint using the existing boundary-guard allowance, not an
        obstacle or vessel-dynamics prediction.
        """
        if motion is None:
            return None
        horizon = self.strategy.options["prediction_horizon_sec"]
        distance = motion["speed_mps"] * horizon
        if not math.isfinite(distance):
            return None
        course_rad = math.radians(motion["course_deg"])
        allowance = self.params["shift_cte_m"] - self.strategy.options["prediction_corridor_margin_m"]
        clearance = float("inf")
        max_turn = 0.0
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            px = bx + fraction * distance * math.sin(course_rad)
            py = by + fraction * distance * math.cos(course_rad)
            if not math.isfinite(px) or not math.isfinite(py):
                return None
            _, left_dist, left_cte, _ = self._closest_segment_metrics("left", px, py)
            _, right_dist, right_cte, _ = self._closest_segment_metrics("right", px, py)
            if not all(math.isfinite(v) for v in (left_dist, right_dist, left_cte, right_cte)):
                return None
            clearance = min(clearance, left_cte + allowance, allowance - right_cte)
            _, _, _, bearing = self._closest_segment_metrics(self.active_lane, px, py, seg_idx + 1)
            max_turn = max(max_turn, abs(wrap_180(bearing - path_bearing_deg)))
        points = self.enu_paths[self.active_lane]
        forecast_idx, _ = interpolate_lookahead(points, seg_idx, bx, by, distance)
        # Inspect the route ahead too: a straight velocity ray can miss a bend.
        for i in range(seg_idx, min(len(points) - 1, max(lookahead_idx, forecast_idx))):
            dx, dy = points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1]
            if math.hypot(dx, dy) > 1e-6:
                bearing = math.degrees(math.atan2(dx, dy)) % 360.0
                max_turn = max(max_turn, abs(wrap_180(bearing - path_bearing_deg)))
        return dict(motion, path_bearing_deg=path_bearing_deg, corridor_clear=clearance >= 0,
                    corridor_clearance_m=clearance, path_turn_deg=max_turn)

    def _line_wp_number(self, s):
        """Centerline waypoint number just ahead of along-course distance s (for display and logs)."""
        wps = self.paths["center"]
        k = int(math.ceil(max(0.0, s) / self.line_guidance.length * (len(wps) - 1)))
        return wps[max(0, min(len(wps) - 1, k))][0]

    def describe_line(self):
        line = self.line_guidance
        if self.active_lane == "test":
            o = self.strategy.options
            return (f"[TEST SQUARE] {o['test_leg_length_m']:g} m rectangles on +-{o['band_half_width_m']:.2f} m "
                    f"(bank lines) then +-{o['line_max_offset_m']:.2f} m (line limits) from the centerline, "
                    f"starting {o['test_lead_m']:g} m ahead; throttle {100 * o['test_throttle']:.0f}%; "
                    f"entry {line.entry_offset_m:+.2f} m from centerline")
        moved = line.line_offset_m - line.entry_offset_m
        return (f"[AUTO LINE] bearing {line.bearing_deg:.1f} deg, entry {line.entry_offset_m:+.2f} m from centerline, "
                f"line {line.line_offset_m:+.2f} m" + (f" (moved {moved:+.2f} m toward the centerline)" if moved else ""))

    def _update_test_square(self, bx, by, boat_heading_deg, now_mono, heading_confident, motion):
        """Follow the test rectangles leg by leg at test_throttle; no bank protection (the legs are the limits)."""
        opts = self.strategy.options
        line = self.line_guidance
        while self.test_leg < len(self.test_targets):
            ax, ay = self.test_leg_start
            tx, ty, label = self.test_targets[self.test_leg]
            leg_len = math.hypot(tx - ax, ty - ay)
            if leg_len > 1e-6:
                ux, uy = (tx - ax) / leg_len, (ty - ay) / leg_len
                along = (bx - ax) * ux + (by - ay) * uy
                if along < leg_len - opts["test_accept_m"]:
                    break
            self.test_leg += 1
            self.test_leg_start = (tx, ty)
            if self.test_leg < len(self.test_targets):
                log_event(f"[TEST SQUARE] leg {self.test_leg + 1}/{len(self.test_targets)}: "
                          f"{self.test_targets[self.test_leg][2]}")
        s, d = line.frame(bx, by)
        wp_number = self._line_wp_number(s)
        if self.test_leg >= len(self.test_targets):
            log_event("[TEST SQUARE] complete")
            return {"finished": True, "active_lane": self.active_lane, "shifted_to": None, "wp_number": wp_number,
                    "cte_right_m": 0.0, "steer_norm": 0.0, "target_bearing_deg": line.bearing_deg,
                    "boundary_guard": False, "throttle_blend": 0.0}

        cross = (bx - ax) * uy - (by - ay) * ux               # + = right of the leg
        speed = motion.get("speed_mps") if isinstance(motion, dict) else None
        speed_ok = isinstance(speed, (int, float)) and math.isfinite(speed)
        lookahead = self.strategy.lookahead_distance(speed if speed_ok else None)
        leg_bearing = math.degrees(math.atan2(ux, uy))
        intercept = max(-60.0, min(60.0, math.degrees(math.atan2(cross, lookahead))))
        target_bearing = (leg_bearing - intercept) % 360.0
        heading_err = None if boat_heading_deg is None else wrap_180(target_bearing - boat_heading_deg)
        command = self.strategy.update(cte_right_m=cross, heading_error_deg=heading_err, boundary_guard=False,
                                       now_mono=now_mono, heading_confident=heading_confident,
                                       speed_mps=speed if speed_ok else None)
        return {
            "finished": False, "active_lane": self.active_lane, "shifted_to": None, "wp_number": wp_number,
            "cte_right_m": cross, "steer_norm": command["steer_norm"], "target_bearing_deg": target_bearing,
            "boundary_guard": False, "throttle_blend": 0.0, "throttle_norm": opts["test_throttle"],
            "lookahead_m": lookahead, "lookahead_target_e_m": tx, "lookahead_target_n_m": ty,
            "cte_rate_mps": None, "predicted_cte_m": None,
            "prediction_status": f"square leg {self.test_leg + 1}: {label}",
            "prediction_corridor_clearance_m": opts["band_half_width_m"] - abs(d), "throttle_target_blend": None,
        }

    def _update_straight_line(self, bx, by, boat_heading_deg, now_mono, heading_confident, motion):
        """One straight line from AUTO entry: line-of-sight course command with bank protection."""
        line = self.line_guidance
        opts = self.strategy.options
        band = opts["band_half_width_m"]
        s, d = line.frame(bx, by)
        cte = d - line.line_offset_m
        wp_number = self._line_wp_number(s)
        if s >= line.length + self.finish_margin_m:
            return {"finished": True, "active_lane": self.active_lane, "shifted_to": None, "wp_number": wp_number,
                    "cte_right_m": cte, "steer_norm": 0.0, "target_bearing_deg": line.bearing_deg,
                    "boundary_guard": False, "throttle_blend": 0.0}

        speed = motion.get("speed_mps") if isinstance(motion, dict) else None
        speed_ok = isinstance(speed, (int, float)) and math.isfinite(speed)
        lookahead = self.strategy.lookahead_distance(speed if speed_ok else None)
        rate = future = None
        if boat_heading_deg is not None and speed_ok:
            rate = line.lateral_rate(speed, boat_heading_deg)
            future = d + opts["prediction_horizon_sec"] * rate
        outward = rate is not None and d * rate > 0

        # Soft zone near a bank: steeper intercept and boosted gain. Hard limit: the 1 s forecast leaves
        # the band while moving outward, so steer straight back toward the centerline.
        status = "line" if abs(cte) <= 0.3 else "merging"
        guard = False
        intercept = opts["max_intercept_deg"]
        if future is not None and outward and abs(future) >= band:
            status, guard = "hard_limit", True
            target_bearing = (line.bearing_deg + math.copysign(opts["recovery_intercept_deg"], -d)) % 360.0
        else:
            if abs(d) >= band - opts["guard_zone_m"] and (rate is None or outward):
                status, guard = "bank_guard", True
                intercept = opts["recovery_intercept_deg"]
            target_bearing = line.desired_course(d, lookahead, intercept)
        if boat_heading_deg is None:
            status = "heading_unavailable"

        heading_err = None if boat_heading_deg is None else wrap_180(target_bearing - boat_heading_deg)
        command = self.strategy.update(cte_right_m=cte, heading_error_deg=heading_err, boundary_guard=guard,
                                       now_mono=now_mono, heading_confident=heading_confident,
                                       speed_mps=speed if speed_ok else None)
        lx, ly = line.point(s + lookahead, line.line_offset_m)
        clearance = band - abs(future if future is not None else d)
        return {
            "finished": False, "active_lane": self.active_lane, "shifted_to": None, "wp_number": wp_number,
            "cte_right_m": cte, "steer_norm": command["steer_norm"], "target_bearing_deg": target_bearing,
            "boundary_guard": guard, "throttle_blend": command["throttle_blend"],
            "lookahead_m": lookahead, "lookahead_target_e_m": lx, "lookahead_target_n_m": ly,
            "cte_rate_mps": rate, "predicted_cte_m": None if future is None else future - line.line_offset_m,
            "prediction_status": status, "prediction_corridor_clearance_m": clearance,
            "throttle_target_blend": command["throttle_blend"],
        }

    def update(self, boat_lat, boat_lon, boat_heading_deg, now_mono=None, heading_confident=False,
               motion=None):
        """
        Call once per control cycle.
        Return a dict: finished, active_lane, shifted_to, wp_number, cte_right_m,
        steer_norm (-1 through +1), target_bearing_deg, boundary_guard, throttle_blend.
        """
        if now_mono is None:
            now_mono = time.monotonic()

        bx, by = latlon_to_enu(boat_lat, boat_lon, self.ref_lat, self.ref_lon)

        if self.line_guidance is not None and self.active_lane == "test":
            return self._update_test_square(bx, by, boat_heading_deg, now_mono, heading_confident, motion)
        if self.line_guidance is not None:
            return self._update_straight_line(bx, by, boat_heading_deg, now_mono, heading_confident, motion)

        # 1) Calculate the closest segment and CTE on the current path.
        predictive = getattr(self.strategy, "uses_predictive_guidance", False) is True
        search_idx = self.guidance_segment_idx + 1 if predictive else self.target_idx
        seg_idx, _, cte_right_m, path_bearing_deg = self._closest_segment_metrics(
            self.active_lane, bx, by, start_idx=search_idx
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

        # 2) Check for completion at the finish, including any configured margin.
        if self._check_finish_passed(self.active_lane, bx, by):
            return {
                "finished": True,
                "active_lane": self.active_lane,
                "shifted_to": None,
                "wp_number": self.paths[self.active_lane][-1][0],
                "cte_right_m": cte_right_m,
                "steer_norm": 0.0,
                "target_bearing_deg": path_bearing_deg,
                "boundary_guard": False,
                "throttle_blend": 0.0
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
                if (self.allow_lane_shift and not_recovering
                        and now_mono - self.last_shift_t >= self.params["shift_cooldown_sec"]):
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
                if (self.allow_lane_shift and not_recovering
                        and now_mono - self.last_shift_t >= self.params["shift_cooldown_sec"]):
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
                self.active_lane, bx, by, start_idx=0 if predictive else max(0, self.target_idx - 2)
            )
            self.target_idx = min(len(self.paths[self.active_lane]) - 1, new_seg + 1)
            seg_idx = new_seg

        # 4) Calculate the lookahead target bearing and servo steering command.
        lh_dist = self.params["boundary_lookahead_m"] if boundary_guard else self.params["lookahead_dist_m"]
        prediction = None
        if predictive:
            self.guidance_segment_idx = seg_idx
            selected_motion = self._fresh_selected_motion(motion, boat_heading_deg, now_mono)
            lh_dist = self.strategy.lookahead_distance(
                selected_motion["speed_mps"] if selected_motion else None, boundary_guard)
            lh_idx, (lx, ly) = interpolate_lookahead(
                self.enu_paths[self.active_lane], seg_idx, bx, by, lh_dist)
            prediction = self._prediction_context(bx, by, seg_idx, lh_idx, path_bearing_deg, selected_motion)
        else:
            lh_idx, (lx, ly) = self._select_lookahead_point(
                self.active_lane, self.target_idx, bx, by, lh_dist
            )
        target_bearing_deg = (math.degrees(math.atan2(lx - bx, ly - by)) + 360.0) % 360.0

        heading_err = None if boat_heading_deg is None else wrap_180(target_bearing_deg - boat_heading_deg)
        command = self.strategy.update(
            cte_right_m=cte_right_m,
            heading_error_deg=heading_err,
            boundary_guard=boundary_guard,
            now_mono=now_mono,
            heading_confident=heading_confident,
            **({"prediction": prediction} if predictive else {}),
        )

        result = {
            "finished": False,
            "active_lane": self.active_lane,
            "shifted_to": shifted_to,
            "wp_number": self.paths[self.active_lane][lh_idx][0],
            "cte_right_m": cte_right_m,
            "steer_norm": command["steer_norm"],
            "target_bearing_deg": target_bearing_deg,
            "boundary_guard": boundary_guard,
            "throttle_blend": command["throttle_blend"]
        }
        if predictive:
            result.update(lookahead_m=lh_dist, lookahead_target_e_m=lx, lookahead_target_n_m=ly,
                          prediction_corridor_clearance_m=prediction["corridor_clearance_m"] if prediction else None)
            for key in ("cte_rate_mps", "predicted_cte_m", "prediction_status", "throttle_target_blend"):
                result[key] = command.get(key)
        return result


def load_calibration(path):
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"Calibration file '{path}' not found. Run 'python3 calibrate_tool.py' to complete calibration first."
        )
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def open_session_log(stamp):
    """Save timestamped diagnostic events separately from the 20 Hz telemetry."""
    try:
        os.makedirs("logs", exist_ok=True)
        filepath = os.path.join("logs", f"multipath_events_{stamp}.log")
        handler = logging.FileHandler(filepath, encoding="utf-8")
        formatter = logging.Formatter("%(asctime)s UTC %(levelname)s %(message)s", "%Y-%m-%dT%H:%M:%S")
        formatter.converter = time.gmtime
        handler.setFormatter(formatter)
        SESSION_LOG.setLevel(logging.INFO)
        SESSION_LOG.addHandler(handler)
        log_event(f"[LOG] Started recording diagnostic events: {filepath}")
        return handler
    except OSError as exc:
        print(f"[LOG WARNING] Cannot write diagnostic events ({exc}); continuing without event logging.")
        return None


def gps_ntrip_diagnostics(gps_snap, ntrip_snap):
    """Keep CSV columns and periodic event summaries consistent. Ages are seconds."""
    return {
        "gps_valid": int(gps_snap["valid"]),
        "gps_age_sec": round(gps_snap["age_sec"], 3),
        "ntrip_connected": int(ntrip_snap["connected"]),
        "ntrip_status": ntrip_snap["status"],
        "ntrip_connection_attempts": ntrip_snap["connection_attempts"],
        "ntrip_last_response": ntrip_snap["last_response"],
        "ntrip_last_error": ntrip_snap["last_error"],
        "rtcm_bytes_rx": ntrip_snap["rtcm_bytes_rx"],
        "rtcm_rx_age_sec": (round(ntrip_snap["rtcm_rx_age_sec"], 3)
                            if ntrip_snap["rtcm_rx_age_sec"] is not None else ""),
        "ntrip_gga_sent": ntrip_snap["gga_sent"],
        "rtcm_bytes_written": gps_snap["rtcm_bytes_written"],
        "rtcm_write_errors": gps_snap["rtcm_write_errors"],
        "rtcm_last_write_error": gps_snap["last_rtcm_write_error"],
        # Receiver Doppler speed/course (RMC), correction age and height/UTC (GGA)
        "gps_sog_mps": round(gps_snap["sog_mps"], 3),
        "gps_cog_deg": round(gps_snap["cog_deg"], 2) if gps_snap["cog_deg"] is not None else "",
        "gps_diff_age_sec": gps_snap["diff_age_sec"] if gps_snap["diff_age_sec"] is not None else "",
        "gps_alt_m": gps_snap["alt_m"] if gps_snap["alt_m"] is not None else "",
        "gps_utc": gps_snap["gps_utc"]
    }


def open_telemetry_log(stamp=None):
    """Create a telemetry CSV for QGIS/analysis; allow operation to continue if the filesystem is read-only."""
    try:
        os.makedirs("logs", exist_ok=True)
        stamp = stamp or datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join("logs", f"multipath_log_{stamp}.csv")
        f = open(filepath, "w", newline="", encoding="utf-8")
        writer = csv.writer(f)
        writer.writerow([
            "timestamp", "mode", "lane", "wp_no", "fix_quality",
            "latitude", "longitude", "cte_right_m", "boat_heading_deg",
            "target_bearing_deg", "servo_us", "esc_us",
            "rc_ch1_us", "rc_ch2_us", "rc_steer_norm", "rc_throttle_norm",
            "num_sats", "hdop",
            *GPS_NTRIP_FIELDS,
            "auto_strategy", "throttle_blend",
            "control_heading_source", "control_heading_age_sec", "heading_confident", "heading_state",
            "legacy_heading_deg", "gps_course_valid", "gps_course_age_sec", "gps_course_utc",
            "gps_course_gga_skew_sec", "position_heading_deg", "position_heading_age_sec",
            "position_speed_mps", "gps_course_deg", "gps_course_speed_mps",
            "lookahead_m", "lookahead_target_e_m", "lookahead_target_n_m", "cte_rate_mps",
            "predicted_cte_m", "prediction_status", "prediction_corridor_clearance_m", "throttle_target_blend",
            "recovery_state"
        ])
        f.flush()
        print(f"[LOG] Started recording telemetry: {filepath}")
        return f, writer
    except OSError as exc:
        print(f"[LOG WARNING] Cannot write file ({exc}) -> continuing without telemetry logging.")
        return None, None


def main(session_stamp=None):
    parser = argparse.ArgumentParser(description="Haesucheon multipath autonomous boat controller")
    parser.add_argument("--config", default=DEFAULT_CALIBRATION_FILE, help="Path to calibration.json")
    parser.add_argument("--arduino-port", default=None, help="Manually override the Arduino port")
    parser.add_argument("--gps-port", default=None, help="Manually override the ZED-F9P GPS port")
    parser.add_argument("--spresense-port", default=None,
                        help="Spresense USB port (prefer /dev/serial/by-id/...); default: auto")
    parser.add_argument("--spresense-serial-number", default=None,
                        help="USB serial number identifying the Spresense, including generic CP210x bridges")
    parser.add_argument("--no-spresense", action="store_true", help="Disable the ArbaLabs GPS USB stream")
    parser.add_argument("--throttle-ratio", type=float, default=None, help="Override AUTO cruise throttle (adaptive strategy can still use full power)")
    parser.add_argument("--finish-margin-m", type=float, default=None,
                        help="Continue this many meters beyond the finish (overrides navigation.finish_margin_m; 0 keeps radius arrival)")
    parser.add_argument("--auto-strategy", type=canonical_auto_strategy_name,
                        choices=sorted(AUTO_STRATEGIES), default=None,
                        help="Override autonomous.strategy for this run (legacy unnumbered names also accepted)")
    parser.add_argument("--use-04", action="store_true",
                        help="Run 04_predictive_recovery whatever the strategy (same as autonomous.use_04_control)")
    parser.add_argument("--line-limit", type=float, default=None, metavar="X",
                        help="05: furthest the straight line may be drawn from the centerline, +-X m (default JSON, 1.5)")
    parser.add_argument("--bank-limit", type=float, default=None, metavar="X",
                        help="05: bank boundary, +-X m from the centerline (default JSON, 2.5)")
    parser.add_argument("--test-square", action="store_true",
                        help="05: on AUTO entry drive rectangles on the bank lines, then the line limits, and stop")
    parser.add_argument("--no-recovery", action="store_true",
                        help="Disable stuck recovery (reverse off the bank and retry) for this run")
    args = parser.parse_args()

    cal = load_calibration(args.config)
    auto_config = cal.get("autonomous", {})
    try:
        nav_params = resolve_navigation_params(cal.get("navigation", {}), args.finish_margin_m)
        auto_config = apply_run_overrides(
            auto_config, strategy=args.auto_strategy, use_04=args.use_04, line_limit_m=args.line_limit,
            bank_limit_m=args.bank_limit, test_square=True if args.test_square else None,
            recovery=False if args.no_recovery else None)
        auto_strategy = create_auto_strategy(auto_config, nav_params)
        recovery = StuckRecovery(resolve_recovery_config(auto_config))
    except ValueError as exc:
        parser.error(str(exc))

    heading_selector = MotionHeadingSelector(auto_strategy.options) if auto_strategy.uses_gps_course else None

    if serial is None or SMBus is None:
        print("[ERROR] Required packages (pyserial, smbus2) are not installed.")
        sys.exit(1)

    hw = cal["hardware"]
    rc_cfg = cal["rc"]
    pwm_cfg = cal["pwm"]
    ms_cfg = cal["mode_switch"]
    spresense_port = args.spresense_port or hw.get("spresense_port", "auto")
    spresense_serial = args.spresense_serial_number or hw.get("spresense_serial_number")

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

    try:
        navigator = MultipathNavigator(paths, nav_params, strategy=auto_strategy)
    except ValueError as exc:
        parser.error(str(exc))

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
    reserved_ports = ()
    if not args.no_spresense:
        reserved_ports = reserved_spresense_ports(spresense_port, spresense_serial)
        # Stock Spresense main boards may advertise only their generic CP210x
        # bridge. Do not let the broad Arduino fallback claim these unidentified
        # ports. A real CP210x-based RC board can be selected explicitly.
        if list_ports is not None:
            unidentified_bridges = tuple(
                p.device for p in list_ports.comports()
                if p.vid == 0x10C4
                and not (args.arduino_port and same_serial_port(p.device, args.arduino_port))
                and not (args.gps_port and same_serial_port(p.device, args.gps_port))
            )
            reserved_ports += unidentified_bridges
    try:
        for override in (args.arduino_port, args.gps_port):
            if override and any(same_serial_port(override, port) for port in reserved_ports):
                raise ValueError("Spresense USB output must use a different port from Arduino and GPS")
        ard_port, gps_port = detect_serial_ports(
            ard_pref, gps_pref, force_manual=force_manual, excluded_ports=reserved_ports)
    except Exception:
        pca.close()
        raise

    if not ard_port:
        pca.close()
        raise RuntimeError("Arduino serial port not found. Check the USB connection; "
                           "select a generic CP210x RC board with --arduino-port.")
    if not gps_port:
        pca.close()
        raise RuntimeError("ZED-F9P GPS serial port not found. Check the USB connection.")

    rc = ArduinoRCReader(ard_port, hw.get("arduino_baud", 115200))
    gps = ZEDF9PReceiver(gps_port, hw.get("gps_baud", 115200))
    ntrip = NGII_NtripClient(NTRIP_CONFIG, gps)
    spresense = None if args.no_spresense else SpresenseGPSStreamer(
        gps.get_latest_gps_sample, port=spresense_port, serial_number=spresense_serial,
        excluded_ports=(ard_port, gps_port), logger=log_event)
    log_file, log_writer = open_telemetry_log(session_stamp)

    running = True

    def shutdown_handler(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    mode = "MANUAL"
    last_esc_us = pwm_cfg["esc_neutral_us"]
    last_recovery_state = ""
    right_steer_hold_start = None
    last_no_gps_warn_t = 0.0
    last_non_rtk_warn_t = 0.0

    # Cruise is the constant strategy output and the adaptive strategy's lower level.
    auto_cruise_us = int(round(
        pwm_cfg["esc_neutral_us"]
        + pwm_cfg["auto_throttle_ratio"] * (pwm_cfg["esc_fwd_max_us"] - pwm_cfg["esc_neutral_us"])
    ))

    print("\n=====================================================================")
    print("  Haesucheon multipath autonomous navigation started (initial mode: MANUAL)")
    log_event("[AUTO STRATEGY] " + json.dumps({
        "strategy": auto_strategy.name,
        "parameters": auto_strategy.options,
        "navigation": nav_params,
        "cruise_ratio": pwm_cfg["auto_throttle_ratio"],
        "cruise_pwm_us": auto_cruise_us,
        "full_pwm_us": pwm_cfg["esc_fwd_max_us"],
        "recovery": recovery.options,
    }, sort_keys=True))
    print(f"  - AUTO throttle  : {auto_strategy.describe_throttle(auto_cruise_us, pwm_cfg['esc_fwd_max_us'])}")
    print(f"  - Finish margin  : {navigator.finish_margin_m:g} m beyond the final waypoint")
    rec_o = recovery.options
    print(f"  - Stuck recovery : " + (
        f"ON (below {rec_o['stuck_speed_mps']:g} m/s for {rec_o['stuck_sec']:g} s -> reverse "
        f"{100 * rec_o['reverse_throttle']:.0f}% for {rec_o['reverse_sec']:g} s, up to {rec_o['max_attempts']} tries)"
        if recovery.enabled else "OFF"))
    if navigator.line_guidance is not None and auto_strategy.options["test_square"]:
        print("  - TEST SQUARE    : AUTO drives the test rectangles, then stops (not a race run)")
    print("  - MANUAL -> AUTO : Hold neutral throttle + right steering for 2.0 seconds (then transmitter steering is ignored)")
    print("  - AUTO -> MANUAL : Apply full reverse throttle to return to MANUAL immediately")
    print("  - Safe shutdown  : Ctrl + C (immediate neutral stop)")
    print("=====================================================================\n")

    period_sec = 1.0 / CONTROL_LOOP_HZ
    last_gps_state = None
    last_diagnostic_log = None
    last_log_flush = time.monotonic()

    def record_diagnostics(now, gps_snap):
        # Called after PWM outputs, including neutral failsafe outputs, are applied.
        nonlocal last_gps_state, last_diagnostic_log, last_log_flush, log_writer
        diagnostics = gps_ntrip_diagnostics(gps_snap, ntrip.get_snapshot())
        gps_state = (gps_snap["fix_quality"], gps_snap["valid"])
        if gps_state != last_gps_state:
            SESSION_LOG.info("[GPS STATE] fix_quality=%d (%s), valid=%s, age=%.3fs",
                             gps_snap["fix_quality"], FIX_QUALITY_NAMES.get(gps_snap["fix_quality"], "Unknown"),
                             gps_snap["valid"], gps_snap["age_sec"])
            last_gps_state = gps_state
        if last_diagnostic_log is None or now - last_diagnostic_log >= 10.0:
            SESSION_LOG.info("[DIAGNOSTICS] mode=%s, fix_quality=%d, %s",
                             mode, gps_snap["fix_quality"], diagnostics)
            last_diagnostic_log = now
        if log_writer is not None and now - last_log_flush >= 1.0:
            try:
                log_file.flush()
            except OSError as exc:
                log_event(f"[LOG WARNING] Telemetry flush failed: {exc}", logging.WARNING)
                log_writer = None
            last_log_flush = now
        return diagnostics

    try:
        rc.start()
        gps.start()
        ntrip.start()
        if spresense is not None:
            spresense.start()
        while running:
            loop_t0 = time.monotonic()
            recovery_state = ""

            ch1_us, ch2_us, rc_alive = rc.get_latest()
            gps_snap = gps.get_snapshot()
            heading_info = {"heading_deg": gps_snap["heading_deg"], "source": "legacy_blend",
                            "age_sec": None, "confident": gps_snap["heading_deg"] is not None,
                            "state": "legacy"}

            # [Safety] Stop in neutral immediately if the Arduino serial connection or wiring is lost.
            if not rc_alive:
                servo_us = pwm_cfg["servo_center_us"]
                esc_us = pwm_cfg["esc_neutral_us"]
                pca.set_pwm_us(servo_ch, servo_us)
                pca.set_pwm_us(esc_ch, esc_us)
                right_steer_hold_start = None
                if heading_selector is not None:
                    heading_selector.reset()
                    auto_strategy.suspend()
                if mode == "AUTO":
                    mode = "MANUAL"
                    print("\n[FAILSAFE] RC receiver signal lost -> stopping in neutral and switching to MANUAL")
                record_diagnostics(loop_t0, gps_snap)
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
                if heading_selector is not None:
                    heading_selector.observe_manual(gps_snap, th_norm, loop_t0)
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
                            if heading_selector is not None:
                                # Preserve only fresh course backed by recent forward motion.
                                # Position fallback still starts from the new AUTO anchor.
                                # Keep the position/course snapshot whose validity was
                                # checked above, while reflecting the anchor reset.
                                gps_snap = dict(gps_snap, heading_deg=None, position_heading_deg=None,
                                                position_heading_age_sec=None, position_heading_time_mono=None,
                                                position_speed_mps=None)
                                reused = heading_selector.enter_auto(gps_snap, loop_t0)
                                heading_info = heading_selector.select(gps_snap, loop_t0)
                                log_event(f"[AUTO HEADING] {'Reusing forward GPS course' if reused else 'Acquiring forward motion'}")
                            lane, wp_no, init_dist = navigator.enter_auto_mode(
                                gps_snap["lat"], gps_snap["lon"], now_mono=loop_t0
                            )
                            if navigator.line_guidance is not None:
                                log_event(navigator.describe_line())
                            recovery.reset(loop_t0)
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
                # Stuck recovery: forward command but no GPS speed -> neutral, reverse, neutral, then retry.
                rec_cmd = recovery.update(loop_t0, last_esc_us > pwm_cfg["esc_neutral_us"],
                                          gps_snap.get("sog_mps") if gps_snap["valid"] else None)
                # 1) Ignore transmitter steering (st_norm); monitor for full reverse throttle.
                if th_norm <= -ms_cfg["full_reverse_ratio_threshold"]:
                    mode = "MANUAL"
                    if heading_selector is not None:
                        heading_selector.reset()
                        auto_strategy.suspend()
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
                    if heading_selector is not None:
                        heading_selector.suspend()
                        auto_strategy.suspend()
                        heading_info = heading_selector.select(gps_snap, loop_t0)
                    if loop_t0 - last_no_gps_warn_t > 2.0:
                        print("\n[GPS WARNING] GPS reception delayed -> holding neutral until reception recovers.")
                        last_no_gps_warn_t = loop_t0

                elif rec_cmd is not None:
                    recovery_state = rec_cmd["state"]
                    if recovery_state != last_recovery_state:
                        log_event(f"[RECOVERY] {recovery_state} (attempt {recovery.attempts}/"
                                  f"{recovery.options['max_attempts']}, GPS speed {gps_snap.get('sog_mps')})",
                                  logging.WARNING)
                    servo_us = pwm_cfg["servo_center_us"]
                    # Reverse motion must not become the steering heading after the retry.
                    gps.reset_heading_anchor()
                    if heading_selector is not None:
                        heading_selector.suspend()
                    auto_strategy.suspend()
                    if recovery_state == "gave_up":
                        mode = "MANUAL"
                        if heading_selector is not None:
                            heading_selector.reset()
                        esc_us = pwm_cfg["esc_neutral_us"]
                        print("\n[RECOVERY] Still stuck after the last attempt -> neutral and MANUAL mode.")
                    else:
                        esc_us = map_norm_to_pwm(rec_cmd["throttle_norm"], pwm_cfg["esc_neutral_us"],
                                                 pwm_cfg["esc_rev_max_us"], pwm_cfg["esc_fwd_max_us"])

                else:
                    # If RTK drops to standard GPS/DGPS, continue navigating and warn every 10 seconds.
                    fq = gps_snap["fix_quality"]
                    if fq not in (4, 5) and (loop_t0 - last_non_rtk_warn_t > 10.0):
                        fq_str = FIX_QUALITY_NAMES.get(fq, f"Code ({fq})")
                        print(f"\n[POSITION WARNING] RTK is not active ({fq_str}) - continuing in AUTO mode.")
                        last_non_rtk_warn_t = loop_t0

                    if heading_selector is not None:
                        heading_info = heading_selector.select(gps_snap, loop_t0)
                    nav_info = navigator.update(
                        gps_snap["lat"],
                        gps_snap["lon"],
                        heading_info["heading_deg"],
                        now_mono=loop_t0,
                        heading_confident=heading_info["confident"],
                        motion=heading_info,
                    )

                    if nav_info["shifted_to"] is not None:
                        print(
                            f"\n[PATH CHANGE] Disturbance detected; switching path -> '{nav_info['shifted_to'].upper()}' "
                            f"(cross-track error: {nav_info['cte_right_m']:+.2f}m)"
                        )

                    if nav_info["finished"]:
                        mode = "MANUAL"
                        if heading_selector is not None:
                            heading_selector.reset()
                            auto_strategy.suspend()
                        servo_us = pwm_cfg["servo_center_us"]
                        esc_us = pwm_cfg["esc_neutral_us"]
                        print(
                            f"\n[COURSE COMPLETE] Reached finish for WP #{nav_info['wp_number']} "
                            f"(margin: {navigator.finish_margin_m:g} m). "
                            f"Stopping the motor in neutral and returning to MANUAL mode."
                        )
                    else:
                        servo_us = map_norm_to_pwm(
                            nav_info["steer_norm"],
                            pwm_cfg["servo_center_us"],
                            pwm_cfg["servo_left_us"],
                            pwm_cfg["servo_right_us"]
                        )
                        if nav_info.get("throttle_norm") is not None:
                            # Fixed throttle requested by the navigator (05 test square).
                            esc_us = map_norm_to_pwm(nav_info["throttle_norm"], pwm_cfg["esc_neutral_us"],
                                                     pwm_cfg["esc_rev_max_us"], pwm_cfg["esc_fwd_max_us"])
                        else:
                            thr_blend = max(0.0, min(1.0, nav_info.get("throttle_blend", 0.0)))
                            esc_us = int(round(
                                auto_cruise_us
                                + thr_blend * (pwm_cfg["esc_fwd_max_us"] - auto_cruise_us)
                            ))

            # 4) Write the servo and ESC hardware outputs.
            pca.set_pwm_us(servo_ch, servo_us)
            pca.set_pwm_us(esc_ch, esc_us)
            last_esc_us = esc_us
            last_recovery_state = recovery_state

            # 5) Display the live terminal status line and write CSV telemetry.
            diagnostics = record_diagnostics(loop_t0, gps_snap)
            fq_short = FIX_QUALITY_NAMES.get(gps_snap["fix_quality"], str(gps_snap["fix_quality"]))
            hdg_txt = f"{heading_info['heading_deg']:5.1f}" if heading_info["heading_deg"] is not None else " N/A "
            if mode == "AUTO" and nav_info is not None and not nav_info["finished"]:
                guard_tag = "[WALL GUARD]" if nav_info["boundary_guard"] else ""
                thr_pct = int(round(100.0 * nav_info.get("throttle_blend", 0.0)))
                status_line = (
                    f"\r[AUTO:{nav_info['active_lane'].upper():6s}{guard_tag}] "
                    f"WP#{nav_info['wp_number']:02d} | CTE:{nav_info['cte_right_m']:+5.2f}m | "
                    f"Heading:{hdg_txt}deg->Target:{nav_info['target_bearing_deg']:5.1f}deg | "
                    f"GPS:{fq_short} | Boost:{thr_pct:3d}% Servo:{servo_us:4d} ESC:{esc_us:4d}   "
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
                        f"{heading_info['heading_deg']:.1f}" if heading_info["heading_deg"] is not None else "",
                        f"{nav_info['target_bearing_deg']:.1f}" if nav_info else "",
                        servo_us,
                        esc_us,
                        ch1_us,
                        ch2_us,
                        st_norm,
                        th_norm,
                        gps_snap["num_sats"],
                        gps_snap["hdop"],
                        *diagnostics.values(),
                        auto_strategy.name,
                        nav_info["throttle_blend"] if nav_info else "",
                        heading_info["source"], heading_info["age_sec"],
                        int(heading_info["confident"]), heading_info["state"],
                        gps_snap["heading_deg"], int(gps_snap.get("course_valid", False)),
                        gps_snap.get("course_age_sec"), gps_snap.get("course_utc"),
                        gps_snap.get("course_gga_skew_sec"), gps_snap.get("position_heading_deg"),
                        gps_snap.get("position_heading_age_sec"), gps_snap.get("position_speed_mps"),
                        gps_snap.get("course_deg"), gps_snap.get("course_speed_mps"),
                        *[nav_info.get(key) if nav_info else None for key in (
                            "lookahead_m", "lookahead_target_e_m", "lookahead_target_n_m", "cte_rate_mps",
                            "predicted_cte_m", "prediction_status", "prediction_corridor_clearance_m", "throttle_target_blend")],
                        recovery_state
                    ])
                except OSError as exc:
                    log_event(f"[LOG WARNING] Telemetry write failed: {exc}", logging.WARNING)
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
        if spresense is not None:
            spresense.stop()
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
    session_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    session_handler = open_session_log(session_stamp)
    try:
        main(session_stamp)
    except Exception:
        SESSION_LOG.exception("[SESSION ERROR] Controller exited with an exception")
        raise
    finally:
        SESSION_LOG.info("[SESSION END]")
        if session_handler is not None:
            SESSION_LOG.removeHandler(session_handler)
            session_handler.close()
