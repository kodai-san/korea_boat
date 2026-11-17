"""Boat dashboard controller with persistent RC takeover.

MANUAL: move either RC stick to take ownership from the dashboard. Hold
neutral throttle + right steering for the calibrated duration (default 2 s)
to enter AUTO with valid GPS. RC-started AUTO requires a live receiver and
can run without the dashboard. Release the entry steering before another
right-steering takeover; full reverse and full left always remain available.
AUTO/BALLOON: full steering either way (85%) or full reverse (calibrated
threshold) takes ownership without an arming delay. Web-only mode and GCS
link hold do not disable takeover. Ownership persists at neutral and after
RC loss (which commands stop after 0.5 s). To hand back to the dashboard,
center the sticks and explicitly select a mode, or use the RC AUTO gesture.
E-stop remains authoritative.
This software path requires a running controller and working RC/I2C hardware.
"""

import os
import sys
import csv
import math
import time
import json
import glob
import signal
import socket
import base64
import threading
from datetime import datetime
from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

# SIGHUP 무시 (SSH 종료 시에도 유지)
if hasattr(signal, "SIGHUP"):
    signal.signal(signal.SIGHUP, signal.SIG_IGN)

# SIGTERM 수신 시 안전 정지 실행
def _handle_sigterm(signum, frame):
    raise SystemExit("SIGTERM received")

if hasattr(signal, "SIGTERM"):
    signal.signal(signal.SIGTERM, _handle_sigterm)

# 안전한 콘솔 출력
def safe_print(*args, **kwargs):
    try:
        print(*args, **kwargs, flush=True)
    except Exception:
        pass

try:
    import serial
    SERIAL_ERR_TYPES = (serial.SerialException, OSError)
except Exception:
    serial = None
    SERIAL_ERR_TYPES = (OSError,)

# Motor output uses the same smbus2 driver as multipath_boat.py.
import subprocess
import shutil

try:
    import cv2
    import numpy as np
    CV_AVAILABLE = True
except Exception:
    cv2 = None
    np = None
    CV_AVAILABLE = False

# AUTO path following uses the same navigator, strategies and heading selector as multipath_boat.py
from multipath_boat import (
    MultipathNavigator, ZEDF9PReceiver, resolve_navigation_params, latlon_to_enu,
    normalize_axis, detect_serial_ports, PCA9685Driver, SMBus,
)
from auto_strategies import create_auto_strategy
from gps_heading import MotionHeadingSelector

HW_AVAILABLE = SMBus is not None

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# 스크립트 기준 경로 반환
def resolve_path(filename):
    p = os.path.join(BASE_DIR, filename)
    return p if os.path.exists(p) else filename

CENTER = 4500
max_LEFT = 5500
max_RIGHT = 3000

STOP = 4443
DEFAULT_MAX_GO = 4800
max_GO = DEFAULT_MAX_GO
max_BACK = 4000
MAX_GO_PRESETS = (4700, 4800, 5000, 5100, 5200)
MAX_GO_CONTINUOUS_SEC = 600.0

STEER_GAIN = 15.0
STEER_SIGN = 1

# 조향 control 파라미터 (P / PID)
PID_CONFIG_FILE = os.path.join(BASE_DIR, "pid_config.json")
STEER_CTRL_MODE = "P"
PID_KP = 15.0
PID_KI = 0.0
PID_KD = 3.0
PID_I_MAX_DEG = 15.0

# ---------------------------------------------------------------------------
# Hardware calibration shared with multipath_boat.py (calibration.json):
# serial ports/bauds, PCA9685 bus/address/frequency/channels, servo and ESC
# microseconds, RC receiver pulses and the full-reverse override ratio.
# boat.py keeps its logical PWM scale above (CENTER/STOP/max_GO ...) because the
# web page uses those numbers; set_pwm()/safe_stop() convert it to calibrated
# microseconds: CENTER..max_LEFT/max_RIGHT -> servo center..left/right,
# STOP..DEFAULT_MAX_GO -> ESC neutral..forward max (higher max_GO presets are
# capped at forward max), STOP..max_BACK -> ESC neutral..reverse max.
# ---------------------------------------------------------------------------
HW_CONFIG_FILE = "calibration.json"


def _load_hw_calibration():
    cfg_path = resolve_path(HW_CONFIG_FILE)
    if not os.path.exists(cfg_path):
        raise SystemExit(f"{HW_CONFIG_FILE} not found: boat.py needs the boat's hardware calibration")
    try:
        with open(cfg_path, "r", encoding="utf-8") as f:
            cal = json.load(f)
        hw, rc, pw = cal["hardware"], cal["rc"], cal["pwm"]
        ms = cal.get("mode_switch", {})
        out = {
            "arduino_port": str(hw.get("arduino_port", "auto")),
            "arduino_baud": int(hw.get("arduino_baud", 115200)),
            "gps_port": str(hw.get("gps_port", "auto")),
            "gps_baud": int(hw.get("gps_baud", 115200)),
            "i2c_bus": int(hw["i2c_bus"]),
            "pca_addr": int(hw["pca9685_addr"]),
            "pca_freq": float(hw["pca9685_freq_hz"]),
            "servo_ch": int(hw["servo_channel"]),
            "esc_ch": int(hw["esc_channel"]),
            "servo_center": float(pw["servo_center_us"]),
            "servo_left": float(pw["servo_left_us"]),
            "servo_right": float(pw["servo_right_us"]),
            "esc_neutral": float(pw["esc_neutral_us"]),
            "esc_fwd": float(pw["esc_fwd_max_us"]),
            "esc_rev": float(pw["esc_rev_max_us"]),
            "st_neu": int(rc["steer_neutral_us"]),
            "st_left": int(rc["steer_left_us"]),
            "st_right": int(rc["steer_right_us"]),
            "st_db": int(rc["steer_deadband_us"]),
            "th_neu": int(rc["throttle_neutral_us"]),
            "th_fwd": int(rc["throttle_fwd_us"]),
            "th_rev": int(rc["throttle_rev_us"]),
            "th_db": int(rc["throttle_deadband_us"]),
            "rev_ratio": float(ms.get("full_reverse_ratio_threshold", 0.75)),
            "auto_right_ratio": float(ms.get("right_steer_ratio_threshold", 0.75)),
            "auto_hold_sec": float(ms.get("right_steer_hold_sec", 2.0)),
        }
    except (KeyError, TypeError, ValueError) as e:
        raise SystemExit(f"{HW_CONFIG_FILE}: invalid or missing hardware/rc/pwm setting ({e})")
    pulses = [out[k] for k in ("servo_center", "servo_left", "servo_right", "esc_neutral", "esc_fwd", "esc_rev")]
    if not all(800.0 <= us <= 2200.0 for us in pulses):
        raise SystemExit(f"{HW_CONFIG_FILE}: servo/ESC pulses must be 800-2200 us")
    if not (min(out["esc_fwd"], out["esc_rev"]) < out["esc_neutral"] < max(out["esc_fwd"], out["esc_rev"])):
        raise SystemExit(f"{HW_CONFIG_FILE}: esc_neutral_us must lie between reverse and forward max")
    if not (0 <= out["servo_ch"] <= 15 and 0 <= out["esc_ch"] <= 15 and out["servo_ch"] != out["esc_ch"]):
        raise SystemExit(f"{HW_CONFIG_FILE}: servo/ESC channels must be different PCA9685 channels 0-15")
    if not (0.0 < out["rev_ratio"] <= 1.0):
        raise SystemExit(f"{HW_CONFIG_FILE}: mode_switch.full_reverse_ratio_threshold must be in (0, 1]")
    if not (0.0 < out["auto_right_ratio"] <= 1.0) or not (math.isfinite(out["auto_hold_sec"]) and out["auto_hold_sec"] > 0):
        raise SystemExit(f"{HW_CONFIG_FILE}: invalid RC AUTO gesture threshold/duration")
    return out


HW = _load_hw_calibration()

# RC receiver (pulses in us, read from the Arduino); channel reversal is handled by normalize_axis()
ST_NEU, TH_NEU = HW["st_neu"], HW["th_neu"]
# RC emergency override: full steer either way, or full reverse (fraction of each stick's travel)
RC_OVERRIDE_STEER_RATIO = 0.85
RC_OVERRIDE_REVERSE_RATIO = HW["rev_ratio"]

RC_VALID_MIN = 750
RC_VALID_MAX = 2200
PWM_WRITE_MIN_DELTA = 15
PWM_PERIOD = 0.05
RC_FILTER = 0.25
MAX_DRIVE_SLEW_STEP = 80

# 다중 경로 전환 및 항법 상수
SWITCH_DIST_M = 1.15
SWITCH_MARGIN_M = 0.35
ARRIVE_DIST_M = 1.5
LOOKAHEAD_MIN_M = 1.5
HEADING_INIT_MOVE_M = 0.35

RC_TIMEOUT = 0.5
GPS_TIMEOUT = 1.0
BAD_FIX_TIMEOUT = 1.0
MAX_GPS_GLITCH_DIST_M = 2.5
MAX_GPS_GLITCH_MPS = 8.0
HEADING_INIT_MAX_SEC = 5.0
STALL_TIMEOUT_SEC = 5.0
STALL_MIN_MOVED_M = 0.5
MAX_SERIAL_LINES_LOOP = 20
SERIAL_RECONNECT_SEC = 1.5

RC_PORT = HW["arduino_port"]    # "auto" = detect by USB ID (same rules as multipath_boat.py)
GPS_PORT = HW["gps_port"]
RC_BAUD = HW["arduino_baud"]
GPS_BAUD = HW["gps_baud"]

NTRIP_ENABLE = True
NTRIP_HOST = "rts2.ngii.go.kr"
NTRIP_PORT = 2101
NTRIP_MOUNT = "VRS-RTCM32"
NTRIP_USER = "lim6246"
NTRIP_PASS = "ngii"

WEB_ENABLE = True
WEB_PORT = 8080
HTML_FILENAME = "gcs_2_usv_hud.html"
GCS_LINK_TIMEOUT_SEC = 3.5

PCA_ADDR = HW["pca_addr"]
PCA_I2C_BUS = HW["i2c_bus"]
PCA_FREQ_HZ = HW["pca_freq"]
SERVO_CH = HW["servo_ch"]
ESC_CH = HW["esc_ch"]

# 카메라(16:9 와이드) 및 비전 control 상수
CAM_ENABLE = True
CAM_SENSOR_WIDTH = 1280
CAM_SENSOR_HEIGHT = 720
CAM_WIDTH = 640
CAM_HEIGHT = 360
CAM_FPS = 30
CAM_FLIP_METHOD = 2  # 0=normal, 2=rotate 180 (nvvidconv flip-method / cv2.flip)
CAM_HFOV_DEG = 62.2
CAM_RECONNECT_SEC = 2.0
CAM_TIMEOUT = 2.5

BALLOON_STEER_GAIN = 50.0
BALLOON_STEER_KD = 3.5
BALLOON_MIN_AREA = 110
BALLOON_CHARGE_ANGLE_DEG = 18.0
BALLOON_MEMORY_SEC = 1.0
BALLOON_POP_FOLLOW_SEC = 1.2
BALLOON_SEARCH_TIMEOUT_SEC = 25.0

# 비전 표적 임무 파라미터
BALLOON_ACTION_MODE = "APPROACH_STOP"  # APPROACH_STOP 또는 RAM_POP
BALLOON_REAL_DIAM_M = 3.50             # 표적 실제 직경(m)
BALLOON_STOP_DIST_M = 3.50             # 목표 정지 거리(m)
BALLOON_STOP_AREA_PCT = 30.0           # 목표 정지 면적(%)
BALLOON_SLOW_DIST_M = 8.0              # 감속 시작 거리(m)
BALLOON_FOCAL_PX = 520.0

# 기본 HSV 색상 프리셋
HSV_PRESETS = {
    "default": {  # 기본 핫핑크
        "lower1": (150, 100, 80), "upper1": (179, 255, 255),
        "lower2": (0, 115, 100),  "upper2": (4, 255, 255),
    },
    "bright": {   # 강한 일사량
        "lower1": (152, 120, 100), "upper1": (179, 255, 255),
        "lower2": (0, 130, 120),   "upper2": (3, 255, 255),
    },
    "cloudy": {   # 흐림/저조도
        "lower1": (146, 80, 65), "upper1": (179, 255, 255),
        "lower2": (0, 95, 80),   "upper2": (5, 255, 255),
    },
}

# 사용자 지정 타겟 색상 (HSV)
CUSTOM_COLOR_ENABLE = True
TARGET_HUE = 166          # 목표 색상 Hue (0~179)
TARGET_HUE_TOL = 10       # 허용 Hue 범위
TARGET_SAT_MIN = 105      # 최소 채도 (0~255)
TARGET_VAL_MIN = 80       # 최소 밝기 (0~255)
TARGET_COLOR_HEX = "#ff2a85"

gps_ser_lock = threading.Lock()
state_lock = threading.Lock()
event_logs = deque(maxlen=40)

# 이벤트 로그 기록
def add_event_log(tag, msg, level="auto"):
    now_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    safe_print(f"[{now_str}] [{tag}] {msg}")
    with state_lock:
        event_logs.append({"time": now_str, "tag": tag, "msg": msg, "level": level})

shared_state = {
    "mode": "MANUAL",
    "estop": False,
    "speed": 0.0,
    "maxSpeed": 0.0,
    "movedDist": 0.0,
    "lapElapsedSec": 0.0,
    "stPulse": ST_NEU,
    "thPulse": TH_NEU,
    "rawSt": None,
    "rawTh": None,
    "steerPwm": CENTER,
    "drivePwm": STOP,
    "targetPath": "Center",
    "pathAutoSwitch": True,
    "wpIndex": 1,
    "wpTotal": 65,
    "wpDist": 0.0,
    "distLeft": 0.0,
    "distCenter": 0.0,
    "distRight": 0.0,
    "fixStatus": 0,
    "satellites": 0,
    "hdop": 99.9,
    "ntripKb": 0.0,
    "ntripConnected": False,
    "lat": 0.0,
    "lon": 0.0,
    "heading": 0.0,
    "targetAngle": 0.0,
    "angleErr": 0.0,
    "loopHz": 50.0,
    "rcHz": 0.0,
    "jetsonCpuTemp": 0.0,
    "jetsonLoad": 0.0,
    "lampRc": "warn",
    "lampGps": "warn",
    "lampGlitch": "ok",
    "lampStall": "ok",
    "lampSerial": "ok",
    "lampCsv": "ok",
    "lampCam": "warn",
    "lampLink": "ok",
    "linkWatchdog": True,
    "linkTimeoutSec": GCS_LINK_TIMEOUT_SEC,
    "linkPaused": False,
    "rcIgnore": False,
    "camLteMode": True,
    "cameraOk": False,
    "balloonDetected": False,
    "balloonState": "IDLE",
    "balloonX": 0.0,
    "balloonY": 0.0,
    "balloonRadius": 0.0,
    "balloonAreaPct": 0.0,
    "balloonDistM": 0.0,
    "balloonAngleErr": 0.0,
    "balloonFps": 0.0,
    "hsvPreset": "default",
    "camFlip": 180,
    "camShowMask": False,
    "steerCtrlMode": STEER_CTRL_MODE,
    "pidKp": PID_KP,
    "pidKi": PID_KI,
    "pidKd": PID_KD,
    "pidIMax": PID_I_MAX_DEG,
    "pidPTerm": 0.0,
    "pidITerm": 0.0,
    "pidDTerm": 0.0,
    "maxGo": max_GO,
    "highPowerElapsedSec": 0.0,
    "balloonActionMode": BALLOON_ACTION_MODE,
    "balloonRealDiamM": BALLOON_REAL_DIAM_M,
    "balloonStopDistM": BALLOON_STOP_DIST_M,
    "balloonStopAreaPct": BALLOON_STOP_AREA_PCT,
    "balloonFinishSeq": 0,
    "balloonFinishMsg": "",
    "customColorEnable": CUSTOM_COLOR_ENABLE,
    "targetHue": TARGET_HUE,
    "targetHueTol": TARGET_HUE_TOL,
    "targetSatMin": TARGET_SAT_MIN,
    "targetValMin": TARGET_VAL_MIN,
    "targetColorHex": TARGET_COLOR_HEX,
}

pending_web_cmd = {"cmd": None, "value": None}
web_cmd_queue = deque(maxlen=32)
_last_gcs_hb_mono = 0.0
_gcs_hb_streak = 0
_gcs_ever_connected = False

ser = None
gps_ser = None
pca = None

# 시리얼 포트 연결 (흐름 control 비활성화 및 중복 포트 제외)
def _open_serial_port(port_name, baud):
    if serial is None or not port_name:
        return None
    try:
        s = serial.Serial(
            port_name,
            baud,
            timeout=0.02,
            write_timeout=0.05,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        try:
            s.reset_input_buffer()
        except Exception:
            pass
        add_event_log("SERIAL", f"Serial port connected: {port_name} ({baud} bps)", "auto")
        return s
    except Exception:
        return None


# Ports come from calibration.json; "auto" (or a missing configured port) uses
# multipath_boat.detect_serial_ports(): u-blox by USB ID for the GPS, then the Arduino.
def open_rc_serial():
    other = getattr(gps_ser, "port", None)
    ard_port, _ = detect_serial_ports(RC_PORT, GPS_PORT, excluded_ports=(other,) if other else ())
    return _open_serial_port(ard_port, RC_BAUD)


def open_gps_serial():
    other = getattr(ser, "port", None)
    _, gps_port = detect_serial_ports(RC_PORT, GPS_PORT, excluded_ports=(other,) if other else ())
    return _open_serial_port(gps_port, GPS_BAUD)

# Initialize PCA9685 on the calibrated I2C bus.
def init_pca9685():
    global pca, _last_steer, _last_drive, _need_safe_stop_retry
    if not HW_AVAILABLE:
        add_event_log("I2C", "Motor output unavailable: install smbus2", "warn")
        return False
    try:
        pca = PCA9685Driver(bus_num=PCA_I2C_BUS, address=PCA_ADDR, freq_hz=PCA_FREQ_HZ)
        _last_steer = _last_drive = None
        _need_safe_stop_retry = True
        # Initialize neutral before accepting commands after startup/reconnection.
        pca.set_pwm_us(SERVO_CH, HW["servo_center"])
        pca.set_pwm_us(ESC_CH, HW["esc_neutral"])
        add_event_log("I2C", f"PCA9685 connected (bus {PCA_I2C_BUS}, 0x{PCA_ADDR:02x})", "auto")
        return True
    except Exception as e:
        if pca is not None:
            try:
                pca.close()
            except Exception:
                pass
        pca = None
        shared_state["lampSerial"] = "err"
        add_event_log("I2C", f"PCA9685 unavailable: {e}", "warn")
        return False

# 주변장치(RC, GPS, PCA9685) 초기화
def init_peripherals():
    global ser, gps_ser, pca
    gps_ser = open_gps_serial()
    ser = open_rc_serial()
    init_pca9685()

# PWM 상수 유효성 확인
def pwm_ready():
    vals = (CENTER, max_LEFT, max_RIGHT, STOP, max_GO, max_BACK)
    return not any(v <= 0 for v in vals)

log_filename = os.path.join(BASE_DIR, f"auto_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
log_file = open(log_filename, "w", encoding="utf-8", buffering=8192)
log_file.write("Time,Mode,Path,WP,Distance,Error,Lat,Lon,RTK,Speed_mps,PWM_Steer,PWM_Thr,"
               "CTE_m,Strategy,ThrottleBlend,HeadingSrc,WallGuard\n")
LOG_NAV_BLANK = ",,,,,"  # MANUAL/BALLOON rows leave the AUTO-only columns blank

# 범위 제한
def clamp(v, lo, hi):
    if lo > hi:
        lo, hi = hi, lo
    return max(lo, min(hi, v))

# 선형 보간
def lerp(a, b, t):
    t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
    return int(a + (b - a) * t)

# Logical PWM (web/controller scale) -> calibrated microseconds -> PCA9685 16-bit duty
def _us_to_duty(us):
    return int(round(clamp(us, 500.0, 2500.0) * PCA_FREQ_HZ * 65535.0 / 1e6))


def steer_us(v):
    """CENTER -> servo center; max_LEFT/max_RIGHT -> calibrated full left/right."""
    if v >= CENTER:
        t = min(1.0, (v - CENTER) / float(max_LEFT - CENTER))
        return HW["servo_center"] + t * (HW["servo_left"] - HW["servo_center"])
    t = min(1.0, (CENTER - v) / float(CENTER - max_RIGHT))
    return HW["servo_center"] + t * (HW["servo_right"] - HW["servo_center"])


def drive_us(v):
    """STOP -> ESC neutral; DEFAULT_MAX_GO (and above) -> forward max; max_BACK -> reverse max."""
    if v >= STOP:
        t = min(1.0, (v - STOP) / float(DEFAULT_MAX_GO - STOP))
        return HW["esc_neutral"] + t * (HW["esc_fwd"] - HW["esc_neutral"])
    t = min(1.0, (STOP - v) / float(STOP - max_BACK))
    return HW["esc_neutral"] + t * (HW["esc_rev"] - HW["esc_neutral"])


def steer_duty(v):
    return _us_to_duty(steer_us(v))


def drive_duty(v):
    return _us_to_duty(drive_us(v))


# RC pulses -> normalized sticks (-1..+1; channel reversal from calibration.json)
def rc_steer_norm(pulse):
    """-1 = full left, +1 = full right."""
    return normalize_axis(pulse, HW["st_neu"], HW["st_left"], HW["st_right"], HW["st_db"])


def rc_throttle_norm(pulse):
    """-1 = full reverse, +1 = full forward."""
    return normalize_axis(pulse, HW["th_neu"], HW["th_rev"], HW["th_fwd"], HW["th_db"])


def rc_to_logical(st_pulse, th_pulse):
    """MANUAL: transmitter sticks -> logical steer/drive (full forward stick = current max_GO)."""
    st_n = rc_steer_norm(st_pulse)
    th_n = rc_throttle_norm(th_pulse)
    if st_n >= 0.0:
        steer = CENTER + st_n * (max_RIGHT - CENTER)
    else:
        steer = CENTER + (-st_n) * (max_LEFT - CENTER)
    if th_n >= 0.0:
        drive = STOP + th_n * (max_GO - STOP)
    else:
        drive = STOP + (-th_n) * (max_BACK - STOP)
    return int(round(steer)), int(round(drive))


def rc_takeover_requested(mode, steer_pulse, throttle_pulse):
    """Any stick in MANUAL; full steering or reverse in autonomous modes."""
    steer = rc_steer_norm(steer_pulse)
    throttle = rc_throttle_norm(throttle_pulse)
    if mode == "MANUAL":
        return steer != 0.0 or throttle != 0.0
    return abs(steer) >= RC_OVERRIDE_STEER_RATIO or throttle <= -RC_OVERRIDE_REVERSE_RATIO


# 각도 정규화 (-180 ~ +180)
def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0

# 두 좌표 간 방위각(0~360) 계산
def gps_bearing(lat1, lon1, lat2, lon2):
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    x = math.sin(dl) * math.cos(p2)
    y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0

# 두 좌표 간 거리(m) 계산
def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return R * 2.0 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))

# 경로 CSV 로드
def load_csv_auto(filename):
    filepath = resolve_path(filename)
    with open(filepath, "r", encoding="utf-8-sig") as f:
        rows = [r for r in csv.reader(f) if any(cell.strip() for cell in r)]
    if not rows:
        raise ValueError(f"{filename} is empty.")

    first_row = [c.strip().lower() for c in rows[0]]
    has_header = False
    for c in first_row:
        try:
            float(c)
        except ValueError:
            has_header = True
            break

    data_rows = rows[1:] if has_header else rows
    lat_idx, lon_idx = None, None

    if has_header:
        for i, c in enumerate(first_row):
            if c in ("lat", "latitude") or "lat" in c:
                lat_idx = i
            elif c in ("lon", "lng", "long", "longitude") or "lon" in c or "lng" in c:
                lon_idx = i

    if lat_idx is None or lon_idx is None:
        sample = data_rows[0]
        for i, val in enumerate(sample):
            try:
                v = float(val)
                if lat_idx is None and 20.0 <= v <= 50.0:
                    lat_idx = i
                elif lon_idx is None and 110.0 <= v <= 150.0:
                    lon_idx = i
            except ValueError:
                continue

    if lat_idx is None or lon_idx is None:
        lat_idx, lon_idx = (1, 2) if len(data_rows[0]) >= 3 else (0, 1)

    coords = []
    for r in data_rows:
        try:
            coords.append((float(r[lat_idx]), float(r[lon_idx])))
        except (ValueError, IndexError):
            continue

    if not coords:
        raise ValueError(f"No valid coordinates in {filename}.")
    return coords

# 3중 경로 정렬 상태 검증
def verify_multipath_alignment(c_list, l_list, r_list):
    len_c, len_l, len_r = len(c_list), len(l_list), len(r_list)
    csv_ok = (len_c == len_l == len_r)
    n = min(len_c, len_l, len_r)
    for i in range(n):
        d_cl = haversine(c_list[i][0], c_list[i][1], l_list[i][0], l_list[i][1])
        d_cr = haversine(c_list[i][0], c_list[i][1], r_list[i][0], r_list[i][1])
        if d_cl > 15.0 or d_cr > 15.0:
            csv_ok = False
            break
    shared_state["lampCsv"] = "ok" if csv_ok else "warn"
    return n

try:
    path_center = load_csv_auto("center.csv")
    path_left = load_csv_auto("left.csv")
    path_right = load_csv_auto("right.csv")
    PATHS = {"Center": path_center, "Left": path_left, "Right": path_right}
    total_waypoints = verify_multipath_alignment(path_center, path_left, path_right)
    shared_state["wpTotal"] = total_waypoints
    shared_state["lat"] = path_center[0][0]
    shared_state["lon"] = path_center[0][1]
    PATHS_JSON_BYTES = json.dumps({
        "Center": [[lat, lon] for lat, lon in path_center],
        "Left": [[lat, lon] for lat, lon in path_left],
        "Right": [[lat, lon] for lat, lon in path_right],
    }).encode("utf-8")
except Exception as e:
    raise SystemExit(f"CSV load error: {e}")

# ---------------------------------------------------------------------------
# AUTO controller (shared with multipath_boat.py). Uses the autonomous / navigation /
# pwm.auto_throttle_ratio settings in calibration.json; without that file the defaults
# apply (02_adaptive_throttle, cruise ratio 0.60). Steering [-1, 1] maps to
# CENTER..max_LEFT/max_RIGHT; throttle blend [0, 1] maps from cruise (that ratio of
# STOP..max_GO) up to max_GO.
# ---------------------------------------------------------------------------
AUTO_CONFIG_FILE = "calibration.json"
LANE_KEYS = {"Left": "left", "Center": "center", "Right": "right"}
LANE_NAMES = {v: k for k, v in LANE_KEYS.items()}


def load_auto_controller():
    cfg = {}
    cfg_path = resolve_path(AUTO_CONFIG_FILE)
    found = os.path.exists(cfg_path)
    if found:
        with open(cfg_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
    nav_params = resolve_navigation_params(cfg.get("navigation", {}))
    strategy = create_auto_strategy(cfg.get("autonomous", {}), nav_params)
    ratio = cfg.get("pwm", {}).get("auto_throttle_ratio", 0.60)
    if isinstance(ratio, bool) or not isinstance(ratio, (int, float)) or not math.isfinite(ratio):
        raise ValueError("pwm.auto_throttle_ratio must be a finite number")
    nav_paths = {
        LANE_KEYS[name]: [(i + 1, lat, lon) for i, (lat, lon) in enumerate(pts)]
        for name, pts in PATHS.items()
    }
    navigator = MultipathNavigator(nav_paths, nav_params, strategy=strategy)
    selector = MotionHeadingSelector(strategy.options) if strategy.uses_gps_course else None
    return navigator, strategy, selector, clamp(float(ratio), 0.20, 1.00), found


try:
    NAVIGATOR, AUTO_STRATEGY, HEADING_SELECTOR, AUTO_CRUISE_RATIO, AUTO_CONFIG_FOUND = load_auto_controller()
except Exception as e:
    raise SystemExit(f"AUTO controller config error ({AUTO_CONFIG_FILE}): {e}")

# Parser copy for the controller: read_latest_gps() hands it every NMEA sentence (boat.py owns the port)
NAV_GPS = ZEDF9PReceiver(port=None)


def nav_steer_to_pwm(steer_norm):
    """+1 = full right rudder; STEER_SIGN = -1 swaps sides."""
    n = clamp(float(steer_norm), -1.0, 1.0) * STEER_SIGN
    if n >= 0.0:
        raw = CENTER + n * (max_RIGHT - CENTER)
    else:
        raw = CENTER + (-n) * (max_LEFT - CENTER)
    return int(round(clamp(raw, max_LEFT, max_RIGHT)))


def nav_throttle_to_pwm(blend):
    """Blend 0 = cruise, 1 = max_GO (the web max-forward setting)."""
    cruise = lerp(STOP, max_GO, AUTO_CRUISE_RATIO)
    b = clamp(float(blend), 0.0, 1.0)
    return int(round(cruise + b * (max_GO - cruise)))

# NMEA XOR 체크섬 검증
def verify_nmea_checksum(line):
    if not line or not line.startswith("$") or "*" not in line:
        return False
    body, _, chk_str = line[1:].partition("*")
    chk_str = chk_str.strip()[:2]
    if len(chk_str) != 2:
        return False
    try:
        expected = int(chk_str, 16)
    except ValueError:
        return False
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    return calc == expected

# NMEA GGA 파싱
def parse_gga(line):
    if not (line.startswith("$GNGGA") or line.startswith("$GPGGA")) or not verify_nmea_checksum(line):
        return None
    parts = line.split(",")
    if len(parts) < 10 or not parts[6]:
        return None
    try:
        fix = int(parts[6])
        sats = int(parts[7]) if parts[7] else 0
        hdop = float(parts[8]) if parts[8] else 99.9
        if not parts[2] or not parts[4]:
            return None, None, fix, sats, hdop
        lat = float(parts[2][:2]) + float(parts[2][2:]) / 60.0
        if parts[3] == "S":
            lat = -lat
        lon = float(parts[4][:3]) + float(parts[4][3:]) / 60.0
        if parts[5] == "W":
            lon = -lon
        return lat, lon, fix, sats, hdop
    except (ValueError, IndexError):
        return None

# NMEA RMC 파싱
def parse_rmc(line):
    if not (line.startswith("$GNRMC") or line.startswith("$GPRMC")) or not verify_nmea_checksum(line):
        return None
    parts = line.split(",")
    if len(parts) < 10 or parts[2] != "A" or not parts[3] or not parts[5]:
        return None
    try:
        lat = float(parts[3][:2]) + float(parts[3][2:]) / 60.0
        if parts[4] == "S":
            lat = -lat
        lon = float(parts[5][:3]) + float(parts[5][3:]) / 60.0
        if parts[6] == "W":
            lon = -lon
        cog = float(parts[8]) if parts[8] else None
        spd_kn = float(parts[7]) if parts[7] else 0.0
        return lat, lon, cog, spd_kn
    except (ValueError, IndexError):
        return None

# 아두이노 RC 펄스 파싱
def get_arduino_data(data_str):
    if not data_str:
        return None, None
    try:
        s = data_str.replace("ST:", "").replace("TH:", "")
        parts = s.split(",")
        if len(parts) >= 2:
            st = int(float(parts[0].strip()))
            th = int(float(parts[1].strip()))
            if not (RC_VALID_MIN <= st <= RC_VALID_MAX and RC_VALID_MIN <= th <= RC_VALID_MAX):
                return None, None
            return st, th
    except (ValueError, TypeError):
        pass
    return None, None

_last_rc_reconnect_t = 0.0
_last_gps_reconnect_t = 0.0
_rc_read_count = 0
_last_rc_valid_pkt_t = 0.0
_rc_silent_reset_count = 0

# 최신 RC 신호 읽기 및 무응답 시 자동 복구
def read_latest_rc():
    global ser, _last_rc_reconnect_t, _rc_read_count, _last_rc_valid_pkt_t, _rc_silent_reset_count
    now = time.time()
    if ser is None:
        if now - _last_rc_reconnect_t >= SERIAL_RECONNECT_SEC:
            _last_rc_reconnect_t = now
            ser = open_rc_serial()
            _last_rc_valid_pkt_t = now
        return None, None

    lines = []
    lines_read = 0
    try:
        while ser.in_waiting > 0 and lines_read < MAX_SERIAL_LINES_LOOP:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            lines_read += 1
            if line:
                lines.append(line)
    except SERIAL_ERR_TYPES as e:
        add_event_log("SERIAL", f"RC serial disconnect detected ({e})", "warn")
        shared_state["lampSerial"] = "err"
        try:
            ser.close()
        except Exception:
            pass
        ser = None
        return None, None

    for candidate in reversed(lines):
        st, th = get_arduino_data(candidate)
        if st is not None and th is not None:
            _rc_read_count += 1
            _last_rc_valid_pkt_t = now
            _rc_silent_reset_count = 0
            return st, th

    # 유효 패킷 장기 미수신 시 백오프 기반 버퍼 정리 (조종기 OFF 시 무한 리셋·로그 스팸 방지)
    if _last_rc_valid_pkt_t == 0.0:
        _last_rc_valid_pkt_t = now
    else:
        backoff_sec = 6.0 if _rc_silent_reset_count == 0 else (20.0 if _rc_silent_reset_count == 1 else 60.0)
        if (now - _last_rc_valid_pkt_t) >= backoff_sec:
            _last_rc_valid_pkt_t = now
            _rc_silent_reset_count += 1
            if _rc_silent_reset_count <= 2:
                cur_port = getattr(ser, "port", RC_PORT)
                add_event_log("SERIAL", f"RC silent ({int(backoff_sec)}s) -> flush serial buffer #{_rc_silent_reset_count}", "warn")
                try:
                    ser.reset_input_buffer()
                    ser.close()
                except Exception:
                    pass
                ser = None

    return None, None

_last_lat, _last_lon, _last_cog, _last_spd_kn = None, None, None, 0.0
_last_fix, _last_sats, _last_hdop = 0, 0, 99.9
_last_gga_line = None
_last_valid_gps_t = 0.0
_glitch_reject_count = 0
_glitch_cand_lat, _glitch_cand_lon = None, None

# 최신 GPS 읽기 및 Glitch 필터링
def read_latest_gps(is_reversing=False):
    global gps_ser, _last_gps_reconnect_t
    global _last_lat, _last_lon, _last_cog, _last_spd_kn, _last_fix, _last_sats, _last_hdop
    global _last_gga_line, _last_valid_gps_t, _glitch_reject_count, _glitch_cand_lat, _glitch_cand_lon

    now = time.time()
    if gps_ser is None:
        if now - _last_gps_reconnect_t >= SERIAL_RECONNECT_SEC:
            _last_gps_reconnect_t = now
            with gps_ser_lock:
                gps_ser = open_gps_serial()
        return None

    updated = False
    cand_lat, cand_lon = None, None
    cand_cog = None
    cand_spd_kn = _last_spd_kn
    cand_fix, cand_sats, cand_hdop = _last_fix, _last_sats, _last_hdop

    lines_read = 0
    try:
        with gps_ser_lock:
            while gps_ser is not None and gps_ser.in_waiting > 0 and lines_read < MAX_SERIAL_LINES_LOOP:
                line = gps_ser.readline().decode("ascii", errors="ignore").strip()
                lines_read += 1
                if line.startswith(("$GNGGA", "$GPGGA", "$GNRMC", "$GPRMC")):
                    try:
                        NAV_GPS.ingest_nmea_line(line)
                    except (ValueError, IndexError):
                        pass
                if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
                    gga = parse_gga(line)
                    if gga:
                        g_lat, g_lon, cand_fix, cand_sats, cand_hdop = gga
                        _last_fix, _last_sats, _last_hdop = cand_fix, cand_sats, cand_hdop
                        if g_lat is not None and g_lon is not None:
                            cand_lat, cand_lon = g_lat, g_lon
                            _last_gga_line = line
                            updated = True
                elif line.startswith("$GNRMC") or line.startswith("$GPRMC"):
                    rmc = parse_rmc(line)
                    if rmc:
                        r_lat, r_lon, r_cog, r_spd = rmc
                        cand_lat, cand_lon = r_lat, r_lon
                        if r_cog is not None:
                            cand_cog = r_cog
                        cand_spd_kn = r_spd
                        updated = True
    except SERIAL_ERR_TYPES as e:
        add_event_log("SERIAL", f"GPS serial disconnect detected ({e})", "warn")
        shared_state["lampSerial"] = "err"
        with gps_ser_lock:
            try:
                gps_ser.close()
            except Exception:
                pass
            gps_ser = None
        return None

    if updated and cand_lat is not None and cand_lon is not None:
        if _last_lat is not None and _last_lon is not None and _last_valid_gps_t > 0:
            dt = max(0.1, now - _last_valid_gps_t)
            jump_dist = haversine(_last_lat, _last_lon, cand_lat, cand_lon)
            jump_mps = jump_dist / dt
            if jump_dist > MAX_GPS_GLITCH_DIST_M and jump_mps > MAX_GPS_GLITCH_MPS:
                # 연속 이상치 일관성 확인
                consistent_with_cand = (
                    _glitch_cand_lat is not None
                    and haversine(_glitch_cand_lat, _glitch_cand_lon, cand_lat, cand_lon) <= MAX_GPS_GLITCH_DIST_M
                )
                _glitch_cand_lat, _glitch_cand_lon = cand_lat, cand_lon
                if _glitch_reject_count < 3 or (not consistent_with_cand and _glitch_reject_count < 8):
                    _glitch_reject_count += 1
                    shared_state["lampGlitch"] = "warn"
                    add_event_log("GLITCH", f"Reject wall-reflection GPS glitch #{_glitch_reject_count} ({jump_dist:.1f}m / {jump_mps:.1f}m/s)", "warn")
                    return None

        _glitch_reject_count = 0
        _glitch_cand_lat, _glitch_cand_lon = None, None
        shared_state["lampGlitch"] = "ok"
        _last_lat, _last_lon = cand_lat, cand_lon
        _last_spd_kn = cand_spd_kn

        if is_reversing:
            _last_cog = None
        elif cand_cog is not None:
            _last_cog = cand_cog

        _last_valid_gps_t = now
        return _last_lat, _last_lon, _last_cog, _last_spd_kn, _last_fix, _last_sats, _last_hdop

    return None

# NTRIP RTCM3 보정 신호 수신 워커
def ntrip_worker():
    auth = base64.b64encode(f"{NTRIP_USER}:{NTRIP_PASS}".encode("utf-8")).decode("utf-8")
    req_header = (
        f"GET /{NTRIP_MOUNT} HTTP/1.0\r\n"
        f"User-Agent: NTRIP PythonClient/1.0\r\n"
        f"Accept: */*\r\n"
        f"Connection: close\r\n"
        f"Authorization: Basic {auth}\r\n"
        f"\r\n"
    ).encode("utf-8")

    total_bytes = 0

    while True:
        if not NTRIP_ENABLE or NTRIP_USER in ("", "YOUR_ID") or _last_gga_line is None:
            time.sleep(1.0)
            continue

        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((NTRIP_HOST, NTRIP_PORT))
            sock.sendall(req_header)

            resp = sock.recv(4096)
            resp_text = resp.decode("latin1", errors="ignore")

            if "200 OK" not in resp_text:
                shared_state["ntripConnected"] = False
                sock.close()
                time.sleep(5.0)
                continue

            shared_state["ntripConnected"] = True
            add_event_log("NTRIP", f"RTCM3 corrections started ({NTRIP_MOUNT})", "ntrip")

            if b"\r\n\r\n" in resp:
                body = resp.split(b"\r\n\r\n", 1)[1]
                if body:
                    with gps_ser_lock:
                        if gps_ser is not None:
                            try:
                                gps_ser.write(body)
                                total_bytes += len(body)
                            except Exception:
                                pass

            sock.settimeout(1.0)
            last_gga_sent = 0.0
            last_rtcm_t = time.time()

            while True:
                now_t = time.time()
                gga_to_send = _last_gga_line
                if gga_to_send and (now_t - last_gga_sent >= 5.0):
                    sock.sendall((gga_to_send + "\r\n").encode("ascii"))
                    last_gga_sent = now_t

                try:
                    data = sock.recv(4096)
                    if not data:
                        shared_state["ntripConnected"] = False
                        break
                    with gps_ser_lock:
                        if gps_ser is not None:
                            try:
                                gps_ser.write(data)
                                total_bytes += len(data)
                            except Exception:
                                pass
                    shared_state["ntripKb"] = round(total_bytes / 1024.0, 1)
                    last_rtcm_t = now_t
                except socket.timeout:
                    if now_t - last_rtcm_t > 15.0:
                        shared_state["ntripConnected"] = False
                        break
                    continue
        except Exception:
            shared_state["ntripConnected"] = False
        finally:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass
        time.sleep(3.0)

_last_steer = None
_last_drive = None
_need_safe_stop_retry = False
_pca_err_streak = 0

# 서보·추진기 PWM 출력 (Slew-Rate 제한)
def set_pwm(steer, drive, force=False):
    global pca, _last_steer, _last_drive, _need_safe_stop_retry, _pca_err_streak
    steer = int(clamp(steer, min(max_LEFT, max_RIGHT), max(max_LEFT, max_RIGHT)))
    target_drive = int(clamp(drive, min(max_BACK, max_GO), max(max_BACK, max_GO)))

    if target_drive != STOP and _last_drive is not None and not force:
        diff = target_drive - _last_drive
        if abs(diff) > MAX_DRIVE_SLEW_STEP:
            target_drive = _last_drive + (MAX_DRIVE_SLEW_STEP if diff > 0 else -MAX_DRIVE_SLEW_STEP)

    steer_need = (
        force or _need_safe_stop_retry or _last_steer is None
        or abs(steer - _last_steer) >= PWM_WRITE_MIN_DELTA
        or (steer in (CENTER, max_LEFT, max_RIGHT) and _last_steer != steer)
    )
    drive_need = (
        force or _need_safe_stop_retry or _last_drive is None
        or abs(target_drive - _last_drive) >= PWM_WRITE_MIN_DELTA
        or (target_drive in (STOP, max_GO, max_BACK) and _last_drive != target_drive)
    )

    if pca is None:
        _last_steer = _last_drive = None
        _need_safe_stop_retry = True
        shared_state["lampSerial"] = "err"
        return

    try:
        if steer_need:
            pca.set_pwm_us(SERVO_CH, steer_us(steer))
            _last_steer = steer
        if drive_need:
            pca.set_pwm_us(ESC_CH, drive_us(target_drive))
            _last_drive = target_drive
        _need_safe_stop_retry = False
        _pca_err_streak = 0
        shared_state["steerPwm"] = _last_steer if _last_steer is not None else steer
        shared_state["drivePwm"] = _last_drive if _last_drive is not None else target_drive
    except Exception:
        _pca_err_streak += 1
        shared_state["lampSerial"] = "warn"
        if _pca_err_streak >= 5:
            shared_state["lampSerial"] = "err"
            pca = None
            _pca_err_streak = 0

# 모터 즉시 정지 (중립/정지 3회 재시도)
def safe_stop():
    global pca, _last_steer, _last_drive, _need_safe_stop_retry
    shared_state["steerPwm"] = CENTER
    shared_state["drivePwm"] = STOP
    if pca is None:
        _last_steer, _last_drive = CENTER, STOP
        return

    for _ in range(3):
        try:
            pca.set_pwm_us(SERVO_CH, steer_us(CENTER))
            pca.set_pwm_us(ESC_CH, drive_us(STOP))
            _last_steer, _last_drive = CENTER, STOP
            _need_safe_stop_retry = False
            return
        except Exception:
            time.sleep(0.005)

    _last_steer = None
    _last_drive = None
    _need_safe_stop_retry = True

_pid_integral = 0.0
_pid_prev_err = None
_pid_prev_t = 0.0
_pid_prev_err_t = 0.0
_pid_d_filt = 0.0
_pid_last_p = 0.0
_pid_last_i = 0.0
_pid_last_d = 0.0

# control/비전 설정 파일 저장
def save_pid_config():
    try:
        cfg = {
            "mode": STEER_CTRL_MODE,
            "kp": PID_KP,
            "ki": PID_KI,
            "kd": PID_KD,
            "imax": PID_I_MAX_DEG,
            "maxGo": min(int(max_GO), 5100),
            "balloonActionMode": BALLOON_ACTION_MODE,
            "balloonRealDiamM": BALLOON_REAL_DIAM_M,
            "balloonStopDistM": BALLOON_STOP_DIST_M,
            "balloonStopAreaPct": BALLOON_STOP_AREA_PCT,
            "customColorEnable": CUSTOM_COLOR_ENABLE,
            "targetHue": TARGET_HUE,
            "targetHueTol": TARGET_HUE_TOL,
            "targetSatMin": TARGET_SAT_MIN,
            "targetValMin": TARGET_VAL_MIN,
            "targetColorHex": TARGET_COLOR_HEX,
            "camLteMode": _cam_lte_mode,
            "camFlip": 180 if _cam_flip_method == 2 else 0,
            "linkWatchdog": shared_state.get("linkWatchdog", True),
            "linkTimeoutSec": GCS_LINK_TIMEOUT_SEC,
        }
        with open(PID_CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2, ensure_ascii=False)
    except Exception as e:
        add_event_log("CTRL", f"Failed to save settings file ({e})", "warn")

# 저장된 control/비전 설정 로드
def load_pid_config():
    global STEER_CTRL_MODE, PID_KP, PID_KI, PID_KD, PID_I_MAX_DEG, STEER_GAIN
    global BALLOON_ACTION_MODE, BALLOON_REAL_DIAM_M, BALLOON_STOP_DIST_M, BALLOON_STOP_AREA_PCT, max_GO
    global CUSTOM_COLOR_ENABLE, TARGET_HUE, TARGET_HUE_TOL, TARGET_SAT_MIN, TARGET_VAL_MIN, TARGET_COLOR_HEX
    global _cam_lte_mode, _cam_flip_method, GCS_LINK_TIMEOUT_SEC
    max_GO = DEFAULT_MAX_GO
    if not os.path.exists(PID_CONFIG_FILE):
        return
    try:
        with open(PID_CONFIG_FILE, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        if cfg.get("mode") in ("P", "PID"):
            STEER_CTRL_MODE = cfg["mode"]
        if "kp" in cfg:
            PID_KP = round(clamp(float(cfg["kp"]), 1.0, 45.0), 2)
            STEER_GAIN = PID_KP
        if "ki" in cfg:
            PID_KI = round(clamp(float(cfg["ki"]), 0.0, 10.0), 3)
        if "kd" in cfg:
            PID_KD = round(clamp(float(cfg["kd"]), 0.0, 25.0), 2)
        if "imax" in cfg:
            PID_I_MAX_DEG = round(clamp(float(cfg["imax"]), 2.0, 40.0), 1)
        if "maxGo" in cfg:
            saved_mg = int(float(cfg["maxGo"]))
            if saved_mg in MAX_GO_PRESETS and saved_mg < 5200:
                max_GO = saved_mg
            else:
                max_GO = DEFAULT_MAX_GO
        if cfg.get("balloonActionMode") in ("APPROACH_STOP", "RAM_POP"):
            BALLOON_ACTION_MODE = cfg["balloonActionMode"]
        if "balloonRealDiamM" in cfg:
            BALLOON_REAL_DIAM_M = round(clamp(float(cfg["balloonRealDiamM"]), 0.2, 8.0), 2)
        if "balloonStopDistM" in cfg:
            BALLOON_STOP_DIST_M = round(clamp(float(cfg["balloonStopDistM"]), 0.8, 15.0), 2)
        if "balloonStopAreaPct" in cfg:
            BALLOON_STOP_AREA_PCT = round(clamp(float(cfg["balloonStopAreaPct"]), 5.0, 80.0), 1)
        if "customColorEnable" in cfg:
            CUSTOM_COLOR_ENABLE = bool(cfg["customColorEnable"])
        if "targetHue" in cfg:
            TARGET_HUE = int(clamp(int(float(cfg["targetHue"])), 0, 179))
        if "targetHueTol" in cfg:
            TARGET_HUE_TOL = int(clamp(int(float(cfg["targetHueTol"])), 3, 40))
        if "targetSatMin" in cfg:
            TARGET_SAT_MIN = int(clamp(int(float(cfg["targetSatMin"])), 25, 245))
        if "targetValMin" in cfg:
            TARGET_VAL_MIN = int(clamp(int(float(cfg["targetValMin"])), 25, 245))
        if isinstance(cfg.get("targetColorHex"), str) and cfg["targetColorHex"].startswith("#"):
            TARGET_COLOR_HEX = cfg["targetColorHex"][:7]
        if "camLteMode" in cfg:
            _cam_lte_mode = bool(cfg["camLteMode"])
        if "camFlip" in cfg:
            _cam_flip_method = 2 if int(cfg["camFlip"]) == 180 else 0
        if "linkTimeoutSec" in cfg:
            GCS_LINK_TIMEOUT_SEC = round(clamp(float(cfg["linkTimeoutSec"]), 2.0, 10.0), 1)
        loaded_watchdog = bool(cfg.get("linkWatchdog", True))
        with state_lock:
            shared_state["camLteMode"] = _cam_lte_mode
            shared_state["camFlip"] = 180 if _cam_flip_method == 2 else 0
            shared_state["linkWatchdog"] = loaded_watchdog
            shared_state["linkTimeoutSec"] = GCS_LINK_TIMEOUT_SEC
            shared_state["customColorEnable"] = CUSTOM_COLOR_ENABLE
            shared_state["targetHue"] = TARGET_HUE
            shared_state["targetHueTol"] = TARGET_HUE_TOL
            shared_state["targetSatMin"] = TARGET_SAT_MIN
            shared_state["targetValMin"] = TARGET_VAL_MIN
            shared_state["targetColorHex"] = TARGET_COLOR_HEX
            shared_state["steerCtrlMode"] = STEER_CTRL_MODE
            shared_state["pidKp"] = PID_KP
            shared_state["pidKi"] = PID_KI
            shared_state["pidKd"] = PID_KD
            shared_state["pidIMax"] = PID_I_MAX_DEG
            shared_state["maxGo"] = max_GO
            shared_state["highPowerElapsedSec"] = 0.0
            shared_state["balloonActionMode"] = BALLOON_ACTION_MODE
            shared_state["balloonRealDiamM"] = BALLOON_REAL_DIAM_M
            shared_state["balloonStopDistM"] = BALLOON_STOP_DIST_M
            shared_state["balloonStopAreaPct"] = BALLOON_STOP_AREA_PCT
            shared_state["customColorEnable"] = CUSTOM_COLOR_ENABLE
            shared_state["targetHue"] = TARGET_HUE
            shared_state["targetHueTol"] = TARGET_HUE_TOL
            shared_state["targetSatMin"] = TARGET_SAT_MIN
            shared_state["targetValMin"] = TARGET_VAL_MIN
            shared_state["targetColorHex"] = TARGET_COLOR_HEX
        add_event_log(
            "CTRL",
            f"Loaded saved settings [{STEER_CTRL_MODE}] Kp={PID_KP}, max_GO={max_GO}, target={BALLOON_REAL_DIAM_M}m ({BALLOON_ACTION_MODE}, stop={BALLOON_STOP_DIST_M}m)",
            "auto",
        )
    except Exception as e:
        add_event_log("CTRL", f"Skipped loading settings file ({e})", "warn")

# PID 내부 상태 초기화
def reset_pid_state():
    global _pid_integral, _pid_prev_err, _pid_prev_t, _pid_prev_err_t, _pid_d_filt
    global _pid_last_p, _pid_last_i, _pid_last_d
    _pid_integral = 0.0
    _pid_prev_err = None
    _pid_prev_t = 0.0
    _pid_prev_err_t = 0.0
    _pid_d_filt = 0.0
    _pid_last_p = 0.0
    _pid_last_i = 0.0
    _pid_last_d = 0.0
    with state_lock:
        shared_state["pidPTerm"] = 0.0
        shared_state["pidITerm"] = 0.0
        shared_state["pidDTerm"] = 0.0

# WP 좌표 조회
def wp_coord(path_list, idx):
    return path_list[idx]

# 최적 경로(L/C/R) 선택
def pick_path(c_lat, c_lon, idx, current_name):
    dists = {}
    for name, path_list in PATHS.items():
        lat, lon = path_list[idx]
        dists[name] = haversine(c_lat, c_lon, lat, lon)

    cur_d = dists[current_name]
    best_name = min(dists, key=dists.get)
    best_d = dists[best_name]

    if cur_d > SWITCH_DIST_M and (cur_d - best_d) >= SWITCH_MARGIN_M:
        return best_name, dists
    return current_name, dists

# 전방 주시 좌표(Lookahead) 계산
def get_lookahead_target(c_lat, c_lon, path_list, idx):
    if idx >= len(path_list):
        return path_list[-1]
    t_lat, t_lon = path_list[idx]
    d_cur = haversine(c_lat, c_lon, t_lat, t_lon)
    if d_cur >= LOOKAHEAD_MIN_M or idx + 1 >= len(path_list):
        return t_lat, t_lon

    curr_i = idx
    while curr_i + 1 < len(path_list):
        p1_lat, p1_lon = path_list[curr_i]
        p2_lat, p2_lon = path_list[curr_i + 1]
        d2 = haversine(c_lat, c_lon, p2_lat, p2_lon)
        if d2 >= LOOKAHEAD_MIN_M:
            d1 = haversine(c_lat, c_lon, p1_lat, p1_lon)
            seg = max(0.05, d2 - d1)
            ratio = clamp((LOOKAHEAD_MIN_M - d1) / seg, 0.0, 1.0)
            return (
                p1_lat + (p2_lat - p1_lat) * ratio,
                p1_lon + (p2_lon - p1_lon) * ratio,
            )
        curr_i += 1
    return path_list[-1]

# 시스템 온도·부하 조회
def read_jetson_health():
    temp_c = 42.0
    load_pct = 15.0
    try:
        if os.path.exists("/sys/class/thermal/thermal_zone0/temp"):
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp_c = float(f.read().strip()) / 1000.0
        if os.path.exists("/proc/loadavg"):
            with open("/proc/loadavg", "r") as f:
                load1 = float(f.read().split()[0])
                load_pct = min(100.0, round(load1 * 25.0, 1))
    except Exception:
        pass
    return round(temp_c, 1), round(load_pct, 1)

_cam_lock = threading.Lock()
_active_cam_obj = None
_latest_bgr_frame = None
_latest_jpeg_bytes = None
_latest_jpeg_seq = 0
_cam_lte_mode = True
_last_jpeg_encode_t = 0.0
_active_stream_clients = 0
_stream_generation = 0
_last_cam_http_req_t = 0.0

# OpenCV HSV -> HEX 변환
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
    return f"#{clamp(r,0,255):02x}{clamp(g,0,255):02x}{clamp(b,0,255):02x}"

# HEX -> OpenCV HSV 변환
def hex_to_hsv_cv(hex_str):
    s = str(hex_str).strip().lstrip("#")
    if len(s) != 6:
        return 166, 200, 255
    r = int(s[0:2], 16) / 255.0
    g = int(s[2:4], 16) / 255.0
    b = int(s[4:6], 16) / 255.0
    mx = max(r, g, b)
    mn = min(r, g, b)
    diff = mx - mn
    if diff == 0:
        h_deg = 0.0
    elif mx == r:
        h_deg = (60.0 * ((g - b) / diff) + 360.0) % 360.0
    elif mx == g:
        h_deg = (60.0 * ((b - r) / diff) + 120.0) % 360.0
    else:
        h_deg = (60.0 * ((r - g) / diff) + 240.0) % 360.0
    sat = 0 if mx == 0 else int(round((diff / mx) * 255.0))
    val = int(round(mx * 255.0))
    return int(round(h_deg / 2.0)) % 180, sat, val

# 타겟 색상 HSV 마스크 생성
def build_target_hsv_mask(hsv_img, preset_name="default"):
    if not CUSTOM_COLOR_ENABLE:
        preset = HSV_PRESETS.get(preset_name, HSV_PRESETS["default"])
        m1 = cv2.inRange(hsv_img, np.array(preset["lower1"], dtype=np.uint8), np.array(preset["upper1"], dtype=np.uint8))
        m2 = cv2.inRange(hsv_img, np.array(preset["lower2"], dtype=np.uint8), np.array(preset["upper2"], dtype=np.uint8))
        return cv2.bitwise_or(m1, m2)

    h_center = int(clamp(TARGET_HUE, 0, 179))
    h_tol = int(clamp(TARGET_HUE_TOL, 3, 40))
    s_min = int(clamp(TARGET_SAT_MIN, 25, 245))
    v_min = int(clamp(TARGET_VAL_MIN, 25, 245))

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
    else:
        return cv2.inRange(hsv_img, np.array([low_h, s_min, v_min], dtype=np.uint8), np.array([high_h, 255, 255], dtype=np.uint8))

# 영상 클릭 지점 색상 추출
def sample_color_from_latest_frame(u_norm, v_norm):
    global CUSTOM_COLOR_ENABLE, TARGET_HUE, TARGET_HUE_TOL, TARGET_SAT_MIN, TARGET_VAL_MIN, TARGET_COLOR_HEX
    with _cam_lock:
        frame = None if _latest_bgr_frame is None else _latest_bgr_frame.copy()
    if frame is None or not CV_AVAILABLE:
        return False, "Camera frame is not ready."

    h, w = frame.shape[:2]
    cx = int(clamp(round(float(u_norm) * w), 0, w - 1))
    cy = int(clamp(round(float(v_norm) * h), 0, h - 1))
    r = 5
    x0, x1 = max(0, cx - r), min(w, cx + r + 1)
    y0, y1 = max(0, cy - r), min(h, cy + r + 1)
    patch_bgr = frame[y0:y1, x0:x1]
    patch_hsv = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2HSV)

    # 패치 중앙값 HSV 계산
    med_s = int(np.median(patch_hsv[:, :, 1]))
    med_v = int(np.median(patch_hsv[:, :, 2]))
    angles = patch_hsv[:, :, 0].astype(np.float32) * (2.0 * math.pi / 180.0)
    mean_ang = math.atan2(float(np.mean(np.sin(angles))), float(np.mean(np.cos(angles))))
    if mean_ang < 0:
        mean_ang += 2.0 * math.pi
    med_h = int(round(mean_ang * 180.0 / (2.0 * math.pi))) % 180

    CUSTOM_COLOR_ENABLE = True
    TARGET_HUE = med_h
    TARGET_SAT_MIN = int(clamp(med_s - 45, 55, 225))
    TARGET_VAL_MIN = int(clamp(med_v - 55, 45, 225))
    TARGET_COLOR_HEX = hsv_to_hex(med_h, max(140, med_s), max(160, med_v))

    with state_lock:
        shared_state["customColorEnable"] = CUSTOM_COLOR_ENABLE
        shared_state["targetHue"] = TARGET_HUE
        shared_state["targetHueTol"] = TARGET_HUE_TOL
        shared_state["targetSatMin"] = TARGET_SAT_MIN
        shared_state["targetValMin"] = TARGET_VAL_MIN
        shared_state["targetColorHex"] = TARGET_COLOR_HEX

    save_pid_config()
    return True, f"Click color sample complete (H:{TARGET_HUE} ±{TARGET_HUE_TOL}, S≥{TARGET_SAT_MIN}, V≥{TARGET_VAL_MIN}, {TARGET_COLOR_HEX})"
_cam_reopen_requested = False
_cam_flip_method = CAM_FLIP_METHOD
_cam_show_mask = False
_hsv_preset_name = "default"

_vision_state = {
    "ok": False,
    "last_frame_t": 0.0,
    "fps": 0.0,
    "detected": False,
    "norm_x": 0.0,
    "norm_y": 0.0,
    "radius": 0.0,
    "area_pct": 0.0,
    "dist_m": 0.0,
    "angle_err": 0.0,
    "d_angle_err": 0.0,
}

# CSI 카메라 GStreamer 파이프라인 (16:9)
def build_csi_gstreamer_pipeline(
    sensor_id=0,
    sensor_w=CAM_SENSOR_WIDTH,
    sensor_h=CAM_SENSOR_HEIGHT,
    out_w=CAM_WIDTH,
    out_h=CAM_HEIGHT,
    fps=CAM_FPS,
    flip_method=0,
):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){sensor_w}, height=(int){sensor_h}, "
        f"format=(string)NV12, framerate=(fraction){fps}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){out_w}, height=(int){out_h}, format=(string)BGRx ! "
        f"videoconvert ! video/x-raw, format=(string)BGR ! "
        f"appsink drop=1 max-buffers=1"
    )

# gst-launch 서브프로세스 캡처 래퍼
class GstPipeCapture:
    def __init__(self, sensor_id=0, width=CAM_WIDTH, height=CAM_HEIGHT, fps=CAM_FPS, flip_method=0):
        self.width = width
        self.height = height
        self.frame_bytes = width * height * 3
        cmd = [
            "gst-launch-1.0", "-q",
            "nvarguscamerasrc", f"sensor-id={sensor_id}",
            "!", f"video/x-raw(memory:NVMM),width={CAM_SENSOR_WIDTH},height={CAM_SENSOR_HEIGHT},format=NV12,framerate={fps}/1",
            "!", "nvvidconv", f"flip-method={flip_method}",
            "!", f"video/x-raw,width={width},height={height},format=BGRx",
            "!", "videoconvert",
            "!", "video/x-raw,format=BGR",
            "!", "fdsink", "fd=1",
        ]
        self.proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=self.frame_bytes * 2,
        )

    def isOpened(self):
        return self.proc is not None and self.proc.poll() is None

    def read(self):
        if not self.isOpened() or self.proc.stdout is None:
            return False, None
        raw = self.proc.stdout.read(self.frame_bytes)
        if not raw or len(raw) != self.frame_bytes:
            return False, None
        frame = np.frombuffer(raw, dtype=np.uint8).reshape((self.height, self.width, 3)).copy()
        return True, frame

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

def _v4l2_device_name(dev_path):
    """Return the sysfs name for a /dev/videoN node, or ''."""
    try:
        base = os.path.basename(str(dev_path))
        with open(f"/sys/class/video4linux/{base}/name", "r") as f:
            return f.read().strip()
    except Exception:
        return ""


def _is_csi_v4l2_node(dev_path):
    """Jetson CSI ribbon cameras show up as vi-output-* and need nvargus, not raw V4L2."""
    return _v4l2_device_name(dev_path).startswith("vi-output")


def _frame_looks_valid_bgr(frame):
    """Reject empty / solid-green CSI garbage that raw V4L2 often returns on Jetson."""
    if frame is None or getattr(frame, "size", 0) <= 0:
        return False
    if len(frame.shape) != 3 or frame.shape[2] != 3:
        return False
    # Sample a coarse grid; pure Argus/V4L2 green dump is ~G=255, R≈B≈0.
    sample = frame[::16, ::16]
    mean_b, mean_g, mean_r = [float(x) for x in sample.mean(axis=(0, 1))]
    if mean_g > 200.0 and mean_r < 40.0 and mean_b < 40.0:
        return False
    return True


# 카메라 순차 탐색 및 연결
def open_camera_with_fallback(flip_method=0):
    if not CV_AVAILABLE:
        return None

    def _try_v4l2(dev):
        """Open a USB/V4L2 device; skip Jetson CSI vi-output nodes (they go green)."""
        path = dev if isinstance(dev, str) else f"/dev/video{int(dev)}"
        if _is_csi_v4l2_node(path):
            return None
        for api_pref in (getattr(cv2, "CAP_V4L2", 0), 0):
            try:
                cap = cv2.VideoCapture(dev, api_pref) if api_pref else cv2.VideoCapture(dev)
                if cap is not None and cap.isOpened():
                    # Many USB cams need MJPG to deliver real color frames at 640x360+.
                    try:
                        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                    except Exception:
                        pass
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
                    cap.set(cv2.CAP_PROP_FPS, CAM_FPS)
                    if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
                        try:
                            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        except Exception:
                            pass
                    ret, frame = cap.read()
                    if ret and _frame_looks_valid_bgr(frame):
                        return cap
                if cap is not None:
                    cap.release()
            except Exception:
                pass
        return None

    def _try_csi_opencv(sid):
        for sw, sh in ((1280, 720), (1640, 1232)):
            try:
                gst = build_csi_gstreamer_pipeline(
                    sensor_id=sid, sensor_w=sw, sensor_h=sh, flip_method=flip_method
                )
                cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
                if cap is not None and cap.isOpened():
                    ret, frame = cap.read()
                    if ret and _frame_looks_valid_bgr(frame):
                        add_event_log(
                            "CAMERA",
                            f"CSI camera connected (CAM{sid}/dev/video{sid}, {sw}x{sh}->{CAM_WIDTH}x{CAM_HEIGHT}, flip={flip_method})",
                            "auto",
                        )
                        return cap
                if cap is not None:
                    cap.release()
            except Exception:
                pass
        return None

    def _try_csi_gstpipe(sid):
        if not shutil.which("gst-launch-1.0"):
            return None
        try:
            gcap = GstPipeCapture(sensor_id=sid, flip_method=flip_method)
            ret, frame = gcap.read()
            if ret and _frame_looks_valid_bgr(frame):
                add_event_log(
                    "CAMERA",
                    f"CSI camera (gst-launch pipe) connected (CAM{sid}/dev/video{sid}, flip={flip_method})",
                    "auto",
                )
                return gcap
            gcap.release()
        except Exception:
            pass
        return None

    # /dev/video0 on this Jetson board: prefer CSI Argus if it is a vi-output node.
    video0_is_csi = _is_csi_v4l2_node("/dev/video0")
    if video0_is_csi:
        add_event_log(
            "CAMERA",
            f"/dev/video0={_v4l2_device_name('/dev/video0')} -> nvarguscamerasrc (raw V4L2 would be green)",
            "auto",
        )
        # gst-launch BGR pipe is more reliable here than OpenCV appsink (avoids green/dmabuf).
        for sid in (0, 1):
            gcap = _try_csi_gstpipe(sid)
            if gcap is not None:
                return gcap
        for sid in (0, 1):
            cap = _try_csi_opencv(sid)
            if cap is not None:
                return cap
    else:
        # Real USB/UVC camera at /dev/video0
        for dev in ("/dev/video0", 0):
            cap = _try_v4l2(dev)
            if cap is not None:
                add_event_log("CAMERA", "V4L2 camera connected (/dev/video0)", "auto")
                return cap

    # Other USB nodes
    for dev_idx in (1, 2):
        cap = _try_v4l2(dev_idx)
        if cap is not None:
            add_event_log("CAMERA", f"V4L2/USB camera connected (/dev/video{dev_idx})", "auto")
            return cap

    # Last-resort CSI attempts (in case video0 naming was unavailable)
    if not video0_is_csi:
        for sid in (0, 1):
            gcap = _try_csi_gstpipe(sid)
            if gcap is not None:
                return gcap
        for sid in (0, 1):
            cap = _try_csi_opencv(sid)
            if cap is not None:
                return cap
    return None

# 카메라 리소스 해제
def release_active_camera():
    global _active_cam_obj
    if _active_cam_obj is not None:
        try:
            _active_cam_obj.release()
        except Exception:
            pass
        _active_cam_obj = None

# 표적 색상 검출 및 OSD 렌더링
def detect_pink_balloon(frame, preset_name="default", flip_v4l2=False, show_mask=False):
    global _latest_bgr_frame
    if flip_v4l2:
        frame = cv2.flip(frame, -1)
    with _cam_lock:
        _latest_bgr_frame = frame
    h, w = frame.shape[:2]
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = build_target_hsv_mask(hsv, preset_name=preset_name)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    cnt_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = cnt_info[0] if len(cnt_info) == 2 else cnt_info[1]

    best_cnt = None
    best_score = 0.0
    best_area = 0.0
    best_circle = (0.0, 0.0, 0.0)

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
            continue
        perim = cv2.arcLength(hull, True)
        if perim <= 0:
            continue
        circularity = (4.0 * math.pi * hull_area) / (perim * perim)
        if circularity < 0.24:
            continue
        (cx, cy), radius = cv2.minEnclosingCircle(hull)
        if radius < 5.0:
            continue
        circle_area = math.pi * (radius ** 2)
        fill_ratio = hull_area / max(1.0, circle_area)
        if fill_ratio < 0.26:
            continue
        score = hull_area * (0.5 + 0.5 * min(1.0, circularity))
        if score > best_score:
            best_score = score
            best_cnt = hull
            best_area = hull_area
            best_circle = (float(cx), float(cy), float(radius))

    if show_mask:
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

    mid_x, mid_y = w // 2, h // 2
    cv2.line(vis, (mid_x, 0), (mid_x, h), (180, 180, 180), 1)
    cv2.line(vis, (0, mid_y), (w, mid_y), (180, 180, 180), 1)

    if best_cnt is not None:
        cx, cy, radius = best_circle
        norm_x = (cx - (w / 2.0)) / (w / 2.0)
        norm_y = (cy - (h / 2.0)) / (h / 2.0)
        area_pct = (best_area / float(w * h)) * 100.0
        angle_err = norm_x * (CAM_HFOV_DEG / 2.0)
        est_dist_m = (BALLOON_REAL_DIAM_M * BALLOON_FOCAL_PX) / max(4.0, 2.0 * radius)
        est_dist_m = clamp(est_dist_m, 0.05, 60.0)

        icx, icy, ir = int(round(cx)), int(round(cy)), int(round(radius))
        cv2.circle(vis, (icx, icy), ir, (147, 20, 255), 3)
        cv2.circle(vis, (icx, icy), 4, (0, 255, 255), -1)
        cv2.line(vis, (mid_x, h - 20), (icx, icy), (0, 255, 157), 2)
        label = f"TARGET {est_dist_m:.2f}m ({angle_err:+.1f}deg)"
        cv2.putText(vis, label, (max(10, icx - 90), max(25, icy - ir - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        return True, norm_x, norm_y, radius, area_pct, est_dist_m, angle_err, vis

    cv2.putText(vis, f"SCANNING TARGET (H:{TARGET_HUE} S>={TARGET_SAT_MIN})...", (14, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 212, 255), 2)
    return False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, vis

# 카메라 캡처 및 비전 처리 스레드
def camera_worker():
    global _latest_bgr_frame, _latest_jpeg_bytes, _latest_jpeg_seq, _cam_reopen_requested, _active_cam_obj, _last_jpeg_encode_t
    if not CV_AVAILABLE:
        add_event_log("CAMERA", "OpenCV (cv2) missing -> camera vision disabled", "warn")
        with state_lock:
            shared_state["lampCam"] = "err"
            shared_state["cameraOk"] = False
        return

    cap = None
    last_reconnect_t = 0.0
    cam_ever_connected = False
    cam_init_fail_count = 0
    fps_cnt = 0
    fps_t = time.time()
    prev_angle_err = 0.0
    prev_err_t = time.time()

    while True:
        now = time.time()
        if _cam_reopen_requested:
            _cam_reopen_requested = False
            cam_init_fail_count = 0
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
                cap = None
                _active_cam_obj = None

        if cap is None:
            if not cam_ever_connected and cam_init_fail_count >= 2:
                time.sleep(1.0)
                continue
            if now - last_reconnect_t >= CAM_RECONNECT_SEC:
                last_reconnect_t = now
                cap = open_camera_with_fallback(flip_method=_cam_flip_method)
                _active_cam_obj = cap
                if cap is not None:
                    cam_ever_connected = True
                    cam_init_fail_count = 0
                else:
                    cam_init_fail_count += 1
                    with _cam_lock:
                        _vision_state["ok"] = False
                        _vision_state["detected"] = False
                    with state_lock:
                        shared_state["cameraOk"] = False
                        shared_state["lampCam"] = "warn"
                        shared_state["balloonDetected"] = False
                    if not cam_ever_connected and cam_init_fail_count == 2:
                        add_event_log("CAMERA", "No camera detected -> vision search stopped (MANUAL/AUTO still OK)", "warn")
            time.sleep(0.1)
            continue

        ret, frame = cap.read()
        if not ret or frame is None or frame.size == 0:
            add_event_log("CAMERA", "Pi camera frames stopped -> waiting to reconnect", "warn")
            try:
                cap.release()
            except Exception:
                pass
            cap = None
            _active_cam_obj = None
            with _cam_lock:
                _vision_state["ok"] = False
                _vision_state["detected"] = False
            with state_lock:
                shared_state["cameraOk"] = False
                shared_state["lampCam"] = "err"
                shared_state["balloonDetected"] = False
            time.sleep(0.2)
            continue

        fps_cnt += 1
        if now - fps_t >= 1.0:
            cur_fps = round(fps_cnt / (now - fps_t), 1)
            fps_cnt = 0
            fps_t = now
        else:
            cur_fps = _vision_state["fps"]

        try:
            # Soft-flip only for raw USB/V4L2. CSI (GstPipe / OpenCV GStreamer) already
            # applies flip-method in the pipeline — flipping again would cancel 180°.
            if isinstance(cap, GstPipeCapture):
                need_soft_flip = False
            else:
                try:
                    backend = (cap.getBackendName() or "").upper()
                except Exception:
                    backend = ""
                need_soft_flip = "GSTREAMER" not in backend
            vision_tracking_active = (shared_state.get("mode") == "BALLOON")
            if vision_tracking_active:
                detected, nx, ny, rad, area_pct, dist_m, ang_err, vis = detect_pink_balloon(
                    frame,
                    preset_name=_hsv_preset_name,
                    flip_v4l2=(need_soft_flip and _cam_flip_method == 2),
                    show_mask=_cam_show_mask,
                )
            else:
                if need_soft_flip and _cam_flip_method == 2:
                    frame = cv2.flip(frame, -1)
                with _cam_lock:
                    _latest_bgr_frame = frame
                detected, nx, ny, rad, area_pct, dist_m, ang_err = False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                vis = frame
            dt_e = max(0.01, now - prev_err_t)
            d_ang_err = (ang_err - prev_angle_err) / dt_e if detected else 0.0
            prev_angle_err = ang_err if detected else 0.0
            prev_err_t = now

            has_cam_viewer = (_active_stream_clients > 0) or ((now - _last_cam_http_req_t) < 4.0) or (_latest_jpeg_bytes is None)
            min_encode_dt = (0.082 if _cam_lte_mode else 0.038) if has_cam_viewer else 1.0
            jpg_qual = 58 if _cam_lte_mode else 74
            if (now - _last_jpeg_encode_t) >= min_encode_dt:
                ok_jpg, buf = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), jpg_qual])
                if ok_jpg:
                    _latest_jpeg_bytes = buf.tobytes()
                    _latest_jpeg_seq += 1
                    _last_jpeg_encode_t = now

            with _cam_lock:
                _vision_state["ok"] = True
                _vision_state["last_frame_t"] = now
                _vision_state["fps"] = cur_fps
                _vision_state["detected"] = detected
                _vision_state["norm_x"] = round(nx, 3)
                _vision_state["norm_y"] = round(ny, 3)
                _vision_state["radius"] = round(rad, 1)
                _vision_state["area_pct"] = round(area_pct, 2)
                _vision_state["dist_m"] = round(dist_m, 2)
                _vision_state["angle_err"] = round(ang_err, 1)
                _vision_state["d_angle_err"] = round(d_ang_err, 1)

            with state_lock:
                shared_state["cameraOk"] = True
                shared_state["lampCam"] = "ok"
                shared_state["balloonDetected"] = detected
                shared_state["balloonX"] = round(nx, 3)
                shared_state["balloonY"] = round(ny, 3)
                shared_state["balloonRadius"] = round(rad, 1)
                shared_state["balloonAreaPct"] = round(area_pct, 2)
                shared_state["balloonDistM"] = round(dist_m, 2)
                shared_state["balloonAngleErr"] = round(ang_err, 1)
                shared_state["balloonFps"] = cur_fps
        except Exception:
            pass

        time.sleep(0.005)

# 비전 모드 PD 조향 계산
def balloon_steer(cam_angle_err, d_angle_err=0.0):
    pd_term = (cam_angle_err * BALLOON_STEER_GAIN) + (d_angle_err * BALLOON_STEER_KD)
    raw = CENTER - int(STEER_SIGN * pd_term)
    return clamp(raw, max_LEFT, max_RIGHT)

# 비전 모드 추진 출력 계산
def balloon_drive(cam_angle_err, area_pct=0.0, dist_m=10.0):
    a = abs(cam_angle_err)
    if BALLOON_ACTION_MODE == "RAM_POP":
        if a <= BALLOON_CHARGE_ANGLE_DEG or area_pct >= 2.0:
            return max_GO
        if a >= 32.0:
            t = 0.55
        else:
            t = 0.55 + 0.45 * (32.0 - a) / (32.0 - BALLOON_CHARGE_ANGLE_DEG)
        return lerp(STOP, max_GO, t)

    # APPROACH_STOP 감속 계산
    slow_start_d = max(BALLOON_STOP_DIST_M + 1.5, BALLOON_SLOW_DIST_M)
    if dist_m <= BALLOON_STOP_DIST_M or area_pct >= BALLOON_STOP_AREA_PCT:
        return STOP
    elif dist_m < slow_start_d:
        dist_ratio = (dist_m - BALLOON_STOP_DIST_M) / max(0.5, slow_start_d - BALLOON_STOP_DIST_M)
        t_dist = 0.35 + 0.65 * clamp(dist_ratio, 0.0, 1.0)
    else:
        t_dist = 1.0

    if a >= 32.0:
        t_ang = 0.55
    elif a <= BALLOON_CHARGE_ANGLE_DEG:
        t_ang = 1.0
    else:
        t_ang = 0.55 + 0.45 * (32.0 - a) / (32.0 - BALLOON_CHARGE_ANGLE_DEG)

    return lerp(STOP, max_GO, min(t_dist, t_ang))

# 로컬 IP 조회
def get_local_hotspot_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.5)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            return "127.0.0.1"

# Tailscale 가상 IP 조회 (설치·연결 시 100.x.y.z 반환)
def get_tailscale_ip():
    try:
        ts_bin = shutil.which("tailscale")
        if ts_bin:
            out = subprocess.check_output([ts_bin, "ip", "-4"], timeout=1.2, stderr=subprocess.DEVNULL)
            lines = [ln.strip() for ln in out.decode("utf-8", errors="ignore").splitlines() if ln.strip()]
            if lines:
                return lines[0]
    except Exception:
        pass
    return None

_cached_html_bytes = None
_cached_html_mtime = 0.0

# 대시보드 HTML 메모리 캐시 로드
def get_cached_html_bytes():
    global _cached_html_bytes, _cached_html_mtime
    for fname in (HTML_FILENAME, "gcs_2_usv_hud.html", "dashboard.html", "dashboard-v5.html"):
        cand = resolve_path(fname)
        if os.path.exists(cand):
            try:
                mt = os.path.getmtime(cand)
                if _cached_html_bytes is None or mt != _cached_html_mtime:
                    with open(cand, "rb") as f:
                        _cached_html_bytes = f.read()
                    _cached_html_mtime = mt
                return _cached_html_bytes
            except Exception:
                pass
    return _cached_html_bytes

class ThreadingGCSServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    request_queue_size = 64

class GCSRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    timeout = 5.0

    # 역방향 DNS 조회 비활성화 (핫스팟 지연 방지)
    def address_string(self):
        return self.client_address[0]

    # HTTP 액세스 로그 생략
    def log_message(self, format, *args):
        pass

    # 응답 바이트 전송 헬퍼 (Content-Length 명시로 Keep-Alive 지원)
    def _send_bytes(self, status_code, content_type, body_bytes, cache_ctrl="no-store, no-cache, must-revalidate, max-age=0"):
        self.send_response(status_code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body_bytes)))
        self.send_header("Cache-Control", cache_ctrl)
        self.send_header("Pragma", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body_bytes)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Content-Length", "0")
        self.end_headers()

    # GET 요청 처리 (HTML, 상태, 경로, 영상)
    def do_GET(self):
        global _last_gcs_hb_mono, _gcs_hb_streak, _gcs_ever_connected
        global _active_stream_clients, _stream_generation, _last_cam_http_req_t
        try:
            if (
                self.path == "/"
                or self.path.startswith("/?")
                or self.path.startswith("/index")
                or self.path.startswith("/dashboard")
                or self.path.startswith("/gcs")
            ):
                html_bytes = get_cached_html_bytes()
                if html_bytes:
                    self._send_bytes(200, "text/html; charset=utf-8", html_bytes, "no-store, no-cache, must-revalidate, max-age=0")
                else:
                    self._send_bytes(404, "text/plain; charset=utf-8", b"dashboard html file not found")
            elif self.path.startswith("/favicon"):
                self._send_bytes(204, "image/x-icon", b"", "max-age=86400")
            elif self.path.startswith("/api/state"):
                if "hb=1" in self.path:
                    _last_gcs_hb_mono = time.monotonic()
                    _gcs_hb_streak = min(100, _gcs_hb_streak + 1)
                    _gcs_ever_connected = True
                with state_lock:
                    payload = dict(shared_state)
                    payload["logs"] = list(event_logs)[-15:]
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self._send_bytes(200, "application/json; charset=utf-8", body)
            elif self.path.startswith("/api/paths"):
                self._send_bytes(200, "application/json; charset=utf-8", PATHS_JSON_BYTES)
            elif self.path.startswith("/api/camera_frame"):
                _last_cam_http_req_t = time.time()
                jpg = _latest_jpeg_bytes
                if jpg:
                    self._send_bytes(200, "image/jpeg", jpg, "no-store, no-cache, must-revalidate, max-age=0")
                else:
                    self._send_bytes(204, "text/plain", b"")
            elif self.path.startswith("/api/camera_stream"):
                _last_cam_http_req_t = time.time()
                _stream_generation += 1
                my_gen = _stream_generation
                _active_stream_clients += 1
                self.close_connection = True
                try:
                    self.send_response(200)
                    self.send_header("Age", "0")
                    self.send_header("Cache-Control", "no-cache, private")
                    self.send_header("Pragma", "no-cache")
                    self.send_header("Connection", "close")
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    last_sent_seq = -1
                    last_frame_push_t = time.time()
                    while True:
                        # 새 스트림 연결이 2개 초과 누적되면 오래된 좀비 스트림 자동 정리
                        if (_stream_generation - my_gen) >= 2:
                            break
                        jpg = _latest_jpeg_bytes
                        cur_seq = _latest_jpeg_seq
                        now_st = time.time()
                        if jpg and cur_seq != last_sent_seq:
                            try:
                                self.wfile.write(b"--frame\r\n")
                                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                                self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii"))
                                self.wfile.write(jpg)
                                self.wfile.write(b"\r\n")
                                self.wfile.flush()
                                last_sent_seq = cur_seq
                                last_frame_push_t = now_st
                                _last_cam_http_req_t = now_st
                            except (BrokenPipeError, ConnectionResetError, OSError):
                                break
                        elif (now_st - last_frame_push_t) > 12.0:
                            break
                        time.sleep(0.07 if _cam_lte_mode else 0.035)
                finally:
                    _active_stream_clients = max(0, _active_stream_clients - 1)
            else:
                self._send_bytes(404, "text/plain; charset=utf-8", b"not found")
        except Exception:
            self.close_connection = True

    # POST 명령 큐 적재
    def do_POST(self):
        global _last_gcs_hb_mono, _gcs_hb_streak, _gcs_ever_connected
        try:
            if self.path.startswith("/api/cmd"):
                length = int(self.headers.get("Content-Length", 0))
                raw = self.rfile.read(length).decode("utf-8", errors="ignore") if length > 0 else "{}"
                try:
                    data = json.loads(raw)
                    cmd_name = data.get("cmd")
                    cmd_val = data.get("value")
                    _last_gcs_hb_mono = time.monotonic()
                    _gcs_hb_streak = min(100, _gcs_hb_streak + 1)
                    _gcs_ever_connected = True
                    with state_lock:
                        pending_web_cmd["cmd"] = cmd_name
                        pending_web_cmd["value"] = cmd_val
                        if cmd_name:
                            web_cmd_queue.append({"cmd": cmd_name, "value": cmd_val})
                    self._send_bytes(200, "application/json; charset=utf-8", b'{"ok":true}')
                except Exception as e:
                    err_b = json.dumps({"ok": False, "err": str(e)}).encode("utf-8")
                    self._send_bytes(400, "application/json; charset=utf-8", err_b)
            else:
                self._send_bytes(404, "text/plain; charset=utf-8", b"not found")
        except Exception:
            self.close_connection = True

# 웹 서버 스레드 실행
def web_server_worker():
    try:
        server = ThreadingGCSServer(("0.0.0.0", WEB_PORT), GCSRequestHandler)
        local_ip = get_local_hotspot_ip()
        ts_ip = get_tailscale_ip()
        if ts_ip:
            add_event_log("WEB", f"Web GCS server started -> remote (Tailscale): http://{ts_ip}:{WEB_PORT} | local: http://{local_ip}:{WEB_PORT}", "auto")
        else:
            add_event_log("WEB", f"Web GCS server started -> http://{local_ip}:{WEB_PORT} (on Tailscale, try http://100.x.y.z:8080)", "auto")
        server.serve_forever()
    except Exception as e:
        add_event_log("WEB", f"Web server failed to start ({e})", "warn")

# 50Hz 메인 control 루프
def main_loop():
    global _rc_read_count, _last_cog
    global STEER_CTRL_MODE, PID_KP, PID_KI, PID_KD, PID_I_MAX_DEG, STEER_GAIN, max_GO
    global BALLOON_ACTION_MODE, BALLOON_REAL_DIAM_M, BALLOON_STOP_DIST_M, BALLOON_STOP_AREA_PCT
    global CUSTOM_COLOR_ENABLE, TARGET_HUE, TARGET_HUE_TOL, TARGET_SAT_MIN, TARGET_VAL_MIN, TARGET_COLOR_HEX
    global _hsv_preset_name, _cam_flip_method, _cam_reopen_requested, _cam_show_mask, _cam_lte_mode
    global _gcs_hb_streak, GCS_LINK_TIMEOUT_SEC

    init_peripherals()

    current_mode = "MANUAL"
    estop_active = False
    current_point = 0
    target_path = "Center"
    path_auto_switch = True
    path_switch_candidate = None
    path_switch_count = 0
    last_pca_retry_t = 0.0
    last_csv_flush_t = time.time()

    last_pwm_t = 0.0
    last_dbg = 0.0
    last_health_t = 0.0

    filt_st = float(ST_NEU)
    filt_th = float(TH_NEU)
    last_raw_st = ST_NEU
    last_raw_th = TH_NEU
    last_rc_t = 0.0
    last_gps_t = 0.0

    last_lat = last_lon = None
    est_heading = None
    t_dist = 0.0
    angle_err = 0.0
    target_angle = 240.6
    c_lat = c_lon = None
    fix_status = 0
    spd_mps = 0.0
    total_moved_m = 0.0
    prev_pos_lat = prev_pos_lon = None

    bad_fix_start_t = 0.0
    heading_none_start_t = 0.0
    stall_check_start_t = 0.0
    stall_ref_lat = stall_ref_lon = None
    auto_start_time = time.time()
    high_power_drive_sec = 0.0
    high_power_warn_5m = False
    high_power_warn_8m = False
    prev_loop_t = time.time()

    nav_info = None
    heading_info = None
    last_nav_t = 0.0
    was_auto = False

    balloon_last_seen_t = 0.0
    balloon_mode_start_t = 0.0
    balloon_last_angle_err = 0.0
    balloon_substate = "IDLE"
    balloon_arrive_count = 0
    web_manual_steer = CENTER
    web_manual_drive = STOP
    web_manual_until_t = 0.0
    web_manual_active = False
    web_manual_stop_hold_t = 0.0
    rc_ignore_mode = False
    rc_manual_latched = False
    rc_auto_active = False
    rc_entry_steer_held = False
    rc_auto_hold_start = None
    rc_auto_gesture_consumed = False
    gcs_watchdog_enabled = True
    gcs_link_paused = False

    ready = pwm_ready()
    if ready:
        safe_stop()
        time.sleep(2.0)

    load_pid_config()
    gcs_watchdog_enabled = bool(shared_state.get("linkWatchdog", True))
    shared_state["autoStrategy"] = AUTO_STRATEGY.name
    add_event_log(
        "AUTO",
        f"AUTO controller: {AUTO_STRATEGY.name} | cruise ratio {AUTO_CRUISE_RATIO:.2f} | "
        f"finish margin {NAVIGATOR.finish_margin_m:g} m | settings: "
        + (AUTO_CONFIG_FILE if AUTO_CONFIG_FOUND else "defaults (no calibration.json)"),
        "auto",
    )

    if NTRIP_ENABLE:
        threading.Thread(target=ntrip_worker, daemon=True).start()

    if WEB_ENABLE:
        threading.Thread(target=web_server_worker, daemon=True).start()

    if CAM_ENABLE:
        threading.Thread(target=camera_worker, daemon=True).start()

    add_event_log("START", "Main control loop started (50Hz, MANUAL)", "auto")
    loop_counter = 0
    loop_rate_t = time.time()

    while True:
        now = time.time()
        now_mono = time.monotonic()
        loop_counter += 1

        if now - loop_rate_t >= 1.0:
            dt_rate = now - loop_rate_t
            shared_state["loopHz"] = round(loop_counter / dt_rate, 1)
            shared_state["rcHz"] = round(_rc_read_count / dt_rate, 1)
            loop_counter = 0
            _rc_read_count = 0
            loop_rate_t = now

        if now - last_health_t >= 2.5:
            last_health_t = now
            temp_c, load_pct = read_jetson_health()
            shared_state["jetsonCpuTemp"] = temp_c
            shared_state["jetsonLoad"] = load_pct
            if pca is None and HW_AVAILABLE and (now - last_pca_retry_t) >= 10.0:
                last_pca_retry_t = now
                init_pca9685()
            if (now - last_csv_flush_t) >= 5.0:
                last_csv_flush_t = now
                try:
                    log_file.flush()
                except Exception:
                    pass
            if ser is not None and gps_ser is not None and pca is not None:
                shared_state["lampSerial"] = "ok"
            elif pca is None:
                shared_state["lampSerial"] = "err"

        # Always receive RC, including web-only operation.
        raw_st, raw_th = read_latest_rc()
        if raw_st is not None and raw_th is not None:
            last_rc_t = now
            last_raw_st, last_raw_th = raw_st, raw_th
            filt_st = filt_st + RC_FILTER * (raw_st - filt_st)
            filt_th = filt_th + RC_FILTER * (raw_th - filt_th)

        ST_V, TH_V = int(round(filt_st)), int(round(filt_th))
        rc_ok = (now - last_rc_t) < RC_TIMEOUT

        # 2. GPS 수신
        is_reversing = (_last_drive is not None and _last_drive < STOP - 30)
        if is_reversing:
            est_heading = None
            _last_cog = None
            if c_lat is not None and c_lon is not None:
                last_lat, last_lon = c_lat, c_lon

        gps = read_latest_gps(is_reversing=is_reversing)
        fix_status = _last_fix
        shared_state["satellites"] = _last_sats
        shared_state["hdop"] = _last_hdop

        if gps:
            c_lat, c_lon, gps_cog, spd_kn, fix_status, sats, hdop = gps
            last_gps_t = now
            spd_mps = spd_kn * 0.514444

            if prev_pos_lat is not None and prev_pos_lon is not None:
                step_m = haversine(prev_pos_lat, prev_pos_lon, c_lat, c_lon)
                if 0.05 <= step_m <= 5.0:
                    total_moved_m += step_m
            prev_pos_lat, prev_pos_lon = c_lat, c_lon

            if not is_reversing:
                if last_lat is not None and last_lon is not None:
                    moved = haversine(last_lat, last_lon, c_lat, c_lon)
                    min_move_thresh = HEADING_INIT_MOVE_M if (est_heading is None and _last_cog is None) else 0.8
                    if moved >= min_move_thresh:
                        est_heading = gps_bearing(last_lat, last_lon, c_lat, c_lon)
                        last_lat, last_lon = c_lat, c_lon
                else:
                    last_lat, last_lon = c_lat, c_lon

            if current_mode == "MANUAL" and current_point < total_waypoints:
                _, preview_dists = pick_path(c_lat, c_lon, current_point, target_path)
                shared_state["distCenter"] = round(preview_dists.get("Center", 0.0), 2)
                shared_state["distLeft"] = round(preview_dists.get("Left", 0.0), 2)
                shared_state["distRight"] = round(preview_dists.get("Right", 0.0), 2)
                t_lat, t_lon = wp_coord(PATHS[target_path], current_point)
                t_dist = haversine(c_lat, c_lon, t_lat, t_lon)
                look_lat, look_lon = get_lookahead_target(c_lat, c_lon, PATHS[target_path], current_point)
                target_angle = gps_bearing(c_lat, c_lon, look_lat, look_lon)

        gps_ok = c_lat is not None and (now - last_gps_t) < GPS_TIMEOUT

        # 3. 웹 명령 처리 (큐에 적재된 모든 명령을 지연 없이 순차 처리)
        cmds_to_process = []
        with state_lock:
            while web_cmd_queue:
                cmds_to_process.append(web_cmd_queue.popleft())
            if not cmds_to_process and pending_web_cmd["cmd"] is not None:
                cmds_to_process.append({"cmd": pending_web_cmd["cmd"], "value": pending_web_cmd["value"]})
            pending_web_cmd["cmd"] = None
            pending_web_cmd["value"] = None

        # A deliberate RC gesture uses the same AUTO initialization as the dashboard.
        rc_auto_gesture_input = (
            rc_ok and rc_throttle_norm(last_raw_th) == 0.0
            and rc_steer_norm(last_raw_st) >= HW["auto_right_ratio"]
        )
        rc_auto_gesture = current_mode == "MANUAL" and not estop_active and rc_auto_gesture_input
        if not rc_auto_gesture:
            rc_auto_hold_start = None
            # A held entry gesture cannot restart AUTO after a finish/failsafe.
            if not rc_auto_gesture_input:
                rc_auto_gesture_consumed = False
        elif not rc_auto_gesture_consumed:
            if rc_auto_hold_start is None:
                rc_auto_hold_start = now_mono
            elif now_mono - rc_auto_hold_start >= HW["auto_hold_sec"]:
                rc_auto_gesture_consumed = True
                cmds_to_process.append({"cmd": "set_mode", "value": "AUTO", "source": "rc"})

        for _cmd_item in cmds_to_process:
          cmd = _cmd_item["cmd"]
          cmd_val = _cmd_item["value"]
          from_rc = _cmd_item.get("source") == "rc"
          if cmd == "estop_on":
              estop_active = True
              current_mode = "MANUAL"
              balloon_substate = "IDLE"
              reset_pid_state()
              safe_stop()
              add_event_log("E-STOP", "E-STOP engaged! All motors cut", "warn")
          elif cmd == "estop_off":
              estop_active = False
              current_mode = "MANUAL"
              balloon_substate = "IDLE"
              reset_pid_state()
              safe_stop()
              add_event_log("E-STOP", "E-STOP cleared -> MANUAL idle", "auto")
          elif cmd == "set_mode" and cmd_val in ("MANUAL", "AUTO", "BALLOON") and not estop_active:
              # A takeover is released only by an explicit mode selection at neutral.
              if not from_rc and rc_manual_latched and rc_ok and (rc_steer_norm(last_raw_st) != 0 or rc_throttle_norm(last_raw_th) != 0):
                  continue
              if not from_rc:
                  rc_manual_latched = False
                  rc_auto_active = False
                  rc_entry_steer_held = False
              if cmd_val == "AUTO" and current_mode != "AUTO":
                  if not gps_ok or not (2 <= fix_status <= 5):
                      safe_stop()
                      add_event_log("FAILSAFE", f"GPS not ready (fix={fix_status}) -> AUTO entry rejected", "warn")
                  else:
                      current_mode = "AUTO"
                      rc_manual_latched = False
                      rc_auto_active = from_rc
                      rc_entry_steer_held = from_rc
                      web_manual_until_t = web_manual_stop_hold_t = 0.0
                      web_manual_active = False
                      reset_pid_state()
                      heading_none_start_t = now
                      stall_check_start_t = 0.0
                      bad_fix_start_t = 0.0
                      auto_start_time = now
                      # 진입 직후 오버라이드 오작동 방지
                      filt_st, filt_th = float(ST_NEU), float(TH_NEU)
                      ST_V, TH_V = ST_NEU, TH_NEU
                      # 저속 진입 시 COG만 초기화
                      if spd_mps < 0.5:
                          _last_cog = None
                          last_lat, last_lon = c_lat, c_lon
                      # Shared navigator entry: nearest lane (or the locked lane) and a waypoint ahead
                      NAV_GPS.reset_heading_anchor()
                      heading_reused = False
                      if HEADING_SELECTOR is not None:
                          entry_snap = dict(
                              NAV_GPS.get_snapshot(), heading_deg=None, position_heading_deg=None,
                              position_heading_age_sec=None, position_heading_time_mono=None,
                              position_speed_mps=None,
                          )
                          heading_reused = HEADING_SELECTOR.enter_auto(entry_snap, now_mono)
                      NAVIGATOR.allow_lane_shift = path_auto_switch
                      nav_lane, nav_wp, nav_dist = NAVIGATOR.enter_auto_mode(
                          c_lat, c_lon, now_mono=now_mono,
                          lane=None if path_auto_switch else LANE_KEYS[target_path],
                      )
                      target_path = LANE_NAMES[nav_lane]
                      current_point = NAVIGATOR.target_idx
                      nav_info = None
                      last_nav_t = 0.0
                      with state_lock:
                          shared_state["mode"] = current_mode
                          shared_state["targetPath"] = target_path
                      mode_tag = "AUTO-LANE" if path_auto_switch else f"{target_path}-LOCKED"
                      hdg_tag = " | heading: reusing recent GPS course" if heading_reused else ""
                      add_event_log(
                          "MODE",
                          f"{'RC gesture' if from_rc else 'Web command'} -> AUTO (WP {nav_wp}, {target_path} [{mode_tag}], {nav_dist:.2f} m from lane, "
                          f"{AUTO_STRATEGY.name}){hdg_tag}",
                          "auto",
                      )
              elif cmd_val == "MANUAL" and current_mode != "MANUAL":
                  current_mode = "MANUAL"
                  balloon_substate = "IDLE"
                  with state_lock:
                      shared_state["mode"] = current_mode
                  safe_stop()
                  add_event_log("MODE", "Web command -> MANUAL", "warn")
              elif cmd_val == "BALLOON" and current_mode != "BALLOON":
                  with _cam_lock:
                      cam_ready = _vision_state["ok"] and ((now - _vision_state["last_frame_t"]) < CAM_TIMEOUT)
                      b_det_now = _vision_state["detected"]
                  if not cam_ready:
                      safe_stop()
                      add_event_log("FAILSAFE", "Pi camera not ready -> BALLOON entry rejected", "warn")
                  else:
                      current_mode = "BALLOON"
                      stall_check_start_t = 0.0
                      balloon_arrive_count = 0
                      balloon_mode_start_t = now
                      balloon_last_seen_t = now if b_det_now else 0.0
                      balloon_substate = "TRACKING" if b_det_now else "SEARCHING"
                      filt_st, filt_th = float(ST_NEU), float(TH_NEU)
                      ST_V, TH_V = ST_NEU, TH_NEU
                      with state_lock:
                          shared_state["mode"] = current_mode
                      add_event_log("BALLOON", "Web command -> VISION (optical target tracking)", "auto")
          elif cmd == "reset_wp":
              current_point = 0
              total_moved_m = 0.0
              auto_start_time = now
              reset_pid_state()
              add_event_log("WP_RESET", "Waypoint reset to WP 1", "auto")
          elif cmd == "set_path":
              if cmd_val == "AUTO":
                  path_auto_switch = True
                  path_switch_candidate = None
                  path_switch_count = 0
                  if c_lat is not None and c_lon is not None and current_point < total_waypoints:
                      cand_now, _ = pick_path(c_lat, c_lon, current_point, target_path)
                      if cand_now != target_path:
                          target_path = cand_now
                  reset_pid_state()
                  with state_lock:
                      shared_state["pathAutoSwitch"] = True
                      shared_state["targetPath"] = target_path
                  add_event_log("PATH", f"AUTO lane switching enabled -> current '{target_path}'", "auto")
              elif isinstance(cmd_val, dict):
                  req_p = cmd_val.get("path")
                  req_auto = cmd_val.get("auto")
                  if req_p in PATHS:
                      target_path = req_p
                  if req_auto is not None:
                      path_auto_switch = bool(req_auto)
                  else:
                      path_auto_switch = False
                  path_switch_candidate = None
                  path_switch_count = 0
                  reset_pid_state()
                  with state_lock:
                      shared_state["targetPath"] = target_path
                      shared_state["pathAutoSwitch"] = path_auto_switch
                  mode_str = "disturbance AUTO switch" if path_auto_switch else "manual lock"
                  add_event_log("PATH", f"Follow lane '{target_path}' set ({mode_str})", "auto")
              elif cmd_val in PATHS:
                  target_path = cmd_val
                  path_auto_switch = False
                  path_switch_candidate = None
                  path_switch_count = 0
                  reset_pid_state()
                  with state_lock:
                      shared_state["targetPath"] = target_path
                      shared_state["pathAutoSwitch"] = False
                  add_event_log("PATH", f"Follow lane '{target_path}' locked manually (AUTO disturbance switch off)", "auto")
              # Apply to the shared navigator (re-enter on that lane when already in AUTO)
              NAVIGATOR.allow_lane_shift = path_auto_switch
              if current_mode == "AUTO" and c_lat is not None and c_lon is not None:
                  nav_lane, _, _ = NAVIGATOR.enter_auto_mode(
                      c_lat, c_lon, now_mono=now_mono,
                      lane=None if path_auto_switch else LANE_KEYS[target_path],
                  )
                  target_path = LANE_NAMES[nav_lane]
                  current_point = NAVIGATOR.target_idx
                  last_nav_t = 0.0
                  with state_lock:
                      shared_state["targetPath"] = target_path
          elif cmd == "set_ctrl_mode" and cmd_val in ("P", "PID"):
              STEER_CTRL_MODE = cmd_val
              reset_pid_state()
              with state_lock:
                  shared_state["steerCtrlMode"] = STEER_CTRL_MODE
              save_pid_config()
              add_event_log("CTRL", f"AUTO steering mode changed and saved -> {STEER_CTRL_MODE} control", "auto")
          elif cmd == "set_pid_gains" and isinstance(cmd_val, dict):
              try:
                  if cmd_val.get("mode") in ("P", "PID"):
                      STEER_CTRL_MODE = cmd_val.get("mode")
                  if cmd_val.get("kp") is not None:
                      PID_KP = round(clamp(float(cmd_val["kp"]), 1.0, 45.0), 2)
                      STEER_GAIN = PID_KP
                  if cmd_val.get("ki") is not None:
                      PID_KI = round(clamp(float(cmd_val["ki"]), 0.0, 10.0), 3)
                  if cmd_val.get("kd") is not None:
                      PID_KD = round(clamp(float(cmd_val["kd"]), 0.0, 25.0), 2)
                  if cmd_val.get("imax") is not None:
                      PID_I_MAX_DEG = round(clamp(float(cmd_val["imax"]), 2.0, 40.0), 1)
                  if PID_KI <= 0.0 or STEER_CTRL_MODE == "P":
                      reset_pid_state()
                  with state_lock:
                      shared_state["steerCtrlMode"] = STEER_CTRL_MODE
                      shared_state["pidKp"] = PID_KP
                      shared_state["pidKi"] = PID_KI
                      shared_state["pidKd"] = PID_KD
                      shared_state["pidIMax"] = PID_I_MAX_DEG
                  save_pid_config()
                  add_event_log(
                      "CTRL",
                      f"Steering settings applied and saved [{STEER_CTRL_MODE}] Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}, IMax={PID_I_MAX_DEG}°",
                      "auto",
                  )
              except (ValueError, TypeError) as e:
                  add_event_log("CTRL", f"PID parameter format error ignored ({e})", "warn")
          elif cmd == "reset_pid_i":
              reset_pid_state()
              add_event_log("CTRL", "PID integral (I) and derivative state reset to 0.", "auto")
          elif cmd == "set_max_go":
              try:
                  req_max_go = int(float(cmd_val))
                  if req_max_go in MAX_GO_PRESETS:
                      max_GO = req_max_go
                  else:
                      max_GO = int(clamp(req_max_go, STOP + 150, 5200))
                  if max_GO < 5200:
                      high_power_drive_sec = 0.0
                      high_power_warn_5m = False
                      high_power_warn_8m = False
                  with state_lock:
                      shared_state["maxGo"] = max_GO
                  save_pid_config()
                  pct_str = round(((max_GO - STOP) / float(DEFAULT_MAX_GO - STOP)) * 100)
                  if max_GO >= 5200:
                      add_event_log("DRIVE", f"High-thrust preset ({max_GO}, {pct_str}%) enabled for all modes (do not run >10 min continuous)", "warn")
                  else:
                      add_event_log("DRIVE", f"Max forward motor (max_GO) changed -> {max_GO} (vs default {pct_str}%, all modes)", "auto")
              except (ValueError, TypeError) as e:
                  add_event_log("DRIVE", f"max_GO setting error ignored ({e})", "warn")
          elif cmd == "set_balloon_config" and isinstance(cmd_val, dict):
              try:
                  if cmd_val.get("actionMode") in ("APPROACH_STOP", "RAM_POP"):
                      BALLOON_ACTION_MODE = cmd_val["actionMode"]
                  if cmd_val.get("realDiamM") is not None:
                      BALLOON_REAL_DIAM_M = round(clamp(float(cmd_val["realDiamM"]), 0.2, 8.0), 2)
                  if cmd_val.get("stopDistM") is not None:
                      BALLOON_STOP_DIST_M = round(clamp(float(cmd_val["stopDistM"]), 0.8, 15.0), 2)
                  if cmd_val.get("stopAreaPct") is not None:
                      BALLOON_STOP_AREA_PCT = round(clamp(float(cmd_val["stopAreaPct"]), 5.0, 80.0), 1)
                  if cmd_val.get("maxGo") is not None:
                      req_mg = int(float(cmd_val["maxGo"]))
                      max_GO = req_mg if req_mg in MAX_GO_PRESETS else int(clamp(req_mg, STOP + 150, 5200))
                      if max_GO < 5200:
                          high_power_drive_sec = 0.0
                          high_power_warn_5m = False
                          high_power_warn_8m = False
                  balloon_arrive_count = 0
                  with state_lock:
                      shared_state["maxGo"] = max_GO
                      shared_state["balloonActionMode"] = BALLOON_ACTION_MODE
                      shared_state["balloonRealDiamM"] = BALLOON_REAL_DIAM_M
                      shared_state["balloonStopDistM"] = BALLOON_STOP_DIST_M
                      shared_state["balloonStopAreaPct"] = BALLOON_STOP_AREA_PCT
                  save_pid_config()
                  mode_kor = "approach-stop" if BALLOON_ACTION_MODE == "APPROACH_STOP" else "ram-pop"
                  add_event_log(
                      "VISION",
                      f"Saved target/thrust settings: [{mode_kor}] diam={BALLOON_REAL_DIAM_M}m, stopDist={BALLOON_STOP_DIST_M}m, max_GO={max_GO}",
                      "auto",
                  )
              except (ValueError, TypeError) as e:
                  add_event_log("VISION", f"Vision target setting format error ignored ({e})", "warn")
          elif cmd == "set_hsv_preset" and cmd_val in HSV_PRESETS:
              _hsv_preset_name = cmd_val
              CUSTOM_COLOR_ENABLE = False
              with state_lock:
                  shared_state["hsvPreset"] = _hsv_preset_name
                  shared_state["customColorEnable"] = False
              save_pid_config()
              add_event_log("CAMERA", f"Target HSV preset -> '{_hsv_preset_name}'", "auto")
          elif cmd == "set_target_color" and isinstance(cmd_val, dict):
              try:
                  CUSTOM_COLOR_ENABLE = bool(cmd_val.get("customEnable", True))
                  if cmd_val.get("hex") and str(cmd_val["hex"]).startswith("#") and cmd_val.get("fromHexOnly"):
                      h_cv, s_cv, v_cv = hex_to_hsv_cv(cmd_val["hex"])
                      TARGET_HUE = h_cv
                      TARGET_COLOR_HEX = str(cmd_val["hex"])[:7]
                  else:
                      if cmd_val.get("hue") is not None:
                          TARGET_HUE = int(clamp(int(float(cmd_val["hue"])), 0, 179))
                      if cmd_val.get("hex") and str(cmd_val["hex"]).startswith("#"):
                          TARGET_COLOR_HEX = str(cmd_val["hex"])[:7]
                      else:
                          TARGET_COLOR_HEX = hsv_to_hex(TARGET_HUE, max(160, TARGET_SAT_MIN), 240)
                  if cmd_val.get("hueTol") is not None:
                      TARGET_HUE_TOL = int(clamp(int(float(cmd_val["hueTol"])), 3, 40))
                  if cmd_val.get("satMin") is not None:
                      TARGET_SAT_MIN = int(clamp(int(float(cmd_val["satMin"])), 25, 245))
                  if cmd_val.get("valMin") is not None:
                      TARGET_VAL_MIN = int(clamp(int(float(cmd_val["valMin"])), 25, 245))
                  with state_lock:
                      shared_state["customColorEnable"] = CUSTOM_COLOR_ENABLE
                      shared_state["targetHue"] = TARGET_HUE
                      shared_state["targetHueTol"] = TARGET_HUE_TOL
                      shared_state["targetSatMin"] = TARGET_SAT_MIN
                      shared_state["targetValMin"] = TARGET_VAL_MIN
                      shared_state["targetColorHex"] = TARGET_COLOR_HEX
                  save_pid_config()
                  add_event_log(
                      "VISION",
                      f"Target color set: H={TARGET_HUE}(±{TARGET_HUE_TOL}), S≥{TARGET_SAT_MIN}, V≥{TARGET_VAL_MIN} [{TARGET_COLOR_HEX}]",
                      "auto",
                  )
              except (ValueError, TypeError) as e:
                  add_event_log("VISION", f"Target color setting error ignored ({e})", "warn")
          elif cmd == "pick_color_at" and isinstance(cmd_val, dict):
              try:
                  u_n = clamp(float(cmd_val.get("u", 0.5)), 0.0, 1.0)
                  v_n = clamp(float(cmd_val.get("v", 0.5)), 0.0, 1.0)
                  ok_pick, msg_pick = sample_color_from_latest_frame(u_n, v_n)
                  add_event_log("VISION", msg_pick, "auto" if ok_pick else "warn")
              except (ValueError, TypeError) as e:
                  add_event_log("VISION", f"Click color sample error ({e})", "warn")
          elif cmd == "toggle_cam_flip":
              _cam_flip_method = 2 if _cam_flip_method == 0 else 0
              _cam_reopen_requested = True
              shared_state["camFlip"] = 180 if _cam_flip_method == 2 else 0
              add_event_log("CAMERA", f"Pi camera flip set -> {shared_state['camFlip']} deg", "auto")
          elif cmd == "toggle_cam_mask":
              _cam_show_mask = not _cam_show_mask
              shared_state["camShowMask"] = _cam_show_mask
              add_event_log("CAMERA", f"Pi camera full-frame HSV mask -> {'ON' if _cam_show_mask else 'OFF'}", "auto")
          elif cmd == "toggle_cam_lte":
              _cam_lte_mode = not _cam_lte_mode
              with state_lock:
                  shared_state["camLteMode"] = _cam_lte_mode
              mode_txt = "LTE low-latency (12 FPS)" if _cam_lte_mode else "High quality (25 FPS)"
              add_event_log("CAMERA", f"Camera stream mode -> {mode_txt}", "auto")
          elif cmd == "set_rc_ignore":
              rc_ignore_mode = bool(cmd_val)
              with state_lock:
                  shared_state["rcIgnore"] = rc_ignore_mode
              if rc_ignore_mode:
                  add_event_log("REMOTE", "Web-only control ON -> idle RC suppressed; RC takeover remains active", "warn")
              else:
                  add_event_log("REMOTE", "RC link restored -> RC manual and emergency override active", "auto")
          elif cmd == "set_link_watchdog":
              gcs_watchdog_enabled = bool(cmd_val)
              if not gcs_watchdog_enabled:
                  gcs_link_paused = False
              with state_lock:
                  shared_state["linkWatchdog"] = gcs_watchdog_enabled
                  shared_state["linkPaused"] = gcs_link_paused
              save_pid_config()
              add_event_log("LINK", f"Link-loss auto-hold (watchdog) -> {'ON' if gcs_watchdog_enabled else 'OFF'}", "auto")
          elif cmd == "set_remote_config" and isinstance(cmd_val, dict):
              try:
                  if cmd_val.get("camLteMode") is not None:
                      _cam_lte_mode = bool(cmd_val["camLteMode"])
                  if cmd_val.get("linkWatchdog") is not None:
                      gcs_watchdog_enabled = bool(cmd_val["linkWatchdog"])
                      if not gcs_watchdog_enabled:
                          gcs_link_paused = False
                  if cmd_val.get("linkTimeoutSec") is not None:
                      GCS_LINK_TIMEOUT_SEC = round(clamp(float(cmd_val["linkTimeoutSec"]), 2.0, 10.0), 1)
                  if cmd_val.get("rcIgnore") is not None:
                      rc_ignore_mode = bool(cmd_val["rcIgnore"])
                  if cmd_val.get("camFlip") is not None:
                      want_flip = 2 if int(cmd_val["camFlip"]) == 180 else 0
                      if want_flip != _cam_flip_method:
                          _cam_flip_method = want_flip
                          _cam_reopen_requested = True
                  with state_lock:
                      shared_state["camLteMode"] = _cam_lte_mode
                      shared_state["linkWatchdog"] = gcs_watchdog_enabled
                      shared_state["linkTimeoutSec"] = GCS_LINK_TIMEOUT_SEC
                      shared_state["linkPaused"] = gcs_link_paused
                      shared_state["rcIgnore"] = rc_ignore_mode
                      shared_state["camFlip"] = 180 if _cam_flip_method == 2 else 0
                  save_pid_config()
                  add_event_log(
                      "REMOTE",
                      f"Remote/low-latency settings saved: video={'LTE(12fps)' if _cam_lte_mode else 'HQ(25fps)'}, linkHold={'ON' if gcs_watchdog_enabled else 'OFF'}({GCS_LINK_TIMEOUT_SEC}s), rcIgnore={'ON' if rc_ignore_mode else 'OFF'}",
                      "auto",
                  )
              except (ValueError, TypeError) as e:
                  add_event_log("REMOTE", f"Remote settings format error ignored ({e})", "warn")
          elif cmd == "web_manual_drive" and isinstance(cmd_val, dict) and not estop_active and not rc_manual_latched:
              try:
                  current_mode = "MANUAL"
                  balloon_substate = "IDLE"
                  req_st = int(float(cmd_val.get("steer", CENTER)))
                  req_dr = int(float(cmd_val.get("drive", STOP)))
                  dur_s = clamp(float(cmd_val.get("duration", 0.45)), 0.15, 1.2)
                  web_manual_steer = int(clamp(req_st, min(max_LEFT, max_RIGHT), max(max_LEFT, max_RIGHT)))
                  web_manual_drive = int(clamp(req_dr, min(max_BACK, max_GO), max(max_BACK, max_GO)))
                  web_manual_until_t = now + dur_s
                  web_manual_stop_hold_t = 0.0
                  web_manual_active = True
              except Exception:
                  pass
          elif cmd == "web_manual_stop" and not rc_manual_latched:
              web_manual_until_t = 0.0
              web_manual_active = False
              web_manual_steer = CENTER
              web_manual_drive = STOP
              web_manual_stop_hold_t = now + 0.35
              safe_stop()
              last_pwm_t = now
        heading = None
        if not is_reversing:
            if _last_cog is not None and _last_spd_kn >= 0.8:
                heading = _last_cog
            elif est_heading is not None:
                heading = est_heading
            elif _last_cog is not None:
                heading = _last_cog

        rc_effective_ok = (rc_ok and not rc_ignore_mode)
        shared_state["lampRc"] = "warn" if rc_ignore_mode else ("ok" if rc_ok else "err")
        shared_state["lampGps"] = "ok" if (gps_ok and 2 <= fix_status <= 5) else ("warn" if gps_ok else "err")

        # 원격 관제(Tailscale/LTE) 통신 두절 감시 -> 연결 회복 전까지 제자리 정지(HOLD)
        now_mono = time.monotonic()
        if gcs_watchdog_enabled and _gcs_ever_connected:
            since_gcs_hb = now_mono - _last_gcs_hb_mono
            if since_gcs_hb > GCS_LINK_TIMEOUT_SEC:
                if not gcs_link_paused:
                    gcs_link_paused = True
                    _gcs_hb_streak = 0
                    web_manual_until_t = 0.0
                    web_manual_active = False
                    reset_pid_state()
                    safe_stop()
                    add_event_log("LINK", f"GCS link lost ({since_gcs_hb:.1f}s) -> hold stop until link recovers", "warn")
            elif gcs_link_paused and _gcs_hb_streak >= 2:
                gcs_link_paused = False
                stall_check_start_t = 0.0
                heading_none_start_t = now
                bad_fix_start_t = 0.0
                reset_pid_state()
                add_event_log("LINK", f"GCS link recovered -> hold released ({current_mode})", "auto")
        else:
            gcs_link_paused = False

        shared_state["lampLink"] = "err" if gcs_link_paused else ("ok" if _gcs_ever_connected else "warn")
        shared_state["linkPaused"] = gcs_link_paused
        shared_state["linkWatchdog"] = gcs_watchdog_enabled
        shared_state["linkTimeoutSec"] = GCS_LINK_TIMEOUT_SEC
        shared_state["rcIgnore"] = rc_ignore_mode
        shared_state["rcManualActive"] = rc_manual_latched

        if estop_active:
            safe_stop()
            with state_lock:
                shared_state["mode"] = "MANUAL"
                shared_state["estop"] = True
                shared_state["speed"] = 0.0
            elapsed = time.time() - now
            time.sleep(max(0.001, 0.02 - elapsed))
            continue

        # RC-started AUTO has its own link requirement, independent of the browser.
        if rc_auto_active and not rc_ok:
            safe_stop()
            current_mode = "MANUAL"
            rc_manual_latched = True
            rc_auto_active = False
            rc_entry_steer_held = False
            add_event_log("FAILSAFE", "RC lost during RC AUTO -> stop and MANUAL", "warn")
        if rc_entry_steer_held and rc_ok and rc_steer_norm(last_raw_st) < HW["auto_right_ratio"]:
            rc_entry_steer_held = False
        suppress_entry_steering = (
            rc_entry_steer_held and rc_ok
            and rc_steer_norm(last_raw_st) >= HW["auto_right_ratio"]
            and rc_throttle_norm(last_raw_th) > -RC_OVERRIDE_REVERSE_RATIO
        )

        # Raw RC always remains available, even in web-only mode and link hold.
        if rc_ok and not suppress_entry_steering and rc_takeover_requested(current_mode, last_raw_st, last_raw_th):
            if not rc_manual_latched:
                add_event_log("OVERRIDE", "RC takeover -> MANUAL (dashboard control released)", "warn")
            rc_manual_latched = True
            shared_state["rcManualActive"] = True
            shared_state["lampRc"] = "ok"
            current_mode = "MANUAL"
            balloon_substate = "IDLE"
            web_manual_until_t = web_manual_stop_hold_t = 0.0
            web_manual_active = False
            reset_pid_state()

        # 원격 통신 끊김 시 연결 회복 전까지 제자리 정지 유지
        if rc_manual_latched:
            # Keep ownership through neutral and RC loss; never resume web/AUTO implicitly.
            if rc_ok:
                steer, drive = rc_to_logical(last_raw_st, last_raw_th)
                if ready and (now - last_pwm_t) >= PWM_PERIOD:
                    set_pwm(steer, drive)
                    last_pwm_t = now
            else:
                safe_stop()
        elif gcs_link_paused and not rc_auto_active:
            safe_stop()
            stall_check_start_t = 0.0
            heading_none_start_t = now
            bad_fix_start_t = 0.0
            if current_mode == "AUTO":
                # Holding still breaks forward motion: reacquire heading and power ramp on resume
                nav_info = None
                last_nav_t = 0.0
                if HEADING_SELECTOR is not None:
                    HEADING_SELECTOR.suspend()
                AUTO_STRATEGY.suspend()
            if now - last_dbg > 1.0:
                safe_print(f"LINK_HOLD [waiting for GCS link -> holding stop] mode={current_mode} [RTK:{fix_status}]")
                last_dbg = now

        # MANUAL 모드 control (누르는 동안만 작동하는 모멘터리 웹 조종 & RC 수동 지원)
        elif current_mode == "MANUAL":
            if _pid_last_p != 0.0 or _pid_last_i != 0.0 or _pid_last_d != 0.0:
                reset_pid_state()
            balloon_substate = "IDLE"
            shared_state["lampStall"] = "ok"
            if web_manual_active and now >= web_manual_until_t:
                web_manual_active = False
                web_manual_steer, web_manual_drive = CENTER, STOP
                safe_stop()
                last_pwm_t = now
            if now < web_manual_until_t:
                steer, drive = web_manual_steer, web_manual_drive
                if ready and (now - last_pwm_t) >= PWM_PERIOD:
                    set_pwm(steer, drive, force=True)
                    last_pwm_t = now
                if now - last_dbg > 0.5:
                    safe_print(f"WEB_MANUAL pwm={steer},{drive} max={max_GO} [RTK:{fix_status}]")
                    last_dbg = now
            elif now < web_manual_stop_hold_t:
                safe_stop()
            elif rc_effective_ok and ST_V is not None:
                steer, drive = rc_to_logical(ST_V, TH_V)
                if ready and (now - last_pwm_t) >= PWM_PERIOD:
                    set_pwm(steer, drive)
                    last_pwm_t = now
                if now - last_dbg > 0.5:
                    safe_print(f"MANUAL raw={last_raw_st},{last_raw_th} filt={ST_V},{TH_V} pwm={steer},{drive} max={max_GO} [RTK:{fix_status}]")
                    last_dbg = now
                    current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    val_lat = f"{c_lat:.7f}" if c_lat is not None else "0.0"
                    val_lon = f"{c_lon:.7f}" if c_lon is not None else "0.0"
                    try:
                        log_file.write(f"{current_time},{current_mode},{target_path},{current_point + 1},{t_dist:.1f},{angle_err:.1f},{val_lat},{val_lon},{fix_status},{spd_mps:.2f},{steer},{drive}{LOG_NAV_BLANK}\n")
                    except Exception:
                        pass
            else:
                safe_stop()
                if now - last_dbg > 1.5:
                    safe_print(f"MANUAL [RC signal missing!] ser={getattr(ser, 'port', None)} pca={'OK' if pca else 'FAIL'} [RTK:{fix_status}]")
                    last_dbg = now

        # BALLOON(비전) 모드 control
        elif current_mode == "BALLOON":
            with _cam_lock:
                cam_ok = _vision_state["ok"] and ((now - _vision_state["last_frame_t"]) < CAM_TIMEOUT)
                b_detected = _vision_state["detected"]
                b_nx = _vision_state["norm_x"]
                b_ny = _vision_state["norm_y"]
                b_rad = _vision_state["radius"]
                b_area = _vision_state["area_pct"]
                b_dist = _vision_state["dist_m"]
                b_ang_err = _vision_state["angle_err"]
                b_d_ang_err = _vision_state["d_angle_err"]

            if not cam_ok:
                safe_stop()
                current_mode = "MANUAL"
                balloon_substate = "IDLE"
                add_event_log("FAILSAFE", "Pi camera timeout -> abort BALLOON, switch to MANUAL", "warn")
            else:
                if b_detected:
                    balloon_last_seen_t = now
                    balloon_last_angle_err = b_ang_err
                    t_dist = b_dist
                    angle_err = b_ang_err
                    if heading is not None:
                        target_angle = (heading + b_ang_err) % 360.0

                    # 목표 거리 도달 시 정지 및 MANUAL 전환
                    if BALLOON_ACTION_MODE == "APPROACH_STOP" and (
                        b_dist <= BALLOON_STOP_DIST_M or b_area >= BALLOON_STOP_AREA_PCT
                    ):
                        balloon_arrive_count += 1
                    else:
                        balloon_arrive_count = 0

                    if BALLOON_ACTION_MODE == "APPROACH_STOP" and balloon_arrive_count >= 5:
                        safe_stop()
                        current_mode = "MANUAL"
                        balloon_substate = "FINISH"
                        balloon_arrive_count = 0
                        steer, drive = CENTER, STOP
                        finish_msg = f"Approach complete! (range {b_dist:.2f}m / area {b_area:.1f}%) -> safe stop and MANUAL"
                        with state_lock:
                            shared_state["balloonFinishSeq"] = shared_state.get("balloonFinishSeq", 0) + 1
                            shared_state["balloonFinishMsg"] = finish_msg
                        add_event_log("FINISH", finish_msg, "auto")
                    else:
                        steer = balloon_steer(b_ang_err, b_d_ang_err)
                        drive = balloon_drive(b_ang_err, b_area, b_dist)
                        if abs(b_ang_err) <= BALLOON_CHARGE_ANGLE_DEG or b_area >= 2.0:
                            balloon_substate = "CHARGE"
                        else:
                            balloon_substate = "TRACKING"
                else:
                    balloon_arrive_count = 0
                    since_seen = (now - balloon_last_seen_t) if balloon_last_seen_t > 0.0 else 999.0
                    if (
                        BALLOON_ACTION_MODE == "RAM_POP"
                        and since_seen <= BALLOON_POP_FOLLOW_SEC
                        and abs(balloon_last_angle_err) <= BALLOON_CHARGE_ANGLE_DEG
                    ):
                        # RAM_POP 돌진 유지
                        balloon_substate = "CHARGE"
                        angle_err = balloon_last_angle_err * 0.5
                        steer = balloon_steer(angle_err, 0.0)
                        drive = max_GO
                    elif since_seen <= BALLOON_MEMORY_SEC:
                        # 일시 소실 시 직전 방향 유지
                        balloon_substate = "TRACKING"
                        angle_err = balloon_last_angle_err * 0.8
                        steer = balloon_steer(angle_err, 0.0)
                        drive = lerp(STOP, max_GO, 0.65)
                    else:
                        # 미검출 시 저속 선회 탐색
                        balloon_substate = "SEARCHING"
                        ref_t = balloon_last_seen_t if balloon_last_seen_t > 0.0 else balloon_mode_start_t
                        if (now - ref_t) >= BALLOON_SEARCH_TIMEOUT_SEC:
                            safe_stop()
                            current_mode = "MANUAL"
                            balloon_substate = "IDLE"
                            add_event_log("BALLOON", f"{int(BALLOON_SEARCH_TIMEOUT_SEC)}s with no pink balloon found -> safe stop and MANUAL", "warn")
                            steer, drive = CENTER, STOP
                        else:
                            scan_dir = 1 if balloon_last_angle_err >= 0 else -1
                            angle_err = scan_dir * 10.0
                            steer = clamp(CENTER - int(STEER_SIGN * scan_dir * 5000), max_LEFT, max_RIGHT)
                            drive = lerp(STOP, max_GO, 0.30)

                if current_mode == "BALLOON":
                    # 고출력 전진 중 걸림(Stall) 감시
                    if gps_ok and balloon_substate in ("CHARGE", "TRACKING") and drive > STOP + 160:
                        if stall_check_start_t == 0.0 or stall_ref_lat is None:
                            stall_check_start_t = now
                            stall_ref_lat, stall_ref_lon = c_lat, c_lon
                        else:
                            moved_since_ref = haversine(stall_ref_lat, stall_ref_lon, c_lat, c_lon)
                            if moved_since_ref >= STALL_MIN_MOVED_M or spd_mps >= 0.2:
                                stall_check_start_t = now
                                stall_ref_lat, stall_ref_lon = c_lat, c_lon
                                shared_state["lampStall"] = "ok"
                            elif (now - stall_check_start_t) >= STALL_TIMEOUT_SEC * 1.6:
                                safe_stop()
                                current_mode = "MANUAL"
                                balloon_substate = "IDLE"
                                stall_check_start_t = 0.0
                                shared_state["lampStall"] = "err"
                                add_event_log("STALL", "Boat stalled ≥8s (BALLOON) -> safe stop and MANUAL", "warn")
                    else:
                        stall_check_start_t = 0.0

                if current_mode == "BALLOON":
                    if ready and (now - last_pwm_t) >= PWM_PERIOD:
                        set_pwm(steer, drive)
                        last_pwm_t = now

                    if now - last_dbg > 0.5:
                        safe_print(
                            f"BALLOON [{balloon_substate}] det={b_detected} X={b_nx:+.2f} "
                            f"area={b_area:.1f}% dist={b_dist:.2f}m err={angle_err:+.1f}deg pwm={steer},{drive}"
                        )
                        last_dbg = now
                        current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        val_lat = f"{c_lat:.7f}" if c_lat is not None else "0.0"
                        val_lon = f"{c_lon:.7f}" if c_lon is not None else "0.0"
                        try:
                            log_file.write(
                                f"{current_time},{current_mode},Balloon_{balloon_substate},{current_point + 1},"
                                f"{t_dist:.1f},{angle_err:.1f},{val_lat},{val_lon},{fix_status},{spd_mps:.2f},{steer},{drive}"
                                f"{LOG_NAV_BLANK}\n"
                            )
                        except Exception:
                            pass

        # AUTO mode: shared multipath navigator + selected strategy (same control as multipath_boat.py)
        else:
            balloon_substate = "IDLE"
            if not gps_ok:
                safe_stop()
                current_mode = "MANUAL"
                add_event_log("FAILSAFE", "GPS timeout -> immediate stop and MANUAL", "warn")
            else:
                fix_abort = False
                if not (2 <= fix_status <= 5):
                    if bad_fix_start_t == 0.0:
                        bad_fix_start_t = now
                    elif (now - bad_fix_start_t) >= BAD_FIX_TIMEOUT:
                        safe_stop()
                        current_mode = "MANUAL"
                        bad_fix_start_t = 0.0
                        fix_abort = True
                        add_event_log("FAILSAFE", f"GPS fix quality dropped (fix={fix_status}) -> switch to MANUAL", "warn")
                else:
                    bad_fix_start_t = 0.0

                # Update at the PWM rate (20 Hz): the strategies' steering slew limit assumes 20 Hz
                if not fix_abort and (now - last_nav_t) >= PWM_PERIOD:
                    last_nav_t = now
                    nav_snap = NAV_GPS.get_snapshot()
                    if HEADING_SELECTOR is not None:
                        heading_info = HEADING_SELECTOR.select(nav_snap, now_mono)
                    else:
                        heading_info = {
                            "heading_deg": nav_snap["heading_deg"], "source": "legacy_blend", "age_sec": None,
                            "confident": nav_snap["heading_deg"] is not None, "state": "legacy",
                        }
                    prev_wp = current_point
                    nav_info = NAVIGATOR.update(
                        c_lat, c_lon, heading_info["heading_deg"], now_mono=now_mono,
                        heading_confident=heading_info["confident"], motion=heading_info,
                    )
                    target_path = LANE_NAMES[nav_info["active_lane"]]
                    current_point = NAVIGATOR.target_idx
                    if nav_info["shifted_to"] is not None:
                        add_event_log(
                            "PATH",
                            f"Disturbance AUTO lane switch -> {target_path} (CTE {nav_info['cte_right_m']:+.2f} m)",
                            "auto",
                        )
                    elif current_point != prev_wp and not nav_info["finished"]:
                        add_event_log("NAV", f"WP {prev_wp + 1} passed -> WP {current_point + 1} tracking", "auto")

                    # Values shown on the web page
                    nav_pts = NAVIGATOR.paths[nav_info["active_lane"]]
                    _, t_lat, t_lon = nav_pts[min(current_point, len(nav_pts) - 1)]
                    t_dist = haversine(c_lat, c_lon, t_lat, t_lon)
                    target_angle = nav_info["target_bearing_deg"]
                    if heading_info["heading_deg"] is not None:
                        heading = heading_info["heading_deg"]
                        angle_err = wrap180(target_angle - heading)
                    else:
                        angle_err = 0.0
                    bx, by = latlon_to_enu(c_lat, c_lon, NAVIGATOR.ref_lat, NAVIGATOR.ref_lon)
                    for lane_key, lane_name in LANE_NAMES.items():
                        _, lane_d, _, _ = NAVIGATOR._closest_segment_metrics(lane_key, bx, by)
                        shared_state["dist" + lane_name] = round(lane_d, 2)

                    if nav_info["finished"]:
                        safe_stop()
                        current_mode = "MANUAL"
                        add_event_log(
                            "FINISH",
                            f"Finish crossed (WP {nav_info['wp_number']}, margin {NAVIGATOR.finish_margin_m:g} m) -> stop and MANUAL",
                            "auto",
                        )
                    else:
                        steer = nav_steer_to_pwm(nav_info["steer_norm"])
                        drive = nav_throttle_to_pwm(nav_info["throttle_blend"])
                        auto_abort = False

                        # No heading yet: cruise straight ahead to acquire one; stop if it takes too long
                        if heading_info["heading_deg"] is None:
                            if heading_none_start_t == 0.0:
                                heading_none_start_t = now
                            elif (now - heading_none_start_t) > HEADING_INIT_MAX_SEC:
                                safe_stop()
                                current_mode = "MANUAL"
                                auto_abort = True
                                add_event_log(
                                    "FAILSAFE",
                                    f"Heading not acquired (>{HEADING_INIT_MAX_SEC:g}s, {heading_info['state']}) -> safe stop and MANUAL",
                                    "warn",
                                )
                        else:
                            heading_none_start_t = 0.0

                        # Stall watchdog while driving forward
                        if not auto_abort:
                            if drive > STOP + 100:
                                if stall_check_start_t == 0.0 or stall_ref_lat is None:
                                    stall_check_start_t = now
                                    stall_ref_lat, stall_ref_lon = c_lat, c_lon
                                else:
                                    moved_since_ref = haversine(stall_ref_lat, stall_ref_lon, c_lat, c_lon)
                                    if moved_since_ref >= STALL_MIN_MOVED_M or spd_mps >= 0.2:
                                        stall_check_start_t = now
                                        stall_ref_lat, stall_ref_lon = c_lat, c_lon
                                        shared_state["lampStall"] = "ok"
                                    elif (now - stall_check_start_t) >= STALL_TIMEOUT_SEC:
                                        safe_stop()
                                        current_mode = "MANUAL"
                                        stall_check_start_t = 0.0
                                        auto_abort = True
                                        shared_state["lampStall"] = "err"
                                        add_event_log("STALL", "Boat stalled ≥5s -> MANUAL", "warn")
                            else:
                                stall_check_start_t = 0.0

                        if not auto_abort:
                            if ready:
                                set_pwm(steer, drive)
                                last_pwm_t = now

                            if now - last_dbg > 0.5:
                                fix_str = "FIX" if fix_status == 4 else "FLT" if fix_status == 5 else "DGN" if fix_status == 2 else str(fix_status)
                                guard = " [WALL]" if nav_info["boundary_guard"] else ""
                                safe_print(
                                    f"AUTO {AUTO_STRATEGY.name} P:{target_path}{guard} WP:{current_point + 1}/{total_waypoints} "
                                    f"CTE:{nav_info['cte_right_m']:+.2f}m ERR:{angle_err:+.1f}deg "
                                    f"HDG:{heading_info['source']} BOOST:{nav_info['throttle_blend']:.2f} "
                                    f"pwm={steer},{drive} [{fix_str}]"
                                )
                                last_dbg = now
                                current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                                try:
                                    log_file.write(
                                        f"{current_time},{current_mode},{target_path},{current_point + 1},{t_dist:.1f},"
                                        f"{angle_err:.1f},{c_lat:.7f},{c_lon:.7f},{fix_status},{spd_mps:.2f},{steer},{drive},"
                                        f"{nav_info['cte_right_m']:.3f},{AUTO_STRATEGY.name},{nav_info['throttle_blend']:.3f},"
                                        f"{heading_info['source']},{int(nav_info['boundary_guard'])}\n"
                                    )
                                except Exception:
                                    pass

        # Leaving AUTO (finish, failsafe, override, E-stop, mode change): reset heading/strategy state
        if was_auto and current_mode != "AUTO":
            rc_auto_active = False
            rc_entry_steer_held = False
            nav_info = None
            if HEADING_SELECTOR is not None:
                HEADING_SELECTOR.reset()
            AUTO_STRATEGY.suspend()
        shared_state["rcAutoActive"] = rc_auto_active and current_mode == "AUTO"
        shared_state["rcManualActive"] = rc_manual_latched
        was_auto = (current_mode == "AUTO")
        # Outside AUTO, watch the actual forward command so the next AUTO entry can reuse a fresh GPS course
        if HEADING_SELECTOR is not None and current_mode != "AUTO":
            thr_sign = 0 if _last_drive is None else (1 if _last_drive > STOP else (-1 if _last_drive < STOP else 0))
            HEADING_SELECTOR.observe_manual(NAV_GPS.get_snapshot(), thr_sign, now_mono)

        # 5200 고출력 연속 주행 시간 감시 (10분 초과 시 4800 자동 하향)
        dt_hp = clamp(now - prev_loop_t, 0.0, 0.2)
        prev_loop_t = now
        if max_GO >= 5200 and not estop_active and (_last_drive is not None and _last_drive >= 5100):
            high_power_drive_sec += dt_hp
            if high_power_drive_sec >= 300.0 and not high_power_warn_5m:
                high_power_warn_5m = True
                add_event_log("THERMAL", "5200 high thrust for 5 min -> watch motor/ESC heat", "warn")
            if high_power_drive_sec >= 480.0 and not high_power_warn_8m:
                high_power_warn_8m = True
                add_event_log("THERMAL", "5200 high thrust for 8 min -> auto-limit to 4800 in 2 min", "warn")
            if high_power_drive_sec >= MAX_GO_CONTINUOUS_SEC:
                max_GO = DEFAULT_MAX_GO
                high_power_drive_sec = 0.0
                high_power_warn_5m = False
                high_power_warn_8m = False
                add_event_log("THERMAL", "5200 high-thrust 10 min limit -> auto-reduce to 4800 for thermal protection", "warn")
        else:
            high_power_drive_sec = max(0.0, high_power_drive_sec - dt_hp * 0.5)
            if high_power_drive_sec < 60.0:
                high_power_warn_5m = False
                high_power_warn_8m = False

        with state_lock:
            shared_state["mode"] = current_mode
            shared_state["estop"] = estop_active
            shared_state["speed"] = round(spd_mps, 2)
            if spd_mps > shared_state["maxSpeed"]:
                shared_state["maxSpeed"] = round(spd_mps, 2)
            shared_state["movedDist"] = round(total_moved_m, 1)
            shared_state["lapElapsedSec"] = round(now - auto_start_time, 1)
            shared_state["stPulse"] = ST_V
            shared_state["thPulse"] = TH_V
            shared_state["rawSt"] = last_raw_st if rc_ok else None
            shared_state["rawTh"] = last_raw_th if rc_ok else None
            shared_state["targetPath"] = target_path
            shared_state["pathAutoSwitch"] = path_auto_switch
            shared_state["wpIndex"] = min(current_point + 1, total_waypoints)
            shared_state["wpDist"] = round(t_dist, 2)
            shared_state["fixStatus"] = fix_status
            if c_lat is not None and c_lon is not None:
                shared_state["lat"] = c_lat
                shared_state["lon"] = c_lon
            if heading is not None:
                shared_state["heading"] = round(heading, 1)
            shared_state["targetAngle"] = round(target_angle, 1)
            shared_state["angleErr"] = round(angle_err, 1)
            shared_state["balloonState"] = balloon_substate
            shared_state["steerCtrlMode"] = STEER_CTRL_MODE
            shared_state["pidKp"] = PID_KP
            shared_state["pidKi"] = PID_KI
            shared_state["pidKd"] = PID_KD
            shared_state["pidIMax"] = PID_I_MAX_DEG
            shared_state["pidPTerm"] = round(_pid_last_p, 1)
            shared_state["pidITerm"] = round(_pid_last_i, 1)
            shared_state["pidDTerm"] = round(_pid_last_d, 1)
            shared_state["maxGo"] = max_GO
            shared_state["highPowerElapsedSec"] = round(high_power_drive_sec, 1)
            shared_state["camLteMode"] = _cam_lte_mode
            shared_state["balloonActionMode"] = BALLOON_ACTION_MODE
            shared_state["balloonRealDiamM"] = BALLOON_REAL_DIAM_M
            shared_state["balloonStopDistM"] = BALLOON_STOP_DIST_M
            shared_state["balloonStopAreaPct"] = BALLOON_STOP_AREA_PCT
            shared_state["customColorEnable"] = CUSTOM_COLOR_ENABLE
            shared_state["targetHue"] = TARGET_HUE
            shared_state["targetHueTol"] = TARGET_HUE_TOL
            shared_state["targetSatMin"] = TARGET_SAT_MIN
            shared_state["targetValMin"] = TARGET_VAL_MIN
            shared_state["targetColorHex"] = TARGET_COLOR_HEX

        # 50Hz 주기 유지
        elapsed = time.time() - now
        time.sleep(max(0.001, 0.02 - elapsed))

if __name__ == "__main__":
    try:
        main_loop()
    except (KeyboardInterrupt, SystemExit):
        safe_print("\n[EXIT] Shutdown signal received; performing safe stop.")
    finally:
        safe_stop()
        release_active_camera()
        try:
            log_file.flush()
            log_file.close()
        except Exception:
            pass
        if pca is not None:
            try:
                pca.close()
            except Exception:
                pass
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        if gps_ser is not None:
            try:
                gps_ser.close()
            except Exception:
                pass
