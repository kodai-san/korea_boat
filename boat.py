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

try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    HW_AVAILABLE = True
except Exception:
    board = None
    busio = None
    PCA9685 = None
    HW_AVAILABLE = False

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

# 조향 제어 파라미터 (P / PID)
PID_CONFIG_FILE = os.path.join(BASE_DIR, "pid_config.json")
STEER_CTRL_MODE = "P"
PID_KP = 15.0
PID_KI = 0.0
PID_KD = 3.0
PID_I_MAX_DEG = 15.0

ST_MIN, ST_NEU, ST_MAX = 835, 1360, 1891
TH_MIN, TH_NEU, TH_MAX = 1086, 1356, 1621
TH_NEU_LOW, TH_NEU_HIGH = 1335, 1375

# RC 비상 오버라이드 임계값
ST_OVERRIDE_LEFT = 900
ST_OVERRIDE_RIGHT = 1820
TH_REV_OVERRIDE = 1550

RC_VALID_MIN = 750
RC_VALID_MAX = 2050
RC_DEADZONE = 80
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

RC_TIMEOUT = 1.0
GPS_TIMEOUT = 1.0
BAD_FIX_TIMEOUT = 1.0
MAX_GPS_GLITCH_DIST_M = 2.5
MAX_GPS_GLITCH_MPS = 8.0
HEADING_INIT_MAX_SEC = 5.0
STALL_TIMEOUT_SEC = 5.0
STALL_MIN_MOVED_M = 0.5
MAX_SERIAL_LINES_LOOP = 20
SERIAL_RECONNECT_SEC = 1.5

RC_PORT = "/dev/ttyUSB0"
GPS_PORT = "/dev/ttyACM0"
RC_BAUD = 115200
GPS_BAUD = 460800

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

PCA_ADDR = 0x40
PCA_I2C_BUS = 0

# 카메라(16:9 와이드) 및 비전 제어 상수
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

BALLOON_STEER_GAIN = 18.0
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

# 시리얼 포트 연결 (흐름 제어 비활성화 및 중복 포트 제외)
def open_serial_with_fallback(preferred_port, baud, prefix, exclude_port=None):
    if serial is None:
        return None
    candidates = [preferred_port]
    for p in sorted(glob.glob(f"/dev/{prefix}*")):
        if p not in candidates and p != exclude_port:
            candidates.append(p)
    for port_name in candidates:
        if exclude_port and port_name == exclude_port:
            continue
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
            add_event_log("SERIAL", f"시리얼 포트 연결 성공: {port_name} ({baud} bps)", "auto")
            return s
        except Exception:
            continue
    return None

# PCA9685 초기화 및 버스 자동 탐색 (Bus 0 / Bus 1)
def init_pca9685():
    global pca
    if not HW_AVAILABLE:
        return False
    builders = []
    try:
        from adafruit_extended_bus import ExtendedI2C
        builders.append(("ExtendedI2C(0)", lambda: ExtendedI2C(0)))
        builders.append(("ExtendedI2C(1)", lambda: ExtendedI2C(1)))
    except Exception:
        pass
    if hasattr(board, "SCL_1") and hasattr(board, "SDA_1"):
        builders.append(("board.SCL_1/SDA_1(Bus0)", lambda: busio.I2C(board.SCL_1, board.SDA_1)))
    if hasattr(board, "SCL") and hasattr(board, "SDA"):
        builders.append(("board.SCL/SDA(Bus1)", lambda: busio.I2C(board.SCL, board.SDA)))

    last_err = None
    for name, fn in builders:
        try:
            b = fn()
            cand = PCA9685(b, address=PCA_ADDR)
            cand.frequency = 50.0
            pca = cand
            shared_state["lampSerial"] = "ok"
            add_event_log("I2C", f"PCA9685 모터 드라이버 연결 성공 ({name}, 0x{PCA_ADDR:02x})", "auto")
            return True
        except Exception as e:
            last_err = e
    pca = None
    shared_state["lampSerial"] = "err"
    add_event_log("I2C", f"PCA9685 응답 없음 (Bus 0/1 모두 실패: {last_err}) -> I2C 배선/전원 확인!", "warn")
    return False

# 주변장치(RC, GPS, PCA9685) 초기화
def init_peripherals():
    global ser, gps_ser, pca
    ser = open_serial_with_fallback(RC_PORT, RC_BAUD, "ttyUSB", exclude_port=GPS_PORT)
    rc_used_port = getattr(ser, "port", None) or RC_PORT
    gps_ser = open_serial_with_fallback(GPS_PORT, GPS_BAUD, "ttyACM", exclude_port=rc_used_port)
    init_pca9685()

# PWM 상수 유효성 확인
def pwm_ready():
    vals = (CENTER, max_LEFT, max_RIGHT, STOP, max_GO, max_BACK)
    return not any(v <= 0 for v in vals)

log_filename = os.path.join(BASE_DIR, f"auto_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
log_file = open(log_filename, "w", encoding="utf-8", buffering=8192)
log_file.write("Time,Mode,Path,WP,Distance,Error,Lat,Lon,RTK,Speed_mps,PWM_Steer,PWM_Thr\n")

# 범위 제한
def clamp(v, lo, hi):
    if lo > hi:
        lo, hi = hi, lo
    return max(lo, min(hi, v))

# 선형 보간
def lerp(a, b, t):
    t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t
    return int(a + (b - a) * t)

# RC 펄스 -> PWM 변환
def map_pulse_to_pwm(pulse, in_min, in_max, out_min, out_max, neutral, deadzone=15, out_neutral=None):
    if out_neutral is None:
        out_neutral = (out_min + out_max) / 2.0
    if (neutral - deadzone) <= pulse <= (neutral + deadzone):
        return int(out_neutral)
    pulse = clamp(pulse, in_min, in_max)
    if pulse < (neutral - deadzone):
        denom = (neutral - deadzone) - in_min
        if denom <= 0:
            return int(out_min)
        t = (pulse - in_min) / float(denom)
        return int(out_min + (out_neutral - out_min) * t)
    else:
        denom = in_max - (neutral + deadzone)
        if denom <= 0:
            return int(out_max)
        t = (pulse - (neutral + deadzone)) / float(denom)
        return int(out_neutral + (out_max - out_neutral) * t)

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
        raise ValueError(f"{filename} 파일이 비어 있습니다.")

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
            if c in ("lat", "latitude", "위도") or "lat" in c:
                lat_idx = i
            elif c in ("lon", "lng", "long", "longitude", "경도") or "lon" in c or "lng" in c:
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
        raise ValueError(f"{filename}에서 유효한 좌표를 읽지 못했습니다.")
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
            gps_used_port = getattr(gps_ser, "port", None) or GPS_PORT
            ser = open_serial_with_fallback(RC_PORT, RC_BAUD, "ttyUSB", exclude_port=gps_used_port)
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
        add_event_log("SERIAL", f"RC 포트 순단 감지 ({e})", "warn")
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
                add_event_log("SERIAL", f"RC 무응답({int(backoff_sec)}초) -> 시리얼 버퍼 정리 #{_rc_silent_reset_count}", "warn")
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
            rc_used_port = getattr(ser, "port", None) or RC_PORT
            with gps_ser_lock:
                gps_ser = open_serial_with_fallback(GPS_PORT, GPS_BAUD, "ttyACM", exclude_port=rc_used_port)
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
        add_event_log("SERIAL", f"GPS 포트 순단 감지 ({e})", "warn")
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
                    add_event_log("GLITCH", f"담벼락 반사 이상치 무시 #{_glitch_reject_count} ({jump_dist:.1f}m / {jump_mps:.1f}m/s)", "warn")
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
            add_event_log("NTRIP", f"RTCM3 보정 신호 수신 시작 ({NTRIP_MOUNT})", "ntrip")

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
        _last_steer = steer
        _last_drive = target_drive
        shared_state["steerPwm"] = _last_steer
        shared_state["drivePwm"] = _last_drive
        return

    try:
        if steer_need:
            pca.channels[0].duty_cycle = steer
            _last_steer = steer
        if drive_need:
            pca.channels[6].duty_cycle = target_drive
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
            pca.channels[0].duty_cycle = CENTER
            pca.channels[6].duty_cycle = STOP
            _last_steer, _last_drive = CENTER, STOP
            _need_safe_stop_retry = False
            return
        except Exception:
            time.sleep(0.005)

    _last_steer = None
    _last_drive = None
    _need_safe_stop_retry = True

# AUTO 전진 출력 계산
def auto_drive(angle_err):
    a = abs(angle_err)
    if a >= 70:
        t = 0.15
    elif a <= 15:
        t = 1.0
    else:
        t = 0.15 + 0.85 * (70.0 - a) / 55.0
    return lerp(STOP, max_GO, t)

_pid_integral = 0.0
_pid_prev_err = None
_pid_prev_t = 0.0
_pid_prev_err_t = 0.0
_pid_d_filt = 0.0
_pid_last_p = 0.0
_pid_last_i = 0.0
_pid_last_d = 0.0

# 제어/비전 설정 파일 저장
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
        add_event_log("CTRL", f"설정 파일 저장 실패 ({e})", "warn")

# 저장된 제어/비전 설정 로드
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
            f"저장된 설정 로드 [{STEER_CTRL_MODE}] Kp={PID_KP}, max_GO={max_GO}, 표적={BALLOON_REAL_DIAM_M}m ({BALLOON_ACTION_MODE}, 정지={BALLOON_STOP_DIST_M}m)",
            "auto",
        )
    except Exception as e:
        add_event_log("CTRL", f"설정 파일 로드 생략 ({e})", "warn")

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

# AUTO 조향 출력 계산 (P / PID)
def auto_steer(angle_err, now_t=None, spd_mps=0.0, gps_updated=True, wp_changed=False):
    global _pid_integral, _pid_prev_err, _pid_prev_t, _pid_prev_err_t, _pid_d_filt
    global _pid_last_p, _pid_last_i, _pid_last_d

    if now_t is None:
        now_t = time.time()

    if STEER_CTRL_MODE != "PID":
        p_term = angle_err * PID_KP
        _pid_last_p, _pid_last_i, _pid_last_d = p_term, 0.0, 0.0
        raw = CENTER - int(STEER_SIGN * p_term)
        return clamp(raw, max_LEFT, max_RIGHT)

    dt_loop = (now_t - _pid_prev_t) if (_pid_prev_t > 0.0 and (now_t - _pid_prev_t) <= 0.5) else 0.02
    dt_loop = max(0.005, dt_loop)
    _pid_prev_t = now_t

    # P항
    p_term = angle_err * PID_KP

    # I항 (저속 잠금 및 Anti-Windup)
    if PID_KI <= 0.0:
        _pid_integral = 0.0
        i_term = 0.0
    else:
        if _pid_prev_err is not None and not wp_changed and (angle_err * _pid_prev_err) < 0.0:
            _pid_integral *= 0.5
        if spd_mps >= 0.35 and abs(angle_err) < 60.0:
            _pid_integral = clamp(_pid_integral + angle_err * dt_loop, -PID_I_MAX_DEG, PID_I_MAX_DEG)
        i_term = _pid_integral * PID_KI

    # D항 (GPS 주기 기준 변화율 및 LPF)
    if PID_KD <= 0.0:
        _pid_d_filt = 0.0
        d_term = 0.0
        _pid_prev_err = angle_err
        _pid_prev_err_t = now_t
    elif _pid_prev_err is None or wp_changed:
        was_none = (_pid_prev_err is None)
        _pid_prev_err = angle_err
        _pid_prev_err_t = now_t
        if was_none:
            _pid_d_filt = 0.0
        d_term = _pid_d_filt * PID_KD
    else:
        if gps_updated or abs(wrap180(angle_err - _pid_prev_err)) > 1e-4:
            dt_gps = max(0.02, min(0.5, now_t - _pid_prev_err_t)) if _pid_prev_err_t > 0.0 else dt_loop
            delta_err = wrap180(angle_err - _pid_prev_err)
            raw_rate = clamp(delta_err / dt_gps, -45.0, 45.0)
            _pid_d_filt = 0.60 * _pid_d_filt + 0.40 * raw_rate
            _pid_prev_err = angle_err
            _pid_prev_err_t = now_t
        elif _pid_prev_err_t > 0.0 and (now_t - _pid_prev_err_t) > 0.5:
            _pid_d_filt *= 0.9
        d_term = _pid_d_filt * PID_KD

    _pid_last_p, _pid_last_i, _pid_last_d = p_term, i_term, d_term

    total_term = p_term + i_term + d_term
    raw = CENTER - int(STEER_SIGN * total_term)
    return clamp(raw, max_LEFT, max_RIGHT)

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

# WP 도달 판정 및 인덱스 갱신
def advance_index(c_lat, c_lon, path_name, idx):
    path_list = PATHS[path_name]
    while idx < total_waypoints:
        lat, lon = path_list[idx]
        d_cur = haversine(c_lat, c_lon, lat, lon)
        if d_cur <= ARRIVE_DIST_M:
            idx += 1
            continue
        if idx + 1 < total_waypoints and d_cur <= ARRIVE_DIST_M * 2.5:
            nlat, nlon = path_list[idx + 1]
            d_next = haversine(c_lat, c_lon, nlat, nlon)
            if d_next + 0.3 < d_cur:
                idx += 1
        break
    return idx

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

# 전방 최근접 WP 탐색
def find_forward_waypoint(c_lat, c_lon, path_name, current_idx):
    if c_lat is None or c_lon is None:
        return current_idx if current_idx < total_waypoints else 0
    path_list = PATHS[path_name]
    start_i = 0 if current_idx >= total_waypoints else max(0, current_idx)

    best_idx = start_i
    best_dist = float("inf")
    for i in range(start_i, total_waypoints):
        lat, lon = path_list[i]
        d = haversine(c_lat, c_lon, lat, lon)
        if d < best_dist:
            best_dist = d
            best_idx = i
    return best_idx

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
        return False, "카메라 프레임이 준비되지 않았습니다."

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
    return True, f"화면 클릭 색상 추출 완료 (H:{TARGET_HUE} ±{TARGET_HUE_TOL}, S≥{TARGET_SAT_MIN}, V≥{TARGET_VAL_MIN}, {TARGET_COLOR_HEX})"
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
                            f"CSI 카메라 연결 성공 (CAM{sid}/dev/video{sid}, {sw}x{sh}->{CAM_WIDTH}x{CAM_HEIGHT}, flip={flip_method})",
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
                    f"CSI 파이 카메라(gst-launch 파이프) 연결 성공 (CAM{sid}/dev/video{sid}, flip={flip_method})",
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
                add_event_log("CAMERA", "V4L2 카메라 연결 성공 (/dev/video0)", "auto")
                return cap

    # Other USB nodes
    for dev_idx in (1, 2):
        cap = _try_v4l2(dev_idx)
        if cap is not None:
            add_event_log("CAMERA", f"V4L2/USB 카메라 연결 성공 (/dev/video{dev_idx})", "auto")
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
        add_event_log("CAMERA", "OpenCV(cv2) 미설치 -> 카메라 비전 비활성화", "warn")
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
                        add_event_log("CAMERA", "카메라 미장착 감지 -> 비전 탐색 자동 중지 (MANUAL/AUTO 정상 작동)", "warn")
            time.sleep(0.1)
            continue

        ret, frame = cap.read()
        if not ret or frame is None or frame.size == 0:
            add_event_log("CAMERA", "파이 카메라 프레임 수신 끊김 -> 재연결 대기", "warn")
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
            add_event_log("WEB", f"웹 관제 서버 시작 -> 원격(Tailscale): http://{ts_ip}:{WEB_PORT} | 로컬: http://{local_ip}:{WEB_PORT}", "auto")
        else:
            add_event_log("WEB", f"웹 관제 서버 시작 -> http://{local_ip}:{WEB_PORT} (Tailscale 연결 시 100.x.y.z:8080 접속 가능)", "auto")
        server.serve_forever()
    except Exception as e:
        add_event_log("WEB", f"웹 서버 시작 실패 ({e})", "warn")

# 50Hz 메인 제어 루프
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

    balloon_last_seen_t = 0.0
    balloon_mode_start_t = 0.0
    balloon_last_angle_err = 0.0
    balloon_substate = "IDLE"
    balloon_arrive_count = 0
    mode_enter_t = 0.0
    override_armed = False
    web_manual_steer = CENTER
    web_manual_drive = STOP
    web_manual_until_t = 0.0
    web_manual_active = False
    web_manual_stop_hold_t = 0.0
    rc_ignore_mode = False
    gcs_watchdog_enabled = True
    gcs_link_paused = False

    ready = pwm_ready()
    if ready:
        safe_stop()
        time.sleep(2.0)

    load_pid_config()
    gcs_watchdog_enabled = bool(shared_state.get("linkWatchdog", True))

    if NTRIP_ENABLE:
        threading.Thread(target=ntrip_worker, daemon=True).start()

    if WEB_ENABLE:
        threading.Thread(target=web_server_worker, daemon=True).start()

    if CAM_ENABLE:
        threading.Thread(target=camera_worker, daemon=True).start()

    add_event_log("START", "메인 제어 루프 시작 (50Hz, MANUAL)", "auto")
    loop_counter = 0
    loop_rate_t = time.time()

    while True:
        now = time.time()
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

        # 1. RC 수신 (웹 단독 모드 시 불필요한 시리얼 재연결 폴링 생략)
        raw_st, raw_th = (None, None) if rc_ignore_mode else read_latest_rc()
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

        for _cmd_item in cmds_to_process:
          cmd = _cmd_item["cmd"]
          cmd_val = _cmd_item["value"]
          if cmd == "estop_on":
              estop_active = True
              current_mode = "MANUAL"
              balloon_substate = "IDLE"
              reset_pid_state()
              safe_stop()
              add_event_log("E-STOP", "비상 정지 작동! 모든 모터 차단", "warn")
          elif cmd == "estop_off":
              estop_active = False
              current_mode = "MANUAL"
              balloon_substate = "IDLE"
              reset_pid_state()
              safe_stop()
              add_event_log("E-STOP", "비상 정지 해제 -> MANUAL 대기", "auto")
          elif cmd == "set_mode" and not estop_active:
              if cmd_val == "AUTO" and current_mode != "AUTO":
                  if not gps_ok or not (2 <= fix_status <= 5):
                      safe_stop()
                      add_event_log("FAILSAFE", f"GPS 미준비(fix={fix_status}) -> AUTO 진입 거부됨", "warn")
                  else:
                      current_mode = "AUTO"
                      mode_enter_t = now
                      override_armed = False
                      reset_pid_state()
                      heading_none_start_t = now
                      stall_check_start_t = 0.0
                      bad_fix_start_t = 0.0
                      auto_start_time = now
                      # 진입 직후 오버라이드 오작동 방지
                      filt_st, filt_th = float(ST_NEU), float(TH_NEU)
                      last_raw_st, last_raw_th = ST_NEU, TH_NEU
                      ST_V, TH_V = ST_NEU, TH_NEU
                      raw_st, raw_th = None, None
                      # 저속 진입 시 COG만 초기화
                      if spd_mps < 0.5:
                          _last_cog = None
                          last_lat, last_lon = c_lat, c_lon
                      current_point = find_forward_waypoint(c_lat, c_lon, target_path, current_point)
                      if path_auto_switch and current_point < total_waypoints:
                          init_cand, _ = pick_path(c_lat, c_lon, current_point, target_path)
                          if init_cand != target_path:
                              target_path = init_cand
                              current_point = find_forward_waypoint(c_lat, c_lon, target_path, current_point)
                      with state_lock:
                          shared_state["mode"] = current_mode
                          shared_state["targetPath"] = target_path
                      mode_tag = "자동경로" if path_auto_switch else f"{target_path}고정"
                      add_event_log("MODE", f"웹 명령 -> AUTO 진입 (WP {current_point + 1}, {target_path} [{mode_tag}])", "auto")
              elif cmd_val == "MANUAL" and current_mode != "MANUAL":
                  current_mode = "MANUAL"
                  balloon_substate = "IDLE"
                  with state_lock:
                      shared_state["mode"] = current_mode
                  safe_stop()
                  add_event_log("MODE", "웹 명령 -> MANUAL 전환", "warn")
              elif cmd_val == "BALLOON" and current_mode != "BALLOON":
                  with _cam_lock:
                      cam_ready = _vision_state["ok"] and ((now - _vision_state["last_frame_t"]) < CAM_TIMEOUT)
                      b_det_now = _vision_state["detected"]
                  if not cam_ready:
                      safe_stop()
                      add_event_log("FAILSAFE", "파이 카메라 미준비 -> BALLOON 모드 진입 거부됨", "warn")
                  else:
                      current_mode = "BALLOON"
                      mode_enter_t = now
                      override_armed = False
                      stall_check_start_t = 0.0
                      balloon_arrive_count = 0
                      balloon_mode_start_t = now
                      balloon_last_seen_t = now if b_det_now else 0.0
                      balloon_substate = "TRACKING" if b_det_now else "SEARCHING"
                      filt_st, filt_th = float(ST_NEU), float(TH_NEU)
                      last_raw_st, last_raw_th = ST_NEU, TH_NEU
                      ST_V, TH_V = ST_NEU, TH_NEU
                      raw_st, raw_th = None, None
                      with state_lock:
                          shared_state["mode"] = current_mode
                      add_event_log("BALLOON", "웹 명령 -> VISION (광학 표적 추종) 모드 진입", "auto")
          elif cmd == "reset_wp":
              current_point = 0
              total_moved_m = 0.0
              auto_start_time = now
              reset_pid_state()
              add_event_log("WP리셋", "WP 1로 초기화됨", "auto")
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
                  add_event_log("PATH", f"자율 경로 자동 전환(AUTO) 활성화 -> 현재 '{target_path}'", "auto")
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
                  mode_str = "외란 자동전환" if path_auto_switch else "수동 고정"
                  add_event_log("PATH", f"추종 경로 '{target_path}' 설정 ({mode_str})", "auto")
              elif cmd_val in PATHS:
                  target_path = cmd_val
                  path_auto_switch = False
                  path_switch_candidate = None
                  path_switch_count = 0
                  reset_pid_state()
                  with state_lock:
                      shared_state["targetPath"] = target_path
                      shared_state["pathAutoSwitch"] = False
                  add_event_log("PATH", f"추종 경로 '{target_path}' 수동 고정 (외란 자동전환 해제)", "auto")
          elif cmd == "set_ctrl_mode" and cmd_val in ("P", "PID"):
              STEER_CTRL_MODE = cmd_val
              reset_pid_state()
              with state_lock:
                  shared_state["steerCtrlMode"] = STEER_CTRL_MODE
              save_pid_config()
              add_event_log("CTRL", f"AUTO 조향 제어 방식 변경 및 저장 -> {STEER_CTRL_MODE} 제어", "auto")
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
                      f"조향 설정 적용 및 파일 저장 완료 [{STEER_CTRL_MODE}] Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}, IMax={PID_I_MAX_DEG}°",
                      "auto",
                  )
              except (ValueError, TypeError) as e:
                  add_event_log("CTRL", f"PID 파라미터 형식 오류 무시됨 ({e})", "warn")
          elif cmd == "reset_pid_i":
              reset_pid_state()
              add_event_log("CTRL", "PID 적분(I) 누적값 및 미분 상태가 0으로 초기화되었습니다.", "auto")
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
                      add_event_log("DRIVE", f"고출력 프리셋({max_GO}, {pct_str}%) 활성화 -> 전 모드 적용 (10분 이상 연속 주행 금지)", "warn")
                  else:
                      add_event_log("DRIVE", f"모터 최대 전진 속도(max_GO) 변경 -> {max_GO} (기본 대비 {pct_str}%, 전 모드 적용)", "auto")
              except (ValueError, TypeError) as e:
                  add_event_log("DRIVE", f"max_GO 설정 오류 무시됨 ({e})", "warn")
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
                  mode_kor = "근접 정지(주행완료)" if BALLOON_ACTION_MODE == "APPROACH_STOP" else "돌진 타격(터뜨리기)"
                  add_event_log(
                      "VISION",
                      f"표적/추진 설정 저장: [{mode_kor}] 표적직경={BALLOON_REAL_DIAM_M}m, 정지거리={BALLOON_STOP_DIST_M}m, max_GO={max_GO}",
                      "auto",
                  )
              except (ValueError, TypeError) as e:
                  add_event_log("VISION", f"비전 표적 설정 형식 오류 무시됨 ({e})", "warn")
          elif cmd == "set_hsv_preset" and cmd_val in HSV_PRESETS:
              _hsv_preset_name = cmd_val
              CUSTOM_COLOR_ENABLE = False
              with state_lock:
                  shared_state["hsvPreset"] = _hsv_preset_name
                  shared_state["customColorEnable"] = False
              save_pid_config()
              add_event_log("CAMERA", f"표적 HSV 프리셋 전환 -> '{_hsv_preset_name}'", "auto")
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
                      f"타겟 색상 지정 완료: H={TARGET_HUE}(±{TARGET_HUE_TOL}), S≥{TARGET_SAT_MIN}, V≥{TARGET_VAL_MIN} [{TARGET_COLOR_HEX}]",
                      "auto",
                  )
              except (ValueError, TypeError) as e:
                  add_event_log("VISION", f"타겟 색상 설정 오류 무시됨 ({e})", "warn")
          elif cmd == "pick_color_at" and isinstance(cmd_val, dict):
              try:
                  u_n = clamp(float(cmd_val.get("u", 0.5)), 0.0, 1.0)
                  v_n = clamp(float(cmd_val.get("v", 0.5)), 0.0, 1.0)
                  ok_pick, msg_pick = sample_color_from_latest_frame(u_n, v_n)
                  add_event_log("VISION", msg_pick, "auto" if ok_pick else "warn")
              except (ValueError, TypeError) as e:
                  add_event_log("VISION", f"클릭 색상 추출 오류 ({e})", "warn")
          elif cmd == "toggle_cam_flip":
              _cam_flip_method = 2 if _cam_flip_method == 0 else 0
              _cam_reopen_requested = True
              shared_state["camFlip"] = 180 if _cam_flip_method == 2 else 0
              add_event_log("CAMERA", f"파이 카메라 상하 반전 설정 -> {shared_state['camFlip']}도", "auto")
          elif cmd == "toggle_cam_mask":
              _cam_show_mask = not _cam_show_mask
              shared_state["camShowMask"] = _cam_show_mask
              add_event_log("CAMERA", f"파이 카메라 HSV 마스크 전체화면 표시 -> {'ON' if _cam_show_mask else 'OFF'}", "auto")
          elif cmd == "toggle_cam_lte":
              _cam_lte_mode = not _cam_lte_mode
              with state_lock:
                  shared_state["camLteMode"] = _cam_lte_mode
              mode_txt = "LTE 절약·저지연(12FPS)" if _cam_lte_mode else "고화질(25FPS)"
              add_event_log("CAMERA", f"카메라 스트리밍 전송 모드 -> {mode_txt}", "auto")
          elif cmd == "set_rc_ignore":
              rc_ignore_mode = bool(cmd_val)
              with state_lock:
                  shared_state["rcIgnore"] = rc_ignore_mode
              if rc_ignore_mode:
                  add_event_log("REMOTE", "웹 단독 제어 활성화 -> 조종기(RC) 전파 간섭·오버라이드 무시", "warn")
              else:
                  add_event_log("REMOTE", "조종기(RC) 연동 복구 -> RC 수동 및 비상 오버라이드 활성", "auto")
          elif cmd == "set_link_watchdog":
              gcs_watchdog_enabled = bool(cmd_val)
              if not gcs_watchdog_enabled:
                  gcs_link_paused = False
              with state_lock:
                  shared_state["linkWatchdog"] = gcs_watchdog_enabled
                  shared_state["linkPaused"] = gcs_link_paused
              save_pid_config()
              add_event_log("LINK", f"원격 통신 끊김 자동 정지(Watchdog) -> {'ON' if gcs_watchdog_enabled else 'OFF'}", "auto")
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
                      f"원격/저지연 설정 저장: 영상={'LTE(12fps)' if _cam_lte_mode else 'HQ(25fps)'}, 끊김정지={'ON' if gcs_watchdog_enabled else 'OFF'}({GCS_LINK_TIMEOUT_SEC}s), RC무시={'ON' if rc_ignore_mode else 'OFF'}",
                      "auto",
                  )
              except (ValueError, TypeError) as e:
                  add_event_log("REMOTE", f"원격 설정 형식 오류 무시됨 ({e})", "warn")
          elif cmd == "web_manual_drive" and isinstance(cmd_val, dict) and not estop_active:
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
                  if ready:
                      set_pwm(web_manual_steer, web_manual_drive, force=True)
                      last_pwm_t = now
              except Exception:
                  pass
          elif cmd == "web_manual_stop":
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
                    add_event_log("LINK", f"원격 관제 연결 끊김({since_gcs_hb:.1f}초) -> 연결 회복 전까지 제자리 정지", "warn")
            elif gcs_link_paused and _gcs_hb_streak >= 2:
                gcs_link_paused = False
                stall_check_start_t = 0.0
                heading_none_start_t = now
                bad_fix_start_t = 0.0
                reset_pid_state()
                add_event_log("LINK", f"원격 관제 연결 회복 완료 -> 정지 대기 해제 ({current_mode})", "auto")
        else:
            gcs_link_paused = False

        shared_state["lampLink"] = "err" if gcs_link_paused else ("ok" if _gcs_ever_connected else "warn")
        shared_state["linkPaused"] = gcs_link_paused
        shared_state["linkWatchdog"] = gcs_watchdog_enabled
        shared_state["linkTimeoutSec"] = GCS_LINK_TIMEOUT_SEC
        shared_state["rcIgnore"] = rc_ignore_mode

        if estop_active:
            safe_stop()
            with state_lock:
                shared_state["mode"] = "MANUAL"
                shared_state["estop"] = True
                shared_state["speed"] = 0.0
            elapsed = time.time() - now
            time.sleep(max(0.001, 0.02 - elapsed))
            continue

        # RC 비상 오버라이드 감시 (웹 단독 제어 시 RC 노이즈 무시)
        if current_mode in ("AUTO", "BALLOON") and rc_effective_ok and ST_V is not None and TH_V is not None:
            in_normal_zone = (
                (ST_OVERRIDE_LEFT <= ST_V <= ST_OVERRIDE_RIGHT)
                and (TH_V <= TH_REV_OVERRIDE)
                and (raw_st is None or (ST_OVERRIDE_LEFT <= raw_st <= ST_OVERRIDE_RIGHT))
                and (raw_th is None or (raw_th <= TH_REV_OVERRIDE))
            )
            if in_normal_zone:
                override_armed = True

            if override_armed and (now - mode_enter_t) >= 0.50:
                filt_steer_full = (ST_V < ST_OVERRIDE_LEFT) or (ST_V > ST_OVERRIDE_RIGHT)
                filt_rev_full = (TH_V > TH_REV_OVERRIDE)
                if filt_steer_full or filt_rev_full:
                    prev_mode = current_mode
                    current_mode = "MANUAL"
                    balloon_substate = "IDLE"
                    override_armed = False
                    last_pwm_t = 0.0
                    if prev_mode == "BALLOON":
                        add_event_log("OVERRIDE", f"조종기 비상 조작 감지(ST:{ST_V}, TH:{TH_V}) -> BALLOON 중단 및 MANUAL 전환", "warn")
                    else:
                        add_event_log("OVERRIDE", f"조종기 비상 조작 감지(ST:{ST_V}, TH:{TH_V}) -> 즉시 MANUAL 전환", "warn")

        # 원격 통신 끊김 시 연결 회복 전까지 제자리 정지 유지
        if gcs_link_paused:
            safe_stop()
            stall_check_start_t = 0.0
            heading_none_start_t = now
            bad_fix_start_t = 0.0
            if now - last_dbg > 1.0:
                safe_print(f"LINK_HOLD [원격 연결 회복 대기 중 -> 제자리 정지] mode={current_mode} [RTK:{fix_status}]")
                last_dbg = now

        # MANUAL 모드 제어 (누르는 동안만 작동하는 모멘터리 웹 조종 & RC 수동 지원)
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
                steer = map_pulse_to_pwm(ST_V, ST_MIN, ST_MAX, max_LEFT, max_RIGHT, ST_NEU, RC_DEADZONE, CENTER)
                drive = map_pulse_to_pwm(TH_V, TH_MIN, TH_MAX, max_GO, max_BACK, TH_NEU, RC_DEADZONE, STOP)
                if TH_NEU_LOW <= TH_V <= TH_NEU_HIGH:
                    drive = STOP
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
                        log_file.write(f"{current_time},{current_mode},{target_path},{current_point + 1},{t_dist:.1f},{angle_err:.1f},{val_lat},{val_lon},{fix_status},{spd_mps:.2f},{steer},{drive}\n")
                    except Exception:
                        pass
            else:
                safe_stop()
                if now - last_dbg > 1.5:
                    safe_print(f"MANUAL [RC 신호 미수신!] ser={getattr(ser, 'port', None)} pca={'OK' if pca else 'FAIL'} [RTK:{fix_status}]")
                    last_dbg = now

        # BALLOON(비전) 모드 제어
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
                add_event_log("FAILSAFE", "파이 카메라 수신 타임아웃 -> BALLOON 중단 및 MANUAL 전환", "warn")
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
                        finish_msg = f"표적 접근 주행 완료! (거리 {b_dist:.2f}m / 면적 {b_area:.1f}%) -> 안전 정지 및 MANUAL 전환"
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
                            add_event_log("BALLOON", f"{int(BALLOON_SEARCH_TIMEOUT_SEC)}초간 분홍 풍선 미발견 -> 안전 정지 및 MANUAL 전환", "warn")
                            steer, drive = CENTER, STOP
                        else:
                            scan_dir = 1 if balloon_last_angle_err >= 0 else -1
                            angle_err = scan_dir * 10.0
                            steer = clamp(CENTER - int(STEER_SIGN * scan_dir * 220), max_LEFT, max_RIGHT)
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
                                add_event_log("STALL", "8초 이상 보트 걸림 감지(BALLOON) -> 안전 정지 및 MANUAL 전환", "warn")
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
                                f"{t_dist:.1f},{angle_err:.1f},{val_lat},{val_lon},{fix_status},{spd_mps:.2f},{steer},{drive}\n"
                            )
                        except Exception:
                            pass

        # AUTO 모드 경로 추종
        else:
            balloon_substate = "IDLE" 
            if current_point >= total_waypoints:
                safe_stop()
                current_mode = "MANUAL"
                add_event_log("FINISH", f"최종 WP {total_waypoints} 도착 -> MANUAL 대기", "auto")
            elif not gps_ok:
                safe_stop()
                current_mode = "MANUAL"
                add_event_log("FAILSAFE", "GPS 수신 타임아웃 -> 즉시 정지 및 MANUAL 전환", "warn")
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
                        add_event_log("FAILSAFE", f"GPS 정밀도 하락(fix={fix_status}) -> MANUAL 전환", "warn")
                else:
                    bad_fix_start_t = 0.0

                if not fix_abort:
                    prev_wp = current_point
                    current_point = advance_index(c_lat, c_lon, target_path, current_point)

                    if current_point != prev_wp and current_point < total_waypoints:
                        add_event_log("NAV", f"WP {prev_wp + 1} 통과 -> WP {current_point + 1} 추종", "auto")

                    if current_point >= total_waypoints:
                        safe_stop()
                        current_mode = "MANUAL"
                        add_event_log("FINISH", f"최종 WP {total_waypoints} 완주 -> MANUAL 전환", "auto")
                    else:
                        cand_path, dists = pick_path(c_lat, c_lon, current_point, target_path)
                        if path_auto_switch and cand_path != target_path:
                            if gps:
                                if path_switch_candidate == cand_path:
                                    path_switch_count += 1
                                else:
                                    path_switch_candidate = cand_path
                                    path_switch_count = 1
                                if path_switch_count >= 2:
                                    add_event_log("PATH", f"외란 자동 경로 전환: {target_path} -> {cand_path}", "auto")
                                    target_path = cand_path
                                    path_switch_count = 0
                                    current_point = min(advance_index(c_lat, c_lon, target_path, current_point), total_waypoints - 1)
                        else:
                            path_switch_candidate = None
                            path_switch_count = 0

                        shared_state["distCenter"] = round(dists.get("Center", 0.0), 2)
                        shared_state["distLeft"] = round(dists.get("Left", 0.0), 2)
                        shared_state["distRight"] = round(dists.get("Right", 0.0), 2)

                        t_lat, t_lon = wp_coord(PATHS[target_path], current_point)
                        t_dist = haversine(c_lat, c_lon, t_lat, t_lon)
                        look_lat, look_lon = get_lookahead_target(c_lat, c_lon, PATHS[target_path], current_point)
                        target_angle = gps_bearing(c_lat, c_lon, look_lat, look_lon)

                        heading_abort = False
                        if heading is None:
                            if heading_none_start_t == 0.0:
                                heading_none_start_t = now
                            if last_lat is not None and last_lon is not None and haversine(last_lat, last_lon, c_lat, c_lon) >= HEADING_INIT_MOVE_M:
                                est_heading = gps_bearing(last_lat, last_lon, c_lat, c_lon)
                                last_lat, last_lon = c_lat, c_lon
                                heading = est_heading
                                heading_none_start_t = 0.0
                                angle_err = wrap180(target_angle - heading)
                                steer = auto_steer(
                                    angle_err,
                                    now_t=now,
                                    spd_mps=spd_mps,
                                    gps_updated=(gps is not None),
                                    wp_changed=(current_point != prev_wp),
                                )
                                drive = auto_drive(angle_err)
                            elif (now - heading_none_start_t) > min(HEADING_INIT_MAX_SEC, 2.5):
                                safe_stop()
                                current_mode = "MANUAL"
                                heading_abort = True
                                add_event_log("FAILSAFE", "초기 방향각 미확보(2.5초 초과) -> 안전 정지 및 MANUAL 전환", "warn")
                            else:
                                angle_err = 0.0
                                steer, drive = CENTER, lerp(STOP, max_GO, 0.20)
                        else:
                            heading_none_start_t = 0.0
                            angle_err = wrap180(target_angle - heading)
                            steer = auto_steer(
                                angle_err,
                                now_t=now,
                                spd_mps=spd_mps,
                                gps_updated=(gps is not None),
                                wp_changed=(current_point != prev_wp),
                            )
                            drive = auto_drive(angle_err)

                        if not heading_abort:
                            stall_abort = False
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
                                        stall_abort = True
                                        shared_state["lampStall"] = "err"
                                        add_event_log("STALL", "5초 이상 보트 걸림 감지 -> MANUAL 전환", "warn")
                            else:
                                stall_check_start_t = 0.0

                            if not stall_abort:
                                if ready and (now - last_pwm_t) >= PWM_PERIOD:
                                    set_pwm(steer, drive)
                                    last_pwm_t = now

                                if now - last_dbg > 0.5:
                                    fix_str = "FIX" if fix_status == 4 else "FLT" if fix_status == 5 else "DGN" if fix_status == 2 else str(fix_status)
                                    safe_print(f"AUTO P:{target_path} WP:{current_point + 1}/{total_waypoints} D:{t_dist:.1f}m ERR:{angle_err:.1f}deg [{fix_str}]")
                                    last_dbg = now
                                    current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                                    val_lat = f"{c_lat:.7f}" if c_lat is not None else "0.0"
                                    val_lon = f"{c_lon:.7f}" if c_lon is not None else "0.0"
                                    try:
                                        log_file.write(f"{current_time},{current_mode},{target_path},{current_point + 1},{t_dist:.1f},{angle_err:.1f},{val_lat},{val_lon},{fix_status},{spd_mps:.2f},{steer},{drive}\n")
                                    except Exception:
                                        pass

        # 5200 고출력 연속 주행 시간 감시 (10분 초과 시 4800 자동 하향)
        dt_hp = clamp(now - prev_loop_t, 0.0, 0.2)
        prev_loop_t = now
        if max_GO >= 5200 and not estop_active and (_last_drive is not None and _last_drive >= 5100):
            high_power_drive_sec += dt_hp
            if high_power_drive_sec >= 300.0 and not high_power_warn_5m:
                high_power_warn_5m = True
                add_event_log("THERMAL", "5200 고출력 연속 주행 5분 경과 -> 모터/ESC 발열 주의", "warn")
            if high_power_drive_sec >= 480.0 and not high_power_warn_8m:
                high_power_warn_8m = True
                add_event_log("THERMAL", "5200 고출력 연속 주행 8분 경과 -> 2분 후 4800으로 자동 감속", "warn")
            if high_power_drive_sec >= MAX_GO_CONTINUOUS_SEC:
                max_GO = DEFAULT_MAX_GO
                high_power_drive_sec = 0.0
                high_power_warn_5m = False
                high_power_warn_8m = False
                add_event_log("THERMAL", "5200 고출력 10분 한도 도달 -> 과열 보호를 위해 4800으로 자동 하향", "warn")
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
        safe_print("\n[종료] 프로그램 종료 신호를 수신하여 안전 정지합니다.")
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
                pca.deinit()
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