#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
multipath_boat.py
국립한국해양대학교 해수천 자율운항 보트 - 멀티패스(Left/Center/Right) 자율주행 메인 제어 코드

[핵심 동작 요약]
1. 수동 모드(MANUAL): 조종기(Spektrum SR215 -> Arduino) 조향·추진 신호를 그대로 추종
2. 수동 -> 자동 전환 : 추진 중립 유지 + 우조향 2초 이상 유지 시 자동 모드(AUTO) 진입
   - 진입 시 현재 위치에서 가장 가까운 경로(left / center / right) 및 전방 웨이포인트 자동 선택
   - GPS가 RTK Fix/Float가 아닌 일반 GPS/DGPS여도 경고만 출력하고 진입 허용 (미측위 0일 때만 차단)
3. 자동 모드(AUTO)  :
   - 조종기 조향(CH1) 신호는 완전히 무시하고, 추진(CH2) 풀 후진 입력만 감시하여 감지 즉시 수동 복귀
   - 외란(바람·파도) 없을 시 현재 경로 유지
   - 외란으로 횡방향 이탈(CTE) + 이동 방향 치우침이 일정 시간 지속되면 힘을 거스르지 않고 인접 경로로 이동
   - 양 끝 경로(left/right)에서 바깥쪽 벽 방향으로 더 밀릴 때는 경로 전환을 잠그고 복귀 조향 강화
   - 마지막 65번 웨이포인트 결승선 통과(또는 반경 진입) 시 중립 정지 후 자동으로 수동 모드 복귀
4. 국토지리정보원(NGII) RTK 보정(NTRIP): 내장 백그라운드 클라이언트가 $GNGGA 전송 및 RTCM3 주입 수행
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
# [사용자 설정 구역] 현장 상황에 맞게 바로 수정할 수 있는 파라미터
# =====================================================================

# 1) 국토지리정보원(NGII) 네트워크 RTK (NTRIP) 계정 설정
NTRIP_CONFIG = {
    "enabled": True,
    "host": "rts2.ngii.go.kr",        # VRS: rts1.ngii.go.kr / FKP: rts2.ngii.go.kr
    "port": 2101,
    "mountpoint": "VRS-RTCM32",       # 조에서 사용하는 마운트포인트명 입력 (예: VRS-RTCM31, FKP-RTCM31)
    "user": "yeseong5226",           # 국토정보플랫폼 아이디 입력
    "password": "ngii",               # 비밀번호 (기본 ngii 또는 설정한 비밀번호)
    "gga_interval_sec": 5.0           # VRS 서버로 현재 위치($GNGGA) 재전송 주기(초)
}

# 2) 경로 CSV 파일 경로 (같은 폴더 기준)
PATH_FILES = {
    "left": "left.csv",
    "center": "center.csv",
    "right": "right.csv"
}

# 3) 멀티패스 및 자율주행 제어 파라미터
NAV_PARAMS = {
    "lookahead_dist_m": 3.5,          # 기본 전방 주시 거리 (m) - 웨이포인트 간격(1.44m)의 약 2.5배
    "boundary_lookahead_m": 2.4,      # 양 끝 경로에서 벽 쪽으로 밀릴 때 복귀용 짧은 주시 거리 (m)
    "wp_reach_radius_m": 1.8,         # 웨이포인트 도달 반경 (m)
    "finish_radius_m": 1.8,           # 마지막 65번 웨이포인트 도착 판정 반경 (m)

    # 외란 감지 및 경로 변경 기준 (차선 간격 1.50m 기준)
    "shift_cte_m": 0.70,              # 현재 경로에서 좌/우 0.70m 이상 밀리면 외란 후보로 감지
    "shift_hold_sec": 1.2,            # 외란 상태가 1.2초 이상 지속되면 인접 경로로 자연스럽게 이동
    "shift_cooldown_sec": 3.0,        # 경로 이동 직후 연속 전환 방지 안정화 시간 (초)
    "drift_oppose_tol_deg": 5.0,      # 이미 경로 쪽으로 5도 이상 복귀 중이면 차선 변경 보류

    # 조향 제어 게인 및 GPS 헤딩 필터
    "max_steer_angle_deg": 35.0,      # 조향 오차가 35도일 때 서보 최대 타각(100%) 사용
    "boundary_gain_boost": 1.45,      # 끝 경로 바깥(벽 방향) 이탈 시 복귀 조향 강화 배수
    "servo_slew_per_cycle": 0.18,     # 급격한 서보 흔들림 방지용 주기당 최대 조향 변화량 (20Hz 기준)
    "min_heading_move_m": 0.25,       # 정지 노이즈 방지를 위한 GPS 헤딩 갱신 최소 이동 거리 (m)
    "sog_cog_blend_mps": 0.45         # $GNRMC COG를 신뢰하기 시작하는 최소 이동 속도 (m/s)
}

DEFAULT_CALIBRATION_FILE = "calibration.json"
CONTROL_LOOP_HZ = 20.0                # 메인 제어 주기 (20Hz = 50ms)

# =====================================================================


EARTH_RADIUS_M = 6_378_137.0

FIX_QUALITY_NAMES = {
    0: "미측위(No Fix)",
    1: "일반 GPS(SPS)",
    2: "DGPS(차분보정)",
    4: "RTK FIX(정밀)",
    5: "RTK FLOAT"
}


def wrap_180(angle_deg):
    """각도를 -180 ~ +180 범위로 정규화"""
    return (angle_deg + 180.0) % 360.0 - 180.0


def latlon_to_enu(lat, lon, ref_lat, ref_lon):
    """기준 위경도(ref) 대비 동쪽(x, m), 북쪽(y, m) 평면 좌표 변환"""
    dlat = math.radians(lat - ref_lat)
    dlon = math.radians(lon - ref_lon)
    east_m = dlon * EARTH_RADIUS_M * math.cos(math.radians(ref_lat))
    north_m = dlat * EARTH_RADIUS_M
    return east_m, north_m


def bearing_deg_between(lat1, lon1, lat2, lon2):
    """두 위경도 사이의 진북 기준 방위각 (0~360도, 시계방향: 북=0, 동=90)"""
    dx, dy = latlon_to_enu(lat2, lon2, lat1, lon1)
    return (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0


def distance_m_between(lat1, lon1, lat2, lon2):
    dx, dy = latlon_to_enu(lat2, lon2, lat1, lon1)
    return math.hypot(dx, dy)


def normalize_axis(pulse_us, neutral_us, neg_limit_us, pos_limit_us, deadband_us):
    """조종기 펄스를 채널 방향(정/역) 무관하게 -1.0 ~ +1.0으로 변환"""
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
    """정규화 값(-1.0 ~ +1.0)을 서보/ESC 실제 PWM(us)으로 변환"""
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
            raise RuntimeError("smbus2 패키지가 없습니다 ('pip3 install smbus2').")
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
    USB 연결 순서가 바뀌어 ttyACM0 / ttyACM1이 뒤바뀌더라도
    제조사/VID(u-blox 0x1546 vs Arduino)를 먼저 대조해 안전하게 매칭
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

        # 1순위: u-blox로 명확히 식별된 포트를 GPS로 배정
        for dev, (is_ublox, _) in port_map.items():
            if is_ublox:
                gps_port = dev
                break

        # 2순위: calibration.json에 저장된 아두이노 포트가 현재도 아두이노(비 GPS)이면 우선 사용
        if arduino_pref and arduino_pref != "auto" and arduino_pref in port_map:
            is_ublox, _ = port_map[arduino_pref]
            if not is_ublox and arduino_pref != gps_port:
                ard_port = arduino_pref

        # 3순위: 나머지 아두이노 후보 포트 자동 배정
        if ard_port is None:
            for dev, (is_ublox, is_ard) in port_map.items():
                if is_ard and dev != gps_port:
                    ard_port = dev
                    break

        # GPS가 VID로 안 잡힌 특수 보드일 경우 설정값 폴백
        if gps_port is None and gps_pref and gps_pref != "auto" and os.path.exists(gps_pref) and gps_pref != ard_port:
            gps_port = gps_pref

    if ard_port is None and arduino_pref and arduino_pref != "auto" and os.path.exists(arduino_pref):
        ard_port = arduino_pref
    if gps_port is None and gps_pref and gps_pref != "auto" and os.path.exists(gps_pref):
        gps_port = gps_pref

    return ard_port, gps_port


class ArduinoRCReader:
    """아두이노 시리얼("CH1,CH2\n") 비동기 수신 스레드"""

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
        print(f"[RC] 아두이노 연결 중 ({self.port}, {self.baud}bps)... 2초 대기")
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
    """NMEA ddmm.mmmmmmm(위도) / dddmm.mmmmmmm(경도) 고정밀 파싱"""
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
    """NMEA 0183 체크섬(*XX) 검증"""
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
    """ZED-F9P USB 양방향 통신: NMEA($GxGGA, $GxRMC) 수신 + RTCM3 보정 데이터 주입"""

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

        # 이동 벡터 기반 보트 헤딩 추정용 앵커
        self._anchor_lat = None
        self._anchor_lon = None
        self.heading_deg = None

    def start(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        print(f"[GPS] ZED-F9P 연결 완료 ({self.port}, {self.baud}bps)")

    def write_rtcm(self, data_bytes):
        if not data_bytes or self.ser is None or not self.ser.is_open:
            return
        with self.write_lock:
            try:
                self.ser.write(data_bytes)
            except Exception:
                pass

    def _update_movement_heading(self, new_lat, new_lon, sog_mps, rmc_cog):
        """최소 이동 거리 필터와 RMC COG를 결합해 안정적인 보트 헤딩 산출"""
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
    """국토지리정보원 NTRIP 서버에서 RTCM3 보정 데이터를 받아 ZED-F9P로 주입하는 스레드"""

    def __init__(self, ntrip_cfg, gps_receiver):
        self.cfg = ntrip_cfg
        self.gps = gps_receiver
        self.running = False
        self.thread = None
        self.connected = False
        self.rtcm_bytes_rx = 0

    def start(self):
        if not self.cfg.get("enabled", True):
            print("[NTRIP] 설정에서 비활성화되어 있습니다.")
            return
        user = str(self.cfg.get("user", "")).strip()
        if not user or user == "YOUR_NGII_ID":
            print("[NTRIP 경고] NTRIP_CONFIG에 국토지리정보원 아이디가 입력되지 않았습니다. 일반/DGPS 모드로 진행합니다.")
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
                    first_line = msg[0] if msg else "응답 없음"
                    print(f"[NTRIP 오류] 서버 인증/마운트포인트 확인 필요: {first_line}")
                    time.sleep(5.0)
                    continue

                self.connected = True
                print(f"[NTRIP] 국토지리정보원({host}:{port}/{mp}) 연결 성공! RTCM3 수신 시작")
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
                            raise ConnectionError("NTRIP 스트림 종료됨")
                        self.rtcm_bytes_rx += len(data)
                        self.gps.write_rtcm(data)
                    except socket.timeout:
                        continue

            except Exception as exc:
                if self.connected:
                    print(f"[NTRIP 재연결 대기] 연결 끊김: {exc}")
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
    """CSV(waypoint_number,latitude,longitude)를 읽어 [(wp_no, lat, lon), ...] 반환"""
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"경로 파일을 찾을 수 없습니다: {filepath}")
    pts = []
    with open(filepath, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        for row in reader:
            wp_no = int(row["waypoint_number"])
            lat = float(row["latitude"])
            lon = float(row["longitude"])
            pts.append((wp_no, lat, lon))
    if len(pts) < 2:
        raise ValueError(f"경로 파일에 최소 2개 이상의 웨이포인트가 필요합니다: {filepath}")
    return pts


class MultipathNavigator:
    """
    해수천 3차선(left / center / right) 멀티패스 알고리즘 엔진
    - 외란 없을 시 현재 경로 유지
    - 바람·파도 외란 시 힘을 거스르지 않고 밀리는 방향의 인접 경로로 전환
    - 양 끝 경로에서 바깥 벽 쪽으로 밀릴 때는 벽 충돌 방지를 위해 복귀 조향 강화
    """

    LANE_ORDER = ["left", "center", "right"]

    def __init__(self, paths_dict, params):
        self.paths = paths_dict
        self.params = params
        self.ref_lat = paths_dict["center"][0][1]
        self.ref_lon = paths_dict["center"][0][2]

        # 빠른 벡터 계산을 위해 ENU 평면 좌표 미리 변환
        self.enu_paths = {}
        for name, wps in self.paths.items():
            self.enu_paths[name] = [latlon_to_enu(lat, lon, self.ref_lat, self.ref_lon) for (_, lat, lon) in wps]

        self.active_lane = "center"
        self.target_idx = 1
        self.disturb_dir = 0             # -1: 왼쪽 밀림, 0: 정상, +1: 오른쪽 밀림
        self.disturb_start_t = None
        self.last_shift_t = -999.0
        self.prev_steer_norm = 0.0

    def _closest_segment_metrics(self, lane_name, boat_x, boat_y, start_idx=0):
        """
        지정된 경로에서 보트와 가장 가까운 선분(i -> i+1)을 찾고,
        최근접 거리(best_dist_m), 우측 법선 기준 부호 있는 이탈 거리(cte_right_m), 선분 방위각 반환
        - cte_right_m > 0 : 경로 기준 오른쪽으로 밀림
        - cte_right_m < 0 : 경로 기준 왼쪽으로 밀림
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
                # 진행 벡터 (tx, ty) 기준 우측 수직 단위벡터 n_right = (ty, -tx)
                nx_r, ny_r = ty, -tx
                best_cte_right = (boat_x - cx) * nx_r + (boat_y - cy) * ny_r
                best_seg_idx = i
                best_seg_bearing = (math.degrees(math.atan2(tx, ty)) + 360.0) % 360.0

        return best_seg_idx, best_dist_m, best_cte_right, best_seg_bearing

    def enter_auto_mode(self, boat_lat, boat_lon, now_mono=None):
        """자동 모드 진입 시 현재 위치에서 가장 가까운 경로와 전방 타겟 웨이포인트를 자동 선택"""
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
        """마지막 65번 웨이포인트 반경 도달 또는 수직 결승선 통과(내적 >= 0) 판정"""
        pts = self.enu_paths[lane_name]
        p_prev = pts[-2]
        p_last = pts[-1]

        dist_to_last = math.hypot(boat_x - p_last[0], boat_y - p_last[1])
        if dist_to_last <= self.params["finish_radius_m"]:
            return True

        # 후반부 구간(전체 경로의 75% 이후)에서 결승선 수직면을 넘어섰는지 벡터 내적으로 확인
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
        매 제어 주기 호출:
        반환 dict: finished, active_lane, shifted_to, wp_number, cte_right_m, steer_norm(-1~+1), target_bearing_deg
        """
        if now_mono is None:
            now_mono = time.monotonic()

        bx, by = latlon_to_enu(boat_lat, boat_lon, self.ref_lat, self.ref_lon)

        # 1) 현재 경로 최근접 선분 및 CTE 계산
        seg_idx, _, cte_right_m, path_bearing_deg = self._closest_segment_metrics(
            self.active_lane, bx, by, start_idx=self.target_idx
        )
        if seg_idx + 1 > self.target_idx:
            self.target_idx = min(len(self.paths[self.active_lane]) - 1, seg_idx + 1)

        # 현재 타겟 웨이포인트 반경 진입 시 다음 점으로 전진
        cur_pts = self.enu_paths[self.active_lane]
        while self.target_idx < len(cur_pts) - 1:
            tx_pt, ty_pt = cur_pts[self.target_idx]
            if math.hypot(tx_pt - bx, ty_pt - by) <= self.params["wp_reach_radius_m"]:
                self.target_idx += 1
            else:
                break

        # 2) 마지막 65번 웨이포인트 종료 판정
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

        # 3) 외란 감지 및 멀티패스 전환 판단
        lane_idx = self.LANE_ORDER.index(self.active_lane)
        drift_deg = wrap_180(boat_heading_deg - path_bearing_deg) if boat_heading_deg is not None else 0.0
        shifted_to = None
        boundary_guard = False

        # 오른쪽 외란 감지 (cte_right_m > +shift_cte_m)
        if cte_right_m >= self.params["shift_cte_m"]:
            if lane_idx < 2:  # left -> center 또는 center -> right 이동 가능
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
                # 이미 right 끝 경로인데 오른쪽으로 더 밀림 -> 해수천 우측 벽 방어!
                boundary_guard = True
                self.disturb_dir = 0
                self.disturb_start_t = None

        # 왼쪽 외란 감지 (cte_right_m < -shift_cte_m)
        elif cte_right_m <= -self.params["shift_cte_m"]:
            if lane_idx > 0:  # right -> center 또는 center -> left 이동 가능
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
                # 이미 left 끝 경로인데 왼쪽으로 더 밀림 -> 해수천 좌측 벽 방어!
                boundary_guard = True
                self.disturb_dir = 0
                self.disturb_start_t = None
        else:
            self.disturb_dir = 0
            self.disturb_start_t = None

        # 경로가 전환되었으면 새 경로 기준으로 최근접 인덱스 및 CTE 즉시 재동기화
        if shifted_to is not None:
            self.last_shift_t = now_mono
            self.disturb_dir = 0
            self.disturb_start_t = None
            new_seg, _, cte_right_m, path_bearing_deg = self._closest_segment_metrics(
                self.active_lane, bx, by, start_idx=max(0, self.target_idx - 2)
            )
            self.target_idx = min(len(self.paths[self.active_lane]) - 1, new_seg + 1)

        # 4) Lookahead 타겟 방위각 및 서보 조향 명령 산출
        lh_dist = self.params["boundary_lookahead_m"] if boundary_guard else self.params["lookahead_dist_m"]
        lh_idx, (lx, ly) = self._select_lookahead_point(
            self.active_lane, self.target_idx, bx, by, lh_dist
        )
        target_bearing_deg = (math.degrees(math.atan2(lx - bx, ly - by)) + 360.0) % 360.0

        # 출발 직후 아직 25cm 이상 전진하지 않아 GPS 헤딩이 없을 때는 직진(0.0) 유지하며 전진 가속
        if boat_heading_deg is None:
            raw_steer = 0.0
        else:
            heading_err = wrap_180(target_bearing_deg - boat_heading_deg)
            gain = 1.0 / self.params["max_steer_angle_deg"]
            if boundary_guard:
                gain *= self.params["boundary_gain_boost"]
            raw_steer = max(-1.0, min(1.0, heading_err * gain))

        # 급격한 타각 스파이크 완화 (Slew-rate 제한)
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
            f"보정 파일('{path}')이 없습니다. 먼저 'python3 calibrate_tool.py'로 보정을 완료해 주세요."
        )
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def open_telemetry_log():
    """QGIS/분석용 주행 기록 CSV 생성 (파일시스템 읽기 전용 오류 발생 시에도 주행은 계속되도록 보호)"""
    try:
        os.makedirs("logs", exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join("logs", f"multipath_log_{stamp}.csv")
        f = open(filepath, "w", newline="", encoding="utf-8")
        writer = csv.writer(f)
        writer.writerow([
            "timestamp", "mode", "lane", "wp_no", "fix_quality",
            "latitude", "longitude", "cte_right_m", "boat_heading_deg",
            "target_bearing_deg", "servo_us", "esc_us"
        ])
        print(f"[로그] 주행 데이터 기록 시작: {filepath}")
        return f, writer
    except OSError as exc:
        print(f"[로그 경고] 파일 쓰기 불가({exc}) -> 로그 저장 없이 주행을 계속합니다.")
        return None, None


def main():
    parser = argparse.ArgumentParser(description="해수천 멀티패스 자율운항 보트 메인 제어기")
    parser.add_argument("--config", default=DEFAULT_CALIBRATION_FILE, help="calibration.json 경로")
    parser.add_argument("--arduino-port", default=None, help="아두이노 포트 수동 강제 지정")
    parser.add_argument("--gps-port", default=None, help="ZED-F9P GPS 포트 수동 강제 지정")
    parser.add_argument("--throttle-ratio", type=float, default=None, help="자동 모드 추진 비율 덮어쓰기 (예: 0.60, 0.70)")
    args = parser.parse_args()

    if serial is None or SMBus is None:
        print("[오류] 필수 패키지(pyserial, smbus2)가 설치되어 있지 않습니다.")
        sys.exit(1)

    cal = load_calibration(args.config)
    hw = cal["hardware"]
    rc_cfg = cal["rc"]
    pwm_cfg = cal["pwm"]
    ms_cfg = cal["mode_switch"]

    if args.throttle_ratio is not None:
        pwm_cfg["auto_throttle_ratio"] = max(0.20, min(1.00, args.throttle_ratio))

    # 1) 해수천 3개 경로 CSV 로드
    paths = {}
    for lane_name, csv_path in PATH_FILES.items():
        paths[lane_name] = load_waypoints_csv(csv_path)
    print(
        f"[경로] CSV 로드 완료 (Left:{len(paths['left'])}점, "
        f"Center:{len(paths['center'])}점, Right:{len(paths['right'])}점)"
    )

    navigator = MultipathNavigator(paths, NAV_PARAMS)

    # 2) PCA9685 초기화 및 ESC 2초 안전 시동(Arming)
    pca = PCA9685Driver(
        bus_num=hw["i2c_bus"],
        address=hw["pca9685_addr"],
        freq_hz=hw["pca9685_freq_hz"]
    )
    servo_ch = hw["servo_channel"]
    esc_ch = hw["esc_channel"]

    print(f"[PWM] PCA9685 초기화 및 ESC 중립({pwm_cfg['esc_neutral_us']}us) 2초 시동(Arming) 중...")
    pca.set_pwm_us(servo_ch, pwm_cfg["servo_center_us"])
    pca.set_pwm_us(esc_ch, pwm_cfg["esc_neutral_us"])
    time.sleep(2.0)

    # 3) 아두이노(RC) 및 ZED-F9P(GPS) 시리얼 포트 연결
    ard_pref = args.arduino_port or hw.get("arduino_port", "auto")
    gps_pref = args.gps_port or hw.get("gps_port", "auto")
    force_manual = (args.arduino_port is not None) and (args.gps_port is not None)
    ard_port, gps_port = detect_serial_ports(ard_pref, gps_pref, force_manual=force_manual)

    if not ard_port:
        pca.close()
        raise RuntimeError("아두이노 시리얼 포트를 찾을 수 없습니다. USB 연결을 확인하세요.")
    if not gps_port:
        pca.close()
        raise RuntimeError("ZED-F9P GPS 시리얼 포트를 찾을 수 없습니다. USB 연결을 확인하세요.")

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

    # 자동 모드 전진 정속 PWM 계산
    auto_cruise_us = int(round(
        pwm_cfg["esc_neutral_us"]
        + pwm_cfg["auto_throttle_ratio"] * (pwm_cfg["esc_fwd_max_us"] - pwm_cfg["esc_neutral_us"])
    ))

    print("\n=====================================================================")
    print("  해수천 멀티패스 자율운항 시스템 가동 시작 (초기 모드: MANUAL)")
    print(f"  - 자동 추진 설정 : {int(pwm_cfg['auto_throttle_ratio']*100)}% ({auto_cruise_us} us)")
    print("  - 수동 -> 자동   : 추진 중립 + 우조향 2.0초 유지 (이후 조종기 조향 무시)")
    print("  - 자동 -> 수동   : 추진 풀 후진 입력 시 즉시 수동 전환")
    print("  - 안전 종료      : Ctrl + C (즉시 중립 정지)")
    print("=====================================================================\n")

    period_sec = 1.0 / CONTROL_LOOP_HZ

    try:
        while running:
            loop_t0 = time.monotonic()

            ch1_us, ch2_us, rc_alive = rc.get_latest()
            gps_snap = gps.get_snapshot()

            # [안전 보호] 아두이노 시리얼 끊김/배선 분리 시 즉시 중립 정지
            if not rc_alive:
                servo_us = pwm_cfg["servo_center_us"]
                esc_us = pwm_cfg["esc_neutral_us"]
                pca.set_pwm_us(servo_ch, servo_us)
                pca.set_pwm_us(esc_ch, esc_us)
                right_steer_hold_start = None
                if mode == "AUTO":
                    mode = "MANUAL"
                    print("\n[비상 보호] RC 수신기 신호 끊김 감지 -> 중립 정지 및 MANUAL 전환")
                sys.stdout.write("\r[대기] RC 수신기 신호 확인 중...                                            ")
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

                # 수동 -> 자동 모드 전환 감지 (추진 중립 + 우조향 2초 유지)
                if th_norm == 0.0 and st_norm >= ms_cfg["right_steer_ratio_threshold"]:
                    if right_steer_hold_start is None:
                        right_steer_hold_start = loop_t0
                    elif (loop_t0 - right_steer_hold_start) >= ms_cfg["right_steer_hold_sec"]:
                        right_steer_hold_start = None
                        if not gps_snap["valid"]:
                            if loop_t0 - last_no_gps_warn_t > 2.0:
                                print("\n[진입 보류] 유효한 GPS 위치가 아직 수신되지 않아 MANUAL 모드를 유지합니다.")
                                last_no_gps_warn_t = loop_t0
                        else:
                            fq = gps_snap["fix_quality"]
                            fq_str = FIX_QUALITY_NAMES.get(fq, f"코드({fq})")
                            if fq not in (4, 5):
                                print(f"\n[경고] 현재 RTK 미적용 상태({fq_str})입니다. 현 측위 상태로 AUTO 모드를 시작합니다!")
                                last_non_rtk_warn_t = loop_t0

                            gps.reset_heading_anchor()
                            lane, wp_no, init_dist = navigator.enter_auto_mode(
                                gps_snap["lat"], gps_snap["lon"], now_mono=loop_t0
                            )
                            mode = "AUTO"
                            servo_us = pwm_cfg["servo_center_us"]
                            esc_us = auto_cruise_us
                            print(
                                f"\n[모드 전환] >>> AUTO 모드 진입! 초기 최근접 경로: '{lane.upper()}' "
                                f"(전방 타겟 WP #{wp_no}, 경로 거리 {init_dist:.2f}m, 측위: {fq_str})"
                            )
                else:
                    right_steer_hold_start = None

            else:
                # ==================== AUTO 모드 ====================
                # 1) 조종기 조향(st_norm)은 완전히 무시하고, 추진 풀 후진만 즉시 감시!
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
                    print("\n[수동 복귀] <<< 조종기 풀 후진 감지 -> 즉시 MANUAL 모드로 전환합니다!")

                elif not gps_snap["valid"]:
                    # 일시적 GPS 끊김 시 급조향 없이 중립 정지 보호
                    servo_us = pwm_cfg["servo_center_us"]
                    esc_us = pwm_cfg["esc_neutral_us"]
                    if loop_t0 - last_no_gps_warn_t > 2.0:
                        print("\n[GPS 경고] GPS 수신 지연 -> 복구될 때까지 일시 중립 대기합니다.")
                        last_no_gps_warn_t = loop_t0

                else:
                    # RTK 상태가 중간에 일반/DGPS로 떨어지면 10초 간격으로 경고만 출력하고 주행 유지
                    fq = gps_snap["fix_quality"]
                    if fq not in (4, 5) and (loop_t0 - last_non_rtk_warn_t > 10.0):
                        fq_str = FIX_QUALITY_NAMES.get(fq, f"코드({fq})")
                        print(f"\n[측위 경고] 현재 RTK 아님 ({fq_str}) - AUTO 주행을 유지합니다.")
                        last_non_rtk_warn_t = loop_t0

                    nav_info = navigator.update(
                        gps_snap["lat"],
                        gps_snap["lon"],
                        gps_snap["heading_deg"],
                        now_mono=loop_t0
                    )

                    if nav_info["shifted_to"] is not None:
                        print(
                            f"\n[멀티패스 이동] 외란 감지로 경로 전환 -> '{nav_info['shifted_to'].upper()}' "
                            f"(이탈량 CTE: {nav_info['cte_right_m']:+.2f}m)"
                        )

                    if nav_info["finished"]:
                        mode = "MANUAL"
                        servo_us = pwm_cfg["servo_center_us"]
                        esc_us = pwm_cfg["esc_neutral_us"]
                        print(
                            f"\n[완주 완료] 마지막 WP #{nav_info['wp_number']} 통과! "
                            f"모터를 중립 정지하고 MANUAL 모드로 복귀합니다."
                        )
                    else:
                        servo_us = map_norm_to_pwm(
                            nav_info["steer_norm"],
                            pwm_cfg["servo_center_us"],
                            pwm_cfg["servo_left_us"],
                            pwm_cfg["servo_right_us"]
                        )
                        esc_us = auto_cruise_us

            # 4) 서보 및 ESC 실제 하드웨어 출력
            pca.set_pwm_us(servo_ch, servo_us)
            pca.set_pwm_us(esc_ch, esc_us)

            # 5) 실시간 터미널 상태줄 출력 및 CSV 로깅
            fq_short = FIX_QUALITY_NAMES.get(gps_snap["fix_quality"], str(gps_snap["fix_quality"]))
            hdg_txt = f"{gps_snap['heading_deg']:5.1f}" if gps_snap["heading_deg"] is not None else " 대기 "
            if mode == "AUTO" and nav_info is not None and not nav_info["finished"]:
                guard_tag = "[벽방어]" if nav_info["boundary_guard"] else ""
                status_line = (
                    f"\r[AUTO:{nav_info['active_lane'].upper():6s}{guard_tag}] "
                    f"WP#{nav_info['wp_number']:02d} | CTE:{nav_info['cte_right_m']:+5.2f}m | "
                    f"선수:{hdg_txt}도->목표:{nav_info['target_bearing_deg']:5.1f}도 | "
                    f"GPS:{fq_short} | 서보:{servo_us:4d} ESC:{esc_us:4d}   "
                )
            else:
                status_line = (
                    f"\r[MANUAL] GPS:{fq_short}(위성{gps_snap['num_sats']:02d}) | "
                    f"조향:{st_norm:+.2f}({servo_us:4d}us) | 추진:{th_norm:+.2f}({esc_us:4d}us)   "
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
                        esc_us
                    ])
                except OSError:
                    log_writer = None

            elapsed = time.monotonic() - loop_t0
            if elapsed < period_sec:
                time.sleep(period_sec - elapsed)

    finally:
        print("\n\n[종료 처리] 서보·ESC 안전 중립 복귀 및 장치 연결 해제 중...")
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
        print("[종료 완료] 모든 출력이 안전하게 정지되었습니다.")


if __name__ == "__main__":
    main()