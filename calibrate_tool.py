#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrate_tool.py
국립한국해양대학교 해수천 자율운항 보트 - RC 수신기(SR215) 및 서보·ESC PWM 터미널 보정 툴

[주요 기능]
1. 아두이노(SR215 수신기) 실시간 펄스 모니터 (CH1 조향, CH2 추진)
2. 조종기(RC) 중립 / 좌·우 최대 조향 / 최대 전·후진 펄스 측정 (채널 반전 자동 대응)
3. PCA9685(I2C Bus 0) 서보 모터 직진 중립 및 좌·우 최대 타각 PWM 설정
4. PCA9685(I2C Bus 0) ESC 중립 시동(Arming), 최대 전·후진 및 자동모드 추진 비율 설정
5. 수동 매핑 및 모드 전환 트리거(중립+우조향 2초 / 풀 후진 즉시 복귀) 통합 검증
6. calibration.json 저장 (메인 멀티패스 자율주행 코드와 자동 연동)
"""

import argparse
import json
import os
import signal
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

DEFAULT_CONFIG_PATH = "calibration.json"


def default_calibration():
    return {
        "hardware": {
            "arduino_port": "auto",
            "arduino_baud": 115200,
            "gps_port": "auto",
            "gps_baud": 115200,
            "i2c_bus": 0,
            "pca9685_addr": 64,       # 0x40
            "pca9685_freq_hz": 50,
            "servo_channel": 0,       # CH0: 조향 서보
            "esc_channel": 1          # CH1: 추진 ESC
        },
        "rc": {
            "steer_neutral_us": 1500,
            "steer_left_us": 1100,
            "steer_right_us": 1900,
            "steer_deadband_us": 35,
            "throttle_neutral_us": 1500,
            "throttle_fwd_us": 1900,
            "throttle_rev_us": 1100,
            "throttle_deadband_us": 40
        },
        "pwm": {
            "servo_center_us": 1500,
            "servo_left_us": 1150,
            "servo_right_us": 1850,
            "esc_neutral_us": 1500,
            "esc_fwd_max_us": 1900,
            "esc_rev_max_us": 1100,
            "auto_throttle_ratio": 0.60
        },
        "mode_switch": {
            "right_steer_ratio_threshold": 0.75,
            "right_steer_hold_sec": 2.0,
            "full_reverse_ratio_threshold": 0.75
        }
    }


class PCA9685Driver:
    """외부 CircuitPython 의존성 없이 smbus2로 직접 동작하는 PCA9685 드라이버"""
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06
    OSC_CLOCK_HZ = 25_000_000.0

    def __init__(self, bus_num=0, address=0x40, freq_hz=50):
        if SMBus is None:
            raise RuntimeError("smbus2 패키지가 없습니다 ('pip3 install smbus2').")
        self.bus_num = bus_num
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
        sleep_mode = (old_mode & 0x7F) | 0x10  # SLEEP 진입
        self.bus.write_byte_data(self.address, self.MODE1, sleep_mode)
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode)
        time.sleep(0.005)
        # RESTART(0x80) + AI(Auto-Increment, 0x20) 활성화
        self.bus.write_byte_data(self.address, self.MODE1, old_mode | 0xA0)

    def set_pwm_us(self, channel, pulse_us):
        period_us = 1_000_000.0 / self.actual_freq_hz
        ticks = int(round((float(pulse_us) / period_us) * 4096.0))
        ticks = max(0, min(4095, ticks))
        reg = self.LED0_ON_L + 4 * int(channel)
        self.bus.write_i2c_block_data(
            self.address,
            reg,
            [0x00, 0x00, ticks & 0xFF, (ticks >> 8) & 0x0F]
        )

    def close(self):
        if self.bus is not None:
            self.bus.close()
            self.bus = None


class ArduinoRCReader:
    """아두이노 시리얼("CH1,CH2\n") 비동기 수신 스레드"""

    def __init__(self, port, baud=115200):
        if serial is None:
            raise RuntimeError("pyserial 패키지가 없습니다 ('pip3 install pyserial').")
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
        print(f"[안내] 아두이노 포트({self.port}) 연결 중... (초기 리셋 대기 2초)")
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
                line = raw.decode("ascii", errors="ignore").strip()
                parts = line.split(",")
                if len(parts) != 2:
                    continue
                c1 = int(parts[0])
                c2 = int(parts[1])
                with self.lock:
                    self.ch1_us = c1
                    self.ch2_us = c2
                    self.last_rx_time = time.monotonic()
            except (ValueError, OSError, Exception):
                time.sleep(0.02)

    def get_latest(self):
        with self.lock:
            age = time.monotonic() - self.last_rx_time if self.last_rx_time > 0 else 999.0
            if age > 0.5:
                return 0, 0, False
            alive = (800 <= self.ch1_us <= 2200) and (800 <= self.ch2_us <= 2200)
            return self.ch1_us, self.ch2_us, alive

    def sample_average(self, duration_sec=1.2):
        samples_c1 = []
        samples_c2 = []
        end_t = time.monotonic() + duration_sec
        while time.monotonic() < end_t:
            c1, c2, alive = self.get_latest()
            if alive:
                samples_c1.append(c1)
                samples_c2.append(c2)
            time.sleep(0.03)
        if not samples_c1:
            return None
        avg1 = int(round(sum(samples_c1) / len(samples_c1)))
        avg2 = int(round(sum(samples_c2) / len(samples_c2)))
        jitter1 = max(samples_c1) - min(samples_c1)
        jitter2 = max(samples_c2) - min(samples_c2)
        return avg1, avg2, jitter1, jitter2

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=0.5)
        if self.ser is not None and self.ser.is_open:
            self.ser.close()


def detect_arduino_port(preferred_port="auto"):
    if list_ports is None:
        print("[경고] pyserial이 없어 시리얼 포트를 탐색할 수 없습니다 ('pip3 install pyserial').")
        return None

    ports = list(list_ports.comports())
    if preferred_port and preferred_port != "auto" and os.path.exists(preferred_port):
        return preferred_port

    candidates = []
    for p in ports:
        desc = f"{p.manufacturer or ''} {p.description or ''} {p.hwid or ''}".lower()
        # u-blox GPS(VID 0x1546) 제외
        if p.vid == 0x1546 or "u-blox" in desc or "ublox" in desc:
            continue
        if p.vid in (0x2341, 0x2A03, 0x1A86, 0x10C4, 0x0403) or "arduino" in desc or "ttyacm" in p.device.lower() or "ttyusb" in p.device.lower():
            candidates.append(p)

    if len(candidates) == 1:
        return candidates[0].device

    print("\n[감지된 시리얼 포트 목록]")
    for idx, p in enumerate(ports, start=1):
        print(f"  {idx}) {p.device}  ({p.manufacturer or 'Unknown'} - {p.description or ''})")

    if not ports:
        manual = input("연결된 포트가 보이지 않습니다. 직접 경로 입력 (취소 Enter): ").strip()
        return manual if manual else None

    sel = input("아두이노 포트 번호 또는 경로를 입력하세요 (기본 1): ").strip()
    if not sel:
        return ports[0].device
    if sel.isdigit() and 1 <= int(sel) <= len(ports):
        return ports[int(sel) - 1].device
    return sel


def normalize_axis(pulse_us, neutral_us, neg_limit_us, pos_limit_us, deadband_us):
    """
    채널 방향(정방향/역방향)에 상관없이 입력 펄스를 -1.0 ~ +1.0으로 정규화
    - 조향: 왼쪽 = -1.0, 중립 = 0.0, 오른쪽 = +1.0
    - 추진: 후진 = -1.0, 중립 = 0.0, 전진   = +1.0
    """
    diff = pulse_us - neutral_us
    if abs(diff) <= deadband_us:
        return 0.0

    pos_span = pos_limit_us - neutral_us
    neg_span = neg_limit_us - neutral_us
    if pos_span == 0 or neg_span == 0:
        return 0.0

    # 입력이 양(+)의 끝점 방향과 같은 부호인지 확인
    if (diff > 0 and pos_span > 0) or (diff < 0 and pos_span < 0):
        effective = abs(diff) - deadband_us
        max_eff = max(1, abs(pos_span) - deadband_us)
        return min(1.0, max(0.0, effective / max_eff))
    else:
        effective = abs(diff) - deadband_us
        max_eff = max(1, abs(neg_span) - deadband_us)
        return -min(1.0, max(0.0, effective / max_eff))


def map_norm_to_pwm(norm_val, center_us, neg_pwm_us, pos_pwm_us):
    """정규화된 값(-1.0 ~ +1.0)을 서보/ESC 실제 PWM(us)으로 변환"""
    norm_val = max(-1.0, min(1.0, norm_val))
    if norm_val >= 0.0:
        return int(round(center_us + norm_val * (pos_pwm_us - center_us)))
    return int(round(center_us + (-norm_val) * (neg_pwm_us - center_us)))


def load_config(path):
    cfg = default_calibration()
    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                loaded = json.load(f)
            for section in cfg:
                if section in loaded and isinstance(loaded[section], dict):
                    cfg[section].update(loaded[section])
            print(f"[불러오기 완료] 기존 설정 파일({path})을 읽었습니다.")
        except Exception as exc:
            print(f"[경고] 기존 설정 파일 읽기 실패({exc}), 기본값으로 시작합니다.")
    return cfg


def save_config(cfg, path):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2, ensure_ascii=False)
    print(f"\n[저장 완료] 보정 설정이 '{os.path.abspath(path)}'에 저장되었습니다.")


def monitor_rc_live(rc_reader):
    print("\n--- [1] 실시간 RC 수신기 신호 모니터 (종료: Ctrl+C) ---")
    try:
        while True:
            c1, c2, alive = rc_reader.get_latest()
            status = "정상 수신" if alive else "신호 끊김/대기"
            sys.stdout.write(
                f"\r상태: {status:10s} | CH1(STR 조향): {c1:4d} us | CH2(THR 추진): {c2:4d} us   "
            )
            sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n모니터를 종료합니다.")


def calibrate_rc_wizard(rc_reader, cfg):
    print("\n--- [2] RC 조종기(SR215) 펄스 보정 마법사 ---")
    print("안내에 따라 조종기를 조작한 채로 Enter를 누르세요 (약 1.2초간 평균 측정).\n")

    input("1단계 [중립]: 조종기 핸들과 스로틀에서 손을 완전히 떼고 Enter를 누르세요...")
    res = rc_reader.sample_average(1.2)
    if not res:
        print("[실패] 유효한 RC 신호가 들어오지 않습니다. 배선과 전원을 확인하세요.")
        return
    st_neu, th_neu, st_jit, th_jit = res
    st_db = max(30, st_jit + 15)
    th_db = max(35, th_jit + 15)
    print(f"  -> 중립 측정 완료: 조향={st_neu}us (흔들림 {st_jit}us), 추진={th_neu}us (흔들림 {th_jit}us)")

    input("\n2단계 [좌조향 끝]: 조종기 핸들을 왼쪽 끝까지 돌린 상태로 유지하며 Enter...")
    res = rc_reader.sample_average(1.0)
    if not res:
        print("[실패] 신호 끊김이 발생했습니다.")
        return
    st_left = res[0]
    print(f"  -> 좌조향 최대 측정값: {st_left} us")

    input("\n3단계 [우조향 끝]: 조종기 핸들을 오른쪽 끝까지 돌린 상태로 유지하며 Enter...")
    res = rc_reader.sample_average(1.0)
    if not res:
        print("[실패] 신호 끊김이 발생했습니다.")
        return
    st_right = res[0]
    print(f"  -> 우조향 최대 측정값: {st_right} us")

    input("\n4단계 [최대 전진]: 스로틀 트리거를 전진 끝까지 당긴 상태로 유지하며 Enter...")
    res = rc_reader.sample_average(1.0)
    if not res:
        print("[실패] 신호 끊김이 발생했습니다.")
        return
    th_fwd = res[1]
    print(f"  -> 최대 전진 측정값: {th_fwd} us")

    input("\n5단계 [최대 후진]: 스로틀 트리거를 후진 끝까지 민 상태로 유지하며 Enter...")
    res = rc_reader.sample_average(1.0)
    if not res:
        print("[실패] 신호 끊김이 발생했습니다.")
        return
    th_rev = res[1]
    print(f"  -> 최대 후진 측정값: {th_rev} us")

    if abs(st_right - st_neu) < 100 or abs(st_left - st_neu) < 100:
        print("[경고] 조향 변화폭이 100us 미만입니다. 조종기 Dual Rate 다이얼을 확인하세요.")
    if abs(th_fwd - th_neu) < 100 or abs(th_rev - th_neu) < 100:
        print("[경고] 추진 변화폭이 100us 미만입니다. 배선(D2/D3 뒤바뀜 여부)을 확인하세요.")

    cfg["rc"].update({
        "steer_neutral_us": st_neu,
        "steer_left_us": st_left,
        "steer_right_us": st_right,
        "steer_deadband_us": st_db,
        "throttle_neutral_us": th_neu,
        "throttle_fwd_us": th_fwd,
        "throttle_rev_us": th_rev,
        "throttle_deadband_us": th_db
    })
    print("\n[완료] RC 조종기 보정값이 메모리에 반영되었습니다.")


def calibrate_servo_wizard(pca, cfg):
    ch = cfg["hardware"]["servo_channel"]
    cur = cfg["pwm"]["servo_center_us"]
    pca.set_pwm_us(ch, cur)

    print(f"\n--- [3] 조향 서보 모터 PWM 보정 (PCA9685 CH{ch}) ---")
    print("조작 명령어:")
    print("  -50 / -10 / +10 / +50 : 현재 펄스 증감 (예: -10 입력 후 Enter)")
    print("  직접 숫자 입력 (900~2100): 해당 us 즉시 출력 (예: 1480)")
    print("  set_center : 현재 위치를 '직진 중립'으로 저장")
    print("  set_left   : 현재 위치를 '최대 좌조향'으로 저장 (기구물 간섭 직전까지만!)")
    print("  set_right  : 현재 위치를 '최대 우조향'으로 저장 (기구물 간섭 직전까지만!)")
    print("  q          : 중립 복귀 후 메뉴로 나가기\n")

    while True:
        print(
            f"[현재 출력: {cur} us] (저장값 -> 좌:{cfg['pwm']['servo_left_us']} / "
            f"중립:{cfg['pwm']['servo_center_us']} / 우:{cfg['pwm']['servo_right_us']})"
        )
        cmd = input("서보 명령 입력: ").strip().lower()
        if cmd == "q":
            pca.set_pwm_us(ch, cfg["pwm"]["servo_center_us"])
            break
        elif cmd == "set_center":
            cfg["pwm"]["servo_center_us"] = cur
            print(f"  -> 직진 중립 저장됨: {cur} us")
        elif cmd == "set_left":
            cfg["pwm"]["servo_left_us"] = cur
            print(f"  -> 최대 좌조향 저장됨: {cur} us")
        elif cmd == "set_right":
            cfg["pwm"]["servo_right_us"] = cur
            print(f"  -> 최대 우조향 저장됨: {cur} us")
        elif cmd.startswith(("+", "-")) and cmd[1:].isdigit():
            cur = max(850, min(2150, cur + int(cmd)))
            pca.set_pwm_us(ch, cur)
        elif cmd.isdigit():
            val = int(cmd)
            if 850 <= val <= 2150:
                cur = val
                pca.set_pwm_us(ch, cur)
            else:
                print("  [안내] 안전 범위(850 ~ 2150 us) 내 숫자를 입력하세요.")
        else:
            print("  [안내] 알 수 없는 명령어입니다.")


def calibrate_esc_wizard(pca, cfg):
    ch = cfg["hardware"]["esc_channel"]
    cur = cfg["pwm"]["esc_neutral_us"]
    pca.set_pwm_us(ch, cur)

    print(f"\n--- [4] 추진 ESC 모터 PWM 보정 (PCA9685 CH{ch}) ---")
    print("[안전 경고] 반드시 프로펠러가 공중에 떠 있거나 분리된 상태에서 진행하세요!")
    print("조작 명령어:")
    print("  arm        : 중립 신호를 2초간 보내 ESC 시동(Arming) 수행")
    print("  0 (또는 s) : 즉시 중립 정지 출력")
    print("  -50 / -10 / +10 / +50 또는 직접 숫자(1000~2000) : 출력 변경")
    print("  set_neutral: 현재 값을 'ESC 중립(정지)'으로 저장")
    print("  set_fwd    : 현재 값을 '수동 최대 전진'으로 저장")
    print("  set_rev    : 현재 값을 '수동 최대 후진'으로 저장")
    print("  ratio      : 자동 모드 추진 비율(기본 0.60 = 60%) 변경 및 전진 PWM 미리보기")
    print("  q          : 중립 정지 후 메뉴로 나가기\n")

    while True:
        neu = cfg["pwm"]["esc_neutral_us"]
        fwd = cfg["pwm"]["esc_fwd_max_us"]
        ratio = cfg["pwm"]["auto_throttle_ratio"]
        auto_us = int(round(neu + ratio * (fwd - neu)))
        print(
            f"[현재 ESC 출력: {cur} us] (후진최대:{cfg['pwm']['esc_rev_max_us']} / "
            f"중립:{neu} / 전진최대:{fwd} / 자동추진({int(ratio*100)}%):{auto_us} us)"
        )
        cmd = input("ESC 명령 입력: ").strip().lower()
        if cmd == "q":
            pca.set_pwm_us(ch, cfg["pwm"]["esc_neutral_us"])
            break
        elif cmd in ("0", "s", "stop"):
            cur = cfg["pwm"]["esc_neutral_us"]
            pca.set_pwm_us(ch, cur)
            print(f"  -> 즉시 중립 정지 ({cur} us)")
        elif cmd == "arm":
            cur = cfg["pwm"]["esc_neutral_us"]
            print(f"  -> ESC 중립({cur} us) 2.0초 인가 중...")
            pca.set_pwm_us(ch, cur)
            time.sleep(2.0)
            print("  -> 시동(Arming) 완료.")
        elif cmd == "set_neutral":
            cfg["pwm"]["esc_neutral_us"] = cur
            print(f"  -> ESC 중립 저장됨: {cur} us")
        elif cmd == "set_fwd":
            cfg["pwm"]["esc_fwd_max_us"] = cur
            print(f"  -> 수동 최대 전진 저장됨: {cur} us")
        elif cmd == "set_rev":
            cfg["pwm"]["esc_rev_max_us"] = cur
            print(f"  -> 수동 최대 후진 저장됨: {cur} us")
        elif cmd == "ratio":
            val_str = input("자동 모드 추진 비율 입력 (0.30 ~ 1.00, 예: 0.60 또는 0.70): ").strip()
            try:
                r = float(val_str)
                if 0.20 <= r <= 1.00:
                    cfg["pwm"]["auto_throttle_ratio"] = round(r, 2)
                    preview_us = int(round(cfg["pwm"]["esc_neutral_us"] + r * (cfg["pwm"]["esc_fwd_max_us"] - cfg["pwm"]["esc_neutral_us"])))
                    print(f"  -> 자동 모드 추진 비율 {int(r*100)}% 저장 (출력 예정 PWM: {preview_us} us)")
                else:
                    print("  [안내] 0.20 ~ 1.00 사이로 입력하세요.")
            except ValueError:
                print("  [안내] 올바른 소수를 입력하세요.")
        elif cmd.startswith(("+", "-")) and cmd[1:].isdigit():
            cur = max(950, min(2050, cur + int(cmd)))
            pca.set_pwm_us(ch, cur)
        elif cmd.isdigit():
            val = int(cmd)
            if 950 <= val <= 2050:
                cur = val
                pca.set_pwm_us(ch, cur)
            else:
                print("  [안내] 안전 범위(950 ~ 2050 us) 내 숫자를 입력하세요.")
        else:
            print("  [안내] 알 수 없는 명령어입니다.")


def run_integrated_test(rc_reader, pca, cfg):
    print("\n--- [5] 통합 수동 연동 및 모드 전환 트리거 검증 (종료: Ctrl+C) ---")
    print("1) 조종기 조작에 따라 서보·모터가 올바른 방향으로 움직이는지 확인하세요.")
    print("2) [수동->자동 테스트] 스로틀 중립 + 우조향 2초 유지 시 AUTO 전환 판정이 뜨는지 확인하세요.")
    print("3) [자동->수동 테스트] AUTO 상태에서 풀 후진 입력 시 즉시 MANUAL로 복귀하는지 확인하세요.\n")

    rc_cfg = cfg["rc"]
    pwm_cfg = cfg["pwm"]
    ms_cfg = cfg["mode_switch"]
    hw_cfg = cfg["hardware"]

    sim_mode = "MANUAL"
    right_hold_start = None

    if pca is not None:
        pca.set_pwm_us(hw_cfg["servo_channel"], pwm_cfg["servo_center_us"])
        pca.set_pwm_us(hw_cfg["esc_channel"], pwm_cfg["esc_neutral_us"])

    try:
        while True:
            c1, c2, alive = rc_reader.get_latest()
            now = time.monotonic()

            if not alive:
                st_out = pwm_cfg["servo_center_us"]
                th_out = pwm_cfg["esc_neutral_us"]
                right_hold_start = None
                if pca is not None:
                    pca.set_pwm_us(hw_cfg["servo_channel"], st_out)
                    pca.set_pwm_us(hw_cfg["esc_channel"], th_out)
                sys.stdout.write("\r[신호 끊김] 안전 중립 출력 중...                                         ")
                sys.stdout.flush()
                time.sleep(0.05)
                continue

            st_norm = normalize_axis(
                c1,
                rc_cfg["steer_neutral_us"],
                rc_cfg["steer_left_us"],
                rc_cfg["steer_right_us"],
                rc_cfg["steer_deadband_us"]
            )
            th_norm = normalize_axis(
                c2,
                rc_cfg["throttle_neutral_us"],
                rc_cfg["throttle_rev_us"],
                rc_cfg["throttle_fwd_us"],
                rc_cfg["throttle_deadband_us"]
            )

            if sim_mode == "MANUAL":
                st_out = map_norm_to_pwm(
                    st_norm,
                    pwm_cfg["servo_center_us"],
                    pwm_cfg["servo_left_us"],
                    pwm_cfg["servo_right_us"]
                )
                th_out = map_norm_to_pwm(
                    th_norm,
                    pwm_cfg["esc_neutral_us"],
                    pwm_cfg["esc_rev_max_us"],
                    pwm_cfg["esc_fwd_max_us"]
                )
                # 중립 추진 + 우조향 2초 감지
                if th_norm == 0.0 and st_norm >= ms_cfg["right_steer_ratio_threshold"]:
                    if right_hold_start is None:
                        right_hold_start = now
                    held = now - right_hold_start
                    if held >= ms_cfg["right_steer_hold_sec"]:
                        sim_mode = "AUTO"
                        right_hold_start = None
                        print("\n[전환 성공] >>> 우조향 2.0초 감지 -> AUTO 모드 진입 (조종기 조향 무시 시작)")
                else:
                    right_hold_start = None
                hold_txt = f"{(now - right_hold_start):.1f}s" if right_hold_start else "0.0s"
            else:
                # AUTO 시뮬레이션: 조종기 조향은 무시(서보 직진 중립 고정), 추진 풀 후진만 감시
                st_out = pwm_cfg["servo_center_us"]
                th_out = pwm_cfg["esc_neutral_us"]  # 지상 테스트 안전을 위해 모터는 중립 유지
                hold_txt = "무시중"
                if th_norm <= -ms_cfg["full_reverse_ratio_threshold"]:
                    sim_mode = "MANUAL"
                    print("\n[복귀 성공] <<< 풀 후진 감지 -> 즉시 MANUAL 모드 복귀!")

            if pca is not None:
                pca.set_pwm_us(hw_cfg["servo_channel"], st_out)
                pca.set_pwm_us(hw_cfg["esc_channel"], th_out)

            sys.stdout.write(
                f"\r모드:{sim_mode:6s} | 조향:{st_norm:+.2f}({st_out:4d}us) | "
                f"추진:{th_norm:+.2f}({th_out:4d}us) | 우조향유지:{hold_txt:5s}   "
            )
            sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n통합 테스트를 종료하고 중립으로 복귀합니다.")
    finally:
        if pca is not None:
            pca.set_pwm_us(hw_cfg["servo_channel"], pwm_cfg["servo_center_us"])
            pca.set_pwm_us(hw_cfg["esc_channel"], pwm_cfg["esc_neutral_us"])


def main():
    parser = argparse.ArgumentParser(description="해수천 자율운항 보트 RC·PWM 터미널 보정 툴")
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH, help="보정 JSON 파일 경로")
    parser.add_argument("--port", default="auto", help="아두이노 시리얼 포트 (기본: auto)")
    parser.add_argument("--i2c-bus", type=int, default=None, help="PCA9685 I2C 버스 번호 (기본: 0)")
    args = parser.parse_args()

    cfg = load_config(args.config)
    if args.port != "auto":
        cfg["hardware"]["arduino_port"] = args.port
    if args.i2c_bus is not None:
        cfg["hardware"]["i2c_bus"] = args.i2c_bus

    rc_reader = None
    pca = None

    def safe_shutdown(*_):
        if pca is not None:
            try:
                pca.set_pwm_us(cfg["hardware"]["servo_channel"], cfg["pwm"]["servo_center_us"])
                pca.set_pwm_us(cfg["hardware"]["esc_channel"], cfg["pwm"]["esc_neutral_us"])
                pca.close()
            except Exception:
                pass
        if rc_reader is not None:
            rc_reader.stop()

    signal.signal(signal.SIGTERM, lambda s, f: (safe_shutdown(), sys.exit(0)))

    # PCA9685 연결 시도
    try:
        pca = PCA9685Driver(
            bus_num=cfg["hardware"]["i2c_bus"],
            address=cfg["hardware"]["pca9685_addr"],
            freq_hz=cfg["hardware"]["pca9685_freq_hz"]
        )
        pca.set_pwm_us(cfg["hardware"]["servo_channel"], cfg["pwm"]["servo_center_us"])
        pca.set_pwm_us(cfg["hardware"]["esc_channel"], cfg["pwm"]["esc_neutral_us"])
        print(f"[정상] PCA9685 연결됨 (I2C Bus {cfg['hardware']['i2c_bus']}, 주소 0x{cfg['hardware']['pca9685_addr']:02X})")
    except Exception as exc:
        print(f"[참고] PCA9685 미연결 상태 ({exc}) -> RC 조종기 단독 보정은 정상 사용 가능합니다.")
        pca = None

    # 아두이노 시리얼 연결 시도
    ard_port = detect_arduino_port(cfg["hardware"]["arduino_port"])
    if ard_port:
        try:
            rc_reader = ArduinoRCReader(ard_port, cfg["hardware"]["arduino_baud"])
            rc_reader.start()
            cfg["hardware"]["arduino_port"] = ard_port
        except Exception as exc:
            print(f"[경고] 아두이노 포트({ard_port}) 열기 실패: {exc}")
            rc_reader = None

    try:
        while True:
            print("\n=======================================================")
            print("  해수천 보트 RC 수신기 & 모터 PWM 보정 툴 (CLI)")
            print("=======================================================")
            print("  1. 실시간 RC 수신기 펄스 모니터 (CH1 조향 / CH2 추진)")
            print("  2. RC 조종기 중립·끝점 자동 측정 마법사")
            print("  3. 조향 서보 모터 PWM 중립·좌우 끝각 보정 (PCA9685 CH0)")
            print("  4. 추진 ESC 모터 시동(Arming)·전후진 PWM 보정 (PCA9685 CH1)")
            print("  5. 통합 매핑 및 모드 전환 트리거(우조향 2초/풀후진) 테스트")
            print("  6. 현재 보정 설정 확인 및 calibration.json 저장")
            print("  0. 안전 정지 후 종료")
            print("=======================================================")
            choice = input("메뉴 번호 선택: ").strip()

            if choice == "1":
                if rc_reader is None:
                    print("[오류] 아두이노가 연결되어 있지 않습니다.")
                else:
                    monitor_rc_live(rc_reader)
            elif choice == "2":
                if rc_reader is None:
                    print("[오류] 아두이노가 연결되어 있지 않습니다.")
                else:
                    calibrate_rc_wizard(rc_reader, cfg)
            elif choice == "3":
                if pca is None:
                    print("[오류] PCA9685(I2C Bus 0)가 연결되어 있지 않습니다.")
                else:
                    calibrate_servo_wizard(pca, cfg)
            elif choice == "4":
                if pca is None:
                    print("[오류] PCA9685(I2C Bus 0)가 연결되어 있지 않습니다.")
                else:
                    calibrate_esc_wizard(pca, cfg)
            elif choice == "5":
                if rc_reader is None:
                    print("[오류] 아두이노가 연결되어 있지 않습니다.")
                else:
                    run_integrated_test(rc_reader, pca, cfg)
            elif choice == "6":
                print("\n[현재 보정 데이터 요약]")
                print(json.dumps(cfg, indent=2, ensure_ascii=False))
                yn = input(f"\n'{args.config}' 파일로 저장할까요? (Y/n): ").strip().lower()
                if yn in ("", "y", "yes"):
                    save_config(cfg, args.config)
            elif choice == "0":
                yn = input(f"종료 전 '{args.config}'에 변경사항을 저장할까요? (Y/n): ").strip().lower()
                if yn in ("", "y", "yes"):
                    save_config(cfg, args.config)
                break
            else:
                print("[안내] 0 ~ 6 사이의 번호를 선택하세요.")
    finally:
        safe_shutdown()
        print("서보·ESC 중립 복귀 및 포트 종료 완료.")


if __name__ == "__main__":
    main()
