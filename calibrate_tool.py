#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrate_tool.py
Haesucheon autonomous boat, Korea Maritime & Ocean University - terminal calibration tool for the RC receiver (SR215) and servo/ESC PWM

[Main features]
1. Live Arduino (SR215 receiver) pulse monitor (CH1 steering, CH2 throttle)
2. Measure RC transmitter neutral, full left/right steering, and full forward/reverse pulses (automatically handles reversed channels)
3. Set PCA9685 (I2C bus 0) servo PWM for straight-ahead neutral and full left/right steering
4. Set PCA9685 (I2C bus 0) ESC neutral arming, full forward/reverse, and automatic-mode throttle ratio
5. Test manual control mapping and mode-switch triggers (neutral throttle + right steering for 2 seconds / immediate return on full reverse)
6. Save calibration.json (automatically used by the main multipath autonomous navigation code)
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
            "servo_channel": 0,       # CH0: steering servo
            "esc_channel": 1          # CH1: propulsion ESC
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
    """PCA9685 driver using smbus2 directly, without a CircuitPython dependency."""
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06
    OSC_CLOCK_HZ = 25_000_000.0

    def __init__(self, bus_num=0, address=0x40, freq_hz=50):
        if SMBus is None:
            raise RuntimeError("The smbus2 package is missing ('pip3 install smbus2').")
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
        sleep_mode = (old_mode & 0x7F) | 0x10  # Enter SLEEP mode
        self.bus.write_byte_data(self.address, self.MODE1, sleep_mode)
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode)
        time.sleep(0.005)
        # Enable RESTART (0x80) + AI (Auto-Increment, 0x20)
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
    """Asynchronous reader thread for Arduino serial data ("CH1,CH2\n")."""

    def __init__(self, port, baud=115200):
        if serial is None:
            raise RuntimeError("The pyserial package is missing ('pip3 install pyserial').")
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
        print(f"[INFO] Connecting to Arduino port ({self.port})... (waiting 2 seconds for startup reset)")
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
        print("[WARNING] Cannot scan serial ports without pyserial ('pip3 install pyserial').")
        return None

    ports = list(list_ports.comports())
    if preferred_port and preferred_port != "auto" and os.path.exists(preferred_port):
        return preferred_port

    candidates = []
    for p in ports:
        desc = f"{p.manufacturer or ''} {p.description or ''} {p.hwid or ''}".lower()
        # Exclude u-blox GPS devices (VID 0x1546)
        if p.vid == 0x1546 or "u-blox" in desc or "ublox" in desc:
            continue
        if p.vid in (0x2341, 0x2A03, 0x1A86, 0x10C4, 0x0403) or "arduino" in desc or "ttyacm" in p.device.lower() or "ttyusb" in p.device.lower():
            candidates.append(p)

    if len(candidates) == 1:
        return candidates[0].device

    print("\n[Detected serial ports]")
    for idx, p in enumerate(ports, start=1):
        print(f"  {idx}) {p.device}  ({p.manufacturer or 'Unknown'} - {p.description or ''})")

    if not ports:
        manual = input("No connected ports found. Enter a port path manually (Enter to cancel): ").strip()
        return manual if manual else None

    sel = input("Enter the Arduino port number or path (default: 1): ").strip()
    if not sel:
        return ports[0].device
    if sel.isdigit() and 1 <= int(sel) <= len(ports):
        return ports[int(sel) - 1].device
    return sel


def normalize_axis(pulse_us, neutral_us, neg_limit_us, pos_limit_us, deadband_us):
    """
    Normalize input pulses to -1.0 to +1.0 regardless of channel direction (normal/reversed).
    - Steering: left = -1.0, neutral = 0.0, right = +1.0
    - Throttle: reverse = -1.0, neutral = 0.0, forward = +1.0
    """
    diff = pulse_us - neutral_us
    if abs(diff) <= deadband_us:
        return 0.0

    pos_span = pos_limit_us - neutral_us
    neg_span = neg_limit_us - neutral_us
    if pos_span == 0 or neg_span == 0:
        return 0.0

    # Check whether the input has the same sign as the positive endpoint.
    if (diff > 0 and pos_span > 0) or (diff < 0 and pos_span < 0):
        effective = abs(diff) - deadband_us
        max_eff = max(1, abs(pos_span) - deadband_us)
        return min(1.0, max(0.0, effective / max_eff))
    else:
        effective = abs(diff) - deadband_us
        max_eff = max(1, abs(neg_span) - deadband_us)
        return -min(1.0, max(0.0, effective / max_eff))


def map_norm_to_pwm(norm_val, center_us, neg_pwm_us, pos_pwm_us):
    """Convert a normalized value (-1.0 to +1.0) to servo/ESC PWM (us)."""
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
            print(f"[LOADED] Read the existing configuration file ({path}).")
        except Exception as exc:
            print(f"[WARNING] Could not read the existing configuration file ({exc}); starting with defaults.")
    return cfg


def save_config(cfg, path):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2, ensure_ascii=False)
    print(f"\n[SAVED] Calibration settings saved to '{os.path.abspath(path)}'.")


def monitor_rc_live(rc_reader):
    print("\n--- [1] Live RC receiver signal monitor (exit: Ctrl+C) ---")
    try:
        while True:
            c1, c2, alive = rc_reader.get_latest()
            status = "Receiving" if alive else "No signal/waiting"
            sys.stdout.write(
                f"\rStatus: {status:10s} | CH1(STR steering): {c1:4d} us | CH2(THR throttle): {c2:4d} us   "
            )
            sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nExiting the monitor.")


def calibrate_rc_wizard(rc_reader, cfg):
    print("\n--- [2] RC transmitter (SR215 receiver) pulse calibration wizard ---")
    print("Hold the transmitter controls as instructed and press Enter (averaged over approximately 1.2 seconds).\n")

    input("Step 1 [Neutral]: Release the steering wheel and throttle completely, then press Enter...")
    res = rc_reader.sample_average(1.2)
    if not res:
        print("[FAILED] No valid RC signal received. Check the wiring and power.")
        return
    st_neu, th_neu, st_jit, th_jit = res
    st_db = max(30, st_jit + 15)
    th_db = max(35, th_jit + 15)
    print(f"  -> Neutral measured: steering={st_neu}us (jitter {st_jit}us), throttle={th_neu}us (jitter {th_jit}us)")

    input("\nStep 2 [Full left]: Hold the steering wheel fully left, then press Enter...")
    res = rc_reader.sample_average(1.0)
    if not res:
        print("[FAILED] Signal lost.")
        return
    st_left = res[0]
    print(f"  -> Full left measured: {st_left} us")

    input("\nStep 3 [Full right]: Hold the steering wheel fully right, then press Enter...")
    res = rc_reader.sample_average(1.0)
    if not res:
        print("[FAILED] Signal lost.")
        return
    st_right = res[0]
    print(f"  -> Full right measured: {st_right} us")

    input("\nStep 4 [Full forward]: Hold the throttle trigger fully pulled for forward, then press Enter...")
    res = rc_reader.sample_average(1.0)
    if not res:
        print("[FAILED] Signal lost.")
        return
    th_fwd = res[1]
    print(f"  -> Full forward measured: {th_fwd} us")

    input("\nStep 5 [Full reverse]: Hold the throttle trigger fully pushed for reverse, then press Enter...")
    res = rc_reader.sample_average(1.0)
    if not res:
        print("[FAILED] Signal lost.")
        return
    th_rev = res[1]
    print(f"  -> Full reverse measured: {th_rev} us")

    if abs(st_right - st_neu) < 100 or abs(st_left - st_neu) < 100:
        print("[WARNING] Steering range is less than 100us. Check the transmitter Dual Rate dial.")
    if abs(th_fwd - th_neu) < 100 or abs(th_rev - th_neu) < 100:
        print("[WARNING] Throttle range is less than 100us. Check the wiring (D2/D3 may be swapped).")

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
    print("\n[DONE] RC transmitter calibration values updated in memory.")


def calibrate_servo_wizard(pca, cfg):
    ch = cfg["hardware"]["servo_channel"]
    cur = cfg["pwm"]["servo_center_us"]
    pca.set_pwm_us(ch, cur)

    print(f"\n--- [3] Steering servo PWM calibration (PCA9685 CH{ch}) ---")
    print("Commands:")
    print("  -50 / -10 / +10 / +50 : adjust the current pulse (e.g., enter -10 and press Enter)")
    print("  Enter a number (900~2100): immediately output that pulse in us (e.g., 1480)")
    print("  set_center : save the current position as 'straight-ahead neutral'")
    print("  set_left   : save the current position as 'full left' (stop before mechanical interference!)")
    print("  set_right  : save the current position as 'full right' (stop before mechanical interference!)")
    print("  q          : return to neutral and exit to the menu\n")

    while True:
        print(
            f"[Current output: {cur} us] (saved -> left:{cfg['pwm']['servo_left_us']} / "
            f"neutral:{cfg['pwm']['servo_center_us']} / right:{cfg['pwm']['servo_right_us']})"
        )
        cmd = input("Enter a servo command: ").strip().lower()
        if cmd == "q":
            pca.set_pwm_us(ch, cfg["pwm"]["servo_center_us"])
            break
        elif cmd == "set_center":
            cfg["pwm"]["servo_center_us"] = cur
            print(f"  -> Straight-ahead neutral saved: {cur} us")
        elif cmd == "set_left":
            cfg["pwm"]["servo_left_us"] = cur
            print(f"  -> Full left saved: {cur} us")
        elif cmd == "set_right":
            cfg["pwm"]["servo_right_us"] = cur
            print(f"  -> Full right saved: {cur} us")
        elif cmd.startswith(("+", "-")) and cmd[1:].isdigit():
            cur = max(850, min(2150, cur + int(cmd)))
            pca.set_pwm_us(ch, cur)
        elif cmd.isdigit():
            val = int(cmd)
            if 850 <= val <= 2150:
                cur = val
                pca.set_pwm_us(ch, cur)
            else:
                print("  [INFO] Enter a number within the safe range (850 ~ 2150 us).")
        else:
            print("  [INFO] Unknown command.")


def calibrate_esc_wizard(pca, cfg):
    ch = cfg["hardware"]["esc_channel"]
    cur = cfg["pwm"]["esc_neutral_us"]
    pca.set_pwm_us(ch, cur)

    print(f"\n--- [4] Propulsion ESC PWM calibration (PCA9685 CH{ch}) ---")
    print("[SAFETY WARNING] Proceed only with the propeller suspended in the air or removed!")
    print("Commands:")
    print("  arm        : arm the ESC by sending a neutral signal for 2 seconds")
    print("  0 (or s)   : immediately output neutral to stop")
    print("  -50 / -10 / +10 / +50 or a number (1000~2000): change the output")
    print("  set_neutral: save the current value as 'ESC neutral (stop)'")
    print("  set_fwd    : save the current value as 'manual full forward'")
    print("  set_rev    : save the current value as 'manual full reverse'")
    print("  ratio      : change the automatic-mode throttle ratio (default 0.60 = 60%) and preview forward PWM")
    print("  q          : stop at neutral and exit to the menu\n")

    while True:
        neu = cfg["pwm"]["esc_neutral_us"]
        fwd = cfg["pwm"]["esc_fwd_max_us"]
        ratio = cfg["pwm"]["auto_throttle_ratio"]
        auto_us = int(round(neu + ratio * (fwd - neu)))
        print(
            f"[Current ESC output: {cur} us] (full reverse:{cfg['pwm']['esc_rev_max_us']} / "
            f"neutral:{neu} / full forward:{fwd} / auto throttle({int(ratio*100)}%):{auto_us} us)"
        )
        cmd = input("Enter an ESC command: ").strip().lower()
        if cmd == "q":
            pca.set_pwm_us(ch, cfg["pwm"]["esc_neutral_us"])
            break
        elif cmd in ("0", "s", "stop"):
            cur = cfg["pwm"]["esc_neutral_us"]
            pca.set_pwm_us(ch, cur)
            print(f"  -> Stopped immediately at neutral ({cur} us)")
        elif cmd == "arm":
            cur = cfg["pwm"]["esc_neutral_us"]
            print(f"  -> Applying ESC neutral ({cur} us) for 2.0 seconds...")
            pca.set_pwm_us(ch, cur)
            time.sleep(2.0)
            print("  -> Arming complete.")
        elif cmd == "set_neutral":
            cfg["pwm"]["esc_neutral_us"] = cur
            print(f"  -> ESC neutral saved: {cur} us")
        elif cmd == "set_fwd":
            cfg["pwm"]["esc_fwd_max_us"] = cur
            print(f"  -> Manual full forward saved: {cur} us")
        elif cmd == "set_rev":
            cfg["pwm"]["esc_rev_max_us"] = cur
            print(f"  -> Manual full reverse saved: {cur} us")
        elif cmd == "ratio":
            val_str = input("Enter the automatic-mode throttle ratio (0.30 ~ 1.00, e.g., 0.60 or 0.70): ").strip()
            try:
                r = float(val_str)
                if 0.20 <= r <= 1.00:
                    cfg["pwm"]["auto_throttle_ratio"] = round(r, 2)
                    preview_us = int(round(cfg["pwm"]["esc_neutral_us"] + r * (cfg["pwm"]["esc_fwd_max_us"] - cfg["pwm"]["esc_neutral_us"])))
                    print(f"  -> Automatic-mode throttle ratio saved: {int(r*100)}% (preview PWM: {preview_us} us)")
                else:
                    print("  [INFO] Enter a value between 0.20 and 1.00.")
            except ValueError:
                print("  [INFO] Enter a valid decimal number.")
        elif cmd.startswith(("+", "-")) and cmd[1:].isdigit():
            cur = max(950, min(2050, cur + int(cmd)))
            pca.set_pwm_us(ch, cur)
        elif cmd.isdigit():
            val = int(cmd)
            if 950 <= val <= 2050:
                cur = val
                pca.set_pwm_us(ch, cur)
            else:
                print("  [INFO] Enter a number within the safe range (950 ~ 2050 us).")
        else:
            print("  [INFO] Unknown command.")


def run_integrated_test(rc_reader, pca, cfg):
    print("\n--- [5] Integrated manual control and mode-switch trigger test (exit: Ctrl+C) ---")
    print("1) Check that the servo and motor move in the correct direction in response to transmitter input.")
    print("2) [MANUAL->AUTO test] Hold neutral throttle + right steering for 2 seconds and check for the AUTO transition message.")
    print("3) [AUTO->MANUAL test] In AUTO mode, apply full reverse and check that it immediately returns to MANUAL.\n")

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
                sys.stdout.write("\r[SIGNAL LOST] Outputting safe neutral...                                         ")
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
                # Detect neutral throttle + right steering held for 2 seconds.
                if th_norm == 0.0 and st_norm >= ms_cfg["right_steer_ratio_threshold"]:
                    if right_hold_start is None:
                        right_hold_start = now
                    held = now - right_hold_start
                    if held >= ms_cfg["right_steer_hold_sec"]:
                        sim_mode = "AUTO"
                        right_hold_start = None
                        print("\n[SWITCHED] >>> Right steering held for 2.0 seconds -> AUTO mode (transmitter steering is now ignored)")
                else:
                    right_hold_start = None
                hold_txt = f"{(now - right_hold_start):.1f}s" if right_hold_start else "0.0s"
            else:
                # AUTO simulation: ignore transmitter steering (servo fixed at straight-ahead neutral); monitor only full reverse.
                st_out = pwm_cfg["servo_center_us"]
                th_out = pwm_cfg["esc_neutral_us"]  # Keep the motor at neutral for safe testing on land.
                hold_txt = "Ignored"
                if th_norm <= -ms_cfg["full_reverse_ratio_threshold"]:
                    sim_mode = "MANUAL"
                    print("\n[RETURNED] <<< Full reverse detected -> immediately returning to MANUAL mode!")

            if pca is not None:
                pca.set_pwm_us(hw_cfg["servo_channel"], st_out)
                pca.set_pwm_us(hw_cfg["esc_channel"], th_out)

            sys.stdout.write(
                f"\rMode:{sim_mode:6s} | Steering:{st_norm:+.2f}({st_out:4d}us) | "
                f"Throttle:{th_norm:+.2f}({th_out:4d}us) | Right-steer hold:{hold_txt:5s}   "
            )
            sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nExiting the integrated test and returning to neutral.")
    finally:
        if pca is not None:
            pca.set_pwm_us(hw_cfg["servo_channel"], pwm_cfg["servo_center_us"])
            pca.set_pwm_us(hw_cfg["esc_channel"], pwm_cfg["esc_neutral_us"])


def main():
    parser = argparse.ArgumentParser(description="Haesucheon autonomous boat RC/PWM terminal calibration tool")
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH, help="Path to the calibration JSON file")
    parser.add_argument("--port", default="auto", help="Arduino serial port (default: auto)")
    parser.add_argument("--i2c-bus", type=int, default=None, help="PCA9685 I2C bus number (default: 0)")
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

    # Try to connect to the PCA9685.
    try:
        pca = PCA9685Driver(
            bus_num=cfg["hardware"]["i2c_bus"],
            address=cfg["hardware"]["pca9685_addr"],
            freq_hz=cfg["hardware"]["pca9685_freq_hz"]
        )
        pca.set_pwm_us(cfg["hardware"]["servo_channel"], cfg["pwm"]["servo_center_us"])
        pca.set_pwm_us(cfg["hardware"]["esc_channel"], cfg["pwm"]["esc_neutral_us"])
        print(f"[OK] PCA9685 connected (I2C bus {cfg['hardware']['i2c_bus']}, address 0x{cfg['hardware']['pca9685_addr']:02X})")
    except Exception as exc:
        print(f"[NOTE] PCA9685 is not connected ({exc}) -> RC transmitter calibration is still available.")
        pca = None

    # Try to connect to the Arduino serial port.
    ard_port = detect_arduino_port(cfg["hardware"]["arduino_port"])
    if ard_port:
        try:
            rc_reader = ArduinoRCReader(ard_port, cfg["hardware"]["arduino_baud"])
            rc_reader.start()
            cfg["hardware"]["arduino_port"] = ard_port
        except Exception as exc:
            print(f"[WARNING] Could not open Arduino port ({ard_port}): {exc}")
            rc_reader = None

    try:
        while True:
            print("\n=======================================================")
            print("  Haesucheon boat RC receiver & motor PWM calibration tool (CLI)")
            print("=======================================================")
            print("  1. Live RC receiver pulse monitor (CH1 steering / CH2 throttle)")
            print("  2. RC transmitter neutral and endpoint measurement wizard")
            print("  3. Steering servo neutral and left/right endpoint PWM calibration (PCA9685 CH0)")
            print("  4. Propulsion ESC arming and forward/reverse PWM calibration (PCA9685 CH1)")
            print("  5. Integrated mapping and mode-switch trigger test (2-second right steering/full reverse)")
            print("  6. Review current calibration settings and save calibration.json")
            print("  0. Stop safely and exit")
            print("=======================================================")
            choice = input("Select a menu number: ").strip()

            if choice == "1":
                if rc_reader is None:
                    print("[ERROR] Arduino is not connected.")
                else:
                    monitor_rc_live(rc_reader)
            elif choice == "2":
                if rc_reader is None:
                    print("[ERROR] Arduino is not connected.")
                else:
                    calibrate_rc_wizard(rc_reader, cfg)
            elif choice == "3":
                if pca is None:
                    print("[ERROR] PCA9685 (I2C bus 0) is not connected.")
                else:
                    calibrate_servo_wizard(pca, cfg)
            elif choice == "4":
                if pca is None:
                    print("[ERROR] PCA9685 (I2C bus 0) is not connected.")
                else:
                    calibrate_esc_wizard(pca, cfg)
            elif choice == "5":
                if rc_reader is None:
                    print("[ERROR] Arduino is not connected.")
                else:
                    run_integrated_test(rc_reader, pca, cfg)
            elif choice == "6":
                print("\n[Current calibration data summary]")
                print(json.dumps(cfg, indent=2, ensure_ascii=False))
                yn = input(f"\nSave to '{args.config}'? (Y/n): ").strip().lower()
                if yn in ("", "y", "yes"):
                    save_config(cfg, args.config)
            elif choice == "0":
                yn = input(f"Save changes to '{args.config}' before exiting? (Y/n): ").strip().lower()
                if yn in ("", "y", "yes"):
                    save_config(cfg, args.config)
                break
            else:
                print("[INFO] Select a number from 0 to 6.")
    finally:
        safe_shutdown()
        print("Servo and ESC returned to neutral; ports closed.")


if __name__ == "__main__":
    main()
