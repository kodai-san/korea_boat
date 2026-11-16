#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hardware_check.py
Read-only hardware diagnostic for the Haesucheon boat (run on the Jetson Nano).

[What it answers]
1. GPS: is the module a ZED-F9R (built-in IMU) or ZED-F9P? Firmware, USB or UART, navigation rate,
   dynamic model, which NMEA sentences it outputs, whether sensor fusion is on, whether IMU data is available.
2. Arduino: what it sends over serial (number of fields, rate, value ranges; is CH3 forwarded?).
3. PCA9685: is it on the I2C bus, its prescale, and the pulses it is outputting right now.
4. Jetson: model, software versions, input voltage, temperatures inside the sealed hull, USB/power errors, clock.
5. Network: can the NTRIP correction server be reached?

[Safety]
- Never writes to the PCA9685: the servo and motor are not commanded.
- Only sends u-blox *poll* requests and CFG-VALGET (read config) to the GPS: no configuration is changed.
- Opening the Arduino port resets the Uno (normal); it only reads the RC receiver, so nothing moves.
- Refuses to run while multipath_boat.py or calibrate_tool.py is running (they use the same ports).
Standard precaution anyway: boat out of the water or propeller clear.

Usage (from the korea_boat folder):
    python3 hardware_check.py                 # full check, ~1 minute, asks to move the sticks once
    python3 hardware_check.py --no-prompt     # fully non-interactive
The report is printed and saved to logs/hardware_report_<date>_<time>.txt. Please push that file.
"""

import argparse
import calendar
import datetime
import glob
import json
import os
import platform
import socket
import struct
import subprocess
import sys
import time

os.chdir(os.path.dirname(os.path.abspath(__file__)))

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

import multipath_boat as mb


# =====================================================================
# Report helper
# =====================================================================

class Report:
    def __init__(self):
        self.lines = []
        self.answers = {}

    def section(self, title):
        self.out("")
        self.out("=" * 72)
        self.out(title)
        self.out("=" * 72)

    def out(self, text=""):
        print(text)
        sys.stdout.flush()
        self.lines.append(text)

    def answer(self, key, value):
        self.answers[key] = value

    def save(self):
        os.makedirs("logs", exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join("logs", "hardware_report_{}.txt".format(stamp))
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(self.lines))
            f.write("\n\n# SUMMARY (JSON)\n")
            f.write(json.dumps(self.answers, indent=2, default=str))
            f.write("\n")
        return path


def run_cmd(cmd, timeout=5):
    try:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             universal_newlines=True, timeout=timeout)
        return res.returncode, res.stdout.strip(), res.stderr.strip()
    except Exception as exc:
        return -1, "", str(exc)


def read_text(path):
    try:
        with open(path, "r") as f:
            return f.read().strip().replace("\x00", "")
    except Exception:
        return None


# =====================================================================
# u-blox UBX / NMEA stream (read-only polls)
# =====================================================================

UBX_SYNC = b"\xb5\x62"
CLS_NAV, CLS_ACK, CLS_CFG, CLS_MON, CLS_ESF = 0x01, 0x05, 0x06, 0x0A, 0x10
ALLOWED_TX = {(CLS_MON, 0x04), (CLS_NAV, 0x07), (CLS_NAV, 0x05), (CLS_ESF, 0x10),
              (CLS_ESF, 0x15), (CLS_ESF, 0x14), (CLS_CFG, 0x8B)}   # polls + CFG-VALGET only

# CFG-VALGET keys (u-blox F9 interface description). A NAK means "not supported on this module".
CFG_KEYS = [
    ("CFG-RATE-MEAS (ms between fixes)", 0x30210001),
    ("CFG-RATE-NAV (fixes per solution)", 0x30210002),
    ("CFG-NAVSPG-DYNMODEL", 0x20110021),
    ("CFG-NMEA-HIGHPREC", 0x10930006),
    ("CFG-SFCORE-USE_SF (sensor fusion, F9R only)", 0x10080001),
    ("CFG-USBOUTPROT-UBX", 0x10780001),
    ("CFG-USBOUTPROT-NMEA", 0x10780002),
    ("CFG-USBINPROT-RTCM3X", 0x10770004),
    ("CFG-UART1-BAUDRATE", 0x40520001),
    ("CFG-UART1OUTPROT-UBX", 0x10740001),
    ("CFG-UART1OUTPROT-NMEA", 0x10740002),
    ("CFG-UART1INPROT-RTCM3X", 0x10730004),
]
DYN_MODELS = {0: "portable", 2: "stationary", 3: "pedestrian", 4: "automotive", 5: "sea",
              6: "airborne <1g", 7: "airborne <2g", 8: "airborne <4g", 9: "wrist",
              10: "bike", 11: "lawn mower", 12: "e-scooter"}
ESF_SENSOR_TYPES = {5: "gyro z", 6: "wheel tick FL", 7: "wheel tick FR", 8: "wheel tick RL",
                    9: "wheel tick RR", 10: "speed tick", 11: "speed", 12: "gyro temperature",
                    13: "gyro y", 14: "gyro x", 16: "accel x", 17: "accel y", 18: "accel z"}
FUSION_MODES = {0: "initializing", 1: "FUSION ACTIVE", 2: "suspended", 3: "disabled"}


def ubx_frame(cls, mid, payload=b""):
    if (cls, mid) not in ALLOWED_TX:
        raise RuntimeError("Refusing to send non-poll UBX message 0x%02X 0x%02X" % (cls, mid))
    body = struct.pack("<BBH", cls, mid, len(payload)) + payload
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return UBX_SYNC + body + bytes([ck_a, ck_b])


class GnssStream:
    def __init__(self, ser):
        self.ser = ser
        self.buf = bytearray()

    def read_messages(self, duration):
        """Yield ('NMEA', text, t) or ('UBX', (cls, id, payload), t) for `duration` seconds."""
        end = time.monotonic() + duration
        while time.monotonic() < end:
            n = self.ser.in_waiting
            data = self.ser.read(n if n else 1)
            if data:
                self.buf += data
            for msg in self._parse():
                yield msg

    def _parse(self):
        while True:
            i_n = self.buf.find(b"$")
            i_u = self.buf.find(UBX_SYNC)
            cands = [i for i in (i_n, i_u) if i >= 0]
            if not cands:
                del self.buf[:max(0, len(self.buf) - 1)]
                return
            i = min(cands)
            del self.buf[:i]
            if self.buf[:1] == b"$":
                j = self.buf.find(b"\n")
                if j < 0:
                    if len(self.buf) > 300:
                        del self.buf[:1]
                        continue
                    return
                line = bytes(self.buf[:j]).decode("ascii", errors="replace").strip()
                del self.buf[:j + 1]
                yield ("NMEA", line, time.time())
            else:
                if len(self.buf) < 6:
                    return
                cls, mid, length = struct.unpack("<BBH", bytes(self.buf[2:6]))
                if length > 4096:
                    del self.buf[:2]
                    continue
                if len(self.buf) < 8 + length:
                    return
                frame = bytes(self.buf[:8 + length])
                ck_a = ck_b = 0
                for b in frame[2:6 + length]:
                    ck_a = (ck_a + b) & 0xFF
                    ck_b = (ck_b + ck_a) & 0xFF
                if frame[-2] == ck_a and frame[-1] == ck_b:
                    del self.buf[:8 + length]
                    yield ("UBX", (cls, mid, frame[6:6 + length]), time.time())
                else:
                    del self.buf[:2]

    def poll(self, cls, mid, payload=b"", timeout=1.5, reply=None):
        """Send a poll and wait for the reply (cls, id). Returns (payload, t) or ('NAK', t) or (None, None)."""
        want = reply or (cls, mid)
        self.ser.write(ubx_frame(cls, mid, payload))
        self.ser.flush()
        for kind, msg, t in self.read_messages(timeout):
            if kind != "UBX":
                continue
            c, m, p = msg
            if (c, m) == want:
                return p, t
            if c == CLS_ACK and m == 0x00 and len(p) >= 2 and (p[0], p[1]) == (cls, mid):
                return "NAK", t
        return None, None


def nmea_ok(line):
    return mb.verify_nmea_checksum(line)


def check_gps(rep, port, baud, listen_sec):
    rep.section("2. GPS RECEIVER ({} @ {} baud)".format(port, baud))
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except Exception as exc:
        rep.out("[FAIL] Cannot open GPS port: {}".format(exc))
        rep.answer("gps_open_error", str(exc))
        return
    ser.reset_input_buffer()
    gs = GnssStream(ser)

    # --- Passive listen: which messages and how often ---
    rep.out("Listening passively for {:.0f} s...".format(listen_sec))
    counts, bad, ubx_counts = {}, 0, {}
    gga_times, fixq, lat_decimals = set(), {}, set()
    t0 = time.monotonic()
    for kind, msg, _ in gs.read_messages(listen_sec):
        if kind == "NMEA":
            if not nmea_ok(msg):
                bad += 1
                continue
            f = msg.split("*")[0].split(",")
            counts[f[0]] = counts.get(f[0], 0) + 1
            if f[0].endswith("GGA") and len(f) > 6:
                gga_times.add(f[1])
                fixq[f[6]] = fixq.get(f[6], 0) + 1
                if "." in f[2]:
                    lat_decimals.add(len(f[2].split(".")[1]))
        else:
            key = "UBX 0x%02X 0x%02X" % (msg[0], msg[1])
            ubx_counts[key] = ubx_counts.get(key, 0) + 1
    dur = time.monotonic() - t0
    rep.out("NMEA sentences per second:")
    for k in sorted(counts):
        rep.out("  {:8s} {:5.1f} /s".format(k, counts[k] / dur))
    if ubx_counts:
        rep.out("Periodic UBX messages already enabled:")
        for k in sorted(ubx_counts):
            rep.out("  {} {:5.1f} /s".format(k, ubx_counts[k] / dur))
    rep.out("Bad-checksum NMEA lines: {}".format(bad))
    rep.out("Distinct GGA epochs: {} -> fix rate ~{:.1f} Hz".format(len(gga_times), len(gga_times) / dur))
    rep.out("GGA fix quality counts: {}".format(fixq))
    rep.out("Latitude minute decimals in GGA: {} (7 = high-precision NMEA on)".format(sorted(lat_decimals)))
    rep.answer("nmea_rates_per_s", {k: round(v / dur, 2) for k, v in counts.items()})
    rep.answer("gps_fix_rate_hz", round(len(gga_times) / dur, 2))
    rep.answer("gga_fix_quality_counts", fixq)

    # --- MON-VER: module identity ---
    p, _ = gs.poll(CLS_MON, 0x04, timeout=2.0)
    module = "unknown"
    if p and p != "NAK":
        sw = p[0:30].split(b"\x00")[0].decode(errors="replace")
        hw = p[30:40].split(b"\x00")[0].decode(errors="replace")
        ext = [p[i:i + 30].split(b"\x00")[0].decode(errors="replace") for i in range(40, len(p), 30)]
        rep.out("MON-VER: SW='{}' HW='{}'".format(sw, hw))
        for e in ext:
            rep.out("  ext: {}".format(e))
        joined = " ".join(ext)
        if "F9R" in joined or "FWVER=HPS" in joined:
            module = "ZED-F9R (has built-in IMU)"
        elif "F9P" in joined or "FWVER=HPG" in joined:
            module = "ZED-F9P (no IMU)"
        else:
            module = "other: " + joined
        rep.answer("gps_mon_ver", {"sw": sw, "hw": hw, "ext": ext})
    else:
        rep.out("[WARN] No MON-VER reply ({}). UBX input may be disabled on this port.".format(p))
    rep.out(">>> GPS MODULE: {}".format(module))
    rep.answer("gps_module", module)

    # --- Configuration (read only) ---
    rep.out("Configuration (RAM layer, read with CFG-VALGET):")
    cfg = {}
    for name, key in CFG_KEYS:
        p, _ = gs.poll(CLS_CFG, 0x8B, struct.pack("<BBHI", 0, 0, 0, key), timeout=1.0)
        if p is None:
            val = "no reply"
        elif p == "NAK":
            val = "not supported (NAK)"
        else:
            size = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8}.get((key >> 28) & 0x7, 1)
            raw = p[8:8 + size]
            val = int.from_bytes(raw, "little")
            if key == 0x20110021:
                val = "{} ({})".format(val, DYN_MODELS.get(val, "?"))
        cfg[name] = val
        rep.out("  {:46s} = {}".format(name, val))
    rep.answer("gps_config", cfg)

    # --- NAV-PVT: velocity, accuracy, time ---
    rep.out("NAV-PVT samples (polled):")
    pvts = []
    for _ in range(3):
        p, t_rx = gs.poll(CLS_NAV, 0x07, timeout=1.5)
        if not p or p == "NAK" or len(p) < 92:
            rep.out("  no NAV-PVT reply ({})".format(p if p is None or p == "NAK" else "short"))
            break
        (year, month, day, hour, minute, sec, valid) = struct.unpack_from("<HBBBBBB", p, 4)
        nano = struct.unpack_from("<i", p, 16)[0]
        fix_type, flags, _, num_sv = struct.unpack_from("<BBBB", p, 20)
        h_acc, v_acc = struct.unpack_from("<II", p, 40)
        vel_n, vel_e, vel_d, g_speed, head_mot = struct.unpack_from("<iiiii", p, 48)
        s_acc, head_acc = struct.unpack_from("<II", p, 68)
        head_veh = struct.unpack_from("<i", p, 84)[0]
        carr = {0: "none", 1: "RTK float", 2: "RTK fixed"}[(flags >> 6) & 3]
        d = {"fixType": fix_type, "fixType_name": {0: "no fix", 1: "dead reckoning only", 2: "2D", 3: "3D",
                                                    4: "GNSS + dead reckoning (FUSED)", 5: "time only"}.get(fix_type),
             "carrSoln": carr, "numSV": num_sv, "hAcc_m": h_acc / 1000.0, "vAcc_m": v_acc / 1000.0,
             "velN_mps": vel_n / 1000.0, "velE_mps": vel_e / 1000.0, "velD_mps": vel_d / 1000.0,
             "gSpeed_mps": g_speed / 1000.0, "sAcc_mps": s_acc / 1000.0,
             "headMot_deg": head_mot * 1e-5, "headAcc_deg": head_acc * 1e-5,
             "headVehValid": bool(flags & 0x20), "headVeh_deg": head_veh * 1e-5}
        if (valid & 0x03) == 0x03:
            gps_epoch = calendar.timegm((year, month, day, hour, minute, sec)) + nano * 1e-9
            d["system_clock_minus_gps_s"] = round(t_rx - gps_epoch, 3)
        pvts.append(d)
        rep.out("  " + json.dumps(d))
    rep.answer("gps_nav_pvt", pvts)

    # --- F9R-only: sensor fusion and IMU ---
    p, _ = gs.poll(CLS_ESF, 0x10, timeout=1.5)
    if p and p != "NAK" and len(p) >= 16:
        fusion = FUSION_MODES.get(p[12], str(p[12]))
        n = p[15]
        rep.out("ESF-STATUS: fusion mode = {}, {} sensors".format(fusion, n))
        sensors = []
        for i in range(n):
            o = 16 + 4 * i
            if o + 4 > len(p):
                break
            s1, s2, freq, faults = p[o], p[o + 1], p[o + 2], p[o + 3]
            s = {"type": ESF_SENSOR_TYPES.get(s1 & 0x3F, s1 & 0x3F), "used": bool(s1 & 0x40),
                 "ready": bool(s1 & 0x80), "calib": ["not calibrated", "calibrating", "calibrated", "calibrated"][s2 & 3],
                 "freq_hz": freq, "faults": faults}
            sensors.append(s)
            rep.out("  " + json.dumps(s))
        rep.answer("gps_fusion_mode", fusion)
        rep.answer("gps_imu_sensors", sensors)
    else:
        rep.out("ESF-STATUS: no reply ({}) -> no sensor fusion / IMU (expected on a ZED-F9P)".format(p))
        rep.answer("gps_fusion_mode", "unavailable")

    p, _ = gs.poll(CLS_ESF, 0x15, timeout=1.5)
    if p and p != "NAK" and len(p) >= 36:
        bits = struct.unpack_from("<I", p, 0)[0]
        rates = [v * 1e-3 for v in struct.unpack_from("<iii", p, 12)]
        accs = [v * 1e-2 for v in struct.unpack_from("<iii", p, 24)]
        rep.out("ESF-INS (bias-compensated IMU): ang rate x/y/z = {:.2f} {:.2f} {:.2f} deg/s, "
                "accel x/y/z = {:.2f} {:.2f} {:.2f} m/s2, valid bits 0x{:X}".format(*(rates + accs + [bits >> 8])))
        rep.answer("gps_esf_ins", {"ang_rate_dps": rates, "accel_mps2": accs, "valid_bits": bits >> 8})
    else:
        rep.out("ESF-INS: no reply ({})".format(p))

    p, _ = gs.poll(CLS_ESF, 0x14, timeout=1.5)
    if p and p != "NAK" and len(p) >= 16:
        flags = p[5]
        yaw = struct.unpack_from("<I", p, 8)[0] * 1e-2
        pitch, roll = [v * 1e-2 for v in struct.unpack_from("<hh", p, 12)]
        status = {0: "user-defined angles", 1: "roll/pitch aligning", 2: "roll/pitch/yaw aligning",
                  3: "coarse alignment", 4: "fine alignment"}.get((flags >> 1) & 7, "?")
        rep.out("ESF-ALG (IMU mount alignment): auto={} status={} yaw={:.1f} pitch={:.1f} roll={:.1f} deg".format(
            bool(flags & 1), status, yaw, pitch, roll))
        rep.answer("gps_imu_alignment", {"auto": bool(flags & 1), "status": status, "yaw": yaw, "pitch": pitch, "roll": roll})

    p, _ = gs.poll(CLS_NAV, 0x05, timeout=1.5)
    if p and p != "NAK" and len(p) >= 32:
        roll, pitch, heading = [v * 1e-5 for v in struct.unpack_from("<iii", p, 8)]
        acc = [v * 1e-5 for v in struct.unpack_from("<III", p, 20)]
        rep.out("NAV-ATT: roll {:.1f} pitch {:.1f} heading {:.1f} deg (acc {:.1f}/{:.1f}/{:.1f})".format(
            roll, pitch, heading, *acc))
        rep.answer("gps_nav_att", {"roll": roll, "pitch": pitch, "heading": heading, "acc": acc})
    else:
        rep.out("NAV-ATT: no reply ({})".format(p))
    ser.close()


def probe_uart(rep, gps_port):
    rep.section("3. JETSON UART PORTS (passive listen, to see if the GPS is wired by UART too)")
    found = {}
    for dev in sorted(glob.glob("/dev/ttyTHS*")):
        if dev == gps_port:
            continue
        result = "nothing"
        for baud in (9600, 38400, 115200, 230400, 460800):
            try:
                ser = serial.Serial(dev, baud, timeout=0.1)
            except Exception as exc:
                result = "cannot open ({})".format(exc)
                break
            ser.reset_input_buffer()
            data = b""
            end = time.monotonic() + 1.5
            while time.monotonic() < end:
                data += ser.read(256)
            ser.close()
            if b"$G" in data or UBX_SYNC in data:
                result = "GNSS data at {} baud ({} bytes in 1.5 s)".format(baud, len(data))
                break
            if data:
                result = "{} bytes of unrecognised data at {} baud".format(len(data), baud)
        found[dev] = result
        rep.out("  {:16s} {}".format(dev, result))
    if not found:
        rep.out("  no /dev/ttyTHS* ports found")
    rep.answer("uart_probe", found)


# =====================================================================
# Arduino RC reader
# =====================================================================

def check_arduino(rep, port, baud, prompt):
    rep.section("4. ARDUINO RC READER ({} @ {} baud)".format(port, baud))
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as exc:
        rep.out("[FAIL] Cannot open Arduino port: {}".format(exc))
        rep.answer("arduino_open_error", str(exc))
        return
    rep.out("Opened (the Uno resets on open); waiting 2.5 s...")
    time.sleep(2.5)
    ser.reset_input_buffer()

    def collect(sec):
        lines, other = [], []
        end = time.monotonic() + sec
        while time.monotonic() < end:
            raw = ser.readline()
            if not raw:
                continue
            txt = raw.decode("ascii", errors="replace").strip()
            parts = txt.split(",")
            try:
                lines.append([int(x) for x in parts])
            except ValueError:
                other.append(txt)
        return lines, other

    lines, other = collect(5.0)
    widths = sorted({len(l) for l in lines})
    rep.out("Numeric lines in 5 s: {} ({:.1f} /s), fields per line: {}".format(len(lines), len(lines) / 5.0, widths))
    if other:
        rep.out("Non-numeric lines (first 5): {}".format(other[:5]))
    if lines:
        rep.out("First lines: {}".format(lines[:3]))
        for k in range(max(widths)):
            vals = [l[k] for l in lines if len(l) > k]
            rep.out("  field {}: min {} max {} last {}".format(k + 1, min(vals), max(vals), vals[-1]))
    rep.answer("arduino_fields_per_line", widths)
    rep.answer("arduino_line_rate_hz", round(len(lines) / 5.0, 1))
    rep.answer("arduino_non_numeric_lines", other[:5])

    if prompt and sys.stdin.isatty():
        rep.out("")
        rep.out("INTERACTIVE: transmitter ON. After pressing Enter, for 10 s: move steering fully left/right,")
        rep.out("move the throttle trigger, and flip the CH3 (mode) switch several times.")
        try:
            input("Press Enter to start (Ctrl+C to skip)... ")
            lines, _ = collect(10.0)
            for k in range(max([len(l) for l in lines] or [0])):
                vals = [l[k] for l in lines if len(l) > k]
                rep.out("  field {}: min {} max {} (range {})".format(k + 1, min(vals), max(vals), max(vals) - min(vals)))
            rep.answer("arduino_interactive_ranges",
                       {k + 1: [min(l[k] for l in lines if len(l) > k), max(l[k] for l in lines if len(l) > k)]
                        for k in range(max([len(l) for l in lines] or [0]))})
            rep.out("If only 2 fields exist, CH3 is not forwarded by the Arduino sketch.")
        except KeyboardInterrupt:
            rep.out("  skipped")
    ser.close()


# =====================================================================
# PCA9685 (read only)
# =====================================================================

def check_pca(rep, cal):
    hw = cal["hardware"]
    bus_n, addr = hw["i2c_bus"], hw["pca9685_addr"]
    rep.section("5. PCA9685 PWM DRIVER (read only, I2C bus {} addr 0x{:02X})".format(bus_n, addr))
    if SMBus is None:
        rep.out("[SKIP] smbus2 not installed")
        return
    try:
        bus = SMBus(bus_n)
        mode1 = bus.read_byte_data(addr, 0x00)
        mode2 = bus.read_byte_data(addr, 0x01)
        prescale = bus.read_byte_data(addr, 0xFE)
    except Exception as exc:
        rep.out("[FAIL] PCA9685 not responding: {}".format(exc))
        rep.answer("pca9685", "not responding: {}".format(exc))
        return
    freq = 25e6 / (4096.0 * (prescale + 1))
    rep.out("MODE1=0x{:02X} (sleep={}) MODE2=0x{:02X} PRESCALE={} -> {:.2f} Hz assuming a 25 MHz oscillator".format(
        mode1, bool(mode1 & 0x10), mode2, prescale, freq))
    out = {"mode1": mode1, "mode2": mode2, "prescale": prescale, "freq_hz_nominal": round(freq, 2)}
    for name, ch in (("servo", hw["servo_channel"]), ("esc", hw["esc_channel"])):
        reg = 0x06 + 4 * ch
        on_l, on_h, off_l, off_h = [bus.read_byte_data(addr, reg + i) for i in range(4)]
        if off_h & 0x10:
            pulse = "full OFF (no pulses)"
        elif on_h & 0x10:
            pulse = "full ON"
        else:
            ticks = (((off_h & 0x0F) << 8 | off_l) - ((on_h & 0x0F) << 8 | on_l)) % 4096
            pulse = "{:.0f} us".format(ticks / 4096.0 * 1e6 / freq)
        rep.out("  {} (channel {}): currently outputting {}".format(name, ch, pulse))
        out[name + "_pulse"] = pulse
    bus.close()
    rep.out("Note: the real oscillator can differ from 25 MHz; that needs a scope or a loopback wire to measure.")
    rep.answer("pca9685", out)


# =====================================================================
# Jetson system, power, temperature, network
# =====================================================================

def check_system(rep, cal):
    rep.section("1. SYSTEM")
    rep.out("Date (system clock): {}".format(datetime.datetime.now().isoformat()))
    rep.out("Model: {}".format(read_text("/proc/device-tree/model")))
    rep.out("L4T: {}".format((read_text("/etc/nv_tegra_release") or "?").splitlines()[0]))
    rep.out("Kernel: {}  Python: {}".format(platform.release(), sys.version.split()[0]))
    rep.out("pyserial: {}  smbus2: {}".format(getattr(serial, "VERSION", "missing"),
                                               "ok" if SMBus is not None else "missing"))
    _, groups, _ = run_cmd(["id", "-nG"])
    rep.out("User groups: {}".format(groups))
    _, ntp, _ = run_cmd(["timedatectl", "show", "-p", "NTPSynchronized", "--value"])
    rep.out("NTP synchronized: {}".format(ntp or "unknown"))
    rep.out("Uptime: {}".format(read_text("/proc/uptime")))
    rep.answer("jetson_model", read_text("/proc/device-tree/model"))
    rep.answer("python", sys.version.split()[0])
    rep.answer("ntp_synchronized", ntp)

    rep.out("calibration.json ports: arduino={} gps={}".format(cal["hardware"].get("arduino_port"),
                                                               cal["hardware"].get("gps_port")))
    rep.out("Serial ports:")
    ports = []
    for p in list_ports.comports():
        info = {"device": p.device, "vid": "0x%04X" % p.vid if p.vid else None,
                "pid": "0x%04X" % p.pid if p.pid else None, "manufacturer": p.manufacturer,
                "product": p.product, "serial": p.serial_number, "location": p.location}
        ports.append(info)
        rep.out("  " + json.dumps(info))
    rep.answer("serial_ports", ports)
    by_id = glob.glob("/dev/serial/by-id/*")
    for path in by_id:
        rep.out("  by-id: {} -> {}".format(os.path.basename(path), os.path.realpath(path)))

    rep.out("Power input (INA3221 monitor, if present):")
    rails = {}
    for dev in glob.glob("/sys/bus/i2c/drivers/ina3221x/*/iio:device*"):
        for i in range(3):
            name = read_text(os.path.join(dev, "rail_name_{}".format(i)))
            v = read_text(os.path.join(dev, "in_voltage{}_input".format(i)))
            c = read_text(os.path.join(dev, "in_current{}_input".format(i)))
            if name:
                rails[name] = {"mV": v, "mA": c}
                rep.out("  {:12s} {} mV {} mA".format(name, v, c))
    rep.answer("power_rails", rails)

    rep.out("Temperatures (sealed hull):")
    temps = {}
    for z in sorted(glob.glob("/sys/class/thermal/thermal_zone*")):
        name, t = read_text(z + "/type"), read_text(z + "/temp")
        if name and t and t.lstrip("-").isdigit():
            temps[name] = int(t) / 1000.0
            rep.out("  {:12s} {:.1f} C".format(name, int(t) / 1000.0))
    rep.answer("temperatures_c", temps)

    code, dmesg, err = run_cmd(["dmesg"])
    keys = ("under-voltage", "undervoltage", "over-current", "overcurrent", "throttl", "soctherm",
            "usb disconnect", "ttyacm", "cdc_acm", "brown")
    hits = [l for l in dmesg.splitlines() if any(k in l.lower() for k in keys)]
    if code != 0:
        rep.out("dmesg not readable without sudo ({}); optional: sudo dmesg | grep -iE 'volt|current|ttyACM|disconnect'".format(err[:60]))
    else:
        rep.out("Kernel log: power/USB events (last 20 of {}):".format(len(hits)))
        for l in hits[-20:]:
            rep.out("  " + l)
    rep.answer("dmesg_power_usb_events", hits[-20:])

    usage = os.statvfs(".")
    free_gb = usage.f_bavail * usage.f_frsize / 1e9
    rep.out("Free disk space: {:.1f} GB".format(free_gb))
    rep.answer("free_disk_gb", round(free_gb, 1))


def check_network(rep):
    rep.section("6. NTRIP SERVER REACHABILITY (TCP connect only, no login sent)")
    host, port = mb.NTRIP_CONFIG["host"], int(mb.NTRIP_CONFIG["port"])
    try:
        t0 = time.monotonic()
        s = socket.create_connection((host, port), timeout=5.0)
        s.close()
        msg = "reachable ({:.0f} ms)".format(1000 * (time.monotonic() - t0))
    except Exception as exc:
        msg = "NOT reachable: {}".format(exc)
    rep.out("{}:{} -> {}".format(host, port, msg))
    rep.answer("ntrip_reachable", msg)


def other_boat_processes():
    me = os.getpid()
    found = []
    for d in glob.glob("/proc/[0-9]*"):
        try:
            if int(os.path.basename(d)) == me:
                continue
            with open(d + "/cmdline", "rb") as f:
                cmd = f.read().replace(b"\x00", b" ").decode(errors="replace")
        except Exception:
            continue
        if "multipath_boat.py" in cmd or "calibrate_tool.py" in cmd:
            found.append(cmd.strip())
    return found


def main():
    parser = argparse.ArgumentParser(description="Read-only hardware diagnostic for the boat")
    parser.add_argument("--gps-port", default=None)
    parser.add_argument("--arduino-port", default=None)
    parser.add_argument("--listen", type=float, default=6.0, help="Passive GPS listen time (s)")
    parser.add_argument("--no-prompt", action="store_true", help="Skip the interactive stick/CH3 test")
    parser.add_argument("--skip-network", action="store_true")
    args = parser.parse_args()

    rep = Report()
    if serial is None:
        print("[ERROR] pyserial is not installed (pip3 install pyserial)")
        sys.exit(1)
    busy = other_boat_processes()
    if busy:
        print("[STOP] Boat software is running; stop it first:\n  " + "\n  ".join(busy))
        sys.exit(1)

    cal = mb.load_calibration(mb.DEFAULT_CALIBRATION_FILE)
    hw = cal["hardware"]
    rep.out("hardware_check.py - read-only diagnostic (no PCA9685 writes, no GPS configuration changes)")

    steps = [("system", lambda: check_system(rep, cal))]
    ard_pref = args.arduino_port or hw.get("arduino_port", "auto")
    gps_pref = args.gps_port or hw.get("gps_port", "auto")
    ard_port, gps_port = mb.detect_serial_ports(ard_pref, gps_pref,
                                                force_manual=bool(args.arduino_port and args.gps_port))
    rep.answer("detected_ports", {"arduino": ard_port, "gps": gps_port})

    steps.append(("gps", lambda: check_gps(rep, gps_port, hw.get("gps_baud", 115200), args.listen)
                  if gps_port else rep.out("[FAIL] GPS port not detected")))
    steps.append(("uart", lambda: probe_uart(rep, gps_port)))
    steps.append(("arduino", lambda: check_arduino(rep, ard_port, hw.get("arduino_baud", 115200), not args.no_prompt)
                  if ard_port else rep.out("[FAIL] Arduino port not detected")))
    steps.append(("pca9685", lambda: check_pca(rep, cal)))
    if not args.skip_network:
        steps.append(("network", lambda: check_network(rep)))

    rep.out("Detected ports (same logic as multipath_boat.py): Arduino={} GPS={}".format(ard_port, gps_port))
    for name, fn in steps:
        try:
            fn()
        except KeyboardInterrupt:
            rep.out("[{}] interrupted".format(name))
        except Exception as exc:
            rep.out("[{}] ERROR: {!r}".format(name, exc))
            rep.answer(name + "_error", repr(exc))

    rep.section("SUMMARY")
    for k in ("gps_module", "gps_fusion_mode", "detected_ports", "gps_fix_rate_hz",
              "arduino_fields_per_line", "ntrip_reachable", "temperatures_c"):
        rep.out("  {:24s} {}".format(k, rep.answers.get(k)))
    path = rep.save()
    rep.out("")
    rep.out("Report saved to {} -- please commit and push it.".format(path))


if __name__ == "__main__":
    main()
