#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_check.py - check that a camera works on the Jetson Nano over SSH (Python 3.6+, JetPack OpenCV)

Supports
  CSI ribbon cameras (Raspberry Pi v2 IMX219, IMX477 ...) via nvarguscamerasrc (GStreamer)
  USB webcams via V4L2

Usage
  python3 camera_check.py                     # diagnostics + test every camera found + save snapshots
  python3 camera_check.py --stream            # live view in your laptop's browser: http://<jetson-ip>:8090
  python3 camera_check.py --stream --red      # same, with a simple red-object (balloon) detector overlay
  python3 camera_check.py --device csi0       # pick a camera: csi0, csi1, /dev/video1, 1, or 'test' (no camera)
  options: --width 1280 --height 720 --fps 30 --flip 0 (CSI only: 2 = rotate 180) --port 8090

Snapshots are written to ./camera_check/ ; copy them to the laptop with scp (see the printed command).
"""

import argparse
import glob
import os
import socket
import subprocess
import sys
import threading
import time

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import numpy as np
except ImportError:
    np = None

try:
    from http.server import BaseHTTPRequestHandler, HTTPServer
    from socketserver import ThreadingMixIn
except ImportError:  # pragma: no cover
    BaseHTTPRequestHandler = HTTPServer = ThreadingMixIn = object

OUT_DIR = "camera_check"


# ----------------------------------------------------------------------------------------------
# diagnostics
# ----------------------------------------------------------------------------------------------
def run(cmd, timeout=5):
    try:
        out = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             timeout=timeout, universal_newlines=True)
        return out.stdout.strip()
    except Exception as exc:
        return "({})".format(exc)


def header(title):
    print("\n" + "=" * 72 + "\n" + title + "\n" + "=" * 72)


def video_devices():
    """[(path, name, is_csi)] from /sys/class/video4linux."""
    devs = []
    for path in sorted(glob.glob("/dev/video*"), key=lambda p: int(p.replace("/dev/video", "") or 0)):
        name_file = "/sys/class/video4linux/{}/name".format(os.path.basename(path))
        try:
            with open(name_file) as f:
                name = f.read().strip()
        except OSError:
            name = "?"
        devs.append((path, name, name.startswith("vi-output")))
    return devs


def diagnostics():
    header("1. SYSTEM")
    print("Model : {}".format(run("tr -d '\\0' < /proc/device-tree/model")))
    print("L4T   : {}".format(run("head -n 1 /etc/nv_tegra_release")))
    print("Python: {}".format(sys.version.split()[0]))
    if cv2 is None:
        print("OpenCV: NOT FOUND -> sudo apt install python3-opencv   (JetPack normally ships it)")
    else:
        gst = [l for l in cv2.getBuildInformation().splitlines() if "GStreamer" in l]
        print("OpenCV: {}   {}".format(cv2.__version__, gst[0].strip() if gst else "GStreamer: unknown"))
    print("User groups: {}".format(run("id -nG")))
    print("IP address : {}".format(run("hostname -I")))

    header("2. VIDEO DEVICES")
    devs = video_devices()
    if not devs:
        print("No /dev/video* devices. CSI: check the ribbon cable (contacts facing the right way, latch closed),")
        print("then reboot. USB: re-plug the camera and run 'lsusb'.")
    for path, name, is_csi in devs:
        print("  {:12s} {:35s} {}".format(path, name, "(CSI -> use nvarguscamerasrc)" if is_csi else "(USB/V4L2)"))
    print("\nUSB devices:\n" + "\n".join("  " + l for l in run("lsusb").splitlines()))
    v4l = run("v4l2-ctl --list-devices 2>/dev/null")
    if v4l and "not found" not in v4l:
        print("\nv4l2-ctl --list-devices:\n" + "\n".join("  " + l for l in v4l.splitlines()))
    else:
        print("\n(optional: sudo apt install v4l-utils  for 'v4l2-ctl --list-formats-ext')")

    header("3. CSI CAMERA SERVICE")
    print("nvargus-daemon: {}".format(run("systemctl is-active nvargus-daemon")))
    sensors = run("dmesg 2>/dev/null | grep -iE 'imx219|imx477|ov5693|imx' | tail -n 5")
    print("kernel messages about camera sensors:\n" + ("\n".join("  " + l for l in sensors.splitlines())
                                                        if sensors else "  (none, or dmesg needs sudo)"))
    return devs


# ----------------------------------------------------------------------------------------------
# camera sources
# ----------------------------------------------------------------------------------------------
def csi_pipeline(sensor_id, width, height, fps, flip):
    return ("nvarguscamerasrc sensor-id={} ! "
            "video/x-raw(memory:NVMM), width={}, height={}, framerate={}/1, format=NV12 ! "
            "nvvidconv flip-method={} ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! appsink drop=1 max-buffers=1"
            ).format(sensor_id, width, height, fps, flip)


class TestPattern(object):
    """Synthetic camera (moving red disc) to test the stream without hardware."""

    def __init__(self, width, height):
        self.w, self.h, self.t0 = width, height, time.time()

    def isOpened(self):
        return True

    def read(self):
        time.sleep(1 / 30.0)
        img = np.full((self.h, self.w, 3), (90, 140, 60), np.uint8)
        t = time.time() - self.t0
        cx = int(self.w / 2 + self.w / 3 * np.sin(t))
        cv2.circle(img, (cx, self.h // 2), self.h // 6, (40, 40, 220), -1)
        return True, img

    def release(self):
        pass


def open_camera(spec, width, height, fps, flip):
    """spec: csi0 / csi1 / /dev/videoN / N / test. Returns (capture, description)."""
    if spec == "test":
        return TestPattern(width, height), "synthetic test pattern"
    if spec.startswith("csi"):
        sid = int(spec[3:] or 0)
        pipe = csi_pipeline(sid, width, height, fps, flip)
        return cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER), "CSI sensor-id={} via nvarguscamerasrc".format(sid)
    idx = int(spec.replace("/dev/video", ""))
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))   # most webcams need MJPG for 720p30
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap, "USB /dev/video{} via V4L2".format(idx)


def auto_specs(devs):
    specs, csi_n = [], 0
    for path, _, is_csi in devs:
        if is_csi:
            specs.append("csi{}".format(csi_n))
            csi_n += 1
        else:
            specs.append(path)
    return specs


# ----------------------------------------------------------------------------------------------
# red object detector (very simple, for a first look at the balloon)
# ----------------------------------------------------------------------------------------------
def detect_red(img, min_area_frac=0.001):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255)) | cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]   # OpenCV 3 and 4
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < min_area_frac * img.shape[0] * img.shape[1]:
        return None
    x, y, w, h = cv2.boundingRect(c)
    cx = x + w / 2.0
    bearing = (cx / img.shape[1] - 0.5)          # -0.5 (left edge) .. +0.5 (right edge)
    return {"bbox": (x, y, w, h), "area_frac": area / float(img.shape[0] * img.shape[1]), "bearing": bearing}


def annotate(img, fps, desc, red=False):
    out = img.copy()
    det = detect_red(out) if red else None
    if det:
        x, y, w, h = det["bbox"]
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.line(out, (x + w // 2, 0), (x + w // 2, out.shape[0]), (0, 255, 0), 1)
        cv2.putText(out, "red {:.1f}% of image, offset {:+.2f}".format(100 * det["area_frac"], det["bearing"]),
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.line(out, (out.shape[1] // 2, 0), (out.shape[1] // 2, 20), (255, 255, 255), 2)
    cv2.putText(out, "{}  {}x{}  {:.1f} fps  {}".format(desc, out.shape[1], out.shape[0], fps,
                                                        time.strftime("%H:%M:%S")),
                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return out


# ----------------------------------------------------------------------------------------------
# one-shot test
# ----------------------------------------------------------------------------------------------
def test_camera(spec, args):
    cap, desc = open_camera(spec, args.width, args.height, args.fps, args.flip)
    print("\n--- {} ({})".format(spec, desc))
    if not cap.isOpened():
        print("  FAILED to open.")
        if spec.startswith("csi"):
            print("  Try: sudo systemctl restart nvargus-daemon   and check the ribbon cable / reboot.")
            print("  Raw test: gst-launch-1.0 nvarguscamerasrc num-buffers=30 ! fakesink")
        else:
            print("  Is another program using it? (sudo fuser {})".format(spec if spec.startswith("/dev") else
                                                                         "/dev/video" + spec))
        return False
    t_open = time.time()
    frames, first, last = 0, None, None
    while frames < args.frames and time.time() - t_open < 10:
        ok, img = cap.read()
        if not ok or img is None:
            continue
        if first is None:
            first = img
            t_first = time.time()
        last = img
        frames += 1
    cap.release()
    if first is None:
        print("  Opened, but no frames arrived within 10 s.")
        return False
    dt = time.time() - t_first
    fps = (frames - 1) / dt if dt > 0 and frames > 1 else 0.0
    mean = float(last.mean())
    os.makedirs(OUT_DIR, exist_ok=True)
    name = spec.replace("/dev/", "")
    path = os.path.join(OUT_DIR, "snapshot_{}_{}.jpg".format(name, time.strftime("%H%M%S")))
    cv2.imwrite(path, annotate(last, fps, desc, red=args.red))
    print("  OK: {} frames, {}x{}, {:.1f} fps, mean brightness {:.0f}/255{}".format(
        frames, last.shape[1], last.shape[0], fps, mean,
        "  (very dark: lens cap? indoors?)" if mean < 15 else "  (very bright: overexposed?)" if mean > 240 else ""))
    print("  snapshot: {}".format(os.path.abspath(path)))
    return True


# ----------------------------------------------------------------------------------------------
# MJPEG stream to a browser
# ----------------------------------------------------------------------------------------------
class FrameGrabber(threading.Thread):
    def __init__(self, cap, desc, red, quality):
        threading.Thread.__init__(self, daemon=True)
        self.cap, self.desc, self.red, self.quality = cap, desc, red, quality
        self.lock = threading.Lock()
        self.jpeg = None
        self.fps = 0.0
        self.running = True

    def run(self):
        n, t0 = 0, time.time()
        while self.running:
            ok, img = self.cap.read()
            if not ok or img is None:
                time.sleep(0.01)
                continue
            n += 1
            if time.time() - t0 >= 1.0:
                self.fps, n, t0 = n / (time.time() - t0), 0, time.time()
            out = annotate(img, self.fps, self.desc, self.red)
            ok, buf = cv2.imencode(".jpg", out, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
            if ok:
                with self.lock:
                    self.jpeg = buf.tobytes()

    def latest(self):
        with self.lock:
            return self.jpeg


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def make_handler(grabber):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                page = (b"<html><head><title>Jetson camera</title></head><body style='margin:0;background:#111'>"
                        b"<img src='/stream' style='width:100%;height:auto'></body></html>")
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(page)))
                self.end_headers()
                self.wfile.write(page)
            elif self.path.startswith("/snapshot"):
                jpg = grabber.latest() or b""
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpg)))
                self.end_headers()
                self.wfile.write(jpg)
            elif self.path.startswith("/stream"):
                self.send_response(200)
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                last = None
                try:
                    while True:
                        jpg = grabber.latest()
                        if jpg is None or jpg is last:
                            time.sleep(0.01)
                            continue
                        last = jpg
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                         str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                except OSError:          # browser closed / disconnected
                    pass
            else:
                self.send_error(404)
    return Handler


def stream(spec, args):
    cap, desc = open_camera(spec, args.width, args.height, args.fps, args.flip)
    if not cap.isOpened():
        print("Could not open {} ({}). Run without --stream first for diagnostics.".format(spec, desc))
        return
    grabber = FrameGrabber(cap, desc, args.red, args.quality)
    grabber.start()
    srv = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(grabber))
    ips = [ip for ip in run("hostname -I").split() if ":" not in ip] or ["<jetson-ip>"]
    print("Streaming {} ({}). Open in the laptop browser (same network):".format(spec, desc))
    for ip in ips:
        print("   http://{}:{}        (single frame: /snapshot.jpg)".format(ip, args.port))
    print("Ctrl+C to stop.")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        grabber.running = False
        srv.server_close()
        cap.release()


# ----------------------------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Jetson camera check over SSH")
    ap.add_argument("--device", default="auto", help="auto, csi0, csi1, /dev/video0, 0, or test")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--flip", type=int, default=0, help="CSI only: 0 none, 2 rotate 180, 4 mirror, 6 flip")
    ap.add_argument("--frames", type=int, default=60, help="frames to grab in the one-shot test")
    ap.add_argument("--stream", action="store_true", help="serve live MJPEG on --port")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--quality", type=int, default=70, help="stream JPEG quality")
    ap.add_argument("--red", action="store_true", help="overlay a simple red-object detector")
    ap.add_argument("--skip-diag", action="store_true")
    args = ap.parse_args()

    if cv2 is None or np is None:
        print("OpenCV/numpy for python3 not found. JetPack includes them; otherwise: sudo apt install python3-opencv")
        sys.exit(1)

    devs = [] if args.skip_diag and args.device != "auto" else (
        video_devices() if args.skip_diag else diagnostics())
    if args.device == "auto":
        specs = auto_specs(devs)
        if not specs:
            print("\nNo camera found. Use --device test to check the streaming path without a camera.")
            sys.exit(1)
    else:
        specs = [args.device]

    if args.stream:
        stream(specs[0], args)
        return

    header("4. CAPTURE TEST")
    results = {s: test_camera(s, args) for s in specs}
    header("SUMMARY")
    for s, ok in results.items():
        print("  {:12s} {}".format(s, "WORKING" if ok else "FAILED"))
    if any(results.values()):
        user = run("whoami")
        ip = (run("hostname -I").split() or ["<jetson-ip>"])[0]
        print("\nCopy the snapshots to your laptop (run this ON THE LAPTOP):")
        print('  scp {}@{}:"{}/*.jpg" .'.format(user, ip, os.path.abspath(OUT_DIR)))
        print("Live view:  python3 camera_check.py --stream   then open http://{}:{}".format(ip, args.port))


if __name__ == "__main__":
    main()
