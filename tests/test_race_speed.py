"""Offline tests for the race-speed changes: full_throttle strategy, Doppler-course heading, NTRIP stall watchdog.

Run from the korea_boat folder:
    PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p 'test_*.py' -v
"""

import os
import socket
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import multipath_boat as mb  # noqa: E402
from auto_strategies import AUTO_STRATEGIES, create_auto_strategy  # noqa: E402


def nmea(body):
    c = 0
    for ch in body:
        c ^= ord(ch)
    return ("$%s*%02X\r\n" % (body, c)).encode()


class FullThrottleStrategyTest(unittest.TestCase):
    def make(self, **opts):
        return create_auto_strategy({"strategy": "full_throttle", "full_throttle": opts}, mb.NAV_PARAMS)

    def test_registered_and_always_full(self):
        self.assertIn("full_throttle", AUTO_STRATEGIES)
        s = self.make()
        for cte, herr in ((0.0, 0.0), (2.0, 40.0), (-1.5, None)):
            out = s.update(cte_right_m=cte, heading_error_deg=herr, boundary_guard=False, now_mono=1.0)
            self.assertEqual(out["throttle_blend"], 1.0)

    def test_integral_removes_steady_bias(self):
        # A constant heading error (rudder trim) makes the integral grow and steer harder over time.
        s = self.make(heading_integral_gain=0.6, integral_limit_deg=20.0)
        steer = [s.update(cte_right_m=0.0, heading_error_deg=3.0, boundary_guard=False, now_mono=0.05 * k)["steer_norm"]
                 for k in range(200)]
        p_only = 3.0 / mb.NAV_PARAMS["max_steer_angle_deg"]
        self.assertGreater(steer[-1], 2 * p_only)
        self.assertLessEqual(steer[-1], (3.0 + 20.0) / mb.NAV_PARAMS["max_steer_angle_deg"] + 1e-9)

    def test_reset_clears_integral(self):
        s = self.make(heading_integral_gain=0.6)
        for k in range(100):
            s.update(cte_right_m=0.0, heading_error_deg=5.0, boundary_guard=False, now_mono=0.05 * k)
        s.reset()
        out = s.update(cte_right_m=0.0, heading_error_deg=0.0, boundary_guard=False, now_mono=10.0)
        self.assertEqual(out["steer_norm"], 0.0)

    def test_rejects_bad_options(self):
        with self.assertRaises(ValueError):
            self.make(heading_integral_gain=-1.0)
        with self.assertRaises(ValueError):
            self.make(unknown_option=1.0)


class DopplerHeadingTest(unittest.TestCase):
    LINES = [nmea("GNGGA,072512.00,3504.3865502,N,12904.9750104,E,4,28,0.51,12.3,M,25.1,M,1.0,0000"),
             nmea("GNRMC,072512.00,A,3504.3865502,N,12904.9750104,E,2.600,241.50,161126,,,D"),
             nmea("GNGGA,072512.20,3504.3862000,N,12904.9744000,E,4,28,0.51,12.3,M,25.1,M,1.0,0000"),
             nmea("GNRMC,072512.20,A,3504.3862000,N,12904.9744000,E,2.600,243.00,161126,,,D")]

    def heading(self, mode, lines):
        old = mb.NAV_PARAMS.get("heading_source", "blend")
        mb.NAV_PARAMS["heading_source"] = mode
        try:
            queue = list(lines)

            class Ser:
                is_open = True

                def readline(self):
                    if queue:
                        return queue.pop(0)
                    time.sleep(0.01)
                    return b""

            g = mb.ZEDF9PReceiver("test")
            g.ser = Ser()
            g.running = True
            t = threading.Thread(target=g._read_loop, daemon=True)
            t.start()
            time.sleep(0.2)
            g.running = False
            t.join(1.0)
            return g.get_snapshot()["heading_deg"]
        finally:
            mb.NAV_PARAMS["heading_source"] = old

    def test_cog_mode_uses_latest_doppler_course(self):
        self.assertAlmostEqual(self.heading("cog", self.LINES), 243.0, places=3)

    def test_cog_mode_falls_back_when_slow(self):
        slow = [nmea(x.decode().strip()[1:].split("*")[0].replace("2.600", "0.300")) for x in self.LINES]
        h = self.heading("cog", slow)
        self.assertIsNotNone(h)
        self.assertGreater(abs(h - 243.0), 1.0)   # Movement bearing, not the low-speed RMC course

    def test_blend_mode_unchanged(self):
        h = self.heading("blend", self.LINES)
        self.assertNotAlmostEqual(h, 243.0, places=1)


class NtripStallWatchdogTest(unittest.TestCase):
    def test_reconnects_when_stream_stalls(self):
        srv = socket.socket()
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", 0))
        srv.listen(5)
        port = srv.getsockname()[1]
        conns = []

        def serve():
            srv.settimeout(0.2)
            while len(conns) < 3:
                try:
                    c, _ = srv.accept()
                except socket.timeout:
                    continue
                except OSError:
                    return
                conns.append(c)
                c.recv(1024)
                c.sendall(b"ICY 200 OK\r\n\r\n")
                c.sendall(b"\xd3" * 50)          # Some correction bytes, then silence with the socket open

        threading.Thread(target=serve, daemon=True).start()

        class FakeGps:
            def get_snapshot(self):
                return {"gga_raw": None}

            def write_rtcm(self, data):
                pass

        cfg = dict(mb.NTRIP_CONFIG, host="127.0.0.1", port=port, user="test", stall_timeout_sec=1.0)
        client = mb.NGII_NtripClient(cfg, FakeGps())
        client.start()
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline and client.get_snapshot()["connection_attempts"] < 2:
            time.sleep(0.1)
        attempts = client.get_snapshot()["connection_attempts"]
        client.running = False
        srv.close()
        for c in conns:
            c.close()
        self.assertGreaterEqual(attempts, 2, "client should reconnect after the stream stalls")


if __name__ == "__main__":
    unittest.main()
