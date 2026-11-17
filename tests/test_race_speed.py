"""Offline tests for the race-speed changes: Doppler-course heading and the NTRIP stall watchdog.

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


def nmea(body):
    c = 0
    for ch in body:
        c ^= ord(ch)
    return ("$%s*%02X\r\n" % (body, c)).encode()


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
