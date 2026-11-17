"""Optional accuracy messages through the real NMEA read loop, with fake serial."""

import unittest
from unittest.mock import patch

import multipath_boat as boat


def sentence(body):
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return ("$%s*%02X\r\n" % (body, checksum)).encode("ascii")


def gga(utc="123456.20"):
    return sentence("GNGGA," + utc + ",3504.3865502,N,12904.9750104,E,4,28,0.51,12.3,M,25.1,M,1.0,0000")


def pubx(utc="123456.20", accuracy="0.15", status="RK"):
    return sentence("PUBX,00," + utc + ",3504.3865502,N,12904.9750104,E,12.3," + status + "," + accuracy + ",0.2,1,240,0,1,0.5,0.7,0.8,28,0,0")


class AccuracyReceiverTests(unittest.TestCase):
    def consume(self, observations, snapshot_at):
        receiver = boat.ZEDF9PReceiver("unused")
        pending, clock = list(observations), [100.]

        class Serial:
            def readline(self):
                if pending:
                    clock[0], raw = pending.pop(0)
                    return raw
                receiver.running = False
                return b""

        receiver.ser = Serial()
        receiver.running = True
        with patch.object(boat.time, "monotonic", side_effect=lambda: clock[0]):
            receiver._read_loop()
            clock[0] = snapshot_at
            return receiver.get_snapshot(), receiver.get_latest_gps_sample()

    def test_optional_messages_are_not_required_for_normal_gga(self):
        snap, sample = self.consume([(100., gga())], 100.1)
        self.assertTrue(snap["valid"])
        self.assertEqual(sample["timestamp"], "123456.20")
        self.assertIsNone(snap["hacc_m"])
        self.assertIsNone(snap["hacc_age_sec"])
        self.assertIsNone(snap["hacc_gga_skew_sec"])

    def test_pubx_is_recorded_without_overwriting_the_position(self):
        snap, sample = self.consume([(100., gga()), (100.1, pubx())], 100.3)
        self.assertAlmostEqual(snap["hacc_m"], .15)
        self.assertAlmostEqual(snap["hacc_age_sec"], .2)
        self.assertEqual(snap["hacc_gga_skew_sec"], 0.)
        self.assertEqual(snap["hacc_source"], "pubx")
        self.assertEqual(snap["gps_utc"], sample["timestamp"])
        self.assertEqual(snap["lat"], sample["latitude"])

    def test_gst_source_skew_crosses_midnight_and_age_keeps_increasing(self):
        gst = sentence("GNGST,000000.10,0,0,0,0,0.3,0.4,0.9")
        snap, _ = self.consume([(100., gga("235959.90")), (100.1, gst)], 101.1)
        self.assertAlmostEqual(snap["hacc_m"], .5)
        self.assertAlmostEqual(snap["hacc_gga_skew_sec"], .2)
        self.assertAlmostEqual(snap["hacc_age_sec"], 1.)
        self.assertEqual(snap["hacc_source"], "gst")

    def test_malformed_or_bad_checksum_messages_cannot_refresh_accuracy(self):
        bad_checksum = pubx(accuracy="0.01").replace(b"0.01", b"9.99")
        snap, _ = self.consume([
            (100., gga()), (100.1, pubx()), (100.3, pubx(accuracy="nan")),
            (100.5, pubx(status="NF")), (100.7, bad_checksum),
        ], 101.)
        self.assertAlmostEqual(snap["hacc_m"], .15)
        self.assertAlmostEqual(snap["hacc_age_sec"], .9)


if __name__ == "__main__":
    unittest.main()
