"""Fresh GPS motion observations without hardware or receiver threads."""

import math
import unittest
from unittest.mock import Mock, patch

import multipath_boat as boat


def nmea(payload):
    checksum = 0
    for char in payload:
        checksum ^= ord(char)
    return f"${payload}*{checksum:02X}\r\n".encode("ascii")


def rmc(utc="043015.20", date="161126", status="A", speed="1.0", course="90.0", mode="A"):
    return nmea(f"GNRMC,{utc},{status},3500.0000,N,12900.0000,E,{speed},{course},{date},,,{mode}")


def gga(utc="043015.20", latitude="3500.0000", longitude="12900.0000", quality=4):
    return nmea(f"GNGGA,{utc},{latitude},N,{longitude},E,{quality},15,0.70,10.0,M,0.0,M,,")


class CourseObservationTests(unittest.TestCase):
    def setUp(self):
        self.gps = boat.ZEDF9PReceiver("unused")
        self.now = 100.0
        timer = patch.object(boat.time, "monotonic", side_effect=lambda: self.now)
        timer.start()
        self.addCleanup(timer.stop)

    def receive(self, *sentences):
        incoming = iter(sentences)

        def read():
            try:
                return next(incoming)
            except StopIteration:
                self.gps.running = False
                return b""

        self.gps.ser = Mock()
        self.gps.ser.readline.side_effect = read
        self.gps.running = True
        with patch.object(boat.time, "sleep"):
            self.gps._read_loop()
        return self.gps.get_snapshot()

    def test_course_available_without_position_displacement(self):
        snap = self.receive(gga(), rmc())
        self.assertTrue(snap["course_valid"])
        self.assertEqual(snap["course_deg"], 90)
        self.assertAlmostEqual(snap["course_speed_mps"], .514444)
        self.assertEqual(snap["course_utc"], "2026-11-16T04:30:15.200Z")
        self.assertEqual(snap["course_gga_skew_sec"], 0)
        self.assertEqual(snap["course_age_sec"], 0)
        self.assertIsNone(snap["heading_deg"])
        self.assertIsNone(snap["position_heading_deg"])
        self.now += .3
        self.assertAlmostEqual(self.gps.get_snapshot()["course_age_sec"], .3)

    def test_invalid_rmc_clears_course_immediately(self):
        cases = [rmc(status="V"), rmc(mode="N"), rmc(speed=""),
                 rmc(speed="nan"), rmc(speed="inf"), rmc(speed="-1"),
                 rmc(course="nan"), rmc(course="inf"), rmc(course="360"),
                 rmc(course="-1"), rmc(course="bad"), rmc(date="321126"),
                 rmc(date=""), rmc(utc="246015.00"), nmea("GNRMC,043015.40,A")]
        for sentence in cases:
            with self.subTest(sentence=sentence):
                self.gps = boat.ZEDF9PReceiver("unused")
                self.receive(rmc())
                snap = self.receive(sentence)
                self.assertFalse(snap["course_valid"])
                for key in ("course_deg", "course_speed_mps", "course_time_mono", "course_age_sec"):
                    self.assertIsNone(snap[key])

    def test_blank_course_retains_valid_low_speed_observation(self):
        self.receive(gga(), rmc())
        self.now += .2
        snap = self.receive(rmc(utc="043015.40", speed="0.02", course=""))
        self.assertFalse(snap["course_valid"])
        self.assertIsNone(snap["course_deg"])
        self.assertAlmostEqual(snap["course_speed_mps"], .02 * .514444)
        self.assertEqual(snap["course_age_sec"], 0)
        self.assertEqual(snap["course_time_mono"], 100.2)

    def test_bad_checksum_is_ignored_without_refreshing_observation(self):
        self.receive(rmc())
        self.now += .5
        bad = rmc(utc="043015.40").split(b"*")[0] + b"*ZZ\r\n"
        snap = self.receive(bad)
        self.assertTrue(snap["course_valid"])
        self.assertAlmostEqual(snap["course_age_sec"], .5)

    def test_duplicate_and_older_epoch_do_not_refresh_course(self):
        self.receive(rmc())
        self.now += .5
        snap = self.receive(rmc(course="100"), rmc(utc="043015.00", course="110"))
        self.assertEqual(snap["course_deg"], 90)
        self.assertAlmostEqual(snap["course_age_sec"], .5)
        self.assertEqual(snap["course_time_mono"], 100)
        self.now += .1
        snap = self.receive(rmc(utc="043015.40", course="120"))
        self.assertEqual(snap["course_deg"], 120)
        self.assertEqual(snap["course_age_sec"], 0)

    def test_invalid_then_duplicate_does_not_resurrect_old_course(self):
        self.receive(rmc(), rmc(status="V"))
        self.assertFalse(self.receive(rmc())["course_valid"])
        self.assertTrue(self.receive(rmc(utc="043015.40"))["course_valid"])

    def test_midnight_progression_and_signed_gga_skew(self):
        self.receive(gga(utc="235959.80"), rmc(utc="235959.80"))
        self.now += .2
        snap = self.receive(rmc(utc="000000.00", date="171126", course="100"))
        self.assertTrue(snap["course_valid"])
        self.assertEqual(snap["course_deg"], 100)
        self.assertAlmostEqual(snap["course_gga_skew_sec"], .2)
        snap = self.receive(gga(utc="000000.20"))
        self.assertAlmostEqual(snap["course_gga_skew_sec"], -.2)
        snap = self.receive(gga(utc="bad"))
        self.assertIsNone(snap["course_gga_skew_sec"])

    def test_legacy_inputs_remain_independent_of_strict_course_validation(self):
        self.receive(rmc(course="180", speed="2"))
        snap = self.receive(rmc(status="V"))
        self.assertFalse(snap["course_valid"])
        self.assertEqual(snap["cog_deg"], 180)
        self.assertAlmostEqual(snap["sog_mps"], 2 * .514444)

    def test_position_heading_stays_pure_and_reports_measured_speed(self):
        self.receive(gga(), rmc(course="90", speed="2"))
        self.now += .5
        snap = self.receive(gga(utc="043015.70", latitude="3500.0003"))
        distance = boat.distance_m_between(35, 129, snap["lat"], snap["lon"])
        self.assertAlmostEqual(snap["position_heading_deg"], 0)
        self.assertAlmostEqual(snap["heading_deg"], 45)
        self.assertAlmostEqual(snap["position_speed_mps"], distance / .5)
        self.assertEqual(snap["position_heading_time_mono"], 100.5)
        self.now += .1
        self.assertAlmostEqual(self.gps.get_snapshot()["position_heading_age_sec"], .1)
        self.gps.reset_heading_anchor()
        snap = self.gps.get_snapshot()
        for key in ("heading_deg", "position_heading_deg", "position_heading_age_sec", "position_speed_mps"):
            self.assertIsNone(snap[key])
        self.assertEqual(snap["course_deg"], 90)

    def test_no_fix_breaks_position_speed_baseline(self):
        self.receive(gga())
        self.now += .5
        self.receive(gga(utc="043015.70", latitude="3500.0003"))
        self.assertIsNotNone(self.gps.position_heading_deg)
        self.receive(gga(utc="043015.90", quality=0))
        self.assertIsNone(self.gps.position_heading_deg)
        self.now += .5
        snap = self.receive(gga(utc="043016.20", latitude="3500.0006"))
        self.assertIsNone(snap["position_speed_mps"])
        self.now += .5
        snap = self.receive(gga(utc="043016.70", latitude="3500.0009"))
        self.assertTrue(math.isfinite(snap["position_speed_mps"]))

    def test_duplicate_and_out_of_order_gga_do_not_create_or_refresh_position_heading(self):
        self.receive(gga())
        self.now += .2
        snap = self.receive(gga(latitude="3500.0003"))
        self.assertIsNone(snap["position_heading_deg"])
        self.assertIsNotNone(snap["heading_deg"])  # Legacy behavior is unchanged.
        self.now += .2
        snap = self.receive(gga(utc="043015.60", latitude="3500.0006"))
        observed_at = snap["position_heading_time_mono"]
        self.assertIsNotNone(observed_at)
        self.now += .2
        snap = self.receive(gga(utc="043015.60", longitude="12900.0003"),
                            gga(utc="043015.40", longitude="12900.0006"))
        self.assertEqual(snap["position_heading_time_mono"], observed_at)
        self.assertEqual(snap["position_heading_deg"], 0)
        self.assertAlmostEqual(snap["position_heading_age_sec"], .2)

    def test_pure_position_requires_rtk_at_both_ends(self):
        self.receive(gga(quality=2))
        self.now += .2
        snap = self.receive(gga(utc="043015.40", latitude="3500.0100"))
        self.assertIsNone(snap["position_heading_deg"])
        self.now += .2
        snap = self.receive(gga(utc="043015.60", latitude="3500.0103", quality=5))
        self.assertEqual(snap["position_heading_deg"], 0)
        self.now += .2
        snap = self.receive(gga(utc="043015.80", latitude="3500.0200", quality=2))
        self.assertIsNone(snap["position_heading_deg"])
        self.now += .2
        snap = self.receive(gga(utc="043016.00", latitude="3500.0106"))
        self.assertIsNone(snap["position_heading_deg"])

    def test_position_timestamp_allows_midnight_and_rejects_malformed_time(self):
        self.receive(gga(utc="235959.80"))
        self.now += .2
        snap = self.receive(gga(utc="000000.00", latitude="3500.0003"))
        self.assertEqual(snap["position_heading_deg"], 0)
        self.now += .2
        snap = self.receive(gga(utc="bad", latitude="3500.0006"))
        self.assertIsNone(snap["position_heading_deg"])
        self.now += .2
        snap = self.receive(gga(utc="000000.40", latitude="3500.0009"))
        self.assertIsNone(snap["position_heading_deg"])
        self.now += .2
        snap = self.receive(gga(utc="000000.60", latitude="3500.0012"))
        self.assertEqual(snap["position_heading_deg"], 0)


if __name__ == "__main__":
    unittest.main()
