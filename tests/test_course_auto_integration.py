"""Exercise the real AUTO loop with a synthetic GPS stream and mocked hardware."""

import contextlib
import csv
import io
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import multipath_boat as boat


class CourseAutoIntegrationTests(unittest.TestCase):
    def run_controller(self, launch="forward", disturbance=None):
        cal = boat.load_calibration(Path(boat.__file__).parent / "calibration.json")
        cal["autonomous"]["strategy"] = "gps_course_progressive"
        cal["mode_switch"]["right_steer_hold_sec"] = .05
        clock, cycle = [100.0], [0]
        gps = boat.ZEDF9PReceiver("unused")
        gps.lat, gps.lon, gps.fix_quality = 35.0001, 129., 4
        gps.last_fix_time = 100.
        with patch.object(boat.time, "monotonic", return_value=100.):
            base = gps.get_snapshot()
        pca, rc = Mock(), Mock()
        ntrip = boat.NGII_NtripClient({"enabled": False}, gps)

        def snapshot():
            result = dict(base, valid=True, age_sec=0., course_valid=True,
                          course_deg=0., course_speed_mps=1., course_age_sec=0.,
                          course_time_mono=clock[0], course_gga_skew_sec=0.,
                          course_utc="2026-11-16T12:00:00Z", sog_mps=1., cog_deg=0.)
            # No displacement heading at any point: proves RMC bypasses that gate.
            if disturbance == "gps_hold" and 75 <= cycle[0] <= 80:
                result.update(valid=False, fix_quality=0)
            elif disturbance == "no_rtk" and cycle[0] >= 75:
                result["fix_quality"] = 1
            elif disturbance == "turn" and cycle[0] >= 75:
                result.update(course_deg=90., cog_deg=90.)
            return result

        def receiver():
            cycle[0] += 1
            if cycle[0] > 110:
                raise KeyboardInterrupt()
            throttle = cal["rc"]["throttle_neutral_us"]
            steer = cal["rc"]["steer_right_us"]
            if cycle[0] <= 20:
                steer = cal["rc"]["steer_neutral_us"]
                if launch == "forward":
                    throttle = cal["rc"]["throttle_fwd_us"]
                elif launch == "reverse":
                    throttle = cal["rc"]["throttle_rev_us"]
            return steer, throttle, True

        def sleep(delay):
            clock[0] += delay

        rc.get_latest.side_effect = receiver
        with tempfile.TemporaryDirectory() as folder, contextlib.ExitStack() as stack:
            stack.callback(os.chdir, os.getcwd())
            os.chdir(folder)
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            for name in ("serial", "SMBus"):
                stack.enter_context(patch.object(boat, name, Mock()))
            for name, result in (("load_calibration", cal), ("PCA9685Driver", pca),
                                 ("ArduinoRCReader", rc), ("ZEDF9PReceiver", gps),
                                 ("NGII_NtripClient", ntrip),
                                 ("detect_serial_ports", ("unused-rc", "unused-gps")),
                                 ("load_waypoints_csv", [(1, 35., 129.),
                                                         (2, 35.001, 129.),
                                                         (3, 35.002, 129.)])):
                stack.enter_context(patch.object(boat, name, return_value=result))
            for obj in (gps, ntrip):
                for name in ("start", "stop"):
                    stack.enter_context(patch.object(obj, name))
            stack.enter_context(patch.object(gps, "get_snapshot", side_effect=snapshot))
            stack.enter_context(patch.object(boat.signal, "signal"))
            stack.enter_context(patch.object(boat.time, "monotonic", side_effect=lambda: clock[0]))
            stack.enter_context(patch.object(boat.time, "time", side_effect=lambda: clock[0]))
            stack.enter_context(patch.object(boat.time, "sleep", side_effect=sleep))
            stack.enter_context(patch.object(boat.sys, "argv", ["multipath_boat.py", "--no-spresense"]))
            with self.assertRaises(KeyboardInterrupt):
                boat.main("course-test")
            with open("logs/multipath_log_course-test.csv", newline="") as f:
                rows = list(csv.DictReader(f))
        self.assertEqual(len(rows), 110)
        self.assertTrue(all(None not in row for row in rows))
        return rows, pca, cal

    def test_moving_entry_reuses_course_then_ramps_actual_pwm_without_position_heading(self):
        rows, pca, cal = self.run_controller()
        auto = [r for r in rows if r["mode"] == "AUTO"]
        self.assertEqual(auto[0]["control_heading_source"], "gps_course")
        self.assertEqual(auto[0]["boat_heading_deg"], "0.0")
        self.assertEqual(int(auto[0]["esc_us"]), 1914)
        self.assertTrue(all(r["legacy_heading_deg"] == "" for r in auto))
        self.assertTrue(any(1914 < int(r["esc_us"]) < 1990 for r in auto))
        self.assertEqual(int(auto[-1]["esc_us"]), 1990)
        pca.set_pwm_us.assert_any_call(cal["hardware"]["esc_channel"], 1990)

    def test_stopped_or_reverse_entry_waits_for_new_forward_observation(self):
        for launch in ("stopped", "reverse"):
            with self.subTest(launch=launch):
                rows, _, _ = self.run_controller(launch=launch)
                auto = [r for r in rows if r["mode"] == "AUTO"]
                acquired = next(r for r in auto if r["control_heading_source"] == "gps_course")
                self.assertEqual(auto[0]["heading_state"], "acquiring_forward_motion")
                self.assertGreaterEqual(float(acquired["timestamp"]) - float(auto[0]["timestamp"]), .599)
                self.assertTrue(all(int(r["esc_us"]) == 1914 for r in auto
                                    if r["control_heading_source"] == "none"))
                self.assertEqual(int(auto[-1]["esc_us"]), 1990)

    def test_gps_hold_neutralizes_and_restarts_acquisition_and_power_ramp(self):
        rows, _, _ = self.run_controller(disturbance="gps_hold")
        self.assertEqual(int(rows[73]["esc_us"]), 1990)
        self.assertTrue(all(int(r["esc_us"]) == 1800 for r in rows[74:80]))
        recovered = rows[80:]
        self.assertEqual(recovered[0]["heading_state"], "acquiring_forward_motion")
        self.assertEqual(int(recovered[0]["esc_us"]), 1914)
        self.assertTrue(any(1914 < int(r["esc_us"]) < 1990 for r in recovered))
        self.assertLess(int(recovered[-1]["esc_us"]), 1990)

    def test_rtk_loss_or_large_turn_immediately_removes_boost(self):
        for disturbance in ("no_rtk", "turn"):
            with self.subTest(disturbance=disturbance):
                rows, _, _ = self.run_controller(disturbance=disturbance)
                self.assertEqual(int(rows[73]["esc_us"]), 1990)
                self.assertTrue(all(int(r["esc_us"]) == 1914 for r in rows[74:]))
                self.assertEqual(rows[74]["control_heading_source"], "gps_course")
                if disturbance == "no_rtk":
                    self.assertEqual(rows[74]["heading_confident"], "0")
                else:
                    self.assertNotEqual(int(rows[74]["servo_us"]), 1750)


if __name__ == "__main__":
    unittest.main()
