"""Exercise the real AUTO loop with a synthetic GPS stream and mocked hardware."""

import contextlib
import csv
import io
import math
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import multipath_boat as boat


class CourseAutoIntegrationTests(unittest.TestCase):
    def run_controller(self, launch="forward", disturbance=None, strategy="gps_course_progressive"):
        cal = boat.load_calibration(Path(boat.__file__).parent / "calibration.json")
        cal["pwm"]["auto_throttle_ratio"] = .60
        cal["navigation"] = {"finish_margin_m": 0.0}
        cal["autonomous"]["strategy"] = strategy
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
            elif disturbance == "stale_course" and cycle[0] >= 75:
                result.update(course_age_sec=.7, course_time_mono=clock[0] - .7)
            elif disturbance in ("recovery", "overshoot") and cycle[0] >= 75:
                offset, heading, speed = (.9, 340., 1.5) if disturbance == "recovery" else (.1, 350., 4.)
                longitude = 129. + math.degrees(offset / (boat.EARTH_RADIUS_M * math.cos(math.radians(35.))))
                result.update(lon=longitude, course_deg=heading, cog_deg=heading,
                              course_speed_mps=speed, sog_mps=speed)
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

        def waypoints(filename):
            offset = {"left.csv": -1.5, "center.csv": 0., "right.csv": 1.5}[filename]
            longitude = 129. + math.degrees(offset / (boat.EARTH_RADIUS_M * math.cos(math.radians(35.))))
            return [(1, 35., longitude), (2, 35.001, longitude), (3, 35.002, longitude)]

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
                                 ("detect_serial_ports", ("unused-rc", "unused-gps"))):
                stack.enter_context(patch.object(boat, name, return_value=result))
            stack.enter_context(patch.object(boat, "load_waypoints_csv", side_effect=waypoints))
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


class PredictiveAutoIntegrationTests(CourseAutoIntegrationTests):
    """Run the same mode-transition checks with predictive guidance enabled."""

    def run_controller(self, launch="forward", disturbance=None, strategy="predictive_recovery"):
        return super().run_controller(launch=launch, disturbance=disturbance, strategy=strategy)

    def test_recovery_increases_actual_power_and_logs_continuous_target(self):
        predictive, _, _ = self.run_controller(disturbance="recovery")
        progressive, _, _ = self.run_controller(disturbance="recovery", strategy="gps_course_progressive")
        row, baseline = predictive[-1], progressive[-1]
        self.assertEqual(row["prediction_status"], "recovering")
        self.assertGreater(int(row["esc_us"]), int(baseline["esc_us"]))
        self.assertAlmostEqual(float(row["lookahead_m"]), 2.7)
        self.assertLess(float(row["predicted_cte_m"]), float(row["cte_right_m"]))
        self.assertGreater(float(row["prediction_corridor_clearance_m"]), 0)
        self.assertNotEqual(row["lookahead_target_n_m"], "")

    def test_overshoot_reduces_actual_power_before_crossing_center(self):
        rows, _, _ = self.run_controller(disturbance="overshoot")
        self.assertEqual(int(rows[73]["esc_us"]), 1990)
        self.assertLess(int(rows[74]["esc_us"]), 1990)
        self.assertEqual(rows[74]["prediction_status"], "overshoot")
        self.assertGreater(float(rows[74]["cte_right_m"]), 0)
        self.assertLess(float(rows[74]["predicted_cte_m"]), -.3)

    def test_stale_course_removes_boost_and_uses_fixed_lookahead(self):
        rows, _, _ = self.run_controller(disturbance="stale_course")
        self.assertEqual(int(rows[73]["esc_us"]), 1990)
        for row in rows[74:]:
            self.assertEqual(int(row["esc_us"]), 1914)
            self.assertEqual(row["prediction_status"], "prediction_missing")
            self.assertEqual(float(row["lookahead_m"]), 3.5)
            self.assertEqual(row["throttle_target_blend"], "0.0")


if __name__ == "__main__":
    unittest.main()
