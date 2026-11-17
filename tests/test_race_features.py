"""Offline tests for the race features: 04 switch, line/bank limits, test square and stuck recovery.

Run from the korea_boat folder:
    PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p 'test_*.py' -v
"""

import contextlib
import csv
import io
import math
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import multipath_boat as mb  # noqa: E402
from auto_strategies import (apply_run_overrides, create_auto_strategy, default_autonomous_config,  # noqa: E402
                             resolve_recovery_config)
from recovery import StuckRecovery, resolve_recovery_options  # noqa: E402

LINE = "05_straight_line"


class RecoveryTimingTest(unittest.TestCase):
    OPTS = {"stuck_speed_mps": 0.15, "stuck_sec": 1.0, "startup_grace_sec": 2.0, "neutral_sec": 0.3,
            "reverse_sec": 2.0, "reverse_throttle": 0.5, "max_attempts": 2}

    def run_stuck(self, rec, start, seconds, speed=0.0, forward=True):
        states = []
        for k in range(int(round(seconds / 0.05))):
            t = start + k * 0.05 + 1e-6       # Just after each 20 Hz tick, so float rounding cannot shift phases
            out = rec.update(t, forward, speed)
            states.append((round(t, 2), None if out is None else out["state"],
                           None if out is None else out["throttle_norm"]))
        return states

    def test_no_trigger_during_startup_grace_or_when_moving_or_without_speed(self):
        rec = StuckRecovery(self.OPTS)
        rec.reset(0.0)
        self.assertTrue(all(s is None for _, s, _ in self.run_stuck(rec, 0.0, 2.0)))
        self.assertTrue(all(s is None for _, s, _ in self.run_stuck(rec, 2.0, 5.0, speed=0.8)))
        self.assertTrue(all(s is None for _, s, _ in self.run_stuck(rec, 7.0, 5.0, speed=None)))
        self.assertTrue(all(s is None for _, s, _ in self.run_stuck(rec, 12.0, 5.0, forward=False)))

    def test_stuck_runs_neutral_reverse_neutral_then_retries(self):
        rec = StuckRecovery(self.OPTS)
        rec.reset(0.0)
        states = self.run_stuck(rec, 0.0, 6.0)
        first = next(i for i, (_, s, _) in enumerate(states) if s is not None)
        self.assertAlmostEqual(states[first][0], 3.0, places=2)          # 2 s grace + 1 s stuck
        seq = [s for _, s, _ in states[first:]]
        self.assertEqual(seq[:6], ["neutral_in"] * 6)
        self.assertEqual(seq[6:46], ["reverse"] * 40)
        self.assertEqual(seq[46:52], ["neutral_out"] * 6)
        self.assertIsNone(seq[52])                                       # Normal control again
        self.assertTrue(all(th == -0.5 for _, s, th in states if s == "reverse"))
        self.assertEqual(rec.attempts, 1)

    def test_gives_up_after_max_attempts(self):
        rec = StuckRecovery(self.OPTS)
        rec.reset(0.0)
        states = self.run_stuck(rec, 0.0, 30.0)
        self.assertEqual(states[-1][1], "gave_up")
        self.assertEqual(rec.attempts, 2)
        rec.reset(40.0)
        self.assertFalse(rec.gave_up)

    def test_disabled_never_acts(self):
        rec = StuckRecovery(dict(self.OPTS, enabled=False))
        rec.reset(0.0)
        self.assertTrue(all(s is None for _, s, _ in self.run_stuck(rec, 0.0, 20.0)))

    def test_rejects_bad_settings(self):
        for bad in ({"enabled": 1}, {"reverse_throttle": 0.0}, {"max_attempts": 1.5}, {"unknown": 1}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                resolve_recovery_options(bad)


class RunSettingsTest(unittest.TestCase):
    def cal(self):
        return mb.load_calibration(Path(mb.__file__).parent / "calibration.json")["autonomous"]

    def test_checked_in_json_selects_05_with_recovery_on(self):
        auto = self.cal()
        self.assertEqual(create_auto_strategy(auto, mb.NAV_PARAMS).name, LINE)
        self.assertFalse(auto["use_04_control"])
        self.assertTrue(resolve_recovery_config(auto)["enabled"])
        o = create_auto_strategy(auto, mb.NAV_PARAMS).options
        self.assertEqual((o["line_max_offset_m"], o["band_half_width_m"], o["test_square"]), (1.5, 2.5, False))

    def test_use_04_flag_in_json_or_cli(self):
        auto = dict(self.cal(), use_04_control=True)
        self.assertEqual(create_auto_strategy(auto, mb.NAV_PARAMS).name, "04_predictive_recovery")
        self.assertEqual(create_auto_strategy(apply_run_overrides(self.cal(), use_04=True), mb.NAV_PARAMS).name,
                         "04_predictive_recovery")
        with self.assertRaises(ValueError):
            create_auto_strategy(dict(self.cal(), use_04_control="yes"), mb.NAV_PARAMS)

    def test_line_and_bank_limits_override_json(self):
        auto = apply_run_overrides(self.cal(), line_limit_m=1.0, bank_limit_m=3.2, test_square=True)
        o = create_auto_strategy(auto, mb.NAV_PARAMS).options
        self.assertEqual((o["line_max_offset_m"], o["band_half_width_m"], o["test_square"]), (1.0, 3.2, True))
        with self.assertRaises(ValueError):    # The line limit plus guard zone must stay inside the bank limit
            create_auto_strategy(apply_run_overrides(self.cal(), line_limit_m=2.2), mb.NAV_PARAMS)

    def test_no_recovery_override(self):
        self.assertFalse(resolve_recovery_config(apply_run_overrides(self.cal(), recovery=False))["enabled"])

    def test_defaults_for_the_calibration_tool_are_valid(self):
        auto = default_autonomous_config()
        self.assertEqual(create_auto_strategy(auto, mb.NAV_PARAMS).name, "02_adaptive_throttle")
        self.assertTrue(resolve_recovery_config(auto)["enabled"])


class TestSquareNavigatorTest(unittest.TestCase):
    def setUp(self):
        paths = {lane: mb.load_waypoints_csv(os.path.join(os.path.dirname(__file__), "..", p))
                 for lane, p in mb.PATH_FILES.items()}
        strategy = create_auto_strategy({"strategy": LINE, LINE: {"test_square": True, "band_half_width_m": 2.5,
                                                                  "line_max_offset_m": 1.5}}, mb.NAV_PARAMS)
        self.nav = mb.MultipathNavigator(paths, dict(mb.NAV_PARAMS, finish_margin_m=1.0), strategy=strategy)
        self.line = self.nav.line_guidance

    def latlon(self, s, d):
        x, y = self.line.point(s, d)
        return (self.nav.ref_lat + math.degrees(y / mb.EARTH_RADIUS_M),
                self.nav.ref_lon + math.degrees(x / (mb.EARTH_RADIUS_M * math.cos(math.radians(self.nav.ref_lat)))))

    def step(self, s, d, t):
        return self.nav.update(*self.latlon(s, d), self.line.bearing_deg, now_mono=t, heading_confident=True,
                               motion={"speed_mps": 1.0})

    def test_square_visits_bank_lines_then_line_limits_and_stops(self):
        lane, _, _ = self.nav.enter_auto_mode(*self.latlon(10.0, 0.0), now_mono=0.0)
        self.assertEqual(lane, "test")
        self.assertIn("+-2.50 m (bank lines) then +-1.50 m (line limits)", self.nav.describe_line())
        first = self.step(10.5, 0.0, 0.1)
        self.assertEqual(first["prediction_status"], "square leg 1: +-2.50 m to start")
        self.assertEqual(first["throttle_norm"], 0.6)
        self.assertFalse(first["finished"])
        visited = []
        t = 0.2
        for s, d in [(13, -2.5), (28, -2.5), (28, 2.5), (13, 2.5), (13, -2.5),
                        (13, -1.5), (28, -1.5), (28, 1.5), (13, 1.5), (13, -1.5)]:
            info = self.step(s, d, t)
            t += 0.1
            visited.append(info.get("prediction_status"))
            if info["finished"]:
                break
        self.assertTrue(info["finished"])
        self.assertIn("square leg 2: +-2.50 m left line", visited)
        self.assertIn("square leg 9: +-1.50 m right line", visited)

    def test_left_line_leg_steers_onto_the_bank_line(self):
        self.nav.enter_auto_mode(*self.latlon(10.0, 0.0), now_mono=0.0)
        self.step(13.0, -2.5, 0.1)                              # Reach the first corner
        info = self.step(16.0, -2.0, 0.2)                       # 0.5 m right of the -2.5 m line, heading along it
        self.assertEqual(info["prediction_status"], "square leg 2: +-2.50 m left line")
        self.assertAlmostEqual(info["cte_right_m"], 0.5, places=3)
        self.assertLess(info["steer_norm"], 0.0)                # Steer left, toward the bank line


class AutoLoopFeatureTest(unittest.TestCase):
    """Real main loop with mocked hardware; the boat stays put while AUTO commands forward."""

    def run_loop(self, argv=(), speed=0.0, cycles=200, recovery=None, test_square=False, course_deg=0.0,
                 offset_m=0.0):
        cal = mb.load_calibration(Path(mb.__file__).parent / "calibration.json")
        cal["mode_switch"]["right_steer_hold_sec"] = .05
        cal["autonomous"]["recovery"] = dict(cal["autonomous"]["recovery"], **(recovery or {}))
        cal["autonomous"][LINE]["test_square"] = test_square
        clock, cycle = [100.0], [0]
        gps = mb.ZEDF9PReceiver("unused")
        # The harness course runs due north along longitude 129; offset_m moves the boat east (right).
        gps.lat, gps.lon, gps.fix_quality = 35.0001, 129. + math.degrees(
            offset_m / (mb.EARTH_RADIUS_M * math.cos(math.radians(35.)))), 4
        gps.last_fix_time = 100.
        with patch.object(mb.time, "monotonic", return_value=100.):
            base = gps.get_snapshot()
        pca, rc = Mock(), Mock()
        ntrip = mb.NGII_NtripClient({"enabled": False}, gps)

        def snapshot():
            return dict(base, valid=True, age_sec=0., course_valid=True, course_deg=course_deg,
                        course_speed_mps=speed, course_age_sec=0., course_time_mono=clock[0], course_gga_skew_sec=0.,
                        course_utc="2026-11-16T12:00:00Z", sog_mps=speed, cog_deg=course_deg)

        def receiver():
            cycle[0] += 1
            if cycle[0] > cycles:
                raise KeyboardInterrupt()
            # Hold right stick only long enough to engage AUTO, then let go.
            steer = cal["rc"]["steer_right_us"] if 20 < cycle[0] <= 25 else cal["rc"]["steer_neutral_us"]
            return steer, cal["rc"]["throttle_neutral_us"], True

        def waypoints(filename):
            offset = {"left.csv": -1.5, "center.csv": 0., "right.csv": 1.5}[filename]
            longitude = 129. + math.degrees(offset / (mb.EARTH_RADIUS_M * math.cos(math.radians(35.))))
            return [(1, 35., longitude), (2, 35.001, longitude), (3, 35.002, longitude)]

        def sleep(delay):
            clock[0] += delay

        rc.get_latest.side_effect = receiver
        with tempfile.TemporaryDirectory() as folder, contextlib.ExitStack() as stack:
            stack.callback(os.chdir, os.getcwd())
            os.chdir(folder)
            out = stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            for name in ("serial", "SMBus"):
                stack.enter_context(patch.object(mb, name, Mock()))
            for name, result in (("load_calibration", cal), ("PCA9685Driver", pca), ("ArduinoRCReader", rc),
                                 ("ZEDF9PReceiver", gps), ("NGII_NtripClient", ntrip),
                                 ("detect_serial_ports", ("unused-rc", "unused-gps"))):
                stack.enter_context(patch.object(mb, name, return_value=result))
            stack.enter_context(patch.object(mb, "load_waypoints_csv", side_effect=waypoints))
            for obj in (gps, ntrip):
                for name in ("start", "stop"):
                    stack.enter_context(patch.object(obj, name))
            stack.enter_context(patch.object(gps, "get_snapshot", side_effect=snapshot))
            stack.enter_context(patch.object(mb.signal, "signal"))
            stack.enter_context(patch.object(mb.time, "monotonic", side_effect=lambda: clock[0]))
            stack.enter_context(patch.object(mb.time, "time", side_effect=lambda: clock[0]))
            stack.enter_context(patch.object(mb.time, "sleep", side_effect=sleep))
            stack.enter_context(patch.object(mb.sys, "argv", ["multipath_boat.py", "--no-spresense", *argv]))
            with self.assertRaises(KeyboardInterrupt):
                mb.main("feature-test")
            with open("logs/multipath_log_feature-test.csv", newline="") as f:
                rows = list(csv.DictReader(f))
        return rows, out.getvalue(), cal

    FAST = {"stuck_sec": 0.5, "startup_grace_sec": 0.5, "neutral_sec": 0.1, "reverse_sec": 0.5, "max_attempts": 1}

    def test_stuck_boat_reverses_then_gives_up_to_manual(self):
        rows, events, cal = self.run_loop(recovery=self.FAST)
        states = [r["recovery_state"] for r in rows]
        for state in ("neutral_in", "reverse", "neutral_out", "gave_up"):
            self.assertIn(state, states)
        pwm = cal["pwm"]
        reverse_us = mb.map_norm_to_pwm(-0.6, pwm["esc_neutral_us"], pwm["esc_rev_max_us"], pwm["esc_fwd_max_us"])
        for r in rows:
            if r["recovery_state"] == "reverse":
                self.assertEqual((int(r["esc_us"]), int(r["servo_us"])), (reverse_us, pwm["servo_center_us"]))
        give_up = states.index("gave_up")
        self.assertEqual(int(rows[give_up]["esc_us"]), pwm["esc_neutral_us"])
        self.assertTrue(all(r["mode"] == "MANUAL" for r in rows[give_up + 1:]))
        self.assertIn("[RECOVERY] reverse", events)
        # Between the reverse and giving up, AUTO drove forward again (the retry).
        retry = [r for r in rows[states.index("neutral_out"):give_up] if r["recovery_state"] == ""]
        self.assertTrue(retry and all(int(r["esc_us"]) > pwm["esc_neutral_us"] for r in retry))

    def test_no_recovery_flag_keeps_driving(self):
        rows, _, _ = self.run_loop(argv=["--no-recovery"], recovery=self.FAST)
        self.assertTrue(all(r["recovery_state"] == "" for r in rows))
        self.assertEqual(rows[-1]["mode"], "AUTO")

    def test_moving_boat_never_recovers(self):
        rows, _, _ = self.run_loop(speed=1.0, recovery=self.FAST)
        self.assertTrue(all(r["recovery_state"] == "" for r in rows))

    def test_use_04_flag_runs_04(self):
        rows, _, _ = self.run_loop(argv=["--use-04"], speed=1.0, cycles=60)
        self.assertEqual(rows[-1]["auto_strategy"], "04_predictive_recovery")

    def test_test_square_flag_and_limits_from_the_command_line(self):
        rows, events, cal = self.run_loop(argv=["--test-square", "--bank-limit", "3", "--line-limit", "1.2"],
                                          speed=1.0, cycles=60)
        self.assertIn("+-3.00 m (bank lines) then +-1.20 m (line limits)", events)
        auto = [r for r in rows if r["mode"] == "AUTO"][1:]
        self.assertTrue(auto)
        self.assertTrue(all(r["prediction_status"].startswith("square leg 1: +-3.00 m") for r in auto))
        pwm = cal["pwm"]
        self.assertEqual(int(auto[-1]["esc_us"]),
                         mb.map_norm_to_pwm(0.6, pwm["esc_neutral_us"], pwm["esc_rev_max_us"], pwm["esc_fwd_max_us"]))


if __name__ == "__main__":
    unittest.main()
