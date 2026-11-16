"""Offline strategy selection and AUTO output checks; hardware/network are mocked."""

import contextlib
import copy
import csv
import io
import math
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import auto_strategies as strategies
import calibrate_tool
import multipath_boat as boat


class AutoStrategyTests(unittest.TestCase):
    def create(self, name="adaptive_throttle", options=None):
        config = {"strategy": name}
        if options is not None:
            config[name] = options
        return strategies.create_auto_strategy(config, boat.NAV_PARAMS)

    def update(self, strategy, cte=0.0, heading=0.0, guard=False):
        return strategy.update(cte_right_m=cte, heading_error_deg=heading,
                               boundary_guard=guard, now_mono=100.0)

    def test_missing_config_preserves_current_adaptive_behavior(self):
        strategy = strategies.create_auto_strategy({}, boat.NAV_PARAMS)
        self.assertEqual(strategy.name, "adaptive_throttle")
        self.assertEqual(self.update(strategy)["throttle_blend"], 1.0)

    def test_default_configs_are_independent(self):
        first = strategies.default_autonomous_config()
        first["adaptive_throttle"]["full_throttle_cte_m"] = 99
        second = strategies.default_autonomous_config()
        self.assertEqual(second["adaptive_throttle"]["full_throttle_cte_m"], 0.30)
        self.assertEqual(second["strategy"], strategies.DEFAULT_AUTO_STRATEGY)

    def test_adaptive_requires_both_limits_inclusive_in_both_directions(self):
        cases = [(0, 0, 1), (.3, 11, 1), (-.3, -11, 1),
                 (.300001, 0, 0), (-.300001, 0, 0),
                 (0, 11.000001, 0), (0, -11.000001, 0),
                 (.31, 12, 0), (0, None, 0)]
        for cte, heading, expected in cases:
            with self.subTest(cte=cte, heading=heading):
                self.assertEqual(self.update(self.create(), cte, heading)["throttle_blend"], expected)

    def test_configured_thresholds_take_effect_without_mutating_config(self):
        config = {"strategy": "adaptive_throttle", "adaptive_throttle": {
            "full_throttle_cte_m": .15, "full_throttle_heading_deg": 5}}
        original = copy.deepcopy(config)
        strategy = strategies.create_auto_strategy(config, boat.NAV_PARAMS)
        self.assertEqual(self.update(strategy, .15, 5)["throttle_blend"], 1)
        self.assertEqual(self.update(strategy, .2, 0)["throttle_blend"], 0)
        self.assertEqual(self.update(strategy, 0, 6)["throttle_blend"], 0)
        self.assertEqual(config, original)

    def test_constant_cruise_never_boosts_and_shares_steering(self):
        constant, adaptive = self.create("constant_cruise"), self.create()
        for cte, heading, guard in [(0, None, False), (0, 0, False),
                                    (.1, 10, False), (.8, 20, True),
                                    (-.8, -90, True), (0, 180, False)]:
            with self.subTest(cte=cte, heading=heading, guard=guard):
                original = self.update(constant, cte, heading, guard)
                current = self.update(adaptive, cte, heading, guard)
                self.assertEqual(original["throttle_blend"], 0)
                self.assertEqual(original["steer_norm"], current["steer_norm"])

    def test_reset_clears_steering_history_between_auto_runs(self):
        for name in strategies.AUTO_STRATEGIES:
            with self.subTest(strategy=name):
                strategy = self.create(name)
                for _ in range(8):
                    self.update(strategy, heading=90)
                strategy.reset()
                self.assertEqual(self.update(strategy, heading=None)["steer_norm"], 0)
                first = self.update(strategy, heading=90)
                self.assertAlmostEqual(first["steer_norm"], boat.NAV_PARAMS["servo_slew_per_cycle"])

    def test_invalid_configuration_fails_explicitly(self):
        invalid = [None, [], "adaptive_throttle", {"strategy": "typo"},
                   {"strategy": None}, {"strategy": []},
                   {"strategy": "adaptive_throttle", "adaptive_throttle": []},
                   {"strategy": "adaptive_throttle", "adaptive_throttle": {"typo": 1}}]
        for config in invalid:
            with self.subTest(config=config), self.assertRaises(ValueError):
                strategies.create_auto_strategy(config, boat.NAV_PARAMS)

    def test_invalid_threshold_values_are_rejected(self):
        for key in ("full_throttle_cte_m", "full_throttle_heading_deg"):
            for value in (-1, True, "0.3", None, float("nan"), float("inf"), 10 ** 400):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    self.create(options={key: value})
        with self.assertRaises(ValueError):
            self.create(options={"full_throttle_heading_deg": 181})

    def test_calibration_load_save_preserves_strategy_and_tuning(self):
        config = calibrate_tool.default_calibration()
        self.assertEqual(config["autonomous"]["strategy"], "adaptive_throttle")
        config["autonomous"]["strategy"] = "constant_cruise"
        config["autonomous"]["adaptive_throttle"]["full_throttle_cte_m"] = .12
        with tempfile.TemporaryDirectory() as folder, contextlib.redirect_stdout(io.StringIO()):
            path = Path(folder) / "calibration.json"
            calibrate_tool.save_config(config, path)
            loaded = calibrate_tool.load_config(path)
            calibrate_tool.save_config(loaded, path)
            result = boat.load_calibration(path)
        self.assertEqual(result["autonomous"], config["autonomous"])


class NavigatorStrategyTests(unittest.TestCase):
    @staticmethod
    def paths():
        return {lane: [(i + 1, math.degrees(i * 2 / boat.EARTH_RADIUS_M),
                        math.degrees(x / boat.EARTH_RADIUS_M)) for i in range(65)]
                for lane, x in (("left", -2), ("center", 0), ("right", 2))}

    def test_navigator_resets_strategy_and_finishes_without_control_update(self):
        strategy = Mock()
        strategy.update.return_value = {"steer_norm": .12, "throttle_blend": .5}
        nav = boat.MultipathNavigator(self.paths(), dict(boat.NAV_PARAMS), strategy=strategy)
        strategy.reset.reset_mock()
        lat = math.degrees(20 / boat.EARTH_RADIUS_M)
        nav.enter_auto_mode(lat, 0, now_mono=100)
        strategy.reset.assert_called_once()
        active = nav.update(lat, 0, 359, now_mono=101)
        self.assertEqual(active["steer_norm"], .12)
        self.assertEqual(active["throttle_blend"], .5)
        self.assertAlmostEqual(strategy.update.call_args.kwargs["heading_error_deg"], 1)
        strategy.update.reset_mock()
        done = nav.update(math.degrees(128 / boat.EARTH_RADIUS_M), 0, 0, now_mono=102)
        self.assertTrue(done["finished"])
        self.assertEqual(done["steer_norm"], 0)
        self.assertEqual(done["throttle_blend"], 0)
        strategy.update.assert_not_called()


class AutoControllerIntegrationTests(unittest.TestCase):
    def run_auto(self, strategy, args=(), end="tracking", omit_config=False):
        source = Path(boat.__file__).parent
        cal = boat.load_calibration(source / "calibration.json")
        cal["mode_switch"]["right_steer_hold_sec"] = .01
        if omit_config:
            cal.pop("autonomous", None)
        else:
            cal["autonomous"] = {"strategy": strategy}
        clock = [100.0]
        gps = boat.ZEDF9PReceiver("mock-gps")
        gps.lat, gps.lon, gps.heading_deg = 35.0001, 129.0, 0.0
        gps.fix_quality, gps.last_fix_time = 4, clock[0]
        ntrip = boat.NGII_NtripClient({"enabled": False}, gps)
        pca, rc = Mock(), Mock()
        cycles = [0]

        def receiver():
            cycles[0] += 1
            if cycles[0] > 3:
                raise KeyboardInterrupt()
            steering = cal["rc"]["steer_right_us"]
            throttle = cal["rc"]["throttle_neutral_us"]
            if cycles[0] == 3:
                if end == "no_fix":
                    gps.fix_quality = 0
                elif end == "reverse":
                    throttle = cal["rc"]["throttle_rev_us"]
                elif end == "finish":
                    gps.lat = 35.002
            return steering, throttle, True

        def sleep(delay):
            clock[0] += delay

        rc.get_latest.side_effect = receiver
        with tempfile.TemporaryDirectory() as folder, contextlib.ExitStack() as stack:
            previous_dir = os.getcwd()
            stack.callback(os.chdir, previous_dir)
            os.chdir(folder)
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            for name in ("serial", "SMBus"):
                stack.enter_context(patch.object(boat, name, Mock()))
            for name, result in (("load_calibration", cal), ("PCA9685Driver", pca),
                                 ("ArduinoRCReader", rc), ("ZEDF9PReceiver", gps),
                                 ("NGII_NtripClient", ntrip),
                                 ("detect_serial_ports", ("mock-rc", "mock-gps")),
                                 ("reserved_spresense_ports", ()),
                                 ("load_waypoints_csv", [(1, 35.0, 129.0),
                                                         (2, 35.001, 129.0),
                                                         (3, 35.002, 129.0)])):
                stack.enter_context(patch.object(boat, name, return_value=result))
            for obj in (gps, ntrip):
                for name in ("start", "stop"):
                    stack.enter_context(patch.object(obj, name))
            stack.enter_context(patch.object(gps, "reset_heading_anchor"))
            stack.enter_context(patch.object(boat.signal, "signal"))
            stack.enter_context(patch.object(boat.time, "monotonic", side_effect=lambda: clock[0]))
            stack.enter_context(patch.object(boat.time, "sleep", side_effect=sleep))
            stack.enter_context(patch.object(boat.sys, "argv", ["multipath_boat.py", "--no-spresense", *args]))
            events = stack.enter_context(patch.object(boat, "log_event", wraps=boat.log_event))
            with self.assertRaises(KeyboardInterrupt):
                boat.main("strategy-test")
            with open("logs/multipath_log_strategy-test.csv", newline="") as f:
                rows = list(csv.DictReader(f))
            event_text = "\n".join(c.args[0] for c in events.call_args_list)
        self.assertEqual(len(rows), 3)
        self.assertNotIn(None, rows[-1])
        self.assertIn("[AUTO STRATEGY]", event_text)
        return rows, pca, cal

    def test_selected_strategy_controls_actual_pwm_and_is_logged(self):
        for name, pwm, blend in (("constant_cruise", 1914, 0), ("adaptive_throttle", 1990, 1)):
            with self.subTest(strategy=name):
                rows, pca, cal = self.run_auto(name)
                self.assertEqual(rows[-1]["mode"], "AUTO")
                self.assertEqual(int(rows[-1]["esc_us"]), pwm)
                self.assertEqual(float(rows[-1]["throttle_blend"]), blend)
                self.assertEqual({r["auto_strategy"] for r in rows}, {name})
                pca.set_pwm_us.assert_any_call(cal["hardware"]["esc_channel"], pwm)

    def test_legacy_json_and_cli_override_select_expected_behavior(self):
        for args, omit in (((), True), (("--auto-strategy", "adaptive_throttle"), False)):
            with self.subTest(args=args, legacy_json=omit):
                rows, _, _ = self.run_auto("constant_cruise", args=args, omit_config=omit)
                self.assertEqual(rows[-1]["auto_strategy"], "adaptive_throttle")
                self.assertEqual(int(rows[-1]["esc_us"]), 1990)

    def test_strategy_cannot_override_finish_gps_stop_or_manual_return(self):
        for name in strategies.AUTO_STRATEGIES:
            for end, mode, pwm in (("no_fix", "AUTO", 1800), ("reverse", "MANUAL", 1650),
                                   ("finish", "MANUAL", 1800)):
                with self.subTest(strategy=name, end=end):
                    rows, _, _ = self.run_auto(name, end=end)
                    self.assertEqual(rows[-1]["mode"], mode)
                    self.assertEqual(int(rows[-1]["esc_us"]), pwm)

    def test_unknown_strategy_fails_before_initializing_hardware(self):
        cal = boat.load_calibration(Path(boat.__file__).parent / "calibration.json")
        cal["autonomous"] = {"strategy": "does_not_exist"}
        with contextlib.ExitStack() as stack:
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            stack.enter_context(contextlib.redirect_stderr(io.StringIO()))
            for name in ("serial", "SMBus"):
                stack.enter_context(patch.object(boat, name, Mock()))
            stack.enter_context(patch.object(boat, "load_calibration", return_value=cal))
            pca = stack.enter_context(patch.object(boat, "PCA9685Driver"))
            stack.enter_context(patch.object(boat.sys, "argv", ["multipath_boat.py"]))
            with self.assertRaises((ValueError, SystemExit)):
                boat.main("invalid-strategy")
            pca.assert_not_called()


if __name__ == "__main__":
    unittest.main()
