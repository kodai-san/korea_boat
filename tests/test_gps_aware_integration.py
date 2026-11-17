"""Mode 06 through the real control loop, with hardware replaced by fakes."""

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


class GpsAwareIntegrationTests(unittest.TestCase):
    def run_loop(self, scenario, guard_options=None):
        cal = boat.load_calibration(Path(boat.__file__).parent / "calibration.json")
        cal["autonomous"]["strategy"] = "06_gps_aware"
        cal["autonomous"]["use_04_control"] = False
        cal["autonomous"]["06_gps_aware"].update(guard_options or {})
        cal["pwm"]["auto_throttle_ratio"] = 1.0
        cal["mode_switch"]["right_steer_hold_sec"] = .05
        clock, cycle = [100.], [0]
        gps = boat.ZEDF9PReceiver("unused")
        rc, pca = Mock(), Mock()
        ntrip = boat.NGII_NtripClient({"enabled": False}, gps)
        outputs = []

        def record_output(channel, value):
            outputs.append((round(clock[0] - 100., 6), channel, value))

        pca.set_pwm_us.side_effect = record_output

        def snapshot():
            t = round(clock[0] - 100., 6)
            epoch = math.floor((t + 1e-7) * 5) / 5
            x, quality, valid = 0., 4, True
            if scenario == "quality":
                if 6 <= t < 8:
                    quality = 5
                elif 8 <= t < 10:
                    quality = 2
                elif 10 <= t < 12:
                    quality = 1
                elif 12 <= t < 13:
                    quality, valid = 0, False
            elif scenario == "gps_only":
                quality = 1
            elif scenario == "jump" and 6 <= t < 6.2:
                x = 30.
            elif scenario == "duplicates" and 6 <= t < 7.5:
                epoch = 5.8
            utc = f"1200{epoch:05.2f}"
            return dict(
                valid=valid, fix_quality=quality, lat=35. + math.degrees((10 + epoch) / boat.EARTH_RADIUS_M),
                lon=129. + math.degrees(x / (boat.EARTH_RADIUS_M * math.cos(math.radians(35.)))),
                age_sec=0. if scenario == "duplicates" else t - epoch,
                num_sats=20, hdop=.7, gps_utc=utc,
                diff_age_sec=99. if quality == 1 else 1., alt_m=0.,
                heading_deg=0., sog_mps=1., cog_deg=0., course_valid=True,
                course_deg=0., course_speed_mps=1., course_age_sec=0.,
                course_time_mono=clock[0], course_gga_skew_sec=0.,
                course_utc=f"2026-11-17T12:00:{t:06.3f}Z",
                position_heading_deg=None, position_heading_age_sec=None,
                position_heading_time_mono=None, position_speed_mps=None,
                rtcm_bytes_written=0, rtcm_write_errors=0, last_rtcm_write_error="",
            )

        def receiver():
            cycle[0] += 1
            if cycle[0] > 400:
                raise KeyboardInterrupt()
            t = round(clock[0] - 100., 6)
            steer = cal["rc"]["steer_neutral_us"] if t < 1. else cal["rc"]["steer_right_us"]
            throttle = cal["rc"]["throttle_fwd_us"] if t < 1. else cal["rc"]["throttle_neutral_us"]
            if scenario == "override" and t >= 6:
                throttle = cal["rc"]["throttle_rev_us"]
            if scenario == "finish" and t >= 10:
                steer = cal["rc"]["steer_neutral_us"]
            return steer, throttle, not (scenario == "rc_loss" and t >= 6)

        def paths(filename):
            offset = {"left.csv": -1.5, "center.csv": 0., "right.csv": 1.5}[filename]
            lon = 129. + math.degrees(offset / (boat.EARTH_RADIUS_M * math.cos(math.radians(35.))))
            end = 35. + math.degrees(20 / boat.EARTH_RADIUS_M) if scenario == "finish" else 35.005
            return [(1, 35., lon), (2, end, lon)]

        def sleep(delay):
            # Skip the hardware startup delay in the synthetic timeline.
            if delay != 2.:
                clock[0] += delay

        rc.get_latest.side_effect = receiver
        with tempfile.TemporaryDirectory() as folder, contextlib.ExitStack() as stack:
            stack.callback(os.chdir, os.getcwd())
            os.chdir(folder)
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            for name in ("serial", "SMBus"):
                stack.enter_context(patch.object(boat, name, Mock()))
            for name, obj in (("load_calibration", cal), ("PCA9685Driver", pca),
                              ("ArduinoRCReader", rc), ("ZEDF9PReceiver", gps),
                              ("NGII_NtripClient", ntrip), ("detect_serial_ports", ("rc", "gps"))):
                stack.enter_context(patch.object(boat, name, return_value=obj))
            for obj in (gps, ntrip):
                for name in ("start", "stop"):
                    stack.enter_context(patch.object(obj, name))
            stack.enter_context(patch.object(gps, "get_snapshot", side_effect=snapshot))
            stack.enter_context(patch.object(boat, "load_waypoints_csv", side_effect=paths))
            stack.enter_context(patch.object(boat.signal, "signal"))
            stack.enter_context(patch.object(boat.time, "monotonic", side_effect=lambda: clock[0]))
            stack.enter_context(patch.object(boat.time, "time", side_effect=lambda: clock[0]))
            stack.enter_context(patch.object(boat.time, "sleep", side_effect=sleep))
            stack.enter_context(patch.object(boat.sys, "argv", ["multipath_boat.py", "--no-spresense"]))
            reverse = stack.enter_context(patch.object(boat.StuckRecovery, "update"))
            handler = boat.open_session_log("guarded")
            stack.callback(handler.close)
            stack.callback(boat.SESSION_LOG.removeHandler, handler)
            with self.assertRaises(KeyboardInterrupt):
                boat.main("guarded")
            reverse.assert_not_called()
            with open("logs/multipath_log_guarded.csv", newline="") as f:
                rows = list(csv.DictReader(f))
            events = Path("logs/multipath_events_guarded.log").read_text()
        self.assertTrue(rows)
        self.assertTrue(all(None not in row for row in rows))
        self.outputs = outputs
        self.pwm = cal["pwm"]
        self.hardware = cal["hardware"]
        return rows, events

    def test_actual_pwm_downgrades_all_gps_levels_and_resumes_after_no_fix(self):
        rows, events = self.run_loop("quality")
        auto = [r for r in rows if r["mode"] == "AUTO"]
        self.assertEqual(int(auto[0]["esc_us"]), 1800)
        self.assertTrue(any(r["gps_guard_state"] == "RACE" and int(r["esc_us"]) == 1990 for r in auto))
        for quality, ceiling in (("5", 1933), ("2", 1895), ("1", 1857)):
            selected = [r for r in auto if r["fix_quality"] == quality]
            self.assertTrue(selected)
            self.assertTrue(all(int(r["esc_us"]) <= ceiling for r in selected))
            self.assertEqual(max(int(r["esc_us"]) for r in selected), ceiling)
        missing = [r for r in auto if r["fix_quality"] == "0"]
        self.assertTrue(missing)
        self.assertTrue(all((int(r["esc_us"]), int(r["servo_us"])) == (1800, 1750) for r in missing))
        self.assertEqual((auto[-1]["gps_guard_state"], int(auto[-1]["esc_us"])), ("RACE", 1990))
        self.assertIn("[GPS GUARD]", events)

    def test_ordinary_gps_can_enter_auto_and_move_without_rtk(self):
        rows, _ = self.run_loop("gps_only")
        auto = [r for r in rows if r["mode"] == "AUTO"]
        self.assertTrue(auto)
        self.assertTrue(all(int(r["esc_us"]) <= 1857 for r in auto))
        self.assertEqual((auto[-1]["gps_guard_state"], int(auto[-1]["esc_us"])), ("CAUTION", 1857))
        self.assertTrue(all(r["gps_guard_state"] != "RACE" for r in auto))

    def test_position_jump_cannot_command_a_bank_turn_or_finish(self):
        rows, _ = self.run_loop("jump")
        jumps = [r for r in rows if 106 <= float(r["timestamp"]) < 106.2]
        self.assertTrue(jumps)
        self.assertTrue(all(r["mode"] == "AUTO" and r["gps_guard_state"] == "HOLD" for r in jumps))
        self.assertTrue(all((int(r["esc_us"]), int(r["servo_us"])) == (1800, 1750) for r in jumps))

    def test_json_power_settings_reach_hardware_outputs(self):
        rows, _ = self.run_loop("quality", dict(race_throttle=.8, float_throttle=.4,
                                                dgps_throttle=.25, gps_throttle=.15, boundary_throttle=.25,
                                                launch_throttle=.25))
        auto = [r for r in rows if r["mode"] == "AUTO"]
        for quality, maximum in (("4", 1952), ("5", 1876), ("2", 1848), ("1", 1828)):
            outputs = [int(r["esc_us"]) for r in auto if r["fix_quality"] == quality]
            self.assertTrue(outputs)
            self.assertEqual(max(outputs), maximum)

    def test_repeated_source_position_does_not_keep_throttle_alive(self):
        rows, _ = self.run_loop("duplicates")
        stale = [r for r in rows if 106.5 <= float(r["timestamp"]) < 107.5]
        self.assertTrue(stale)
        self.assertTrue(all(r["gps_guard_state"] == "HOLD" and int(r["esc_us"]) == 1800 for r in stale))

    def test_transmitter_reverse_takes_over_immediately(self):
        rows, _ = self.run_loop("override")
        overridden = [r for r in rows if float(r["timestamp"]) >= 106]
        self.assertTrue(overridden)
        self.assertTrue(all(r["mode"] == "MANUAL" and int(r["esc_us"]) == 1650 for r in overridden))

    def test_accepted_positions_finish_at_the_configured_margin(self):
        rows, events = self.run_loop("finish")
        before = [r for r in rows if 110 <= float(r["timestamp"]) < 110.9]
        after = [r for r in rows if float(r["timestamp"]) >= 111.3]
        self.assertTrue(before and after)
        self.assertTrue(all(r["mode"] == "AUTO" for r in before))
        self.assertTrue(all(r["mode"] == "MANUAL" and int(r["esc_us"]) == 1800 for r in after))
        self.assertIn("[COURSE COMPLETE]", events)

    def test_lost_transmitter_signal_stops_both_outputs(self):
        rows, events = self.run_loop("rc_loss")
        self.assertTrue(any(int(row["esc_us"]) == 1990 for row in rows))
        stopped = [(channel, value) for t, channel, value in self.outputs if t >= 6]
        self.assertTrue(stopped)
        expected = {self.hardware["servo_channel"]: self.pwm["servo_center_us"],
                    self.hardware["esc_channel"]: self.pwm["esc_neutral_us"]}
        self.assertTrue(all(value == expected[channel] for channel, value in stopped))


if __name__ == "__main__":
    unittest.main()
