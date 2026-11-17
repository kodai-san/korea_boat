"""Closed-loop mode 06 checks against the fitted plant, not counterfactual log replay.

Requires the simulator's optional NumPy dependency. Observational faults affect
only the receiver snapshots; commands drive the independent boat state at 20 Hz.
"""

import copy
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
try:
    import numpy as np
except ImportError:
    raise unittest.SkipTest("Install NumPy to run the fitted-plant simulation checks")

import multipath_boat as mb  # noqa: E402
from boat_model import BoatModel, GPSEmulator, DT, GPS_LATENCY_SEC  # noqa: E402
from simulate_boat import AutoDriver, make_navigator, simulate_auto_once  # noqa: E402


class ObservationFault:
    """Inject faults after the receiver's 5 Hz sampling and transport delay."""

    def __init__(self, kind, jump_m=0.0):
        self.kind, self.jump_m = kind, jump_m
        self.last = None

    def __call__(self, snap, now):
        snap = dict(snap)
        epoch = round(snap["course_time_mono"], 6)
        if self.kind == "jump" and 12.0 <= epoch < 12.4:
            # A changed GNSS solution, while Doppler velocity remains physical.
            snap["lat"] += math.degrees(self.jump_m / 6378137.0)
        elif self.kind == "position_outage" and 12.0 <= now < 13.0:
            for key in ("lat", "lon", "gps_utc", "fix_quality"):
                snap[key] = self.last[key]
            snap["age_sec"] = now - self.last["_received"] + self.last["age_sec"]
            snap["course_gga_skew_sec"] = snap["course_time_mono"] - self.last["course_time_mono"]
        elif self.kind == "quality":
            if 12.0 <= now < 16.0:
                snap["fix_quality"] = 5
            elif 16.0 <= now < 20.0:
                snap["fix_quality"] = 2
            elif 20.0 <= now < 24.0:
                snap["fix_quality"] = 1
                snap["diff_age_sec"] = 999.0  # Standalone GPS does not depend on correction age.
            elif 24.0 <= now < 25.0:
                snap["fix_quality"], snap["valid"] = 0, False
        else:
            self.last = dict(snap, _received=now)
        return snap


class GpsGuardSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.calibration = mb.load_calibration(ROOT / "calibration.json")
        points = mb.load_waypoints_csv(str(ROOT / "center.csv"))
        ref = points[0]
        x, y = mb.latlon_to_enu(points[1][1], points[1][2], ref[1], ref[2])
        cls.bearing = math.atan2(x, y)
        cls.paths = {lane: str(ROOT / filename) for lane, filename in mb.PATH_FILES.items()}

    def run_boat(self, *, speed=1.4, fault=None, max_time=110.0,
                 strategy="06_gps_aware", model_file="boat_model_params.json", disturbed=False,
                 gust=0.0, environment=None, seed=2):
        cal = copy.deepcopy(self.calibration)
        cal["autonomous"]["strategy"] = strategy
        cal["autonomous"]["use_04_control"] = False
        args = SimpleNamespace(throttle_ratio=None, start_lane="center", start_offset=0.0,
                               start_heading=0.0, gps_noise=0.02, gps_bias_walk=0.0,
                               gust=gust, max_time=max_time)
        if environment is None:
            environment = ([0.02, -0.01], 1.0, 0.05) if disturbed else ([0.0, 0.0], 1.0, 0.0)
        with patch.dict(mb.PATH_FILES, self.paths):
            return simulate_auto_once(
                args, BoatModel.load(ROOT / model_file), environment, cal,
                mb.MultipathNavigator, seed=seed,
                init_state=(0.0, 0.0, self.bearing, speed, 0.0), snapshot_filter=fault)

    def test_clean_moving_entry_keeps_full_speed_and_completes_across_models(self):
        for model in ("boat_model_params.json", "boat_model_params_holdout_run1.json",
                      "boat_model_params_v1.json"):
            with self.subTest(model=model):
                original = self.run_boat(strategy="05_straight_line", model_file=model, disturbed=True)
                guarded = self.run_boat(model_file=model, disturbed=True)
                self.assertTrue(original["finished"] and guarded["finished"])
                self.assertEqual(max(guarded["esc"]), 1990)
                self.assertLess(guarded["max_abs_lateral_m"], 1.0)
                # Allow acquisition/ramp overhead, while catching an unusably slow default.
                self.assertLess(guarded["time_s"] - original["time_s"], 5.0)
                self.assertTrue(math.isfinite(guarded["mean_abs_cte_m"]))

    def test_stationary_entry_without_direction_stays_neutral(self):
        result = self.run_boat(speed=0.0, max_time=8.0)
        self.assertFalse(result["finished"])
        self.assertTrue(np.all(result["esc"] == 1800))
        self.assertTrue(np.allclose(result["true_xy"], 0.0))
        self.assertTrue(np.all(np.isnan(result["cte"])))
        self.assertTrue(all(state == "HOLD" for state in result["guard_states"]))

    def test_recent_manual_heading_allows_bounded_launch_after_coasting(self):
        result = self.run_boat(max_time=10.0)
        acquisition = [power for state, power in zip(result["guard_states"], result["throttle_norm"])
                       if state == "ACQUIRE"]
        self.assertTrue(acquisition)
        self.assertGreater(max(acquisition), 0.25)
        self.assertLessEqual(max(acquisition), 0.3)
        self.assertLessEqual(len(acquisition) * 0.05, 1.6)
        self.assertEqual(result["guard_states"][-1], "RACE")
        self.assertEqual(result["esc"][-1], 1990)

    def run_neutral_gesture(self, quality=4, max_time=110.0, snapshot_filter=None):
        """Exercise real manual/coast dynamics before AUTO, at one receiver quality."""
        cal = copy.deepcopy(self.calibration)
        cal["autonomous"]["strategy"] = "06_gps_aware"
        paths = {lane: mb.load_waypoints_csv(path) for lane, path in self.paths.items()}
        nav = make_navigator(mb.MultipathNavigator, paths, cal)
        driver = AutoDriver(nav, snapshot_filter=snapshot_filter)
        model = BoatModel.load(ROOT / "boat_model_params.json")
        gps = GPSEmulator(noise_m=0.02, seed=2, fix_quality=quality)
        state = (0.0, 0.0, self.bearing, 1.4, 0.0)
        velocity = (state[3] * math.sin(state[2]), state[3] * math.cos(state[2]))
        gps.update(0.0, *state[:2], *velocity)
        gps_history = [(*state[:2], *velocity)] * (round(GPS_LATENCY_SEC / DT) + 1)
        servos = [1750] * (model.servo_delay + 1)
        powers = [1990] * (model.esc_delay + 1)
        records, entry_speed = [], None
        for index in range(round(max_time / DT)):
            now = index * DT
            if now < 3.0:
                # One second of forward MANUAL, then the actual two-second
                # neutral-throttle gesture; observe every real control cycle.
                manual_throttle = 1.0 if now < 1.0 else 0.0
                driver.guard.observe_manual(driver._snapshot(gps, now), manual_throttle, now)
                servo, esc = 1750, 1990 if manual_throttle else 1800
            else:
                if index == 60:
                    entry_speed = float(state[3])
                    driver.enter(gps, now)  # No synthetic moving-entry history.
                info = driver.update(gps, now)
                servo = mb.map_norm_to_pwm(info["steer_norm"], 1750, 2130, 1030)
                esc = mb.map_norm_to_pwm(info["throttle_norm"], 1800, 1650, 1990)
                records.append(dict(time=now, state=info["gps_guard_state"], reason=info["gps_guard_reason"],
                                    throttle=info["throttle_norm"], finished=info["finished"],
                                    esc=esc, servo=servo, true_xy=state[:2], surge=float(state[3])))
                if info["finished"]:
                    break
            servos.append(servo)
            powers.append(esc)
            state = model.step(state, servos.pop(0), powers.pop(0))
            velocity = (state[3] * math.sin(state[2]), state[3] * math.cos(state[2]))
            gps_history.append((*state[:2], *velocity))
            gps.update((index + 1) * DT, *gps_history.pop(0))
        return records, entry_speed

    def test_two_second_neutral_auto_gesture_preserves_launch_direction(self):
        records, entry_speed = self.run_neutral_gesture()
        self.assertIsNotNone(entry_speed)
        self.assertLess(entry_speed, 0.15)  # The gesture genuinely lost moving-course speed.
        acquisition = [record for record in records if record["state"] == "ACQUIRE"]
        self.assertTrue(acquisition)
        self.assertGreaterEqual(acquisition[0]["time"], 5.0)  # Two-second AUTO qualification also elapsed.
        self.assertLessEqual(max(record["throttle"] for record in acquisition), 0.3)
        self.assertLessEqual(len(acquisition) * DT, 1.6)
        self.assertEqual(max(record["throttle"] for record in records), 1.0)
        self.assertTrue(records[-1]["finished"])

    def test_float_dgps_and_gps_launch_after_neutral_gesture_and_make_progress(self):
        for quality, ratio, pwm in ((5, 0.7, 1933), (2, 0.5, 1895), (1, 0.3, 1857)):
            with self.subTest(quality=quality):
                def ignore_correction_age(snap, now):
                    if quality == 1:
                        snap = dict(snap, diff_age_sec=999.0)
                    return snap
                records, entry_speed = self.run_neutral_gesture(
                    quality=quality, max_time=30.0, snapshot_filter=ignore_correction_age)
                self.assertLess(entry_speed, 0.15)
                acquisition = [r for r in records if r["state"] == "ACQUIRE"]
                self.assertTrue(acquisition)
                self.assertLessEqual(max(r["throttle"] for r in acquisition), 0.3)
                self.assertLessEqual(len(acquisition) * DT, 1.6)
                steady = [r for r in records if r["time"] >= 10.0]
                self.assertTrue(all(r["state"] == "CAUTION" for r in steady))
                self.assertTrue(all(r["throttle"] <= ratio for r in records))
                self.assertTrue(all(r["esc"] == pwm for r in steady))
                self.assertGreater(records[-1]["surge"], 0.45)
                progress = np.linalg.norm(np.array(records[-1]["true_xy"]) - records[0]["true_xy"])
                self.assertGreater(progress, 12.0)  # Commanded power really moves the plant.

    def test_bad_standalone_gps_stops_then_recovers_at_30_percent(self):
        for fault in ("no_fix", "poor_hdop", "large_accuracy"):
            with self.subTest(fault=fault):
                def bad_gps(snap, now):
                    snap = dict(snap, diff_age_sec=999.0)
                    if 12.0 <= now < 13.0:
                        if fault == "no_fix":
                            snap.update(fix_quality=0, valid=False)
                        elif fault == "poor_hdop":
                            snap["hdop"] = 5.0
                        else:
                            snap.update(hacc_m=2.0, hacc_age_sec=0.0, hacc_gga_skew_sec=0.0)
                    elif fault == "large_accuracy":
                        # A fresh usable replacement is needed to clear retained bad accuracy.
                        snap.update(hacc_m=0.5, hacc_age_sec=0.0, hacc_gga_skew_sec=0.0)
                    return snap
                records, _ = self.run_neutral_gesture(quality=1, max_time=30.0, snapshot_filter=bad_gps)
                affected = [r for r in records if 12.0 <= r["time"] < 13.0]
                self.assertTrue(affected)
                self.assertTrue(all((r["state"], r["esc"], r["servo"]) == ("HOLD", 1800, 1750)
                                    for r in affected))
                steady = [r for r in records if r["time"] >= 20.0]
                self.assertTrue(all((r["state"], r["esc"]) == ("CAUTION", 1857) for r in steady))
                self.assertGreater(records[-1]["surge"], 0.45)

    def test_large_position_jumps_do_not_move_truth_or_command_bank_turns(self):
        results = []
        for metres in (22.0, 64.0):
            with self.subTest(jump_m=metres):
                result = self.run_boat(fault=ObservationFault("jump", metres))
                affected = (result["sample_time_s"] >= 12.0) & (result["sample_time_s"] < 12.4)
                self.assertTrue(np.any(affected))
                self.assertTrue(np.all(result["esc"][affected] == 1800))
                self.assertTrue(np.all(result["servo"][affected] == 1750))
                self.assertIn("position_jump", result["guard_reasons"])
                self.assertTrue(result["finished"])
                self.assertGreater(result["time_s"], 60.0)  # No finish caused by the corrupted fix.
                self.assertLess(result["max_abs_lateral_m"], 0.5)
                self.assertEqual(max(result["esc"][result["sample_time_s"] > 20.0]), 1990)
                results.append(result)
        # Both outliers lead to the same rejected-observation controls and physical motion.
        np.testing.assert_allclose(results[0]["true_xy"], results[1]["true_xy"])

    def test_gga_outage_with_fresh_rmc_holds_then_resumes_in_closed_loop(self):
        result = self.run_boat(fault=ObservationFault("position_outage"))
        stale = (result["sample_time_s"] >= 12.65) & (result["sample_time_s"] < 13.0)
        self.assertTrue(np.all(result["esc"][stale] == 1800))
        self.assertIn("position_stale", result["guard_reasons"])
        self.assertTrue(result["finished"])
        self.assertLess(result["max_abs_lateral_m"], 0.5)
        self.assertEqual(max(result["esc"][result["sample_time_s"] > 20.0]), 1990)

    def test_float_dgps_gps_caps_slow_the_plant_no_fix_stops_and_fix_restores_race(self):
        result = self.run_boat(fault=ObservationFault("quality"))
        time = result["sample_time_s"]
        for start, end, cap in ((12.1, 15.9, 1933), (16.1, 19.9, 1895), (20.1, 23.9, 1857)):
            with self.subTest(start=start):
                window = (time >= start) & (time < end)
                self.assertTrue(np.all(result["esc"][window] == cap))
        no_fix = (time >= 24.0) & (time < 25.0)
        self.assertTrue(np.all(result["esc"][no_fix] == 1800))
        positions = result["true_xy"]
        displacement_speed = np.linalg.norm(np.diff(positions, axis=0), axis=1) / 0.05
        initial = np.mean(displacement_speed[(time[:-1] >= 9.0) & (time[:-1] < 11.0)])
        gps = np.mean(displacement_speed[(time[:-1] >= 22.0) & (time[:-1] < 24.0)])
        self.assertLess(gps, initial * 0.65)
        self.assertEqual(max(result["esc"][time > 30.0]), 1990)
        self.assertTrue(result["finished"])
        self.assertLess(result["max_abs_lateral_m"], 0.5)

    def test_emulated_source_utc_is_valid_at_minute_and_day_rollover(self):
        gps = GPSEmulator(noise_m=0.0)
        gps.update(0.0, 0.0, 0.0, 1.0, 0.0)
        for at, expected in ((59.999999999, "000100.00"), (3599.999999999, "010000.00"),
                             (86399.999999999, "000000.00")):
            with self.subTest(at=at):
                gps.fix_t = at
                self.assertEqual(gps.snapshot(at)["gps_utc"], expected)

    def test_fault_recovery_remains_inside_band_with_gust_current_and_trim(self):
        for kind in ("jump", "position_outage", "quality"):
            with self.subTest(fault=kind):
                result = self.run_boat(fault=ObservationFault(kind, 64.0), gust=3.0,
                                       environment=([-0.10, 0.05], 1.0, 0.05))
                self.assertTrue(result["finished"])
                self.assertLess(result["max_abs_lateral_m"], 2.5)
                self.assertEqual(max(result["esc"]), 1990)


if __name__ == "__main__":
    unittest.main()
