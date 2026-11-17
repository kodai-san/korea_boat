"""Offline tests for 05_straight_line: line geometry, course PI, bank protection and the real AUTO loop.

Run from the korea_boat folder:
    PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p 'test_*.py' -v
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import multipath_boat as mb  # noqa: E402
import test_course_auto_integration as harness  # noqa: E402
from auto_strategies import AUTO_STRATEGIES, canonical_auto_strategy_name, create_auto_strategy  # noqa: E402
from path_guidance import StraightLineGuidance  # noqa: E402

NAME = "05_straight_line"


def make_strategy(**options):
    return create_auto_strategy({"strategy": NAME, NAME: options}, mb.NAV_PARAMS)


class StraightLineGuidanceTest(unittest.TestCase):
    def setUp(self):
        # Course due east, 100 m long: right of the centerline is south (-y).
        self.g = StraightLineGuidance((0.0, 0.0), (100.0, 0.0), band_half_width_m=2.5, line_max_offset_m=1.5)

    def test_frame_and_point_are_inverse(self):
        self.assertAlmostEqual(self.g.bearing_deg, 90.0)
        s, d = self.g.frame(30.0, -2.0)
        self.assertAlmostEqual(s, 30.0)
        self.assertAlmostEqual(d, 2.0)
        x, y = self.g.point(s, d)
        self.assertAlmostEqual(x, 30.0)
        self.assertAlmostEqual(y, -2.0)

    def test_line_keeps_entry_offset_inside_limit(self):
        self.assertAlmostEqual(self.g.draw_line(5.0, -1.2), 1.2)
        self.assertAlmostEqual(self.g.entry_offset_m, 1.2)

    def test_line_moves_toward_centerline_near_a_bank(self):
        self.assertAlmostEqual(self.g.draw_line(5.0, -2.3), 1.5)
        self.assertAlmostEqual(self.g.draw_line(5.0, 2.4), -1.5)

    def test_desired_course_turns_toward_line_with_capped_intercept(self):
        self.g.draw_line(0.0, 0.0)
        self.assertAlmostEqual(self.g.desired_course(0.0, 3.0, 15.0), 90.0)
        # 0.5 m right of the line, 3 m lookahead: atan(0.5 / 3) = 9.5 deg to the left.
        self.assertAlmostEqual(self.g.desired_course(0.5, 3.0, 15.0), 90.0 - math.degrees(math.atan2(0.5, 3.0)))
        self.assertAlmostEqual(self.g.desired_course(2.0, 3.0, 15.0), 75.0)    # Capped
        self.assertAlmostEqual(self.g.desired_course(-2.0, 3.0, 15.0), 105.0)

    def test_lateral_rate_sign(self):
        self.assertAlmostEqual(self.g.lateral_rate(1.0, 90.0), 0.0)
        self.assertAlmostEqual(self.g.lateral_rate(1.0, 180.0), 1.0)          # Heading south = moving right
        self.assertAlmostEqual(self.g.lateral_rate(2.0, 60.0), -1.0)

    def test_rejects_line_limit_outside_band(self):
        with self.assertRaises(ValueError):
            StraightLineGuidance((0.0, 0.0), (100.0, 0.0), band_half_width_m=2.5, line_max_offset_m=2.5)


class StraightLineStrategyTest(unittest.TestCase):
    def test_registered_with_alias_and_heading_selector_options(self):
        self.assertIn(NAME, AUTO_STRATEGIES)
        self.assertEqual(canonical_auto_strategy_name("straight_line"), NAME)
        s = make_strategy()
        self.assertTrue(s.uses_gps_course and s.uses_straight_line)
        for key in ("min_course_speed_mps", "course_max_age_sec", "position_heading_max_age_sec",
                    "forward_acquire_sec", "forward_memory_sec"):
            self.assertIn(key, s.options)

    def test_rejects_bad_options(self):
        for options in ({"line_max_offset_m": 2.2}, {"lookahead_max_m": 1.0}, {"course_integral_gain": -1.0},
                        {"recovery_intercept_deg": 10.0}, {"unknown": 1.0}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                make_strategy(**options)

    def test_always_full_throttle(self):
        s = make_strategy()
        for herr in (0.0, 40.0, None):
            out = s.update(cte_right_m=2.0, heading_error_deg=herr, boundary_guard=True, now_mono=1.0)
            self.assertEqual(out["throttle_blend"], 1.0)

    def run_constant_error(self, s, error_deg=3.0, seconds=10.0, **kwargs):
        kw = dict(boundary_guard=False, heading_confident=True, speed_mps=1.5)
        kw.update(kwargs)
        out = None
        for k in range(int(seconds / 0.05)):
            out = s.update(cte_right_m=0.0, heading_error_deg=error_deg, now_mono=0.05 * k, **kw)
        return out

    def test_integral_learns_trim_after_launch_delay(self):
        s = make_strategy()
        early = self.run_constant_error(s, seconds=1.9)
        self.assertEqual(early["trim_deg"], 0.0)
        s = make_strategy()
        late = self.run_constant_error(s, seconds=10.0)
        self.assertGreater(late["trim_deg"], 3.0)
        self.assertLessEqual(late["trim_deg"], s.options["integral_limit_deg"] + 1e-9)

    def test_integral_frozen_when_slow_untrusted_near_bank_or_far_off_course(self):
        for kwargs in ({"speed_mps": 0.3}, {"speed_mps": None}, {"heading_confident": False},
                       {"boundary_guard": True}):
            with self.subTest(**{k: str(v) for k, v in kwargs.items()}):
                out = self.run_constant_error(make_strategy(), **kwargs)
                self.assertEqual(out["trim_deg"], 0.0)
        out = self.run_constant_error(make_strategy(), error_deg=30.0)
        self.assertEqual(out["trim_deg"], 0.0)

    def test_trim_held_without_heading_and_after_suspend_but_cleared_by_reset(self):
        s = make_strategy()
        learned = self.run_constant_error(s)["trim_deg"]
        held = s.update(cte_right_m=0.0, heading_error_deg=None, boundary_guard=False, now_mono=20.0)
        self.assertEqual(held["trim_deg"], learned)
        self.assertNotEqual(held["steer_norm"], 0.0)
        s.suspend()
        self.assertEqual(s.update(cte_right_m=0.0, heading_error_deg=None, boundary_guard=False,
                                  now_mono=30.0)["trim_deg"], learned)
        s.reset()
        out = s.update(cte_right_m=0.0, heading_error_deg=None, boundary_guard=False, now_mono=40.0)
        self.assertEqual((out["trim_deg"], out["steer_norm"]), (0.0, 0.0))

    def test_no_integral_option_is_pure_proportional(self):
        out = self.run_constant_error(make_strategy(course_integral_gain=0.0))
        self.assertEqual(out["trim_deg"], 0.0)
        self.assertAlmostEqual(out["steer_norm"], 3.0 / mb.NAV_PARAMS["max_steer_angle_deg"])


class StraightLineNavigatorTest(unittest.TestCase):
    def setUp(self):
        paths = {lane: mb.load_waypoints_csv(os.path.join(os.path.dirname(__file__), "..", p))
                 for lane, p in mb.PATH_FILES.items()}
        self.nav = mb.MultipathNavigator(paths, dict(mb.NAV_PARAMS, finish_margin_m=1.0), strategy=make_strategy())
        self.line = self.nav.line_guidance

    def latlon(self, s, d):
        x, y = self.line.point(s, d)
        lat = self.nav.ref_lat + math.degrees(y / mb.EARTH_RADIUS_M)
        lon = self.nav.ref_lon + math.degrees(x / (mb.EARTH_RADIUS_M * math.cos(math.radians(self.nav.ref_lat))))
        return lat, lon

    def step(self, s, d, heading_offset_deg=0.0, speed=1.5, t=1.0):
        return self.nav.update(*self.latlon(s, d), (self.line.bearing_deg + heading_offset_deg) % 360.0,
                               now_mono=t, heading_confident=True, motion={"speed_mps": speed})

    def test_line_starts_where_auto_is_engaged(self):
        lane, _, moved = self.nav.enter_auto_mode(*self.latlon(1.0, 0.8), now_mono=0.0)
        self.assertEqual((lane, round(moved, 6)), ("line", 0.0))
        self.assertAlmostEqual(self.line.line_offset_m, 0.8, places=6)
        info = self.step(20.0, 0.8)
        self.assertAlmostEqual(info["cte_right_m"], 0.0, places=6)
        self.assertEqual(info["prediction_status"], "line")
        self.assertEqual(info["throttle_blend"], 1.0)
        self.assertAlmostEqual(info["target_bearing_deg"], self.line.bearing_deg, places=6)
        self.assertIn("entry +0.80 m", self.nav.describe_line())

    def test_near_bank_entry_merges_toward_centerline_at_capped_angle(self):
        _, _, moved = self.nav.enter_auto_mode(*self.latlon(1.0, -2.3), now_mono=0.0)
        self.assertAlmostEqual(moved, 0.8, places=6)
        self.assertAlmostEqual(self.line.line_offset_m, -1.5, places=6)
        self.assertIn("moved +0.80 m toward the centerline", self.nav.describe_line())
        info = self.step(3.0, -2.3)
        self.assertEqual(info["prediction_status"], "merging")
        # Left of the line: steer right, but no more than 15 deg from the course while still moving forward.
        self.assertAlmostEqual(wrap(info["target_bearing_deg"] - self.line.bearing_deg), 15.0, places=6)

    def test_bank_guard_and_hard_limit_turn_back_toward_centerline(self):
        self.nav.enter_auto_mode(*self.latlon(1.0, 1.5), now_mono=0.0)
        guard = self.step(10.0, 2.1, heading_offset_deg=5.0)      # In the soft zone, moving outward slowly
        self.assertEqual(guard["prediction_status"], "bank_guard")
        self.assertTrue(guard["boundary_guard"])
        hard = self.step(12.0, 2.2, heading_offset_deg=30.0, t=1.1)   # 1 s forecast leaves the band
        self.assertEqual(hard["prediction_status"], "hard_limit")
        self.assertAlmostEqual(wrap(hard["target_bearing_deg"] - self.line.bearing_deg), -40.0, places=6)
        self.assertLess(hard["steer_norm"], 0.0)                     # Steering left, away from the right bank
        self.assertLess(hard["prediction_corridor_clearance_m"], 0.0)

    def test_moving_back_from_the_soft_zone_is_not_guarded(self):
        self.nav.enter_auto_mode(*self.latlon(1.0, 1.5), now_mono=0.0)
        info = self.step(10.0, 2.1, heading_offset_deg=-10.0)
        self.assertFalse(info["boundary_guard"])

    def test_finishes_after_margin_past_the_finish_line(self):
        self.nav.enter_auto_mode(*self.latlon(80.0, 0.0), now_mono=0.0)
        self.assertFalse(self.step(self.line.length + 0.9, 0.0)["finished"])
        self.assertTrue(self.step(self.line.length + 1.05, 0.0)["finished"])

    def test_lane_strategies_are_unchanged(self):
        paths = self.nav.paths
        nav = mb.MultipathNavigator(paths, mb.NAV_PARAMS, strategy=create_auto_strategy(
            {"strategy": "04_predictive_recovery"}, mb.NAV_PARAMS))
        self.assertIsNone(nav.line_guidance)
        self.assertIn(nav.enter_auto_mode(paths["center"][3][1], paths["center"][3][2], now_mono=0.0)[0],
                      ("left", "center", "right"))


def wrap(angle):
    return (angle + 180.0) % 360.0 - 180.0


class StraightLineAutoLoopTest(unittest.TestCase):
    """The real main loop with mocked hardware (harness from test_course_auto_integration)."""

    run_controller = harness.CourseAutoIntegrationTests.run_controller

    def test_auto_loop_follows_the_line_at_full_throttle_and_logs_it(self):
        rows, pca, cal = self.run_controller(strategy=NAME)
        auto = [r for r in rows if r["mode"] == "AUTO"]
        self.assertTrue(auto)
        self.assertTrue(all(r["auto_strategy"] == NAME for r in auto))
        self.assertTrue(all(r["lane"] == "line" for r in auto[1:]))    # The entry row has no navigation update
        self.assertTrue(all(int(r["esc_us"]) == cal["pwm"]["esc_fwd_max_us"] for r in auto[1:]))
        self.assertEqual(auto[-1]["prediction_status"], "line")
        self.assertEqual(auto[-1]["control_heading_source"], "gps_course")
        self.assertAlmostEqual(float(auto[-1]["cte_right_m"]), 0.0, places=3)

    def test_auto_loop_merges_back_after_a_sideways_jump(self):
        rows, _, _ = self.run_controller(disturbance="recovery", strategy=NAME)
        row = rows[-1]
        # The harness moves the boat 0.9 m right of where AUTO drew the line, heading 340 deg at 1.5 m/s.
        self.assertEqual(row["prediction_status"], "merging")
        self.assertAlmostEqual(float(row["cte_right_m"]), 0.9, places=2)
        self.assertAlmostEqual(float(row["target_bearing_deg"]), 345.0, places=1)
        self.assertAlmostEqual(float(row["lookahead_m"]), 2.7)


if __name__ == "__main__":
    unittest.main()
