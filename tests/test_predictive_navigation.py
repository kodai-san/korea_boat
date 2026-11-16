"""Navigation-level checks for predictive recovery and continuous lookahead."""

import math
import unittest
from unittest.mock import patch

from auto_strategies import create_auto_strategy
from gps_heading import MotionHeadingSelector
import multipath_boat as boat


class PredictiveNavigationTests(unittest.TestCase):
    @staticmethod
    def latlon(x, y):
        return math.degrees(y / boat.EARTH_RADIUS_M), math.degrees(x / boat.EARTH_RADIUS_M)

    @classmethod
    def paths(cls, points=None, spacing=2.0):
        points = points or [(0, 0), (0, 10), (0, 20), (0, 30)]
        return {lane: [(index + 1, *cls.latlon(x + offset, y))
                       for index, (x, y) in enumerate(points)]
                for lane, offset in (("left", -spacing), ("center", 0), ("right", spacing))}

    def navigator(self, name="predictive_recovery", points=None, spacing=2.0, x=0, y=4):
        strategy = create_auto_strategy({"strategy": name}, boat.NAV_PARAMS)
        navigator = boat.MultipathNavigator(self.paths(points, spacing), dict(boat.NAV_PARAMS), strategy)
        navigator.enter_auto_mode(*self.latlon(x, y), now_mono=100.)
        return navigator

    @staticmethod
    def motion(now=101., speed=2., course=0., **overrides):
        result = {"source": "gps_course", "speed_mps": speed, "heading_deg": course,
                  "age_sec": 0., "observed_at_mono": now, "confident": True}
        result.update(overrides)
        return result

    def update(self, navigator, x=0., y=4., course=0., speed=2., now=101., motion=None):
        return navigator.update(*self.latlon(x, y), course, now_mono=now, heading_confident=True,
                                motion=self.motion(now, speed, course) if motion is None else motion)

    def test_actual_navigation_interpolates_target_between_sparse_waypoints(self):
        nav = self.navigator(x=.6)
        result = self.update(nav, x=.6)
        self.assertAlmostEqual(result["lookahead_m"], 3.6)
        self.assertAlmostEqual(result["lookahead_target_e_m"], 0)
        self.assertAlmostEqual(result["lookahead_target_n_m"], 7.6)
        self.assertAlmostEqual(result["target_bearing_deg"], math.degrees(math.atan2(-.6, 3.6)) % 360)
        self.assertEqual(result["wp_number"], 2)  # Segment endpoint identifies the interpolated target.

    def test_arrival_radius_does_not_push_dense_path_projection_ahead(self):
        dense = self.navigator(points=[(0, index * .1) for index in range(301)], y=4)
        sparse = self.navigator(y=4)
        for y in (4., 4.05, 4.2, 4.35):
            with self.subTest(y=y):
                actual = self.update(dense, y=y)
                expected = self.update(sparse, y=y)
                self.assertAlmostEqual(actual["lookahead_target_n_m"], expected["lookahead_target_n_m"])

    def test_speed_bounds_and_outer_boundary_cap_change_real_target(self):
        for speed, lookahead in ((.5, 2.4), (2., 3.6), (4., 5.)):
            with self.subTest(speed=speed):
                result = self.update(self.navigator(), speed=speed)
                self.assertAlmostEqual(result["lookahead_m"], lookahead)
                self.assertAlmostEqual(result["lookahead_target_n_m"], 4 + lookahead)
        nav = self.navigator(x=2.8)
        result = self.update(nav, x=2.8, speed=4.)
        self.assertEqual(result["active_lane"], "right")
        self.assertTrue(result["boundary_guard"])
        self.assertAlmostEqual(result["lookahead_m"], boat.NAV_PARAMS["boundary_lookahead_m"])
        self.assertEqual(result["throttle_target_blend"], 0)

    def test_invalid_motion_uses_fixed_lookahead_and_cannot_enable_boost(self):
        cases = [{}, self.motion(speed=float("nan")), self.motion(speed=None),
                 self.motion(speed=.1), self.motion(age_sec=.61),
                 self.motion(observed_at_mono=99.), self.motion(observed_at_mono=102.),
                 self.motion(course=20.), self.motion(source="unknown"),
                 self.motion(heading_deg=float("nan"))]
        for motion in cases:
            with self.subTest(motion=motion):
                nav = self.navigator()
                good = self.update(nav, now=100.9)
                self.assertEqual(good["throttle_target_blend"], 1)
                result = self.update(nav, motion=motion)
                self.assertEqual(result["prediction_status"], "prediction_missing")
                self.assertEqual(result["throttle_target_blend"], 0)
                self.assertEqual(result["throttle_blend"], 0)
                self.assertEqual(result["lookahead_m"], boat.NAV_PARAMS["lookahead_dist_m"])
                self.assertIsNone(result["prediction_corridor_clearance_m"])

    def test_missing_motion_never_uses_unpaired_speed(self):
        nav = self.navigator()
        result = nav.update(*self.latlon(0, 4), 0, now_mono=101., heading_confident=True)
        self.assertEqual(result["lookahead_m"], boat.NAV_PARAMS["lookahead_dist_m"])
        self.assertEqual(result["throttle_target_blend"], 0)

    def test_each_motion_source_obeys_its_own_freshness_limit(self):
        for source, expected in (("gps_course", 3.5), ("position", 3.6)):
            with self.subTest(source=source):
                motion = self.motion(source=source, age_sec=.7, observed_at_mono=100.3)
                result = self.update(self.navigator(), motion=motion)
                self.assertAlmostEqual(result["lookahead_m"], expected)
                self.assertEqual(result["throttle_target_blend"], int(source == "position"))

    def test_forecast_outside_outer_routes_removes_boost_before_boundary_guard(self):
        nav = self.navigator(spacing=.4, x=.1)
        result = self.update(nav, x=.1, course=30., speed=2.)
        self.assertEqual(result["active_lane"], "center")
        self.assertFalse(result["boundary_guard"])
        self.assertLess(result["prediction_corridor_clearance_m"], 0)
        self.assertEqual(result["prediction_status"], "outside_corridor")
        self.assertEqual(result["throttle_target_blend"], 0)
        self.assertAlmostEqual(result["predicted_cte_m"], 1.1)

    def test_curve_ahead_disables_extra_recovery_credit(self):
        straight = self.navigator(x=.9, y=6.5)
        curve = self.navigator(points=[(0, 0), (0, 10), (-8, 10), (-8, 30)], x=.9, y=6.5)
        recovering = self.update(straight, x=.9, y=6.5, course=350., speed=2.)
        bending = self.update(curve, x=.9, y=6.5, course=350., speed=2.)
        self.assertEqual(recovering["prediction_status"], "recovering")
        self.assertEqual(bending["prediction_status"], "bend")
        self.assertAlmostEqual(bending["throttle_target_blend"], .5)
        self.assertGreater(recovering["throttle_target_blend"], bending["throttle_target_blend"])
        self.assertAlmostEqual(bending["lookahead_target_e_m"], -.1)
        self.assertAlmostEqual(bending["lookahead_target_n_m"], 10.)

    def test_lane_change_recomputes_target_and_prediction_in_new_lane(self):
        nav = self.navigator(x=.8, y=5.)
        first = self.update(nav, x=.8, y=5., now=103.1, speed=1.5)
        self.assertIsNone(first["shifted_to"])
        shifted = self.update(nav, x=.8, y=5., now=104.4, speed=1.5)
        self.assertEqual(shifted["shifted_to"], "right")
        self.assertEqual(shifted["active_lane"], "right")
        self.assertAlmostEqual(shifted["cte_right_m"], -1.2)
        self.assertAlmostEqual(shifted["predicted_cte_m"], -1.2)
        self.assertAlmostEqual(shifted["lookahead_target_e_m"], 2.)
        self.assertAlmostEqual(shifted["lookahead_target_n_m"], 7.7)
        self.assertAlmostEqual(shifted["target_bearing_deg"], math.degrees(math.atan2(1.2, 2.7)))

    def test_finish_remains_neutral_without_strategy_update(self):
        nav = self.navigator()
        with patch.object(nav.strategy, "update", wraps=nav.strategy.update) as update:
            finished = self.update(nav, y=30.)
        self.assertTrue(finished["finished"])
        self.assertEqual(finished["steer_norm"], 0)
        self.assertEqual(finished["throttle_blend"], 0)
        update.assert_not_called()

    def test_existing_modes_keep_discrete_waypoint_guidance(self):
        for name in ("constant_cruise", "adaptive_throttle", "gps_course_progressive"):
            with self.subTest(strategy=name):
                result = self.update(self.navigator(name, x=.6), x=.6, speed=4.)
                self.assertAlmostEqual(result["target_bearing_deg"], math.degrees(math.atan2(-.6, 6.)) % 360)
                self.assertEqual(result["wp_number"], 2)
                self.assertNotIn("lookahead_m", result)
                self.assertNotIn("prediction_status", result)

    def test_heading_selector_keeps_speed_and_timestamp_paired_with_selected_source(self):
        nav = self.navigator()
        selector = MotionHeadingSelector(nav.strategy.options)
        snap = {"valid": True, "age_sec": 0., "fix_quality": 4,
                "course_valid": True, "course_deg": 10., "course_speed_mps": 2.5,
                "course_age_sec": 0., "course_time_mono": 101., "course_gga_skew_sec": 0.,
                "position_heading_deg": 90., "position_speed_mps": .8,
                "position_heading_age_sec": .1, "position_heading_time_mono": 100.9}
        selector.enter_auto(snap, 100.)
        course = selector.select(snap, 101.)
        self.assertEqual((course["source"], course["heading_deg"], course["speed_mps"], course["observed_at_mono"]),
                         ("gps_course", 10., 2.5, 101.))
        snap.update(course_valid=False, course_speed_mps=None, course_age_sec=None)
        position = selector.select(snap, 101.)
        self.assertEqual((position["source"], position["heading_deg"], position["speed_mps"], position["observed_at_mono"]),
                         ("position", 90., .8, 100.9))


if __name__ == "__main__":
    unittest.main()
