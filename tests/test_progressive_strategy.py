"""Offline checks for progressive power, configuration, and recovery state."""

import unittest

from auto_strategies import create_auto_strategy, default_autonomous_config


NAV_PARAMS = {
    "max_steer_angle_deg": 35.0,
    "boundary_gain_boost": 1.45,
    "servo_slew_per_cycle": 0.18,
}


class ProgressiveStrategyTests(unittest.TestCase):
    def create(self, **options):
        return create_auto_strategy({"strategy": "gps_course_progressive",
                                     "gps_course_progressive": options}, NAV_PARAMS)

    def update(self, strategy, now=100.0, cte=0.0, heading=0.0, confident=True, guard=False):
        return strategy.update(cte_right_m=cte, heading_error_deg=heading,
                               boundary_guard=guard, now_mono=now,
                               heading_confident=confident)

    def settle(self, strategy, **kwargs):
        for step in range(61):
            result = self.update(strategy, now=100.0 + step * .05, **kwargs)
        return result

    def test_new_mode_is_opt_in_and_has_independent_defaults(self):
        defaults = default_autonomous_config()
        self.assertEqual(defaults["strategy"], "02_adaptive_throttle")
        self.assertTrue(self.create().uses_gps_course)
        self.assertFalse(create_auto_strategy({}, NAV_PARAMS).uses_gps_course)
        defaults["03_gps_course_progressive"]["course_max_age_sec"] = 99
        self.assertEqual(self.create().options["course_max_age_sec"], .6)

    def test_both_tracking_errors_limit_power_symmetrically(self):
        for cte, heading, expected in ((0, 0, 1), (.3, 11, 1), (-.3, -11, 1),
                                       (.9, 0, .5), (0, 23, .5), (-.9, -23, .5),
                                       (1.5, 0, 0), (0, 35, 0), (5, 90, 0)):
            with self.subTest(cte=cte, heading=heading):
                result = self.settle(self.create(), cte=cte, heading=heading)
                self.assertAlmostEqual(result["throttle_blend"], expected)

    def test_power_increases_over_time_not_control_cycle_count(self):
        strategy = self.create()
        self.assertEqual(self.update(strategy)["throttle_blend"], 0)
        self.assertAlmostEqual(self.update(strategy, 100.05)["throttle_blend"], .025)
        self.assertAlmostEqual(self.update(strategy, 100.15)["throttle_blend"], .075)
        self.assertAlmostEqual(self.update(strategy, 100.15)["throttle_blend"], .075)

    def test_delay_or_backward_clock_cannot_skip_ramp(self):
        strategy = self.create()
        self.update(strategy, 100)
        self.assertAlmostEqual(self.update(strategy, 200)["throttle_blend"], .05)
        self.assertAlmostEqual(self.update(strategy, 199)["throttle_blend"], .05)

    def test_reductions_are_immediate_for_error_or_lost_confidence(self):
        strategy = self.create()
        self.assertEqual(self.settle(strategy)["throttle_blend"], 1)
        self.assertAlmostEqual(self.update(strategy, 103.05, cte=.9)["throttle_blend"], .5)
        self.assertEqual(self.update(strategy, 103.10, confident=False)["throttle_blend"], 0)
        self.assertAlmostEqual(self.update(strategy, 103.15)["throttle_blend"], .025)

    def test_boundary_missing_or_nonfinite_heading_and_cte_stay_at_cruise(self):
        for kwargs in ({"guard": True}, {"heading": None}, {"heading": float("nan")},
                       {"heading": float("inf")}, {"cte": float("nan")},
                       {"cte": float("inf")}, {"confident": False}):
            with self.subTest(kwargs=kwargs):
                strategy = self.create()
                self.settle(strategy)
                self.assertEqual(self.update(strategy, 103.05, **kwargs)["throttle_blend"], 0)

    def test_omitting_confidence_never_enables_boost(self):
        strategy = self.create()
        for step in range(60):
            result = strategy.update(cte_right_m=0, heading_error_deg=0,
                                     boundary_guard=False, now_mono=100 + step * .05)
        self.assertEqual(result["throttle_blend"], 0)

    def test_reset_and_suspend_restart_boost_and_centered_steering(self):
        strategy = self.create()
        self.settle(strategy, heading=10)
        steering = strategy.prev_steer_norm
        self.assertGreater(steering, 0)
        strategy.suspend()
        self.assertEqual(strategy.prev_steer_norm, 0)
        acquiring = self.update(strategy, 200, heading=None, confident=False)
        self.assertEqual(acquiring["throttle_blend"], 0)
        self.assertEqual(acquiring["steer_norm"], 0)
        strategy.reset()
        self.assertEqual(strategy.prev_steer_norm, 0)
        self.assertEqual(self.update(strategy, 300)["throttle_blend"], 0)

    def test_all_options_reject_invalid_numeric_values_even_when_inactive(self):
        for key in self.create().default_options:
            for value in (-1, True, "0.5", None, float("nan"), float("inf"), 10 ** 400):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    create_auto_strategy({"strategy": "constant_cruise",
                                          "gps_course_progressive": {key: value}}, NAV_PARAMS)

    def test_positive_options_and_ordered_thresholds_are_required(self):
        for key in ("throttle_rise_per_sec", "min_course_speed_mps", "course_max_age_sec",
                    "position_heading_max_age_sec", "forward_acquire_sec", "forward_memory_sec"):
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.create(**{key: 0})
        for options in ({"cruise_cte_m": .3}, {"cruise_cte_m": .2},
                        {"cruise_heading_deg": 11}, {"cruise_heading_deg": 10},
                        {"cruise_heading_deg": 181}, {"full_throttle_heading_deg": 181},
                        {"unknown": 1}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.create(**options)
        self.create(full_throttle_cte_m=0, full_throttle_heading_deg=0)


if __name__ == "__main__":
    unittest.main()
