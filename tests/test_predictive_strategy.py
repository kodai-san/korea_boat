"""Offline checks for bounded recovery prediction and speed-dependent guidance."""

import unittest

from auto_strategies import create_auto_strategy, default_autonomous_config


NAV_PARAMS = {
    "max_steer_angle_deg": 35., "boundary_gain_boost": 1.45,
    "servo_slew_per_cycle": .18, "shift_cte_m": .7,
    "lookahead_dist_m": 3.5, "boundary_lookahead_m": 2.4,
}


class PredictiveStrategyTests(unittest.TestCase):
    def create(self, **options):
        return create_auto_strategy({"strategy": "predictive_recovery",
                                     "predictive_recovery": options}, NAV_PARAMS)

    @staticmethod
    def prediction(**overrides):
        context = {"speed_mps": 1., "course_deg": 0., "path_bearing_deg": 0.,
                   "age_sec": 0., "corridor_clear": True, "path_turn_deg": 0.}
        context.update(overrides)
        return context

    def update(self, strategy, now=100., cte=0., heading=0., confident=True,
               guard=False, prediction="default"):
        if prediction == "default":
            prediction = self.prediction()
        return strategy.update(cte_right_m=cte, heading_error_deg=heading,
                               heading_confident=confident, boundary_guard=guard,
                               now_mono=now, prediction=prediction)

    def settle(self, strategy, **kwargs):
        for step in range(61):
            result = self.update(strategy, 100 + step * .05, **kwargs)
        return result

    def test_new_strategy_is_opt_in_and_owns_independent_options(self):
        defaults = default_autonomous_config()
        self.assertEqual(defaults["strategy"], "02_adaptive_throttle")
        self.assertTrue(self.create().uses_predictive_guidance)
        self.assertFalse(create_auto_strategy({}, NAV_PARAMS).uses_predictive_guidance)
        defaults["04_predictive_recovery"]["lookahead_min_m"] = 10
        self.assertEqual(self.create().options["lookahead_min_m"], 2.4)

    def test_mirrored_convergence_gets_more_power_than_current_error_alone(self):
        baseline = create_auto_strategy({"strategy": "gps_course_progressive"}, NAV_PARAMS)
        for step in range(61):
            old = baseline.update(cte_right_m=.9, heading_error_deg=0,
                                  boundary_guard=False, heading_confident=True,
                                  now_mono=100 + step * .05)
        for cte, course, predicted, rate in ((.9, 330., .4, -.5), (-.9, 30., -.4, .5)):
            with self.subTest(cte=cte):
                result = self.settle(self.create(), cte=cte,
                                     prediction=self.prediction(course_deg=course))
                self.assertEqual(result["prediction_status"], "recovering")
                self.assertAlmostEqual(result["predicted_cte_m"], predicted)
                self.assertAlmostEqual(result["cte_rate_mps"], rate)
                self.assertAlmostEqual(result["throttle_blend"], 11 / 12)
                self.assertGreater(result["throttle_blend"], old["throttle_blend"])

    def test_outward_motion_reduces_power_even_when_current_error_is_small(self):
        for cte, course in ((.1, 30.), (-.1, 330.), (0., 30.)):
            with self.subTest(cte=cte, course=course):
                result = self.settle(self.create(), cte=cte,
                                     prediction=self.prediction(course_deg=course))
                self.assertEqual(result["prediction_status"], "diverging")
                self.assertLess(result["throttle_blend"], 1)

    def test_predicting_overshoot_withdraws_recovery_credit(self):
        result = self.settle(self.create(), cte=.4,
                             prediction=self.prediction(course_deg=300.))
        self.assertEqual(result["prediction_status"], "overshoot")
        self.assertLess(result["predicted_cte_m"], -.3)
        self.assertLess(result["throttle_blend"], (1.5 - .4) / 1.2)

    def test_small_centerline_crossing_inside_tolerance_keeps_recovery_credit(self):
        result = self.settle(self.create(), cte=.4,
                             prediction=self.prediction(course_deg=330.))
        self.assertEqual(result["prediction_status"], "recovering")
        self.assertAlmostEqual(result["predicted_cte_m"], -.1)
        self.assertEqual(result["throttle_blend"], 1)

    def test_bend_or_weak_convergence_cannot_get_extra_credit(self):
        for context, status in ((self.prediction(course_deg=330., path_turn_deg=20.), "bend"),
                                (self.prediction(course_deg=359.), "tracking")):
            with self.subTest(context=context):
                result = self.settle(self.create(), cte=.9, prediction=context)
                self.assertEqual(result["prediction_status"], status)
                self.assertAlmostEqual(result["throttle_blend"], .5)

    def test_current_tracking_envelope_and_corridor_are_hard_power_limits(self):
        cases = [(1.5, self.prediction(course_deg=270.), "outside_tracking_envelope"),
                 (-1.5, self.prediction(course_deg=90.), "outside_tracking_envelope"),
                 (0., self.prediction(corridor_clear=False), "outside_corridor"),
                 (0., self.prediction(corridor_clear=1), "outside_corridor")]
        for cte, context, status in cases:
            with self.subTest(cte=cte, context=context):
                result = self.settle(self.create(), cte=cte, prediction=context)
                self.assertEqual(result["prediction_status"], status)
                self.assertEqual(result["throttle_blend"], 0)

    def test_missing_invalid_or_stale_context_cannot_boost(self):
        contexts = [None, {}, [], self.prediction(age_sec=.81), self.prediction(age_sec=-.1),
                    self.prediction(speed_mps=.44), self.prediction(course_deg=360.),
                    self.prediction(path_bearing_deg=-1.), self.prediction(path_turn_deg=181.)]
        for key in ("speed_mps", "course_deg", "path_bearing_deg", "age_sec", "path_turn_deg"):
            contexts.extend(self.prediction(**{key: value})
                            for value in (None, True, "1", float("nan"), float("inf"), 10 ** 400))
        for context in contexts:
            with self.subTest(context=context):
                result = self.settle(self.create(), prediction=context)
                self.assertEqual(result["throttle_blend"], 0)
                self.assertEqual(result["throttle_target_blend"], 0)

    def test_confidence_boundary_and_heading_limits_keep_their_priority(self):
        for kwargs in ({"confident": False}, {"guard": True}, {"heading": None},
                       {"heading": float("nan")}, {"heading": 35.}, {"cte": float("nan")}):
            with self.subTest(kwargs=kwargs):
                strategy = self.create()
                self.assertEqual(self.settle(strategy)["throttle_blend"], 1)
                self.assertEqual(self.update(strategy, 103.05, **kwargs)["throttle_blend"], 0)

    def test_power_ramps_up_reduces_immediately_and_resets_after_hold(self):
        strategy = self.create()
        first = self.update(strategy, cte=.9, prediction=self.prediction(course_deg=330.))
        self.assertEqual(first["throttle_blend"], 0)
        self.assertAlmostEqual(first["throttle_target_blend"], 11 / 12)
        self.assertAlmostEqual(self.update(strategy, 100.05)["throttle_blend"], .025)
        self.assertEqual(self.settle(strategy)["throttle_blend"], 1)
        self.assertEqual(self.update(strategy, 103.05, prediction=None)["throttle_blend"], 0)
        strategy.suspend()
        acquired = self.update(strategy, 200, heading=None, confident=False)
        self.assertEqual(acquired["steer_norm"], 0)
        self.assertEqual(acquired["throttle_blend"], 0)

    def test_prediction_uses_course_relative_to_path_including_angle_wrap(self):
        result = self.settle(self.create(), cte=.9,
                             prediction=self.prediction(course_deg=350., path_bearing_deg=20.))
        self.assertAlmostEqual(result["cte_rate_mps"], -.5)
        self.assertAlmostEqual(result["predicted_cte_m"], .4)

    def test_lookahead_uses_speed_and_bounds_with_fixed_fallback(self):
        strategy = self.create()
        for speed, distance in ((0., 2.4), (.5, 2.4), (2., 3.6), (10., 5.)):
            self.assertAlmostEqual(strategy.lookahead_distance(speed), distance)
        for speed in (None, float("nan"), float("inf"), -1, True, "1", 10 ** 400):
            self.assertEqual(strategy.lookahead_distance(speed), 3.5)
        self.assertEqual(strategy.lookahead_distance(10., boundary_guard=True), 2.4)
        self.assertEqual(strategy.lookahead_distance(None, boundary_guard=True), 2.4)

    def test_configuration_rejects_bad_values_and_incompatible_bounds(self):
        added = set(self.create().default_options) - set(
            create_auto_strategy({"strategy": "gps_course_progressive"}, NAV_PARAMS).default_options)
        for key in added:
            for value in (-1, True, "1", None, float("nan"), float("inf"), 10 ** 400):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    self.create(**{key: value})
        for key in added - {"prediction_corridor_margin_m", "prediction_max_path_turn_deg"}:
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.create(**{key: 0})
        for options in ({"lookahead_min_m": 6}, {"prediction_corridor_margin_m": .7},
                        {"prediction_max_path_turn_deg": 181}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.create(**options)
        self.create(lookahead_min_m=3, lookahead_max_m=3,
                    prediction_corridor_margin_m=0, prediction_max_path_turn_deg=0)

    def test_inactive_prediction_block_still_checks_navigation_margin(self):
        for margin in (.7, .8):
            with self.subTest(margin=margin), self.assertRaises(ValueError):
                create_auto_strategy({"strategy": "constant_cruise", "predictive_recovery": {
                    "prediction_corridor_margin_m": margin}}, NAV_PARAMS)
        with self.assertRaises(ValueError):
            create_auto_strategy({"strategy": "predictive_recovery"},
                                 dict(NAV_PARAMS, shift_cte_m=.1))


if __name__ == "__main__":
    unittest.main()
