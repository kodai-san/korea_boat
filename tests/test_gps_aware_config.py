"""Mode 06 configuration and its neutral-by-default steering contract."""

import json
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from auto_strategies import (  # noqa: E402
    AUTO_STRATEGIES, GpsAwareStrategy, StraightLineStrategy,
    apply_run_overrides, canonical_auto_strategy_name, create_auto_strategy,
    default_autonomous_config,
)


NAV = {"max_steer_angle_deg": 35.0, "boundary_gain_boost": 1.45,
       "servo_slew_per_cycle": 0.18, "lookahead_dist_m": 3.5}
NAME = GpsAwareStrategy.name


def make(**options):
    return create_auto_strategy({"strategy": NAME, NAME: options}, NAV)


class GpsAwareConfigTest(unittest.TestCase):
    def test_registered_with_stable_number_and_guard_capability(self):
        self.assertIs(AUTO_STRATEGIES[NAME], GpsAwareStrategy)
        self.assertEqual(canonical_auto_strategy_name("gps_aware"), NAME)
        strategy = make()
        self.assertTrue(strategy.uses_gps_guard)
        self.assertTrue(strategy.uses_gps_course)
        self.assertTrue(strategy.uses_straight_line)

    def test_calibration_has_full_options_without_switching_selected_strategy(self):
        cfg = json.loads((ROOT / "calibration.json").read_text())["autonomous"]
        self.assertEqual(cfg["strategy"], "05_straight_line")
        self.assertEqual(cfg[NAME], GpsAwareStrategy.default_options)
        self.assertEqual(create_auto_strategy(cfg, NAV).name, "05_straight_line")
        selected = apply_run_overrides(cfg, strategy=NAME)
        self.assertEqual(create_auto_strategy(selected, NAV).name, NAME)
        self.assertEqual(cfg["strategy"], "05_straight_line")

    def test_default_option_blocks_are_independent(self):
        cfg = default_autonomous_config()
        cfg[NAME]["race_throttle"] = 0.8
        self.assertEqual(default_autonomous_config()[NAME]["race_throttle"], 1.0)
        self.assertNotIn("race_throttle", StraightLineStrategy.default_options)

    def test_all_numeric_guard_options_reject_nonfinite_and_wrong_types(self):
        for key, value in GpsAwareStrategy.guard_default_options.items():
            if isinstance(value, bool):
                continue
            for bad in (True, False, "1", None, math.inf, -math.inf, math.nan):
                with self.subTest(key=key, bad=bad), self.assertRaises(ValueError):
                    make(**{key: bad})

    def test_booleans_and_counts_require_exact_types(self):
        for options in ({"auto_resume": 1}, {"auto_resume": "false"},
                        {"retain_stale_accuracy": 1}, {"retain_stale_accuracy": "true"},
                        {"test_square": 0}, {"qualification_fixes": 5.0},
                        {"qualification_fixes": 1}, {"min_satellites": 8.0},
                        {"min_satellites": 3}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                make(**options)
        self.assertFalse(make(auto_resume=False).options["auto_resume"])

    def test_bad_ranges_and_relationships_are_rejected(self):
        for options in (
                {"unknown": 1}, {"race_throttle": 1.01}, {"launch_throttle": -0.1},
                {"float_throttle": 1.01}, {"gps_throttle": 0.6},
                {"dgps_throttle": 0.8}, {"boundary_throttle": 1.1},
                {"launch_throttle": 0.8}, {"float_uncertainty_m": 0.1},
                {"dgps_uncertainty_m": 0.5}, {"gps_uncertainty_m": 1.0},
                {"accuracy_max_skew_sec": 1.1}, {"max_source_gap_sec": 0.5},
                {"race_clearance_m": 0.1}, {"launch_min_clearance_m": 0.1},
                {"degraded_launch_min_clearance_m": 0.1},
                {"launch_timeout_sec": 0.5}, {"max_speed_mps": 0.45},
                {"race_clearance_m": 2.0}, {"launch_min_clearance_m": 2.0},
                {"race_max_cte_m": 2.5}, {"race_max_heading_error_deg": 181.0},
                {"position_max_age_sec": 0.0}, {"qualification_sec": 0.0},
                {"throttle_rise_per_sec": 0.0}, {"jump_tolerance_m": 0.0}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                make(**options)

    def test_inactive_mode_is_still_validated(self):
        with self.assertRaises(ValueError):
            create_auto_strategy({"strategy": "05_straight_line", NAME: {"qualification_fixes": 1}}, NAV)

    def test_cli_geometry_targets_mode06_and_rejects_test_square(self):
        cfg = {"strategy": "05_straight_line", "05_straight_line": {"line_max_offset_m": 1.5}}
        new = apply_run_overrides(cfg, strategy=NAME, line_limit_m=0.8, bank_limit_m=3.0)
        self.assertEqual(new[NAME], {"line_max_offset_m": 0.8, "band_half_width_m": 3.0})
        self.assertEqual(new["05_straight_line"], cfg["05_straight_line"])
        self.assertEqual(create_auto_strategy(new, NAV).options["line_max_offset_m"], 0.8)
        square = apply_run_overrides(cfg, strategy=NAME, test_square=True)
        with self.assertRaisesRegex(ValueError, "test_square"):
            create_auto_strategy(square, NAV)

    def test_legacy_cli_geometry_still_targets_mode05(self):
        cfg = apply_run_overrides({"strategy": "04_predictive_recovery"}, line_limit_m=1.0)
        self.assertEqual(cfg["05_straight_line"]["line_max_offset_m"], 1.0)
        self.assertNotIn(NAME, cfg)

    def test_guardless_strategy_never_authorizes_throttle(self):
        strategy = make()
        baseline = StraightLineStrategy(NAV)
        for i in range(120):
            args = dict(cte_right_m=0.1, heading_error_deg=3.0, boundary_guard=False,
                        now_mono=i * 0.05, heading_confident=True, speed_mps=1.5)
            guarded = strategy.update(**args)
            original = baseline.update(**args)
            self.assertEqual(guarded["throttle_norm"], 0.0)
            self.assertEqual(guarded["throttle_blend"], 0.0)
            self.assertAlmostEqual(guarded["steer_norm"], original["steer_norm"])
            self.assertAlmostEqual(guarded["trim_deg"], original["trim_deg"])
        description = strategy.describe_throttle(1990, 1990)
        self.assertIn("FIX qualification", description)
        self.assertIn("stuck reverse disabled", description)

    def test_unqualified_heading_cannot_train_trim(self):
        strategy = make()
        for i in range(120):
            strategy.update(cte_right_m=0.1, heading_error_deg=3.0, boundary_guard=False,
                            now_mono=i * 0.05, heading_confident=False, speed_mps=1.5)
        self.assertEqual(strategy.integral_deg_s, 0.0)


if __name__ == "__main__":
    unittest.main()
