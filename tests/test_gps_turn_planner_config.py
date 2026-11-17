"""Mode 08 combines guarded power with 07's asymmetric planning configuration."""

import json
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from auto_strategies import (  # noqa: E402
    AUTO_STRATEGIES, GpsAwareStrategy, GpsAwareTurnPlannerStrategy,
    TurnPlannerStrategy, apply_run_overrides, canonical_auto_strategy_name,
    create_auto_strategy, default_autonomous_config,
)

NAV = {"max_steer_angle_deg": 35.0, "boundary_gain_boost": 1.45,
       "servo_slew_per_cycle": 0.18, "lookahead_dist_m": 3.5}
NAME = GpsAwareTurnPlannerStrategy.name


def make(**options):
    return create_auto_strategy({"strategy": NAME, NAME: options}, NAV)


class GpsTurnPlannerConfigTest(unittest.TestCase):
    def test_modes_keep_stable_ids_and_distinct_capabilities(self):
        self.assertEqual(NAME, "08_gps_aware_turn_planner")
        self.assertIs(AUTO_STRATEGIES[NAME], GpsAwareTurnPlannerStrategy)
        self.assertEqual(canonical_auto_strategy_name("gps_aware_turn_planner"), NAME)
        mode08 = make()
        self.assertTrue(mode08.uses_gps_guard)
        self.assertTrue(mode08.uses_gps_course)
        self.assertTrue(mode08.side_limit_labels)
        self.assertTrue(mode08.options["turn_planner"])
        self.assertNotIn("band_half_width_m", mode08.options)
        self.assertNotIn("line_max_offset_m", mode08.options)
        self.assertNotIn("turn_planner", GpsAwareStrategy.default_options)
        self.assertNotIn("race_throttle", TurnPlannerStrategy.default_options)
        self.assertFalse(getattr(TurnPlannerStrategy, "uses_gps_guard", False))

    def test_calibration_preserves_selection_and_adds_complete08(self):
        cfg = json.loads((ROOT / "calibration.json").read_text())["autonomous"]
        self.assertEqual(create_auto_strategy(cfg, NAV).name, "05_straight_line")
        for cls in (GpsAwareStrategy, TurnPlannerStrategy, GpsAwareTurnPlannerStrategy):
            self.assertEqual(cfg[cls.name], cls.default_options)
            selected = apply_run_overrides(cfg, strategy=cls.name)
            self.assertEqual(create_auto_strategy(selected, NAV).name, cls.name)
        self.assertEqual(cfg["strategy"], "05_straight_line")

    def test_configuration_defaults_do_not_share_mutable_blocks(self):
        cfg = default_autonomous_config()
        cfg[NAME]["bank_left_m"] = 10.0
        cfg[NAME]["race_throttle"] = 0.9
        self.assertEqual(make().options["bank_left_m"], 3.2)
        self.assertEqual(cfg[TurnPlannerStrategy.name]["bank_left_m"], 3.2)
        self.assertEqual(cfg[GpsAwareStrategy.name]["race_throttle"], 1.0)

    def test_cli_targets_selected_mode_and_per_side_overrides_symmetric_shortcuts(self):
        original = default_autonomous_config()
        updated = apply_run_overrides(original, strategy="gps_aware_turn_planner",
                                      line_limit_m=1.5, line_left_m=1.0,
                                      bank_limit_m=4.0, bank_right_m=5.0,
                                      turn_planner=True, three_point_turns=True)
        strategy = create_auto_strategy(updated, NAV)
        self.assertEqual(strategy.limits(), (4.0, 5.0, 1.0, 1.5))
        self.assertTrue(strategy.options["three_point_turns"])
        for cls in (GpsAwareStrategy, TurnPlannerStrategy):
            self.assertEqual(updated[cls.name], original[cls.name])
        self.assertEqual(original[NAME], GpsAwareTurnPlannerStrategy.default_options)
        for name in ("05_straight_line", GpsAwareStrategy.name):
            with self.subTest(name=name), self.assertRaises(ValueError):
                apply_run_overrides(original, strategy=name, three_point_turns=True)

    def test_guard_validates_each_bank_and_asymmetric_line_limit(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                good = {f"bank_{side}_m": 1.6, f"line_{side}_m": 0.5}
                self.assertEqual(make(**good).options[f"bank_{side}_m"], 1.6)
                with self.assertRaisesRegex(ValueError, "both bank limits"):
                    make(**dict(good, **{f"bank_{side}_m": 1.5}))
                with self.assertRaisesRegex(ValueError, "race_max_cte_m"):
                    make(**dict(good, race_max_cte_m=1.6))
                with self.assertRaisesRegex(ValueError, "guard_zone_m"):
                    make(**{f"line_{side}_m": GpsAwareTurnPlannerStrategy.default_options[f"bank_{side}_m"]})

    def test_guard_and_planner_invalid_values_are_both_rejected(self):
        for key, value in GpsAwareStrategy.guard_default_options.items():
            if isinstance(value, bool):
                continue
            for bad in (True, "1", None, math.inf, math.nan):
                with self.subTest(key=key, bad=bad), self.assertRaises(ValueError):
                    make(**{key: bad})
        invalid = ({"qualification_fixes": 5.0}, {"auto_resume": 1},
                   {"gps_throttle": 0.6}, {"test_square": True},
                   {"turn_planner": 1}, {"three_point_turns": "true"},
                   {"turn_exit_deg": 60.0}, {"plan_max_reverses": 1.5},
                   {"plan_reverse_max_sec": 0}, {"plan_reverse_throttle": 1.1})
        for options in invalid:
            with self.subTest(options=options), self.assertRaises(ValueError):
                make(**options)
        with self.assertRaises(ValueError):
            create_auto_strategy({"strategy": "05_straight_line", NAME: {"test_square": True}}, NAV)

    def test_standalone_strategy_cannot_authorize_power_or_train_unqualified_trim(self):
        guarded = make()
        original = TurnPlannerStrategy(NAV)
        for i in range(120):
            kwargs = dict(cte_right_m=0.1, heading_error_deg=3.0, boundary_guard=False,
                          now_mono=i * 0.05, heading_confident=False, speed_mps=1.5)
            command = guarded.update(**kwargs)
            baseline = original.update(**kwargs)
            self.assertEqual(command["throttle_norm"], 0.0)
            self.assertEqual(command["throttle_blend"], 0.0)
            self.assertEqual(command["steer_norm"], baseline["steer_norm"])
            self.assertEqual(command["trim_deg"], 0.0)
        self.assertIn("guarded turn planner", guarded.describe_throttle(1990, 1990))


if __name__ == "__main__":
    unittest.main()
