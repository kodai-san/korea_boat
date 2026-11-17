"""Guarded 08 planning uses inset banks and explicit, unpowered requests."""

import math
import unittest

import multipath_boat as mb
from auto_strategies import create_auto_strategy
from path_guidance import StraightLineGuidance, turn_arc


NAME = "08_gps_aware_turn_planner"


class GuardedPlannerNavigationTests(unittest.TestCase):
    def setUp(self):
        paths = {lane: [(1, 35.0, 129.0), (2, 35.002, 129.0)]
                 for lane in ("left", "center", "right")}
        strategy = create_auto_strategy({"strategy": NAME}, mb.NAV_PARAMS)
        self.nav = mb.MultipathNavigator(paths, mb.NAV_PARAMS, strategy)
        self.line = self.nav.line_guidance
        self.nav.guard_turn_margin_m = .75
        self.nav.enter_auto_mode(*self.latlon(20., 0.), now_mono=0.)

    def latlon(self, s, d):
        x, y = self.line.point(s, d)
        return (self.nav.ref_lat + math.degrees(y / mb.EARTH_RADIUS_M),
                self.nav.ref_lon + math.degrees(x / (mb.EARTH_RADIUS_M * math.cos(math.radians(self.nav.ref_lat)))))

    def step(self, d, phi, t=1., speed=1.):
        heading = None if phi is None else (self.line.bearing_deg + phi) % 360.
        return self.nav.update(*self.latlon(20., d), heading, now_mono=t,
                               heading_confident=True, motion={"speed_mps": speed})

    def test_legacy_symmetric_guidance_constructor_still_works(self):
        line = StraightLineGuidance((0., 0.), (0., 100.), 2.5, 1.5)
        self.assertEqual((line.bank_left_m, line.bank_right_m), (2.5, 2.5))
        self.assertEqual((line.line_left_m, line.line_right_m), (1.5, 1.5))

    def test_no_current_guard_allowance_means_neutral_hold(self):
        self.nav.guard_turn_margin_m = None
        out = self.step(0., 106.)
        self.assertEqual(out["planner_hold_reason"], "planner_clearance_unavailable")
        self.assertEqual(out["planner_throttle_norm"], 0.)
        self.assertEqual(out["throttle_norm"], 0.)

    def test_safe_arc_and_tracking_request_forward_but_cannot_power_alone(self):
        out = self.step(.03, 106., speed=1.65)
        self.assertEqual(out["planner_state"], "turn_right")
        self.assertEqual(out["planner_throttle_norm"], 1.)
        self.assertEqual(out["throttle_norm"], 0.)
        self.nav.cancel_turn_plan()
        out = self.step(0., 0., t=2.)
        self.assertEqual(out["planner_state"], "tracking")
        self.assertEqual(out["planner_throttle_norm"], 1.)
        self.assertEqual(out["throttle_norm"], 0.)

    def test_no_fitting_arc_holds_instead_of_least_bad_turn(self):
        out = self.step(.5, 180.)
        self.assertEqual(out["planner_state"], "hold")
        self.assertEqual(out["planner_hold_reason"], "planner_no_safe_turn")
        self.assertEqual((out["throttle_norm"], out["steer_norm"]), (0., 0.))
        self.assertEqual(self.nav.turn_dir, 0)

    def test_more_uncertainty_cancels_an_unsafe_remaining_arc(self):
        self.assertEqual(self.step(.03, 106., speed=1.65)["planner_state"], "turn_right")
        self.nav.guard_turn_margin_m = 1.5
        out = self.step(.03, 106., t=1.1, speed=1.65)
        self.assertEqual(out["planner_hold_reason"], "planner_turn_clearance_lost")
        self.assertEqual(self.nav.turn_dir, 0)

    def test_line_clamped_to_current_uncertainty_inset(self):
        self.line.set_line(3.4)
        self.nav.guard_turn_margin_m = 1.5
        self.step(2., 0.)
        self.assertLessEqual(self.line.line_offset_m, self.line.bank_right_m - 1.5)
        self.line.set_line(-2.6)
        self.step(-1., 0., t=2.)
        self.assertGreaterEqual(self.line.line_offset_m, -self.line.bank_left_m + 1.5)

    def test_reverse_neutral_and_reacquisition_requests_remain_distinct(self):
        out = self.step(1.6, 90.)
        self.assertEqual((out["planner_state"], out["planner_throttle_norm"]), ("plan_neutral_in", 0.))
        out = self.step(1.6, None, t=1.4, speed=.5)
        self.assertEqual((out["planner_state"], out["planner_throttle_norm"]), ("plan_reverse", -.5))
        self.assertEqual(out["throttle_norm"], 0.)
        out = self.step(.7, None, t=2.4, speed=.5)
        self.assertEqual((out["planner_state"], out["planner_throttle_norm"]), ("plan_neutral_out", 0.))
        out = self.step(.7, None, t=2.8, speed=.1)
        self.assertIn(out["planner_state"], ("turn_left", "turn_right"))
        self.assertEqual(out["planner_throttle_norm"], 1.)
        self.assertTrue(out["reacquire_heading"])

    def test_no_overlap_between_line_limits_and_inset_holds(self):
        self.line.line_right_m = 0.
        self.nav.guard_turn_margin_m = 3.3
        out = self.step(.3, 0.)
        self.assertEqual(out["planner_hold_reason"], "planner_no_safe_line")
        self.assertEqual(out["planner_throttle_norm"], 0.)

    def test_reverse_timeout_does_not_fall_back_to_unsafe_turn(self):
        self.step(1.6, 90.)
        self.step(1.6, None, t=1.4, speed=.5)
        out = self.step(1.6, None, t=5.5, speed=0.)
        self.assertEqual(out["planner_state"], "plan_neutral_out")
        out = self.step(1.6, None, t=5.9, speed=0.)
        self.assertEqual(out["planner_hold_reason"], "planner_no_safe_turn_after_reverse")
        self.assertEqual(out["planner_throttle_norm"], 0.)

    def test_cancel_preserves_line_and_attempt_limit(self):
        self.line.set_line(1.1)
        self.nav.rev_phase, self.nav.turn_dir, self.nav.reverses = "plan_reverse", 1, 2
        self.nav.cancel_turn_plan()
        self.assertIsNone(self.nav.rev_phase)
        self.assertEqual(self.nav.turn_dir, 0)
        self.assertAlmostEqual(self.line.line_offset_m, 1.1)
        self.assertEqual(self.nav.reverses, 2)

    def test_arc_extrema_are_exact_between_samples(self):
        arc = turn_arc(0., 91.3, 2.3, 1)
        self.assertAlmostEqual(arc["max_d"], 2.3 * (math.cos(math.radians(91.3)) + 1.))


if __name__ == "__main__":
    unittest.main()
