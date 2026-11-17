"""Exercise 08 through the actual guard and navigator with timed GPS observations."""

import math
import unittest
from unittest.mock import patch

import multipath_boat as mb
from auto_strategies import create_auto_strategy
from gps_guard import GpsAwareController
from test_gps_guard import observation


class GuardedTurnTests(unittest.TestCase):
    def make_guard(self, **options):
        paths = {lane: [(1, 35., 129.), (2, 35.001, 129.)]
                 for lane in ("left", "center", "right")}
        name = "08_gps_aware_turn_planner"
        strategy = create_auto_strategy({"strategy": name, name: options}, mb.NAV_PARAMS)
        nav = mb.MultipathNavigator(paths, mb.NAV_PARAMS, strategy)
        return nav, GpsAwareController(nav)

    def qualify(self, guard, heading=0., x=0., speed=1.):
        # Short qualification here keeps this deterministic fixture within the banks.
        guard.options["qualification_sec"] = .2
        guard.options["qualification_fixes"] = 3
        for t in (-1., -.2):
            guard.observe_manual(observation(t, x=x, y=10., course_deg=heading,
                                             course_speed_mps=speed), .5, 100. + t)
        guard.enter_auto(observation(0., x=x, y=10., course_deg=heading,
                                     course_speed_mps=speed), 100.)
        for t in (.05, .1, .15, .2, .25):
            out = guard.update(observation(t, x=x, y=10., course_deg=heading,
                                           course_speed_mps=speed), 100. + t)
            if out["gps_guard_reason"].startswith("planner_"):
                break
        return out

    def test_normal_tracking_gets_guard_power_and_quality_cap(self):
        nav, guard = self.make_guard()
        self.qualify(guard)
        for k in range(6, 100):
            t = k / 20.
            out = guard.update(observation(t, y=10. + t), 100. + t)
        self.assertEqual(out["gps_guard_state"], "RACE")
        self.assertAlmostEqual(out["throttle_norm"], 1.)
        out = guard.update(observation(5., y=15., quality=1), 105.)
        self.assertLessEqual(out["throttle_norm"], .3)

    def test_asymmetric_clearance_uses_each_bank(self):
        for x, clearance in ((-2., .7), (2., 1.5)):
            with self.subTest(x=x):
                _, guard = self.make_guard()
                out = guard.enter_auto(observation(0., x=x), 100.)
                self.assertAlmostEqual(out["gps_guard_clearance_m"], clearance, places=5)

    def test_rejected_jump_does_not_reach_planner(self):
        nav, guard = self.make_guard()
        self.qualify(guard)
        with patch.object(nav, "update", wraps=nav.update) as update:
            out = guard.update(observation(.3, x=30.), 100.3)
        self.assertEqual(out["throttle_norm"], 0.)
        self.assertEqual(out["gps_guard_reason"], "position_jump")
        update.assert_not_called()

    def test_planned_neutral_and_reverse_are_preserved_and_capped(self):
        nav, guard = self.make_guard()
        out = self.qualify(guard, heading=90., x=1.8, speed=.5)
        self.assertEqual(out["planner_state"], "plan_neutral_in")
        self.assertEqual(out["throttle_norm"], 0.)
        for k in range(6, 16):
            t = k / 20.
            out = guard.update(observation(t, x=1.8, y=10., course_deg=270.,
                                           course_speed_mps=.5, quality=1), 100. + t)
        self.assertEqual(out["planner_state"], "plan_reverse")
        self.assertLess(out["throttle_norm"], 0.)
        self.assertGreaterEqual(out["throttle_norm"], -.3)
        self.assertIsNone(guard._last_forward_course)
        self.assertFalse(guard.heading_info["confident"])

    def test_fault_during_reverse_cancels_plan_and_preserves_line(self):
        nav, guard = self.make_guard()
        self.qualify(guard, heading=90., x=1.8, speed=.5)
        line = nav.line_guidance.line_offset_m
        out = guard.update(observation(.3, valid=False, quality=0), 100.3)
        self.assertEqual((out["throttle_norm"], out["steer_norm"]), (0., 0.))
        self.assertIsNone(nav.rev_phase)
        self.assertEqual(nav.turn_dir, 0)
        self.assertEqual(nav.line_guidance.line_offset_m, line)
        self.assertIsNone(guard._last_forward_course)

    def test_forward_turn_uses_qualified_course(self):
        nav, guard = self.make_guard()
        out = self.qualify(guard, heading=70., speed=.5)
        self.assertIn(out["planner_state"], ("turn_left", "turn_right"))
        self.assertGreater(out["throttle_norm"], 0.)
        self.assertLessEqual(out["throttle_norm"], guard.options["boundary_throttle"])

    def test_no_safe_turn_holds_instead_of_least_bad_turn(self):
        nav, guard = self.make_guard(plan_max_reverses=0)
        out = self.qualify(guard, heading=180., speed=.5)
        self.assertEqual(out["throttle_norm"], 0.)
        self.assertEqual(out["gps_guard_reason"], "planner_no_safe_turn")
        self.assertEqual(nav.turn_dir, 0)

    def test_reverse_times_out_without_starting_unsafe_arc(self):
        nav, guard = self.make_guard(plan_reverse_max_sec=.4, plan_reverse_mps=3.)
        self.qualify(guard, heading=90., x=1.8, speed=.5)
        outputs = []
        for k in range(6, 28):
            t = k / 20.
            outputs.append(guard.update(observation(t, x=1.8, y=10., moving=False), 100. + t))
        self.assertTrue(any(o.get("planner_state") == "plan_reverse" for o in outputs))
        self.assertEqual(outputs[-1]["throttle_norm"], 0.)
        self.assertEqual(nav.turn_dir, 0)

    def reverse_into_turn(self, guard):
        self.qualify(guard, heading=90., x=1.8, speed=.5)
        for k in range(6, 90):
            t = k / 20.
            x = max(.65, 1.8 - .5 * max(0., t - .6))
            out = guard.update(observation(t, x=x, y=10., course_deg=270.,
                                           course_speed_mps=.5), 100. + t)
            if out.get("reacquire_heading"):
                self.assertEqual(out["gps_guard_state"], "ACQUIRE")
                self.assertGreater(out["throttle_norm"], 0.)
                self.assertFalse(guard.heading_info["confident"])
                return k, x
        self.fail("Reverse did not reach bounded forward turn acquisition")

    def test_backward_course_never_qualifies_after_reverse_and_launch_times_out(self):
        nav, guard = self.make_guard()
        start, x = self.reverse_into_turn(guard)
        for k in range(start + 1, start + 35):
            t = k / 20.
            out = guard.update(observation(t, x=x, y=10., course_deg=270.,
                                           course_speed_mps=.5), 100. + t)
            self.assertFalse(guard.heading_info["confident"])
            if out["gps_guard_state"] == "HOLD":
                break
            self.assertLessEqual(out["throttle_norm"], guard.options["launch_throttle"])
        self.assertEqual(out["gps_guard_reason"], "turn_acquisition_timeout")
        self.assertEqual(out["throttle_norm"], 0.)
        self.assertEqual(nav.turn_dir, 0)

    def test_fresh_forward_course_reacquires_turn_after_reverse(self):
        nav, guard = self.make_guard()
        start, x = self.reverse_into_turn(guard)
        for k in range(start + 1, start + 18):
            t = k / 20.
            out = guard.update(observation(t, x=x, y=10., course_deg=90.,
                                           course_speed_mps=.5), 100. + t)
        self.assertEqual(out["gps_guard_state"], "CAUTION")
        self.assertIsNone(guard._turn_acquire_start)
        self.assertIn(out["planner_state"], ("turn_left", "turn_right"))
        self.assertGreater(out["throttle_norm"], 0.)


if __name__ == "__main__":
    unittest.main()
