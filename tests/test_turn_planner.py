"""Offline tests for 07_turn_planner: turn geometry, direction choice, reverse, line placement.

The starting conditions come from real 05 starts on 2026-11-17 18:39 (routes 2, 3 and 6 of the RTK plot),
where turning the short way round drove the boat toward the bank.

Run from the korea_boat folder:
    PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p 'test_*.py' -v
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import multipath_boat as mb  # noqa: E402
import test_race_features as loop_harness  # noqa: E402
from auto_strategies import create_auto_strategy  # noqa: E402
from path_guidance import plan_turn, plan_turn_by_time, turn_arc  # noqa: E402

NAME = "07_turn_planner"
DEFAULTS = create_auto_strategy({"strategy": NAME}, mb.NAV_PARAMS).options
R_LEFT, R_RIGHT = DEFAULTS["turn_radius_left_m"], DEFAULTS["turn_radius_right_m"]
LOW, HIGH = -DEFAULTS["bank_left_m"], DEFAULTS["bank_right_m"]


def lead(speed):
    return speed * DEFAULTS["turn_delay_sec"]


def wrap(angle):
    return (angle + 180.0) % 360.0 - 180.0


class TurnGeometryTest(unittest.TestCase):
    def test_right_turn_from_pointing_right(self):
        # Pointing straight at the right bank: a right turn carries on outward by one radius, then comes back
        # across to finish two radii to the left of where the circle started.
        arc = turn_arc(0.0, 90.0, 2.0, +1)
        self.assertAlmostEqual(arc["sweep_deg"], 270.0)
        self.assertAlmostEqual(arc["max_d"], 2.0, places=3)
        self.assertAlmostEqual(arc["end_d"], -2.0, places=3)

    def test_left_turn_from_pointing_right(self):
        arc = turn_arc(0.0, 90.0, 2.0, -1)
        self.assertAlmostEqual(arc["sweep_deg"], 90.0)
        self.assertAlmostEqual(arc["end_d"], 2.0, places=3)
        self.assertAlmostEqual(arc["min_d"], 0.0, places=3)

    def test_lead_distance_moves_the_start_of_the_circle(self):
        arc = turn_arc(1.0, 90.0, 2.0, -1, lead_m=0.5)
        self.assertAlmostEqual(arc["end_d"], 1.0 + 0.5 + 2.0, places=3)

    def test_already_on_course_is_no_turn(self):
        arc = turn_arc(0.4, 0.0, 2.0, +1)
        self.assertEqual((arc["sweep_deg"], round(arc["end_d"], 6)), (0.0, 0.4))


class PlanDirectionTest(unittest.TestCase):
    """Real starts from the 18:39 run: d (m from centerline), course offset (deg), GPS speed (m/s)."""

    def plan(self, d, phi, speed):
        return plan_turn(d, phi, lead(speed), R_LEFT, R_RIGHT, LOW, HIGH, DEFAULTS["turn_tolerance_m"])

    def test_route_6_turns_away_from_the_right_bank(self):
        arc, fits = self.plan(2.89, 137.0, 1.38)       # Turned left (short way) and hit the bank on the water
        self.assertTrue(fits)
        self.assertEqual(arc["direction"], +1)
        self.assertLessEqual(arc["max_d"], HIGH)

    def test_route_3_turns_right_instead_of_swinging_out(self):
        arc, fits = self.plan(0.03, 106.0, 1.65)       # Swung out to +3.99 m turning left on the water
        self.assertTrue(fits)
        self.assertEqual(arc["direction"], +1)
        self.assertLess(arc["max_d"], 2.5)

    def test_route_2_takes_the_least_bad_turn(self):
        arc, fits = self.plan(-1.07, -153.0, 1.35)     # Turned right (short way) into the left bank on the water
        self.assertFalse(fits)
        self.assertEqual(arc["direction"], -1)
        self.assertLess(arc["violation_m"], 0.5)

    def test_shortest_arc_wins_when_both_fit(self):
        arc, fits = self.plan(0.0, 70.0, 1.5)
        self.assertTrue(fits)
        self.assertEqual(arc["direction"], -1)


class PlannerNavigatorTest(unittest.TestCase):
    def setUp(self, **options):
        paths = {lane: mb.load_waypoints_csv(os.path.join(os.path.dirname(__file__), "..", p))
                 for lane, p in mb.PATH_FILES.items()}
        strategy = create_auto_strategy({"strategy": NAME, NAME: options}, mb.NAV_PARAMS)
        self.nav = mb.MultipathNavigator(paths, dict(mb.NAV_PARAMS, finish_margin_m=1.0), strategy=strategy)
        self.line = self.nav.line_guidance

    def latlon(self, s, d):
        x, y = self.line.point(s, d)
        return (self.nav.ref_lat + math.degrees(y / mb.EARTH_RADIUS_M),
                self.nav.ref_lon + math.degrees(x / (mb.EARTH_RADIUS_M * math.cos(math.radians(self.nav.ref_lat)))))

    def step(self, s, d, phi, t, speed=1.4):
        heading = None if phi is None else (self.line.bearing_deg + phi) % 360.0
        return self.nav.update(*self.latlon(s, d), heading, now_mono=t, heading_confident=True,
                               motion={"speed_mps": speed})

    def test_route_6_turns_right_holds_the_turn_and_hands_back_to_the_line(self):
        self.nav.enter_auto_mode(*self.latlon(14.0, 2.89), now_mono=0.0)
        first = self.step(14.0, 2.89, 137.0, 0.7)
        self.assertEqual(first["prediction_status"], "turn_right")
        self.assertGreater(first["steer_norm"], 0.0)
        self.assertEqual(first["throttle_blend"], 1.0)
        self.assertAlmostEqual(self.line.line_offset_m, -0.72, places=1)
        # Past 180 deg the short way round would be left again: the planned right turn must continue.
        for k, phi in enumerate((170.0, -150.0, -90.0, -40.0)):
            info = self.step(14.5, 3.0, phi, 0.8 + 0.1 * k)
            self.assertEqual(info["prediction_status"], "turn_right")
            self.assertGreater(info["steer_norm"], 0.0)
        done = self.step(16.0, 1.0, -20.0, 1.5)
        self.assertIn(done["prediction_status"], ("merging", "line"))

    def test_route_3_line_drawn_where_the_turn_ends(self):
        self.nav.enter_auto_mode(*self.latlon(35.4, 0.03), now_mono=0.0)
        info = self.step(35.4, 0.03, 106.0, 0.7, speed=1.65)
        self.assertEqual(info["prediction_status"], "turn_right")
        arc, _ = plan_turn(0.03, 106.0, lead(1.65), R_LEFT, R_RIGHT, LOW, HIGH, DEFAULTS["turn_tolerance_m"])
        self.assertAlmostEqual(self.line.line_offset_m, max(-self.line.line_left_m, arc["end_d"]), places=6)

    def test_facing_the_bank_reverses_until_a_turn_fits_then_turns_at_once(self):
        self.nav.enter_auto_mode(*self.latlon(20.0, 3.5), now_mono=0.0)
        start = self.step(20.0, 3.5, 90.0, 1.0, speed=1.0)
        self.assertEqual((start["prediction_status"], start["throttle_norm"], start["steer_norm"]),
                         ("plan_neutral_in", 0.0, 0.0))
        t = 1.0 + DEFAULTS["plan_neutral_sec"] + 0.05
        rev = self.step(20.0, 3.5, -90.0, t, speed=0.5)             # GPS course now points backward
        self.assertEqual(rev["prediction_status"], "plan_reverse")
        self.assertEqual(rev["throttle_norm"], -DEFAULTS["plan_reverse_throttle"])
        still = self.step(20.0, 2.6, -90.0, t + 1.0, speed=0.5)      # Not enough room yet: keep reversing
        self.assertEqual(still["prediction_status"], "plan_reverse")
        room = self.step(20.0, 1.2, -90.0, t + 2.0, speed=0.5)       # A turn now fits
        self.assertEqual(room["prediction_status"], "plan_neutral_out")
        turn = self.step(20.0, 1.2, -90.0, t + 2.0 + DEFAULTS["plan_neutral_sec"] + 0.05, speed=0.2)
        self.assertIn(turn["prediction_status"], ("turn_right", "turn_left"))
        self.assertTrue(turn["reacquire_heading"])                    # Backward course dropped, no forward re-run
        self.assertNotIn("throttle_norm", turn)                        # Full forward again
        arc, fits = plan_turn(1.2, 90.0, DEFAULTS["turn_restart_lead_m"], R_LEFT, R_RIGHT, LOW, HIGH,
                              DEFAULTS["turn_tolerance_m"])
        self.assertTrue(fits)
        self.assertEqual(turn["prediction_status"], "turn_right" if arc["direction"] > 0 else "turn_left")
        # The turn continues while the course is still unknown after the reverse.
        self.assertEqual(self.step(20.5, 1.3, None, t + 2.6)["prediction_status"], turn["prediction_status"])

    def test_reverse_gives_up_after_the_time_limit(self):
        self.nav.enter_auto_mode(*self.latlon(20.0, 3.5), now_mono=0.0)
        self.step(20.0, 3.5, 90.0, 1.0, speed=1.0)
        t = 1.0 + DEFAULTS["plan_neutral_sec"] + 0.05
        self.step(20.0, 3.5, -90.0, t, speed=0.5)
        stuck = self.step(20.0, 3.5, -90.0, t + DEFAULTS["plan_reverse_max_sec"] + 0.1, speed=0.0)
        self.assertEqual(stuck["prediction_status"], "plan_neutral_out")
        least_bad = self.step(20.0, 3.5, -90.0, t + DEFAULTS["plan_reverse_max_sec"] + 0.5, speed=0.0)
        self.assertIn(least_bad["prediction_status"], ("turn_right", "turn_left"))

    def test_no_reverse_when_it_would_not_gain_room(self):
        # Pointing straight backward mid-channel: reversing moves along the course, not away from a limit.
        self.nav.enter_auto_mode(*self.latlon(40.0, 0.5), now_mono=0.0)
        info = self.step(40.0, 0.5, 180.0, 1.0, speed=1.0)
        self.assertIn(info["prediction_status"], ("turn_right", "turn_left"))
        self.assertEqual(self.nav.reverses, 0)

    def test_reverse_limit_then_least_bad_turn(self):
        self.setUp(plan_max_reverses=0)
        self.nav.enter_auto_mode(*self.latlon(20.0, 3.5), now_mono=0.0)
        info = self.step(20.0, 3.5, 90.0, 1.0, speed=1.0)
        self.assertIn(info["prediction_status"], ("turn_right", "turn_left"))

    def test_small_first_course_draws_the_line_where_the_turn_ends(self):
        self.nav.enter_auto_mode(*self.latlon(1.0, 2.28), now_mono=0.0)
        self.assertAlmostEqual(self.line.line_offset_m, 2.28, places=6)        # Inside the +3.4 m line limit
        info = self.step(1.5, 2.28, -15.0, 0.7, speed=1.7)
        expected = turn_arc(2.28, -15.0, R_RIGHT, +1, lead(1.7))["end_d"]
        self.assertAlmostEqual(self.line.line_offset_m, expected, places=6)
        self.assertIn(info["prediction_status"], ("merging", "line"))

    def test_line_limits_are_per_side(self):
        self.nav.enter_auto_mode(*self.latlon(1.0, 3.9), now_mono=0.0)
        self.assertAlmostEqual(self.line.line_offset_m, self.line.line_right_m, places=6)
        self.nav.enter_auto_mode(*self.latlon(1.0, -3.0), now_mono=0.0)
        self.assertAlmostEqual(self.line.line_offset_m, -self.line.line_left_m, places=6)

    def test_mid_run_upset_triggers_a_new_plan(self):
        self.nav.enter_auto_mode(*self.latlon(10.0, 0.0), now_mono=0.0)
        self.step(11.0, 0.0, 0.0, 0.7)
        self.assertEqual(self.step(12.0, 0.0, 2.0, 0.8)["prediction_status"], "line")
        hit = self.step(13.0, 0.5, 120.0, 0.9)                                 # Knocked sideways, e.g. a collision
        self.assertIn(hit["prediction_status"], ("turn_right", "turn_left"))

    def test_planner_off_turns_the_short_way(self):
        self.setUp(turn_planner=False)
        self.nav.enter_auto_mode(*self.latlon(14.0, 2.89), now_mono=0.0)
        info = self.step(14.0, 2.89, 137.0, 0.7, speed=1.38)
        self.assertNotIn(info["prediction_status"], ("turn_right", "turn_left", "plan_neutral_in"))
        self.assertLess(wrap(info["target_bearing_deg"] - (self.line.bearing_deg + 137.0)), 0.0)


class ThreePointTurnTest(unittest.TestCase):
    """--three-point-turns: pick turn right / turn left / reverse-then-turn by estimated time."""

    def by_time(self, d, phi, speed, **overrides):
        o = dict(DEFAULTS, **overrides)
        return plan_turn_by_time(d, phi, lead(speed), o["turn_radius_left_m"], o["turn_radius_right_m"], LOW, HIGH,
                                 o["turn_tolerance_m"], o["plan_cruise_mps"], o["plan_reverse_mps"],
                                 o["plan_restart_sec"], o["plan_neutral_sec"],
                                 o["plan_reverse_mps"] * o["plan_reverse_max_sec"], o["turn_restart_lead_m"])

    def test_short_reverse_and_short_turn_beats_a_full_circle(self):
        best = self.by_time(0.44, 106.0, 1.65)                   # Route 3: the forward-only planner turns right 254 deg
        self.assertEqual(best["direction"], -1)
        self.assertGreater(best["reverse_m"], 0.0)
        self.assertLess(best["reverse_m"], 0.5)
        self.assertAlmostEqual(best["arc"]["sweep_deg"], 106.0, places=0)

    def test_no_reverse_when_the_forward_turn_is_quicker(self):
        best = self.by_time(3.28, 137.0, 1.4)                    # Route 6
        self.assertEqual((best["direction"], best["reverse_m"]), (+1, 0.0))

    def test_slow_restart_makes_the_full_circle_win(self):
        best = self.by_time(0.44, 106.0, 1.65, plan_restart_sec=20.0)
        self.assertEqual((best["direction"], best["reverse_m"]), (+1, 0.0))

    def test_cost_counts_ground_lost_behind_the_start(self):
        # The 254 deg circle finishes about 2 m behind its start; the 106 deg turn finishes ahead.
        circle = turn_arc(0.44, 106.0, R_RIGHT, +1)
        short = turn_arc(0.44, 106.0, R_LEFT, -1)
        self.assertLess(circle["advance_m"], -1.5)
        self.assertGreater(short["advance_m"], 2.5)

    def test_navigator_reverses_for_the_chosen_turn_then_turns(self):
        nav_test = PlannerNavigatorTest("test_line_limits_are_per_side")
        nav_test.setUp(three_point_turns=True)
        nav_test.nav.enter_auto_mode(*nav_test.latlon(35.0, 0.44), now_mono=0.0)
        start = nav_test.step(35.0, 0.44, 106.0, 0.7, speed=1.65)
        self.assertEqual(start["prediction_status"], "plan_neutral_in")
        self.assertEqual(nav_test.nav.rev_dir, -1)
        t = 0.7 + DEFAULTS["plan_neutral_sec"] + 0.05
        self.assertEqual(nav_test.step(35.0, 0.44, -74.0, t, speed=0.3)["prediction_status"], "plan_reverse")
        done = nav_test.step(34.9, 0.10, -74.0, t + 1.0, speed=0.4)          # Backed off: the left turn fits
        self.assertEqual(done["prediction_status"], "plan_neutral_out")
        turn = nav_test.step(34.9, 0.10, -74.0, t + 1.0 + DEFAULTS["plan_neutral_sec"] + 0.05, speed=0.1)
        self.assertEqual(turn["prediction_status"], "turn_left")
        self.assertLess(turn["steer_norm"], 0.0)

    def test_flag_off_keeps_the_forward_only_planner(self):
        nav_test = PlannerNavigatorTest("test_line_limits_are_per_side")
        nav_test.setUp()
        nav_test.nav.enter_auto_mode(*nav_test.latlon(35.0, 0.44), now_mono=0.0)
        self.assertEqual(nav_test.step(35.0, 0.44, 106.0, 0.7, speed=1.65)["prediction_status"], "turn_right")

    def test_flag_is_07_only(self):
        from auto_strategies import apply_run_overrides
        auto = mb.load_calibration(os.path.join(os.path.dirname(__file__), "..", "calibration.json"))["autonomous"]
        with self.assertRaises(ValueError):
            apply_run_overrides(dict(auto, strategy="05_straight_line"), three_point_turns=True)
        strategy = create_auto_strategy(apply_run_overrides(dict(auto, strategy=NAME), three_point_turns=True),
                                        mb.NAV_PARAMS)
        self.assertTrue(strategy.options["three_point_turns"])
        self.assertIn("three-point", strategy.describe_throttle(1914, 1990))


class SeparateFrom05Test(unittest.TestCase):
    """07 is a separate strategy: 05 keeps the night-test behaviour (symmetric limits, no planner)."""

    def navigator(self, name, **options):
        paths = {lane: mb.load_waypoints_csv(os.path.join(os.path.dirname(__file__), "..", p))
                 for lane, p in mb.PATH_FILES.items()}
        strategy = create_auto_strategy({"strategy": name, name: options}, mb.NAV_PARAMS)
        return mb.MultipathNavigator(paths, dict(mb.NAV_PARAMS, finish_margin_m=1.0), strategy=strategy)

    def test_05_has_no_planner_and_symmetric_limits(self):
        nav = self.navigator("05_straight_line")
        line = nav.line_guidance
        self.assertEqual((line.bank_left_m, line.bank_right_m, line.line_left_m, line.line_right_m),
                         (2.5, 2.5, 1.5, 1.5))
        x, y = line.point(14.0, 2.0)
        lat = nav.ref_lat + math.degrees(y / mb.EARTH_RADIUS_M)
        lon = nav.ref_lon + math.degrees(x / (mb.EARTH_RADIUS_M * math.cos(math.radians(nav.ref_lat))))
        nav.enter_auto_mode(lat, lon, now_mono=0.0)
        info = nav.update(lat, lon, (line.bearing_deg + 137.0) % 360.0, now_mono=0.7, heading_confident=True,
                          motion={"speed_mps": 1.4})
        self.assertNotIn(info["prediction_status"], ("turn_right", "turn_left", "plan_neutral_in"))
        self.assertAlmostEqual(line.line_offset_m, 1.5, places=6)               # Clamped at entry, never moved

    def test_07_test_square_uses_per_side_limits(self):
        nav = self.navigator(NAME, test_square=True)
        line = nav.line_guidance
        x, y = line.point(10.0, 0.0)
        lat = nav.ref_lat + math.degrees(y / mb.EARTH_RADIUS_M)
        lon = nav.ref_lon + math.degrees(x / (mb.EARTH_RADIUS_M * math.cos(math.radians(nav.ref_lat))))
        nav.enter_auto_mode(lat, lon, now_mono=0.0)
        self.assertIn("-3.20/+4.00 m (bank limits) then -2.60/+3.40 m (line limits)", nav.describe_line())
        info = nav.update(lat, lon, line.bearing_deg, now_mono=0.1, heading_confident=True, motion={"speed_mps": 1.0})
        self.assertEqual(info["prediction_status"], "square leg 1: bank -3.20/+4.00 m to start")

    def test_per_side_flags_refused_for_05(self):
        from auto_strategies import apply_run_overrides
        auto = mb.load_calibration(os.path.join(os.path.dirname(__file__), "..", "calibration.json"))["autonomous"]
        with self.assertRaises(ValueError):
            apply_run_overrides(dict(auto, strategy="05_straight_line"), bank_right_m=4.5)
        o = create_auto_strategy(apply_run_overrides(dict(auto, strategy=NAME), bank_right_m=4.5, line_limit_m=2.0),
                                 mb.NAV_PARAMS).options
        self.assertEqual((o["bank_left_m"], o["bank_right_m"], o["line_left_m"], o["line_right_m"]), (3.2, 4.5, 2.0, 2.0))


class PlannerAutoLoopTest(unittest.TestCase):
    """The real main loop with mocked hardware, boat 3.5 m right of the centerline facing the right bank."""

    run_loop = loop_harness.AutoLoopFeatureTest.run_loop

    def test_reverse_reaches_the_esc_and_reacquires_the_course(self):
        rows, events, cal = self.run_loop(argv=["--auto-strategy", NAME], speed=1.0, cycles=150, course_deg=90.0,
                                          offset_m=3.5)
        self.assertTrue(all(r["auto_strategy"] == NAME for r in rows if r["mode"] == "AUTO"))
        statuses = [r["prediction_status"] for r in rows]
        pwm = cal["pwm"]
        reverse_us = mb.map_norm_to_pwm(-DEFAULTS["plan_reverse_throttle"], pwm["esc_neutral_us"],
                                        pwm["esc_rev_max_us"], pwm["esc_fwd_max_us"])
        reversing = [r for r in rows if r["prediction_status"] == "plan_reverse"]
        self.assertTrue(reversing)
        self.assertTrue(all(int(r["esc_us"]) == reverse_us and int(r["servo_us"]) == pwm["servo_center_us"]
                            for r in reversing))
        after = statuses.index("plan_neutral_out")
        self.assertIn("acquiring_forward_motion", [r["heading_state"] for r in rows[after:]])
        self.assertIn("[TURN PLAN]", events)
        self.assertIn("reverse until a turn fits", events)
        self.assertTrue(all(r["recovery_state"] == "" for r in rows))       # Not mistaken for being stuck

    def test_three_point_flag_from_the_command_line(self):
        # 0.5 m right of the centerline, 106 deg off: the forward-only planner would circle 254 deg to the right.
        rows, events, _ = self.run_loop(argv=["--auto-strategy", NAME, "--three-point-turns"], speed=1.65,
                                        cycles=60, course_deg=106.0, offset_m=0.5)
        self.assertIn("three-point turn, reverse about", events)
        self.assertIn("plan_reverse", [r["prediction_status"] for r in rows])

    def test_command_line_selects_07_with_one_sided_limit(self):
        rows, events, _ = self.run_loop(argv=["--auto-strategy", NAME, "--test-square", "--bank-right", "4.5"],
                                        speed=1.0, cycles=60)
        self.assertIn("-3.20/+4.50 m (bank limits)", events)
        auto = [r for r in rows if r["mode"] == "AUTO"][1:]
        self.assertTrue(auto and all(r["prediction_status"].startswith("square leg 1: bank -3.20/+4.50 m") for r in auto))


if __name__ == "__main__":
    unittest.main()
