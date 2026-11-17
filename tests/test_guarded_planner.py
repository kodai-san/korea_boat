"""Offline tests for 08_guarded_turn_planner: 06's GPS guard in front of 07's turn planner.

Run from the korea_boat folder:
    PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p 'test_*.py' -v
"""

import contextlib
import io
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import multipath_boat as mb  # noqa: E402
import test_race_features as loop_harness  # noqa: E402
from auto_strategies import create_auto_strategy  # noqa: E402
from guarded_planner import GuardedPlannerController  # noqa: E402

NAME = "08_guarded_turn_planner"
DT = 0.2                                   # One receiver epoch


class GuardedPlannerTest(unittest.TestCase):
    def setUp(self, **options):
        paths = {lane: mb.load_waypoints_csv(os.path.join(os.path.dirname(__file__), "..", p))
                 for lane, p in mb.PATH_FILES.items()}
        strategy = create_auto_strategy({"strategy": NAME, NAME: options}, mb.NAV_PARAMS)
        self.nav = mb.MultipathNavigator(paths, dict(mb.NAV_PARAMS, finish_margin_m=1.0), strategy=strategy)
        self.line = self.nav.line_guidance
        self.guard = GuardedPlannerController(self.nav)
        self.epoch = 0
        self.t = 100.0

    def snap(self, s, d, phi, quality=4, speed=1.0):
        """One fresh receiver epoch at (s, d) in the course frame, moving at phi deg off the race direction."""
        x, y = self.line.point(s, d)
        seconds = 43200.0 + self.epoch * DT
        utc = "%02d%02d%05.2f" % (int(seconds // 3600), int(seconds % 3600 // 60), seconds % 60)
        return dict(valid=True, fix_quality=quality,
                    lat=self.nav.ref_lat + math.degrees(y / mb.EARTH_RADIUS_M),
                    lon=self.nav.ref_lon + math.degrees(x / (mb.EARTH_RADIUS_M * math.cos(math.radians(self.nav.ref_lat)))),
                    age_sec=0.0, gps_utc=utc, num_sats=20, hdop=0.7, diff_age_sec=1.0,
                    course_valid=True, course_deg=(self.line.bearing_deg + phi) % 360.0, course_speed_mps=speed,
                    course_age_sec=0.0, course_time_mono=self.t, course_gga_skew_sec=0.0)

    def tick(self, s, d, phi, **kw):
        self.epoch += 1
        self.t += DT
        with contextlib.redirect_stdout(io.StringIO()):
            return self.guard.update(self.snap(s, d, phi, **kw), self.t)

    def engage(self, s, d, phi, **kw):
        """Drive forward in MANUAL along phi for 1 s (06's forward evidence), then engage AUTO."""
        for _ in range(5):
            self.epoch += 1
            self.t += DT
            self.guard.observe_manual(self.snap(s, d, phi, **kw), 0.5, self.t)
        self.epoch += 1
        self.t += DT
        with contextlib.redirect_stdout(io.StringIO()):
            return self.guard.enter_auto(self.snap(s, d, phi, **kw), self.t)

    def run_for(self, seconds, s, d, phi, **kw):
        out = None
        for _ in range(int(round(seconds / DT))):
            out = self.tick(s, d, phi, **kw)
        return out

    def test_qualifies_then_races_at_full_power_on_the_line(self):
        first = self.engage(20.0, 0.0, 0.0)
        self.assertEqual((first["gps_guard_state"], first["throttle_norm"]), ("HOLD", 0.0))
        out = self.run_for(6.0, 20.0, 0.0, 0.0)
        self.assertEqual(out["gps_guard_state"], "RACE")
        self.assertAlmostEqual(out["throttle_norm"], 1.0)

    def test_route_6_planned_turn_runs_at_race_power_not_the_heading_cap(self):
        # 137 deg off at +2.89 m: 07 plans a right turn; 06 alone would cap at 35% for a 20+ deg heading error.
        self.engage(14.0, 2.89, 137.0)
        out = self.run_for(5.0, 14.0, 2.89, 137.0)
        self.assertEqual(out["prediction_status"], "turn_right")
        self.assertEqual(out["gps_guard_state"], "TURN")
        self.assertEqual(out["gps_guard_throttle_cap"], 1.0)
        self.assertGreater(out["throttle_norm"], 0.35)
        self.assertGreater(out["steer_norm"], 0.0)

    def test_planner_reverses_on_fix_but_not_on_float(self):
        self.engage(20.0, 3.5, 90.0)                          # Facing the right bank, no turn fits
        out = self.run_for(3.0, 20.0, 3.5, 90.0)
        statuses = []
        for _ in range(4):
            out = self.tick(20.0, 3.5, 90.0)
            statuses.append(out["prediction_status"])
        self.assertIn("plan_reverse", statuses)
        self.assertLess(out["throttle_norm"], 0.0)
        floated = self.tick(20.0, 3.5, -90.0, quality=5)      # RTK drops to FLOAT mid-reverse
        self.assertGreaterEqual(floated["throttle_norm"], 0.0)
        self.assertEqual(floated["gps_guard_state"], "HOLD")

    def test_near_bank_on_dgps_still_holds_like_06(self):
        self.engage(20.0, 3.5, 90.0, quality=2)
        out = self.run_for(4.0, 20.0, 3.5, 90.0, quality=2)
        self.assertEqual((out["gps_guard_state"], out["gps_guard_reason"]), ("HOLD", "insufficient_clearance"))
        self.assertEqual(out["throttle_norm"], 0.0)

    def test_position_jump_still_holds_and_ends_the_manoeuvre(self):
        self.engage(14.0, 2.89, 137.0)
        self.run_for(5.0, 14.0, 2.89, 137.0)
        self.assertNotEqual(self.nav.turn_dir, 0)
        jump = self.tick(44.0, 2.89, 137.0)                    # 30 m in one epoch
        self.assertEqual((jump["gps_guard_state"], jump["gps_guard_reason"]), ("HOLD", "position_jump"))
        self.assertEqual(self.nav.turn_dir, 0)
        self.assertTrue(self.nav.plan_pending)

    def test_float_turn_uses_the_quality_cap(self):
        self.engage(14.0, 0.3, 106.0, quality=5)
        out = self.run_for(5.0, 14.0, 0.3, 106.0, quality=5)
        self.assertEqual(out["prediction_status"], "turn_right")
        self.assertEqual(out["gps_guard_throttle_cap"], 0.7)

    def test_strategy_limits_and_flags(self):
        strategy = self.nav.strategy
        self.assertEqual(strategy.limits(), (3.2, 4.0, 1.7, 2.5))
        self.assertTrue(strategy.uses_gps_guard and strategy.guard_with_planner and strategy.options["turn_planner"])
        with self.assertRaises(ValueError):
            create_auto_strategy({"strategy": NAME, NAME: {"test_square": True}}, mb.NAV_PARAMS)
        with self.assertRaises(ValueError):          # Race clearance must fit inside the nearer bank limit
            create_auto_strategy({"strategy": NAME, NAME: {"bank_left_m": 1.2, "line_left_m": 0.5}}, mb.NAV_PARAMS)


class GuardedPlannerAutoLoopTest(unittest.TestCase):
    """The real main loop builds the 08 guard (the harness GPS never qualifies, so AUTO must hold neutral)."""

    run_loop = loop_harness.AutoLoopFeatureTest.run_loop

    def test_main_loop_uses_the_08_guard(self):
        rows, events, cal = self.run_loop(argv=["--auto-strategy", NAME], speed=1.0, cycles=60)
        auto = [r for r in rows if r["mode"] == "AUTO"]
        self.assertTrue(auto and all(r["auto_strategy"] == NAME for r in auto))
        self.assertTrue(all(int(r["esc_us"]) == cal["pwm"]["esc_neutral_us"] for r in auto))
        self.assertIn("AUTO armed with GPS guard", events)
        self.assertIn("[GPS GUARD]", events)
        self.assertIn("OFF (GPS-guarded modes 06/08", events)


if __name__ == "__main__":
    unittest.main()
