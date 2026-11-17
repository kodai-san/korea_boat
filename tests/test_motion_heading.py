"""Offline checks for new-mode motion heading acquisition and freshness."""

import unittest

from auto_strategies import GpsCourseProgressiveStrategy
from gps_heading import MotionHeadingSelector


class MotionHeadingSelectorTests(unittest.TestCase):
    def setUp(self):
        self.selector = MotionHeadingSelector(GpsCourseProgressiveStrategy.default_options)

    @staticmethod
    def snapshot(now, **overrides):
        snap = {
            "valid": True,
            "age_sec": 0.0,
            "fix_quality": 4,
            "course_valid": True,
            "course_deg": 12.0,
            "course_speed_mps": 1.0,
            "course_age_sec": 0.0,
            "course_time_mono": now,
            "course_gga_skew_sec": 0.0,
            "position_heading_deg": 90.0,
            "position_heading_age_sec": 0.0,
            "position_heading_time_mono": now,
            "position_speed_mps": 1.0,
            # Never silently reuse the old blended/held heading.
            "heading_deg": 270.0,
        }
        snap.update(overrides)
        return snap

    def forward_then_coast(self):
        self.selector.observe_manual(self.snapshot(100.0), 0.5, 100.0)
        self.selector.observe_manual(self.snapshot(100.7), 0.5, 100.7)
        self.selector.observe_manual(self.snapshot(102.7), 0.0, 102.7)

    def start_from_rest(self):
        self.assertFalse(self.selector.enter_auto(self.snapshot(100.0), 100.0))

    def test_fresh_forward_course_survives_neutral_entry_gesture(self):
        self.forward_then_coast()
        self.assertTrue(self.selector.enter_auto(self.snapshot(102.7), 102.7))
        selected = self.selector.select(self.snapshot(102.7), 102.7)
        self.assertEqual(selected["source"], "gps_course")
        self.assertEqual(selected["heading_deg"], 12.0)
        self.assertTrue(selected["confident"])

    def test_reverse_stop_or_stale_observation_discards_forward_history(self):
        cases = [(self.snapshot(102.8), -0.1),
                 (self.snapshot(102.8, course_speed_mps=0.0), 0.0),
                 (self.snapshot(102.8, course_age_sec=0.61), 0.0),
                 (self.snapshot(102.8, valid=False), 0.0),
                 (self.snapshot(102.8, course_gga_skew_sec=0.61), 0.0)]
        for snap, throttle in cases:
            with self.subTest(snapshot=snap, throttle=throttle):
                self.selector.reset()
                self.forward_then_coast()
                self.selector.observe_manual(snap, throttle, 102.8)
                self.assertFalse(self.selector.enter_auto(self.snapshot(102.9), 102.9))
                self.assertIsNone(self.selector.select(self.snapshot(102.9), 102.9)["heading_deg"])

    def test_forward_evidence_requires_a_new_observation_after_acquisition(self):
        self.selector.observe_manual(self.snapshot(100.0), 0.5, 100.0)
        self.selector.observe_manual(self.snapshot(100.7, course_time_mono=100.5,
                                                   course_age_sec=0.2), 0.5, 100.7)
        self.assertFalse(self.selector.enter_auto(self.snapshot(100.8), 100.8))

    def test_forward_memory_expires_even_if_current_course_is_fresh(self):
        self.forward_then_coast()
        self.selector.observe_manual(self.snapshot(105.0), 0.0, 105.0)
        self.assertFalse(self.selector.enter_auto(self.snapshot(105.0), 105.0))

    def test_startup_waits_for_thrust_time_and_new_observation(self):
        self.start_from_rest()
        before = self.selector.select(self.snapshot(100.59), 100.59)
        self.assertEqual(before["state"], "acquiring_forward_motion")
        old = self.snapshot(100.7, course_time_mono=100.5, course_age_sec=0.2,
                            position_heading_time_mono=100.5, position_heading_age_sec=0.2)
        self.assertIsNone(self.selector.select(old, 100.7)["heading_deg"])
        self.assertEqual(self.selector.select(self.snapshot(100.8), 100.8)["source"], "gps_course")

    def test_course_can_steer_without_rtk_but_cannot_enable_boost(self):
        self.start_from_rest()
        for quality, confident in ((1, False), (2, False), (4, True), (5, True)):
            with self.subTest(quality=quality):
                selected = self.selector.select(self.snapshot(101.0, fix_quality=quality), 101.0)
                self.assertEqual(selected["source"], "gps_course")
                self.assertEqual(selected["confident"], confident)

    def test_fresh_pure_position_heading_is_used_when_course_is_unavailable(self):
        self.start_from_rest()
        snap = self.snapshot(101.0, course_valid=False, course_deg=None,
                             course_speed_mps=None, course_age_sec=None)
        selected = self.selector.select(snap, 101.0)
        self.assertEqual(selected["source"], "position")
        self.assertEqual(selected["heading_deg"], 90.0)
        self.assertTrue(selected["confident"])

    def test_course_freshness_and_epoch_alignment_gate_direct_selection(self):
        self.start_from_rest()
        for overrides in ({"course_age_sec": 0.61}, {"course_age_sec": -0.1},
                          {"course_gga_skew_sec": 0.61}, {"course_gga_skew_sec": -0.61},
                          {"course_gga_skew_sec": None}, {"course_deg": float("nan")},
                          {"course_deg": 360.0}, {"course_speed_mps": float("inf")}):
            with self.subTest(overrides=overrides):
                selected = self.selector.select(self.snapshot(101.0, **overrides), 101.0)
                self.assertEqual(selected["source"], "position")

    def test_fallback_requires_recent_rtk_motion_after_acquisition(self):
        self.start_from_rest()
        for overrides in ({"fix_quality": 2}, {"position_heading_age_sec": 0.81},
                          {"position_heading_age_sec": -0.1}, {"position_speed_mps": 0.44},
                          {"position_speed_mps": float("nan")},
                          {"position_heading_deg": None}, {"position_heading_deg": 360.0},
                          {"position_heading_time_mono": 100.5}):
            with self.subTest(overrides=overrides):
                snap = self.snapshot(101.0, course_valid=False, course_speed_mps=None,
                                     course_age_sec=None, **overrides)
                selected = self.selector.select(snap, 101.0)
                self.assertIsNone(selected["heading_deg"])
                self.assertFalse(selected["confident"])

    def test_fresh_low_speed_report_overrides_held_position_heading(self):
        self.start_from_rest()
        snap = self.snapshot(101.0, course_speed_mps=0.0)
        selected = self.selector.select(snap, 101.0)
        self.assertEqual(selected["state"], "below_minimum_speed")
        self.assertIsNone(selected["heading_deg"])

    def test_low_speed_report_with_blank_course_also_suppresses_fallback(self):
        self.start_from_rest()
        snap = self.snapshot(101.0, course_valid=False, course_deg=None, course_speed_mps=0.0)
        selected = self.selector.select(snap, 101.0)
        self.assertEqual(selected["state"], "below_minimum_speed")
        self.assertIsNone(selected["heading_deg"])

    def test_stale_low_speed_report_does_not_hide_new_position_motion(self):
        self.start_from_rest()
        snap = self.snapshot(101.0, course_speed_mps=0.0, course_age_sec=0.61)
        self.assertEqual(self.selector.select(snap, 101.0)["source"], "position")

    def test_unusable_position_suppresses_all_heading_sources(self):
        self.start_from_rest()
        for overrides in ({"valid": False}, {"age_sec": 0.61}, {"age_sec": -0.1},
                          {"age_sec": float("nan")}, {"age_sec": None}):
            with self.subTest(overrides=overrides):
                selected = self.selector.select(self.snapshot(101.0, **overrides), 101.0)
                self.assertEqual(selected["state"], "position_unavailable")
                self.assertIsNone(selected["heading_deg"])

    def test_gps_hold_requires_forward_reacquisition_after_recovery(self):
        self.start_from_rest()
        self.assertEqual(self.selector.select(self.snapshot(101.0), 101.0)["source"], "gps_course")
        self.selector.suspend()
        self.selector.select(self.snapshot(102.0, valid=False), 102.0)
        self.assertEqual(self.selector.select(self.snapshot(105.0), 105.0)["state"],
                         "acquiring_forward_motion")
        self.assertIsNone(self.selector.select(self.snapshot(105.59), 105.59)["heading_deg"])
        old = self.snapshot(105.7, course_time_mono=105.5, course_age_sec=0.2,
                            position_heading_time_mono=105.5, position_heading_age_sec=0.2)
        self.assertIsNone(self.selector.select(old, 105.7)["heading_deg"])
        self.assertEqual(self.selector.select(self.snapshot(105.8), 105.8)["source"], "gps_course")

    def test_reset_removes_entry_memory_and_requires_a_new_auto_entry(self):
        self.forward_then_coast()
        self.assertTrue(self.selector.enter_auto(self.snapshot(102.7), 102.7))
        self.selector.reset()
        self.assertIsNone(self.selector.select(self.snapshot(102.8), 102.8)["heading_deg"])
        self.assertFalse(self.selector.enter_auto(self.snapshot(102.8), 102.8))


if __name__ == "__main__":
    unittest.main()
