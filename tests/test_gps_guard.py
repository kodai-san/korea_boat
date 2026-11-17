"""Offline safety scenarios for the mode 06 observation and power governor."""

import math
import unittest

from auto_strategies import create_auto_strategy
from gps_guard import EARTH_RADIUS_M, GpsAwareController
from path_guidance import StraightLineGuidance


class RecordingNavigator:
    """Real strategy/line geometry with recorded navigator inputs and no hardware."""

    ref_lat, ref_lon = 35.0, 129.0

    def __init__(self, **options):
        params = dict(max_steer_angle_deg=35., boundary_gain_boost=1.45,
                      servo_slew_per_cycle=.18, lookahead_dist_m=3.5)
        self.strategy = create_auto_strategy(
            {"strategy": "06_gps_aware", "06_gps_aware": options}, params)
        opts = self.strategy.options
        self.line_guidance = StraightLineGuidance((0., 0.), (0., 100.),
                                                 opts["band_half_width_m"], opts["line_max_offset_m"])
        self.entries, self.updates = [], []

    def xy(self, lat, lon):
        return (math.radians(lon - self.ref_lon) * EARTH_RADIUS_M * math.cos(math.radians(self.ref_lat)),
                math.radians(lat - self.ref_lat) * EARTH_RADIUS_M)

    def enter_auto_mode(self, lat, lon, now_mono):
        self.entries.append((lat, lon))
        self.line_guidance.draw_line(*self.xy(lat, lon))
        self.strategy.reset()
        return "line", 1, 0.

    def update(self, lat, lon, heading, now_mono, heading_confident, motion):
        self.updates.append((lat, lon))
        s, d = self.line_guidance.frame(*self.xy(lat, lon))
        target = self.line_guidance.desired_course(d, 3., 15.)
        error = (target - heading + 180) % 360 - 180
        guard = abs(d) >= 2.
        command = self.strategy.update(cte_right_m=d - self.line_guidance.line_offset_m,
                                       heading_error_deg=error, boundary_guard=guard, now_mono=now_mono,
                                       heading_confident=heading_confident, speed_mps=motion["speed_mps"])
        return dict(finished=s >= 100., active_lane="line", shifted_to=None, wp_number=1,
                    cte_right_m=d - self.line_guidance.line_offset_m, target_bearing_deg=target,
                    boundary_guard=guard, **command)


def observation(t, x=0., y=None, quality=4, moving=True, source=None, **overrides):
    if y is None:
        y = 10. + t if moving else 10.
    seconds = t if source is None else source
    source_seconds = (43200. + seconds) % 86400.
    hours = int(source_seconds // 3600)
    minutes = int(source_seconds % 3600 // 60)
    second = source_seconds % 60
    result = dict(valid=True, fix_quality=quality,
                  lat=35. + math.degrees(y / EARTH_RADIUS_M),
                  lon=129. + math.degrees(x / (EARTH_RADIUS_M * math.cos(math.radians(35.)))),
                  age_sec=0., gps_utc=f"{hours:02d}{minutes:02d}{second:05.2f}",
                  num_sats=20, hdop=.7, diff_age_sec=1.,
                  course_valid=moving, course_deg=0., course_speed_mps=1. if moving else 0.,
                  course_age_sec=0., course_time_mono=100. + t, course_gga_skew_sec=0.)
    result.update(overrides)
    return result


class GpsGuardTests(unittest.TestCase):
    def make_guard(self, **options):
        nav = RecordingNavigator(**options)
        return nav, GpsAwareController(nav)

    def run_to_race(self, guard):
        guard.observe_manual(observation(-1.), .5, 99.)
        guard.observe_manual(observation(-.2), .5, 99.8)
        output = guard.enter_auto(observation(0.), 100.)
        self.assertEqual(output["throttle_norm"], 0.)
        for tick in range(1, 121):
            t = tick / 20.
            output = guard.update(observation(t), 100. + t)
        self.assertEqual(output["gps_guard_state"], "RACE")
        self.assertAlmostEqual(output["throttle_norm"], 1.)
        return output

    def test_entry_qualifies_distinct_source_epochs_before_navigation(self):
        nav, guard = self.make_guard()
        sample = observation(0.)
        guard.enter_auto(sample, 100.)
        for tick in range(1, 50):
            out = guard.update(sample, 100. + tick / 100.)
        self.assertEqual(out["gps_guard_state"], "HOLD")
        self.assertEqual(nav.entries, [])
        self.assertEqual(nav.updates, [])
        self.assertEqual(guard._good_fixes, 1)

    def test_full_power_and_immediate_float_dgps_gps_caps(self):
        nav, guard = self.make_guard()
        self.run_to_race(guard)
        floated = guard.update(observation(6.05, quality=5), 106.05)
        dgps = guard.update(observation(6.1, quality=2), 106.1)
        gps = guard.update(observation(6.15, quality=1), 106.15)
        self.assertEqual((floated["gps_guard_state"], floated["throttle_norm"]), ("CAUTION", .7))
        self.assertEqual((dgps["gps_guard_state"], dgps["throttle_norm"]), ("CAUTION", .5))
        self.assertEqual((gps["gps_guard_state"], gps["throttle_norm"]), ("CAUTION", .3))
        self.assertFalse(guard.heading_info["confident"])

    def test_standalone_gps_can_be_explicitly_disabled(self):
        _, guard = self.make_guard(gps_throttle=0.)
        self.run_to_race(guard)
        out = guard.update(observation(6.05, quality=1), 106.05)
        self.assertEqual((out["gps_guard_state"], out["throttle_norm"]), ("HOLD", 0.))

    def test_jump_is_blocked_before_navigation_and_finish(self):
        nav, guard = self.make_guard()
        self.run_to_race(guard)
        calls, anchor = len(nav.updates), nav.updates[-1]
        for x, y in ((30., 16.05), (0., 200.)):
            out = guard.update(observation(6.05, x=x, y=y), 106.05)
            self.assertEqual(out["gps_guard_state"], "HOLD")
            self.assertEqual(out["steer_norm"], 0.)
            self.assertFalse(out["finished"])
            self.assertEqual(len(nav.updates), calls)
            self.assertEqual((out["gps_guard_accepted_latitude"], out["gps_guard_accepted_longitude"]), anchor)

    def test_changed_position_on_duplicate_epoch_is_not_a_new_fix(self):
        nav, guard = self.make_guard()
        self.run_to_race(guard)
        calls = len(nav.updates)
        out = guard.update(observation(6.05, source=6., x=.2), 106.05)
        self.assertEqual(out["gps_guard_reason"], "duplicate_position_changed")
        self.assertEqual(out["throttle_norm"], 0.)
        self.assertEqual(len(nav.updates), calls)

    def test_replayed_epoch_ages_despite_reported_zero_age(self):
        _, guard = self.make_guard()
        self.run_to_race(guard)
        sample = observation(6.)
        out = guard.update(sample, 106.61)
        self.assertEqual(out["gps_guard_reason"], "position_source_stale")
        self.assertEqual(out["throttle_norm"], 0.)

    def test_course_loss_stops_immediately_and_does_not_apply_learned_trim(self):
        nav, guard = self.make_guard()
        self.run_to_race(guard)
        nav.strategy.integral_deg_s = 20.
        calls = len(nav.updates)
        out = guard.update(observation(6.05, course_valid=False), 106.05)
        self.assertEqual((out["gps_guard_state"], out["throttle_norm"], out["steer_norm"]), ("HOLD", 0., 0.))
        self.assertEqual(len(nav.updates), calls)
        self.assertEqual(nav.strategy.integral_deg_s, 0.)

    def test_stable_fixes_resume_automatically_without_redrawing_line(self):
        nav, guard = self.make_guard()
        self.run_to_race(guard)
        guard.update(observation(6.05, quality=0, valid=False), 106.05)
        out = guard.update(observation(6.1), 106.1)
        self.assertEqual(out["throttle_norm"], 0.)
        for tick in range(123, 215):
            t = tick / 20.
            out = guard.update(observation(t), 100. + t)
        self.assertEqual(out["gps_guard_state"], "RACE")
        self.assertAlmostEqual(out["throttle_norm"], 1.)
        self.assertEqual(len(nav.entries), 1)

    def test_no_course_without_forward_evidence_never_blindly_launches(self):
        nav, guard = self.make_guard()
        guard.enter_auto(observation(0., moving=False), 100.)
        outputs = [guard.update(observation(t / 20., moving=False), 100. + t / 20.) for t in range(1, 160)]
        self.assertTrue(all(out["throttle_norm"] == 0. for out in outputs))
        self.assertEqual(nav.updates, [])

    def test_one_bounded_acquisition_ramps_power_and_never_repeats_after_timeout(self):
        _, guard = self.make_guard(launch_heading_memory_sec=30.)
        guard.observe_manual(observation(-1.), .5, 99.)
        guard.observe_manual(observation(-.1), .5, 99.9)
        guard.enter_auto(observation(0., moving=False), 100.)
        outputs = [guard.update(observation(t / 20., moving=False), 100. + t / 20.) for t in range(1, 240)]
        acquire = [out for out in outputs if out["gps_guard_state"] == "ACQUIRE"]
        self.assertTrue(acquire)
        self.assertGreater(max(out["throttle_norm"] for out in acquire), .2)
        self.assertLessEqual(max(out["throttle_norm"] for out in acquire), .3)
        self.assertTrue(all(out["throttle_norm"] == 0. for out in outputs[75:]))
        self.assertEqual(sum(outputs[i]["gps_guard_state"] == "ACQUIRE" and
                             outputs[i - 1]["gps_guard_state"] != "ACQUIRE" for i in range(1, len(outputs))), 1)

    def test_dgps_uncertainty_near_bank_stops_instead_of_steering(self):
        nav, guard = self.make_guard()
        guard.observe_manual(observation(-1., x=1.), .5, 99.)
        guard.observe_manual(observation(-.2, x=1.), .5, 99.8)
        guard.enter_auto(observation(0., x=1.), 100.)
        for tick in range(1, 121):
            t = tick / 20.
            out = guard.update(observation(t, x=1.), 100. + t)
        calls = len(nav.updates)
        out = guard.update(observation(6.05, x=1., quality=2), 106.05)
        self.assertEqual(out["gps_guard_reason"], "insufficient_clearance")
        self.assertEqual(len(nav.updates), calls)

    def test_accuracy_is_conservative_and_malformed_values_hold(self):
        _, guard = self.make_guard()
        self.run_to_race(guard)
        out = guard.update(observation(6.05, hacc_m=1.1, hacc_age_sec=0., hacc_gga_skew_sec=0.), 106.05)
        self.assertEqual(out["gps_guard_reason"], "insufficient_clearance")
        out = guard.update(observation(6.1, hacc_m=float("nan"), hacc_age_sec=0., hacc_gga_skew_sec=0.), 106.1)
        self.assertEqual(out["gps_guard_reason"], "invalid_accuracy")

    def test_midnight_wrap_is_advancing_and_out_of_order_epoch_holds(self):
        _, guard = self.make_guard()
        guard.enter_auto(observation(0., source=43199.8), 100.)
        out = guard.update(observation(.2, source=43200.), 100.2)
        self.assertEqual(out["gps_guard_reason"], "qualifying_position")
        out = guard.update(observation(.3, source=43199.9), 100.3)
        self.assertEqual(out["gps_guard_reason"], "position_out_of_order")

    def test_fresh_course_without_validated_forward_thrust_cannot_steer(self):
        for manual_actions in ([], [(-1., -.5)], [(-1., .5), (-.8, .5)]):
            with self.subTest(manual_actions=manual_actions):
                nav, guard = self.make_guard()
                for t, throttle in manual_actions:
                    guard.observe_manual(observation(t), throttle, 100. + t)
                guard.enter_auto(observation(0.), 100.)
                outputs = [guard.update(observation(t / 20.), 100. + t / 20.) for t in range(1, 121)]
                self.assertTrue(all(out["throttle_norm"] == 0. for out in outputs))
                self.assertEqual(nav.updates, [])

    def test_non_fixed_manual_forward_evidence_allows_conditional_caution(self):
        for quality, cap in ((5, .7), (2, .5), (1, .3)):
            with self.subTest(quality=quality):
                _, guard = self.make_guard()
                guard.observe_manual(observation(-1., quality=quality), .5, 99.)
                guard.observe_manual(observation(-.2, quality=quality), .5, 99.8)
                guard.enter_auto(observation(0., quality=quality), 100.)
                for tick in range(1, 121):
                    t = tick / 20.
                    out = guard.update(observation(t, quality=quality), 100. + t)
                self.assertEqual((out["gps_guard_state"], out["throttle_norm"]), ("CAUTION", cap))
                self.assertFalse(guard.heading_info["confident"])

    def test_receiver_publication_just_after_loop_timestamp_is_usable(self):
        _, guard = self.make_guard()
        self.run_to_race(guard)
        out = guard.update(observation(6.05, course_time_mono=106.07), 106.05)
        self.assertEqual(out["gps_guard_state"], "RACE")
        self.assertEqual(guard.heading_info["age_sec"], 0.)

    def test_stale_bad_accuracy_cannot_silently_revert_to_good_floor(self):
        _, guard = self.make_guard()
        self.run_to_race(guard)
        out = guard.update(observation(6.05, hacc_m=1.2, hacc_age_sec=0., hacc_gga_skew_sec=0.), 106.05)
        self.assertEqual(out["gps_guard_reason"], "insufficient_clearance")
        for tick in range(122, 165):
            t = tick / 20.
            out = guard.update(observation(t, hacc_m=1.2, hacc_age_sec=t - 6.05, hacc_gga_skew_sec=0.), 100. + t)
            self.assertEqual(out["throttle_norm"], 0.)
            self.assertGreaterEqual(out["gps_guard_uncertainty_m"], 2.4)
        out = guard.update(observation(8.25, hacc_m=.1, hacc_age_sec=0., hacc_gga_skew_sec=0.), 108.25)
        self.assertEqual(out["gps_guard_reason"], "qualifying_position")
        self.assertAlmostEqual(out["gps_guard_uncertainty_m"], .25)

    def test_explicit_stale_accuracy_opt_out_uses_quality_floor(self):
        _, guard = self.make_guard(retain_stale_accuracy=False)
        self.run_to_race(guard)
        guard.update(observation(6.05, hacc_m=1.2, hacc_age_sec=0., hacc_gga_skew_sec=0.), 106.05)
        out = guard.update(observation(6.1, hacc_m=1.2, hacc_age_sec=2., hacc_gga_skew_sec=0.), 106.1)
        self.assertAlmostEqual(out["gps_guard_uncertainty_m"], .25)

    def test_automatic_resume_can_be_disabled(self):
        nav, guard = self.make_guard(auto_resume=False)
        self.run_to_race(guard)
        guard.update(observation(6.05, valid=False, quality=0), 106.05)
        calls = len(nav.updates)
        for tick in range(122, 240):
            t = tick / 20.
            out = guard.update(observation(t), 100. + t)
            self.assertEqual((out["gps_guard_reason"], out["throttle_norm"]), ("manual_rearm_required", 0.))
        self.assertEqual(len(nav.updates), calls)

    def test_coherent_float_dgps_and_gps_clusters_can_rebase_after_source_gap(self):
        # Keep direction evidence available here to isolate position requalification;
        # a long stationary hold with expired direction evidence is tested separately.
        for quality, cap in ((5, .7), (2, .5), (1, .3)):
            with self.subTest(quality=quality):
                nav, guard = self.make_guard(forward_memory_sec=20., launch_heading_memory_sec=20.)
                self.run_to_race(guard)
                guard.update(observation(6.05, valid=False, quality=0), 106.05)
                calls, original = len(nav.updates), guard._anchor["lat"]
                out = guard.update(observation(8.4, quality=quality), 108.4)
                self.assertEqual(out["gps_guard_reason"], "position_gap")
                self.assertEqual(out["gps_guard_accepted_latitude"], original)
                for tick in range(169, 208):
                    t = tick / 20.
                    out = guard.update(observation(t, quality=quality), 100. + t)
                    self.assertEqual(len(nav.updates), calls)
                for tick in range(208, 285):
                    t = tick / 20.
                    out = guard.update(observation(t, quality=quality), 100. + t)
                self.assertEqual((out["gps_guard_state"], out["throttle_norm"]), ("CAUTION", cap))
                self.assertGreater(out["gps_guard_accepted_latitude"], original)
                self.assertEqual(len(nav.entries), 1)

    def test_small_inconsistent_steps_are_rejected_when_they_accumulate(self):
        _, guard = self.make_guard()
        self.run_to_race(guard)
        rejected = None
        for tick in range(121, 140):
            t = tick / 20.
            out = guard.update(observation(t, x=2. * (t - 6.)), 100. + t)
            if out["gps_guard_state"] == "HOLD":
                rejected = out
                break
        self.assertIsNotNone(rejected)
        self.assertEqual(rejected["gps_guard_reason"], "position_jump")
        self.assertEqual(rejected["throttle_norm"], 0.)

    def test_first_fix_after_float_cannot_immediately_enable_trim_learning(self):
        _, guard = self.make_guard()
        self.run_to_race(guard)
        guard.update(observation(6.05, quality=5), 106.05)
        out = guard.update(observation(6.1), 106.1)
        self.assertEqual(out["gps_guard_state"], "CAUTION")
        self.assertFalse(guard.heading_info["confident"])
        for tick in range(123, 166):
            t = tick / 20.
            out = guard.update(observation(t), 100. + t)
        self.assertEqual(out["gps_guard_state"], "RACE")
        self.assertTrue(guard.heading_info["confident"])

    def test_expired_forward_orientation_cannot_restart_after_a_long_hold(self):
        nav, guard = self.make_guard()
        self.run_to_race(guard)
        guard.update(observation(6.05, valid=False, quality=0), 106.05)
        calls = len(nav.updates)
        # Stable position close to the previous boat location requalifies, but the
        # old forward orientation must not authorize a burst many seconds later.
        for tick in range(300, 440):
            t = tick / 20.
            out = guard.update(observation(t, y=17., moving=False), 100. + t)
            self.assertEqual(out["throttle_norm"], 0.)
        self.assertEqual(len(nav.updates), calls)

    def test_normal_two_second_neutral_gesture_preserves_bounded_launch_evidence(self):
        nav, guard = self.make_guard()
        for tick in range(21):
            t = tick / 20.
            guard.observe_manual(observation(t), .5, 100. + t)
        for tick in range(21, 61):
            t = tick / 20.
            guard.observe_manual(observation(t, moving=False, y=11.), 0., 100. + t)
        self.assertEqual(guard._manual_course[0], 101.)
        guard.enter_auto(observation(3., moving=False, y=11.), 103.)
        outputs = []
        for tick in range(61, 134):
            t = tick / 20.
            outputs.append(guard.update(observation(t, moving=False, y=11.), 100. + t))
        acquire = [out for out in outputs if out["gps_guard_state"] == "ACQUIRE"]
        self.assertTrue(acquire)
        self.assertGreater(max(out["throttle_norm"] for out in acquire), .2)
        self.assertEqual(outputs[-1]["gps_guard_state"], "HOLD")
        self.assertEqual(outputs[-1]["throttle_norm"], 0.)
        self.assertEqual(nav.updates, [])

    def test_reverse_or_contradictory_neutral_course_erases_launch_evidence(self):
        for throttle, heading in ((-.1, 0.), (0., 180.)):
            with self.subTest(throttle=throttle, heading=heading):
                _, guard = self.make_guard()
                guard.observe_manual(observation(0.), .5, 100.)
                guard.observe_manual(observation(1.), .5, 101.)
                self.assertIsNotNone(guard._manual_course)
                guard.observe_manual(observation(1.1, course_deg=heading), throttle, 101.1)
                self.assertIsNone(guard._manual_course)

    def test_normal_neutral_gesture_can_launch_and_continue_on_each_non_fixed_quality(self):
        for quality, cap in ((5, .7), (2, .5), (1, .3)):
            with self.subTest(quality=quality):
                _, guard = self.make_guard()
                for tick in range(21):
                    t = tick / 20.
                    guard.observe_manual(observation(t, quality=quality), .5, 100. + t)
                for tick in range(21, 61):
                    t = tick / 20.
                    guard.observe_manual(observation(t, quality=quality, moving=False, y=11.), 0., 100. + t)
                guard.enter_auto(observation(3., quality=quality, moving=False, y=11.), 103.)
                outputs = []
                for tick in range(61, 181):
                    t = tick / 20.
                    moving = t >= 5.65
                    y = 11. + max(0., t - 5.65)
                    outputs.append(guard.update(observation(t, quality=quality, moving=moving, y=y), 100. + t))
                acquire = [out for out in outputs if out["gps_guard_state"] == "ACQUIRE"]
                self.assertTrue(acquire)
                self.assertTrue(all(out["throttle_norm"] <= min(.3, cap) for out in acquire))
                self.assertTrue(any(out["throttle_norm"] > 0. for out in acquire))
                self.assertEqual((outputs[-1]["gps_guard_state"], outputs[-1]["throttle_norm"]), ("CAUTION", cap))
                self.assertFalse(guard.heading_info["confident"])

    def test_gps_acquisition_respects_a_configured_cap_below_launch_throttle(self):
        _, guard = self.make_guard(gps_throttle=.2, launch_heading_memory_sec=30.)
        guard.observe_manual(observation(-1., quality=1), .5, 99.)
        guard.observe_manual(observation(-.2, quality=1), .5, 99.8)
        guard.enter_auto(observation(0., quality=1, moving=False), 100.)
        outputs = [guard.update(observation(t / 20., quality=1, moving=False), 100. + t / 20.) for t in range(1, 160)]
        acquire = [out for out in outputs if out["gps_guard_state"] == "ACQUIRE"]
        self.assertTrue(acquire)
        self.assertAlmostEqual(max(out["throttle_norm"] for out in acquire), .2)
        self.assertTrue(all(out["gps_guard_throttle_cap"] == .2 for out in acquire))
        self.assertEqual(outputs[-1]["throttle_norm"], 0.)

    def test_standalone_gps_ignores_correction_age_but_differential_fixes_do_not(self):
        for quality in (1, 2, 4, 5):
            with self.subTest(quality=quality):
                _, guard = self.make_guard()
                self.run_to_race(guard)
                out = guard.update(observation(6.05, quality=quality, diff_age_sec=999.), 106.05)
                if quality == 1:
                    self.assertEqual((out["gps_guard_state"], out["throttle_norm"]), ("CAUTION", .3))
                else:
                    self.assertEqual((out["gps_guard_reason"], out["throttle_norm"]), ("corrections_stale", 0.))

    def test_enabled_gps_still_holds_for_no_fix_bad_accuracy_bank_or_jump(self):
        for overrides, reason in (
                ({"quality": 0, "valid": False}, "no_position_fix"),
                ({"hacc_m": 1.2, "hacc_age_sec": 0., "hacc_gga_skew_sec": 0.}, "insufficient_clearance"),
                ({"x": .8}, "insufficient_clearance"),
                ({"x": 30.}, "position_jump")):
            with self.subTest(reason=reason, overrides=overrides):
                nav, guard = self.make_guard()
                self.run_to_race(guard)
                calls = len(nav.updates)
                parameters = dict(quality=1, **{k: v for k, v in overrides.items() if k != "quality"})
                parameters["quality"] = overrides.get("quality", 1)
                out = guard.update(observation(6.05, **parameters), 106.05)
                self.assertEqual((out["gps_guard_reason"], out["throttle_norm"], out["steer_norm"]), (reason, 0., 0.))
                self.assertEqual(len(nav.updates), calls)


if __name__ == "__main__":
    unittest.main()
