"""Dashboard AUTO must use accepted GPS and absolute guard power at the actual PWM boundary."""
import math
import unittest
from unittest.mock import patch

import test_boat_rc_control as rc_harness


class DashboardGpsGuardTests(unittest.TestCase):
    run_loop = rc_harness.RCControlTests.run_loop

    def setUp(self):
        self.b = rc_harness.load_boat()
        self.addCleanup(self.b.log_file.close)
        for handler in self.b.SESSION_LOG.handlers[:]:
            if isinstance(handler, self.b._EventLogBridge):
                self.addCleanup(self.b.SESSION_LOG.removeHandler, handler)

    def configure(self, name):
        b = self.b
        options = dict(qualification_sec=.4, qualification_fixes=2, forward_acquire_sec=.1)
        strategy = b.create_auto_strategy({'strategy': name, name: options}, b.NAVIGATOR.params)
        paths = {b.LANE_KEYS[name]: [(i + 1, lat, lon) for i, (lat, lon) in enumerate(points)]
                 for name, points in b.PATHS.items()}
        b.NAVIGATOR = b.MultipathNavigator(paths, b.NAVIGATOR.params, strategy=strategy)
        b.AUTO_STRATEGY, b.HEADING_SELECTOR = strategy, None
        b.GPS_GUARD = b.GpsAwareController(b.NAVIGATOR)
        b._last_drive = None
        return b.GPS_GUARD

    def run_guarded(self, name, scenario='quality', cycles=500, commands=None, pulses=None):
        b = self.b
        guard = self.configure(name)
        line = b.NAVIGATOR.line_guidance
        if scenario == 'finish':
            line.length = 14.
        def snapshot():
            t = round(b.time.monotonic() - 102., 6)  # Main loop first arms for two seconds.
            epoch = math.floor((t + 1e-7) * 5) / 5
            quality = 4
            if scenario == 'gps_only':
                quality = 1
            elif scenario == 'quality':
                quality = 5 if 3 <= t < 4 else 0 if 4 <= t < 6 else 1 if t >= 6 else 4
            s = 10. + epoch
            if scenario == 'jump' and 4 <= t < 4.2:
                s += 100.
            x, y = line.point(s, 0.)
            lat = b.NAVIGATOR.ref_lat + math.degrees(y / 6378137.)
            lon = b.NAVIGATOR.ref_lon + math.degrees(x / (6378137. * math.cos(math.radians(b.NAVIGATOR.ref_lat))))
            return dict(valid=quality != 0, fix_quality=quality, lat=lat, lon=lon,
                        age_sec=t - epoch, num_sats=20, hdop=.7, gps_utc=f'1200{epoch:05.2f}',
                        diff_age_sec=1., heading_deg=line.bearing_deg, sog_mps=1., cog_deg=line.bearing_deg,
                        course_valid=True, course_deg=line.bearing_deg, course_speed_mps=1.,
                        course_age_sec=t - epoch, course_time_mono=102. + epoch, course_gga_skew_sec=0.)
        def gps(**kwargs):
            snap = snapshot()
            if not snap['valid']:
                return None
            return (snap['lat'], snap['lon'], snap['course_deg'], 1. / .514444,
                    snap['fix_quality'], 20, .7)
        selections = {1: [{'cmd': 'set_link_watchdog', 'value': False}],
                      51: [{'cmd': 'set_mode', 'value': 'AUTO', 'source': 'rc'}]}
        selections.update(commands or {})
        receiver = pulses or (lambda n: (b.ST_NEU, b.HW['th_fwd'] if n <= 50 else b.TH_NEU))
        with patch.object(b.NAV_GPS, 'get_snapshot', side_effect=snapshot), \
                patch.object(b.NAVIGATOR, 'update', wraps=b.NAVIGATOR.update) as nav_update:
            records, stops = self.run_loop(selections, receiver, cycles=cycles, gps=gps)
        return records, stops, nav_update, guard

    def test_guard_owns_raw_fix_loss_recovery_and_quality_power_in_both_modes(self):
        for name in ('06_gps_aware', '08_gps_aware_turn_planner'):
            with self.subTest(strategy=name):
                records, _, _, guard = self.run_guarded(name)
                self.assertEqual(self.b.shared_state['mode'], 'AUTO')
                missing = [drive for cycle, _, drive in records if 202 <= cycle < 300]
                self.assertTrue(missing)
                self.assertTrue(all(drive == self.b.STOP for drive in missing))
                gps_only = [drive for cycle, _, drive in records if cycle >= 400]
                ceiling = self.b.nav_absolute_throttle_to_pwm(.3, guarded=True)
                self.assertTrue(gps_only)
                self.assertTrue(all(self.b.STOP < drive <= ceiling for drive in gps_only))
                self.assertEqual(guard.state, 'CAUTION')

    def test_ordinary_gps_can_enter_dashboard_auto(self):
        records, _, _, guard = self.run_guarded('08_gps_aware_turn_planner', 'gps_only', cycles=250)
        self.assertEqual(self.b.shared_state['mode'], 'AUTO')
        self.assertEqual(guard.state, 'CAUTION')
        self.assertGreater(records[-1][2], self.b.STOP)

    def test_jump_does_not_reach_navigation_or_finish(self):
        records, _, nav_update, _ = self.run_guarded('08_gps_aware_turn_planner', 'jump', cycles=300)
        b = self.b
        self.assertEqual(b.shared_state['mode'], 'AUTO')
        for call in nav_update.call_args_list:
            x, y = b.latlon_to_enu(*call.args[:2], b.NAVIGATOR.ref_lat, b.NAVIGATOR.ref_lon)
            self.assertLess(b.NAVIGATOR.line_guidance.frame(x, y)[0], 20.)
        rejected = [drive for cycle, _, drive in records if 202 <= cycle < 210]
        self.assertTrue(rejected)
        self.assertTrue(all(drive == b.STOP for drive in rejected))
        self.assertFalse(any(event['tag'] == 'FINISH' for event in b.event_logs))

    def test_finish_resets_guard_and_returns_to_manual(self):
        _, stops, _, guard = self.run_guarded('08_gps_aware_turn_planner', 'finish', cycles=400)
        self.assertEqual(self.b.shared_state['mode'], 'MANUAL')
        self.assertFalse(guard._active)
        self.assertGreater(stops, 0)
        self.assertTrue(any(event['tag'] == 'FINISH' for event in self.b.event_logs))

    def test_rc_reverse_takeover_resets_guard_immediately(self):
        b = self.b
        records, _, _, guard = self.run_guarded(
            '08_gps_aware_turn_planner', 'normal', cycles=170,
            pulses=lambda n: (b.ST_NEU, b.HW['th_fwd'] if n <= 50 else
                             b.HW['th_rev'] if n >= 150 else b.TH_NEU))
        self.assertEqual(b.shared_state['mode'], 'MANUAL')
        self.assertFalse(guard._active)
        self.assertTrue(all(drive == b.max_BACK for cycle, _, drive in records if cycle >= 150))

    def test_rc_loss_stops_and_resets_guard(self):
        b = self.b
        records, stops, _, guard = self.run_guarded(
            '08_gps_aware_turn_planner', 'normal', cycles=200,
            pulses=lambda n: (None, None) if n >= 150 else
                             (b.ST_NEU, b.HW['th_fwd'] if n <= 50 else b.TH_NEU))
        self.assertEqual(b.shared_state['mode'], 'MANUAL')
        self.assertFalse(guard._active)
        self.assertGreater(stops, 1)
        self.assertLess(max(cycle for cycle, _, _ in records), 180)

    def test_estop_resets_guard_even_when_loop_skips_normal_cleanup(self):
        records, stops, _, guard = self.run_guarded(
            '08_gps_aware_turn_planner', 'normal', cycles=180,
            commands={150: [{'cmd': 'estop_on', 'value': True}]})
        self.assertEqual(self.b.shared_state['mode'], 'MANUAL')
        self.assertFalse(guard._active)
        self.assertGreater(stops, 1)
        self.assertTrue(all(cycle < 150 for cycle, _, _ in records))

    def test_signed_absolute_power_uses_reverse_range_and_caps_high_web_preset(self):
        b = self.b
        b.max_GO = 5200
        for guarded in (False, True):
            self.assertLess(b.nav_absolute_throttle_to_pwm(-.5, guarded), b.STOP)
            self.assertEqual(b.nav_absolute_throttle_to_pwm(-1., guarded), b.max_BACK)
            self.assertEqual(b.nav_absolute_throttle_to_pwm(0., guarded), b.STOP)
        forward = b.drive_us(b.nav_absolute_throttle_to_pwm(.3, guarded=True))
        self.assertLessEqual(forward, b.HW['esc_neutral'] + .3 * (b.HW['esc_fwd'] - b.HW['esc_neutral']))
        reverse = b.drive_us(b.nav_absolute_throttle_to_pwm(-.3, guarded=True))
        self.assertLessEqual(abs(reverse - b.HW['esc_neutral']), .3 * abs(b.HW['esc_rev'] - b.HW['esc_neutral']))


if __name__ == '__main__':
    unittest.main()
