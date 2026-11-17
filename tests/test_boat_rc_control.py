"""Exercise boat.py's real loop without opening hardware or creating session logs."""
import builtins
import contextlib
import importlib.util
import io
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]


def load_boat():
    real_open = builtins.open
    def open_file(path, *args, **kwargs):
        if str(path).startswith(str(ROOT / 'auto_log_')):
            return io.StringIO()
        return real_open(path, *args, **kwargs)
    spec = importlib.util.spec_from_file_location('boat_rc_test', ROOT / 'boat.py')
    module = importlib.util.module_from_spec(spec)
    with patch('builtins.open', side_effect=open_file), patch('signal.signal'), contextlib.redirect_stdout(io.StringIO()):
        spec.loader.exec_module(module)
    return module


class RCControlTests(unittest.TestCase):
    def setUp(self):
        self.b = load_boat()
        self.addCleanup(self.b.log_file.close)
        for handler in self.b.SESSION_LOG.handlers[:]:
            if isinstance(handler, self.b._EventLogBridge):
                self.addCleanup(self.b.SESSION_LOG.removeHandler, handler)

    def test_takeover_gestures(self):
        b = self.b
        for mode in ('AUTO', 'BALLOON'):
            for steer, throttle in ((b.HW['st_left'], b.TH_NEU), (b.HW['st_right'], b.TH_NEU), (b.ST_NEU, b.HW['th_rev'])):
                self.assertTrue(b.rc_takeover_requested(mode, steer, throttle))
            self.assertFalse(b.rc_takeover_requested(mode, b.ST_NEU, b.TH_NEU))
            self.assertFalse(b.rc_takeover_requested(mode, b.ST_NEU, b.HW['th_fwd']))
        self.assertTrue(b.rc_takeover_requested('MANUAL', b.ST_NEU, b.HW['th_fwd']))

    def run_loop(self, commands, pulses, cycles=8, gps=None):
        b = self.b
        clock, cycle, records = [100.], [0], []
        b.WEB_ENABLE = b.CAM_ENABLE = b.NTRIP_ENABLE = False
        b._gcs_ever_connected = True  # Dashboard has disconnected.
        b._last_gcs_hb_mono = 0.
        def receiver():
            cycle[0] += 1
            if cycle[0] > cycles:
                raise KeyboardInterrupt
            for cmd in commands.get(cycle[0], []):
                b.web_cmd_queue.append(cmd)
            return pulses(cycle[0])
        def sleep(delay):
            clock[0] += max(delay, .02)
        def output(steer, drive, **kwargs):
            b._last_steer, b._last_drive = steer, drive
            records.append((cycle[0], steer, drive))
        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.object(b.NTRIP, 'start'))
            for name, value in [('init_peripherals', Mock()), ('load_pid_config', Mock()),
                                ('save_pid_config', Mock()), ('read_jetson_health', Mock(return_value=(0, 0))),
                                ('read_latest_rc', Mock(side_effect=receiver)),
                                ('read_latest_gps', Mock(side_effect=gps) if callable(gps) else Mock(return_value=gps)),
                                ('set_pwm', Mock(side_effect=output)), ('safe_stop', Mock())]:
                stack.enter_context(patch.object(b, name, value))
            stack.enter_context(patch.object(b.time, 'time', side_effect=lambda: clock[0]))
            stack.enter_context(patch.object(b.time, 'monotonic', side_effect=lambda: clock[0]))
            stack.enter_context(patch.object(b.time, 'sleep', side_effect=sleep))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            with self.assertRaises(KeyboardInterrupt):
                b.main_loop()
            return records, b.safe_stop.call_count

    def test_rc_overrides_web_only_and_link_hold_and_stays_at_neutral(self):
        b = self.b
        records, _ = self.run_loop({1: [{'cmd': 'set_rc_ignore', 'value': True}],
                                   3: [{'cmd': 'web_manual_drive', 'value': {'drive': b.max_GO}}]},
                                  lambda n: (b.ST_NEU, b.HW['th_fwd'] if n < 3 else b.TH_NEU))
        self.assertTrue(records)
        self.assertEqual(records[0][2], b.max_GO)
        self.assertTrue(all(drive == b.STOP for n, _, drive in records if n >= 3))

    def test_auto_takeover_has_no_arming_delay(self):
        b = self.b
        lat, lon = b.PATHS['Center'][0]
        records, _ = self.run_loop({1: [{'cmd': 'set_mode', 'value': 'AUTO'}]},
                                  lambda n: (b.HW['st_left'], b.TH_NEU),
                                  gps=(lat, lon, 0., 1., 4, 15, .8))
        self.assertEqual(records[0], (1, b.max_LEFT, b.STOP))
        self.assertEqual(b.shared_state['mode'], 'MANUAL')

    def run_rc_auto(self, pulses=None, commands=None, gps=True, cycles=140):
        b = self.b
        lat, lon = b.PATHS['Center'][0]
        return self.run_loop(commands or {}, pulses or (lambda n: (b.HW['st_right'], b.TH_NEU)),
                             cycles=cycles, gps=(lat, lon, 0., 1., 4, 15, .8) if gps else None)

    def test_rc_auto_entry_survives_held_right_and_dashboard_loss(self):
        b = self.b
        self.run_rc_auto(commands={1: [{'cmd': 'set_rc_ignore', 'value': True}]})
        self.assertEqual(b.shared_state['mode'], 'AUTO')
        self.assertTrue(b.shared_state['rcAutoActive'])
        self.assertFalse(b.shared_state['rcManualActive'])
        self.assertEqual(b.shared_state['autoGuidance'], 'STRAIGHT LINE')
        self.assertGreater(b.shared_state['wpDist'], 0.)

    def test_test_square_keeps_its_course_and_configured_throttle(self):
        b = self.b
        b.AUTO_STRATEGY.options.update(test_square=True, test_throttle=0.25)
        line = b.NAVIGATOR.line_guidance
        with patch.object(line, 'draw_line', wraps=line.draw_line) as draw:
            records, _ = self.run_rc_auto(commands={120: [{'cmd': 'set_path', 'value': 'Left'}]})
            draw.assert_called_once()
        auto_drive = [drive for cycle, _, drive in records if cycle >= 110]
        self.assertTrue(auto_drive)
        self.assertTrue(all(drive == b.lerp(b.STOP, b.max_GO, 0.25) for drive in auto_drive))
        self.assertEqual(b.NAVIGATOR.active_lane, 'test')
        self.assertEqual(b.shared_state['autoGuidance'], 'TEST SQUARE')

    def test_straight_line_finish_returns_to_manual(self):
        b = self.b
        line = b.NAVIGATOR.line_guidance
        east, north = line.point(line.length + b.NAVIGATOR.finish_margin_m + 1., 0.)
        end = (b.NAVIGATOR.ref_lat + b.math.degrees(north / 6371000.),
               b.NAVIGATOR.ref_lon + b.math.degrees(east / (6371000. * b.math.cos(b.math.radians(b.NAVIGATOR.ref_lat)))))
        reads = [0]
        def gps(**kwargs):
            reads[0] += 1
            lat, lon = b.PATHS['Center'][0] if reads[0] < 3 else end
            return lat, lon, 0., 1., 4, 15, .8
        self.run_loop({1: [{'cmd': 'set_link_watchdog', 'value': False}, {'cmd': 'set_mode', 'value': 'AUTO'}]},
                      lambda n: (b.ST_NEU, b.TH_NEU), gps=gps)
        self.assertEqual(b.shared_state['mode'], 'MANUAL')
        self.assertTrue(any(event['tag'] == 'FINISH' for event in b.event_logs))

    def test_lane_strategy_keeps_locked_path(self):
        b = self.b
        b.AUTO_STRATEGY = b.create_auto_strategy({'strategy': '04_predictive_recovery'}, b.NAVIGATOR.params)
        paths = {b.LANE_KEYS[name]: [(i + 1, lat, lon) for i, (lat, lon) in enumerate(points)]
                 for name, points in b.PATHS.items()}
        b.NAVIGATOR = b.MultipathNavigator(paths, b.NAVIGATOR.params, strategy=b.AUTO_STRATEGY)
        b.HEADING_SELECTOR = b.MotionHeadingSelector(b.AUTO_STRATEGY.options)
        lat, lon = b.PATHS['Center'][0]
        self.run_loop({1: [{'cmd': 'set_link_watchdog', 'value': False}, {'cmd': 'set_path', 'value': 'Left'},
                          {'cmd': 'set_mode', 'value': 'AUTO'}]}, lambda n: (b.ST_NEU, b.TH_NEU),
                      gps=(lat, lon, 0., 1., 4, 15, .8))
        self.assertEqual(b.NAVIGATOR.active_lane, 'left')
        self.assertEqual(b.shared_state['targetPath'], 'Left')
        self.assertEqual(b.shared_state['autoGuidance'], '')

    def test_rc_auto_entry_requires_continuous_hold_and_neutral_throttle(self):
        b = self.b
        self.run_rc_auto(pulses=lambda n: (b.ST_NEU if n == 70 else b.HW['st_right'], b.TH_NEU))
        self.assertEqual(b.shared_state['mode'], 'MANUAL')
        self.run_rc_auto(pulses=lambda n: (b.HW['st_right'], b.HW['th_fwd']))
        self.assertEqual(b.shared_state['mode'], 'MANUAL')

    def test_rc_auto_rejects_missing_gps_and_estop(self):
        b = self.b
        self.run_rc_auto(gps=False)
        self.assertEqual(b.shared_state['mode'], 'MANUAL')
        records, _ = self.run_rc_auto(commands={1: [{'cmd': 'estop_on', 'value': True}]})
        self.assertEqual(records, [])
        self.assertEqual(b.shared_state['mode'], 'MANUAL')

    def test_reverse_always_exits_rc_auto_even_with_entry_steering_held(self):
        b = self.b
        self.run_rc_auto(pulses=lambda n: (b.HW['st_right'], b.TH_NEU if n < 115 else b.HW['th_rev']))
        self.assertEqual(b.shared_state['mode'], 'MANUAL')
        self.assertTrue(b.shared_state['rcManualActive'])

    def test_right_steering_takeover_rearms_after_release(self):
        b = self.b
        self.run_rc_auto(pulses=lambda n: (b.ST_NEU if 115 <= n < 120 else b.HW['st_right'], b.TH_NEU))
        self.assertEqual(b.shared_state['mode'], 'MANUAL')
        self.assertTrue(b.shared_state['rcManualActive'])

    def test_rc_auto_stops_on_receiver_loss(self):
        b = self.b
        records, stops = self.run_rc_auto(pulses=lambda n: (b.HW['st_right'], b.TH_NEU) if n < 115 else (None, None), cycles=155)
        self.assertEqual(b.shared_state['mode'], 'MANUAL')
        self.assertTrue(b.shared_state['rcManualActive'])
        self.assertFalse(b.shared_state['rcAutoActive'])
        self.assertLess(max(n for n, _, _ in records), 150)
        self.assertGreater(stops, 2)

    def test_estop_still_blocks_rc(self):
        b = self.b
        records, stops = self.run_loop({1: [{'cmd': 'estop_on', 'value': True}]},
                                       lambda n: (b.ST_NEU, b.HW['th_fwd']))
        self.assertEqual(records, [])
        self.assertGreater(stops, 1)

    def test_rc_loss_stops_latched_control(self):
        b = self.b
        records, stops = self.run_loop({}, lambda n: (b.ST_NEU, b.HW['th_fwd']) if n == 1 else (None, None), cycles=40)
        self.assertTrue(records)
        self.assertLess(max(n for n, _, _ in records), 35)
        self.assertGreater(stops, 2)

    def test_link_hold_still_stops_web_without_rc_takeover(self):
        b = self.b
        records, stops = self.run_loop({1: [{'cmd': 'web_manual_drive', 'value': {'drive': b.max_GO}}]},
                                       lambda n: (b.ST_NEU, b.TH_NEU))
        self.assertEqual(records, [])
        self.assertGreater(stops, 1)

    def test_driver_initializes_on_calibrated_bus_in_neutral(self):
        b = self.b
        b._last_steer, b._last_drive = b.max_LEFT, b.max_GO
        b._need_safe_stop_retry = False
        with patch.object(b, 'PCA9685Driver') as driver:
            driver.return_value.actual_freq_hz = b.PCA_FREQ_HZ
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertTrue(b.init_pca9685())
            driver.assert_called_once_with(bus_num=b.HW['i2c_bus'], address=b.HW['pca_addr'], freq_hz=b.HW['pca_freq'])
            driver.return_value.set_pwm_us.assert_any_call(b.ESC_CH, b.HW['esc_neutral'])
            self.assertIsNone(b._last_steer)
            self.assertIsNone(b._last_drive)
            self.assertTrue(b._need_safe_stop_retry)

    def test_driver_initialization_failure_closes_partial_connection(self):
        b = self.b
        with patch.object(b, 'PCA9685Driver') as driver:
            driver.return_value.set_pwm_us.side_effect = OSError('disconnected')
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertFalse(b.init_pca9685())
            driver.return_value.close.assert_called_once_with()
        self.assertIsNone(b.pca)
        self.assertTrue(b._need_safe_stop_retry)
        self.assertEqual(b.shared_state['lampSerial'], 'err')

    def test_threaded_rc_delivers_each_packet_once_and_expires(self):
        b = self.b
        b.rc_reader = b.ArduinoRCReader('unused')
        b.rc_reader.ch1_us, b.rc_reader.ch2_us = b.ST_NEU, b.HW['th_fwd']
        b.rc_reader.last_rx_time = 100.
        with patch.object(b.time, 'monotonic', return_value=100.1):
            self.assertEqual(b.read_latest_rc(), (b.ST_NEU, b.HW['th_fwd']))
            self.assertEqual(b.read_latest_rc(), (None, None))
            b.rc_reader.last_rx_time = 100.2
            self.assertEqual(b.read_latest_rc(), (b.ST_NEU, b.HW['th_fwd']))
        b.rc_reader.last_rx_time = 100.3
        with patch.object(b.time, 'monotonic', return_value=100.9):
            self.assertEqual(b.read_latest_rc(), (None, None))

    def test_calibrated_driver_outputs_and_missing_driver(self):
        b = self.b
        b.pca = Mock()
        b.set_pwm(b.max_LEFT, b.max_GO, force=True)
        b.pca.set_pwm_us.assert_any_call(b.SERVO_CH, b.HW['servo_left'])
        b.pca.set_pwm_us.assert_any_call(b.ESC_CH, b.HW['esc_fwd'])
        b.safe_stop()
        b.pca.set_pwm_us.assert_any_call(b.ESC_CH, b.HW['esc_neutral'])
        b.pca = None
        b.set_pwm(b.CENTER, b.max_GO)
        self.assertIsNone(b._last_drive)
        self.assertEqual(b.shared_state['lampSerial'], 'err')


if __name__ == '__main__':
    unittest.main()
