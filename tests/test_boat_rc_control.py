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
            for name, value in [('init_peripherals', Mock()), ('load_pid_config', Mock()),
                                ('save_pid_config', Mock()), ('read_jetson_health', Mock(return_value=(0, 0))),
                                ('read_latest_rc', Mock(side_effect=receiver)),
                                ('read_latest_gps', Mock(return_value=gps)),
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
        with patch.object(b, 'HW_AVAILABLE', True), patch.object(b, 'PCA9685Driver') as driver:
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertTrue(b.init_pca9685())
            driver.assert_called_once_with(bus_num=b.HW['i2c_bus'], address=b.HW['pca_addr'], freq_hz=b.HW['pca_freq'])
            driver.return_value.set_pwm_us.assert_any_call(b.ESC_CH, b.HW['esc_neutral'])
            self.assertTrue(b._need_safe_stop_retry)

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
