"""Optional NMEA accuracy parsing and epoch/freshness qualification."""

import math
import unittest

from gps_accuracy import parse_accuracy_message, nmea_utc_seconds, usable_accuracy, utc_skew_seconds


class AccuracyMessageTests(unittest.TestCase):
    @staticmethod
    def pubx(**changes):
        fields = "$PUBX,00,123456.20,3504.0,N,12904.0,E,10.0,RK,0.15,0.22,0.0,0.0,0.0,1.0,0.7,0.8,0.9,20,0,0".split(",")
        for index, value in changes.items():
            fields[int(index)] = value
        return fields

    def test_pubx_uses_horizontal_accuracy_and_source_time(self):
        result = parse_accuracy_message(self.pubx())
        self.assertEqual(result, {"utc": "123456.20", "hacc_m": .15, "source": "pubx"})

    def test_gst_combines_latitude_and_longitude_standard_deviations(self):
        for talker in ("GP", "GN", "GL"):
            result = parse_accuracy_message(f"${talker}GST,123456.20,9,8,7,6,3,4,99".split(","))
            self.assertEqual(result, {"utc": "123456.20", "hacc_m": 5., "source": "gst"})

    def test_pubx_without_a_position_solution_is_rejected(self):
        for status in ("NF", "DR", "TT", ""):
            with self.subTest(status=status):
                self.assertIsNone(parse_accuracy_message(self.pubx(**{"8": status})))

    def test_bad_or_missing_accuracy_is_never_zero(self):
        for value in ("", "nan", "inf", "-inf", "-1", "bad", None, True):
            with self.subTest(value=value):
                self.assertIsNone(parse_accuracy_message(self.pubx(**{"9": value})))
                fields = "$GNGST,123456.20,0,0,0,0,3,4,0".split(",")
                fields[6] = value
                self.assertIsNone(parse_accuracy_message(fields))
        fields = "$GNGST,123456.20,0,0,0,0,1.7e308,1.7e308,0".split(",")
        self.assertIsNone(parse_accuracy_message(fields))

    def test_invalid_times_and_unrelated_sentences_are_rejected(self):
        for utc in ("", "12345", "123456.", "240000", "126000", "125960", "-10000", "123456x", None):
            with self.subTest(utc=utc):
                self.assertIsNone(parse_accuracy_message(self.pubx(**{"2": utc})))
        for fields in (None, [], "PUBX", ["$PUBX"], ["$PUBX", "03"], ["$GNGST"], ["$GNGGA"], [None]):
            self.assertIsNone(parse_accuracy_message(fields))


class AccuracyFreshnessTests(unittest.TestCase):
    def test_midnight_skew_uses_the_nearest_day(self):
        self.assertAlmostEqual(utc_skew_seconds("000000.10", "235959.90"), .2)
        self.assertAlmostEqual(utc_skew_seconds("235959.90", "000000.10"), -.2)
        self.assertAlmostEqual(nmea_utc_seconds("235959.90"), 86399.9)

    def test_fresh_matched_epoch_is_usable(self):
        observation = dict(utc="000000.10", hacc_m=.25, source="gst", received_at=10.)
        self.assertEqual(usable_accuracy(observation, "235959.90", 10.3, .5, .3), .25)

    def test_missing_stale_future_or_mismatched_observations_are_unusable(self):
        observation = dict(utc="120000.00", hacc_m=.25, source="pubx", received_at=10.)
        for current, gga in ((11., "120000.00"), (9., "120000.00"), (10.1, "120001.00"), (10.1, "invalid")):
            self.assertIsNone(usable_accuracy(observation, gga, current, .5, .3))
        for changes in ({"hacc_m": math.nan}, {"received_at": None}, {"utc": ""}):
            self.assertIsNone(usable_accuracy(dict(observation, **changes), "120000.00", 10.1, .5, .3))
        self.assertIsNone(usable_accuracy(None, "120000.00", 10.1, .5, .3))


if __name__ == "__main__":
    unittest.main()
