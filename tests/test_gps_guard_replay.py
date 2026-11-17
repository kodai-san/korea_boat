"""Mode-06 replay against known historical GPS discontinuities and clean input."""

import contextlib
import csv
import io
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import multipath_boat as boat
from replay_gps_guard import ROOT, main, replay_log, replay_rows, snapshot_from_row


class ReplaySnapshotTests(unittest.TestCase):
    def test_observation_ages_and_source_utc_are_preserved(self):
        row = dict(gps_valid="1", fix_quality="5", latitude="35", longitude="129",
                   gps_age_sec="1.2", gps_utc="235959.80", gps_course_valid="1",
                   gps_course_age_sec=".4", gps_course_gga_skew_sec=".2",
                   gps_course_deg="240", gps_course_speed_mps="1.5",
                   position_heading_age_sec=".5", hacc_m=".8", hacc_age_sec=".6")
        snap = snapshot_from_row(row, 100.)
        self.assertEqual(snap["gps_utc"], "235959.80")
        self.assertEqual(snap["age_sec"], 1.2)
        self.assertEqual(snap["course_time_mono"], 99.6)
        self.assertEqual(snap["position_heading_time_mono"], 99.5)
        self.assertEqual(snap["hacc_m"], .8)
        self.assertEqual(snap["hacc_age_sec"], .6)

    def test_missing_accuracy_or_course_is_not_fabricated(self):
        snap = snapshot_from_row({}, 100.)
        self.assertFalse(snap["valid"])
        self.assertFalse(snap["course_valid"])
        for name in ("hacc_m", "hacc_age_sec", "hacc_gga_skew_sec", "course_time_mono"):
            self.assertIsNone(snap[name])


class HistoricalReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mode04 = replay_log(ROOT / "logs/multipath_log_20261117_151246.csv")
        cls.mode05 = replay_log(ROOT / "logs/multipath_log_20261117_152711.csv")

    def test_mode04_position_discontinuities_are_neutral_and_centered(self):
        selected = {s["line"]: s for s in self.mode04["samples"]}
        for line in (1167, 2141):
            sample = selected[line]
            self.assertEqual((sample["state"], sample["throttle_norm"], sample["steer_norm"]), ("HOLD", 0., 0.))
            self.assertIn(sample["reason"], ("position_jump", "position_gap", "position_quarantined"))
            self.assertFalse(sample["finished"])

    def test_mode05_position_discontinuities_are_neutral_and_centered(self):
        selected = {s["line"]: s for s in self.mode05["samples"]}
        for line in (244, 474):
            sample = selected[line]
            self.assertEqual((sample["state"], sample["throttle_norm"], sample["steer_norm"]), ("HOLD", 0., 0.))
            self.assertIn(sample["reason"], ("position_jump", "position_gap", "position_quarantined"))
            self.assertFalse(sample["finished"])

    def test_no_full_throttle_without_heading_and_recorded_mode_changes_are_respected(self):
        for report in (self.mode04, self.mode05):
            self.assertEqual(report["full_throttle_without_heading_sec"], 0.)
            self.assertGreater(report["recorded_full_throttle_without_heading_sec"], 10.)
            self.assertGreater(report["rejected_jump_count"], 0)
            self.assertAlmostEqual(sum(report["state_seconds"].values()), report["observed_auto_sec"], places=5)
        self.assertEqual(self.mode04["auto_entries"], 2)
        self.assertEqual(self.mode05["auto_entries"], 1)


class ReplayControlTests(unittest.TestCase):
    def test_clean_source_reaches_race_so_replay_does_not_force_neutral(self):
        cal = boat.load_calibration(ROOT / "calibration.json")
        lat0, lon0 = 35., 129.
        end = lat0 + math.degrees(100. / boat.EARTH_RADIUS_M)
        paths = {lane: [(1, lat0, lon0), (2, end, lon0)] for lane in ("left", "center", "right")}
        rows = []
        for index in range(200):
            t = index * .05
            source = math.floor((t + 1e-8) * 5) / 5
            age = max(0., t - source)
            rows.append(dict(
                timestamp=str(1000. + t), mode="MANUAL" if t < 1. else "AUTO",
                gps_valid="1", fix_quality="4", latitude=str(lat0 + math.degrees((10 + source) / boat.EARTH_RADIUS_M)),
                longitude=str(lon0), gps_age_sec=str(age), gps_utc=f"1200{source:05.2f}",
                num_sats="20", hdop=".6", gps_diff_age_sec="1", gps_sog_mps="1", gps_cog_deg="0",
                gps_course_valid="1", gps_course_deg="0", gps_course_speed_mps="1",
                gps_course_age_sec=str(age), gps_course_gga_skew_sec="0", rc_throttle_norm=".6",
                boat_heading_deg="0", esc_us="1990",
            ))
        report = replay_rows(rows, cal, paths)
        self.assertGreater(report["state_seconds"].get("RACE", 0), 1.)
        self.assertTrue(any(s["throttle_norm"] == 1. for s in report["samples"]))
        self.assertEqual(report["full_throttle_without_heading_sec"], 0.)

    def test_json_output_does_not_overwrite_an_existing_file(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "report.json"
            output.write_text("keep me")
            with patch("replay_gps_guard.replay_log", return_value={"samples": []}), contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(FileExistsError):
                    main(["unused.csv", "--json-output", str(output)])
            self.assertEqual(output.read_text(), "keep me")


if __name__ == "__main__":
    unittest.main()
