"""Motion-course selection for experimental AUTO; independent of serial/hardware I/O."""

import math


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class MotionHeadingSelector:
    """Prefer fresh RMC velocity course, with a fresh RTK displacement fallback.

    Course is travel direction, not bow orientation. Recent forward commands are
    evidence for entry reuse; a new launch waits for forward thrust and a new fix.
    """

    def __init__(self, options):
        self.options = dict(options)
        self.reset()

    def reset(self):
        self._forward_start = None
        self._last_forward = None
        self._acquire_after = None
        self._suspended = False

    def _fresh_position(self, snap):
        age = snap.get("age_sec")
        return (snap.get("valid", False) and _finite(age)
                and 0 <= age <= self.options["course_max_age_sec"])

    def _fresh_course(self, snap):
        age = snap.get("course_age_sec")
        speed = snap.get("course_speed_mps")
        course = snap.get("course_deg")
        skew = snap.get("course_gga_skew_sec")
        return (self._fresh_position(snap) and snap.get("course_valid", False)
                and _finite(age) and 0 <= age <= self.options["course_max_age_sec"]
                and _finite(skew) and abs(skew) <= self.options["course_max_age_sec"]
                and _finite(speed) and speed >= self.options["min_course_speed_mps"]
                and _finite(course) and 0 <= course < 360)

    def observe_manual(self, snap, throttle_norm, now_mono):
        """Neutral coasting may retain forward evidence; reverse/stop/stale data clears it."""
        if throttle_norm < 0 or not self._fresh_course(snap):
            self._forward_start = None
            self._last_forward = None
            return
        if throttle_norm > 0:
            if self._forward_start is None:
                self._forward_start = now_mono
            ready_at = self._forward_start + self.options["forward_acquire_sec"]
            observed_at = snap.get("course_time_mono")
            if now_mono >= ready_at and _finite(observed_at) and observed_at >= ready_at:
                self._last_forward = now_mono
        else:
            self._forward_start = None

    def enter_auto(self, snap, now_mono):
        reuse = (self._last_forward is not None
                 and 0 <= now_mono - self._last_forward <= self.options["forward_memory_sec"]
                 and self._fresh_course(snap))
        # Reuse only the current validated course; never the held legacy blend.
        self._acquire_after = (snap["course_time_mono"] if reuse else
                               now_mono + self.options["forward_acquire_sec"])
        self._suspended = False
        return reuse

    def suspend(self):
        """A neutral GPS hold breaks the assumption of continuous forward thrust."""
        self._suspended = True
        self._last_forward = None
        self._forward_start = None

    @staticmethod
    def _missing(state):
        return {"heading_deg": None, "source": "none", "age_sec": None,
                "confident": False, "state": state}

    def select(self, snap, now_mono):
        if not self._fresh_position(snap):
            return self._missing("position_unavailable")
        if self._suspended:
            self._acquire_after = now_mono + self.options["forward_acquire_sec"]
            self._suspended = False
        if self._acquire_after is None or now_mono < self._acquire_after:
            return self._missing("acquiring_forward_motion")

        observed_at = snap.get("course_time_mono")
        if (self._fresh_course(snap) and _finite(observed_at)
                and observed_at >= self._acquire_after):
            return {"heading_deg": snap["course_deg"], "source": "gps_course",
                    "age_sec": snap["course_age_sec"],
                    "confident": snap.get("fix_quality") in (4, 5), "state": "tracking"}

        # A fresh low-speed report supersedes the previous movement vector.
        course_age = snap.get("course_age_sec")
        speed = snap.get("course_speed_mps")
        if (_finite(course_age)
                and 0 <= course_age <= self.options["course_max_age_sec"]
                and _finite(speed) and speed < self.options["min_course_speed_mps"]):
            return self._missing("below_minimum_speed")

        heading = snap.get("position_heading_deg")
        age = snap.get("position_heading_age_sec")
        observed_at = snap.get("position_heading_time_mono")
        speed = snap.get("position_speed_mps")
        if (snap.get("fix_quality") in (4, 5) and _finite(heading) and 0 <= heading < 360
                and _finite(age) and 0 <= age <= self.options["position_heading_max_age_sec"]
                and _finite(observed_at) and observed_at >= self._acquire_after
                and _finite(speed) and speed >= self.options["min_course_speed_mps"]):
            return {"heading_deg": heading, "source": "position", "age_sec": age,
                    "confident": True, "state": "tracking"}
        return self._missing("heading_unavailable")
