"""Selectable AUTO control laws. Navigation and hardware safety stay in the controller."""

import math

from recovery import DEFAULT_RECOVERY, resolve_recovery_options


DEFAULT_AUTO_STRATEGY = "02_adaptive_throttle"


class ConstantCruiseStrategy:
    """Original main behavior: proportional heading steering and constant cruise."""

    name = "01_constant_cruise"
    uses_gps_course = False
    uses_predictive_guidance = False
    default_options = {}
    option_limits = {}

    def __init__(self, nav_params, options=None):
        self.nav_params = dict(nav_params)
        self.options = self.validate_options({} if options is None else options)
        self.validate_navigation_options(self.nav_params, self.options)
        self.reset()

    @classmethod
    def validate_options(cls, options):
        if not isinstance(options, dict):
            raise ValueError(f"autonomous.{cls.name} must be a JSON object")
        unknown = options.keys() - cls.default_options.keys()
        if unknown:
            raise ValueError(f"Unknown autonomous.{cls.name} option(s): {', '.join(sorted(unknown))}")
        resolved = dict(cls.default_options, **options)
        for key, (minimum, maximum) in cls.option_limits.items():
            value = resolved[key]
            try:
                finite = math.isfinite(value)
            except (TypeError, OverflowError):
                finite = False
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not finite or value < minimum
                    or (maximum is not None and value > maximum)):
                bounds = f"{minimum} to {maximum}" if maximum is not None else f">= {minimum}"
                raise ValueError(f"autonomous.{cls.name}.{key} must be a finite number {bounds}")
        return resolved

    @classmethod
    def validate_navigation_options(cls, nav_params, options):
        """Validate relationships with shared guidance settings, when applicable."""

    def reset(self):
        """Called on every AUTO entry, so previous steering does not carry over."""
        self.prev_steer_norm = 0.0

    def suspend(self):
        """Called while GPS control is suspended; legacy strategies retain their behavior."""

    def update(self, *, cte_right_m, heading_error_deg, boundary_guard, now_mono,
               heading_confident=True):
        """Return normalized steering [-1, 1] and cruise-to-full throttle blend [0, 1]."""
        if heading_error_deg is None:
            raw_steer = 0.0
        else:
            gain = 1.0 / self.nav_params["max_steer_angle_deg"]
            if boundary_guard:
                gain *= self.nav_params["boundary_gain_boost"]
            raw_steer = max(-1.0, min(1.0, heading_error_deg * gain))

        max_step = self.nav_params["servo_slew_per_cycle"]
        delta = max(-max_step, min(max_step, raw_steer - self.prev_steer_norm))
        steer_norm = max(-1.0, min(1.0, self.prev_steer_norm + delta))
        self.prev_steer_norm = steer_norm
        return {
            "steer_norm": steer_norm,
            "throttle_blend": self.throttle_blend(cte_right_m, heading_error_deg),
        }

    def throttle_blend(self, cte_right_m, heading_error_deg):
        return 0.0

    def describe_throttle(self, cruise_us, full_us):
        return f"constant cruise {cruise_us} us"


class AdaptiveThrottleStrategy(ConstantCruiseStrategy):
    """Branch behavior: full ESC only when both tracking errors are small."""

    name = "02_adaptive_throttle"
    default_options = {"full_throttle_cte_m": 0.30, "full_throttle_heading_deg": 11.0}
    option_limits = {"full_throttle_cte_m": (0.0, None), "full_throttle_heading_deg": (0.0, 180.0)}

    def throttle_blend(self, cte_right_m, heading_error_deg):
        return 1.0 if (
            heading_error_deg is not None
            and abs(cte_right_m) <= self.options["full_throttle_cte_m"]
            and abs(heading_error_deg) <= self.options["full_throttle_heading_deg"]
        ) else 0.0

    def describe_throttle(self, cruise_us, full_us):
        return (
            f"cruise {cruise_us} us; full {full_us} us when "
            f"|CTE|<={self.options['full_throttle_cte_m']:.2f}m "
            f"and |heading error|<={self.options['full_throttle_heading_deg']:.1f}deg"
        )


class GpsCourseProgressiveStrategy(ConstantCruiseStrategy):
    """Experimental fresh-course steering with gradually increasing tracking power.

    The heading selector uses the speed, freshness, and forward-motion options.
    Throttle interpolates between the full-power and cruise error limits, taking
    the more restrictive error. Power reductions are immediate; increases are
    limited in time so GPS reacquisition cannot produce a full-power step.
    """

    name = "03_gps_course_progressive"
    uses_gps_course = True
    default_options = {
        "full_throttle_cte_m": 0.30,
        "cruise_cte_m": 1.5,
        "full_throttle_heading_deg": 11.0,
        "cruise_heading_deg": 35.0,
        "throttle_rise_per_sec": 0.5,
        "min_course_speed_mps": 0.45,
        "course_max_age_sec": 0.6,
        "position_heading_max_age_sec": 0.8,
        "forward_acquire_sec": 0.6,
        "forward_memory_sec": 4.0,
    }
    option_limits = {
        key: (0.0, 180.0 if key.endswith("heading_deg") else None)
        for key in default_options
    }

    @classmethod
    def validate_options(cls, options):
        resolved = super().validate_options(options)
        for full_key, cruise_key in (
                ("full_throttle_cte_m", "cruise_cte_m"),
                ("full_throttle_heading_deg", "cruise_heading_deg")):
            if resolved[cruise_key] <= resolved[full_key]:
                raise ValueError(f"autonomous.{cls.name}.{cruise_key} must exceed {full_key}")
        for key in ("throttle_rise_per_sec", "min_course_speed_mps", "course_max_age_sec",
                    "position_heading_max_age_sec", "forward_acquire_sec", "forward_memory_sec"):
            if resolved[key] <= 0.0:
                raise ValueError(f"autonomous.{cls.name}.{key} must be greater than zero")
        return resolved

    def reset(self):
        super().reset()
        self.suspend()

    def suspend(self):
        # The main controller centers the physical servo during a neutral hold.
        # Match that output before resuming the steering slew limiter.
        self.prev_steer_norm = 0.0
        self.prev_throttle_blend = 0.0
        self.last_update_mono = None

    @staticmethod
    def _tracking_score(error, full_limit, cruise_limit):
        return max(0.0, min(1.0, (cruise_limit - abs(error)) / (cruise_limit - full_limit)))

    def update(self, *, cte_right_m, heading_error_deg, boundary_guard, now_mono,
               heading_confident=False):
        heading_finite = heading_error_deg is not None and math.isfinite(heading_error_deg)
        command = super().update(
            cte_right_m=cte_right_m,
            heading_error_deg=heading_error_deg if heading_finite else None,
            boundary_guard=boundary_guard,
            now_mono=now_mono,
        )
        desired = 0.0
        if heading_confident and heading_finite and math.isfinite(cte_right_m) and not boundary_guard:
            desired = min(
                self._tracking_score(cte_right_m, self.options["full_throttle_cte_m"],
                                     self.options["cruise_cte_m"]),
                self._tracking_score(heading_error_deg, self.options["full_throttle_heading_deg"],
                                     self.options["cruise_heading_deg"]),
            )
        # At 20 Hz the normal step is 0.025. Cap elapsed time so a delayed loop
        # cannot skip the ramp. The first control update starts at cruise.
        dt = 0.0 if self.last_update_mono is None else max(0.0, min(0.1, now_mono - self.last_update_mono))
        blend = min(desired, self.prev_throttle_blend + self.options["throttle_rise_per_sec"] * dt)
        self.prev_throttle_blend = blend
        self.last_update_mono = now_mono
        command["throttle_blend"] = blend
        return command

    def describe_throttle(self, cruise_us, full_us):
        return (
            f"experimental GPS-course progressive: cruise {cruise_us} to full {full_us} us; "
            f"CTE {self.options['cruise_cte_m']:.2f}->{self.options['full_throttle_cte_m']:.2f}m, "
            f"heading {self.options['cruise_heading_deg']:.1f}->{self.options['full_throttle_heading_deg']:.1f}deg; "
            f"boost rise {self.options['throttle_rise_per_sec']:.2f}/s, immediate reductions"
        )


class PredictiveRecoveryStrategy(GpsCourseProgressiveStrategy):
    """Give recovery credit only to fresh motion that predicts convergence.

    The navigator checks the predicted motion against the route envelope and
    supplies path geometry. This constant-velocity forecast does not model hull
    dynamics, and its envelope is not a surveyed obstacle boundary.
    """

    name = "04_predictive_recovery"
    uses_predictive_guidance = True
    default_options = dict(
        GpsCourseProgressiveStrategy.default_options,
        lookahead_time_sec=1.8,
        lookahead_min_m=2.4,
        lookahead_max_m=5.0,
        prediction_horizon_sec=1.0,
        prediction_min_convergence_mps=0.05,
        prediction_max_path_turn_deg=15.0,
        prediction_corridor_margin_m=0.2,
    )
    option_limits = dict(
        GpsCourseProgressiveStrategy.option_limits,
        lookahead_time_sec=(0.0, None),
        lookahead_min_m=(0.0, None),
        lookahead_max_m=(0.0, None),
        prediction_horizon_sec=(0.0, None),
        prediction_min_convergence_mps=(0.0, None),
        prediction_max_path_turn_deg=(0.0, 180.0),
        prediction_corridor_margin_m=(0.0, None),
    )

    @classmethod
    def validate_options(cls, options):
        resolved = super().validate_options(options)
        for key in ("lookahead_time_sec", "lookahead_min_m", "lookahead_max_m",
                    "prediction_horizon_sec", "prediction_min_convergence_mps"):
            if resolved[key] <= 0:
                raise ValueError(f"autonomous.{cls.name}.{key} must be greater than zero")
        if resolved["lookahead_max_m"] < resolved["lookahead_min_m"]:
            raise ValueError(f"autonomous.{cls.name}.lookahead_max_m must be >= lookahead_min_m")
        return resolved

    @classmethod
    def validate_navigation_options(cls, nav_params, options):
        if options["prediction_corridor_margin_m"] >= nav_params.get("shift_cte_m", 0.70):
            raise ValueError(
                f"autonomous.{cls.name}.prediction_corridor_margin_m must be less than navigation shift_cte_m")

    @staticmethod
    def _finite_number(value):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return False
        try:
            return math.isfinite(value)
        except OverflowError:
            return False

    def lookahead_distance(self, speed_mps=None, boundary_guard=False):
        """Return bounded along-path lookahead; missing speed uses the fixed default."""
        if self._finite_number(speed_mps) and speed_mps >= 0:
            requested = speed_mps * self.options["lookahead_time_sec"]
        else:
            requested = self.nav_params.get("lookahead_dist_m", 3.5)
        distance = max(self.options["lookahead_min_m"], min(self.options["lookahead_max_m"], requested))
        if boundary_guard:
            distance = min(distance, self.nav_params.get("boundary_lookahead_m", 2.4))
        return distance

    def _valid_prediction(self, prediction):
        if not isinstance(prediction, dict):
            return False
        keys = ("speed_mps", "course_deg", "path_bearing_deg", "age_sec", "path_turn_deg")
        if not all(self._finite_number(prediction.get(key)) for key in keys):
            return False
        max_age = max(self.options["course_max_age_sec"], self.options["position_heading_max_age_sec"])
        return (prediction["speed_mps"] >= self.options["min_course_speed_mps"]
                and 0 <= prediction["course_deg"] < 360
                and 0 <= prediction["path_bearing_deg"] < 360
                and 0 <= prediction["age_sec"] <= max_age
                and 0 <= prediction["path_turn_deg"] <= 180)

    def update(self, *, cte_right_m, heading_error_deg, boundary_guard, now_mono,
               heading_confident=False, prediction=None):
        cte_valid = self._finite_number(cte_right_m)
        heading_valid = self._finite_number(heading_error_deg)
        context_valid = self._valid_prediction(prediction)
        effective_cte = abs(cte_right_m) if cte_valid else 0.0
        rate = future = None
        status = "prediction_missing" if prediction is None else "prediction_invalid"
        qualified = False

        if context_valid and cte_valid:
            rate = prediction["speed_mps"] * math.sin(math.radians(
                prediction["course_deg"] - prediction["path_bearing_deg"]))
            future = cte_right_m + self.options["prediction_horizon_sec"] * rate
            if not self._finite_number(future):
                rate = future = None
            else:
                overshoot = (cte_right_m * future < 0
                             and abs(future) > self.options["full_throttle_cte_m"])
                bend = prediction["path_turn_deg"] > self.options["prediction_max_path_turn_deg"]
                converging = (cte_right_m * rate < 0
                              and abs(rate) >= self.options["prediction_min_convergence_mps"]
                              and abs(future) < abs(cte_right_m))
                recovery_credit = converging and not overshoot and not bend
                effective_cte = abs(future) if recovery_credit else max(abs(cte_right_m), abs(future))
                qualified = True
                if boundary_guard:
                    status, qualified = "boundary_guard", False
                elif not heading_confident or not heading_valid:
                    status, qualified = "heading_untrusted", False
                elif prediction.get("corridor_clear") is not True:
                    status, qualified = "outside_corridor", False
                elif abs(cte_right_m) >= self.options["cruise_cte_m"]:
                    status, qualified = "outside_tracking_envelope", False
                elif overshoot:
                    status = "overshoot"
                elif bend:
                    status = "bend"
                elif recovery_credit:
                    status = "recovering"
                elif abs(future) > abs(cte_right_m):
                    status = "diverging"
                else:
                    status = "tracking"

        desired = 0.0
        if qualified:
            desired = min(
                self._tracking_score(effective_cte, self.options["full_throttle_cte_m"],
                                     self.options["cruise_cte_m"]),
                self._tracking_score(heading_error_deg, self.options["full_throttle_heading_deg"],
                                     self.options["cruise_heading_deg"]),
            )
        command = super().update(
            cte_right_m=effective_cte,
            heading_error_deg=heading_error_deg if heading_valid else None,
            boundary_guard=boundary_guard,
            now_mono=now_mono,
            heading_confident=qualified,
        )
        command.update(predicted_cte_m=future, cte_rate_mps=rate, prediction_status=status,
                       throttle_target_blend=desired)
        return command

    def describe_throttle(self, cruise_us, full_us):
        return (
            f"experimental predictive recovery: cruise {cruise_us} to full {full_us} us; "
            f"forecast {self.options['prediction_horizon_sec']:.2f}s, "
            f"lookahead {self.options['lookahead_min_m']:.2f}-{self.options['lookahead_max_m']:.2f}m "
            f"at {self.options['lookahead_time_sec']:.2f}s of travel; "
            f"boost rise {self.options['throttle_rise_per_sec']:.2f}/s, immediate reductions"
        )


class StraightLineStrategy(ConstantCruiseStrategy):
    """Race line: one straight line from the AUTO entry point, full throttle, GPS-course PI steering.

    The navigator draws a line parallel to the course through the entry point,
    moved toward the centerline when the entry is near a bank, and passes the
    line-of-sight course error. Course over ground already includes current,
    so the outer loop has no integral; the integral here acts on course error
    and removes rudder trim. It is frozen while launching, slow, without a
    trusted course, near a bank, or far from the commanded course.
    """

    name = "05_straight_line"
    uses_gps_course = True
    uses_straight_line = True
    default_options = {
        "min_course_speed_mps": 0.45,
        "course_max_age_sec": 0.6,
        "position_heading_max_age_sec": 0.8,
        "forward_acquire_sec": 0.6,
        "forward_memory_sec": 4.0,
        "band_half_width_m": 2.5,
        "line_max_offset_m": 1.5,
        "guard_zone_m": 0.5,
        "max_intercept_deg": 15.0,
        "recovery_intercept_deg": 40.0,
        "lookahead_time_sec": 1.8,
        "lookahead_min_m": 2.4,
        "lookahead_max_m": 5.0,
        "prediction_horizon_sec": 1.0,
        "course_gain": 1.0,
        "course_integral_gain": 0.3,
        "integral_limit_deg": 20.0,
        "integral_delay_sec": 2.0,
        "integral_max_error_deg": 20.0,
        # Test pattern: a rectangle on the bank lines (+-band_half_width_m), then on the line limits
        # (+-line_max_offset_m), to compare where the code puts them with the real water.
        "test_square": False,
        "test_leg_length_m": 15.0,
        "test_lead_m": 3.0,
        "test_throttle": 0.6,
        "test_accept_m": 1.0,
    }
    option_limits = dict(
        {key: (0.0, 90.0 if key.endswith("_deg") else None)
         for key, value in default_options.items() if not isinstance(value, bool)},
        test_throttle=(0.1, 1.0),
    )

    @classmethod
    def validate_options(cls, options):
        resolved = super().validate_options(options)
        for key in ("min_course_speed_mps", "course_max_age_sec", "position_heading_max_age_sec",
                    "forward_acquire_sec", "forward_memory_sec", "band_half_width_m", "max_intercept_deg",
                    "recovery_intercept_deg", "lookahead_time_sec", "lookahead_min_m", "lookahead_max_m",
                    "prediction_horizon_sec", "course_gain"):
            if resolved[key] <= 0.0:
                raise ValueError(f"autonomous.{cls.name}.{key} must be greater than zero")
        if resolved["line_max_offset_m"] + resolved["guard_zone_m"] >= resolved["band_half_width_m"]:
            raise ValueError(f"autonomous.{cls.name}.line_max_offset_m + guard_zone_m must be inside band_half_width_m")
        if resolved["lookahead_max_m"] < resolved["lookahead_min_m"]:
            raise ValueError(f"autonomous.{cls.name}.lookahead_max_m must be >= lookahead_min_m")
        if resolved["recovery_intercept_deg"] < resolved["max_intercept_deg"]:
            raise ValueError(f"autonomous.{cls.name}.recovery_intercept_deg must be >= max_intercept_deg")
        if not isinstance(resolved["test_square"], bool):
            raise ValueError(f"autonomous.{cls.name}.test_square must be true or false")
        for key in ("test_leg_length_m", "test_accept_m"):
            if resolved[key] <= 0.0:
                raise ValueError(f"autonomous.{cls.name}.{key} must be greater than zero")
        return resolved

    def reset(self):
        super().reset()
        self.integral_deg_s = 0.0
        self.last_update_mono = None
        self.entry_mono = None

    def suspend(self):
        # The servo is centered during a neutral hold; keep the learned trim.
        self.prev_steer_norm = 0.0
        self.last_update_mono = None

    def lookahead_distance(self, speed_mps=None):
        """Speed-dependent line-of-sight distance; missing speed uses the fixed default."""
        if (isinstance(speed_mps, (int, float)) and not isinstance(speed_mps, bool)
                and math.isfinite(speed_mps) and speed_mps >= 0):
            requested = speed_mps * self.options["lookahead_time_sec"]
        else:
            requested = self.nav_params.get("lookahead_dist_m", 3.5)
        return max(self.options["lookahead_min_m"], min(self.options["lookahead_max_m"], requested))

    def update(self, *, cte_right_m, heading_error_deg, boundary_guard, now_mono,
               heading_confident=False, speed_mps=None):
        if self.entry_mono is None:
            self.entry_mono = now_mono
        dt = 0.0 if self.last_update_mono is None else max(0.0, min(0.2, now_mono - self.last_update_mono))
        self.last_update_mono = now_mono
        ki = self.options["course_integral_gain"]
        valid = (heading_error_deg is not None and not isinstance(heading_error_deg, bool)
                 and math.isfinite(heading_error_deg))
        fast = (isinstance(speed_mps, (int, float)) and math.isfinite(speed_mps)
                and speed_mps >= self.options["min_course_speed_mps"])
        if (ki > 0 and valid and heading_confident and fast and not boundary_guard
                and now_mono - self.entry_mono >= self.options["integral_delay_sec"]
                and abs(heading_error_deg) <= self.options["integral_max_error_deg"]):
            limit = self.options["integral_limit_deg"] / ki
            self.integral_deg_s = max(-limit, min(limit, self.integral_deg_s + heading_error_deg * dt))
        trim_deg = ki * self.integral_deg_s
        # Without a course the rudder holds only the learned trim (zero until it has been learned).
        effective = self.options["course_gain"] * heading_error_deg + trim_deg if valid else trim_deg
        command = super().update(cte_right_m=cte_right_m, heading_error_deg=effective,
                                 boundary_guard=boundary_guard, now_mono=now_mono)
        command["trim_deg"] = trim_deg
        return command

    side_limit_labels = False

    def limits(self):
        """(bank_left, bank_right, line_left, line_right): distances from the centerline."""
        o = self.options
        return o["band_half_width_m"], o["band_half_width_m"], o["line_max_offset_m"], o["line_max_offset_m"]

    def throttle_blend(self, cte_right_m, heading_error_deg):
        return 1.0

    def describe_throttle(self, cruise_us, full_us):
        o = self.options
        return (
            f"full {full_us} us whenever AUTO is steering; straight line from AUTO entry "
            f"(within +-{o['line_max_offset_m']:.1f} m of the centerline, band +-{o['band_half_width_m']:.1f} m); "
            f"course PI, integral {o['course_integral_gain']:.2f}/s"
            + (f"; TEST SQUARE at {100 * o['test_throttle']:.0f}% throttle" if o["test_square"] else "")
        )


class GpsAwareStrategy(StraightLineStrategy):
    """Straight-line steering with GPS-qualified, absolute power from GpsAwareController.

    This low-level strategy cannot authorize propulsion. The guard supplies
    accepted observations, marks only qualified FIX headings confident for trim
    learning, and replaces the neutral throttle command after applying its caps.
    """

    name = "06_gps_aware"
    uses_gps_guard = True
    guard_default_options = {
        "position_max_age_sec": 0.6,
        "accuracy_max_age_sec": 1.0,
        "accuracy_max_skew_sec": 0.25,
        "retain_stale_accuracy": True,
        "race_throttle": 1.0,
        "float_throttle": 0.7,
        "dgps_throttle": 0.5,
        "gps_throttle": 0.3,
        "boundary_throttle": 0.35,
        "throttle_rise_per_sec": 0.5,
        "qualification_sec": 2.0,
        "qualification_fixes": 5,
        "max_hdop": 2.5,
        "min_satellites": 8,
        "max_correction_age_sec": 10.0,
        "fixed_uncertainty_m": 0.25,
        "float_uncertainty_m": 0.75,
        "dgps_uncertainty_m": 1.25,
        "gps_uncertainty_m": 1.5,
        "accuracy_multiplier": 2.0,
        "uncertainty_growth_mps": 0.5,
        "hull_margin_m": 0.25,
        "min_clearance_m": 0.25,
        "race_clearance_m": 0.75,
        "race_max_heading_error_deg": 20.0,
        "race_max_cte_m": 0.5,
        "jump_tolerance_m": 1.0,
        "innovation_growth_mps": 0.5,
        "max_speed_mps": 4.0,
        "max_source_gap_sec": 2.0,
        "source_clock_tolerance_sec": 0.5,
        "max_reacquire_shift_m": 3.0,
        "launch_throttle": 0.3,
        "launch_timeout_sec": 1.5,
        "launch_min_clearance_m": 1.0,
        "degraded_launch_min_clearance_m": 0.5,
        "launch_heading_memory_sec": 5.0,
        "auto_resume": True,
    }
    default_options = dict(StraightLineStrategy.default_options, **guard_default_options)
    option_limits = dict(StraightLineStrategy.option_limits, **{
        key: (0.0, 1.0 if key.endswith("_throttle") else
              180.0 if key == "race_max_heading_error_deg" else None)
        for key, value in guard_default_options.items() if not isinstance(value, bool)
    })

    @classmethod
    def validate_options(cls, options):
        resolved = super().validate_options(options)
        return GpsAwareStrategy.validate_guard_options(cls.name, resolved, resolved["band_half_width_m"],
                                                       "band_half_width_m")

    @staticmethod
    def validate_guard_options(name, resolved, narrowest_bank_m, bank_key):
        """GPS-guard option checks shared by 06 and 08; ``narrowest_bank_m`` is the closest bank limit."""
        if resolved["test_square"]:
            raise ValueError(f"autonomous.{name}.test_square is unsupported; use 05_straight_line")
        for key in ("auto_resume", "retain_stale_accuracy"):
            if not isinstance(resolved[key], bool):
                raise ValueError(f"autonomous.{name}.{key} must be true or false")
        for key, minimum in (("qualification_fixes", 2), ("min_satellites", 4)):
            if not isinstance(resolved[key], int) or isinstance(resolved[key], bool) or resolved[key] < minimum:
                raise ValueError(f"autonomous.{name}.{key} must be an integer >= {minimum}")
        for key in ("position_max_age_sec", "accuracy_max_age_sec", "accuracy_max_skew_sec",
                    "throttle_rise_per_sec", "qualification_sec", "max_hdop", "max_correction_age_sec",
                    "fixed_uncertainty_m", "accuracy_multiplier", "uncertainty_growth_mps",
                    "jump_tolerance_m", "innovation_growth_mps", "max_speed_mps", "max_source_gap_sec",
                    "source_clock_tolerance_sec", "max_reacquire_shift_m", "launch_timeout_sec",
                    "launch_heading_memory_sec"):
            if resolved[key] <= 0.0:
                raise ValueError(f"autonomous.{name}.{key} must be greater than zero")
        for smaller, larger in (
                ("gps_throttle", "dgps_throttle"), ("dgps_throttle", "float_throttle"),
                ("float_throttle", "race_throttle"), ("boundary_throttle", "race_throttle"),
                ("launch_throttle", "float_throttle"),
                ("fixed_uncertainty_m", "float_uncertainty_m"),
                ("float_uncertainty_m", "dgps_uncertainty_m"),
                ("dgps_uncertainty_m", "gps_uncertainty_m"),
                ("accuracy_max_skew_sec", "accuracy_max_age_sec"),
                ("position_max_age_sec", "max_source_gap_sec"),
                ("min_clearance_m", "race_clearance_m"),
                ("min_clearance_m", "launch_min_clearance_m"),
                ("min_clearance_m", "degraded_launch_min_clearance_m"),
                ("forward_acquire_sec", "launch_timeout_sec")):
            if resolved[smaller] > resolved[larger]:
                raise ValueError(f"autonomous.{name}.{larger} must be >= {smaller}")
        if resolved["max_speed_mps"] <= resolved["min_course_speed_mps"]:
            raise ValueError(f"autonomous.{name}.max_speed_mps must exceed min_course_speed_mps")
        for key in ("race_clearance_m", "launch_min_clearance_m"):
            if resolved[key] + resolved["fixed_uncertainty_m"] + resolved["hull_margin_m"] >= narrowest_bank_m:
                raise ValueError(f"autonomous.{name}.{key} plus FIX uncertainty and hull margin must fit inside {bank_key}")
        if resolved["race_max_cte_m"] >= narrowest_bank_m:
            raise ValueError(f"autonomous.{name}.race_max_cte_m must be inside {bank_key}")
        return resolved

    def update(self, **kwargs):
        command = super().update(**kwargs)
        command["throttle_blend"] = 0.0
        command["throttle_norm"] = 0.0
        return command

    def throttle_blend(self, cte_right_m, heading_error_deg):
        return 0.0

    def describe_throttle(self, cruise_us, full_us):
        return (f"GPS-guarded straight line; up to {100 * self.options['race_throttle']:.0f}% forward "
                f"({full_us} us maximum) only after FIX qualification; FLOAT <= {100 * self.options['float_throttle']:.0f}%, "
                f"DGPS <= {100 * self.options['dgps_throttle']:.0f}% when clearance permits; "
                f"GPS <= {100 * self.options['gps_throttle']:.0f}%; "
                "otherwise neutral; automatic stuck reverse disabled")


class TurnPlannerStrategy(StraightLineStrategy):
    """05's straight race line plus a turn planner and separate left/right limits.

    When the course is far from the race direction (entry, takeover, collision), the navigator works out both
    full-lock turns with measured radii and picks the one that stays inside the bank limits in the shortest arc;
    if neither fits and the bow points toward the limit it would cross, it reverses with the rudder centred until
    a turn fits, then turns at once. The line is drawn where the turn onto the race direction ends. Built after
    the 2026-11-17 18:39 starts, where turning the short way round drove the boat toward the bank.
    """

    name = "07_turn_planner"
    side_limit_labels = True
    default_options = dict(
        {key: value for key, value in StraightLineStrategy.default_options.items()
         if key not in ("band_half_width_m", "line_max_offset_m")},
        # Distances from the centerline. Banks measured from RTK bank hits (bank_mapping): left -4.20 m,
        # right +5.11 m; the bank limits keep about 1 m of water, the line limits sit 0.6 m inside them.
        bank_left_m=3.2,
        bank_right_m=4.0,
        line_left_m=2.6,
        line_right_m=3.4,
        # Radii measured from full-throttle full-lock turns on 2026-11-16/17 (right 1.9-2.2 m, left 2.1-2.3 m on
        # 11-17, 2.9-3.5 m on 11-16); turn_delay_sec covers steering and GPS delay.
        turn_planner=True,
        turn_plan_deg=60.0,
        turn_exit_deg=25.0,
        turn_radius_left_m=3.0,
        turn_radius_right_m=2.3,
        turn_delay_sec=0.4,
        turn_tolerance_m=0.3,
        turn_restart_lead_m=0.2,
        plan_neutral_sec=0.3,
        plan_reverse_max_sec=4.0,
        plan_reverse_throttle=0.5,
        plan_max_reverses=2.0,
        # Three-point turns: choose the fastest of turn right / turn left / reverse-then-turn by estimated time,
        # even when a long forward turn fits (e.g. a 254 deg circle vs a short reverse and a 106 deg turn).
        three_point_turns=False,
        plan_cruise_mps=1.6,
        plan_reverse_mps=0.5,
        plan_restart_sec=0.6,
    )
    option_limits = dict(
        {key: (0.0, 90.0 if key.endswith("_deg") else None)
         for key, value in default_options.items() if not isinstance(value, bool)},
        test_throttle=(0.1, 1.0),
        plan_reverse_throttle=(0.1, 1.0),
        plan_max_reverses=(0.0, 5.0),
    )

    @classmethod
    def validate_options(cls, options):
        # Skip 05's symmetric-limit checks; the generic number checks still apply.
        resolved = super(StraightLineStrategy, cls).validate_options(options)
        for key in ("min_course_speed_mps", "course_max_age_sec", "position_heading_max_age_sec",
                    "forward_acquire_sec", "forward_memory_sec", "bank_left_m", "bank_right_m", "max_intercept_deg",
                    "recovery_intercept_deg", "lookahead_time_sec", "lookahead_min_m", "lookahead_max_m",
                    "prediction_horizon_sec", "course_gain", "turn_plan_deg", "turn_radius_left_m",
                    "turn_radius_right_m", "plan_reverse_max_sec", "test_leg_length_m", "test_accept_m",
                    "plan_cruise_mps", "plan_reverse_mps"):
            if resolved[key] <= 0.0:
                raise ValueError(f"autonomous.{cls.name}.{key} must be greater than zero")
        for side in ("left", "right"):
            if resolved[f"line_{side}_m"] + resolved["guard_zone_m"] >= resolved[f"bank_{side}_m"]:
                raise ValueError(f"autonomous.{cls.name}.line_{side}_m + guard_zone_m must be inside bank_{side}_m")
        if resolved["lookahead_max_m"] < resolved["lookahead_min_m"]:
            raise ValueError(f"autonomous.{cls.name}.lookahead_max_m must be >= lookahead_min_m")
        if resolved["recovery_intercept_deg"] < resolved["max_intercept_deg"]:
            raise ValueError(f"autonomous.{cls.name}.recovery_intercept_deg must be >= max_intercept_deg")
        if resolved["turn_exit_deg"] >= resolved["turn_plan_deg"]:
            raise ValueError(f"autonomous.{cls.name}.turn_exit_deg must be less than turn_plan_deg")
        if not float(resolved["plan_max_reverses"]).is_integer():
            raise ValueError(f"autonomous.{cls.name}.plan_max_reverses must be a whole number")
        for key in ("test_square", "turn_planner", "three_point_turns"):
            if not isinstance(resolved[key], bool):
                raise ValueError(f"autonomous.{cls.name}.{key} must be true or false")
        return resolved

    def limits(self):
        o = self.options
        return o["bank_left_m"], o["bank_right_m"], o["line_left_m"], o["line_right_m"]

    def describe_throttle(self, cruise_us, full_us):
        o = self.options
        return (
            f"full {full_us} us whenever AUTO is steering; straight line from AUTO entry "
            f"(line within -{o['line_left_m']:.1f}/+{o['line_right_m']:.1f} m of the centerline, "
            f"bank limits -{o['bank_left_m']:.1f}/+{o['bank_right_m']:.1f} m); "
            + ("turn planner on" + (" with three-point turns; " if o["three_point_turns"] else "; ")
               if o["turn_planner"] else "turn planner off; ")
            + f"course PI, integral {o['course_integral_gain']:.2f}/s"
            + (f"; TEST SQUARE at {100 * o['test_throttle']:.0f}% throttle" if o["test_square"] else "")
        )


class GuardedTurnPlannerStrategy(TurnPlannerStrategy):
    """07's turn planner and per-side limits behind 06's GPS guard.

    The guard (guarded_planner.GuardedPlannerController) accepts only fresh, consistent positions and sets an
    absolute power cap from GPS quality; within that, 07's planner chooses turns and reverses. Planned turns run
    at full power on qualified RTK FIX (the measured turn radii are for full throttle), and the planner may
    reverse only on qualified FIX. The line limits are narrower than 07's so that the line keeps the guard's
    race clearance after FIX uncertainty and hull margin (full power needs race_clearance_m).
    """

    name = "08_guarded_turn_planner"
    uses_gps_guard = True
    guard_with_planner = True
    default_options = dict(
        TurnPlannerStrategy.default_options,
        # Bank limits as 07. Full power needs bank - |d| - fixed_uncertainty - hull_margin >= race_clearance
        # (3.2/4.0 - 0.25 - 0.25 - 0.75), so the line stays within -1.95/+2.75 m; 0.25 m is kept spare.
        line_left_m=1.7,
        line_right_m=2.5,
        **GpsAwareStrategy.guard_default_options
    )
    option_limits = dict(TurnPlannerStrategy.option_limits, **{
        key: (0.0, 1.0 if key.endswith("_throttle") else
              180.0 if key == "race_max_heading_error_deg" else None)
        for key, value in GpsAwareStrategy.guard_default_options.items() if not isinstance(value, bool)
    })

    @classmethod
    def validate_options(cls, options):
        resolved = super().validate_options(options)
        for key in ("auto_resume", "retain_stale_accuracy"):
            if not isinstance(resolved[key], bool):
                raise ValueError(f"autonomous.{cls.name}.{key} must be true or false")
        return GpsAwareStrategy.validate_guard_options(
            cls.name, resolved, min(resolved["bank_left_m"], resolved["bank_right_m"]), "the nearer bank limit")

    def describe_throttle(self, cruise_us, full_us):
        o = self.options
        return (
            f"GPS-guarded straight line with the 07 turn planner; up to {100 * o['race_throttle']:.0f}% forward "
            f"({full_us} us maximum) only after FIX qualification (planned turns too); "
            f"FLOAT <= {100 * o['float_throttle']:.0f}%, DGPS <= {100 * o['dgps_throttle']:.0f}%, "
            f"GPS <= {100 * o['gps_throttle']:.0f}%; line within -{o['line_left_m']:.1f}/+{o['line_right_m']:.1f} m, "
            f"bank limits -{o['bank_left_m']:.1f}/+{o['bank_right_m']:.1f} m; "
            + ("turn planner on" + (" with three-point turns" if o["three_point_turns"] else "")
               if o["turn_planner"] else "turn planner off")
            + "; planner reverse only on qualified FIX; automatic stuck reverse disabled"
        )


# Numbers are permanent creation-order identifiers. Append 05, 06, ... for new
# strategies; never renumber existing modes when adding or retiring one.
AUTO_STRATEGIES = {
    cls.name: cls for cls in (ConstantCruiseStrategy, AdaptiveThrottleStrategy, GpsCourseProgressiveStrategy,
                            PredictiveRecoveryStrategy, StraightLineStrategy, GpsAwareStrategy, TurnPlannerStrategy,
                            GuardedTurnPlannerStrategy)
}

AUTO_STRATEGY_ALIASES = {
    "constant_cruise": ConstantCruiseStrategy.name,
    "adaptive_throttle": AdaptiveThrottleStrategy.name,
    "gps_course_progressive": GpsCourseProgressiveStrategy.name,
    "predictive_recovery": PredictiveRecoveryStrategy.name,
    "straight_line": StraightLineStrategy.name,
    "gps_aware": GpsAwareStrategy.name,
    "turn_planner": TurnPlannerStrategy.name,
    "guarded_turn_planner": GuardedTurnPlannerStrategy.name,
}


def canonical_auto_strategy_name(name):
    """Accept historical names while reporting one stable numbered identifier."""
    if isinstance(name, str):
        canonical = AUTO_STRATEGY_ALIASES.get(name, name)
        if canonical in AUTO_STRATEGIES:
            return canonical
    raise ValueError(f"Unknown autonomous strategy {name!r}; choose {', '.join(AUTO_STRATEGIES)}")


# Settings in the autonomous section that are not strategy option blocks.
#   use_04_control: true runs 04_predictive_recovery whatever "strategy" selects (quick way back from 05).
#   recovery: stuck-recovery settings (recovery.py), bypassed by GPS-guarded mode 06.
FALLBACK_AUTO_STRATEGY = PredictiveRecoveryStrategy.name
RUN_SETTINGS = ("use_04_control", "recovery")


def normalize_autonomous_config(config):
    """Rename selectors and option blocks without mutating settings or choosing between duplicates."""
    if not isinstance(config, dict):
        raise ValueError("autonomous must be a JSON object")
    normalized = {"strategy": canonical_auto_strategy_name(config.get("strategy", DEFAULT_AUTO_STRATEGY))}
    for name, options in config.items():
        if name == "strategy":
            continue
        if name in RUN_SETTINGS:
            normalized[name] = options
            continue
        if name not in AUTO_STRATEGIES and name not in AUTO_STRATEGY_ALIASES:
            raise ValueError(f"Unknown autonomous setting {name!r}")
        canonical = canonical_auto_strategy_name(name)
        if canonical in normalized:
            raise ValueError(f"Duplicate autonomous options for {canonical}; use one numbered or legacy block")
        normalized[canonical] = options
    return normalized


def default_autonomous_config():
    """Return independent defaults for the controller and calibration tool."""
    return {
        "strategy": DEFAULT_AUTO_STRATEGY,
        "use_04_control": False,
        "recovery": dict(DEFAULT_RECOVERY),
        **{name: dict(cls.default_options) for name, cls in AUTO_STRATEGIES.items() if cls.default_options},
    }


def apply_run_overrides(config, strategy=None, use_04=False, line_limit_m=None, bank_limit_m=None,
                        test_square=None, recovery=None, line_left_m=None, line_right_m=None,
                        bank_left_m=None, bank_right_m=None, turn_planner=None, three_point_turns=None):
    """Return a copy of the autonomous section with command-line choices applied (None keeps the JSON).

    The line/bank limit and test-square choices go to the selected strategy's block when that is 06, 07 or 08,
    otherwise to 05's.
    line_limit_m / bank_limit_m set both sides; the per-side values, turn_planner and three_point_turns exist
    only in 07 and 08.
    """
    config = normalize_autonomous_config(config)
    if strategy is not None:
        config["strategy"] = canonical_auto_strategy_name(strategy)
    if use_04:
        config["use_04_control"] = True
    per_side = {"line_left_m": line_left_m, "line_right_m": line_right_m, "bank_left_m": bank_left_m,
                "bank_right_m": bank_right_m, "turn_planner": turn_planner, "three_point_turns": three_point_turns}
    if config["strategy"] in (TurnPlannerStrategy.name, GuardedTurnPlannerStrategy.name):
        target = config["strategy"]
        line_options = {"line_left_m": line_limit_m, "line_right_m": line_limit_m, "bank_left_m": bank_limit_m,
                        "bank_right_m": bank_limit_m, "test_square": test_square}
        line_options.update({key: value for key, value in per_side.items() if value is not None})
    else:
        if any(value is not None for value in per_side.values()):
            raise ValueError("per-side limits, the turn planner switch and three-point turns apply to "
                             f"{TurnPlannerStrategy.name} and {GuardedTurnPlannerStrategy.name} only "
                             "(select one with --auto-strategy)")
        target = GpsAwareStrategy.name if config["strategy"] == GpsAwareStrategy.name else StraightLineStrategy.name
        line_options = {"line_max_offset_m": line_limit_m, "band_half_width_m": bank_limit_m,
                        "test_square": test_square}
    line_options = {key: value for key, value in line_options.items() if value is not None}
    if line_options:
        config[target] = dict(config.get(target, {}), **line_options)
    if recovery is not None:
        config["recovery"] = dict(config.get("recovery") or {}, enabled=recovery)
    return config


def resolve_recovery_config(config):
    """Validated stuck-recovery settings from the autonomous section."""
    return resolve_recovery_options(normalize_autonomous_config(config).get("recovery"))


def create_auto_strategy(config, nav_params):
    """Validate configuration before hardware initialization; omitted settings use defaults."""
    config = normalize_autonomous_config(config)
    use_04 = config.get("use_04_control", False)
    if not isinstance(use_04, bool):
        raise ValueError("autonomous.use_04_control must be true or false")
    resolve_recovery_options(config.get("recovery"))
    name = FALLBACK_AUTO_STRATEGY if use_04 else config["strategy"]
    for configured_name, options in config.items():
        if configured_name not in ("strategy",) + RUN_SETTINGS:
            strategy_type = AUTO_STRATEGIES[configured_name]
            resolved = strategy_type.validate_options(options)
            strategy_type.validate_navigation_options(nav_params, resolved)
    return AUTO_STRATEGIES[name](nav_params, config.get(name, {}))
