"""Selectable AUTO control laws. Navigation and hardware safety stay in the controller."""

import math


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


# Numbers are permanent creation-order identifiers. Append 05, 06, ... for new
# strategies; never renumber existing modes when adding or retiring one.
AUTO_STRATEGIES = {
    cls.name: cls for cls in (ConstantCruiseStrategy, AdaptiveThrottleStrategy, GpsCourseProgressiveStrategy,
                            PredictiveRecoveryStrategy)
}

AUTO_STRATEGY_ALIASES = {
    "constant_cruise": ConstantCruiseStrategy.name,
    "adaptive_throttle": AdaptiveThrottleStrategy.name,
    "gps_course_progressive": GpsCourseProgressiveStrategy.name,
    "predictive_recovery": PredictiveRecoveryStrategy.name,
}


def canonical_auto_strategy_name(name):
    """Accept historical names while reporting one stable numbered identifier."""
    if isinstance(name, str):
        canonical = AUTO_STRATEGY_ALIASES.get(name, name)
        if canonical in AUTO_STRATEGIES:
            return canonical
    raise ValueError(f"Unknown autonomous strategy {name!r}; choose {', '.join(AUTO_STRATEGIES)}")


def normalize_autonomous_config(config):
    """Rename selectors and option blocks without mutating settings or choosing between duplicates."""
    if not isinstance(config, dict):
        raise ValueError("autonomous must be a JSON object")
    normalized = {"strategy": canonical_auto_strategy_name(config.get("strategy", DEFAULT_AUTO_STRATEGY))}
    for name, options in config.items():
        if name == "strategy":
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
        **{name: dict(cls.default_options) for name, cls in AUTO_STRATEGIES.items() if cls.default_options},
    }


def create_auto_strategy(config, nav_params):
    """Validate configuration before hardware initialization; omitted settings use defaults."""
    config = normalize_autonomous_config(config)
    name = config["strategy"]
    for configured_name, options in config.items():
        if configured_name != "strategy":
            strategy_type = AUTO_STRATEGIES[configured_name]
            resolved = strategy_type.validate_options(options)
            strategy_type.validate_navigation_options(nav_params, resolved)
    return AUTO_STRATEGIES[name](nav_params, config.get(name, {}))
