"""Selectable AUTO control laws. Navigation and hardware safety stay in the controller."""

import math


DEFAULT_AUTO_STRATEGY = "adaptive_throttle"


class ConstantCruiseStrategy:
    """Original main behavior: proportional heading steering and constant cruise."""

    name = "constant_cruise"
    uses_gps_course = False
    default_options = {}
    option_limits = {}

    def __init__(self, nav_params, options=None):
        self.nav_params = dict(nav_params)
        self.options = self.validate_options({} if options is None else options)
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

    name = "adaptive_throttle"
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

    name = "gps_course_progressive"
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


# Register new control strategies here; the main loop does not need strategy branches.
AUTO_STRATEGIES = {
    cls.name: cls for cls in (ConstantCruiseStrategy, AdaptiveThrottleStrategy, GpsCourseProgressiveStrategy)
}


def default_autonomous_config():
    """Return independent defaults for the controller and calibration tool."""
    return {
        "strategy": DEFAULT_AUTO_STRATEGY,
        **{name: dict(cls.default_options) for name, cls in AUTO_STRATEGIES.items() if cls.default_options},
    }


def create_auto_strategy(config, nav_params):
    """Validate configuration before hardware initialization; omitted settings use defaults."""
    if not isinstance(config, dict):
        raise ValueError("autonomous must be a JSON object")
    unknown = config.keys() - ({"strategy"} | AUTO_STRATEGIES.keys())
    if unknown:
        raise ValueError(f"Unknown autonomous setting(s): {', '.join(sorted(unknown))}")
    name = config.get("strategy", DEFAULT_AUTO_STRATEGY)
    if not isinstance(name, str) or name not in AUTO_STRATEGIES:
        raise ValueError(f"Unknown autonomous strategy {name!r}; choose {', '.join(AUTO_STRATEGIES)}")
    for configured_name, options in config.items():
        if configured_name != "strategy":
            AUTO_STRATEGIES[configured_name].validate_options(options)
    return AUTO_STRATEGIES[name](nav_params, config.get(name, {}))
