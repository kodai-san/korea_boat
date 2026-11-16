"""Selectable AUTO control laws. Navigation and hardware safety stay in the controller."""

import math


DEFAULT_AUTO_STRATEGY = "adaptive_throttle"


class ConstantCruiseStrategy:
    """Original main behavior: proportional heading steering and constant cruise."""

    name = "constant_cruise"
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

    def update(self, *, cte_right_m, heading_error_deg, boundary_guard, now_mono):
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


# Register new control strategies here; the main loop does not need strategy branches.
AUTO_STRATEGIES = {
    cls.name: cls for cls in (ConstantCruiseStrategy, AdaptiveThrottleStrategy)
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
