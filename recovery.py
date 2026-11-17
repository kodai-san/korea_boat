"""Stuck recovery for AUTO: back off when the boat is driven forward but does not move.

Pure timing logic with no hardware access. The main loop calls update() every AUTO
cycle before navigation and applies the returned throttle with the rudder centered.
"""

import math

DEFAULT_RECOVERY = {
    "enabled": True,             # Turn the feature on or off
    "stuck_speed_mps": 0.15,     # GPS speed below this while driving forward counts as stuck
    "stuck_sec": 1.5,            # ...for this long
    "startup_grace_sec": 3.0,    # No detection this long after AUTO entry or a recovery (accelerating)
    "neutral_sec": 0.3,          # Neutral before and after reversing (many ESCs need it to change direction)
    "reverse_sec": 2.0,          # Reverse duration
    "reverse_throttle": 0.6,     # Fraction of full reverse (pwm.esc_rev_max_us)
    "max_attempts": 3,           # Then stop in neutral and return to MANUAL
}

_LIMITS = {
    "stuck_speed_mps": (0.0, 1.0),
    "stuck_sec": (0.2, 30.0),
    "startup_grace_sec": (0.0, 30.0),
    "neutral_sec": (0.0, 5.0),
    "reverse_sec": (0.2, 10.0),
    "reverse_throttle": (0.05, 1.0),
    "max_attempts": (1, 20),
}


def resolve_recovery_options(options=None):
    """Merge with defaults and validate; raise ValueError for unknown or out-of-range settings."""
    if options is None:
        options = {}
    if not isinstance(options, dict):
        raise ValueError("autonomous.recovery must be a JSON object")
    unknown = options.keys() - DEFAULT_RECOVERY.keys()
    if unknown:
        raise ValueError(f"Unknown autonomous.recovery option(s): {', '.join(sorted(unknown))}")
    resolved = dict(DEFAULT_RECOVERY, **options)
    if not isinstance(resolved["enabled"], bool):
        raise ValueError("autonomous.recovery.enabled must be true or false")
    for key, (minimum, maximum) in _LIMITS.items():
        value = resolved[key]
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                or not minimum <= value <= maximum):
            raise ValueError(f"autonomous.recovery.{key} must be a number from {minimum} to {maximum}")
    if not float(resolved["max_attempts"]).is_integer():
        raise ValueError("autonomous.recovery.max_attempts must be a whole number")
    resolved["max_attempts"] = int(resolved["max_attempts"])
    return resolved


class StuckRecovery:
    """neutral -> reverse -> neutral, then normal control again; give up after max_attempts per AUTO run."""

    PHASES = ("neutral_in", "reverse", "neutral_out")

    def __init__(self, options=None):
        self.options = resolve_recovery_options(options)
        self.reset(0.0)

    @property
    def enabled(self):
        return self.options["enabled"]

    def reset(self, now):
        """Call on every AUTO entry."""
        self.attempts = 0
        self.phase = None
        self.phase_end = None
        self.slow_since = None
        self.gave_up = False
        self.grace_until = now + self.options["startup_grace_sec"]

    def _start(self, phase, now):
        self.phase = phase
        duration = self.options["reverse_sec"] if phase == "reverse" else self.options["neutral_sec"]
        self.phase_end = now + duration

    def _command(self):
        if self.phase == "reverse":
            return {"state": "reverse", "throttle_norm": -self.options["reverse_throttle"]}
        return {"state": self.phase, "throttle_norm": 0.0}

    def update(self, now, forward_commanded, speed_mps):
        """Return None for normal control, or {"state", "throttle_norm"} to apply with the rudder centered.

        state "gave_up" means stop in neutral and return to MANUAL. speed_mps is GPS speed over ground;
        None (no fresh measurement) never triggers a recovery.
        """
        if not self.enabled:
            return None
        if self.gave_up:
            return {"state": "gave_up", "throttle_norm": 0.0}
        if self.phase is not None:
            while self.phase is not None and now >= self.phase_end:
                index = self.PHASES.index(self.phase)
                if index + 1 < len(self.PHASES):
                    self._start(self.PHASES[index + 1], self.phase_end)
                else:
                    self.phase = None
                    self.slow_since = None
                    self.grace_until = now + self.options["startup_grace_sec"]
            return None if self.phase is None else self._command()

        speed_known = (isinstance(speed_mps, (int, float)) and not isinstance(speed_mps, bool)
                       and math.isfinite(speed_mps))
        if now < self.grace_until or not forward_commanded or not speed_known:
            self.slow_since = None
            return None
        if speed_mps >= self.options["stuck_speed_mps"]:
            self.slow_since = None
            return None
        if self.slow_since is None:
            self.slow_since = now
        if now - self.slow_since < self.options["stuck_sec"]:
            return None
        if self.attempts >= self.options["max_attempts"]:
            self.gave_up = True
            return {"state": "gave_up", "throttle_norm": 0.0}
        self.attempts += 1
        self._start("neutral_in", now)
        return self._command()
