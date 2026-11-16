# Autonomous control strategies

Select the AUTO controller in `calibration.json`:

```json
"autonomous": {
  "strategy": "adaptive_throttle",
  "adaptive_throttle": {
    "full_throttle_cte_m": 0.30,
    "full_throttle_heading_deg": 11.0
  }
}
```

This is a section of the existing JSON file. Keep the hardware, receiver, PWM,
and mode-switch sections alongside it.

| Strategy | Behavior |
| --- | --- |
| `constant_cruise` | Original main behavior: proportional heading steering and constant cruise throttle. |
| `adaptive_throttle` | Existing branch behavior: the same steering, with full throttle when absolute cross-track error is at most 0.30 m and absolute heading error is at most 11 degrees; cruise otherwise. |

Change only `autonomous.strategy` to switch strategies. The inactive strategy's
settings can stay in the file. Missing `autonomous` or `strategy` settings select
`adaptive_throttle`, preserving this branch's previous default. Settings are read
at startup; restart the controller after editing the file.

`pwm.auto_throttle_ratio` sets the cruise throttle for both strategies. With the
current calibration, 0.60 produces 1914 microseconds; adaptive full power is
1990 microseconds. The ratio is a cruise setting, not an adaptive maximum-power
limit. Adaptive thresholds are inclusive and independently configurable. The CTE
threshold must be a finite nonnegative number, and heading must be between 0 and
180 degrees. Unknown names, options, and malformed settings are rejected before
hardware initialization, including settings supplied for an inactive strategy.

To select a strategy for one run without changing the JSON:

```sh
python3 multipath_boat.py --auto-strategy constant_cruise
python3 multipath_boat.py --auto-strategy adaptive_throttle --throttle-ratio 0.60
```

These commands launch the normal boat controller and initialize its hardware.
`--config path/to/calibration.json` selects a different configuration file.
The calibration tool preserves the autonomous section when loading and saving.

## Shared navigation and safety

Both initial strategies use the same path selection, lookahead, lane changes,
boundary recovery, proportional steering, and steering slew limit. Missing
heading uses cruise throttle and no initial steering correction. AUTO entry
resets steering state. Manual control, transmitter override, RC-loss handling,
invalid-GPS neutral output, and course-completion neutral output remain in the
main controller and take precedence over strategy commands.

The terminal and event log record `[AUTO STRATEGY]` with the effective strategy,
options, navigation parameters, cruise ratio, and PWM levels. Telemetry appends
`auto_strategy` and `throttle_blend` after the existing columns. A blend of 0
means cruise, and 1 means full power; it does not mean stopped versus moving.
The blend is blank when no navigation update ran, such as MANUAL mode, AUTO
entry, or a GPS hold. Use `mode` and `esc_us` to determine the actual output.

## Adding another strategy

Implement a strategy in `auto_strategies.py` and register it in `AUTO_STRATEGIES`.
The registry supplies JSON selection, CLI choices, validation, and calibration
defaults. Each strategy provides:

- A stable `name`, `default_options`, and option validation. `option_limits`
  supports finite numeric bounds; override `validate_options` for other options.
- An initializer accepting navigation parameters and strategy options, with
  resolved settings exposed as `options`.
- `reset()`, called on every AUTO entry to clear experimental controller state.
- `update(cte_right_m=..., heading_error_deg=..., boundary_guard=..., now_mono=...)`,
  returning `steer_norm` in [-1, 1] and `throttle_blend` in [0, 1]. Heading error
  is wrapped to [-180, 180) or is `None`; time is monotonic seconds.
- `describe_throttle(cruise_us, full_us)` for the startup description.

For a throttle-only experiment, subclass `ConstantCruiseStrategy` and override
`throttle_blend`. For a different steering controller, override `update` and
reset any added state in `reset`. Keep navigation and safety handling shared.
The current interface supports throttle between cruise and full forward;
strategies requiring a lower throttle or different path guidance need an
explicit extension of that interface.

Add offline tests for the new strategy, its configuration, and mode transitions:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p 'test_*.py' -v
```
