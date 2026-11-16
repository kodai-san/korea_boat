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
| `gps_course_progressive` | Experimental fresh GPS-course steering, validated forward-motion handover, and progressive power between cruise and full throttle. |

Change only `autonomous.strategy` to switch strategies. The inactive strategy's
settings can stay in the file. Missing `autonomous` or `strategy` settings select
`adaptive_throttle`, preserving this branch's previous default. Settings are read
at startup; restart the controller after editing the file.

`pwm.auto_throttle_ratio` sets the cruise throttle for all strategies. With the
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
python3 multipath_boat.py --auto-strategy gps_course_progressive
```

These commands launch the normal boat controller and initialize its hardware.
`--config path/to/calibration.json` selects a different configuration file.
The calibration tool preserves the autonomous section when loading and saving.

## GPS-course progressive experiment

To enable the new mode for subsequent runs, set `autonomous.strategy` to
`"gps_course_progressive"`. The checked-in JSON keeps `adaptive_throttle`
selected and includes the new mode's settings for editing. The existing two
strategies retain their heading and throttle behavior.

The new mode uses the receiver's RMC course over ground as soon as it is usable,
without waiting for a 25 cm position displacement. By default, a usable course
requires speed of at least 0.45 m/s, a valid current position, course and position
ages no greater than 0.6 seconds, and RMC/GGA source UTC times within 0.6 seconds.
Invalid or malformed RMC course data is rejected, and repeated source timestamps
do not keep an old course fresh. This uses the receiver's existing velocity-course
output; it does not add a raw Doppler processor or require a new GPS message type.

AUTO entry can reuse the current course when recent manual forward commands
provided at least 0.6 seconds of moving-course evidence. Neutral coasting can
retain that evidence for up to 4 seconds after forward throttle, provided course
and speed remain usable. Reverse, stopped motion, and stale course data clear
the evidence. Otherwise, AUTO begins at cruise and acquires direction after
0.6 seconds of forward AUTO operation and a new qualifying observation. Course
over ground is the direction of travel, not a direct measurement of bow heading;
forward-throttle evidence and the acquisition delay cannot guarantee the boat
has stopped drifting or moving backward.

If fresh course is unavailable, the mode can use a separate, unblended position
heading from at least 25 cm of displacement. The fallback requires RTK fixed or
float quality (GGA 4 or 5) at both endpoints, advancing GPS timestamps,
a heading age of at most 0.8 seconds, and measured
displacement speed of at least 0.45 m/s. A fresh low-speed course report prevents
reuse of a preceding movement vector. Fresh course can guide steering with a
non-RTK fix, but increased power requires RTK quality 4 or 5 for either source.

Power depends on both tracking errors. Each error produces a linear score from
0 at its cruise limit to 1 at its full-throttle limit; the lower score determines
the requested blend. With the defaults, full power requires absolute CTE at most
0.30 m and heading error at most 11 degrees. CTE of 1.5 m or heading error of
35 degrees requests cruise. For example, CTE of 0.90 m with a small heading
error requests a blend of 0.5: 1952 microseconds with the current calibration,
once the ramp reaches that level.

The blend rises at most 0.5 per second, so uninterrupted good tracking takes
about 2 seconds to ramp from cruise to full power. Delayed control cycles cannot
skip the ramp. Reductions take effect immediately. Missing or insufficiently
trusted heading and boundary recovery request cruise. A GPS hold still commands
neutral, clears the power ramp and steering history, and requires forward-motion reacquisition when
control resumes.

The options in `autonomous.gps_course_progressive` are:

| Setting | Default | Meaning |
| --- | --- | --- |
| `full_throttle_cte_m` | 0.30 | CTE at or below which its power score is 1. |
| `cruise_cte_m` | 1.5 | CTE at or above which its power score is 0. |
| `full_throttle_heading_deg` | 11.0 | Heading error at or below which its power score is 1. |
| `cruise_heading_deg` | 35.0 | Heading error at or above which its power score is 0. |
| `throttle_rise_per_sec` | 0.5 | Maximum increase in cruise-to-full blend per second. |
| `min_course_speed_mps` | 0.45 | Minimum speed for course or position-derived heading. |
| `course_max_age_sec` | 0.6 | Maximum course/position age and RMC/GGA source-time separation. |
| `position_heading_max_age_sec` | 0.8 | Maximum age of the position fallback heading. |
| `forward_acquire_sec` | 0.6 | Forward observation/acquisition interval. |
| `forward_memory_sec` | 4.0 | Maximum neutral coasting time for reuse of manual forward evidence. |

Cruise error limits must exceed their respective full-throttle limits; heading
limits cannot exceed 180 degrees. Speeds, ages, acquisition intervals, and ramp
rate must be positive finite numbers. These defaults are experimental and need
field validation; earlier steering and intermediate power do not guarantee a
faster run or better recovery.

## Shared navigation and safety

All strategies use the same path selection, lookahead, lane changes,
boundary recovery, proportional steering, and steering slew limit. Missing
heading uses cruise throttle and no initial steering correction. AUTO entry
resets steering state. Manual control, transmitter override, RC-loss handling,
invalid-GPS neutral output, and course-completion neutral output remain in the
main controller and take precedence over strategy commands.

The terminal and event log record `[AUTO STRATEGY]` with the effective strategy,
options, navigation parameters, cruise ratio, and PWM levels. Telemetry includes
`auto_strategy` and `throttle_blend`. A blend of 0
means cruise, and 1 means full power; it does not mean stopped versus moving.
The blend is blank when no navigation update ran, such as MANUAL mode, AUTO
entry, or a GPS hold. Use `mode` and `esc_us` to determine the actual output.

During AUTO navigation, `boat_heading_deg` records the selected control heading.
The appended `legacy_heading_deg` retains the previous blended heading for
comparison. `control_heading_source`, `control_heading_age_sec`, and
`heading_state` explain selection or acquisition. `heading_confident` marks
eligibility for increased power in the new mode, rather than a statistical
confidence estimate. Additional fields record `gps_course_valid`,
`gps_course_deg`, `gps_course_speed_mps`, `gps_course_age_sec`, `gps_course_utc`, `gps_course_gga_skew_sec`,
`position_heading_deg`, `position_heading_age_sec`, and `position_speed_mps`.
The validity field describes the parsed observation; speed, age, and fix checks
still determine whether the controller uses it.

## Adding another strategy

Implement a strategy in `auto_strategies.py` and register it in `AUTO_STRATEGIES`.
The registry supplies JSON selection, CLI choices, validation, and calibration
defaults. Each strategy provides:

- A stable `name`, `default_options`, and option validation. `option_limits`
  supports finite numeric bounds; override `validate_options` for other options.
- An initializer accepting navigation parameters and strategy options, with
  resolved settings exposed as `options`.
- `reset()`, called on every AUTO entry to clear experimental controller state.
- `suspend()`, called during GPS holds for strategies using the course selector.
  The progressive strategy clears its throttle and steering state here.
- `uses_gps_course`, which selects the shared experimental motion-course policy
  when true; false preserves the legacy heading path.
- `update(cte_right_m=..., heading_error_deg=..., boundary_guard=..., now_mono=..., heading_confident=...)`,
  returning `steer_norm` in [-1, 1] and `throttle_blend` in [0, 1]. Heading error
  is wrapped to [-180, 180) or is `None`; time is monotonic seconds. The new
  progressive strategy requires `heading_confident` to enable increased power.
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
