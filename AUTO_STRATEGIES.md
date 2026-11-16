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
| `predictive_recovery` | GPS-course handover with predictive recovery power and continuous, speed-dependent lookahead along the selected path. |

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
python3 multipath_boat.py --auto-strategy predictive_recovery
```

These commands launch the normal boat controller and initialize its hardware.
`--config path/to/calibration.json` selects a different configuration file.
The calibration tool preserves the autonomous section when loading and saving.

## GPS-course progressive experiment

To enable this mode for subsequent runs, set `autonomous.strategy` to
`"gps_course_progressive"`. The checked-in JSON keeps `adaptive_throttle`
selected and includes the new mode's settings for editing. The existing two
strategies retain their heading and throttle behavior.

This mode uses the receiver's RMC course over ground as soon as it is usable,
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

## Predictive recovery experiment

Select `"predictive_recovery"` in `autonomous.strategy`, or use the CLI command
above. Its independent settings live in `autonomous.predictive_recovery`.
The checked-in JSON still selects `adaptive_throttle`; the other three modes
retain their control behavior.

This mode inherits the GPS-course progressive mode's heading acquisition,
same-source speed checks, RTK boost requirement, steering controller, and power
ramp. The selected course or pure position heading supplies its own speed and
timestamp; prediction never pairs a position heading with a held RMC speed.
Motion is rechecked against its source's age limit on every navigation update:
0.6 seconds for course, or 0.8 seconds for the position fallback, by default.
Missing or stale motion disables increased power.

The short forecast is:

```text
lateral_speed = speed × sin(course − local_path_bearing)
predicted_CTE = current_CTE + prediction_horizon × lateral_speed
```

CTE is positive to the right of the path, so the formula handles left and right
recovery symmetrically. When the boat is clearly converging, the controller can
use the smaller predicted error for its power score. By default, convergence
must be at least 0.05 m/s. A forecast that crosses the centerline and overshoots
by more than the 0.30 m full-power tolerance receives no extra recovery credit.
Neither does a route ahead that turns more than 15 degrees relative to the local
path. Without recovery credit, the larger of current and predicted absolute
CTE limits power, allowing outward motion to reduce power before the current
error becomes large. Heading error still independently limits power.

The forecast also samples travel at 0%, 25%, 50%, 75%, and 100% of the configured
horizon, which defaults to 1 second. Every sample, including the current
position, must remain inside an envelope around the outer route paths. The
outward allowance is the shared `shift_cte_m` boundary threshold minus
`prediction_corridor_margin_m`: 0.70 − 0.20 = 0.50 m beyond each outer path with
the defaults. These paths define a controller envelope; they are not surveyed
banks or obstacle boundaries. The sampled constant-velocity forecast does not
guarantee clearance between samples or model the boat's acceleration and turning.
Current absolute CTE at or above `cruise_cte_m` (1.5 m by default), an envelope
violation, or the existing boundary guard requests cruise even if the final
predicted CTE is small.

For steering, the navigator projects the boat onto the closest selected-path
segment, then walks forward along the polyline and interpolates a target. It
starts from that projection, not the already-advanced target waypoint index.
This avoids jumping the steering target between discrete waypoints. The target
distance is `speed × lookahead_time_sec`, clamped to the configured minimum and
maximum. Defaults give 1.8 seconds of travel, bounded to 2.4–5.0 m. Boundary
recovery caps this at the existing 2.4 m boundary lookahead. Missing or stale
speed uses the fixed 3.5 m lookahead, clamped to the configured bounds, while
power remains at cruise. Targets clamp to the final path endpoint. A lane
change recomputes projection and prediction against the new lane.

These settings are additional to the inherited GPS-course progressive options:

| Setting | Default | Meaning |
| --- | --- | --- |
| `lookahead_time_sec` | 1.8 | Travel time used to calculate along-path lookahead. |
| `lookahead_min_m` | 2.4 | Minimum normal lookahead distance. |
| `lookahead_max_m` | 5.0 | Maximum normal lookahead distance. |
| `prediction_horizon_sec` | 1.0 | Constant-velocity forecast horizon. |
| `prediction_min_convergence_mps` | 0.05 | Minimum inward lateral speed for recovery credit. |
| `prediction_max_path_turn_deg` | 15.0 | Maximum route turn ahead that permits recovery credit. |
| `prediction_corridor_margin_m` | 0.2 | Margin subtracted from the outer-path boundary allowance. |

Lookahead distances, travel time, horizon, and minimum convergence speed must be
positive finite numbers. Maximum lookahead must be at least minimum lookahead.
The turn limit is between 0 and 180 degrees. The corridor margin is nonnegative
and must be smaller than the shared `shift_cte_m` threshold. These relationships
are validated for inactive configuration blocks too, before hardware setup.

The inherited 0.5-per-second increase limit still applies; reductions are
immediate. This remains an experiment: compare repeated on-water runs for finish
time, maximum deviation, and interventions. Replaying a log can compare commands,
but cannot establish the trajectory or time savings caused by changed commands.

## Shared navigation and safety

All strategies use the same path selection, lane changes,
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

Predictive navigation additionally records `lookahead_m`,
`lookahead_target_e_m`, `lookahead_target_n_m`, `cte_rate_mps`,
`predicted_cte_m`, `prediction_status`, `prediction_corridor_clearance_m`, and
`throttle_target_blend`. Target coordinates are local east/north metres relative
to the first center-path waypoint. Corridor clearance is the smallest signed
clearance across the five samples; a negative value disallows increased power.
The target blend is the requested value before the increase ramp, while
`throttle_blend` records the applied value. These fields are blank for modes or
cycles that do not compute predictive navigation.

## Adding another strategy

Implement a strategy in `auto_strategies.py` and register it in `AUTO_STRATEGIES`.
The registry supplies JSON selection, CLI choices, validation, and calibration
defaults. Each strategy provides:

- A stable `name`, `default_options`, and option validation. `option_limits`
  supports finite numeric bounds; override `validate_options` for other options.
  `validate_navigation_options(nav_params, options)` checks relationships with
  shared navigation settings for active and explicitly configured inactive modes.
- An initializer accepting navigation parameters and strategy options, with
  resolved settings exposed as `options`.
- `reset()`, called on every AUTO entry to clear experimental controller state.
- `suspend()`, called during GPS holds for strategies using the course selector.
  The progressive strategy clears its throttle and steering state here.
- `uses_gps_course`, which selects the shared experimental motion-course policy
  when true; false preserves the legacy heading path.
- `uses_predictive_guidance`, which enables continuous target interpolation,
  `lookahead_distance(speed_mps, boundary_guard)`, and the `prediction` context
  passed to `update`. It is false for all other current modes.
- `update(cte_right_m=..., heading_error_deg=..., boundary_guard=..., now_mono=..., heading_confident=...)`,
  returning `steer_norm` in [-1, 1] and `throttle_blend` in [0, 1]. Heading error
  is wrapped to [-180, 180) or is `None`; time is monotonic seconds. The new
  progressive strategy requires `heading_confident` to enable increased power.
- `describe_throttle(cruise_us, full_us)` for the startup description.

For a throttle-only experiment, subclass `ConstantCruiseStrategy` and override
`throttle_blend`. For a different steering controller, override `update` and
reset any added state in `reset`. Keep navigation and safety handling shared.
The current interface supports throttle between cruise and full forward.
Strategies requiring a lower throttle or another guidance method need an
explicit extension of that interface.

Add offline tests for the new strategy, its configuration, and mode transitions:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p 'test_*.py' -v
```
