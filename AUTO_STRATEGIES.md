# Autonomous control strategies

Select the AUTO controller in `calibration.json`:

```json
"autonomous": {
  "strategy": "02_adaptive_throttle",
  "02_adaptive_throttle": {
    "full_throttle_cte_m": 0.30,
    "full_throttle_heading_deg": 11.0
  }
}
```

This is a section of the existing JSON file. Keep the hardware, receiver, PWM,
and mode-switch sections alongside it.

The numbers follow creation order and stay fixed as more strategies are added.

| Number | Strategy | Behavior |
| --- | --- | --- |
| 1 | `01_constant_cruise` | Original main behavior: proportional heading steering and constant cruise throttle. |
| 2 | `02_adaptive_throttle` | Existing branch behavior: the same steering, with full throttle when absolute cross-track error is at most 0.30 m and absolute heading error is at most 11 degrees; cruise otherwise. |
| 3 | `03_gps_course_progressive` | Experimental fresh GPS-course steering, validated forward-motion handover, and progressive power between cruise and full throttle. |
| 4 | `04_predictive_recovery` | GPS-course handover with predictive recovery power and continuous, speed-dependent lookahead along the selected path. |
| 5 | `05_straight_line` | Race line: one straight line drawn where AUTO is engaged (moved toward the centerline near a bank), full throttle, GPS-course line-of-sight steering with a PI loop on course error, and bank protection inside a +-2.5 m band. No lanes. |

The original names `constant_cruise`, `adaptive_throttle`,
`gps_course_progressive`, `predictive_recovery`, and `straight_line` remain accepted aliases for
their corresponding numbered strategies in JSON selectors, option-block keys,
and CLI arguments. Defaults, new logs, and files saved by the calibration tool
use the numbered names. Existing logs retain their recorded names.

Use one option block per strategy. Supplying both its original and numbered
option-block names is rejected, even if the settings are identical.

Change only `autonomous.strategy` to switch strategies. The inactive strategy's
settings can stay in the file. Missing `autonomous` or `strategy` settings select
`02_adaptive_throttle`, preserving this branch's previous default. Settings are read
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
python3 multipath_boat.py --auto-strategy 01_constant_cruise
python3 multipath_boat.py --auto-strategy 02_adaptive_throttle --throttle-ratio 0.60
python3 multipath_boat.py --auto-strategy 03_gps_course_progressive
python3 multipath_boat.py --auto-strategy 04_predictive_recovery
```

These commands launch the normal boat controller and initialize its hardware.
`--config path/to/calibration.json` selects a different configuration file.
The calibration tool preserves the autonomous section when loading and saving.

## boat.py (web control page)

`boat.py` uses the same AUTO controller: the multipath navigator, the strategy
selected here, the GPS-course heading selector, and `navigation.finish_margin_m`.
It also uses the same hardware calibration as multipath_boat.py. At start-up it
reads `hardware`, `rc`, `pwm`, `mode_switch`, `autonomous` and `navigation` from
`calibration.json`, and it refuses to start without that file.

- **Hardware:** boat.py talks to the hardware with multipath_boat.py's own
  drivers, so both programs behave the same way. The serial ports, baud rates,
  PCA9685 bus, address, frequency, servo channel and ESC channel come from
  `hardware`.
  - **Motor and steering:** `PCA9685Driver`, a direct I2C driver that takes
    microseconds. It needs `smbus2` (`pip3 install smbus2`).
  - **RC:** `ArduinoRCReader` reads the Arduino's `CH1,CH2` lines in its own thread.
  - **GPS:** `ZEDF9PReceiver` reads the ZED-F9P in its own thread, and
    `NGII_NtripClient` injects the RTCM corrections through the same receiver.
    The NTRIP login is in `NTRIP_CONFIG` in multipath_boat.py, not in boat.py.
  - **Ports:** `"auto"` uses the same USB-ID detection as multipath_boat.py. A
    background thread opens the ports and reopens them after an unplug, so a
    reconnect never stalls the control loop. A GPS port that stays silent for 5 s
    is reopened.
- **PWM:** the web page and the controllers keep boat.py's internal scale
  (`CENTER` 4500, `STOP` 4443, `max_GO`). Only the output to the PCA9685 is
  converted:
  - `CENTER`..`max_LEFT`/`max_RIGHT` becomes `servo_center_us`..`servo_left_us`/`servo_right_us`.
  - `STOP`..4800 becomes `esc_neutral_us`..`esc_fwd_max_us`.
  - `STOP`..`max_BACK` becomes `esc_neutral_us`..`esc_rev_max_us`.

  `max_GO` presets above 4800 are capped at `esc_fwd_max_us`, so the 5000–5200
  presets give no more thrust than 4800.
- **RC:** the stick pulses are normalized with the `rc` neutral, end-point and
  deadband values, so reversed channels work. The emergency override triggers on
  85% steering either way, or on `mode_switch.full_reverse_ratio_threshold`
  reverse.
- **Steering:** the strategy's [-1, 1] output maps to `CENTER`..`max_RIGHT`
  (positive) or `CENTER`..`max_LEFT` (negative). `STEER_SIGN = -1` swaps sides.
- **Throttle:** cruise is `auto_throttle_ratio` of the way from `STOP` to the web
  page's `max_GO`, and full power is `max_GO`. With a ratio of 1.0, cruise and
  full power are the same.
- **Lanes:** the web page's lane lock sets the AUTO lane and turns off lane
  changes for lane-based strategies. With `05_straight_line`, AUTO entry draws
  the course line; the dashboard labels straight-line or test-square guidance
  and disables lane selection. Test-square mode uses its configured test throttle.
  Wall-guard recovery still applies during normal straight-line guidance.
- **Safety:** boat.py keeps its own failsafes. These are the RC override, E-stop,
  link-loss hold, GPS timeout, the 1 s bad-fix stop, the 5 s stall stop, and a 5 s
  no-heading stop.
- **Log columns:** the `logs/auto_log_*.csv` file has these extra columns: `CTE_m`,
  `Strategy`, `ThrottleBlend`, `HeadingSrc` and `WallGuard`.

The web page's P/PID steering gains no longer affect AUTO. The camera BALLOON
mode is unchanged.

## Finish margin

Set `navigation.finish_margin_m` in `calibration.json` to continue past the finish
before stopping in neutral and returning to MANUAL. For example, this section
sets a 3 m margin for every AUTO strategy:

```json
"navigation": {
  "finish_margin_m": 3.0
}
```

The margin extends each lane's final segment straight ahead by that distance.
Steering continues toward the extended endpoint, and completion requires
crossing its perpendicular line. The arrival radius does not shorten a positive
margin, even if it is smaller than `finish_radius_m`. The existing final-quarter
progress check and 10 m lateral limit still apply. CSV files and waypoint numbers
remain unchanged. The distance is measured from the GPS position; it is the
neutral-command threshold, not a guarantee of where the moving boat stops.

The default is `0.0`, which preserves completion within the existing 1.8 m finish
radius or across the original finish line. Missing navigation settings also use
this default. Values must be finite, nonnegative numbers. The calibration tool
preserves this section when loading and saving; restart after editing it.

To override the margin for one run:

```sh
python3 multipath_boat.py --finish-margin-m 3
```

The effective value appears in the startup summary and session event log.

## 03 — GPS-course progressive experiment

To enable this mode for subsequent runs, set `autonomous.strategy` to
`"03_gps_course_progressive"`. The checked-in JSON keeps `02_adaptive_throttle`
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

The options in `autonomous["03_gps_course_progressive"]` are:

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

## 04 — Predictive recovery experiment

Select `"04_predictive_recovery"` in `autonomous.strategy`, or use the CLI command
above. Its independent settings live in `autonomous["04_predictive_recovery"]`.
The checked-in JSON still selects `02_adaptive_throttle`; the other three modes
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
power remains at cruise. Targets clamp to the final path endpoint, including any
configured finish margin. A lane change recomputes projection and prediction
against the new lane.

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
defaults. Give the next strategy the prefix `05_`, then continue in creation
order. Never renumber existing strategies: their names identify configurations
and historical runs. Each strategy provides:

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

## 05 — Straight line (race line)

`calibration.json` selects `"05_straight_line"`. Settings live in
`autonomous["05_straight_line"]`. Point the boat down the course before
engaging AUTO: the boat has no heading until it is moving.

Quick changes at the water (each also has a JSON setting):

```sh
python3 multipath_boat.py --use-04            # back to 04 control (JSON: autonomous.use_04_control)
python3 multipath_boat.py --line-limit 2.0    # line within +-2.0 m of the centerline, both sides (line_left_m, line_right_m)
python3 multipath_boat.py --bank-limit 3.0    # bank limits at +-3.0 m, both sides (bank_left_m, bank_right_m)
python3 multipath_boat.py --bank-right 4.5 --line-left 2.0   # one side only; also --bank-left, --line-right
python3 multipath_boat.py --no-turn-planner   # turn the short way round (turn_planner)
python3 multipath_boat.py --test-square       # boundary test pattern (test_square)
python3 multipath_boat.py --no-recovery       # stuck recovery off (autonomous.recovery.enabled)
```

On each side, `line limit + guard_zone_m (0.5)` must stay inside the bank limit,
otherwise the controller refuses to start and says why.

**Line.** On AUTO entry the boat's RTK position defines a line parallel to the
course (the bearing of `center.csv`, first to last waypoint). Once the first
forward course is known, the line moves to where the boat's turn onto the race
direction will end (full-lock radius plus `turn_delay_sec` of travel). The line
never lies further out than `line_left_m` / `line_right_m`; beyond that the boat
merges onto it while driving forward, never turning more than
`max_intercept_deg` off the course. Re-engaging AUTO draws a new line. Lanes,
lane switching and `left.csv`/`right.csv` are not used. The event log records
`[AUTO LINE]` with the entry and line offsets.

**Turn planner.** When the course is more than `turn_plan_deg` (60) off the race
direction (at entry, after a takeover, or after a collision), the navigator
works out both full-lock turns with the measured radii (`turn_radius_left_m`
3.0, `turn_radius_right_m` 2.3) and chooses the one that stays inside the bank
limits in the shortest arc. The short way round can point the boat straight at
the bank (17 Nov routes 2, 3 and 6). The boat holds full lock in that direction
until the course is within `turn_exit_deg` (25), even when the other way
becomes shorter, and the line is drawn where the turn ends. If neither turn
fits and the boat is heading toward the limit it would cross, it goes neutral
(`plan_neutral_sec`), reverses at `plan_reverse_throttle` for
`plan_reverse_sec` with the rudder centered, goes neutral, and plans again from
fresh forward motion (at most `plan_max_reverses` times per AUTO run; then the
least-bad turn). The event log records `[TURN PLAN]` with the choice and the
predicted sideways range; `prediction_status` shows `turn_right`, `turn_left`,
`plan_neutral_in`, `plan_reverse` or `plan_neutral_out`. Full throttle is kept
while turning: the measured turns are tighter at full throttle (about 2.2 m)
than at 60% (2.1-6.9 m), because the propeller wash drives the rudder. Reversing
needs a land check first (does this ESC reverse after the neutral pause?).

**Steering.** Outer loop: line-of-sight course
`course_cmd = course_bearing - clamp(atan(cte / lookahead), +-max_intercept_deg)`,
with the speed-dependent lookahead of 04. GPS course over ground already
includes current, so this loop has no integral. Inner loop: proportional
steering on course error (`course_gain` x the 01-04 gain) plus an integral
(`course_integral_gain`, 1/s, contribution capped at `integral_limit_deg`)
that removes rudder trim. The integral is frozen for `integral_delay_sec` after
entry, below `min_course_speed_mps`, without an RTK-trusted course, in the bank
zones, and when the course error exceeds `integral_max_error_deg`. Without a
course the rudder holds the learned trim (zero at launch).

**Bank protection.** The banks are straight and parallel to the centerline;
`bank_left_m` / `bank_right_m` are the allowed distances from it. Within
`guard_zone_m` of a bank limit and not already moving back, `prediction_status` is
`bank_guard`: boosted gain (`boundary_gain_boost`) and up to
`recovery_intercept_deg` of intercept. If the 1 s forecast
(`prediction_horizon_sec`) passes a bank limit while moving outward, the status is
`hard_limit` and the boat steers `recovery_intercept_deg` back toward the
centerline. Throttle stays full: turning radius does not shrink at low speed.

**Throttle and finish.** Full forward (`esc_fwd_max_us`) whenever AUTO is
steering, whatever `pwm.auto_throttle_ratio` is. AUTO ends
`navigation.finish_margin_m` past the finish line through the last centerline
waypoint, measured along the course.

| Option | Default | Meaning |
| --- | --- | --- |
| `bank_left_m`, `bank_right_m` | 3.2, 4.0 | Bank limits from the centerline. Measured banks (RTK bank hits): left 4.20 m, right 5.11 m. |
| `line_left_m`, `line_right_m` | 2.6, 3.4 | Furthest from the centerline a line may be drawn on each side. |
| `guard_zone_m` | 0.5 | Width of the bank-guard zone inside each bank limit. |
| `turn_planner` | true | Plan large turns against the bank limits (`--no-turn-planner` turns the short way). |
| `turn_plan_deg`, `turn_exit_deg` | 60, 25 | Plan a turn when the course is further off than this; hand back to line following below this. |
| `turn_radius_left_m`, `turn_radius_right_m` | 3.0, 2.3 | Full-lock, full-throttle turn radii (measured 2.1-2.3 m right/left on 17 Nov, left 2.9-3.5 m on 16 Nov). |
| `turn_delay_sec` | 0.4 | Straight travel before a turn bites (servo slew and GPS delay). |
| `plan_neutral_sec`, `plan_reverse_sec`, `plan_reverse_throttle`, `plan_max_reverses` | 0.3, 1.5, 0.5, 2 | The reverse-then-turn manoeuvre. |
| `max_intercept_deg` | 15.0 | Largest angle off the course while merging onto the line. |
| `recovery_intercept_deg` | 40.0 | Intercept in the bank zones and the hard-limit turn. |
| `lookahead_time_sec`, `lookahead_min_m`, `lookahead_max_m` | 1.8, 2.4, 5.0 | Speed-dependent line-of-sight distance. |
| `prediction_horizon_sec` | 1.0 | Forecast for the hard limit. |
| `course_gain` | 1.0 | Multiplier on the proportional steering gain. |
| `course_integral_gain` | 0.3 | Integral gain on course error (1/s); 0 disables it. 0.3 matched 0.6 on trim robustness in simulation and cost 0.2-0.5 s less per race. |
| `integral_limit_deg` | 20.0 | Cap on the integral's steering contribution. |
| `integral_delay_sec` | 2.0 | No integration for this long after entry. |
| `integral_max_error_deg` | 20.0 | No integration while the course error is larger. |
| `min_course_speed_mps`, `course_max_age_sec`, `position_heading_max_age_sec`, `forward_acquire_sec`, `forward_memory_sec` | as 03/04 | GPS-course heading selection. |

Telemetry uses the existing columns: `lane` is `line`, `cte_right_m` is the
error to the line, `prediction_status` is `line`, `merging`, `bank_guard`,
`hard_limit`, `heading_unavailable` or one of the turn-planner states above, and
`prediction_corridor_clearance_m` is the forecast distance to the bank limit. Compare strategies offline with
`python3 benchmark.py compare` and
`python3 simulate_boat.py auto --strategy 05_straight_line` (see `RACE_SPEED.md`).

### Test square (`test_square`)

With `test_square` true (or `--test-square`), AUTO does not race. From the
entry point it drives two rectangles at `test_throttle` (0.6 = 60% forward):
first with long sides on the bank limits (d = -bank_left_m, +bank_right_m), then
on the line limits (d = -line_left_m, +line_right_m). Each rectangle starts `test_lead_m`
ahead, runs `test_leg_length_m` forward along the left line, crosses to the
right line, comes back along it and crosses back; every corner is a right turn
(the hull's tighter direction). A corner counts as reached within
`test_accept_m`. Bank protection is off in this mode because the legs are the
limits; engage it in open water near the start of the course, pointed down the
course, and keep the transmitter ready (full reverse returns to MANUAL).

`prediction_status` names the leg (`square leg 2: bank -3.20/+4.00 m left line`) and the
event log records each leg. Afterwards:

```sh
python3 analyse_square.py logs/multipath_log_YYYYMMDD_HHMMSS.csv
```

reports, per leg, where the boat actually was from the centerline and its error
to the commanded line, using only the real log. Compare the "left line" and
"right line" legs with where the team saw the boat relative to the banks.

## Stuck recovery (all strategies)

`autonomous.recovery` (default on). In AUTO, if the ESC is commanded forward but
GPS speed stays below `stuck_speed_mps` (0.15) for `stuck_sec` (1.5 s), the
boat goes neutral (`neutral_sec`, 0.3 s), reverses at `reverse_throttle` (60%)
for `reverse_sec` (2 s) with the rudder centered, goes neutral again, then
resumes normal control on the same line; heading is re-acquired from fresh
forward motion. Detection waits `startup_grace_sec` (3 s) after AUTO entry and
after each attempt. After `max_attempts` (3) failed attempts in one AUTO run it
stops in neutral and returns to MANUAL. A missing GPS speed never triggers it.
`recovery_state` in the telemetry shows `neutral_in`, `reverse`, `neutral_out`
or `gave_up`, and the event log records `[RECOVERY]`.

Before relying on it, check on land with the prop out of the water that this
ESC goes into reverse after the neutral pause (some ESCs need a second
neutral-reverse pulse or a longer `neutral_sec`). The transmitter override
still works throughout.
