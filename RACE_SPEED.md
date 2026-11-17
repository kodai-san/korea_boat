# Race speed changes (branch `feature/race-speed`)

Goal: fastest start-to-finish run on the straight 92.4 m course, robust to wind, current and rudder trim.
Every change below was checked in the fitted simulator; the finish-line fix is also visible in a real log.

## What changed

| Change | Where | Evidence | Gain |
| --- | --- | --- | --- |
| **Full throttle whenever AUTO steers** (`full_throttle` strategy, now the default in `calibration.json`) | `auto_strategies.py`, `calibration.json` | `benchmark.py`: 300 paired scenarios x 3 fitted models, 8 s windows validated on held-out logs. The old `adaptive_throttle` drops to 60% whenever the boat is >0.3 m or >11 deg off line. | **~5.5-6 s per race**, defensible under every model |
| **Drive through the finish line** (stop 1.0 m past the line through waypoint #65, not 1.8 m before it) | `multipath_boat.py` `NAV_PARAMS` `finish_radius_m` 0, `finish_overrun_m` 1.0 | Real 17:27 AUTO run: motor cut 1.71 m before the line, boat coasted over it ~2 s later than at full speed. Simulator: 68.0 s -> 65.3 s | **~2-3 s per race** |
| **Heading integral 0.6/s** (cancels rudder trim that changes between runs) | `calibration.json` `full_throttle.heading_integral_gain` | Full-race simulations at the trims seen on the water: lane error 0.05-0.14 m instead of 0.15-0.59 m; worst drift past the outer lanes 0.3 m instead of 1.1 m | Robustness (same speed) |
| **Heading from the receiver's Doppler course** above 0.45 m/s | `multipath_boat.py` `NAV_PARAMS["heading_source"] = "cog"` | Doppler speed is ~8x less noisy than position differences (17:13 log); no 40 deg heading spike when RTK drops and the position jumps (17:21 swerve) | Robustness |
| **NTRIP stall watchdog**: reconnect after 8 s without correction data | `multipath_boat.py` `NTRIP_CONFIG["stall_timeout_sec"]` | 17:13 log: corrections stopped for up to 122 s while "connected"; RTK lost 31% of the run | Keeps RTK |

Rollback, each independent:
- strategy: `"strategy": "adaptive_throttle"` in `calibration.json` (or `--auto-strategy adaptive_throttle`)
- finish: `NAV_PARAMS["finish_radius_m"] = 1.8`, `"finish_overrun_m" = 0.0`
- heading: `NAV_PARAMS["heading_source"] = "blend"`
- watchdog: `NTRIP_CONFIG["stall_timeout_sec"] = 0`

## Free gains from race procedure
- **Point the boat straight down the course before engaging AUTO.** The two real AUTO entries were 47-60 deg off
  (the 2 s full-right stick hold turns a moving boat). Simulator: 0 deg -> 65.3 s, 45 deg -> 66.2 s.
- **Fresh battery for every timed run.** Speed at full throttle varied 0.87-1.10x between logs.
- **Wait for RTK FIX before starting** (event log `[GPS STATE] fix_quality=4`).
- Optional, if the rules do not require lane changes: hold the start lane (`NAV_PARAMS["shift_cte_m"] = 99`).
  In the harshest simulated case (strongest trim + 6 deg/s gusts) lane switching drifted the boat 1.1 m past the
  outer lane; holding the lane kept it inside in every case, at the same race time.

## Biggest remaining lever: the throttle ceiling (needs a water test)
`pwm.esc_fwd_max_us` is 1990 us, only 190 us above ESC neutral. Speed is still rising at the top
(fitted ~1.29 m/s at 1950 us -> ~1.42 m/s at 1990 us), so more throttle likely means more speed, but there is no
data above 1990 us and the simulator cannot extrapolate. Suggested test, MANUAL mode, straight line, prop clear of
the bank, one step at a time: set `esc_fwd_max_us` with `calibrate_tool.py` (ESC wizard) to 2030, 2070, ...;
log 15 s straight runs in both directions at each step; stop when speed stops increasing or steering gets
twitchy. Then refit (`python3 fit_boat_model.py`) and rerun the benchmark.

## Before the next timed run
1. `git pull` this branch on the Jetson, `python3 -m unittest discover -s tests -p 'test_*.py'` (10 offline tests).
2. On land: `python3 multipath_boat.py`, check the startup line says
   `AUTO throttle : full 1990 us whenever AUTO is steering; heading integral 0.60/s`.
3. Phone gyro (optional but valuable): tape the phone flat, screen up, aligned with the hull, and record
   gyroscope + accelerometer in phyphox.
4. At the start of the session: 20-30 s straight at full throttle, rudder centred, in both directions
   (measures the day's rudder trim directly).

## Tools
```sh
python3 benchmark.py window          # re-validate the scoring window on held-out logs
python3 benchmark.py compare --n 300 # paired comparison: built-ins + boat code variants + tuned
python3 simulate_boat.py auto --runs 20 --yaw-bias 0.10            # full races, current config
python3 simulate_boat.py auto --strategy adaptive_throttle --heading-source blend \
    --nav-param finish_radius_m=1.8 --nav-param finish_overrun_m=0  # the previous configuration
python3 simulate_boat.py validate-auto logs/multipath_log_20261116_172709.csv --t0 48.2 --t1 112.6
```
Add a new controller by subclassing `Controller` in `benchmark.py`, or a new boat strategy in
`auto_strategies.py` (the benchmark runs the real navigator with it via `NavigatorController`).
`RaceEnv` in `benchmark.py` is a gym-style environment for RL with the same scenarios and score.

## Limits
- Absolute simulated race times depend on the model (60-78 s across fitted models); differences are the robust part.
- Rudder trim changed between runs on 2026-11-16 (up to 16 deg/s at full throttle); the integral handles it in
  simulation but this is the largest real-world uncertainty.
- The 3 m bank distance and +-15 deg start heading in `benchmark.py` are assumptions.
