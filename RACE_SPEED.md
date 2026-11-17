# Race speed (branch `azka05`)

Goal: fastest start-to-finish run on the straight 92.4 m course, robust to wind, current and rudder trim.

## What this branch adds to main

| Change | Where | Evidence |
| --- | --- | --- |
| **`05_straight_line` strategy**: one straight line from where AUTO is engaged (moved toward the centerline if the entry is more than 1.5 m off it), full throttle, GPS-course line-of-sight steering with a PI loop on course error, bank protection in a +-2.5 m band. No lanes. | `auto_strategies.py`, `path_guidance.py`, `multipath_boat.py` (navigator hook), `calibration.json` block | Simulator: same speed as full-throttle 02-04 within ~0.5 s, much better line holding and bank safety under rudder trim (table below). Details in `AUTO_STRATEGIES.md`. |
| **NTRIP stall watchdog**: reconnect after 8 s without correction data | `multipath_boat.py` `NTRIP_CONFIG["stall_timeout_sec"]` | 2026-11-16 17:13 log: corrections stopped for up to 122 s while "connected"; RTK lost 31% of the run. Today's logs show 1-2 resets per run. |
| **Optional Doppler-course legacy heading** for 01/02 (`NAV_PARAMS["heading_source"] = "cog"`; default stays `"blend"`) | `multipath_boat.py` | 03-05 already use the GPS course through `gps_heading.py`. |
| **Simulator and benchmark** | `boat_model.py`, `fit_boat_model.py`, `simulate_boat.py`, `benchmark.py` | Fitted from the 2026-11-16 logs; 8 s scoring windows validated on held-out data. The simulator now drives the navigator exactly like the main loop, including the GPS-course heading selector (`simulate_boat.AutoDriver`). |

Full simulated races (10 per cell, start on the centerline pointed down the course, finish margin 1 m,
`yaw_bias` = rudder trim; +0.10 is the strongest right trim seen on 2026-11-16):

| Strategy | trim 0, gusts 3 / 6 deg/s | trim +0.10, gusts 3 / 6 deg/s | worst distance from centerline |
| --- | --- | --- | --- |
| 02 @ full throttle (run B) | 65.9 / 66.5 s | 66.3 / 66.9 s | 3.14 m |
| 04 @ full throttle (run D) | 65.8 / 66.5 s | 66.2 / 66.8 s | 3.15 m |
| 05, integral 0.3 (default) | 66.0 / 66.8 s | 66.2 / 67.0 s | 1.13 m |
| 05, no integral | 65.8 / 66.5 s | 65.8 / 66.5 s | 2.55 m |

The 8 s window benchmark (300 paired scenarios x 3 models) cannot separate any full-throttle controller
(all within 0.1 m per window); 60% cruise (run A) is defensibly ~6 s per race slower under every model.
05 trades up to ~0.5 s for staying inside the +-2.5 m band when the trim is strong; the integral is
what keeps it there. Starts near the bank: pointed straight, 2.0 m off center stays inside the band;
pointed 35 deg at the bank from 2.4 m it overshoots by ~1.1 m during the ~0.6 s before the boat has a
heading (04 and 05 identical).

Finish: main's `navigation.finish_margin_m` (1.0 m in `calibration.json`) drives through the finish line;
it replaces this branch's earlier `finish_overrun_m`. Full throttle: main's `pwm.auto_throttle_ratio` 1.0
makes 02-04 drive at full throttle as well; 05 always does.

Rollback: select another strategy in `calibration.json` (`autonomous.strategy`) or with
`--auto-strategy`; `NTRIP_CONFIG["stall_timeout_sec"] = 0` disables the watchdog.

## Water-day commands
```sh
python3 multipath_boat.py                      # 05_straight_line (calibration.json), stuck recovery on
python3 multipath_boat.py --use-04             # back to 04
python3 multipath_boat.py --auto-strategy 07_turn_planner                  # 05 + turn planner, per-side limits
python3 multipath_boat.py --auto-strategy 07_turn_planner --bank-right 4.5  # one side only (07)
python3 multipath_boat.py --auto-strategy 07_turn_planner --three-point-turns  # reverse-then-turn when faster (07)
python3 multipath_boat.py --auto-strategy 06_gps_aware                       # GPS-guarded straight line (06)
python3 multipath_boat.py --auto-strategy 08_guarded_turn_planner            # 06's GPS guard + 07's turn planner
python3 multipath_boat.py --line-limit 1.0 --bank-limit 3.0
python3 multipath_boat.py --test-square        # drive the boundary rectangles, then stop
python3 analyse_square.py logs/multipath_log_<stamp>.csv
python3 multipath_boat.py --no-recovery
```

## Race procedure
- **Point the boat straight down the course before engaging AUTO.** The boat has no heading until it moves
  (~0.6 s); a start pointed 35 deg at the bank from 2.4 m off center overshoots the 2.5 m band by ~1.1 m in
  simulation (still ~1 m short of the observed banks). Pointed straight, a start 2.0 m off center stays inside.
- **Fresh battery for every timed run.** Speed at full throttle varied 0.87-1.14x between logs.
- **Wait for RTK FIX before starting** (event log `[GPS STATE] fix_quality=4`).
- With 05, engage AUTO wherever the boat should race (within 1.5 m of the centerline); the event log line
  `[AUTO LINE]` shows where the line was drawn.

## Remaining levers
- **Survey the banks and the finish line with RTK.** The lanes and finish come from the previous team; the
  band (+-2.5 m) is based on how far the boat has been on 2026-11-16/17 (~4.2 m left, ~5.2 m right of the
  centerline; less data at 20-30 m and 80-100 m along the course).
- **Throttle ceiling.** `pwm.esc_fwd_max_us` is 1990 us, 190 us above ESC neutral, and speed was still
  rising at the top. Test in MANUAL, straight line, one step at a time (2030, 2070, ... with
  `calibrate_tool.py`), 15 s runs both ways, then refit and rerun the benchmark.
- **Refit the simulator with the 2026-11-17 logs** (the boat was ~14% faster than the fitted range).

## Tools
```sh
python3 benchmark.py compare --n 300   # paired comparison of 01-05 and reference controllers, 3 models
python3 simulate_boat.py auto --runs 20 --strategy 05_straight_line --yaw-bias 0.10
python3 simulate_boat.py auto --strategy 05_straight_line --start-offset 2.3 --start-heading 20
python3 simulate_boat.py validate-auto logs/multipath_log_20261117_095308.csv --t0 39.7 --t1 92.4
python3 benchmark.py window            # re-validate the scoring window on held-out logs
```

## Limits
- Absolute simulated race times depend on the model; differences between controllers are the robust part.
- Rudder trim changed between runs (up to 16 deg/s at full throttle); the integral handles it in simulation
  but this is the largest real-world uncertainty.
- The benchmark's 3 m crash corridor and +-15 deg start-heading spread are assumptions.
