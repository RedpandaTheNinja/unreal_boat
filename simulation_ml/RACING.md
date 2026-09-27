# Racing development baseline

Objective: minimize valid lap time while keeping the whole car inside the track
and avoiding obstacles. A collision or boundary violation invalidates the run;
a faster invalid lap does not beat a slower valid one. The initial controller
drives forward only and stops when no candidate supports its stopping distance.

## Run it

From the Unreal project root:

```powershell
& 'C:/Users/kevin/.conda/envs/ungpt/python.exe' simulation_ml/unreal_fable/python/examples/run_scenario.py car_track_oval --controller racing --seeds 3 --log simulation_ml/test-results/my-racing-run/
```

`--controller racing` enables strict track scoring automatically. Use
`--max-speed 4.0` to set a speed ceiling in m/s. The default ceiling is 85% of
the vehicle's configured maximum; curvature and braking can reduce it further.
This is an experimental baseline, not a claim to have found the fastest possible
racing line or tuned the maximum usable speed.

Run the reproducible baseline comparison, passing, and blocked-lane cases:

```powershell
& 'C:/Users/kevin/.conda/envs/ungpt/python.exe' simulation_ml/scripts/benchmark_racing.py --seeds 3
```

## Verified results, 2026-09-14

All cases use the Python mock backend, seeds 0–2, and conservative footprint
checks. Evidence: `test-results/racing/benchmark.json` and the accompanying CSVs.

| Case | Outcome | Two-lap time | Minimum track clearance across runs |
|---|---|---|---|
| Original pure pursuit, 2.2 m oval | 3/3 completed | 71.58–71.60 s | 0.637 m |
| Racing controller, same 2.2 m oval | 3/3 completed | 44.66 s | 0.637 m |
| Barrel passing, 3.2 m oval | 3/3 completed | 43.08–43.54 s | 0.408 m |
| Wall blocking the 3.2 m lane | 3/3 stopped without contact or boundary exit | No completed lap | 1.137 m |

The same-track comparison reduces elapsed time by about 37.6%. Passing uses a
wider track and is not a like-for-like speed comparison. The blocked test lasts
20 simulated seconds and checks zero final speed and positive clearance; its
lap success remains false. Original scene JSON files are unchanged.

Regression command:

```powershell
& 'C:/Users/kevin/.conda/envs/ungpt/python.exe' -m pytest simulation_ml/tests simulation_ml/unreal_fable/python/tests -q
```

Result: **36 passed**. Tests include crossing continuity, lap wraparound,
teleport rejection, invalid finishes, missing/held stale lidar, passing, and
stopping before a fully blocked lane, plus the existing kit/MCP regressions.

## What changed

- `geometry.py`: a progress-constrained projection avoids switching to a
  distant route segment at crossings. Pure pursuit and lap scoring use it.
- `controllers/racing.py`: samples lateral paths inside the known corridor,
  checks lidar returns with an inflated vehicle envelope, limits speed for
  upcoming curvature and braking distance, and brakes when no candidate fits.
  Held scans retain their acquisition pose in the mock backend.
- `sim.py`: optional strict scoring removes the old boundary tolerance from
  racing validation. A circumscribed circle encloses the car footprint.
  Collision failure takes precedence over finishing on the same observation.
- `mock_sim.py`: reports ground-truth conservative footprint-to-obstacle
  clearance to the evaluator. The controller does not use this ground truth.
- `examples/run_scenario.py`: adds `--controller racing`, `--max-speed`, and
  `--strict-track`; the original baseline remains the default.

## Remaining work

1. **Resolve the original figure-eight.** Continuous projection fixes branch
   switching, but not the full scenario. The new planner stopped near the
   crossing and timed out at seed 0 instead of finishing. A diagnostic with
   borders and crossing markers removed completed, which narrows the issue
   to obstacle geometry/avoidance; that altered case is not counted as a pass
   of the original. Inspect the border geometry and replace the conservative
   circle with oriented footprint checks where necessary. Do not remove
   genuine obstacles merely to obtain a passing score.
2. **Improve trajectory feasibility.** Current candidates are sampled paths,
   not full actuator-delay/steering-rate rollouts. Add a predictive tracking
   model and swept-footprint checks, including combinations of braking and
   cornering. Point-cloud checks do not establish that occluded space is free.
3. **Optimize only after validation.** Sweep speed/clearance settings on fixed
   seeds, then validate the chosen settings on held-out obstacle positions,
   friction, delays and sensor errors. Add racing-line optimization and measured
   per-lap splits; present numbers are elapsed time for the two-lap task.
4. **Add timestamped perception and dynamic obstacles.** The mock's held-scan
   identity supports the stale-scan check. Reconstructed UDP observations need
   an explicit sensor acquisition timestamp; a fresh wrapper is not proof of a
   fresh scan. The current planner assumes stationary obstacles and known pose.
5. **Verify Unreal physics and calibrate the car.** FableBridge/car assets still
   need configuration and compilation. The strict obstacle-footprint ground
   truth field is currently supplied by the mock only; it must be implemented
   or replaced by equivalent contact checks before asserting backend parity.
   No Unreal driving or real-car performance was validated in this work.

Track safety is checked at simulation observations, not formally guaranteed
between them. Three seeds and these small scenarios establish a development
baseline, not robustness across arbitrary tracks or obstacles.
