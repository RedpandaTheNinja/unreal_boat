# Car simulation assessment — 2026-09-14

Update: the selected objective is **fastest valid laps while staying inside the
track and avoiding obstacles**. An experimental racing controller and stricter
scoring are now implemented. See [RACING.md](RACING.md) for current commands,
measured improvements, tests, and remaining limitations. The assessment below
records the original inspection and baseline.

## What was brought up

The original car controller ran on the Python mock backend using the existing
`C:/Users/kevin/.conda/envs/ungpt/python.exe`. CSV trajectories are in
`test-results/current-car/`. The inline oval replay uses seed 0, sampled every
0.2 seconds, with the final sample preserved.

The open Unreal map `/Game/ThirdPerson/Lvl_ThirdPerson` now contains an oval
preview in Outliner folder `Fable_Oval_Preview`, positioned 100 m east of the
original scene. It has a ground slab, cone borders sampled from the oval JSON,
and a static car proxy. This is a visual preview, not an Unreal physics run.
The original scene remains present. The map save tool returned success.

## Measured baseline (mock, seeds 0–2)

| Scenario | Passed | Time (s) | Cross-track RMS (m) | Collisions |
|---|---:|---:|---:|---:|
| Oval, two laps | 3/3 | 71.58–71.60 | 0.029–0.032 | 0 |
| Gravel slalom | 3/3 | 32.96–35.02 | 0.313–0.377 | 0 |
| Figure-eight digital twin | 0/3 | 11.94–13.58 | 1.101–1.237 | 0 |

All figure-eight episodes ended `off_track`. Seeds 1 and 2 reported negative
progress. These are three seeds per world, not a robustness certification.
Slalom waypoint success does not prove containment within the track: its
maximum cross-track deviations were 1.848–2.280 m.

## Next development work, in order

1. **Repair route continuity and scoring.** `Polyline.project()` selects the
   globally nearest segment without previous progress or heading. Both
   `PurePursuit` and `TaskEvaluator` use it. At self-intersections this can select
   another branch. Track a segment/progress window and heading consistency in
   both places; add a figure-eight crossing regression. This is a code-level
   suspect, not a proven sole cause of the observed failures. Isolate reactive
   avoidance, spawn alignment, and speed in separate diagnostic runs.
2. **Define free space.** Derive a local obstacle grid from lidar and the track
   corridor. Include the complete car footprint and clearance margin. Specify
   whether leaving the corridor or reversing is allowed. Preserve observation
   timestamps and distinguish estimated pose from simulator ground truth.
3. **Add a planner between perception and tracking.** Plan a collision-free
   route that respects steering limits, then pass it to a tracker. Hybrid A*
   is a suitable candidate for Ackermann steering; Nav2's Smac implementation
   supports car-like kinematics and footprint collision checking. Start with
   known stationary obstacles and a fixed goal, then local replanning. Source:
   https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/navigation_plugins/setup_navigation_plugins/
4. **Handle speed and stopping.** Check each proposed trajectory for collisions,
   steering rate, curvature, braking distance and command delay. Stop when no
   feasible trajectory exists or sensor data is stale. The present reactive
   avoidance only biases steering and reduces throttle; it is not a planner.
5. **Expand evaluation.** Add a blocked centerline with a valid detour, a fully
   blocked lane requiring a stop, crossing-branch tests, tight turns, and sensor
   dropout. Score footprint boundary violations, collisions, route completion,
   minimum clearance, replanning latency, and stopping distance. Run fixed seed
   comparisons before randomized batches.
6. **Close the Unreal loop.** Install and compile the supplied FableBridge plugin
   in this project; enable its required plugins; create BP_FableBridge and
   BP_FableCarPawn with a wheel-bone skeletal mesh and physics asset. Copy the
   builder scripts into Content/Python. Follow `unreal_fable/docs/UNREAL_SETUP.md`
   for fixed timestep and bridge configuration. Build the real scenario and
   verify UDP observations/actions on port 9800 before calling a run validated
   against Unreal. The bundled C++ is explicitly documented as uncompiled.
7. **Calibrate before transferring results to a car.** Measure wheelbase,
   footprint, steering range/rate, actuator latency, braking and surface grip.
   The default vehicle JSON explicitly contains placeholder RC-car values.

## Information needed from you for the next implementation

- Is the task fastest-lap driving, staying in a marked corridor, or reaching a
  destination around obstacles? May the car reverse or leave the track?
- Which physical car dimensions and sensors should the simulation match?
- Is there an existing Unreal car skeletal mesh/physics asset to use?

## Repeat the baseline from the project root

```powershell
& 'C:/Users/kevin/.conda/envs/ungpt/python.exe' simulation_ml/unreal_fable/python/examples/run_scenario.py car_track_oval --seeds 3 --log simulation_ml/test-results/current-car/
```

Replace `car_track_oval` with `car_track_slalom_gravel` or
`car_figure8_digital_twin`. The copied `simulation_ml/codex-config.toml` still
points at the old Downloads location; the current root configuration uses
Unreal's built-in MCP instead. No model or MCP configuration was changed.
