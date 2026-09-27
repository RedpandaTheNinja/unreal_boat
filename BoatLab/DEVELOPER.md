# BoatLab developer guide - navigation logic, sensors, and how to extend it

For people changing the autonomy code. How to *run* things (launchers, Unreal, dashboard) is in `GUIDE.md`.
This describes the code as of 2026-09-27. Paths are relative to `BoatLab\`.

Contents: 1 Architecture - 2 One control tick - 3 Sensor package and rates - 4 Current navigation logic -
5 Implementing a new navigation algorithm - 6 Which file to edit - 7 Development workflow - 8 Known gaps

---

## 1. Architecture

```
                 config/  boat.json | course_aimm_2025.json | vision_colors*.json | missions/*.json
                                              |
+---------------------------------------------v----------------------------------------------+
| boatnav/runner.py - one loop at 20 Hz:                                                      |
|   hal.state() -> [camera -> detector -> buoy map] -> mission executive -> hal.command()      |
+------+--------------------+-------------------+----------------------------+-----------------+
       |                    |                   |                            |
   boatnav/hal/        perception/         perception/                mission/                  evaluation.py
   backend             detector.py         mapping.py                 executive.py  tasks.py    (judge - sim
   twin | unreal |     (colour blobs ->    (buoy map: prior +         behaviors.py  context.py   truth only)
   real                 Detections)         Kalman per object)              |
       |                                                          planning/  grid.py (A*), course.py (rules),
       |                                                                     approach.py (stand-off / contact)
       |                                                          control/   pure_pursuit.py, hold.py
       |                                                          model.py   boat dynamics (twin + feed-forward)
       v
 twin: Python physics + renderer | unreal: BoatPhysics plugin over UDP 7450 | real: Jetson -> Arduino, GNSS, camera
```

Two rules the whole design depends on:
- **Nothing above `hal/` knows which backend is running.** The same perception, planning, control and mission code
  runs on the Python twin, in Unreal and on the Jetson.
- **Simulation truth (`hal.truth()`) only goes to the judge (`evaluation.py`)**, never into autonomy. Autonomy only
  sees `hal.state()` (with the GNSS error model in simulation) and camera frames.

---

## 2. One control tick (`boatnav/runner.py`, `Runner.run`)

Every 50 ms (`boat.json` > `control.rate_hz` = 20):

1. `st = hal.state()` -> `BoatState` (ENU pose, body velocities, yaw rate, `valid`, `source`).
   If the backend clock went backwards (dashboard "Reset sim", Unreal Play restarted) the time-based schedules below are re-armed.
2. If a camera frame is due (`--camera-hz`, default 4 Hz): `frame = hal.camera()`; in simulation `frame.pose` is replaced
   by the *navigation* pose at the frame's capture time (from `nav_hist`, the last ~10 s of states);
   `dets = detector.detect(frame)`; `bmap.update(dets, t)`.
3. Operator commands from the dashboard (pause, resume, skip, abort, reset, environment, identify colour).
4. If `st.valid`: `(left, right) = executive.step(st)` - the running task generator yields one motor command pair.
   If not valid (GNSS lost, Unreal paused): motors neutral.
5. `hal.command(left, right)`; judge update (simulation); a log row every 0.2 s (`runs/*.jsonl`);
   telemetry to the dashboard every 0.1 s.
6. `hal.tick(dt)` (the twin integrates physics at 1/120 s steps); sleep to hold real time (twin `--realtime`, Unreal, real).

---

## 3. Sensor package and rates

### 3.1 What each backend provides

| Channel | Twin (`hal/twin.py`, `hal/render.py`) | Unreal (`hal/unreal.py` + BoatPhysics v2) | Real boat (`hal/real.py`, planned hardware) |
|---|---|---|---|
| Position + heading | truth + GNSS error model (`boat.json` > `gps`): 2 cm RTK-like bias random walk (120 s correlation) + small jitter, 0.5 deg heading noise; every tick (20 Hz) | plugin truth over UDP + the same error model; ~20 Hz (every motor-command reply carries the state) | Cube Orange+ with Here4 RTK over MAVLink: `GLOBAL_POSITION_INT` + `ATTITUDE`, requested at 20 Hz. Or NMEA GPS (`GGA`/`RMC`/`HDT`, 9600 baud, usually 1-10 Hz). State is invalid if the fix is older than 1 s |
| Velocity, yaw rate | truth | plugin (body velocity, angular velocity) | MAVLink `vx/vy` + `ATTITUDE.yawspeed`; NMEA gives speed over ground only |
| Camera RGB | Python ray-caster, 640x480, 90 deg HFOV | Unreal SceneCapture, 640x480 JPEG q90 | USB webcam via OpenCV (`real.camera_index`) or ZED2i left image (`real.use_zed`) |
| Depth | exact per pixel | SceneCapture depth, 320x240 16-bit mm, upsampled to 640x480 | ZED2i depth; webcam: none (range from waterline / apparent size) |
| Camera rate | sampled by the runner at `--camera-hz` (4 Hz; 2 Hz in `10_Fast_Twin_Test`) | HAL grab thread 4 Hz (`camera_period` 0.25 s), plugin cap 10 Hz (`MaxHz`) | grabbed on demand at `--camera-hz` |
| Camera mount | `boat.json` > `camera`: FLU (1.5, 0, 0.8) m, 8 deg down, 0.925 m above water | same (BoatDynamics > Boat\|Camera, mirrored in `boat.json`) | measure and set in `boat.json` > `camera` |
| Motors | 2 fixed stern thrusters, commands in [-1, 1] at 20 Hz; delay 0.08 s, lag 0.35 s, deadband 0.06, reverse 60 %, neutral after 0.5 s silence | same parameters in the BoatDynamics component | Jetson -> Arduino serial 115200 baud `M,<l>,<r>` (+-1000) at 20 Hz; Arduino watchdog neutral after 500 ms |
| Payload (deploy / launch / recover) | outcome simulated in Python (`evaluation.simulate_actuator`) | simulated in Python from the Unreal truth pose; plugin v2 drops landing markers and hides the recovered case | serial `A,DEPLOY` \| `A,LAUNCH` \| `A,RECOVER` -> servo, reply `ACK,<NAME>` |
| Physics | 3-DOF (surge, sway, yaw) model `model.py`, 120 Hz | Chaos rigid body with buoyancy cells, async physics at a fixed 120 Hz (`DefaultEngine.ini`) | the lake |
| IMU, LiDAR, radar / IR | not modelled | not modelled | not wired (the Cube has an IMU; see 5.3 and 5.5) |

### 3.2 Timing at a glance

| Loop | Rate | Where |
|---|---|---|
| Physics (twin and Unreal) | 120 Hz | `hal/twin.py` `PHYS_DT`, Unreal `AsyncFixedTimeStepSize` |
| Control (state -> command) | 20 Hz | `boat.json` > `control.rate_hz` |
| Motor timeout | 0.5 s (10 missed ticks) | plugin `CommandTimeoutS`, Arduino `WATCHDOG_MS` |
| Camera + detection + map update | 4 Hz | `--camera-hz`, `hal/unreal.py` `camera_period`, `BoatCameraSensor.h` `MaxHz` |
| Telemetry to dashboard / browser poll | 10 Hz / 5 Hz | `runner.py`, `dashboard/dashboard.js` |
| Log rows | 5 Hz | `runner.py` `_log_row` |

### 3.3 Frames and units (used everywhere)
- World: local ENU metres, x east, y north, heading in radians counter-clockwise from east. GPS lat/lon <-> local via `geo.LocalFrame` and `course.geo_origin`.
- Body: FLU (x forward, y left/port, z up). Starboard is negative y. Motors: left = port.
- Unreal: `x = UE_X / 100`, `y = -UE_Y / 100`, `heading = -radians(UE yaw)`. This conversion appears only in `hal/unreal.py` and `unreal/Content/Python/boatlab_course.py`.

---

## 4. Current navigation logic

### 4.1 Navigation state
There is **no state estimator yet**: the pose the HAL returns is used directly (simulation: truth + GNSS error model;
real boat: the Cube's own EKF output or raw NMEA). `runner.nav_hist` only serves to place camera detections at
the frame's capture time. See 5.3 for adding an estimator.

### 4.2 Perception (`perception/detector.py`, `ColorBlobDetector`)
RGB -> HSV -> per-colour masks (`config/vision_colors*.json`) -> morphology -> connected components -> shape split
(tall orange = buoy, flat orange = launch target; tall yellow = buoy, wide yellow = case) -> size-vs-range check ->
range from depth (ZED2i / Unreal), else from the waterline row on flat water, else from apparent height -> bearing
-> world (x, y) using the frame pose. Output: `types.Detection` (label, bearing, range, range source, world x/y).
Status: good on the twin (`tools/eval_vision.py`: precision 0.9-1.0 for gate, slalom and identify colours, lower
for black / case / target), **not usable in Unreal yet** (water reads as blue, see `GUIDE.md` 3.4).

### 4.3 Buoy map (`perception/mapping.py`, `BuoyMap`)
One track per course object, starting at its course position with sigma 1 m (4 m in `vision` mode). Each detection
is associated to the nearest compatible track and fused with a scalar Kalman update (measurement sigma about 3 % of
range with depth, 7 % from the waterline, 20 % from size, plus 5 cm). Tracks drift with process noise 0.08 m/sqrt(s)
so they can follow a slowly wandering GPS bias. Unmatched detections seen 3+ times become new obstacles.
Modes: `prior` (course positions only, vision displayed), `fused` (default), `vision` (course positions are only search hints).

### 4.4 Planning (`planning/`)
- `grid.py` `GridPlanner`: 0.25 m grid; blocked = shore band (half-beam + 0.8 m), marina keepouts and buoys grown by
  half-beam + 0.6 m; soft cost ring around buoys; Gaussian cost around LiDAR detectors. A* (8-neighbour, cost-weighted),
  then string-pulling (drop points while the straight line stays free and <= 5 % costlier), then 2 m corner fillets.
- `course.py`: rule geometry. Gate crossing direction = rot90(red - green) (red stays to starboard); slalom
  waypoints 1.6 m beside each buoy on the rule side; channel corridor with virtual bank walls.
- `approach.py`: stand-off poses for deploy / launch / recover (`p = T - R(theta) * b` over 24 headings, hull-clear,
  scored) and bow-first contact lines for identify / return.
- Paths become `geometry.Path` (polyline + per-point speed; stop profile `v <= sqrt(2 a d)` with a = 0.2 m/s^2).
  Replanned when a newly detected object blocks the next 10 m.

### 4.5 Control (`control/`, `model.py`)
- `pure_pursuit.py`: progress `s` found in a forward window (never jumps branches), look-ahead
  `Ld = clamp(1.5 s x speed + 1.75 m, 3.5, 7.0)`, point on the path at `s + Ld`, angle alpha;
  differential `d` = curvature feed-forward (through `model.py`) + 2.5 alpha - 2.0 yaw rate; surge `c` = feed-forward + PI
  with target speed x max(0.35, cos^2 alpha); `left = c - d`, `right = c + d`, turn has priority. Pivot in place when
  |alpha| > 70 deg (until < 25 deg); reverse legs track with the stern leading.
- `hold.py`: station keeping at a pose (surge PD with forward/reverse thrust, heading PD with differential thrust);
  lateral error cannot be corrected with fixed stern motors, so beyond 0.9 m the task backs off and re-approaches.
- `model.py`: the same planar equations as the Unreal plugin (added mass, linear + quadratic drag, motor lag, delay,
  deadband, reverse ratio). Used by the twin simulator and by the controller feed-forward.

### 4.6 Mission (`mission/`)
- `executive.py` runs the tasks of `config/missions/<name>.json` in order, with per-task time budgets
  (`time_budget_s`), pause / skip / abort from the dashboard.
- `tasks.py`: one Python generator per challenge (`REGISTRY`: gate, slalom, channel, identify, deploy, launch, recover,
  return, dock, waypoints, hold). A generator `yield`s one `(left, right)` per tick and `return`s a result dict.
- `behaviors.py`: shared building blocks - `transit_to` (A* + follow + replan), `follow` (pure pursuit until done /
  timeout / blocked), `hold`, `observe`, `ensure_turn_room`.
- `context.py` (`Ctx`): the objects every task uses (`planner`, `pp`, `hold_ctl`, `model`, `bmap`, `hal`, config, logging).

### 4.7 Judge (`evaluation.py`)
Scores simulation truth against the 2025 rules (gate full pass and direction, slalom sides, detector exposure,
contacts, deploy / launch distances, recover, return) plus shore and pier/boat contacts. Written to `runs/*_summary.json`.

---

## 5. Implementing a new navigation algorithm

General recipe for any replacement: **keep the interface, add the new class next to the old one, select it with a
config key, compare both on the same scenarios** (section 7). The interfaces below are what the rest of the code calls.

### 5.1 Path planner (e.g. Hybrid A*, state lattice, RRT*)
Interface used by the mission code (duck-typed on `GridPlanner`):

| Method | Used by |
|---|---|
| `plan(start, goal, obstacles, detectors=(), smooth_radius=2.0, free_goal_r=0.0) -> list[(x, y)] or None` | `behaviors.plan_points` / `transit_to`, `tasks.waypoints_task` |
| `build(obstacles, detectors=(), extra_clear=0.0)`, `free(x, y)`, `line_clear(a, b)` | `approach.py`, `behaviors.py`, `tasks.py` |
| `blocked`, `X`, `Y`, `astar(start, goal)` | `tasks.channel_task` (corridor planning) |

`obstacles` = iterable of `(x, y, radius)` from `ctx.objects()` (the buoy map); `detectors` = `(x, y, detect_radius)`;
water bounds and marina keepouts come from the course file.

Steps:
1. New module, e.g. `boatnav/planning/hybrid_astar.py`, class `HybridAStarPlanner(GridPlanner)` - inherit so the
   occupancy grid, `free` and `line_clear` stay; override `plan` (and `astar` if the channel should use it).
   Return points no more than ~0.5 m apart with continuous heading; pure pursuit follows any polyline.
2. Select it in `mission/context.py` (`self.planner = ...`) from a new key `boat.json` > `planning.algorithm`.
3. Speeds are attached in `behaviors.transit_to` (`G.Path(points, speeds)`); curvature-dependent speed limits go there
   or into the planner output.
4. Tests: a planning unit test in `tests/test_boatlab.py` (e.g. start/goal around the marina), then section 7.

### 5.2 Path tracker / controller (e.g. LOS guidance with integral, Stanley, MPC)
Interface (as `PurePursuit`, held in `ctx.pp`):
- `set_path(path: geometry.Path, stop_at_end: bool)`
- `update(st: BoatState, dt: float) -> (left, right)` in [-1, 1]
- `.info` (`PPInfo`): `mode, s, length, cross_track, alpha, lookahead, lookahead_m, u_des, r_des, remaining, done`.
  `behaviors.follow` ends a leg when `info.done` is true; the dashboard draws `lookahead` and shows the rest.

Steps: new class in `control/` (reuse `model.BoatModel.feedforward(u_des, r_des)` and `pure_pursuit.mix()`);
select in `mission/context.py` from `boat.json` > `control.tracker`; add a tracking test like
`test_pure_pursuit_converges_on_straight_line`; compare cross-track error in `runs/*.jsonl` (`pp.xte`).
Good first step for this boat: LOS guidance with an integral term, which compensates current/wind drift that pure
pursuit only partly rejects.

### 5.3 State estimator (EKF / UKF fusing GNSS + IMU + visual odometry)
Not present today (4.1). Suggested shape:
1. New package `boatnav/estimation/` with e.g. `ekf.py`: `update(raw: BoatState, imu, dt) -> BoatState` (same fields,
   `source="ekf"`), plus `reset(state)`.
2. IMU channel: add `imu()` to `hal/base.py` (default `None`) and a dataclass in `types.py`; implement in
   `hal/twin.py` (truth + noise/bias), `hal/unreal.py` (the plugin state already carries attitude and angular
   velocity), `hal/real.py` (MAVLink `ATTITUDE` / `RAW_IMU`, request ~50 Hz).
3. In `runner.run`, right after `hal.state()`: `st = self.estimator.update(raw, imu, dt)`; call `reset` where the
   runner re-arms schedules after a clock jump.
4. Evaluate on the twin with plain GPS (`--gps-noise 1.5`, `--current=-0.2,0.1`): log `raw`, `ekf` and `truth` and
   compare. Never feed `truth` into the filter.

### 5.4 Detector (e.g. YOLO / ONNX / TensorRT on the Jetson)
Interface (as `ColorBlobDetector`, held in `runner.detector`):
- `detect(frame: CameraFrame) -> list[Detection]` - labels from the course colour classes (red, green, blue, orange,
  purple, yellow, black, zebra, pink, case, target), `bearing`, `range_m`, `range_source`, and world `x, y`
  computed with `frame.pose` (see `ColorBlobDetector._locate` for the camera geometry).
- `horizon_row(frame)` - used to annotate the dashboard image.

Steps: new class in `perception/`; select in `runner.py` (`self.detector = ...`) from a key in the vision config or a
CLI flag. Training / test data: `tools/tune_colors.py --save <folder>` records frames with poses (Unreal or real);
course positions projected into the image (`tune_colors.project`) give automatic labels. Score with
`tools/eval_vision.py --folder <folder>` (precision, localisation error per label).

### 5.5 New sensor (LiDAR, IMU, radar / IR for the PDF bonus)
1. Dataclass in `types.py` (e.g. `ScanFrame`).
2. Method on `hal/base.py` `BoatInterface` returning `None` by default (e.g. `lidar()`).
3. Twin: simulate in `hal/twin.py` / `hal/render.py` (ray casts against course objects, marina keepouts, shore).
4. Unreal: C++ component in `unreal/BoatPhysics/Source/...` following `BoatCameraSensor` (created in
   `BoatDynamicsComponent::BeginPlay`, captured on request, replied over UDP 7450; LiDAR = line traces), then parse
   it in `hal/unreal.py`. Rebuild with `04_Build_Plugin.cmd` (build runs off Google Drive).
5. Real: driver thread in `hal/real.py` (keep the latest sample under a lock, like the MAVLink thread).
6. Consumer: e.g. `perception/lidar_detector.py` producing `Detection`s (`range_source="lidar"`) into
   `bmap.update`, or a separate obstacle layer passed to the planner.
7. Schedule it in `runner.run` with its own rate (copy the `next_cam` pattern, including the clock-reset re-arm).

### 5.6 New or changed behaviour (task)
Write a generator in `mission/tasks.py` (`transit_to`, `follow`, `hold`, `observe`, `pivot_to`, `back_off`, `depart`
are already imported there from `behaviors.py`):
```python
def my_task(ctx, target="zebra", standoff_m=5.0, **_):
    x, y = ctx.bmap.position(target)                            # live buoy-map estimate
    goal = (x + standoff_m, y)
    st = yield from transit_to(ctx, goal, stop=True, label=f"to {target}")   # yields (left, right) each tick
    if st != "done":
        return {"status": st}
    res = yield from hold(ctx, (goal[0], goal[1], math.pi), timeout=10, label="hold facing west")
    return {"status": "done", "hold": res}
```
Register it in `REGISTRY` (`"my_task": (my_task, <challenge number or None>)`), add it to a mission JSON in
`config/missions/`, and give it a time budget (`time_budget_s`). Scoring for a new challenge goes in `evaluation.py`.

### 5.7 Mapping / GPS-agnostic navigation
`BuoyMap` is used through `update(dets, t)`, `get(id)`, `position(id)`, `find_label(label)`, `unexpected()` and
`ctx.objects()`. The `vision` mode already treats course positions as coarse hints. Relative navigation for the
final approaches exists (`RelTarget` in `tasks.py`: targets kept relative to the boat so a GPS bias does not matter);
extend that pattern for a full GPS-free run.

---

## 6. Which file to edit

| I want to change... | Edit | Also |
|---|---|---|
| speeds, gains, look-ahead, pivot thresholds | `config/boat.json` > `control` | `10_Fast_Twin_Test` |
| clearances, grid, slalom offset, detector cost | `config/boat.json` > `planning` | |
| mechanism mount points / offsets | `config/boat.json` > `mechanisms` | `standoff_tests` mission |
| camera mount / FOV / resolution | `config/boat.json` > `camera` | Unreal: BoatDynamics > Boat\|Camera |
| real-boat ports, MAVLink / NMEA, camera | `config/boat.json` > `real` | `hal/real.py` |
| boat dynamics | `boatnav/model.py` + `boat.json` > `dynamics` | keep in sync with the plugin C++; `test_twin_matches_unreal_step_response` |
| course layout, lake size, marina | `tools/make_course.py` -> run it -> `config/course_aimm_2025.json` | Unreal: MCP `build_course` / `05_Build_Course_In_Unreal.cmd` |
| vision colours | `config/vision_colors.json` (twin), `config/vision_colors_unreal.json` (Unreal) | `tools/tune_colors.py`, `tools/eval_vision.py` |
| task order, colours, time budgets | `config/missions/*.json` | |
| control loop, rates, camera schedule, telemetry | `boatnav/runner.py` | |
| path planning algorithm | `boatnav/planning/grid.py` or a new module | `boatnav/mission/context.py` |
| gate / slalom / channel rule paths | `boatnav/planning/course.py` | `boatnav/mission/tasks.py` |
| stand-off and contact approaches | `boatnav/planning/approach.py` | `tasks.py` (`final_leg`, `hold_locked`) |
| path tracking | `boatnav/control/pure_pursuit.py` or a new module | `mission/context.py` |
| station keeping | `boatnav/control/hold.py` | |
| detector | `boatnav/perception/detector.py` or a new module | `runner.py` |
| map fusion | `boatnav/perception/mapping.py` | `mission/context.py` (`objects`) |
| state estimation | new `boatnav/estimation/` | `runner.py`, `hal/*` (IMU) |
| mission behaviours | `boatnav/mission/tasks.py`, `behaviors.py`, `executive.py` | `config/missions/*.json` |
| twin physics or synthetic camera | `boatnav/hal/twin.py`, `boatnav/hal/render.py` | |
| Unreal link | `boatnav/hal/unreal.py` | plugin C++ `unreal/BoatPhysics/`, `04_Build_Plugin.cmd` |
| real boat | `boatnav/hal/real.py` | `arduino/boat_motor_bridge/boat_motor_bridge.ino` |
| scoring | `boatnav/evaluation.py` | |
| dashboard | `dashboard/server.py`, `index.html`, `dashboard.js` | |
| Unreal level building / MCP tools | `unreal/Content/Python/boatlab_course.py`, `boatlab_toolset.py` | |
| launchers | `Run.ps1`, `*.cmd` | |
| tests | `tests/test_boatlab.py` | |

---

## 7. Development workflow

1. Make the change behind a config switch; add or extend a test in `tests/test_boatlab.py`.
2. `11_Tests.cmd` (about 1 minute; all must pass).
3. `10_Fast_Twin_Test.cmd`: full mission as fast as possible, prints the score, opens the plot. Compare with the
   previous `runs/*_summary.json`.
4. Robustness on the twin (command line from `BoatLab`, `python -m boatnav.runner --backend twin --api-port 0 ...`):
   `--gps-noise 1.5` (plain GPS), `--current=-0.2,0.1`, `--wind=...`, `--fog 25`, `--nav vision`, several `--seed` values.
5. Unreal: `08_Unreal_Mission.cmd` (uses `--nav prior` until the Unreal colours are tuned).
6. Real boat: bench dry run first (`GUIDE.md` section 7).

What to compare (from `runs/*_summary.json` > `score`): `passed_count`, `contacts` (only `identify_<colour>` and
`return_blue` are intended), `keepout_contact_samples`, `shore_contact_samples`, `min_clearance_to_non_target_m`,
`3_evade.detector_exposure_s`, `5_deploy.distance_m`, `6_launch.distance_m`, `elapsed_s`.
Per-tick data (`runs/*.jsonl`, 5 Hz): `t, x, y, heading, u, r, cmd, phase, task, pp{mode, xte}, truth`, plus each new
`path` and `approach`. `tools/plot_run.py <log> [--zoom channel]` draws it.

Pitfalls:
- Never use `hal.truth()` in autonomy code.
- Anything scheduled in sim time must survive the clock going backwards (reset, Play restart) - see `runner.run`.
- Motor commands must be refreshed within 0.5 s or the motors go neutral (plugin and Arduino).
- `model.py`, `boat.json` > `dynamics` and the plugin C++ must stay consistent, or the twin stops matching Unreal.
- Unreal edits from Python must call `Modify()` and save the actors' packages (World Partition), as `boatlab_course.py` does.
- Keep the Python venv and C++ builds off Google Drive (`%LOCALAPPDATA%\BoatLab\venv`, `%TEMP%\boatlab_build`).

---

## 8. Known gaps (candidates for future work)

- No state estimator; the real boat relies on the Cube's EKF (heading from its compass unless dual-antenna GNSS is added).
- Vision is not tuned for Unreal (water trips the blue class); the real course will need re-tuning or a trained detector.
- LiDAR detector reach is an assumption (6 m).
- `hal/real.py` has not run on hardware; the motor controller model is still unknown (Arduino sketch is a template).
- Challenge 8 (Receive: read the deployed sensor wirelessly from the dock) is not implemented.
- Fixed stern thrusters cannot correct lateral error while holding; stand-offs re-approach instead.
- The PDF's IR / radar navigation bonus is not addressed (no such sensor modelled).
