# BoatLab - AIMM-ICC autonomous boat: step-by-step guide

BoatLab lives next to RacingLab in `mcp_gpt_boat/BoatLab`. It gives you:

- **A portable Python autonomy stack** (`boatnav/`): GPS/vision buoy map, A* planner, rule-aware gate/slalom/channel paths, stand-off approaches for deploy/launch/recover, pure pursuit with twin-motor mixing (pivot turns with one motor forward and one in reverse, reverse legs), and a mission state machine for challenges 1-7 and 9.
- **Three interchangeable backends** behind one interface: the Python twin (same equations as your Unreal BoatPhysics plugin, no Unreal needed), Unreal 5.8 over the BoatPhysics UDP port, and the real boat (Jetson + Arduino serial + GPS/MAVLink + webcam/ZED2i).
- **A dashboard** in the RacingLab style: course map, planned path, look-ahead point, camera with detections, motors, mission control, simulated judging.
- **Unreal pieces**: BoatPhysics plugin v2 (adds an RGB + depth camera), a course builder that places the AIMM course in `Lake_300x80ft`, and an Unreal MCP toolset.

Everything is driven from numbered launchers in this folder (double-click), or `START.cmd` for a menu.
How the navigation code works, the sensors and their rates, and where to plug in new algorithms: **`DEVELOPER.md`**.

---

## 1. One-time setup (Windows)

1. **Python 3.12** (or 3.11) from python.org with the "py launcher" option. RacingLab's DQN already asked for this, so it is probably installed.
2. Double-click **`01_Setup.cmd`**. It creates `BoatLab\.venv` and installs `numpy`, `opencv-python`, `matplotlib`, `pyserial`, `pytest`, then runs the install check.
3. Double-click **`11_Tests.cmd`**. Expect `14 passed` (about a minute). These include: twin vs your Unreal step-response logs, pure pursuit convergence, pivot turns, rule sides for gates/slalom, stand-off geometry, detector accuracy, a full 8-challenge mission, the Unreal protocol against a fake plugin, and the dashboard server.
4. If Unreal is not in `C:/Program Files/Epic Games/UE_5.8`, edit `engine_root` in `BoatLab\settings.json` (or set `UE_58_ROOT`).

> Google Drive note: running from `G:\My Drive` works, but Drive can lock files while syncing. If you see "file in use" errors, pause Drive sync or copy the project to a local SSD (the previous handover notes recommend that for Unreal anyway).

---

## 2. First run without Unreal (Python twin)

1. Double-click **`07_Twin_Mission.cmd`**. The dashboard opens at <http://127.0.0.1:8770> and a console shows the mission log. The mission starts **paused**.
2. On the dashboard press **Start / resume**. The boat leaves the dock and runs: C1 Gate, C2 Dodge (slalom), C3 Evade (channel), C4 Identify (touch the blue buoy), C5 Deploy, C6 Launch, C7 Recover, C9 Return + dock. About 5 minutes.
3. Things to try while it runs:
   - **Identify colour** / **Judge random**: changes the colour for C4 before it starts.
   - **Click waypoints** then click the map, then **Go to waypoints**: pure pursuit through your points (obstacle-aware A* between them).
   - **Environment**: add a current (for example x = -0.2) and watch the hold phases fight drift.
   - **Pause / Skip task / Abort (neutral) / Reset sim**.
4. Close the console window (or `09_Stop.cmd`) to stop. Logs go to `BoatLab\runs\` (`*.jsonl` per run + `*_summary.json`).
5. **`10_Fast_Twin_Test.cmd`** runs the same mission as fast as possible (about 3 minutes wall time with the camera), prints the score and opens a plot of the run. Use it after every change.

What the judge on the dashboard means: it scores the **simulation truth** (not what the autonomy believes): full-hull gate passes and direction, slalom sides, time inside LiDAR detector zones, buoy contacts, deploy landing distance, launch landing, recover capture, return contact. It is a geometric approximation of the 2025 rules, not the real judges.

---

## 3. Unreal 5.8

### 3.1 Build the camera plugin (once)
The current BoatPhysics plugin has no camera. **v2** adds `UBoatCameraSensor` (640x480 RGB JPEG + 320x240 ideal depth in millimetres, same chunked UDP pattern that already works for your F1 car), `reset_pose`, and landing markers.

1. **Close the Unreal editor.**
2. Double-click **`04_Build_Plugin.cmd`**. It copies `BoatLab\unreal\BoatPhysics` to a short local path (`%TEMP%\boatlab_build\bls<time>`), runs `RunUAT BuildPlugin`, backs up your current plugin to `BoatLab\unreal\backups\BoatPhysics_<time>`, then installs Source + Binaries into `Plugins\BoatPhysics`. The build must not run on Google Drive: UBA cannot memory-map the 2.5 GB shared PCH there and the compiler crashes.
3. If the build fails, nothing is installed. v2 was compiled on 2026-09-27 against UE 5.8.2 (0 warnings, 0 errors) after fixing one shadowed variable; `-SkipInstall` builds without touching the installed plugin, so you can check a build while the editor is open.
4. Rollback: close the editor and copy the backup folder over `Plugins\BoatPhysics`.

New properties on the `BoatDynamics` component (Details panel, category Boat|Camera): `bEnableCamera`, `CameraMountCm` (150, 0, 80), `CameraPitchDownDeg` (8), `CameraHorizontalFovDeg` (90), resolution, `bCameraDepth`. If you change them, mirror the values in `config\boat.json` > `camera`.

### 3.2 Build the course in the level
1. Double-click **`05_Build_Course_In_Unreal.cmd`**. Unreal opens and runs `BoatLab\unreal\run_build_course.py`, which:
   - **sizes the lake to the course file** (`water` in `config\course_aimm_2025.json`, set by `lake_x_ft` / `lake_y_ft` in `tools\make_course.py`): now **350 x 200 ft**, x -190..+160 ft and y +/-100 ft around the course origin. The PDF gives no pool size (the course is in an open lake); this leaves about 60-70 ft of water north and south of the course and 55 ft west of the channel, like the satellite map. The water actor is renamed `Lake_Water_350ft_x_200ft`.
   - places every object from `config\course_aimm_2025.json`: 10 ft red/green gates, slalom, channel with pink detectors, identify cluster (blue/orange/purple/yellow, 3.2 ft apart as drawn on the map), zebra buoy, black buoy + the 10 ft launch trampoline (orange tube, white top, 8 ft black mat) 15 ft north of it, the 13 x 9.7 x 5.8 in yellow case, blue start/return buoys. All tagged `boatlab`, in the Outliner folder `BoatLab/Course`.
   - builds the **marina** from the satellite map (Outliner folder `BoatLab/Dock`, all colliding): a north-south pier to the north shore with 14 moored boats in slips, an east-west pier with 4 boats moored on its south side and a pontoon at its west end. It replaces the level's original stub pier (`Dock_Plank_*`, `Dock_Post`), and moves the boat to its slot on the north face of the east-west pier, bow west.
   - saves the level (edited actors are marked modified and their World Partition packages saved). Watch the Output Log for `[boatlab]`.
   - Naming note: the level's `Shore_North` actor sits at +UE Y, which is **south** in BoatLab's ENU frame (x = UE X, y = -UE Y). Only the label differs; nothing needs renaming.
2. Alternative from the editor's Python console (Output Log, switch "Cmd" to "Python"):
   ```python
   import sys; sys.path.append(r"G:/My Drive/Unreal projects/mcp_gpt_boat/BoatLab/unreal/Content/Python")
   import boatlab_course as bc; bc.build()      # bc.clear(), bc.describe(), bc.move_object("zebra", -9.0, -8.5)
   ```
3. Select `Boat_13ft_x_3ft_TwinMotors` > `BoatDynamics` and **untick Keyboard Control** for autonomous runs (keys would compete with the autonomy).

### 3.3 Check the connection
1. Press **Play** in the lake level.
2. Double-click **`14_Check_Unreal.cmd`**. You want `"plugin_version": "boatlab_v2"` and `"camera_available": true`.
3. **`06_Dashboard.cmd`** alone shows the boat read-only (pose, thrust, camera) even without a mission, like RacingLab's dashboard.

### 3.4 Tune the vision colours for Unreal
Unreal lighting and exposure change colours. With Play running:
Unreal uses its own colour file, `config\vision_colors_unreal.json` (the runner's Unreal launcher passes `--vision vision_colors_unreal.json`), so tuning for Unreal never changes the twin-tuned `vision_colors.json` that the tests and the twin use.
1. Double-click **`13_Tune_Colors_Unreal.cmd`**. It captures 25 frames, projects every known buoy into the image, samples its HSV and writes `config\vision_colors_unreal.proposed.json`. Drive the boat around between frames (keyboard or dashboard waypoints) so buoys are seen at several ranges; with plugin v2 you can also teleport it with `{"reset_pose": [x_ue_m, y_ue_m, yaw_ue_deg]}` on UDP 7450.
2. Review the printed table, then apply it: `python tools\tune_colors.py --backend unreal --frames 25 --vision vision_colors_unreal.json --apply` (keeps a dated backup of the old file).
3. Score the detector on the saved frames: `python tools\eval_vision.py --folder runs\frames_unreal --vision vision_colors_unreal.json` (precision, median/p90 position error per colour).

Status 2026-09-27 (64 frames from 32 teleport poses): geometry and depth are right (true detections within 4-13 cm), but the twin-tuned colours do not work in Unreal. The dark teal water (HSV about 88/230/30) falls inside the blue range (precision 0.03), sun glints read as zebra tape and dark water/shadows as black, and buoys seen towards the sun are near-black silhouettes, so the automatic HSV proposal is unusable. Needs: a fixed exposure on the scene capture (it is darker than the viewport), water rejection for blue, then re-tuning. Until then run Unreal missions with `--nav prior` (vision shown, not fused).

### 3.5 Run the mission in Unreal
1. Open the project with the lake level (`03_Open_Unreal.cmd`). Keyboard Control can stay on: keys only override the autonomy while pressed.
2. Double-click **`08_Unreal_Mission.cmd`**, then **Start / resume** on the dashboard. The launcher starts **Simulate** in the editor by itself through the editor's MCP server (`tools\unreal_play.py`; `--play` for Play in the viewport, `--stop` to stop). If it cannot (editor still loading, another level open), the runner waits up to 2 minutes for you to press Play instead of failing (`--ue-wait` seconds).
   Unreal runs use `--nav prior` by default until the Unreal colours are tuned (3.4); `Run.ps1 unreal -Nav fused` overrides.
3. Deploy/launch/recover outcomes are simulated in Python from the Unreal truth pose (Unreal has no payload physics); v2 drops a yellow marker where the sensor lands, an orange one for the football, and hides the recovered case.
4. The runner sends motor commands at 20 Hz; if it stops, the plugin neutralises the motors after 0.5 s.

Without Unreal you can still exercise this exact path: `START.cmd` > `fake` starts a fake plugin (twin physics + synthetic camera) on UDP 7450; then run `08_Unreal_Mission.cmd`. That is how the Unreal backend was tested here (8/8 challenges, no unintended contacts).

---

## 4. Connect the Unreal MCP server to Claude (optional)

Your project enables UE 5.8's built-in MCP server (`http://127.0.0.1:8000/mcp`, used by Codex before). To let Claude inspect and edit the level:

1. In the editor's Python console: `import sys; sys.path.append(r"G:/My Drive/Unreal projects/mcp_gpt_boat/BoatLab/unreal/Content/Python"); import boatlab_toolset`. Importing registers the toolset with the registry (the MCP catalog updates by itself; `importlib.reload(boatlab_toolset)` re-registers after edits). To make it permanent add `...\BoatLab\unreal\Content\Python\boatlab_toolset.py` under Project Settings > Plugins > Python > Startup Scripts.
   Toolset `boatlab_toolset.BoatLabCourseToolset`, tools: `build_course`, `clear_course`, `describe_course`, `move_course_object`, `set_boat_start`, `fit_lake`.
2. Claude desktop only launches local MCP servers as commands, so bridge the HTTP server with `mcp-remote` (needs Node.js). Claude desktop > Settings > Developer > Edit Config:
   ```json
   { "mcpServers": { "unreal": { "command": "npx", "args": ["-y", "mcp-remote", "http://127.0.0.1:8000/mcp"] } } }
   ```
   Restart Claude desktop with the editor open. In a chat linked to this computer, the Unreal tools then show up and I can call them directly. For Claude Code, `mcp_gpt_boat\.mcp.json` (written by `ModelContextProtocol.GenerateClientConfig`) is only read when Claude Code is started **in the `mcp_gpt_boat` folder**, not in `Unreal projects`.

---

## 5. How the algorithms work (and what to tune)

All tuning lives in `config\boat.json` unless noted.

**Frames.** Local ENU metres: x east, y north, heading counter-clockwise from east. Unreal: `x = UE_X/100`, `y = -UE_Y/100`, `heading = -yaw`. GPS lat/lon is converted with `geo_origin` in the course file. Body frame: x forward, y port.

**Pure pursuit + twin-motor mixing** (`boatnav/control/pure_pursuit.py`).
- Look-ahead distance `Ld = clamp(lookahead_time_s * speed + 0.5 * lookahead_min_m, lookahead_min_m, lookahead_max_m)`; the look-ahead point is found with a *progress window* so the tracker never jumps to another branch of a slalom or U-turn.
- Heading command: `d = feedforward(path curvature) + kp_heading * alpha - kd_heading * yaw_rate`, where alpha is the bearing to the look-ahead point. Speed: model feed-forward + PI.
- Mixer: `left = c - d`, `right = c + d` (positive d turns counter-clockwise). Under saturation the turn wins.
- **Pivot** when |alpha| > `pivot_enter_deg` (70): surge held at zero, one motor forward and one reverse (clockwise: left forward + right reverse; counter-clockwise: the opposite) until |alpha| < `pivot_exit_deg` and the spin has slowed.
- **Reverse legs** track with the stern leading (back-off after contacts, making turning room).
- Gains were tuned on the twin (sweep: straight-line RMS 5 cm, slalom 8 cm). Real-boat starting point: halve `kp_heading`, raise `lookahead_min_m` to 4-5 m, then increase.

**Getting close without heading dead-on** (`boatnav/planning/approach.py`). Each mechanism has a body-frame mount point and a required target offset (`mechanisms` in boat.json, all starboard by default):
- deploy: zebra buoy about 1.96 m to starboard of the drop point so the float lands about 0.6 m from the buoy (limit 1.83 m);
- launch: target centre `range_m` (4.2 m) along `azimuth_deg` (-90 = starboard beam) from the launcher, boat within 5 m of the black buoy;
- recover: case 0.35 m outboard of the hull side at the recover arm.
For every candidate heading theta the stop pose is `p = T - R(theta) * b`. The planner keeps poses where the whole hull is clear of every other buoy, the straight final leg (7 m) is clear so the boat arrives parallel and slow, and scores distance, turning, launch-area rules and (if you set a current) facing into it. Then: A* to the leg start, pure pursuit down the leg with a stopping speed profile, **station keeping** (surge by forward/reverse thrust, heading by differential; lateral error cannot be corrected with fixed stern motors, so if it exceeds `hold.lateral_abort_m` the boat backs off and re-approaches), fire the mechanism, verify, depart.

**Gates and slalom** (`planning/course.py`). Rule from the PDF (p5): keep green on the port side, red on starboard. A red/green pair therefore fixes the crossing direction `rot90(red - green)`. The boat lines up 9 m before a gate and crosses straight. Slalom waypoints sit `slalom_offset_m` (1.6 m) beside each buoy on the rule side, joined by a Catmull-Rom spline. After gate B the path bends right because the channel's first green buoy is only 15 ft beyond gate B.

**Evade channel.** A* inside the channel corridor with virtual walls along both banks (rule compliance) and a Gaussian cost around each pink detector (`detector_cost_weight`, `detect_radius` in the course file, assumed 6 m).

**Contact tasks** (Identify, Return). A bow-first approach line with the most clearance from neighbours, creep at 0.35 m/s, stop on estimated contact (hull within 4 cm of the buoy) or a stall, then reverse out along the same line.

**Vision** (`perception/detector.py`, `config/vision_colors.json`). HSV thresholds per colour, morphology, connected components, shape split (tall orange = buoy, flat orange = launch target; tall yellow = buoy, wide yellow = case), size-vs-range consistency check, zebra = white merged with its black tape. Range from depth (ZED2i / Unreal), else from the waterline row on flat water, else from apparent height. The launch target centre comes from its two rim tangents (exact for a circle). On the twin: gate/slalom buoys localised to about 2-5 cm.

**Buoy map** (`perception/mapping.py`). Every course object is a prior with sigma 1 m (`fused`, default). Detections update it with a small Kalman filter; unmatched repeated detections become new obstacles and trigger replanning. Modes: `prior` (surveyed GPS only), `fused`, `vision` (priors are only search hints, for the GPS-agnostic bonus). Choose per mission (`nav_mode`) or on the dashboard launcher.

**Mission** (`mission/tasks.py`, `config/missions/*.json`). Tasks are Python generators (transit, acquire, approach, hold, act, verify, depart). Each has a time budget; failures move on to the next task.

---

## 6. Development workflow

(Architecture, interfaces and extension recipes: `DEVELOPER.md`.)

| I want to... | Do this |
|---|---|
| Change speeds, gains, mechanism offsets | `config\boat.json`, then `10_Fast_Twin_Test.cmd` |
| Move or add course objects | edit `tools\make_course.py` (feet, relative to the start buoy) and run it, or edit `config\course_aimm_2025.json`; then re-run `05_Build_Course_In_Unreal.cmd` (or MCP `move_course_object`) |
| Run only some challenges | copy `config\missions\aimm_full.json`, delete tasks, `Run.ps1 twin -Mission <name>` |
| Quick payload tuning | mission `standoff_tests` |
| See what happened | `12_Plot_Last_Run.cmd`, or `tools\plot_run.py <log> --zoom channel` |
| Change the vision | `config\vision_colors.json`, check with `tools\eval_vision.py` |
| Add a behaviour | write a generator in `mission\tasks.py`, register it in `REGISTRY`, reference it in a mission JSON |
| Verify nothing broke | `11_Tests.cmd` |

Command line (from `BoatLab`, with `.venv\Scripts\python`):
```
python -m boatnav.runner --backend twin --mission aimm_full --realtime        # + dashboard
python -m boatnav.runner --backend twin --mission aimm_full --gps-noise 1.5 --current=-0.2,0.1
python -m boatnav.runner --backend unreal --mission aimm_full --nav vision --color purple
python -m boatnav.runner --backend twin --mission aimm_full --fog 25         # twin camera fog
```

---

## 7. Moving to the real boat (Jetson Orin Nano)

The same `boatnav` code runs on the boat; only `--backend real` changes. **`boatnav/hal/real.py` has not been run on hardware.** Bench test in this order.

1. **Copy** `BoatLab\boatnav`, `BoatLab\config` and `BoatLab\requirements-jetson.txt` to the Jetson. `pip3 install -r requirements-jetson.txt` (use JetPack's OpenCV).
2. **Arduino**: open `arduino\boat_motor_bridge\boat_motor_bridge.ino`. Choose `OUTPUT_MODE_RC_PWM` (RC-style ESC or Sabertooth R/C input) or DIR+PWM (Cytron-style), set pins, neutral pulse and range, and the servo positions for the deploy/launch/recover mechanisms. It speaks `M,<left>,<right>` (-1000..1000), `S`, `A,DEPLOY|LAUNCH|RECOVER` and neutralises after 500 ms without commands.
3. **Config** `config\boat.json` > `real`: `motor_serial` (e.g. `/dev/ttyUSB0`), `invert` per motor, `nav_source` (`mavlink` for the Cube Orange+ with Here4, or `nmea`), `mavlink_url`, `camera_index` or `use_zed`. Set `camera.mount_flu_m`, `pitch_down_deg`, `height_above_water_m` to your measured mount, and `hull.length_m` if you want the real 12 ft.
4. **Dry run on the bench** (no serial writes): `python3 -m boatnav.runner --backend real --mission waypoints_demo --dry-run --api-port 8772` and open the dashboard from a laptop (or run the dashboard on the Jetson). Check position, heading sign (turn the boat by hand: heading must increase counter-clockwise) and the camera.
5. **Props out of the water**: remove `--dry-run`, use mission `operator`, send one short waypoint. Check motor directions (left = port) and that the neutral watchdog works when you kill the runner.
6. **Survey the course**: record `id,lat,lon` for the buoys (and `origin`, e.g. the dock end) into a CSV, then `python tools/survey_to_course.py survey.csv --out config/course_lake.json` and run with `--course course_lake.json`.
7. **Water tests**, lowest speeds first (halve `control.speed` values): `waypoints_demo`, then one gate, then `standoff_tests`. Re-tune colours on real images (`tools/tune_colors.py --backend real --save runs/frames_lake`). Keep an RC override/kill switch for safety even though remote intervention costs points.
8. **Calibrate the model** from logs (`simulation_ml\boat\calibrate_surge.py`), put the results in both `config\boat.json` > `dynamics` and the Unreal `BoatDynamics` properties, so the twin, Unreal and the boat stay consistent.

---

## 8. File map

```
BoatLab/
  START.cmd, Run.ps1, 01..15_*.cmd   launchers
  GUIDE.md, DEVELOPER.md             how to run it / how the code works and how to extend it
  settings.json                      engine path, ports, Python path
  config/boat.json                   hull, dynamics (= Unreal plugin), motors, camera, mechanisms, gains, real-boat IO
  config/course_aimm_2025.json       course (generated by tools/make_course.py from the PDF map)
  config/vision_colors.json          HSV thresholds
  config/missions/*.json             aimm_full, standoff_tests, waypoints_demo, operator
  boatnav/                           portable autonomy package (hal, perception, planning, control, mission, runner)
  dashboard/                         server.py (stdlib), index.html, dashboard.js
  unreal/BoatPhysics/                plugin v2 source (camera, reset_pose, markers)
  unreal/build_install_plugin.ps1    build + install with backup
  unreal/Content/Python/             boatlab_course.py (builder), boatlab_toolset.py (MCP tools)
  arduino/boat_motor_bridge/         serial motor/actuator bridge template
  tools/                             make_course, plot_run, eval_vision, tune_colors, fake_unreal, survey_to_course, check_install
  tests/                             pytest suite
  runs/                              logs, summaries, plots
```

---

## 9. Troubleshooting

| Symptom | Fix |
|---|---|
| `No reply from Unreal on UDP 127.0.0.1:7450` | Open `Lake_300x80ft` (the level keeps its old name; the lake is 350 x 200 ft), press Play; check `CommandPort` on BoatDynamics; only one runner at a time |
| Dashboard shows "Unreal (read-only)" and no camera | Plugin v1 still installed: `04_Build_Plugin.cmd` with the editor closed |
| Boat does not move in Unreal | Keyboard Control fighting the runner (untick it), or the mission is paused (Start / resume) |
| Wrong colour detections | `13_Tune_Colors_Unreal.cmd`, check with `eval_vision.py --folder` |
| Boat oscillates on straights (real boat) | lower `kp_heading`, raise `kd_heading` and `lookahead_min_m` |
| Stand-off hold keeps re-approaching | current/wind pushes sideways: set `--current` so it faces into it, raise `hold.lateral_abort_m`, lower approach speed |
| "no collision-free route" | an object/keepout blocks the way; check `course_aimm_2025.json` against the level (`describe_course`) |
| Build fails with long-path errors | move the project to a short local path (e.g. `D:\UE\mcp_gpt_boat`) |

---

## 10. Assumptions to confirm

- **Course reading of the PDF map**: labelled distances are used as given (top line 40/15/30x5/~45 ft, bottom line 40/15/50/50/50/~30 ft, channel 30+15+15 ft); 10 ft gates; the light-blue dots at the gates and under the identify cluster are course-line markers, not buoys; the slalom passes south of red and north of green heading west (p5 rule, p7 diagram); the channel is 40 ft wide and 60 ft long running south with reds west and greens east, detectors on the red bank 45 ft and on the green bank ~14 ft down (the p8 Google Earth pins show the same arrangement). Where the map gives no number, positions are measured from the drawing at ~8 px/ft: identify fenders 3.2 ft apart and 2.2 ft north of the line, centred 15 ft east of the channel exit; the marina footprint and moored boats; the return buoy about 8 ft south of the nearest moored boat (p13 pin). Every value is a parameter in `tools/make_course.py`.
- **Lake 350 x 200 ft**: the PDF has no pool size; the satellite map shows roughly 55-80 ft of water north of the course and at least 60 ft west / 20 ft south (photo cropped).
- **LiDAR detector reach** assumed 6 m (unknown).
- **Mechanisms** on the starboard side with the offsets in `boat.json` (your choice); change them when the hardware exists.
- **Physics** is the uncalibrated Unreal model; the twin matches Unreal within 1-2 cm/s on your logged step responses, not the real boat.
- **Unreal plugin v2 compiles (2026-09-27) but has not run in the editor yet**; the MCP toolset and the course builder have run in the real editor (see section 11).

---

## 11. What was verified before delivery

Simulation judge results for the full `aimm_full` mission (8 scorable challenges). All runs used the current code; plots and summaries of the first two are in `runs\examples`.

| Scenario | Challenges passed | Unintended buoy contacts | Notes |
|---|---|---|---|
| Python twin, RTK-like GPS (2 cm), vision fused | 8 / 8 | none | about 5 min of boat time; deploy 0.57 m from the zebra buoy, football 0.55 m from target centre |
| Unreal UDP protocol (fake plugin: JPEG + depth over UDP, real time) | 8 / 8 | none | the path your Unreal backend will use |
| Twin, plain GPS (1.5 m, slowly drifting) | 8 / 8 | brushed gate A red and gate B red | 10 ft gates are tight without RTK; plan on the Here4 RTK |
| Twin, vision-only map (GPS-agnostic mode) + 0.18 m/s cross current | 8 / 8 | brushed gate B red, channel exit green | fixed stern thrusters cannot hold position sideways; re-approaches cost time |
| `pytest` suite | 14 / 14 | | |

Verified in the real editor on 2026-09-27 (UE 5.8.2, driven through the Unreal MCP server):

| Check | Result |
|---|---|
| `pytest` on numpy 2.5 / OpenCV 5.0 | 14 / 14 |
| Twin `aimm_full` (re-run) | 8 / 8, deploy 0.57 m, football 0.55 m, only intended contacts |
| MCP toolset `boatlab_toolset.BoatLabCourseToolset` | registers with 6 tools; `build_course` placed 26 objects (55 actors) at 0.00 cm from the course JSON |
| Lake widened to 120 ft, persisted | first build: **not saved** (Python transform edits did not mark the World Partition actors dirty; after an editor restart the lake was 80 ft again and the boat stalled on the old shore collision). Fixed with `Modify()` + saving the actors' packages; verified by reloading the level from disk and tracing the shore collision at +/-18.29 m |
| BoatPhysics v2 build (`-SkipInstall`) | compiles, 0 warnings (after renaming a local `G` that shadowed gravity) |
| **Unreal** `aimm_full`, plugin v1, `--nav prior --no-camera` | **8 / 8**, deploy 0.74 m, football 0.16 m, only intended contacts, no shore contact, detector exposure 0 s, min clearance to non-targets 0.25 m |

Level backup taken before the course build: `BoatLab\unreal\backups\level_Lake_300x80ft_20260927_132936` (umap + external actors/objects).

Layout closer to the PDF (lake 350 x 200 ft, marina with moored boats, identify cluster as drawn, trampoline and case to spec), 2026-09-27:

| Check | Result |
|---|---|
| `pytest` | 14 / 14 |
| Twin `aimm_full` | 8 / 8, deploy 0.50 m, football 0.27 m, only intended contacts, no pier/boat contacts, docks at the slot |
| Unreal build via MCP, then level reloaded from disk | 350 x 200 ft, 115 BoatLab actors (56 course + 59 marina), 0 cm placement error, stub pier removed, shore and marina collision where expected |
| **Unreal** `aimm_full`, plugin v2 camera on, `--nav prior` | **8 / 8**, deploy 0.55 m, football 0.06 m, only intended contacts, no pier/boat/shore contacts, closest hull approach to the shore 12.6 m (was 0.74 m in the 120 ft lake), docks at the slot |

Level backup before this rebuild: `BoatLab\unreal\backups\level_Lake_300x80ft_20260927_144557`; previous course file: `config\course_aimm_2025.backup_*.json`.

Plugin v2 installed 2026-09-27 (v1 backup: `BoatLab\unreal\backups\BoatPhysics_20260927_135325`): `plugin_version boatlab_v2`, camera 640x480 RGB + depth streaming, `reset_pose` lands within about 1 cm / 0.1 deg.

Not verified yet: vision-based navigation in Unreal (`--nav fused/vision`, see 3.4), and anything on the real boat (Jetson, Arduino, GPS, cameras).
