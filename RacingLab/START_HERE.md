# RacingLab handover — miniature F1 / Unreal Engine 5.8 / Windows

**Deep Q-learning is now the default for Train/Evaluate. Read [DQN_GUIDE.md](DQN_GUIDE.md) first.** The older tabular demonstration remains in rl_demo.py with policy.json; its duration/reward description below is historical and does not describe the new DQN. DQN uses dqn_settings.json and dqn_models/*.pt, automatic resets, direct steering/speed actions, and ground-truth/LiDAR/IMU observations. RGB remains monitoring only. Run 14_Setup_DQN.cmd on a new computer; unlike the old stdlib demo, DQN requires the local PyTorch/NumPy virtual environment.

Open this folder for daily operation. Double-click **START.cmd**, then type an action from the menu. You can also run `./Run.ps1 dashboard` in PowerShell. Paths are relative to the project; do not detach this folder from the project tree.

Numbered shortcuts `01_Open_Unreal.cmd` through `09_Manual.cmd` run the common actions directly. Daily launchers are collected here; the maintained native plugin, assets and shared dashboard implementation remain in their normal project folders and are included by the exporter.

## First run on another computer

1. Install the same Unreal Engine **5.8 build** as the source computer. The car plugin contains native Windows binaries. A different engine build can require recompilation (Visual Studio 2022 C++ game development tools and Windows SDK).
2. On the source computer, run `Export.ps1 -Destination D:\F1_Handover` from this folder, choosing a NEW destination. Copy that complete resulting directory to the recipient. The exporter includes Content, Config, the car plugin including source/binaries/licenses, track data and tooling; it omits caches and old build packages. It creates a project descriptor without the source computer's optional editor automation/terminal plugins. The car does not depend on those plugins.
3. In the transferred `RacingLab/settings.json`, set `engine_root` if Unreal is installed elsewhere. Alternatively set environment variable `UE_58_ROOT`. UE's bundled Python runs the dashboard/baseline. For DQN install Python 3.12 with the Windows py launcher and run **14_Setup_DQN.cmd** to install its separate dependencies.
4. Double-click `START.cmd`, enter **check**. No missing files should be reported. Run **editor**, allow shader compilation, open `/Game/ThirdPerson/Lvl_ThirdPerson` if necessary, and click **Play**. Do not use the human template map.
5. Run **dashboard**. Open `http://127.0.0.1:8765` if the browser does not appear. Check LIVE, RGB image, four wheel contacts and sensible speed/IMU before running a controller.
6. Run **baseline** for a controller-only reference. Then run **train** or **evaluate**; DQN automatically resets the car to the saved start before each episode. Use Stop/Play before running the older baseline again.

The transfer has been checked locally; installation on a second computer has not been performed. Unreal itself is not included. Existing native plugin binaries are engine-build-specific; use `simulation_ml/integrations/rapyuta/build_port.ps1 -SkipInstall` for a rebuild after reviewing its engine-path settings, then install the resulting plugin with the editor closed. Do not replace a loaded DLL.

## Daily commands

| START.cmd action | What it does | What to watch |
|---|---|---|
| editor | Opens this project's UE editor | Correct track and F1 pawn; click Play |
| dashboard | Opens read-only telemetry browser | Speed, IMU, wheel contacts, front/rear LiDAR, RGB, track position |
| settings | Opens settings.json in Notepad | Save valid JSON, then relaunch controller |
| configure | Applies speed ceiling and tire grip to the running Unreal car, with brake command | Use 10_Apply_Environment.cmd during Play; stop other controllers first |
| baseline | Pure pursuit for settings.json evaluate_seconds | Track margin and cross-track error |
| train | Double DQN with automatic episodes | `dqn_settings.json`, `dqn_models/`, `dqn_runs/`; see DQN_GUIDE.md |
| evaluate | Frozen DQN policy with automatic reset | Evaluation trajectory, reward, progress, failure reason; RGB monitoring only |
| diagnose | Straight low-speed drive, bounded by the same safety stops | Raw IMU, wheel contact, suspension in run log |
| stop | Sends zero-speed/brake demand | Car stops; also terminate any running controller |
| check | Offline file/dependency check | Missing files |
| manual | Opens this file | Transfer and algorithm instructions |

Only run **one driving script at a time**. The read-only dashboard may stay open during training. It is the sole RGB reader; multiple image readers can replace the car's cached frame. Keyboard I/K = forward/reverse, J/L = left/right, Space = brake. Do not use keyboard steering during an autonomous test. Ctrl+C stops the script and sends a braking command. Unreal **Stop** immediately ends the simulation and is the definitive stop if another controller is still sending commands. A `stop` command alone cannot override a continuously running controller.

The native car also brakes after 0.25 seconds without drive commands. DQN resets automatically after normal episode completion/failure. A failed reset or sensor/transport error aborts training and saves available evidence. The older baseline/tabular scripts still require Stop/Play after a safety termination. See DQN_GUIDE.md for episode limits and reset checks.

## Legacy tabular demo (rl_demo.py, not the default Train launcher)

`rl_demo.py` uses real UDP transitions from the Chaos car, not synthetic trajectories. Ground-truth position and the known centreline are privileged simulator inputs, as agreed for this first demonstration. A pure-pursuit controller supplies the main steering angle. Q-learning selects a residual of -0.04, 0 or +0.04 radians. States discretize signed cross-track and heading errors (25 possible states). Unknown/equal-value states choose zero correction. Exploration is enabled only during training.

Reward integrates forward progress rate minus cross-track error, heading error and residual magnitude. Safety termination adds a penalty. Updates follow `Q(s,a) += alpha * (reward + gamma * max Q(next) - Q(s,a))`; terminal transitions have no bootstrap value. Training is one continuing task, not a batch of automatically reset episodes. The saved policy contains the Q table, visit counts, action list and settings. Training resumes that table; archive/remove **only policy.json** to start fresh.

The demo brakes at less than 8 cm estimated body-to-track-edge margin, tilt beyond the configured upright check, lost wheel contact, invalid/stale input or a LiDAR return in a short forward corridor. LiDAR is ideal ray data and may hit terrain; the stop rule is conservative, not a classifier or an obstacle-avoidance planner. The policy does **not** learn obstacle avoidance, optimal speed or a fastest racing line yet. A baseline may outperform a briefly trained policy; report that honestly. No lap-time or 5 m/s capability is implied.

## Legacy/shared files (new DQN files are listed in DQN_GUIDE.md)

| File | Purpose / next development step |
|---|---|
| `settings.json` | Engine/ports; speed, speed ceiling, tire grip, lookahead, control rate; alpha/gamma/epsilon; training duration and action magnitudes. Command ceiling is 138.5824 m/s (310 mph); physical performance is not validated at that speed. |
| `rl_demo.py`: `Track.observe` / `state` | Observation geometry and state discretization; add curvature, speed, upcoming obstacles after validation. |
| `rl_demo.py`: `reward` / `terminal` | Reward and failure conditions. Keep edge/obstacle/contact checks when changing rewards. |
| `rl_demo.py`: main loop | Action selection and Q update. Replace with a learning library only after building a reliable reset/step environment. |
| `policy.json` | Learned Q values and visit counts. Preserve with the training settings and logs. |
| `runs/*_train.json`, `*_evaluate.json`, `*_baseline.json` | Full timestamped telemetry, observations, actions and summary. Compare equal starting pose, duration, speed, course and physics. |
| `runs/latest_summary.json` | Latest result, progress, tracking error, minimum margin, termination reason and update count. |
| `analyze_bumps.py` | Analysis of raw telemetry arrays from `f1_client.py`; centreline-based clearance estimate, not measured collision clearance. |
| `../simulation_ml/integrations/rapyuta/dashboard/` | Maintained browser/server implementation, exposed by the common launcher. |
| `../Plugins/RapyutaSimulationPlugins/Source/RapyutaSimulationPlugins/Private/MiniF1Car.cpp` | Native steering, force suspension, motor control, telemetry. Requires compile/restart. |
| `../Config/DefaultEngine.ini` | Saved physics substep settings; restart editor after editing. |
| `../track/build_smooth_track.py` / `../track/SMOOTH_TRACK.md` | Course generation, curb/runoff geometry, collision and validation. Generator needs NumPy/SciPy; daily runtime does not. |

Keep the native telemetry coordinate conversion: bridge x=(UE_X-10000)/100, y=-UE_Y/100. The track origin is UE (15000,0,0), so its current x is bridge x minus 50 metres. Steering is positive left, in radians. IMU acceleration is **specific force**, approximately +9.81 m/s² vertically at rest, not zero. Track position/IMU orientation are ground truth, not GPS or estimated odometry. LiDAR null means no return, not zero range.

## Speed and grip experiments

`speed_mps` remains metres per second: **2 mph = 0.89408 m/s**. A value of 2 means 2 m/s (about 4.47 mph). The existing user value 1.2 m/s is retained. `max_speed_mps` defaults to 138.5824 m/s, exactly 310 mph. Both Python validation and the native UDP bridge support that command ceiling. This does not change engine power, gearing, brakes, track scale, or actual attainable top speed. The miniature vehicle and short-lookahead controller are not validated for full-size racing speeds; the 3.5 m LiDAR range and 20 Hz control are also inadequate for such a claim. Safety stops remain enabled.

`tire_grip_multiplier` applies equally to all four wheels: 1.0 preserves the original tire model, 0.5 halves its friction multiplier, 0.2 gives much lower grip, and 1.5 increases it. Accepted range is [0,3]. These are relative experimental values, not calibrated dry/wet/ice coefficients. They multiply the original Chaos wheel friction value 1.4 and retain the road physical material's influence. No road collision mesh is changed.

Save settings, Stop/Play for the same start, then launch baseline/train/evaluate. Each automatically sends the configuration and requires acknowledgement from the new native plugin before driving. For keyboard experiments, use `10_Apply_Environment.cmd` after Play. Configuration resets on a new Play session; apply again. Editing JSON alone does not alter an already-running controller. The startup console prints units, ceiling and grip. Raw telemetry includes max_speed_mps, speed_target_mps and both axle grip values (identical with this settings interface).

For fair grip comparisons keep start, speed, policy and duration fixed. DQN uses dqn_models/*.pt and rejects incompatible environment settings; start a separate checkpoint or use --fresh for a new grip experiment. The legacy table uses policy.json. Settings are recorded in each run. Native updates require an Unreal restart; later JSON grip/speed changes do not require compilation or restart.

## Development toward autonomous racing

1. Establish repeated baseline runs and stable physics across the entire course. Record completion, intervention rate, minimum edge clearance, collisions, wheel contact, lap time and acceleration. Use fixed scenarios/seeds for comparisons.
2. The native reset API is implemented and tested. A fully synchronous simulator step contract remains future work; current control is real time, so hardware/render load changes sample timing. Split training and held-out evaluation starts/scenarios. Add repeatable obstacle placement.
3. Expand observations with speed, curvature preview and LiDAR features. Add continuous steering/speed actions with PPO/SAC only after the environment and rewards are tested. Reward progress while penalizing off-track/collisions; do not reward speed in isolation.
4. Train a speed profile and lateral line inside measured track boundaries. Increase speed gradually after physical validation; a 360 m track at constant 5 m/s is 72 seconds arithmetically, not an achievable lap guarantee.
5. For camera/LiDAR-only autonomy, implement perception and localization separately, then replace ground truth in policy observations. Keep simulator truth only for evaluation. Add sensor noise, delay, grip/mass variation and unseen layouts to test robustness.

## Bumps and ride height

The 2026-09-20 straight-section tests found all four wheels in contact and approximately 33–37 mm chassis-hull clearance. Enabling physics substeps (maximum 1/120 second, up to 8 per frame) substantially reduced vertical acceleration variation. These settings are saved in DefaultEngine.ini. Chassis height was not increased: cosmetics such as wings/floor have no collision, and the colliding hull bottom is approximately 2 cm below the chassis origin.

See `VALIDATION.md` for measured evidence and limits. This is a short, low-speed diagnosis, not proof that every curb, crest and speed is smooth. The red/white curbs are intentionally raised up to 6 mm. At higher speeds inspect suspension travel, pitch/roll, contact loss and collision geometry before modifying ride height. Raising the body alone will not repair unstable numerical integration.
