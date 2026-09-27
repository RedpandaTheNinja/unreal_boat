# Deep Q-learning with the Unreal miniature F1

This is an original **PyTorch Double DQN implementation** integrated with the existing Unreal car, track and sensor bridge. It replaces the default Train/Evaluate launchers. The older `rl_demo.py` tabular demonstration and its policy.json remain separate. Learning takes place in Python; all car transitions come from Unreal Chaos.

## Start and monitor

1. On this computer dependencies are installed in `RacingLab/.venv`. On a new computer install **Python 3.12 with the Windows py launcher**, then run **14_Setup_DQN.cmd** once (downloads CPU PyTorch and NumPy). The project transfer exporter excludes virtual environments; recreate one on the recipient. The UE bundled Python is 3.11 and remains suitable for the dashboard, but DQN is tested in its separate Python 3.12 environment. For a custom interpreter path use `DQN.ps1 -Action setup -Python312 C:/path/to/python.exe`.
2. Open Unreal, load the track and click **Play**. Open **02_Dashboard.cmd**. Only one controller should run at a time; do not steer with the keyboard during training.
3. Use **03_Settings.cmd** for the vehicle's speed ceiling, target speed and tire grip. Your current values are preserved. DQN's discrete speeds are fractions of `speed_mps`; `max_speed_mps` is only the native ceiling.
4. Use **13_DQN_Settings.cmd** for episode count, duration, neural-network learning and action choices.
5. Run **05_Train_RL.cmd** or **11_Train_DQN.cmd**. They are aliases. The car now automatically returns to its saved start pose before each episode. No Stop/Play between episodes is needed.
6. Run **06_Evaluate_RL.cmd** or **12_Evaluate_DQN.cmd** for a frozen-policy evaluation. Exploration and weight updates are disabled. This also resets the car first.

Training defaults to **20 additional episodes**, each limited to **120 simulated seconds or 1,200 decisions**, whichever comes first. Decisions target 0.1 simulated seconds; actual elapsed time is measured, because Unreal runs in real time rather than a synchronous fixed-step training server. Failures end an episode early and trigger the next reset. A completed lap also ends an episode. An invalid sensor stream, paused Unreal or a failed reset aborts the session instead of learning from broken data.

The console and `dqn_runs/live_status.json` show episode/step, **cumulative training decisions**, **cumulative gradient updates**, epsilon, reward, progress and loss. The first 128 transitions populate replay memory before gradient updates begin. Defaults use batches of 64; do not mistake zero updates during warmup for failure. The dashboard camera is monitoring only.

Ctrl+C brakes and saves the learner. Unreal **Stop** is the definitive simulation stop. `07_Stop_Command.cmd` sends a brake command but cannot override a controller that continues sending commands; terminate it too.

## What the neural network receives and controls

The network has 74 numeric inputs: signed position relative to the known centreline, heading sine/cosine, speed, IMU yaw rate and acceleration, left/right footprint clearance, four forward track-preview points, four wheel-contact flags, and 12 range sectors plus return-validity flags from **each** LiDAR. Inputs are normalized/clipped. Null LiDAR returns retain a separate no-return flag. Ground-truth pose and track preview are privileged simulation inputs, not camera-derived localization.

There are **22 actions** by default: stop plus 7 absolute steering angles crossed with 3 speed fractions. The network directly chooses a speed/steering pair. It does not add corrections to pure pursuit. The existing native speed controller tracks the requested speed; the DQN does not directly control engine torque.

The neural network is an MLP with two 128-unit ReLU layers. Replay memory breaks up sequential training samples. Double DQN uses the online network to choose the next action and the target network to value it. Training uses Adam, Huber loss, gradient clipping, and a target-network copy every 250 gradient updates. Discounting accounts for actual transition duration. Terminal transitions do not bootstrap; episode time/step limits are truncations and do bootstrap.

Reward = forward track progress minus a time cost, a near-edge penalty and a small steering cost; failure subtracts 5 and lap completion adds 20. There is **no centreline-distance penalty**. Outside–apex–outside positions can therefore earn reward if the body remains inside the boundaries. The known centreline is still used to measure progress, heading and track geometry.

Failure conditions are estimated body footprint outside the track, excessive tilt, sustained wheel contact loss, a close LiDAR return in the forward body corridor, or no progress for 8 simulated seconds. Obstacle detection is a conservative range proxy, not collision classification. The model has not been validated as a full-speed racing vehicle or a sensor-only autonomous system. Random exploration can leave the track; automatic reset is expected, not an error.

## Episodes and continued learning

Edit `dqn_settings.json`, for example:

```json
"episodes": 100,
"max_episode_seconds": 300,
"max_episode_steps": 3000,
"evaluation_every_episodes": 5
```

Each launch trains that many **additional** episodes. `dqn_models/latest.pt` restores network weights, target weights, optimizer, replay memory, random state and counters. A checkpoint is saved after each episode and on interruption. Every 5 episodes a numbered checkpoint and a frozen evaluation are produced; there is also a final evaluation. Early evaluation can fail or choose to stop: a short test is not proof of a capable policy.

To start a new experiment from scratch in PowerShell, from project root:

```powershell
& .\RacingLab\.venv\Scripts\python.exe .\RacingLab\dqn_train.py train --fresh
```

This archives an existing latest checkpoint before replacing it. For a short integration check:

```powershell
& .\RacingLab\.venv\Scripts\python.exe .\RacingLab\dqn_train.py train --episodes 6 --seconds 8
```

Changing target speed, grip, action definitions, observation schema, track or network size makes an old checkpoint incompatible. The runner refuses an incompatible checkpoint; use `--fresh` or `--checkpoint path/to/separate.pt`. This prevents accidentally reporting results from a different environment. Episode duration and exploration tuning can change without resetting weights. Log those changes when comparing runs.

## Visible best trajectory

Successful frozen evaluations are compared under the same track/settings and evaluation duration/step budget. Completed laps rank ahead of partial runs, and shorter lap time wins among completed laps. Until a full lap exists, greater progress wins among evaluations that finish their time/step budget **without failure**. Failed runs do not replace the best.

An accepted best evaluation is uploaded to Unreal as a **green, non-colliding line** through the measured positions just above the car path. It is an editor debug overlay, not a new road asset or an input to the learner. A partial evaluation produces only a partial line and is explicitly labelled partial in the console and `best_evaluation.json`; it is not an optimized full racing line. The line persists across episode resets within Play, but ends when Play ends. Evaluation can upload it again.

`dqn_models/best.pt` stores the corresponding policy, and `dqn_runs/best_evaluation.json` stores its protocol, summary and trajectory. Numbered evaluation logs let you compare progress over training. Improvement is not guaranteed at every checkpoint, and no fastest-line claim should be made without repeatable full-lap results.

## Where to improve the algorithm

| File | Responsibility |
|---|---|
| `dqn_agent.py` | MLP, replay, exploration, Double DQN target, loss, optimizer and checkpoint state |
| `unreal_dqn_env.py`: `observe()` | Ground-truth/IMU/LiDAR feature vector; change schema/signature when changing inputs |
| `unreal_dqn_env.py`: `step()` | Action timing, reward, progress and termination; no RGB image consumption |
| `unreal_dqn_env.py`: `reset()` | Idempotent native reset, settling and fresh-sensor checks |
| `dqn_train.py` | Episode loop, evaluation, checkpointing, best-line selection and logs |
| `dqn_settings.json` | Episode and learning hyperparameters, steering grid and speed fractions |
| `settings.json` | Vehicle speed/grip environment; old tabular train_seconds is not the DQN episode duration |
| `dqn_runs/` | Full transition logs, aborted runs, live counters and evaluation evidence |

Start with repeatable low-speed training, then increase track coverage and speed. Inspect failure reasons and state coverage before enlarging the network. For learned obstacle avoidance, add repeatable obstacle scenarios; an empty track cannot teach avoidance. For grip robustness, deliberately train across grip environments and redesign checkpoint compatibility/configuration logging accordingly. Camera learning would require a separate image encoder and synchronized image observations, and is outside this first agreed sensor-vector version.

## Reference repositories

The requested [jperod project](https://github.com/jperod/AI-self-driving-race-car-Deep-Reinforcement-Learning) demonstrates image-based DQN for Gym CarRacing, including replay, target networks, exploration and discrete driving actions. Its [DQN source](https://github.com/jperod/AI-self-driving-race-car-Deep-Reinforcement-Learning/blob/master/dqn.py) uses older TensorFlow APIs. Those algorithmic ideas informed this independently written PyTorch/Unreal implementation; repository source and pretrained weights were not copied. The requested affandhankwala/RL-based-Racecar URL could not be retrieved during inspection, so no claims are made about its implementation. Neither Gym physics nor repository scores transfer to this Unreal car.

See `DQN_VALIDATION.md` for actual local tests and remaining limitations.
