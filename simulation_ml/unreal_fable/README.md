# unreal_fable — autonomy dev kit: Unreal worlds, Python brains, real-world numbers

> **Codex / GPT-5.6 Sol edition:** Start with [the outer README](../README.md) and
> [Codex setup](docs/CODEX_SETUP.md). The standalone MCP server has been adapted
> and tested for Codex. The original overview below describes the source kit,
> including its original Claude integration and uncompiled Unreal plugin.

Two test scenarios — an **Ackermann car on a track** and a **twin-thruster boat on open
water** — built in Unreal Engine 5.8 from JSON scene specs, driven by your Python control
code over UDP, scored by a task evaluator, with a pure-Python physics stand-in for fast
iteration, calibration tools to fit the sim to your real car and boat, and Claude able to
build and edit the worlds by conversation.

```
                     ┌────────────── DESIGN TIME ──────────────┐
   you / sketch ───▶ Claude ──MCP──▶ Unreal Editor ──▶ scene_spec.json  ◀── you edit by hand too
                                                            │
             ┌──────────────────────── the spec is the contract ────────────────────────┐
             ▼                                              ▼                             ▼
   ┌─── Unreal (PIE / headless) ───┐            ┌─── mock_sim.py ───┐           ┌── evaluator ──┐
   │ FableBridge plugin            │            │ bicycle car       │           │ laps, gates,  │
   │ Chaos car / custom boat       │◀──UDP──▶   │ Fossen boat       │           │ docking, RMS, │
   │ lidar, GPS, wind, current     │  9800      │ same Observation  │           │ contacts …    │
   └───────────────────────────────┘            └───────────────────┘           └───────────────┘
                     ▲                                    ▲
                     └──────── fable.Sim(backend="unreal" | "mock") ────────┘
                                             ▲
                            your controller(obs) -> {"throttle","steer"} / {"thrust_l","thrust_r"}
                                             │
                                  ros2_adapter.py: same function on the real vehicle
```

## 30-second tour

```bash
cd python
pip install -e ".[dev]"                       # numpy, scipy, pytest, jsonschema
python -m pytest tests -q                     # 19 tests: physics, controllers, protocol, calibration
python examples/run_scenario.py car_track_oval            # mock backend, ~2 s
python examples/run_scenario.py boat_docking --seeds 5    # 5 randomized episodes
```

You just ran the four bundled worlds with the baseline controllers and got scorecards —
no Unreal needed. Unreal is the same call with `--backend unreal` once the project is set
up (`docs/UNREAL_SETUP.md`, one evening).

## What is in the box

| Path | What |
|---|---|
| `specs/schema/scene_spec.schema.json` | **The contract.** World + vehicle + task + randomization. Metres, radians, ENU. |
| `specs/schema/vehicle_params.schema.json` | Every physical number of a vehicle, and which tier of measurement it comes from. |
| `specs/worlds/` | `car_track_oval`, `car_track_slalom_gravel`, `boat_buoy_course`, `boat_docking` — regenerate with `specs/make_worlds.py` |
| `specs/vehicles/` | `car_rc_default`, `boat_twin_thruster_default` — placeholders, replace via calibration |
| `python/fable/` | `Sim` facade · `bridge` (UDP client) · `mock_sim` (physics) · `sim.TaskEvaluator` · `controllers/` (pure pursuit, integral LOS, docking, reactive avoid) · `calibration/` (fitters + sysid manoeuvres) · `ros2_adapter` · `fake_unreal` (protocol reference server) |
| `python/examples/` | `run_scenario.py`, `sysid_run.py` |
| `unreal/Plugins/FableBridge/` | C++: UDP bridge actor, `FableCarPawn` (Chaos), `FableBoatPawn` (hydrodynamics), `FableLidar2D` |
| `unreal/Content/Python/` | `fable_build.py` (spec → level), `fable_toolset.py` (Claude's tools, UE 5.8 MCP) |
| `mcp/codex_mcp.py` | Codex MCP server: editor tools via Remote Control + **live** sim editing |
| `tools/sketch_to_heightmap.py` | drawing/photo → terrain mesh; drawn line → route |
| `docs/` | `PROTOCOL.md` · `UNREAL_SETUP.md` · `CALIBRATION.md` · `CLAUDE_WORLD_EDITING.md` |

## The five decisions this kit makes for you

**1. Python talks to Unreal over UDP/JSON, not ROS.** rclUE is Linux-only and you are on
Windows. The wire format is already REP-103 (SI, ENU, body x-forward y-left), so the
controller code is byte-for-byte the same in sim and on the vehicle; `ros2_adapter.py` is
the 150-line shim that turns ROS messages into the same `Observation`. The protocol is
small enough to read in five minutes (`docs/PROTOCOL.md`) and has a reference
implementation in Python (`fake_unreal.py`) that the client is tested against.

**2. A pure-Python physics model ships alongside Unreal, and both read the same
parameter file.** Iterate on the algorithm at 2–4 k steps/s with no engine running; move
to Unreal for the collisions, lidar-on-real-geometry, and the pretty pictures. When the two
disagree, that difference is information about which one is wrong.

**3. The scene spec is the single source of truth.** Claude, the editor, the mock, the
evaluator and your tests all read the same JSON. One file = one reproducible test case. A
world Claude edits by conversation is scored identically by your Python five minutes later.

**4. Calibration is a procedure, not a hope.** Three tiers (tape measure → bench → one
open-loop drive), fitters that are tested by recovering deliberately-perturbed parameters
from simulated logs, and a boat controller whose gains are *derived from* the fitted
parameters (`heading_gains_from_params`) so a correct fit means no retuning on the water.

**5. Fixed 50 Hz timestep, zero-order hold.** Unreal ticks at exactly `dt`; if your
Python is late the vehicle keeps doing what it was told, exactly like the real one. Run
`-nullrhi` for faster than real time. Determinism is what makes A/B comparisons mean anything.

## Writing your own controller

```python
from fable import Sim, load_scene
from fable.types import Observation

class MyController:
    def reset(self): ...
    def __call__(self, obs: Observation) -> dict:
        # obs.pose (x, y, yaw), obs.vel (vx, vy, wz body), obs.lidar.ranges, obs.gps, obs.collision ...
        return {"throttle": 0.3, "steer": -0.1, "brake": 0.0}        # car
        # return {"thrust_l": 0.5, "thrust_r": 0.6}                  # boat

scene = load_scene("car_track_slalom_gravel")
with Sim(scene, backend="mock", log_path="run.csv") as sim:          # or backend="unreal"
    print(sim.run(MyController(), seed=3))
```

`sim.spawn("wall", "surprise", x, y, yaw)` mid-episode drops an obstacle in front of it;
`sim.set_env(wind={"speed": 5})` changes the weather; `sim.set_params(vp.with_overrides(**{"car.max_speed_mps": 3}))`
swaps the vehicle. All of it works identically on both backends.

## Order of work

| | Milestone | Done when |
|---|---|---|
| 1 | Python kit runs | `pytest` green, four scorecards print |
| 2 | Your controller on the mock | it beats the baseline on `car_track_oval` / `boat_buoy_course` |
| 3 | Unreal project builds | `docs/UNREAL_SETUP.md` §1–3; the plugin compiles (expect a few one-line fixes) |
| 4 | Unreal loop closes | `run_scenario.py car_track_oval --backend unreal` prints a scorecard |
| 5 | Claude edits worlds | "add a chicane on the back straight" appears in the editor and in the JSON |
| 6 | Tier 1–2 numbers measured | `*_measured.json` has real mass, geometry, delays, thrust |
| 7 | Tier 3 fit | `fit_car` / `fit_boat` RMS within the targets in `docs/CALIBRATION.md` |
| 8 | Sim ≈ real | same gains, same course, cross-track RMS within 20 % |
| 9 | Randomized robustness | your controller passes 20 seeds of each world with ±20 % parameter bands |

Steps 1–2 today. Step 3 is the long one (Visual Studio, first UE C++ build). Step 7 is
one afternoon outdoors and is where the real payoff is.

## Honest limits

- The C++ plugin was written against the UE 5.8 API from documentation and is **not
  compiled here**; the Python side is fully tested. Expect a short compile-fix session.
- Cameras are not in the bridge (UDP is the wrong pipe for images). The lidar is.
- The car in Unreal uses Chaos Vehicles with a governor that reproduces the mock's speed
  model; tyre behaviour beyond ~0.6 g lateral differs from the mock. The boat's added mass
  is in the mock but not in Unreal. `docs/CALIBRATION.md` Tier 4 lists all of these.
- One vehicle per level. Multi-agent is a namespace away in the protocol but not built.
