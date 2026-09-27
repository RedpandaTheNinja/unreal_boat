# Unreal setup (UE 5.8, Windows)

Time: about an evening. The C++ plugin needs Visual Studio 2022 with the
"Game development with C++" workload — install that first if you have not.

## 0. Where things live

- **Dev kit** (this folder) — fine on G: / Google Drive. Python, specs, docs.
- **Unreal project** — on a **local SSD**, e.g. `D:\UE\FableSim`. Never on Drive:
  the sync client fights the editor over `.uasset` files and `Intermediate/` is
  gigabytes of churn.

## 1. Create the project

1. Epic Launcher → UE 5.8 → **Games → Blank**, **C++**, no starter content, name it `FableSim`, location `D:\UE\FableSim`.
2. Close the editor once it opens.
3. Copy `unreal/Plugins/FableBridge` → `D:\UE\FableSim\Plugins\FableBridge`.
4. Copy `unreal/Content/Python/*.py` → `D:\UE\FableSim\Content\Python\`.
5. Right-click `FableSim.uproject` → **Generate Visual Studio project files**.
6. Open `FableSim.sln`, set configuration **Development Editor / Win64**, **Build**.
   Expect a few compile errors on first build — the plugin was written against the 5.8 API
   from documentation, not compiled here. They will be one-line fixes (a renamed property,
   a missing include). The intent of every line is commented; `docs/PROTOCOL.md` is the
   contract that must survive any fix.
7. Open the project. **Edit → Plugins**, confirm these are enabled (restart if asked):
   - **Fable Bridge** (Project → Simulation)
   - **Chaos Vehicles**
   - **Python Editor Script Plugin**
   - **Unreal MCP** and **All Toolsets** (Editor → AI) — for Claude editing
   - **Remote Control API** — only if you use `mcp/codex_mcp.py` instead of / as well as the built-in MCP

## 2. Project settings that matter

**Edit → Project Settings**

| Setting | Value | Why |
|---|---|---|
| General Settings → Framerate → **Use Fixed Frame Rate** | on, **50** | every tick is exactly `dt = 0.02` regardless of rendering; the bridge relies on it |
| Physics → **Substepping** | on, Max Substep Delta 0.005, Max Substeps 4 | vehicle and buoyancy stability at 50 Hz |
| Physics → Framerate → **Tick Physics Async** | off | keep physics in lockstep with the tick |
| Python → **Additional Paths** | `<devkit>\tools`, `<devkit>\python` | lets editor Python import `sketch_to_heightmap` and `fable` |
| Python → **Startup Scripts** | `fable_toolset.py` | registers Claude's tools on editor start (5.8 MCP) |

Also **Editor Preferences → General → Model Context Protocol → Auto Start Server** = on
(port 8000). Then in the editor console: `ModelContextProtocol.GenerateClientConfig ClaudeCode`
→ `.mcp.json` appears in the project folder. Start Claude Code from that folder.

Set environment variable **`FABLE_SPECS_DIR`** = `G:\My Drive\applications\unreal_fable\specs`
(System → Advanced → Environment Variables) so the toolset finds your worlds. Restart the editor.

## 3. Blueprints (three, once)

Content Browser → new folder **`/Game/Fable`**. Everything below goes there with these exact names —
`fable_build.py` looks them up.

### BP_FableBridge
Blueprint Class → parent **FableBridgeActor**. Open it, **Fable | Spawn → Spawn Kinds**: add entries so
`spawn` from Python / Claude knows what to place at runtime:

| Key | Mesh | DefaultSizeM | Floats |
|---|---|---|---|
| `cone` | `/Engine/BasicShapes/Cone` | 0.36, 0.36, 0.55 | no |
| `barrel` | `/Engine/BasicShapes/Cylinder` | 0.58, 0.58, 0.90 | no |
| `wall` | `/Engine/BasicShapes/Cube` | 1, 1, 1 | no |
| `box` | `/Engine/BasicShapes/Cube` | 0.5, 0.5, 0.5 | no |
| `pole` | `/Engine/BasicShapes/Cylinder` | 0.12, 0.12, 2 | no |
| `rock` | `/Engine/BasicShapes/Sphere` | 0.9, 0.9, 0.7 | no |
| `buoy_red` / `buoy_green` / `buoy_yellow` | `/Engine/BasicShapes/Sphere` | 0.6, 0.6, 0.6 | **yes** |
| `moored_boat` | `/Engine/BasicShapes/Cube` | 3, 1.2, 0.8 | **yes** |

Swap in real meshes later; the keys are what matter. Compile, save.

### BP_FableBoatPawn
Blueprint Class → parent **FableBoatPawn**. It already has a cube hull scaled from the params;
optionally assign a nicer static mesh to **Hull** (keep *Simulate Physics* on). Compile, save.
Done — the boat needs nothing else.

### BP_FableCarPawn
Blueprint Class → parent **FableCarPawn**. Chaos Vehicles needs a **skeletal mesh with wheel bones**:

1. Easiest source: Epic Launcher → Samples → **Vehicle Template** (free). Create it once, migrate
   `SK_SportsCar` (+ its physics asset) into `FableSim`. Or use any RC-car skeletal mesh you have.
2. In BP_FableCarPawn: **Mesh → Skeletal Mesh Asset** = the car; set the mesh **scale** so its
   length matches your real car (SportsCar is ~4.5 m; a 0.5 m RC car is scale ≈ 0.11).
3. **Fable → Wheel Bone Names**: the four wheel bones of that mesh, order FL, FR, RL, RR
   (SportsCar: `Phys_Wheel_FL`, `Phys_Wheel_FR`, `Phys_Wheel_RL`, `Phys_Wheel_RR`).
4. Compile, save. Mass, wheel radius, steer angle, torque, speed are all overwritten from
   `vehicle_params.json` at play — do not tune them in the Blueprint.

> If the car falls through the floor or explodes at play: the physics asset's root body is
> not set to simulate, or the scale made the wheels tiny. Both are one-click fixes in the
> physics asset. If it does nothing: check Output Log for `LogFable: listening on 127.0.0.1:9800`
> and `(vehicle: ...)` not NONE.

## 4. Build a world and play

Editor console (`~`) or the Python console (Window → Developer Tools → Output Log → Python):

```python
import fable_build as fb
fb.new_level_and_build(r"G:/My Drive/applications/unreal_fable/specs/worlds/car_track_oval.json")
```

Or ask Claude: *"build the car_track_oval world"*. Press **Play**. In a terminal:

```
cd G:\My Drive\applications\unreal_fable\python
python examples\run_scenario.py car_track_oval --backend unreal
```

The car should drive two laps and print a scorecard. Do the same with `boat_buoy_course`.

## 5. Running headless / faster than real time

For batch testing, package or run the editor as a game without rendering:

```
UnrealEditor.exe D:\UE\FableSim\FableSim.uproject /Game/Fable/Maps/car_track_oval -game -nullrhi -unattended -log
```

With Use Fixed Frame Rate on, the sim advances 0.02 s per frame as fast as the CPU allows
— typically 5–20× real time for these scenes. Python's `step()` simply returns sooner.
Note `-nullrhi` disables rendering, so anything relying on the renderer (cameras) is off;
the lidar is line traces and works.

## 6. Checklist when something is off

| Symptom | Look at |
|---|---|
| `timeout waiting for 'ack'` in Python | is the level playing? bridge actor in level? port 9800 free? Windows Firewall prompt dismissed? |
| observations arrive but vehicle does not move | Output Log: `vehicle: NONE` → set **Vehicle** on FB_Bridge or add a pawn; car: skeletal mesh / physics asset |
| car drives but steering is inverted | you edited `FableConv.h`; ROS +steer is left |
| boat sinks / launches | `BuoyancyPointsM` z should be slightly below the CoM; `DraftM` must be > 0; substepping on |
| everything twitches at 50 Hz | Use Fixed Frame Rate off, or Tick Physics Async on |
| lidar sees nothing | spawned obstacles must block `Visibility`; check collision presets |
| Claude's tools missing | `ModelContextProtocol.RefreshTools`; `FABLE_SPECS_DIR` set; `fable_toolset.py` in startup scripts |
