# Windows UE 5.8 TurtleBot tandem port

Target: two original **physics-based TurtleBot3 Burgers**, facing the same direction,
joined front-to-back by a six-degree-of-freedom locked constraint. Four driven
wheels; both original passive ball casters remain. Default base spacing: 0.30 m.
This is differential/skid steering, not Ackermann steering.

## Current status: installed and tested

The plugin is compiled and enabled in `mcp_gpt.uproject`; the tandem is saved in
`/Game/ThirdPerson/Lvl_ThirdPerson`. UE 5.8.2 loaded the original Blueprint and all
ten simulated bodies. Each module retains its mass overrides: base 1 kg, lidar
0.1 kg, each wheel 0.5 kg, caster 0.25 kg (4.7 kg total for the pair).
The connection is a physics constraint; no cosmetic connecting bar is added.

Both lidars are now active and validated; see [SENSORS.md](SENSORS.md) for
recording sensor data and running the obstacle-stop example. This addition does
not change the robot's masses, materials, motors, or constraints.

Measured in Chaos on the existing track (see `validation/summary.json` and traces):

| Check | Result |
|---|---|
| Forward, 0.15 m/s command | 0.412 m traveled over 2.97 s |
| Turn, 0.1 m/s and 0.5 rad/s command | 0.205 rad yaw change over 3.98 s |
| Rigid 0.30 m spacing | Maximum error 0.043 mm across these traces |
| Zero-speed command | 0.179 mm net displacement over 0.94 s |
| Command timeout | 0.626 mm drift during the final settled observation window |

Turning response is about 0.051 rad/s in this test, far below the 0.5 rad/s
demand because the joined vehicle must scrub its wheels sideways. This short
smoke test is not a track-completion, obstacle-avoidance, or lap-time validation.
Small contact oscillations remain at rest; the brakes do not freeze bodies.

Compilation passed for Editor Development, Game Development, and Game Shipping
with MSVC 14.44.35229. AutomationTool's final temporary-folder deletion failed
under OneDrive after producing the binaries; those compiled binaries were installed
and exercised in-editor. The build script now uses a short, unique build folder
and `-NoDeleteHostProject` to avoid that cleanup step on subsequent builds.

## Physics provenance

The project plugin is `Plugins/RapyutaSimulationPlugins`. It preserves the original
module/content mount so the upstream data-only `BP_TurtlebotBurger` can retain its
serialized component settings. It is a **physics-only port**, not a full installation
of RapyutaSimulationPlugins. Do not install the upstream plugin beside it.

Pinned source: RapyutaSimulationPlugins `UE5.5`, commit
`c8f2c4804c580b4dc84ae4082060897f16ee7f52` (Apache-2.0).
See `upstream_assets.json` for SHA-256 hashes of the unchanged copied assets.
The C++ assembly comes from `TurtlebotBurgerBase.cpp`, `TurtlebotBurger.cpp`,
and the motor target math from `DifferentialDriveComponent.cpp`.

- Wheel radius 3.3 cm; controller half-separation 7.9 cm (upstream values).
- Wheel joint locations `(3.2, -8, 2.3)` and `(3.2, 8, 2.3)` cm.
- Lidar mount `(0, 0, 17.2)` cm, fully locked.
- Caster mount `(-4.9, 0, -0.5)` cm, translation locked, rotation free.
- Wheel angular velocity drive: position strength 0, velocity strength 1000,
  force limit 1000, as set by upstream `SetWheels`.
- Mesh collision, mass overrides, damping, COM, inertia scaling, and physical
  materials remain in the original Blueprint/assets. No guessed replacements.
- Wheel velocities are sent to Chaos constraints in revolutions/second; no
  tick-based position teleporting or substitute vehicle movement model.

The new rigid joint changes the assembled vehicle's dynamics. Moving from the
upstream engine to Chaos in UE 5.8 can also change solver results. Preserving the
definitions does **not** establish numerical equivalence or the same lap time.
Ground/contact material belongs to the current track, not the robot port.
The browser/Python simulator is unchanged by this integration.

## Build and place

1. Run `./simulation_ml/integrations/rapyuta/build_port.ps1` in PowerShell.
   UE 5.8 rejects early MSVC 14.44 releases; use a supported updated toolchain.
2. Enable **TurtleBot Windows Physics Port** and restart the editor after building.
3. In Unreal's Python console, execute `setup_tandem.py` from this folder.
   It loads `/Game/ThirdPerson/Lvl_ThirdPerson`, adds the tandem on the oval,
   saves the map, and writes `validation/loaded_physics.json`.
4. Start PIE or Simulate. Inspect the actor under
   `Fable_Oval_Preview/TandemTurtleBot`.

Do not run setup against a map with unsaved edits. The port is disabled by default
until its binary has been built, so an incomplete build does not prevent the
existing project from opening.

## Control and measurement

The track now overrides GameMode with `/Game/Fable/TurtleBot/BP_TurtleBotGameMode`.
Its default pawn is None, so Play does not spawn the template human.
`TurtleBot_FollowCamera` auto-activates for Player 0 and is attached to the
selected tandem's front base, with offset (-130, 0, 85) cm and pitch -25 degrees.
Use **Play > Selected Viewport**, then click inside the viewport for keyboard
focus. The second tandem's keyboard control is disabled and its UDP port is
7448; the driven tandem uses 7447. Actor selection does not route keyboard input.

In PIE, focus the viewport: **I/K** forward/reverse, **J/L** left/right,
**Space** stop. The initial manual speed is 0.15 m/s and yaw demand 0.5 rad/s.
These are command defaults, not altered motor force or physics limits.

For repeatable tests, send commands at 20 Hz:

```powershell
python simulation_ml/integrations/rapyuta/control_tandem.py --speed 0.15 --seconds 3 --output simulation_ml/integrations/rapyuta/validation/forward.json
python simulation_ml/integrations/rapyuta/control_tandem.py --speed 0.10 --yaw-rate 0.5 --seconds 3 --output simulation_ml/integrations/rapyuta/validation/turn.json
```

UDP binds only `127.0.0.1:7447`. Request:
`{"v_mps":0.15,"yaw_radps":0.0}`. Reply returns actual Chaos position,
yaw, linear velocity, inter-base separation, and simulation time.
A 0.25-second command timeout sets both motor targets to zero.
This is motor braking, not an instantaneous freeze.
Invalid/non-finite commands do not refresh the timeout.

Coordinates match the track preview: external `x=(UE_X-10000)/100`,
`y=-UE_Y/100`, `yaw=-UE_yaw_degrees*pi/180`. Pose is the midpoint of the
two base origins, not a GPS point or the front axle.

## Path-finding work still required

The existing 44.66-second line is an earlier Ackermann simulation recording.
It remains a useful visual reference, not a measured tandem lap.
First validate straight travel, yaw direction, braking distance, rigid-joint
separation, and contact stability. Then measure feasible speed/curvature under
this vehicle's lateral wheel scrub and the current track friction.

`RR2DLidarComponent` now provides timestamped ideal range scans from both original
mounts, with the complete tandem excluded. See [SENSORS.md](SENSORS.md) for the
interface, recorder, obstacle-stop example, and sensor-model limitations.
RGB camera and chassis IMU are also installed on both tandems. See
[RGB_IMU.md](RGB_IMU.md) for defaults, units, calibration and the Python client.
Before autonomous obstacle avoidance, test obstacles at wheel/body height and
provide sensing for obstacles below the lidar plane.
Use the entire tandem footprint for boundary and obstacle clearance. Adapt the
planner to `(forward speed, yaw rate)`, track the measured pose, and optimize
lap time only after collision-free/boundary-contained laps pass. Do not feed
Ackermann steering-angle commands directly into this interface.
