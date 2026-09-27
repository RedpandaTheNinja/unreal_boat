# Boat physics implementation

The boat has a 13 ft x 3 ft hull, two fixed reversible motors approximately 24 in apart, a 130 lb base weight including equipment, and a 250 lb payload. Loaded mass is 380 lb (172.3651006 kg). The payload is configurable; this is a simulation operating point, not a certified real-hull payload rating.

## Implemented model

`Plugins/BoatPhysics` supplies `BoatDynamicsComponent`, attached to the existing boat actor. One Chaos body owns hull collisions and motion; decorative parts do not contribute extra mass. All calculations use SI units with explicit Unreal force/torque conversions. Body axes are X forward, Y starboard, Z up. Telemetry is simulator truth, not a navigation estimate.

Buoyancy integrates approximately 1,500 vertical columns through the modeled hull's sealed outer displacement envelope. Submerged volume supplies rho*g*volume upward force at each submerged column centroid. This is geometric quadrature, not an arbitrary set of supporting springs. Distributed water-relative heave damping and separate angular drag let trim, roll and pitch settle. The model reports deck immersion but does not simulate flooding of the open hull.

Each motor applies force at its stern mount. The 12 V Minn Kota Endura C2 30 nominal forward rating supplies 133.44665 N per motor. Commands are continuous [-1,1], as confirmed for the Arduino motor controller. A 0.06 deadband, 0.08 s command delay, 0.35 s thrust response and 60% reverse-thrust ratio are initial estimates. Propeller immersion and approximate forward-speed unloading affect delivered force. Optional voltage sag uses shared battery resistance; default resistance is zero until measured. Capacity, discharge, wiring losses and temperature are not yet identified.

Directional linear/quadratic water-relative drag and diagonal added mass/inertia include low-speed body-frame Coriolis terms. Chaos integrates motion and collision response; external fluid forces use effective inertia while collision impulses retain rigid-body mass. The model does not claim CFD accuracy, planing, propeller wake interaction, full coupled hydrodynamic tensors, wave radiation memory or gradient-current accuracy. Added mass is smoothly reduced when emerging; derivatives of that change are not modeled.

Physics uses Unreal's async fixed-step callback at 120 Hz, not the old AddCustomPhysics hook (which this engine version calls at frame time). Socket handling stays on game thread. In this UE version async component callbacks synchronize to game thread. The visible wave mesh and buoyancy share the same sinusoidal surface parameters and simulation timestamp. Wave displacement is interpolated over the mesh grid; normal-map ripples are visual detail only.

Base and payload centers of mass and inertias are estimated. Payload dimensions default to 1.2 x 0.6 x 0.4 m centered at body (0,0,0.18) m. Measured loading locations should replace these values. Diagonal inertia omits products of inertia for off-center payloads, so major asymmetric payload configurations require a full tensor extension.

## Operation

Open `/Game/BoatCourse/Lake_300x80ft`, select `BoatCourse/Boat/Boat_13ft_x_3ft_TwinMotors`. Physics properties are on `BoatDynamics`; edit before Play. The project opens this lake by default.

Play or Simulate enables flotation. The localhost UDP interface uses port 7450 and never opens Arduino serial ports. Send `{"left":0.4,"right":0.4}` repeatedly, or `{}` for read-only state. Commands expire after 0.5 seconds of simulation time; targets go neutral and thrust decays with motor lag. `{"reset":true}` restores the initial pose and clears motor history. Reset does not clear environment settings.

Keyboard Control is enabled on the saved BoatDynamics component. Use Play and click the viewport: I/K drive left motor forward/reverse; O/L drive right motor forward/reverse; Space commands neutral. Hold I+O for forward, K+L for reverse, I+L or K+O for opposite-thrust turning. Neutral is not an instant brake. Releasing keys lets the 0.5 s timeout neutralize the motors. The input controller must exist for keyboard controls; the UDP client also works in Simulate. Disable Keyboard Control during automated tests to prevent competing commands.

The component activates before registering its async physics callback, preventing the inactive-component assertion seen during initial testing. The body uses NeverSleep because continuously applied fluid forces must remain effective at low speed.

Run with the existing project Python environment:

```
python simulation_ml/boat/boat_client.py --seconds 5 --left 0.4 --right 0.4
python simulation_ml/boat/validate_live.py
```

Use the same Python environment that has numpy/scipy installed for calibration. The client can write JSONL telemetry with `--output`. Its port is simulator-only. Do not feed evaluator ground truth directly to the future camera-based navigation policy.

## Calibration and real-water validation (pending measurements)

1. Measure loaded mass, draft/freeboard, motor shaft spacing/depth and payload position. Photograph the boat's resting trim with the 250 lb payload secured in its intended location.
2. Record command, measured voltage and bollard thrust for each motor in both directions across several command levels. Record step response and neutral decay. Do not derive actual dynamic thrust from the nameplate rating alone.
3. Log calm-water straight accelerations at several commands followed by neutral coast-down. Use an independent speed/position measurement; Cube/Here4 are planned, not installed.
4. Log clockwise/counterclockwise circles, opposite-thrust turns, forward-to-reverse stopping, and lateral drift/recovery. Repeat runs for holdout validation, rather than tuning to every run.
5. Fit motor and surge parameters first, then sway/yaw and trim/inertia. `calibrate_surge.py` expects separate training and validation CSV files with `time_s,speed_mps,thrust_n`. Thrust must be independently measured/estimated with a documented model. It fits effective surge mass and nonnegative linear/quadratic drag and reports held-out force error. It does not silently update Unreal parameters or claim to identify other axes.
6. Compare separate held-out runs for speed, coast-down, turn rate/radius, stopping distance and static trim. Agree accuracy thresholds based on navigation tolerances. Record parameter provenance and retain the original data. Only then mark the vessel calibrated.

No real-boat logs have been supplied, so hydrodynamic calibration remains pending even if all simulation tests pass.

## Sources

- User-provided listing: https://www.amazon.com/dp/B0077OMFKK (30 lb thrust, 30 in shaft selection).
- Manufacturer: https://minnkota-help.johnsonoutdoors.com/hc/en-us/articles/15447707630359-Endura-Features-Specifications-and-Manuals-2011-present
- Marine model background: https://www.fossen.biz/html/marineCraftModel.html
- Unreal buoyancy comparison: https://dev.epicgames.com/documentation/unreal-engine/water-buoyancy-component-in-unreal-engine

The stock Endura speed switch has 5 forward/3 reverse settings; this installation uses continuous motor-controller commands per the user. The stock shaft length is not assumed to equal installed propeller depth.
