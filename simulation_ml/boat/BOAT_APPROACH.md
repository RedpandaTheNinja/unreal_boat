# AIMM-ICC boat simulation approach

Target project: `mcp_gpt_boat`, Unreal Engine 5.8. This replaces the car-racing objective for this project. Status: the 300 ft x 80 ft lake, 13 ft x 3 ft boat and fixed-step twin-motor physics are implemented. Simulator behavior checks are recorded in validation/physics/acceptance.json. Real-boat calibration, depth rendering, autopilot bridge and mission execution remain pending. See IMPLEMENTATION.md for controls and physics assumptions.

## Confirmed hardware and objective

- Boat length 13 ft (3.9624 m), width 3 ft (0.9144 m), loaded mass 380 lb (130 lb base plus 250 lb payload); two fixed stern motors with reverse; differential-thrust steering.
- Jetson Orin Nano performs image identification using a ZED2i depth camera and a separate RGB webcam.
- Arduino operates motors and connects to Jetson over serial.
- Cube Orange+ and Here4 GPS/RTK are confirmed planned additions, not installed hardware. The current boat has no operational IMU reported. Current motor command path is Jetson -> serial -> Arduino -> two motors. The planned Cube connects to Jetson.
- Support predefined waypoint missions and autonomous path planning, with task-specific final approaches.
- Sampling has a fixed time period. Distance between samples varies with realized boat speed; do not equate sample index with distance or automatically stop at every sample.

## Course evidence

Source: `boat_course/AIMM-ICC Competition Course Details 12-13 2025.pdf`, all 15 pages reviewed; nine JPEG references reviewed. Page 2 contains a course map and Google Earth view centered at latitude 41.70238808, longitude -85.02280627. This is a map-view center, not a surveyed buoy coordinate or launch origin. Use map dimensions for relative geometry; exact geographic registration remains unverified.

Page 2 labels include 15, 30, 40, approximately 45, and 50 feet. Upper horizontal labels, read left to right: 40, 15, 30, 30, 30, 30, 30, approximately 45 ft. Left vertical labels: 30, 15, 15 ft. Lower horizontal labels: 40, 15, 50, 50, 50, approximately 30 ft. Launch target offset: 15 ft. These labels describe different objects and segments; do not force their sums into an exact rectangle or treat every label as a waypoint interval. The top route proceeds away from the dock toward the left, turns down, then returns toward the dock along the bottom.

Photos show docks, wooded shoreline, colored floating markers, water reflections, clear-weather and substantial fog references. They establish appearance, not measured fog visibility or surveyed geometry.

## Mission behavior

| Challenge | Completion behavior |
|---|---|
| 1 Gate Navigation | Leave dock untethered; whole hull passes gate; distinguish partial crossing and buoy contact. Required for competition scoring. |
| 2 Dodge | Follow buoy slalom rules; time gate A to gate B; distinguish contacts. |
| 3 Evade | Navigate channel and model course detector exposure separately from navigation sensors. |
| 4 Identify | Find selected colored buoy and intentionally contact it. |
| 5 Deploy | Approach zebra buoy; release untethered floating/above-water sensor within 6 ft (1.8288 m) radius at initial deployment. This is payload placement, not boat-center arrival tolerance. |
| 6 Launch | Approach black buoy; align for toy-football target; score release and landing separately. |
| 7 Recover | Find floating case; distinguish capture in water, full removal, and damage. |
| 8 Receive | At dock, receive and validate deployed sensor data. |
| 9 Return | Contact marked return buoy. |

Page 5 says navigate to the right side of green buoys and left side of red buoys. Use travel direction and diagram semantics when constructing traversal checks. The supplied rules allocate one hour, require challenge 1 for a scored run, and distinguish remote intervention and GPS-independent bonuses. These are the supplied 2025 rules, not a claim about a later competition revision.

Represent mission phases as transit, acquire target, approach, perform/hold, verify completion, depart, and abort/retry. Holds apply where the task needs them; success must not be awarded merely for reaching a waypoint. Standoff, heading tolerance and hold duration remain task-specific parameters pending mechanism details.

## Intended architecture

Unreal owns hull motion, water/current/wind disturbances, collision geometry, visual scene and synthetic sensors. ArduPilot Rover/Boat SITL represents Cube behavior during software simulation. Jetson-compatible perception and mission logic consumes timestamped camera/depth and navigation estimates, chooses task goals, and communicates with the autopilot. Motor commands drive a simulated Arduino/actuator adapter with the same eventual serial contract as the real boat. Maintain separate current-hardware and planned-autopilot configurations. The current-hardware configuration uses Jetson-to-Arduino commands and must not assume Cube/Here4 measurements. The planned-autopilot configuration uses simulated GPS/IMU and ArduPilot SITL; final Cube-to-Jetson-to-Arduino command routing remains an implementation decision. Only one controller may own motor outputs at a time.

Use ArduPilot's JSON physics interface for the SITL backend and MAVLink for mission/guided control. Neither is already implemented in this project. Keep coordinate conversions explicit: Unreal centimeters and axis conventions, local navigation meters, and ArduPilot NED/body conventions. Test positions, gravity, yaw sign and motor order independently.

The current Rapyuta integration documents working RGB/IMU for a wheeled tandem, but no depth camera. Its chassis, odometry and motor mapping are not a boat model. Existing Fable boat specs describe a 1.15 m placeholder hull, not the real 13 ft vessel. Reuse infrastructure only after separating vehicle-specific assumptions.

Start depth rendering with explicitly labeled ideal metric depth, then model ZED-like invalid pixels, range limits, latency and degradations. A rendered depth buffer is not a validated ZED2i stereo simulation. Fog must affect perception tests; simulator ground truth is reserved for evaluation and must not silently feed autonomous perception.

Predefined mission points and recorded fixed-period samples are separate representations. Track timestamps and actual pose displacement; use segment-aware waypoint progression to handle irregular spacing and avoid skipping challenge actions. Obstacle avoidance uses full hull footprint and stopping behavior, not a point vehicle. Fixed stern thrusters cannot provide independent lateral translation, so holding position and heading may compete under cross-current.

## Implementation sequence and acceptance

1. Build a separate boat map using measured course labels and source-inspired dock, shoreline and buoy visuals. Mark unsurveyed coordinates and placeholder hull parameters.
2. Implement boat dynamics and twin-motor inputs; test straight motion, turns, reverse if supported, stopping, current drift and command timeout.
3. Attach RGB, depth, GPS and IMU simulation with timestamps and mounts. Validate metric depth against known geometry, axis conventions and stale-data handling.
4. Close ArduPilot SITL loop; verify unevenly spaced mission points and motor ownership, then run without hardware attached.
5. Implement perception-guided approaches and challenge state machine. Separate intentional target contact from forbidden collision.
6. Evaluate course completion, full-hull clearance, approach error, hold behavior, task completion, sensor dropouts, fog, wind/current and GPS degradation. Report Unreal and mock results separately.
7. Connect physical Jetson/Cube/Arduino only after their interfaces are confirmed. Measure hull width, loaded mass, motor spacing, thrust/reverse behavior, camera mounts and command latency before claiming physical fidelity.

## Open integration questions

- Confirmed: fixed motors with reverse. Motor-specific thrust curves and reverse limits remain unmeasured.
- Cube and Here4 are not installed; no existing Cube motor interface needs to be reproduced. Arduino firmware and serial protocol definitions are still needed to match current hardware.
- Loaded beam, mass/draft, motor spacing, camera positions and challenge mechanism reach are not supplied.
- Fixed sample period, per-challenge hold durations and precise target-relative approach tolerances are not supplied.

## Official integration references

- https://ardupilot.org/rover/docs/boat-configuration.html
- https://ardupilot.org/dev/docs/sitl-with-JSON.html
- https://ardupilot.org/rover/docs/loiter-mode.html

ArduPilot boat configuration uses Rover with boat frame class. Loiter permits drift within its configured radius; it must not be treated as guaranteed precision docking or fixed-heading station keeping.



