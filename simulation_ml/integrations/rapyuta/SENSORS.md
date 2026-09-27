# Sensors for autonomous development

## Installed and validated in Windows UE 5.8.2

The sensor DLL is installed and loaded. Editor, Development, and Shipping builds
passed. Live validation on 2026-09-19 verified both 360-ray scans against a box
with a known face position: front range approximately 0.55 m and rear 0.85 m.
The rear ray reached the box through the ignored front robot, confirming tandem
self-filtering. The obstacle-stop example held zero speed with the box present;
after removing it, the clear-path test traveled 0.265 m in approximately 3 s.
Maximum front-scan age during that test was 0.0335 s. These are ideal-simulation
checks, not physical sensor accuracy claims or a completed navigation test.

Evidence: `validation/sensor_validation_summary.json`,
`validation/lidar_obstacle_stop.jsonl`, and `validation/lidar_clear_drive.jsonl`.
The temporary box was removed and the level saved after validation.

**Editor Performance > Use Less CPU when in Background** was disabled for these
tests. Keep it disabled while running an external Python controller; otherwise
background throttling can reduce the simulation to around 3 FPS and the example
will correctly stop on stale scans.

Upstream source: [RapyutaSimulationPlugins UE5.5](https://github.com/rapyuta-robotics/RapyutaSimulationPlugins/tree/UE5.5/Source/RapyutaSimulationPlugins).
Pinned commit and asset hashes are in `upstream_assets.json`.

The original Burger assembly includes `LidarComp` on its physical lidar body.
The upstream plugin separately provides IMU, camera, and odometry components;
their availability in the repository does not mean they are all fitted to this
Burger Blueprint. The Windows port now implements range sensing on both existing
lidar mounts. A front-chassis RGB camera and IMU are now installed and validated
on each tandem; see [RGB_IMU.md](RGB_IMU.md) for calibration, data conventions and
the image client. Depth camera, ROS topics and encoder odometry are not implemented.

## Available interface

Start **Play > Selected Viewport**. Send `{"sensors":true}` to localhost UDP 7447.
This request reads sensors without changing the drive command. Add `v_mps` and
`yaw_radps` to the request when controlling the motors. A second tandem uses 7448.

The response schema is `tandem_sensors_v1`:

- `front_scan`, `rear_scan`: 360 rays each, 0.12–3.5 m, nominal 30 Hz in simulation
  time. Actual updates depend on frame rate. `sequence` identifies new scans;
  repeated polls can return the same cached scan.
- `stamp_s`: simulation timestamp of the scan; compare with response `time_s`.
- `ranges_m`: metres; `null` means no return within range. No NaN/Infinity JSON.
- Angles start at zero (+X forward), increase counterclockwise (+Y left), with
  `angle_increment_rad = 2*pi/N`. There is no duplicated 360-degree endpoint.
- `sensor_pose_ground_truth`: position and orientation at acquisition. This is
  available for debugging/projection; it is not a localization measurement.
- Existing top-level `x_m`, `y_m`, `yaw_rad`, velocities, and separation are
  explicitly labeled `chaos_ground_truth_not_encoder_odometry`.
- `imu`: timestamped specific force (m/s²) and angular velocity (rad/s).
- `rgb`: cached camera metadata. Request `{"rgb":true}` to acquire an image;
  the supplied `rgb_imu_client.py` retrieves its sequence-checked JPEG chunks.

Range geometry follows the upstream RR2DLidar raycast method, using Visibility
complex traces and the original sensor mount. Upstream's range defaults are in
centimetres despite the header's metre comments; its ROS conversion confirms this.
Both robot halves are excluded from self-returns. Other tandems remain detectable.
Green points in Unreal show returns; toggle `bShowLidarRays` on the lidar component.

This adaptation uses synchronous snapshot scans. `time_increment_s=0`; it does
not simulate rolling-scan distortion, upstream Gaussian noise, or material
intensity response. It labels the model `ideal_range_only`. Physics bodies,
constraints, motor forces, masses, and materials are unchanged.

## Record and develop

```powershell
python simulation_ml/integrations/rapyuta/sensor_client.py --seconds 10
```

This records JSON Lines to `validation/sensors.jsonl` without sending motor
commands; you can drive with I/K/J/L while recording.

```powershell
python simulation_ml/integrations/rapyuta/sensor_client.py --drive --seconds 10
```

`--drive` runs a **slow obstacle-stop example**, not a track navigator. It moves
forward at 0.1 m/s and stops when a return falls within a 0.36 m wide corridor,
0.65 m ahead of the front lidar. Missing/stale scans stop the command. Motor
commands time out after 0.25 simulation seconds. Avoid concurrent keyboard/UDP
driving while evaluating a controller.

The sensor plane is near 0.18 m above the track. Low objects can pass below it;
no-return rays also cannot establish that the full vehicle footprint is clear.
Ground paint and the orange racing-line overlay are not track-boundary sensors.

## Next algorithm stages

1. Use the two scan extrinsics to express returns in one vehicle frame. Build a
   local occupancy map; inflate obstacles by the **whole tandem footprint**.
2. Use the surveyed track boundary as a map constraint. Localize using lidar
   scan matching; combine the implemented IMU with encoder odometry once encoders are added.
   Ground-truth pose can validate localization error but should not silently
   become the algorithm's sensor input.
3. Build a speed/yaw-rate trajectory sampler using measured skid-steer response.
   Reject paths that leave the track or hit inflated obstacles. Include stopping
   distance and stale-data handling before optimizing progress/lap time.
4. Validate sensor occlusion, narrow obstacles, below-scan obstacles, slip,
   braking, missed scans, and randomized obstacle placement. Add noise and
   latency deliberately after the ideal-sensor baseline works.

The old 44.66-second line remains a reference, not a validated tandem lap.
