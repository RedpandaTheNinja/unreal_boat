# RGB camera and IMU: Windows tandem

One camera and one IMU are attached to the front physical chassis of each tandem. They follow the actual Chaos body, not the assembly actor's stationary origin. They add no collision, mass, forces or constraints. The original lidar interface and command ports remain intact.

Installed and live-validated on 2026-09-19 in UE 5.8.2. Editor, Development and Shipping builds passed, as did six client tests. Installed DLL SHA256: `9BA558E1ED2BF2EC056BFB141EEE7242AE72A64878A22C6B9F0C50CB34EC383B`. All 25 pinned upstream assets still match their original hashes.

The bounded low-speed acceptance run measured 9.8013 m/s² mean stationary specific force, approximately 60 Hz actual IMU updates, maximum sample age 16.84 ms, and 0.2545 m forward travel. Mean gyro Z during the turn was 0.07549 rad/s versus 0.07741 rad/s from pose change. Both lidar scans retained 360 samples; JPEG output was 640 x 480 and changed with motion. The second tandem also returned RGB and IMU data on port 7448. Evidence is in `validation/rgb_imu_acceptance/summary.json`, its packet/image files, and `validation/rgb_imu_secondary/`. These are functional simulator checks, not commercial sensor accuracy certification or an autonomous lap test.

## Selected defaults

| Setting | Value |
|---|---|
| RGB image | 640 x 480, color, JPEG quality 90 |
| Horizontal field of view | 90 degrees |
| Camera mount from front Base origin | 8 cm forward, 0 sideways, 22 cm up |
| Camera angle | 12 degrees down |
| Image timing | On request; maximum 10 captures per simulation second |
| IMU mount | Front Base origin; aligned with chassis |
| IMU timing | Post-physics tick; requested maximum 100 Hz, limited by frame rate |
| IMU model | Ideal gyroscope and accelerometer; no added noise or bias |
| Network | Localhost UDP 7447; second tandem 7448 |

These are simulation defaults, not a calibrated model of a commercial sensor. Camera pixels include rendered scene lighting. There is no lens distortion, rolling shutter or calibrated exposure/noise model. JPEG is lossy. The synchronous GPU readback can stall the game thread; this first implementation is for development, not high-throughput or deterministic RL stepping.

## Read the data

Start Play in Unreal. From the project root:

```powershell
& 'C:/Program Files/Epic Games/UE_5.8/Engine/Binaries/ThirdParty/Python3/Win64/python.exe' simulation_ml/integrations/rapyuta/rgb_imu_client.py --frames 10 --interval 0.2
```

This writes JPEG images and matching JSON metadata to `validation/rgb_imu`. It does not send motor commands. Use a separate controller or the keyboard to drive. For the other tandem add `--port 7448`. OpenCV decodes JPEG to BGR by default; convert to RGB if the policy expects RGB. Pillow `.convert('RGB')` provides RGB.

Python API: `SensorClient.sensors()` reads lidar and IMU; `SensorClient.camera()` returns `(jpeg_bytes, metadata)`. Close the client when finished.

`{"sensors":true}` retains schema `tandem_sensors_v1` and adds `imu` and cached `rgb` metadata. It does not trigger image rendering. `{"rgb":true}` captures an image or returns the recently cached frame, plus a separately timestamped IMU sample. `{"rgb_sequence":N,"rgb_chunk":I}` retrieves a base64 JPEG chunk. Chunks contain at most 24,000 binary bytes. One cached frame is retained; mismatched sequences fail explicitly. Use one image reader per tandem, and the supplied client to handle retries and reassembly. Polling faster than the camera cap may return the same sequence.

## What the IMU values mean

- `linear_acceleration_mps2`: sensor-frame **specific force**, acceleration minus gravity, in m/s². A stationary level sensor reads approximately `(0,0,+9.81)` rather than zero. On a slope, gravity projects onto multiple axes. Free fall approaches zero specific force.
- `angular_velocity_radps`: rotation rate in rad/s in right-handed local axes, X forward, Y left, Z up. Positive Z means a left/counterclockwise turn.
- `orientation_ground_truth_xyzw`: exact simulator orientation for diagnostics. This is not an orientation estimate from accelerometer/gyro fusion and should be excluded from a realistic sensor-only policy.
- `stamp_s`, `sequence`, `sample_dt_s`, `valid`: acquisition timing and validity. Initial acceleration is invalid until two velocity samples are available. Actual update rate is `1/sample_dt_s`, not automatically 100 Hz.

Acceleration is a finite difference of the physical body's velocity at the sensor point, then transformed into sensor axes. The gyro measures the shortest quaternion rotation between successive actual chassis poses divided by the sample interval. This includes realized constraint motion instead of assuming Chaos's solver angular velocity integrates exactly to the observed pose. Contact vibration and variable timestep can produce spikes even without artificial noise. No covariance or accuracy claim is made. A future reset/teleport protocol must clear sample history before resuming measurements.

## Calibration and coordinate information

Camera metadata supplies image size, horizontal FOV, pinhole intrinsic matrix `K` (row-major), zero distortion coefficients, mount translation and rotation, timestamp and sequence. The camera optical convention is X right, Y down, Z forward. Mount quaternion metadata explicitly uses forward/left/up camera axes; convert camera FLU to optical using `optical=(-left,-up,forward)`. The quaternion maps camera FLU vectors into chassis FLU coordinates.

Camera world pose and IMU orientation fields named `ground_truth` are diagnostic information. World translation retains the existing bridge convention `x=(UE_X-10000)/100`, `y=-UE_Y/100`, `z=UE_Z/100`. For the new track's JSON coordinates, subtract 50 m from bridge X. Do not mix chassis, optical, bridge and track coordinates.

Camera, IMU and lidar are timestamped individually and are not hardware-synchronized. Camera capture happens when requested; IMU and lidar are cached post-physics samples. Align by simulation timestamps and reject stale data. A later fixed-step training interface should explicitly synchronize them.

Keep **Editor Preferences > Performance > Use Less CPU when in Background** disabled when a Python client has focus. Otherwise this editor can throttle to about 3 FPS. The camera retains its rendering state between requested captures so automatic exposure can settle; allow several warm-up frames before using pixels for learning.

## Information needed to match real hardware later

No additional information is needed for the accepted defaults. To match a physical car, provide:

1. Camera model, resolution/frame rate, lens/FOV or calibrated K/distortion, shutter and exposure settings.
2. Measured camera and IMU mounting position and orientation relative to the chassis.
3. IMU model, sample rate, full-scale ranges, accelerometer/gyro noise and bias/drift specifications, and whether an onboard orientation estimate is available.
4. Required output transport (Python/UDP now; ROS2 would require additional integration), timing/latency expectations and whether lossless pixels are required.

For autonomy, use RGB for white-line perception, lidars for obstacle ranges, and IMU for rotation/acceleration. IMU alone does not provide stable absolute position; localization still needs a map and additional measurements such as wheel odometry or visual/lidar localization.
