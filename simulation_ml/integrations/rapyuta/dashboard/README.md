# Mini F1 live dashboard

Open **http://127.0.0.1:8765** while the server is running. Start Unreal Play, then click the Unreal viewport for keyboard driving. The dashboard is read-only; it never sends throttle, speed, steering or brake commands.

Start it from the project workspace:

```powershell
& ./simulation_ml/integrations/rapyuta/launch_dashboard.ps1
```

The launcher starts a hidden Python server and opens the default browser. `-NoBrowser` starts only the server. Re-running it reuses a running dashboard. After reboot, run the launcher again. Python is taken from the existing UE 5.8 installation; no extra packages or external web resources are required.

## Displays

- Speed in m/s and km/h, plus forward/reverse indication.
- IMU body-frame specific force and angular velocity. Specific force includes gravity support, so Z is approximately +9.81 m/s² at rest.
- Front/rear LiDAR returns transformed using their sensor poses, a vehicle-relative radar, and a selectable 0.5–2.0 m proximity alert. The forward corridor is ±0.35 m laterally, starting at the car's nose (0.30 m ahead of its origin). This is a return-proximity indication, not object classification, collision avoidance, or an automatic brake. Null ranges mean no return, not guaranteed free space.
- RGB JPEG at up to 5 fps, with simulation timestamp and frame age.
- Circuit outline, car heading and recent trail, progress around the 360 m track, and approximate edge clearance based on a 0.60×0.25 m vehicle envelope. Track and car positions use simulator ground truth, not SLAM/GPS. The initial/finish-line neighbourhood is shown as 0.0% to avoid a rounding jump to 100%.
- Individual wheel steering angles and ground contact; both front wheels are limited to ±30°.

Telemetry is requested at approximately 10 Hz. Unreal load can reduce these rates. Offline/paused states and stale camera frames are explicitly marked. Only one service should fetch RGB directly from the car's UDP image cache; multiple browser viewers can share this dashboard server.

## Local interfaces and evidence

Server binds only `127.0.0.1:8765` and reads F1 UDP port 7449. `/api/state` exposes the latest telemetry and freshness, `/api/track` the reference track, and `/camera.jpg` the latest JPEG. The page has a current-telemetry download button. No actuator HTTP endpoint exists.

Logs: `../validation/dashboard.log` and `dashboard_errors.log`; PID is in `dashboard.pid`. To stop, identify the server process using that PID and stop that process; do not stop unrelated Python processes.

Tests: `test_dashboard.py` covers map coordinate conversion and envelope margin, LiDAR mount rotation/null filtering/stale scans, and initial offline state. Live evidence is in `../validation/dashboard_obstacle.json` and `dashboard_offline.json`. A temporary 20×40×40 cm cube about 1 m ahead produced 50 returns and an approximately 0.62 m nose clearance; the cube was removed after testing. The page was rendered and visually inspected in isolated headless Chrome because the interactive browser/native automation connectors were unavailable.
