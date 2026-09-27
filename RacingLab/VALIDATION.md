# Validation — 2026-09-20

## Bump diagnosis

Two 12-second straight drives used 0.6 m/s target speed on the start section. Actual speed was lower than the target because of the existing speed controller. Geometry, body clearance and suspension tuning were unchanged; the second drive enabled physics substeps. Physics settings were read back from the running UE settings object and saved in `Config/DefaultEngine.ini`: bSubstepping=True, MaxSubstepDeltaTime=0.008333333, MaxSubsteps=8. These settings apply to project physics, including other simulated actors, not only F1.

| Measurement | Previous frame-step physics | Substeps enabled |
|---|---:|---:|
| Samples | 231 | 236 |
| Four-wheel contact | 100% | 100% |
| Estimated hull clearance | 33.3–36.9 mm | 33.3–34.4 mm |
| Vertical specific-force standard deviation | 0.8593 m/s² | 0.00109 m/s² |
| Vertical specific-force min/max | 6.46 / 18.52 m/s² | 9.786 / 9.798 m/s² |

Evidence: `../car_model/bump_baseline.json`, `../car_model/bump_substep.json`. Reproduce the analysis with UE Python: `RacingLab/analyze_bumps.py car_model/bump_substep.json` from project root. Clearance is approximate, using centreline elevation and the hull's -2 cm lower bound; it is not a contact sensor or exact swept-body clearance. The result supports integration timing as the cause of the observed start-section oscillation, not low ride height. It does not rule out geometry/collision or clearance issues elsewhere, especially at raised curbs, crests or higher speeds. No road mesh or car geometry was changed in this task.

## Actual Unreal RL run

The demonstration used the live Chaos F1, UDP 7449, ground-truth pose, the revised 359.97 m course and a 0.6 m/s target. The dashboard could remain connected because the controller did not fetch RGB images. Each evaluation/baseline started with Unreal Stop/Play at the saved start pose. No synthetic results were substituted.

| Run | Duration | Progress | Mean / max absolute cross-track error | Minimum estimated edge margin |
|---|---:|---:|---:|---:|
| Online training | 60 s | 31.50 m | 5.21 / 8.91 cm | 37.32 cm |
| Frozen policy evaluation | 30 s | 14.25 m | 2.244 / 4.446 cm | 42.85 cm |
| Pure-pursuit baseline | 30 s | 14.25 m | 2.253 / 4.452 cm | 42.84 cm |

Training performed **1,074 Q updates**, visiting only **2 of 25 states**. All three runs finished their requested duration without safety termination. Evaluation performed zero learning updates. The saved table prefers zero residual in both visited states, so this trial demonstrates the training/evaluation pipeline but **does not establish improvement over baseline**. Tiny differences are timing variation, not evidence of a better policy. There is no full lap, high-speed, unseen-start, obstacle-avoidance or sensor-only autonomy claim.

Raw evidence and settings are in `runs/*_train.json`, `runs/*_evaluate.json`, `runs/*_baseline.json`; trained values and visits in `policy.json`. The next run changes `latest_summary.json` but leaves timestamped records intact.

## Handover and software checks

- Offline installation check finds the project, map, native DLL, track and Python runtime scripts.
- Geometry tests verify Unreal-to-track coordinates, steering correction sign and finish-line reward wrapping. Safety test verifies edge, lost-contact and stale-IMU termination using recorded telemetry.
- Exporter dry-run inventories approximately 1.32 GB and excludes caches/venvs. It requires a new target directory and never overwrites one. Optional source-machine editor automation plugins are removed from the exported project descriptor.
- PowerShell common launcher's `check` and `dashboard` actions succeeded with the UE bundled Python; the local browser was opened. Dashboard launch uses the same tested server, configurable ports and engine path. All four geometry/safety tests passed.
- Transfer to another physical computer, binary compatibility with another UE installation, and an actual complete export have not been tested. Those are recipient acceptance checks, not claimed results.

## Speed and grip update � live acceptance after restart

Build180710903 passed Editor, Development and Shipping. Installed DLL SHA256 38C08000930FE5D87DC51657925147C333C3F0BF211CE9407DC64CF7DDA6A29C. Eight Python tests pass. Live configuration accepted max_speed_mps138.5824 and relative all-wheel grip1,.35,0,1.5,1; rejected139m/s without changing the previous limit. Evidence runs/environment_acceptance.json. These checks verify the native configuration/SetWheelFrictionMultiplier path, not calibrated tire-force measurements.

The user's speed_mps1.2 baseline completed10seconds,180samples,10.249m progress,mean tracking error1.581cm,max4.078cm,minmargin43.336cm; actual peak speed1.088m/s. Evidence runs/20260920_141350_baseline.json. Tire grip restored1.0. Persisted120Hz maximum substep settings verified after restart. No310mph physical run or full-lap performance claim. Startup applies user settings to the native car; later JSON changes require relaunch/apply, not recompile.
