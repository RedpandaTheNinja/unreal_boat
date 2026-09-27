# Calibration: making the sim behave like *your* car and boat

The point of the whole kit: a control algorithm tuned in sim should work on the real
vehicle with the same gains. That only happens if the sim's numbers are the vehicle's
numbers. This document is the procedure. Budget one afternoon per vehicle.

There are three tiers of parameter. Do them in order.

## Tier 1 — measure with a tape and a scale (10 minutes)

Open `specs/vehicles/car_rc_default.json` / `boat_twin_thruster_default.json`, save a copy
as `*_measured.json`, and fill in:

| Car | Boat | How |
|---|---|---|
| `mass_kg` | `mass_kg` | bathroom scale, battery installed |
| `length_m`, `width_m`, `height_m` | same + `beam_m`, `draft_m` | tape; draft = waterline depth when floating loaded |
| `wheelbase_m`, `track_width_m`, `wheel_radius_m` | `thruster_offset_y_m` (half the thruster spacing), `thruster_offset_x_m` (astern of CoM is negative) | tape |
| `com_height_m` | `com_offset_m` | balance it on a rod (car) / tilt test (boat); rough is fine |
| `control_rate_hz` | `control_rate_hz` | whatever your stack runs at |

`yaw_inertia_kgm2`: leave blank — derived from the box, then fitted in Tier 3.

## Tier 2 — bench measurements (30 minutes)

These are the ones that are nearly impossible to fit from driving data because they
look like each other (a pure delay and a first-order lag over one step are almost
identical curves). Measure them directly.

**Actuator delay `actuator_delay_s`.** Log the command timestamp and the first moment
the wheel/prop moves (phone video at 240 fps against a clock on screen works). Typical:
RC servo + ESC 20–60 ms; a ROS stack over Wi-Fi adds 20–100 ms more. Put in the total
from *your controller's* command to motion.

**Thrust lag `thrust_time_constant_s` (boat).** Thruster in a bucket, step 0 → 100 %,
video the prop or log ESC current: time to 63 % of final. Typical 0.15–0.4 s.

**Static thrust `max_thrust_fwd_n`, `max_thrust_rev_n` (boat).** Tie the boat to a
luggage scale on a dock line, full forward, read newtons (kg × 9.81). Do reverse. Reverse
is usually 40–60 % of forward for a normal prop.

**Steering `max_steer_rad`, `steer_bias` (car).** Full lock, measure the front wheel angle
with a phone inclinometer app or protractor. Bias: the command that makes it roll dead
straight on a smooth floor; usually ±0.03.

**Deadbands.** Slowly increase command from 0 until the wheel/prop turns: `steer_deadband`,
`thrust_deadband` as a fraction of full command.

## Tier 3 — fit from a drive (1 hour, most of it waiting)

Everything else — top speed, acceleration response, braking, understeer, drag — comes
from one open-loop manoeuvre that excites all of it. The **same** manoeuvre runs in the
sim (`examples/sysid_run.py`) and on the vehicle; the fitter does not care which log it gets.

### 3a. Log format

CSV, one row per control step, columns exactly:

```
t, x, y, yaw, vx, vy, wz, ax, ay, speed, u_throttle, u_steer, u_brake, u_thrust_l, u_thrust_r, collision
```

SI + ENU + body-frame (ROS REP-103). `vx` is body forward speed. Missing columns are read
as zero, so a car log needs no thrust columns. `fable.sim.EpisodeLog` writes this format;
for the real vehicle, a 20-line ROS 2 node that subscribes to `/odom`, `/imu` and your command
topic and writes these rows is all that is needed (`fable/ros2_adapter.py` shows the message
plumbing). Log at the control rate.

### 3b. The manoeuvre

`fable/calibration/maneuvers.py` defines it as a function of time; drive it open-loop.

**Car (44 s, needs ~30 m of clear floor):**
throttle 0.3 for 2 s → 0.6 for 3 s → 1.0 for 3 s → full brake 3 s → slow slalom (0.35
throttle, ±0.8 steer square wave, 4 s period) for 14 s → fast slalom (0.8 throttle, ±0.5
sine, 6 s period) for 15 s → coast 4 s.

**Boat (118 s, calm water — wind is the one thing the fit cannot subtract):**
both thrusters 100 % for 25 s → both 0 and coast 35 s → zig-zag at 0.6 cruise ±0.5
differential, 12 s period, for 40 s → spin (+0.8 / −0.8) 15 s.

Safety: have a kill switch, a spotter, and space. The car reaches its real top speed.

### 3c. Fit

```
python -m fable.calibration.fit_car  logs/car_real.csv  specs/vehicles/car_rc_measured.json  -o specs/vehicles/car_rc_measured.json
python -m fable.calibration.fit_boat logs/boat_real.csv specs/vehicles/boat_measured.json    -o specs/vehicles/boat_measured.json
```

It prints the fitted values and an `rms_error` block and writes them into the file under
`calibration` so you know where every number came from and when.

What good looks like: car speed RMS < 0.15 m/s and yaw-rate RMS < 0.1 rad/s; boat surge
RMS < 0.05 m/s and yaw-rate RMS < 0.05 rad/s. The kit's own test (`tests/test_calibration.py`)
recovers deliberately-perturbed parameters from a simulated log to within 10–25 % — that is
the noise floor of the method, so do not chase the third decimal.

If the fit is bad:

- **Speed fits, yaw does not (car)** — steering is not linear in command (servo horn
  geometry). Fit is linear; either linearise in your driver or accept the error near lock.
- **Yaw fits, speed does not (boat)** — you were not on calm water, or the log's `vx` is
  GPS ground speed, not water speed. Current shows up as an offset.
- **Everything is off by a constant factor** — units. Check the log is in m/s and rad/s,
  not km/h and deg/s. It is always units.

### 3d. Check it closed-loop

Run the baseline controller in sim with the fitted file and on the vehicle with the same
gains on the same course (a taped oval / a two-buoy line):

```
python examples/run_scenario.py car_track_oval --vehicle specs/vehicles/car_rc_measured.json
python -m fable.ros2_adapter --scene specs/worlds/car_track_oval.json --vehicle specs/vehicles/car_rc_measured.json
```

Compare `cross_track_rms_m` and lap time. Within 20 % and you are done. The boat controller
derives its heading gains from the parameters (`heading_gains_from_params`), so a correct
fit makes it behave the same on the water without retuning — that is the payoff.

## Tier 4 — the things the model does not have

Deliberate omissions, so you know what a residual means:

- Car: no suspension, no load transfer, no tyre slip curves — the friction circle is a
  hard clip. Fine below ~0.6 g lateral. Chaos in Unreal has these; the mock does not.
- Boat: added mass is in the mock, not in Unreal. Wind area is a single lateral number.
  No wave drift force. Fine for < 0.3 m chop.
- Both: sensors are white noise plus a fixed rate. Real lidar has range-dependent noise
  and reflectivity dropouts; real GPS has multipath. Add them when they bite.

## Domain randomization ranges (what to put in `spec.randomization.ranges`)

Once fitted, randomize ±20 % around the fitted value for `mass`, `friction`,
`actuator_delay`, and full range for wind/current. A controller that survives that band
survives the battery going flat and the grass being wet.
