# Fable Bridge Protocol v1

The contract between Python (`fable/bridge.py`) and Unreal (`FableBridgeComponent.cpp`).
Both sides implement exactly this; the mock simulator (`fable/mock_sim.py`) speaks the
same observation format so a controller cannot tell which one it is talking to.

## Transport

UDP, one JSON object per datagram, UTF-8, no framing.

| Direction | Endpoint | Notes |
|---|---|---|
| Python → Unreal | `127.0.0.1:9800` (configurable on the bridge actor) | Unreal binds this |
| Unreal → Python | whatever address sent the last `hello` | Python needs no config |

Datagrams over ~60 KB are rejected by the OS; the 2-D lidar at 720 rays is ~7 KB.
Cameras and 3-D lidar are **not** in v1 — they go over a separate TCP/shared-memory
channel when added, not through this socket.

## Conventions — read this twice

Everything on the wire is **SI + ROS REP-103**, so the same Python runs against the
real vehicle:

- metres, seconds, radians, newtons, kilograms
- world frame: **ENU** — x east, y north, z up; yaw = 0 facing +x, positive counter-clockwise
- body frame: x forward, **y left**, z up
- Unreal is centimetres, degrees, left-handed with y *right*. **The C++ side converts.
  Python never sees Unreal units.**

Conversion used in C++ (`FableConv.h`):

```
ros.x =  ue.X / 100
ros.y = -ue.Y / 100
ros.z =  ue.Z / 100
ros.yaw = -radians(ue.Yaw)
ros.roll =  radians(ue.Roll)
ros.pitch = -radians(ue.Pitch)
```

## Timing

Unreal runs with **Use Fixed Frame Rate** enabled (default 50 Hz → `dt = 0.02 s`).
Every tick the bridge sends one `obs`. Python's `step(u)` sends an `act` then blocks
until an `obs` arrives with `tick > last_tick`. If Python is late, Unreal keeps
ticking with the **last action held** (zero-order hold) — the same thing a real
vehicle does when your controller stalls. `obs.seq` tells you which action was
actually in effect.

Faster-than-real-time: run Unreal with `-nullrhi` (no rendering) and it advances
`dt` per frame as fast as the CPU allows. `set_env.time_scale` applies world time
dilation for the rendered case.

## Messages: Python → Unreal

```jsonc
{"type": "hello", "client": "fable-py", "version": 1, "role": "controller"}
// role "controller" (default): receives the observation stream and drives the vehicle.
// role "editor": may spawn/despawn/set_env/query and gets acks, but never takes over
// the observation stream - this is how an MCP world-editor coexists with a running test.

{"type": "act", "seq": 1234, "u": {"throttle": 0.4, "steer": -0.15, "brake": 0.0}}   // car
{"type": "act", "seq": 1234, "u": {"thrust_l": 0.6, "thrust_r": 0.3}}                // boat
// all u values are normalized [-1, 1] (brake [0, 1]); the pawn maps them through vehicle_params

{"type": "reset",
 "pose": {"x": 0, "y": 0, "yaw": 0},          // optional; default = spec spawn
 "seed": 42,                                   // optional; re-rolls randomization
 "clear_dynamic": true}                        // remove runtime-spawned actors

{"type": "set_params", "params": { ...vehicle_params.json contents... }}
// hot-swap physical parameters; applied at the next reset for Chaos vehicles,
// immediately for the boat

{"type": "set_env",
 "wind":    {"speed": 4.0, "dir": 1.57, "gust_std": 1.0},   // m/s, rad (direction it blows TOWARD)
 "current": {"speed": 0.4, "dir": 0.0},
 "waves":   {"amp": 0.15, "period": 3.0, "dir": 0.5},
 "time_scale": 1.0}

{"type": "spawn", "kind": "buoy_red", "label": "gate3_L",
 "x": 12.0, "y": 4.0, "z": 0.0, "yaw": 0.0, "scale": [1, 1, 1], "movable": false}
// kinds: cone barrel wall box pole rock buoy_red buoy_green buoy_yellow dock_piece custom:<AssetPath>

{"type": "despawn", "label": "gate3_L"}     // label "dynamic:*" removes all runtime spawns

{"type": "query", "what": "scene"}          // → scene message
{"type": "bye"}
```

## Messages: Unreal → Python

```jsonc
{"type": "obs",
 "tick": 51234,               // monotonically increasing frame counter
 "t": 1024.68,                // sim time, s
 "dt": 0.02,
 "seq": 1233,                 // seq of the act currently applied (ZOH)
 "vehicle": "car",            // "car" | "boat"
 "pose":  {"x": 12.3, "y": -4.1, "z": 0.21, "roll": 0.01, "pitch": -0.02, "yaw": 1.31},
 "vel":   {"vx": 2.1, "vy": -0.05, "vz": 0.0},     // BODY frame (surge, sway, heave)
 "omega": {"wx": 0.0, "wy": 0.0, "wz": 0.42},      // BODY frame rad/s
 "acc":   {"ax": 0.3, "ay": -0.9, "az": 9.8},      // BODY frame incl. gravity (IMU-like)
 "u_applied": {"throttle": 0.4, "steer": -0.15},   // after rate limits / delay
 "sensors": {
   "lidar": {"angle_min": -2.356, "angle_max": 2.356, "n": 540, "range_max": 20.0,
             "ranges": [ ... ]},                    // metres, range_max if no hit
   "gps":   {"lat": 39.7684, "lon": -86.1581, "hdop": 1.2},     // from spec.world.geo_origin
   "speed": 2.1,                                    // ground speed, m/s
   "wheel": {"steer_angle": -0.11, "rpm": [812, 812, 790, 790]},        // car only
   "thrust": {"l": 14.2, "r": 7.1}                                      // boat only, N
 },
 "events": {"collision": false, "contacts": [], "out_of_bounds": false, "capsized": false},
 "env": {"wind": {"speed": 4.3, "dir": 1.55}, "current": {"speed": 0.4, "dir": 0.0}}
}

{"type": "ack", "for": "reset", "ok": true, "tick": 51235}
{"type": "ack", "for": "spawn", "ok": false, "error": "unknown kind 'tree'"}

{"type": "scene", "spec_name": "boat_buoy_course_v1",
 "actors": [{"label": "gate1_L", "kind": "buoy_red", "x": 10, "y": 3, "z": 0, "yaw": 0, "tags": ["gate"]}, ...],
 "task": { ...spec.task... }}
```

## Versioning

`hello.version` is checked by the bridge. A mismatch answers
`{"type":"ack","for":"hello","ok":false,"error":"version"}` and ignores everything
else from that client. Additive fields are fine without a bump; changing units,
frames or the meaning of an existing field is a bump.
