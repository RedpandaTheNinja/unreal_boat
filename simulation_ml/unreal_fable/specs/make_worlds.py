"""
make_worlds.py - regenerates the example world specs in specs/worlds/.

Each world is built from a few numbers so you can change a radius here rather
than editing 200 coordinates by hand. Claude edits worlds the same way: it
changes the description, re-runs the generator or the MCP tool, and the level
follows. Run:  python specs/make_worlds.py
"""
from __future__ import annotations

import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "worlds")


def oval(a: float, b: float, n: int = 120) -> list[list[float]]:
    """Stadium: two straights of length 2a, two semicircles of radius b."""
    pts = []
    # bottom straight (left -> right)
    for i in range(n // 4):
        t = i / (n // 4)
        pts.append([-a + 2 * a * t, -b])
    # right semicircle
    for i in range(n // 4):
        th = -math.pi / 2 + math.pi * i / (n // 4)
        pts.append([a + b * math.cos(th), b * math.sin(th)])
    # top straight (right -> left)
    for i in range(n // 4):
        t = i / (n // 4)
        pts.append([a - 2 * a * t, b])
    # left semicircle
    for i in range(n // 4):
        th = math.pi / 2 + math.pi * i / (n // 4)
        pts.append([-a + b * math.cos(th), b * math.sin(th)])
    return [[round(x, 3), round(y, 3)] for x, y in pts]


def slalom(length: float = 40.0, amp: float = 2.5, n_gates: int = 8) -> dict:
    spacing = length / (n_gates + 1)
    cones = []
    wps = [[-length / 2, 0.0]]
    for k in range(n_gates):
        x = -length / 2 + spacing * (k + 1)
        cones.append({"label": f"slalom_{k}", "kind": "cone", "xy_m": [round(x, 2), 0.0], "tag": "slalom"})
        wps.append([round(x, 2), round(amp * (1 if k % 2 == 0 else -1), 2)])
    wps.append([length / 2, 0.0])
    return {"cones": cones, "waypoints": wps}


def buoy_course(n_gates: int = 6, spacing: float = 12.0, width: float = 4.0,
                wiggle: float = 5.0) -> list[dict]:
    """Gates along +x with a sinusoidal lateral offset - forces real turning."""
    gates = []
    for k in range(n_gates):
        cx = 10.0 + spacing * k
        cy = wiggle * math.sin(k * math.pi / 2.5)
        heading = math.atan2(wiggle * (math.pi / 2.5) * math.cos(k * math.pi / 2.5), spacing)
        nx, ny = -math.sin(heading), math.cos(heading)      # left normal
        gates.append({
            "label": f"gate{k + 1}",
            "left_m": [round(cx + nx * width / 2, 2), round(cy + ny * width / 2, 2)],
            "right_m": [round(cx - nx * width / 2, 2), round(cy - ny * width / 2, 2)],
        })
    return gates


def figure8(a: float = 14.0, b: float = 6.0, n: int = 42) -> list[list[float]]:
    pts = []
    for i in range(n):
        t = 2 * math.pi * i / n
        pts.append([round(a * math.sin(t), 3), round(b * math.sin(t) * math.cos(t), 3)])
    pts.append(pts[0][:])
    return pts


def channel_course(length_m: float = 140.0, n_gates: int = 11, width_m: float = 4.0) -> tuple[list[list[float]], list[dict]]:
    pts = []
    for i in range(n_gates + 1):
        x = round(length_m * i / n_gates, 3)
        y = round(6.0 * math.sin(2 * math.pi * i / n_gates * 1.5), 3)
        pts.append([x, y])

    gates = []
    half = width_m / 2
    for i in range(1, n_gates):
        x = pts[i][0]
        y = pts[i][1]
        dy = pts[i + 1][1] - pts[i - 1][1]
        dx = pts[i + 1][0] - pts[i - 1][0]
        h = math.atan2(dy, dx)
        nx, ny = -math.sin(h), math.cos(h)
        gates.append({
            "label": f"gate{i}",
            "left_m": [round(x + nx * half, 3), round(y + ny * half, 3)],
            "right_m": [round(x - nx * half, 3), round(y - ny * half, 3)],
        })

    return pts, gates


def write(name: str, spec: dict) -> None:
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name + ".json")
    with open(path, "w") as f:
        json.dump(spec, f, indent=2)
    print("wrote", path)


def main() -> None:
    # ------------------------------------------------------------- car: oval
    cl = oval(a=12.0, b=7.0)
    write("car_track_oval", {
        "spec_version": "2.0", "name": "car_track_oval", "scenario": "car", "seed": 1,
        "notes": "Flat asphalt stadium oval, 24 m straights, 7 m radius turns, 2.2 m wide, coned. Baseline for pure-pursuit tuning: no obstacles, no randomization.",
        "vehicle_params": "vehicles/car_rc_default.json",
        "world": {
            "size_m": [60, 40],
            "geo_origin": {"lat": 39.7684, "lon": -86.1581, "alt": 220},
            "terrain": {"mode": "flat", "surface": {"material_preset": "asphalt", "friction": 0.9, "restitution": 0.1}},
            "lighting": {"preset": "noon"}
        },
        "spawn": {"xy_m": cl[0], "yaw_rad": 0.0, "jitter_m": 0.2, "jitter_yaw_rad": 0.05},
        "track": {"centerline_m": cl, "closed": True, "width_m": 2.2, "border": "cones", "border_spacing_m": 2.0},
        "objects": [],
        "obstacle_fields": [],
        "task": {"type": "lap", "laps": 2, "tolerance_m": 1.0, "max_time_s": 120,
                 "fail_on": ["collision", "off_track", "timeout"],
                 "metrics": ["cross_track_rms", "time", "collisions", "control_effort"]},
        "randomization": {"enabled": False}
    })

    # ----------------------------------------------------------- car: slalom + gravel
    s = slalom()
    write("car_track_slalom_gravel", {
        "spec_version": "2.0", "name": "car_track_slalom_gravel", "scenario": "car", "seed": 3,
        "notes": "40 m slalom through 8 cones at 4.4 m spacing on rough gravel with a few barrels off-line. Tests steering authority, actuator delay and traction. Friction and layout randomize per episode.",
        "vehicle_params": "vehicles/car_rc_default.json",
        "world": {
            "size_m": [50, 20],
            "terrain": {"mode": "procedural",
                        "procedural": {"roughness_m": 0.04, "feature_size_m": 6, "slope_max_deg": 6},
                        "surface": {"material_preset": "gravel", "friction": 0.7, "restitution": 0.05}},
            "lighting": {"preset": "overcast"}
        },
        "spawn": {"xy_m": [-22.0, 0.0], "yaw_rad": 0.0, "jitter_m": 0.3, "jitter_yaw_rad": 0.1},
        "track": {"centerline_m": s["waypoints"], "closed": False, "width_m": 6.0, "border": "none"},
        "objects": s["cones"],
        "obstacle_fields": [{"kind": "barrel", "count": 6, "area_m": [-18, -8, 18, 8], "min_separation_m": 2.5, "avoid_track_m": 3.5}],
        "task": {"type": "waypoints", "waypoints_m": s["waypoints"], "tolerance_m": 0.8, "max_time_s": 60,
                 "fail_on": ["collision", "timeout"],
                 "metrics": ["cross_track_rms", "time", "collisions", "min_clearance_m"]},
        "randomization": {"enabled": True, "vary": ["friction", "obstacle_layout", "spawn_pose", "actuator_delay"],
                          "ranges": {"friction": [0.5, 0.9], "actuator_delay_s": [0.02, 0.1]}}
    })

    # ---------------------------------------------------------- boat: buoy course
    gates = buoy_course()
    wps = [[round((g["left_m"][0] + g["right_m"][0]) / 2, 2), round((g["left_m"][1] + g["right_m"][1]) / 2, 2)] for g in gates]
    # route continues 8 m past the last gate so the boat drives THROUGH it rather than stopping on the line
    gl, gr = gates[-1]["left_m"], gates[-1]["right_m"]
    hx, hy = -(gr[1] - gl[1]), (gr[0] - gl[0])       # rotate left->right by +90 deg = through the gate
    hn = math.hypot(hx, hy)
    finish = [round(wps[-1][0] + 8 * hx / hn, 2), round(wps[-1][1] + 8 * hy / hn, 2)]
    write("boat_buoy_course", {
        "spec_version": "2.0", "name": "boat_buoy_course", "scenario": "boat", "seed": 5,
        "notes": "Open water, six red/green buoy gates 12 m apart with a lateral wiggle, 4 m wide. 3 m/s wind from the west with gusts, 0.3 m/s current to the north-east, small chop. Tests LOS guidance under disturbance.",
        "vehicle_params": "vehicles/boat_twin_thruster_default.json",
        "world": {
            "size_m": [160, 80],
            "geo_origin": {"lat": 39.7684, "lon": -86.1581, "alt": 0},
            "water": {"level_m": 0.0, "waves": {"amp_m": 0.08, "period_s": 2.5, "dir_rad": 0.4},
                      "shore": [{"label": "island", "polygon_m": [[70, 20], [82, 24], [84, 32], [72, 34], [66, 27]], "height_m": 1.5}]},
            "disturbance": {"wind": {"speed_mps": 3.0, "dir_rad": 0.0, "gust_std_mps": 1.0, "gust_period_s": 8},
                            "current": {"speed_mps": 0.3, "dir_rad": 0.785}},
            "lighting": {"preset": "noon"}
        },
        "spawn": {"xy_m": [0.0, 0.0], "yaw_rad": 0.0, "jitter_m": 0.5, "jitter_yaw_rad": 0.2},
        "track": {"centerline_m": [[0.0, 0.0]] + wps + [finish], "closed": False, "width_m": 4.0, "border": "none"},
        "gates": gates,
        "objects": [{"label": "moored_1", "kind": "moored_boat", "xy_m": [40.0, -12.0], "yaw_rad": 1.2, "tag": "obstacle"}],
        "obstacle_fields": [{"kind": "buoy_yellow", "count": 8, "area_m": [5, -25, 90, 25], "min_separation_m": 6.0, "avoid_track_m": 4.0}],
        "task": {"type": "gates", "waypoints_m": wps, "tolerance_m": 2.0, "max_time_s": 240,
                 "fail_on": ["collision", "out_of_bounds", "timeout"],
                 "metrics": ["gates_passed", "time", "cross_track_rms", "collisions", "control_effort"]},
        "randomization": {"enabled": True, "vary": ["wind", "current", "waves", "obstacle_layout", "spawn_pose"],
                          "ranges": {"wind_speed_mps": [0, 6], "wind_dir_rad": [-3.14, 3.14], "current_speed_mps": [0, 0.6], "current_dir_rad": [-3.14, 3.14], "wave_amp_m": [0, 0.2]}}
    })

    # --------------------------------------------------------------- boat: docking
    write("boat_docking", {
        "spec_version": "2.0", "name": "boat_docking", "scenario": "boat", "seed": 9,
        "notes": "Approach from 30 m out and stop inside a 1.6 m wide slip on an 8 m pier in a 1.5 m/s cross-wind. Scores final position/heading error and counts fender contacts (a touch is a contact, not a failure). Two moored boats crowd the approach.",
        "vehicle_params": "vehicles/boat_twin_thruster_default.json",
        "world": {
            "size_m": [80, 60],
            "water": {"level_m": 0.0, "waves": {"amp_m": 0.03, "period_s": 2.0, "dir_rad": 1.57}},
            "disturbance": {"wind": {"speed_mps": 1.5, "dir_rad": 1.571, "gust_std_mps": 0.5, "gust_period_s": 6},
                            "current": {"speed_mps": 0.05, "dir_rad": 1.571}},
            "lighting": {"preset": "dusk"}
        },
        "spawn": {"xy_m": [-30.0, -4.0], "yaw_rad": 0.1, "jitter_m": 1.0, "jitter_yaw_rad": 0.3},
        "dock": {"xy_m": [0.0, 0.0], "yaw_rad": 0.0, "slip_width_m": 1.6, "slip_length_m": 3.0, "pier_length_m": 8.0},
        "objects": [{"label": "moored_A", "kind": "moored_boat", "xy_m": [-6.0, 4.5], "yaw_rad": 0.0, "tag": "obstacle"},
                    {"label": "moored_B", "kind": "moored_boat", "xy_m": [-12.0, -6.0], "yaw_rad": 0.3, "tag": "obstacle"}],
        "obstacle_fields": [],
        "task": {"type": "dock", "goal_m": [0.0, 0.0], "tolerance_m": 0.35, "max_time_s": 180,
                 "fail_on": ["timeout"],
                 "metrics": ["dock_error_m", "heading_error_rms", "time", "collisions", "control_effort"]},
        "randomization": {"enabled": True, "vary": ["wind", "spawn_pose"],
                          "ranges": {"wind_speed_mps": [0, 2.5], "wind_dir_rad": [0.8, 2.3]}}
    })

    # --------------------------------------------------------- car: figure-8 digital twin
    f8 = figure8()
    write("car_figure8_digital_twin", {
        "spec_version": "2.0", "name": "car_figure8_digital_twin", "scenario": "car", "seed": 17,
        "notes": "Figure-8 urban test track for Ackermann path-following and obstacle-avoidance tuning. Includes gated lap task with repeatable geometry and light randomization for algorithm robustness.",
        "vehicle_params": "vehicles/car_orangecube_digital_twin.json",
        "world": {
            "size_m": [36, 24],
            "geo_origin": {"lat": 39.7684, "lon": -86.1581, "alt": 220},
            "terrain": {"mode": "flat", "surface": {"material_preset": "asphalt", "friction": 0.95, "restitution": 0.08}},
            "lighting": {"preset": "noon"}
        },
        "spawn": {"xy_m": [0.0, 0.0], "yaw_rad": 0.0, "jitter_m": 0.15, "jitter_yaw_rad": 0.08},
        "track": {"centerline_m": f8, "closed": True, "width_m": 2.4, "border": "cones", "border_spacing_m": 2.0},
        "objects": [
            {"label": "left_crossing_marker", "kind": "cone", "xy_m": [0.2, 1.1], "scale": [0.9, 0.9, 1.0], "tag": "marker"},
            {"label": "right_crossing_marker", "kind": "cone", "xy_m": [0.2, -1.1], "scale": [0.9, 0.9, 1.0], "tag": "marker"}
        ],
        "obstacle_fields": [{"kind": "barrel", "count": 12, "area_m": [-16.0, -10.0, 16.0, 10.0], "min_separation_m": 2.2,
                            "avoid_track_m": 3.2, "scale_jitter": 0.2}],
        "task": {"type": "lap", "laps": 2, "tolerance_m": 0.9, "max_time_s": 180, "fail_on": ["collision", "off_track", "timeout"],
                 "metrics": ["cross_track_rms", "time", "collisions", "control_effort"]},
        "randomization": {"enabled": True, "vary": ["friction", "obstacle_layout", "spawn_pose", "actuator_delay"],
                          "ranges": {"friction": [0.80, 0.98], "actuator_delay_s": [0.03, 0.08]}}
    })

    # ---------------------------------------------- boat: channel gates digital twin
    wps, gates = channel_course()
    channel_waypoints = wps[::2]
    if channel_waypoints[-1] != wps[-1]:
        channel_waypoints.append(wps[-1])
    write("boat_channel_gates_digital_twin", {
        "spec_version": "2.0", "name": "boat_channel_gates_digital_twin", "scenario": "boat", "seed": 19,
        "notes": "Sinuous channel scenario for twin-thruster boats with buoy gates and floating obstacles. Designed for LOS/path-following and obstacle-aware planning in mixed disturbance conditions.",
        "vehicle_params": "vehicles/boat_twin_thruster_digital_twin.json",
        "world": {
            "size_m": [170, 100],
            "geo_origin": {"lat": 39.7684, "lon": -86.1581, "alt": 0},
            "water": {"level_m": 0.0, "waves": {"amp_m": 0.06, "period_s": 2.8, "dir_rad": 0.2},
                      "shore": [{"label": "island", "polygon_m": [[60, 30], [72, 34], [78, 42], [72, 52], [58, 47]], "height_m": 1.4}]},
            "disturbance": {"wind": {"speed_mps": 1.5, "dir_rad": 0.0, "gust_std_mps": 0.5, "gust_period_s": 7},
                             "current": {"speed_mps": 0.2, "dir_rad": 0.95}},
            "lighting": {"preset": "overcast"}
        },
        "spawn": {"xy_m": [0.0, 0.0], "yaw_rad": 0.0, "jitter_m": 0.3, "jitter_yaw_rad": 0.15},
        "track": {"centerline_m": wps, "closed": False, "width_m": 3.5, "border": "none"},
        "gates": gates,
        "objects": [
            {"label": "shore_marker_1", "kind": "moored_boat", "xy_m": [34.0, 14.0], "yaw_rad": 1.05, "tag": "obstacle"},
            {"label": "shore_marker_2", "kind": "moored_boat", "xy_m": [85.0, -14.0], "yaw_rad": -1.05, "tag": "obstacle"},
            {"label": "floating_rock_1", "kind": "rock", "xy_m": [101.0, 2.2], "tag": "obstacle"}
        ],
        "obstacle_fields": [{"kind": "buoy_yellow", "count": 10, "area_m": [8, -14, 132, 14], "min_separation_m": 5.0, "avoid_track_m": 3.5}],
        "task": {"type": "gates", "waypoints_m": channel_waypoints,
                 "tolerance_m": 2.0, "max_time_s": 300,
                 "fail_on": ["collision", "out_of_bounds", "timeout"],
                 "metrics": ["gates_passed", "time", "cross_track_rms", "collisions", "control_effort"]},
        "randomization": {"enabled": True,
                          "vary": ["wind", "current", "waves", "obstacle_layout", "spawn_pose"],
                          "ranges": {"wind_speed_mps": [0, 3.5], "wind_dir_rad": [-3.14, 3.14],
                                     "current_speed_mps": [0, 0.5], "current_dir_rad": [-3.14, 3.14], "wave_amp_m": [0, 0.15]}}
    })


if __name__ == "__main__":
    main()
