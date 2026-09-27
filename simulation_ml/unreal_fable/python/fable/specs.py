"""
Loading scene specs and vehicle parameters, with defaults filled in.

The JSON schemas in specs/schema/ are the documentation; this module just makes
them convenient to use. Validation against the schema is opt-in (jsonschema is
not a hard dependency).
"""

from __future__ import annotations

import copy
import json
import os
from dataclasses import dataclass, field
from typing import Any

SPECS_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "specs"))


def _deep_get(d: dict, path: str, default=None):
    cur = d
    for k in path.split("."):
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


# ----------------------------------------------------------------- vehicles

CAR_DEFAULTS = {
    "common": {"mass_kg": 3.5, "length_m": 0.5, "width_m": 0.27, "height_m": 0.15, "com_offset_m": [0, 0, 0],
               "actuator_delay_s": 0.0, "control_rate_hz": 50},
    "car": {"wheelbase_m": 0.32, "track_width_m": 0.24, "wheel_radius_m": 0.05, "com_height_m": 0.05,
            "max_steer_rad": 0.4, "steer_rate_rad_s": 4.0, "steer_deadband": 0.0, "steer_bias": 0.0,
            "max_speed_mps": 5.0, "max_reverse_mps": 1.0, "accel_time_constant_s": 0.5,
            "max_accel_mps2": 4.0, "max_brake_mps2": 6.0, "drag_coeff": 0.0, "rolling_resistance": 0.0,
            "tire_friction": 1.0, "understeer_gain": 0.0, "drive": "rwd", "motor_torque_nm": 1.0, "gear_ratio": 1.0},
    "sensors": {"lidar": {"enabled": True, "n_rays": 540, "fov_rad": 4.712, "range_max_m": 12.0,
                          "noise_std_m": 0.02, "mount_m": [0.15, 0, 0.12], "rate_hz": 10},
                "imu": {"gyro_noise_std": 0.005, "accel_noise_std": 0.05, "gyro_bias": 0.0},
                "gps": {"enabled": False, "noise_std_m": 1.5, "rate_hz": 5},
                "odom": {"speed_noise_std": 0.02, "pose_noise_std_m": 0.0}},
}

BOAT_DEFAULTS = {
    "common": {"mass_kg": 15.0, "length_m": 1.2, "width_m": 0.6, "height_m": 0.35, "com_offset_m": [0, 0, 0],
               "actuator_delay_s": 0.0, "control_rate_hz": 20},
    "boat": {"beam_m": 0.6, "draft_m": 0.12, "thruster_offset_y_m": 0.25, "thruster_offset_x_m": -0.4,
             "thruster_depth_m": 0.1, "max_thrust_fwd_n": 20.0, "max_thrust_rev_n": 10.0,
             "thrust_time_constant_s": 0.25, "thrust_deadband": 0.05,
             "added_mass_surge_kg": 0.0, "added_mass_sway_kg": 0.0, "added_inertia_yaw_kgm2": 0.0,
             "drag_surge_lin": 2.0, "drag_surge_quad": 8.0, "drag_sway_lin": 15.0, "drag_sway_quad": 60.0,
             "drag_yaw_lin": 1.5, "drag_yaw_quad": 3.0, "wind_area_m2": 0.2, "wind_coeff": 1.0,
             "max_speed_mps": 2.0, "buoyancy_points": [[0.45, 0.22, -0.1], [0.45, -0.22, -0.1], [-0.45, 0.22, -0.1], [-0.45, -0.22, -0.1]]},
    "sensors": {"lidar": {"enabled": True, "n_rays": 360, "fov_rad": 6.283, "range_max_m": 30.0,
                          "noise_std_m": 0.03, "mount_m": [0.3, 0, 0.4], "rate_hz": 10},
                "imu": {"gyro_noise_std": 0.004, "accel_noise_std": 0.08, "gyro_bias": 0.0},
                "gps": {"enabled": True, "noise_std_m": 1.2, "rate_hz": 5},
                "odom": {"speed_noise_std": 0.05, "pose_noise_std_m": 0.0}},
}


def _merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


@dataclass
class VehicleParams:
    raw: dict[str, Any]

    @property
    def name(self) -> str:
        return self.raw.get("name", "vehicle")

    @property
    def type(self) -> str:
        return self.raw["type"]

    @property
    def is_car(self) -> bool:
        return self.type == "car_ackermann"

    @property
    def is_boat(self) -> bool:
        return self.type == "boat_twin_thruster"

    @property
    def common(self) -> dict:
        return self.raw["common"]

    @property
    def car(self) -> dict:
        return self.raw.get("car", {})

    @property
    def boat(self) -> dict:
        return self.raw.get("boat", {})

    @property
    def sensors(self) -> dict:
        return self.raw.get("sensors", {})

    @property
    def yaw_inertia(self) -> float:
        c = self.common
        if c.get("yaw_inertia_kgm2"):
            return float(c["yaw_inertia_kgm2"])
        return float(c["mass_kg"]) * (c["length_m"] ** 2 + c["width_m"] ** 2) / 12.0

    def get(self, path: str, default=None):
        return _deep_get(self.raw, path, default)

    def with_overrides(self, **flat) -> "VehicleParams":
        """`vp.with_overrides(**{"car.max_speed_mps": 3.0})`"""
        raw = copy.deepcopy(self.raw)
        for path, v in flat.items():
            cur = raw
            keys = path.split(".")
            for k in keys[:-1]:
                cur = cur.setdefault(k, {})
            cur[keys[-1]] = v
        return VehicleParams(raw)

    def to_wire(self) -> dict:
        return copy.deepcopy(self.raw)

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(self.raw, f, indent=2)


def load_vehicle(path_or_dict: str | dict) -> VehicleParams:
    raw = path_or_dict
    if isinstance(path_or_dict, str):
        p = path_or_dict if os.path.isabs(path_or_dict) or os.path.exists(path_or_dict) \
            else os.path.join(SPECS_DIR, path_or_dict)
        with open(p) as f:
            raw = json.load(f)
    if raw.get("params_version") != "1.0":
        raise ValueError("vehicle params_version must be '1.0'")
    defaults = CAR_DEFAULTS if raw["type"] == "car_ackermann" else BOAT_DEFAULTS
    return VehicleParams(_merge(defaults, raw))


# ------------------------------------------------------------------- scenes

@dataclass
class SceneSpec:
    raw: dict[str, Any]
    path: str = ""

    @property
    def name(self) -> str:
        return self.raw["name"]

    @property
    def scenario(self) -> str:
        return self.raw["scenario"]

    @property
    def size(self) -> tuple[float, float]:
        s = self.raw["world"]["size_m"]
        return float(s[0]), float(s[1])

    @property
    def spawn_xy(self) -> tuple[float, float]:
        s = self.raw["spawn"]["xy_m"]
        return float(s[0]), float(s[1])

    @property
    def spawn_yaw(self) -> float:
        return float(self.raw["spawn"].get("yaw_rad", 0.0))

    @property
    def track(self) -> dict:
        return self.raw.get("track") or {}

    @property
    def centerline(self) -> list[list[float]]:
        return self.track.get("centerline_m") or []

    @property
    def gates(self) -> list[dict]:
        return self.raw.get("gates") or []

    @property
    def dock(self) -> dict | None:
        return self.raw.get("dock")

    @property
    def objects(self) -> list[dict]:
        return self.raw.get("objects") or []

    @property
    def obstacle_fields(self) -> list[dict]:
        return self.raw.get("obstacle_fields") or []

    @property
    def task(self) -> dict:
        return self.raw.get("task") or {}

    @property
    def randomization(self) -> dict:
        return self.raw.get("randomization") or {}

    @property
    def disturbance(self) -> dict:
        return self.raw["world"].get("disturbance") or {}

    @property
    def water(self) -> dict:
        return self.raw["world"].get("water") or {}

    @property
    def terrain(self) -> dict:
        return self.raw["world"].get("terrain") or {}

    @property
    def geo_origin(self) -> dict | None:
        return self.raw["world"].get("geo_origin")

    def get(self, path: str, default=None):
        return _deep_get(self.raw, path, default)

    def vehicle(self) -> VehicleParams:
        vp = self.raw.get("vehicle_params")
        if vp:
            p = vp if os.path.isabs(vp) else os.path.join(os.path.dirname(self.path) if self.path else SPECS_DIR, vp)
            if not os.path.exists(p):
                p = os.path.join(SPECS_DIR, vp)
            return load_vehicle(p)
        return load_vehicle({"params_version": "1.0", "name": "default",
                             "type": "car_ackermann" if self.scenario == "car" else "boat_twin_thruster"})

    def save(self, path: str | None = None) -> str:
        path = path or self.path
        with open(path, "w") as f:
            json.dump(self.raw, f, indent=2)
        return path


def load_scene(path_or_dict: str | dict, validate: bool = False) -> SceneSpec:
    raw, path = path_or_dict, ""
    if isinstance(path_or_dict, str):
        path = path_or_dict if os.path.exists(path_or_dict) else os.path.join(SPECS_DIR, "worlds", path_or_dict)
        if not path.endswith(".json") and not os.path.exists(path):
            path += ".json"
        with open(path) as f:
            raw = json.load(f)
    if raw.get("spec_version") != "2.0":
        raise ValueError("scene spec_version must be '2.0' (run specs/make_worlds.py for examples)")
    if validate:
        import jsonschema
        with open(os.path.join(SPECS_DIR, "schema", "scene_spec.schema.json")) as f:
            jsonschema.validate(raw, json.load(f))
    return SceneSpec(raw, path)
