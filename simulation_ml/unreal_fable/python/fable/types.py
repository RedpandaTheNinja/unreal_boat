"""
Typed views over the wire protocol (docs/PROTOCOL.md).

Controllers take an `Observation` and return a plain dict of actuator commands in
[-1, 1]. Keeping the action a dict rather than a class means the same controller
output goes straight into the bridge, the mock sim, and the ROS 2 adapter.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    @property
    def xy(self) -> np.ndarray:
        return np.array([self.x, self.y])

    def dist_to(self, xy) -> float:
        return float(math.hypot(self.x - xy[0], self.y - xy[1]))

    def bearing_to(self, xy) -> float:
        """Angle from heading to target, wrapped to [-pi, pi]."""
        return wrap(math.atan2(xy[1] - self.y, xy[0] - self.x) - self.yaw)


@dataclass
class Twist:
    vx: float = 0.0    # surge (body forward)
    vy: float = 0.0    # sway  (body left)
    vz: float = 0.0
    wx: float = 0.0
    wy: float = 0.0
    wz: float = 0.0    # yaw rate

    @property
    def speed(self) -> float:
        return float(math.hypot(self.vx, self.vy))


@dataclass
class Lidar:
    angle_min: float
    angle_max: float
    range_max: float
    ranges: np.ndarray

    @property
    def angles(self) -> np.ndarray:
        return np.linspace(self.angle_min, self.angle_max, len(self.ranges))

    def min_in_sector(self, center: float, half_width: float) -> float:
        a = self.angles
        m = np.abs(wrap_array(a - center)) <= half_width
        return float(self.ranges[m].min()) if m.any() else self.range_max

    def points_body(self) -> np.ndarray:
        """(N, 2) hit points in the body frame, excluding max-range returns."""
        a = self.angles
        ok = self.ranges < self.range_max * 0.999
        return np.stack([self.ranges[ok] * np.cos(a[ok]), self.ranges[ok] * np.sin(a[ok])], axis=1)


@dataclass
class Observation:
    tick: int = 0
    t: float = 0.0
    dt: float = 0.02
    seq: int = -1
    vehicle: str = "car"
    pose: Pose = field(default_factory=Pose)
    vel: Twist = field(default_factory=Twist)
    acc: tuple[float, float, float] = (0.0, 0.0, 0.0)
    u_applied: dict[str, float] = field(default_factory=dict)
    lidar: Lidar | None = None
    gps: dict[str, float] | None = None
    speed: float = 0.0
    extra: dict[str, Any] = field(default_factory=dict)
    collision: bool = False
    contacts: list[str] = field(default_factory=list)
    out_of_bounds: bool = False
    capsized: bool = False
    env: dict[str, Any] = field(default_factory=dict)

    # ------------------------------------------------------------ wire format
    @classmethod
    def from_wire(cls, m: dict[str, Any]) -> "Observation":
        p, v, w = m.get("pose", {}), m.get("vel", {}), m.get("omega", {})
        s = m.get("sensors", {})
        lid = s.get("lidar")
        ev = m.get("events", {})
        a = m.get("acc", {})
        return cls(
            tick=int(m.get("tick", 0)), t=float(m.get("t", 0.0)), dt=float(m.get("dt", 0.02)),
            seq=int(m.get("seq", -1)), vehicle=str(m.get("vehicle", "car")),
            pose=Pose(**{k: float(p.get(k, 0.0)) for k in ("x", "y", "z", "roll", "pitch", "yaw")}),
            vel=Twist(vx=float(v.get("vx", 0)), vy=float(v.get("vy", 0)), vz=float(v.get("vz", 0)),
                      wx=float(w.get("wx", 0)), wy=float(w.get("wy", 0)), wz=float(w.get("wz", 0))),
            acc=(float(a.get("ax", 0)), float(a.get("ay", 0)), float(a.get("az", 0))),
            u_applied=dict(m.get("u_applied", {})),
            lidar=Lidar(float(lid["angle_min"]), float(lid["angle_max"]), float(lid["range_max"]),
                        np.asarray(lid["ranges"], dtype=np.float32)) if lid else None,
            gps=s.get("gps"), speed=float(s.get("speed", 0.0)),
            extra={k: s[k] for k in s if k not in ("lidar", "gps", "speed")},
            collision=bool(ev.get("collision", False)), contacts=list(ev.get("contacts", [])),
            out_of_bounds=bool(ev.get("out_of_bounds", False)), capsized=bool(ev.get("capsized", False)),
            env=dict(m.get("env", {})),
        )

    def to_wire(self) -> dict[str, Any]:
        s: dict[str, Any] = {"speed": self.speed, **self.extra}
        if self.lidar is not None:
            s["lidar"] = {"angle_min": self.lidar.angle_min, "angle_max": self.lidar.angle_max,
                          "n": len(self.lidar.ranges), "range_max": self.lidar.range_max,
                          "ranges": [round(float(r), 3) for r in self.lidar.ranges]}
        if self.gps is not None:
            s["gps"] = self.gps
        return {
            "type": "obs", "tick": self.tick, "t": round(self.t, 4), "dt": self.dt, "seq": self.seq,
            "vehicle": self.vehicle,
            "pose": {"x": self.pose.x, "y": self.pose.y, "z": self.pose.z,
                     "roll": self.pose.roll, "pitch": self.pose.pitch, "yaw": self.pose.yaw},
            "vel": {"vx": self.vel.vx, "vy": self.vel.vy, "vz": self.vel.vz},
            "omega": {"wx": self.vel.wx, "wy": self.vel.wy, "wz": self.vel.wz},
            "acc": {"ax": self.acc[0], "ay": self.acc[1], "az": self.acc[2]},
            "u_applied": self.u_applied, "sensors": s,
            "events": {"collision": self.collision, "contacts": self.contacts,
                       "out_of_bounds": self.out_of_bounds, "capsized": self.capsized},
            "env": self.env,
        }


# ------------------------------------------------------------------ helpers

def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def wrap_array(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi


def clamp(x: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return lo if x < lo else hi if x > hi else x
