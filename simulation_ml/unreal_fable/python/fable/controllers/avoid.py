"""
ReactiveAvoid - wrap any path follower with a lidar-based steering bias.

Not a planner. It is the "don't hit the barrel that was not on the map" reflex:
it looks at the lidar in a forward cone, computes a repulsive steering term from
the nearest returns, and blends it with the base controller's steering. Use it
to make the slalom/buoy-field scenes survivable while you build a real local
planner - and to test that planner against something.

Works for both vehicles because it edits the steering-like channel only:
'steer' for the car, the yaw split between thrusters for the boat.
"""

from __future__ import annotations

import math

import numpy as np

from ..types import Observation, clamp
from .base import Controller
from .los import allocate_thrust


class ReactiveAvoid(Controller):
    def __init__(self, base: Controller, vehicle: str = "car", cone_half_rad: float = 1.0,
                 react_dist: float = 3.0, gain: float = 0.8, slow_dist: float = 1.5):
        self.base, self.vehicle = base, vehicle
        self.cone, self.react, self.gain, self.slow = cone_half_rad, react_dist, gain, slow_dist

    def reset(self) -> None:
        self.base.reset()

    def _bias(self, obs: Observation) -> tuple[float, float]:
        if obs.lidar is None:
            return 0.0, 1.0
        a = obs.lidar.angles
        r = obs.lidar.ranges
        m = (np.abs(a) < self.cone) & (r < self.react)
        if not m.any():
            return 0.0, 1.0
        w = (self.react - r[m]) / self.react                   # closer = heavier
        # push away from the side with more/closer returns
        bias = -float(np.sum(w * np.sign(a[m] + 1e-6) * np.cos(a[m]))) / max(float(np.sum(w)), 1e-6)
        nearest = float(r[m].min())
        speed_scale = clamp(nearest / self.slow, 0.25, 1.0)
        return clamp(self.gain * bias * (1.0 + (self.react - nearest) / self.react)), speed_scale

    def __call__(self, obs: Observation) -> dict[str, float]:
        u = self.base(obs)
        bias, scale = self._bias(obs)
        if self.vehicle == "car":
            u["steer"] = clamp(u.get("steer", 0.0) + bias)
            u["throttle"] = u.get("throttle", 0.0) * scale
            return u
        surge = (u["thrust_l"] + u["thrust_r"]) / 2 * scale
        yaw = (u["thrust_r"] - u["thrust_l"]) / 2 + bias
        return allocate_thrust(surge, yaw)
