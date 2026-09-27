"""
Pure pursuit path tracker for the Ackermann car, with curvature-aware speed.

The three numbers you will actually tune:
    lookahead_gain   - lookahead = base + gain * speed. Too small oscillates,
                       too large cuts corners. Start 0.4 s.
    v_max            - target straight-line speed. Start at 60% of the vehicle's max.
    a_lat_max        - lateral accel budget; caps speed in corners. Start at
                       ~70% of mu*g for the surface you expect.
"""

from __future__ import annotations

import math

import numpy as np

from ..geometry import Polyline
from ..specs import SceneSpec, VehicleParams
from ..types import Observation, clamp, wrap
from .base import Controller
from .pid import PID


class PurePursuit(Controller):
    def __init__(self, scene: SceneSpec, vehicle: VehicleParams, path=None,
                 v_max: float | None = None, lookahead_base: float = 0.6, lookahead_gain: float = 0.4,
                 a_lat_max: float | None = None, kp_speed: float = 0.8, ki_speed: float = 0.4,
                 brake_gain: float = 0.5):
        pts = path if path is not None else (scene.task.get("waypoints_m") or scene.centerline)
        closed = bool(scene.track.get("closed", False)) if path is None else False
        self.path = Polyline(pts, closed)
        self.L = float(vehicle.car["wheelbase_m"])
        self.max_steer = float(vehicle.car["max_steer_rad"])
        self.v_max = v_max if v_max is not None else 0.6 * float(vehicle.car["max_speed_mps"])
        mu = float(scene.terrain.get("surface", {}).get("friction", 0.9))
        self.a_lat_max = a_lat_max if a_lat_max is not None else 0.7 * mu * 9.81
        self.ld_base, self.ld_gain = lookahead_base, lookahead_gain
        self.speed_pid = PID(kp_speed, ki_speed, 0.0, out_min=-1.0, out_max=1.0, i_limit=0.5)
        self.brake_gain = brake_gain
        self.stop_at_end = not closed
        self.start_s = self.path.project(scene.spawn_xy)[0]
        self.progress_s = self.start_s

    def reset(self) -> None:
        self.speed_pid.reset()
        self.progress_s = self.start_s

    def __call__(self, obs: Observation) -> dict[str, float]:
        v = obs.vel.vx
        xy = obs.pose.xy
        s, ct, _ = self.path.project_near(xy, self.progress_s, max(0.8, abs(v) * obs.dt * 3))
        self.progress_s = s
        ld = self.ld_base + self.ld_gain * abs(v)
        target = self.path.point_at(s + ld)

        # steering: pure pursuit in the body frame
        dx, dy = target - xy
        alpha = wrap(math.atan2(dy, dx) - obs.pose.yaw)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), ld)
        steer = clamp(delta / self.max_steer)

        # speed: curvature ahead limits speed via a lateral accel budget
        kappa = max(abs(self.path.curvature_at(s + ld)), abs(self.path.curvature_at(s + 2 * ld)), 1e-4)
        v_curve = math.sqrt(self.a_lat_max / kappa)
        v_ref = min(self.v_max, v_curve)
        if self.stop_at_end:
            remaining = self.path.length - s
            v_ref = min(v_ref, math.sqrt(2 * 2.0 * max(remaining, 0.0)))     # decel at 2 m/s^2
        e = v_ref - v
        cmd = self.speed_pid(e, obs.dt)
        if cmd >= 0:
            return {"throttle": cmd, "steer": steer, "brake": 0.0}
        return {"throttle": 0.0, "steer": steer, "brake": clamp(-cmd * self.brake_gain, 0, 1)}
