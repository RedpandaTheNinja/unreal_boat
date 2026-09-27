"""Experimental local racing planner for a known, constant-width track.

Samples smooth lateral alternatives inside the corridor and checks them against
lidar returns inflated by a conservative car radius. Curvature and visible free
distance limit speed. This is a local planner, not a globally optimal racing line.
"""
from __future__ import annotations

import math
import numpy as np

from ..geometry import Polyline
from ..types import clamp, wrap
from .base import Controller


class RacingController(Controller):
    def __init__(self, scene, vehicle, v_max=None, margin=0.12):
        if not math.isfinite(margin) or margin < 0 or (v_max is not None and (not math.isfinite(v_max) or v_max <= 0)):
            raise ValueError('margin must be nonnegative and speed must be positive and finite')
        if scene.scenario != 'car' or not scene.track.get('closed'):
            raise ValueError('RacingController requires a closed car track')
        self.path = Polyline(scene.centerline, True)
        self.vehicle = vehicle
        self.start_s = self.path.project(scene.spawn_xy)[0]
        self.L = vehicle.car['wheelbase_m']
        self.max_steer = vehicle.car['max_steer_rad']
        self.radius = math.hypot(vehicle.common['length_m'], vehicle.common['width_m']) / 2
        self.margin = margin
        self.offset_limit = scene.track['width_m'] / 2 - self.radius - margin
        if self.offset_limit <= 0:
            raise ValueError('Track is narrower than the vehicle clearance envelope')
        mu = scene.terrain.get('surface', {}).get('friction', 0.9)
        mu = min(mu, scene.raw.get('randomization', {}).get('ranges', {}).get('friction', [mu])[0])
        self.a_lat = 0.55 * mu * 9.81
        self.decel = min(0.5 * vehicle.car['max_brake_mps2'], 0.5 * mu * 9.81)
        delay_max = scene.raw.get('randomization', {}).get('ranges', {}).get('actuator_delay_s', [vehicle.common['actuator_delay_s']])[-1]
        self.delay = max(vehicle.common['actuator_delay_s'], delay_max) + 1 / vehicle.sensors['lidar']['rate_hz'] + 0.1
        self.v_max = min(v_max if v_max is not None else 0.85 * vehicle.car['max_speed_mps'], vehicle.car['max_speed_mps'])
        self.mount = np.asarray(vehicle.sensors['lidar'].get('mount_m', [0, 0])[:2])
        self.reset()

    def reset(self):
        self.progress_s = self.start_s
        self.offset = 0.0
        self.last_scan = None
        self.scan_points = np.empty((0, 2))
        self.scan_time = None
        self.status = 'starting'
        self.target_speed = 0.0
        self.selected_path = np.empty((0, 2))

    def __call__(self, obs):
        stop = {'throttle': 0.0, 'steer': 0.0, 'brake': 1.0}
        self.target_speed = 0.0
        if obs.lidar is None or not np.all(np.isfinite(obs.lidar.ranges)):
            self.status = 'missing_or_invalid_lidar'
            return stop
        v = max(0.0, obs.vel.vx)
        s, ct, _ = self.path.project_near(obs.pose.xy, self.progress_s, max(0.8, v * obs.dt * 3))
        self.progress_s = s
        if abs(ct) > self.offset_limit + self.margin:
            self.status = 'outside_track'
            return stop
        if obs.lidar is not self.last_scan:
            points = obs.lidar.points_body() + self.mount
            c, sn = math.cos(obs.pose.yaw), math.sin(obs.pose.yaw)
            self.scan_points = points @ np.array([[c, sn], [-sn, c]]) + obs.pose.xy
            self.last_scan, self.scan_time = obs.lidar, obs.t
        if obs.t - self.scan_time > 0.3:
            self.status = 'stale_lidar'
            return stop

        # Restrict planning to the observable forward region and allow braking
        # before its end. The scan's original pose is retained between updates.
        horizon = min(9.0, obs.lidar.range_max * 0.65)
        ds = np.arange(0, horizon + 0.01, 0.2)
        center = np.array([self.path.point_at(s + d) for d in ds])
        heading = np.array([self.path.heading_at(s + d) for d in ds])
        normals = np.column_stack((-np.sin(heading), np.cos(heading)))
        route_curve = np.array([abs(self.path.curvature_at(s + d, ds=0.6)) for d in ds])
        transition = max(2.5, min(4.0, 1.0 + v))
        u = np.clip(ds / transition, 0, 1)
        blend = u*u*(3 - 2*u)
        slope = clamp(math.tan(wrap(obs.pose.yaw - heading[0])), -0.5, 0.5)
        tangent = transition * slope * u * (1-u)**2
        best = None
        for offset in np.unique(np.r_[np.linspace(-self.offset_limit, self.offset_limit, 9), self.offset, 0.0]):
            lateral = ct + (offset - ct) * blend + tangent
            if np.max(np.abs(lateral)) > self.offset_limit + 1e-6:
                continue
            pts = center + normals * lateral[:, None]
            if len(self.scan_points):
                distances = np.linalg.norm(pts[:, None, :] - self.scan_points[None, :, :], axis=2).min(axis=1)
                hit = np.flatnonzero(distances < self.radius + self.margin + 0.1)
                free = max(0.0, ds[hit[0]] - 0.3) if len(hit) else horizon
            else:
                free = horizon
            # Reject routes that cannot even support an emergency stop.
            stopping = v * self.delay + v*v / (2*self.decel)
            if free < stopping + 0.15:
                continue
            curvature = route_curve + abs(offset-ct) * 6 / transition**2
            limits = np.minimum(self.v_max, np.sqrt(self.a_lat / np.maximum(curvature, 0.001)))
            speed = float(np.min(np.sqrt(limits**2 + 2*self.decel*np.maximum(0, ds-v*self.delay))))
            stop_speed = -self.decel*self.delay + math.sqrt((self.decel*self.delay)**2 + 2*self.decel*free)
            speed = min(speed, stop_speed)
            # Prefer a clear horizon before maximizing speed; retain passing side.
            score = speed + 0.4*free - 0.25*abs(offset) - 0.35*abs(offset-self.offset)
            if best is None or score > best[0]:
                best = (score, offset, pts, speed, free)
        if best is None:
            self.status = 'no_safe_candidate'
            return stop
        _, self.offset, self.selected_path, self.target_speed, free = best
        ld = 0.45 + 0.25*v
        target = self.selected_path[min(len(ds)-1, max(1, int(round(ld/0.2))))]
        delta_xy = target - obs.pose.xy
        alpha = wrap(math.atan2(delta_xy[1], delta_xy[0]) - obs.pose.yaw)
        delta = math.atan2(2*self.L*math.sin(alpha), max(np.linalg.norm(delta_xy), 0.1))
        delta *= 1 + self.vehicle.car.get('understeer_gain', 0.0)*v*v
        steer = clamp(delta / self.max_steer)
        error = self.target_speed - v
        self.status = 'driving' if free >= horizon else 'braking_for_obstacle'
        if error < -0.1:
            return {'throttle': 0.0, 'steer': steer, 'brake': clamp(-error / 1.5, 0, 1)}
        throttle = self.target_speed/self.vehicle.car['max_speed_mps'] + 0.35*error
        return {'throttle': clamp(throttle, 0, 1), 'steer': steer, 'brake': 0.0}
