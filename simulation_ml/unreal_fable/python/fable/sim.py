"""
sim.py - one interface over both backends, plus the task evaluator.

    with Sim(scene, backend="mock") as sim:
        obs = sim.reset(seed=0)
        while not sim.done:
            obs = sim.step(controller(obs))
    print(sim.report())

The evaluator reads `spec.task` and scores exactly what it lists, so a scene
spec is a complete, reproducible test case: world + vehicle + pass/fail + metrics.
"""

from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .geometry import Polyline, crossed_gate
from .specs import SceneSpec, VehicleParams
from .types import Observation, wrap


# --------------------------------------------------------------- evaluator

@dataclass
class TaskState:
    t0: float = 0.0
    steps: int = 0
    progress_s: float = 0.0
    laps: int = 0
    waypoint_idx: int = 0
    gates_passed: int = 0
    collisions: int = 0
    cross_track: list[float] = field(default_factory=list)
    heading_err: list[float] = field(default_factory=list)
    effort: float = 0.0
    min_clearance: float = math.inf
    done: bool = False
    success: bool = False
    reason: str = ""
    dock_error_m: float | None = None
    dock_heading_err: float | None = None
    _prev_xy: np.ndarray | None = None
    _prev_u: dict = field(default_factory=dict)
    _was_colliding: bool = False
    _prev_s: float = 0.0
    _hold_steps: int = 0
    _speed_lp: float = 0.0


class TaskEvaluator:
    def __init__(self, spec: SceneSpec, dt: float, vehicle: VehicleParams | None = None,
                 strict_track: bool = False):
        self.spec, self.task, self.dt = spec, spec.task, dt
        self.kind = self.task.get("type", "waypoints")
        self.tol = float(self.task.get("tolerance_m", 0.6))
        self.max_t = float(self.task.get("max_time_s", 120))
        self.fail_on = set(self.task.get("fail_on", []))
        self.line = Polyline(spec.centerline, spec.track.get("closed", False)) if len(spec.centerline) > 1 else None
        wps = self.task.get("waypoints_m") or spec.centerline
        self.wps = [np.asarray(w, float) for w in wps]
        self.gates = spec.gates
        self.dock = spec.dock
        self.st = TaskState()
        self.strict_track = strict_track
        vp = vehicle or spec.vehicle()
        self.footprint_radius = math.hypot(vp.common['length_m'], vp.common['width_m']) / 2
        self.min_track_clearance = math.inf
        if strict_track and (spec.scenario != 'car' or self.line is None):
            raise ValueError('strict_track requires a car with a track centerline')

    def reset(self, obs: Observation) -> None:
        self.st = TaskState(t0=obs.t)
        self.min_track_clearance = math.inf
        self.st._prev_xy = obs.pose.xy.copy()
        if self.line is not None:
            anchor = self.line.project(self.spec.spawn_xy)[0]
            self.st._prev_s, _, _ = self.line.project_near(obs.pose.xy, anchor, 1.0)
            self.st.progress_s = 0.0

    def update(self, obs: Observation, u: dict[str, float]) -> None:
        st = self.st
        if st.done:
            return
        st.steps += 1
        xy = obs.pose.xy
        elapsed = obs.t - st.t0
        # 1 s low-pass on the (noisy) speed sensor for "is it stopped" decisions
        a = self.dt / (1.0 + self.dt)
        st._speed_lp += a * (obs.speed - st._speed_lp)

        # control effort = sum |du| (smoothness), not magnitude
        if st._prev_u:
            st.effort += sum(abs(float(u.get(k, 0)) - float(st._prev_u.get(k, 0))) for k in set(u) | set(st._prev_u))
        st._prev_u = dict(u)

        if obs.lidar is not None:
            st.min_clearance = min(st.min_clearance, float(obs.lidar.ranges.min()))

        if obs.collision and not st._was_colliding:
            st.collisions += 1
        st._was_colliding = obs.collision

        # Failure must win over finishing a lap/goal on the same observation.
        footprint_hit = self.strict_track and obs.extra.get('footprint_obstacle_clearance_m', math.inf) < 0
        if (obs.collision or footprint_hit) and ('collision' in self.fail_on or self.strict_track):
            if footprint_hit and not obs.collision:
                st.collisions += 1
            return self._finish(False, f"collision:{','.join(obs.contacts) or '?'}")
        if obs.out_of_bounds and 'out_of_bounds' in self.fail_on:
            return self._finish(False, 'out_of_bounds')
        if obs.capsized and 'capsize' in self.fail_on:
            return self._finish(False, 'capsized')

        # path tracking metrics
        if self.line is not None:
            if self.kind == 'lap':
                s, ct, _ = self.line.project_near(xy, st._prev_s, max(0.8, obs.speed * self.dt * 3))
            else:
                s, ct, _ = self.line.project(xy)
            st.cross_track.append(ct)
            clearance = self.spec.track.get('width_m', 2.0) / 2 - abs(ct) - self.footprint_radius
            self.min_track_clearance = min(self.min_track_clearance, clearance)
            if self.strict_track and clearance < 0:
                return self._finish(False, 'footprint_off_track')
            st.heading_err.append(wrap(self.line.heading_at(s) - obs.pose.yaw))
            ds = s - st._prev_s
            if self.line.closed:
                if ds < -self.line.length / 2:
                    ds += self.line.length          # wrapped past the start
                elif ds > self.line.length / 2:
                    ds -= self.line.length
            st._prev_s = s
            st.progress_s += ds
            if self.kind == "lap" and st.progress_s >= self.line.length * (st.laps + 1) - self.tol:
                st.laps += 1
                if st.laps >= int(self.task.get("laps", 1)):
                    return self._finish(True, "laps_complete")
            if self.kind == "lap" and "off_track" in self.fail_on and abs(ct) > self.spec.track.get("width_m", 2.0) / 2 + self.tol:
                return self._finish(False, "off_track")

        # waypoints / goal
        if self.kind in ("waypoints", "goal"):
            targets = self.wps if self.kind == "waypoints" else [np.asarray(self.task["goal_m"], float)]
            while st.waypoint_idx < len(targets) and np.linalg.norm(xy - targets[st.waypoint_idx]) < self.tol:
                st.waypoint_idx += 1
            if st.waypoint_idx >= len(targets):
                return self._finish(True, "waypoints_complete")

        # gates
        if self.kind == "gates" and st._prev_xy is not None and st.gates_passed < len(self.gates):
            g = self.gates[st.gates_passed]
            if crossed_gate(st._prev_xy, xy, g["left_m"], g["right_m"]):
                st.gates_passed += 1
                if st.gates_passed >= len(self.gates):
                    return self._finish(True, "all_gates")

        # docking: stopped inside the slip
        if self.kind == "dock" and self.dock:
            d = np.asarray(self.dock["xy_m"], float)
            yaw_d = float(self.dock.get("yaw_rad", 0))
            ax = np.array([math.cos(yaw_d), math.sin(yaw_d)])
            along = abs(float(np.dot(d - xy, ax)))            # error along the slip axis
            err = float(np.linalg.norm(xy - d))
            herr = abs(wrap(obs.pose.yaw - yaw_d))
            st.dock_error_m, st.dock_heading_err = err, herr
            # cross-axis position is bounded by the fenders, so success is judged
            # on along-axis error, heading and being stopped
            if along < self.tol and herr < 0.2 and st._speed_lp < 0.08:
                st._hold_steps += 1
                if st._hold_steps * self.dt >= 2.0:
                    return self._finish(True, "docked")
            else:
                st._hold_steps = 0

        if self.kind == "station_keep":
            d = np.asarray(self.task.get("goal_m", self.spec.spawn_xy), float)
            st.cross_track.append(float(np.linalg.norm(xy - d)))

        # failures
        if obs.collision and "collision" in self.fail_on:
            return self._finish(False, f"collision:{','.join(obs.contacts) or '?'}")
        if obs.out_of_bounds and "out_of_bounds" in self.fail_on:
            return self._finish(False, "out_of_bounds")
        if obs.capsized and "capsize" in self.fail_on:
            return self._finish(False, "capsized")
        if elapsed >= self.max_t:
            return self._finish(False if "timeout" in self.fail_on else (self.kind == "station_keep"), "timeout")

        st._prev_xy = xy.copy()

    def _finish(self, ok: bool, reason: str) -> None:
        self.st.done, self.st.success, self.st.reason = True, ok, reason

    def report(self, obs: Observation | None) -> dict[str, Any]:
        st = self.st
        elapsed = (obs.t - st.t0) if obs else st.steps * self.dt
        r: dict[str, Any] = {"scene": self.spec.name, "task": self.kind, "success": st.success,
                             "reason": st.reason, "time_s": round(elapsed, 2), "steps": st.steps}
        want = set(self.task.get("metrics") or ["cross_track_rms", "time", "collisions"])
        if "cross_track_rms" in want and st.cross_track:
            r["cross_track_rms_m"] = round(float(np.sqrt(np.mean(np.square(st.cross_track)))), 3)
            r["cross_track_max_m"] = round(float(np.max(np.abs(st.cross_track))), 3)
        if "heading_error_rms" in want and st.heading_err:
            r["heading_error_rms_rad"] = round(float(np.sqrt(np.mean(np.square(st.heading_err)))), 3)
        if "collisions" in want:
            r["collisions"] = st.collisions
        if "control_effort" in want:
            r["control_effort"] = round(st.effort, 2)
        if "min_clearance_m" in want and math.isfinite(st.min_clearance):
            r["min_clearance_m"] = round(st.min_clearance, 2)
        if "gates_passed" in want:
            r["gates_passed"] = f"{st.gates_passed}/{len(self.gates)}"
        if "dock_error_m" in want and st.dock_error_m is not None:
            r["dock_error_m"] = round(st.dock_error_m, 3)
            r["dock_heading_err_rad"] = round(st.dock_heading_err or 0.0, 3)
        if self.kind == "lap":
            r["laps"] = st.laps
            r["progress_m"] = round(st.progress_s, 1)
        if self.strict_track:
            r['strict_track'] = True
            r['min_track_clearance_m'] = round(self.min_track_clearance, 3) if math.isfinite(self.min_track_clearance) else None
        if self.kind == "waypoints":
            r["waypoints"] = f"{st.waypoint_idx}/{len(self.wps)}"
        return r


# ------------------------------------------------------------------ logger

class EpisodeLog:
    """CSV log in the calibration format (docs/CALIBRATION.md). Same columns for sim and real."""

    COLS = ["t", "x", "y", "yaw", "vx", "vy", "wz", "ax", "ay", "speed",
            "u_throttle", "u_steer", "u_brake", "u_thrust_l", "u_thrust_r", "collision"]

    def __init__(self, path: str | None):
        self.rows: list[list[float]] = []
        self.path = path

    def add(self, obs: Observation, u: dict[str, float]) -> None:
        self.rows.append([obs.t, obs.pose.x, obs.pose.y, obs.pose.yaw, obs.vel.vx, obs.vel.vy, obs.vel.wz,
                          obs.acc[0], obs.acc[1], obs.speed,
                          u.get("throttle", 0.0), u.get("steer", 0.0), u.get("brake", 0.0),
                          u.get("thrust_l", 0.0), u.get("thrust_r", 0.0), int(obs.collision)])

    def save(self) -> str | None:
        if not self.path:
            return None
        with open(self.path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(self.COLS)
            w.writerows(self.rows)
        return self.path

    def as_array(self) -> np.ndarray:
        return np.asarray(self.rows, dtype=float)


# --------------------------------------------------------------------- Sim

class Sim:
    """
    backend: "mock" (in-process physics) or "unreal" (UDP bridge).
    """

    def __init__(self, scene: SceneSpec, backend: str = "mock", vehicle: VehicleParams | None = None,
                 dt: float | None = None, log_path: str | None = None, realtime: bool = False,
                 strict_track: bool = False, **backend_kw):
        self.scene = scene
        self.vehicle = vehicle or scene.vehicle()
        self.dt = dt or 1.0 / float(self.vehicle.common.get("control_rate_hz", 50))
        self.backend_name = backend
        if backend == "mock":
            from .mock_sim import MockBackend
            self.backend = MockBackend(scene, self.vehicle, dt=self.dt)
        elif backend == "unreal":
            from .bridge import UnrealBackend
            self.backend = UnrealBackend(scene, self.vehicle, dt=self.dt, **backend_kw)
        else:
            raise ValueError("backend must be 'mock' or 'unreal'")
        self.eval = TaskEvaluator(scene, self.dt, self.vehicle, strict_track)
        self.log = EpisodeLog(log_path)
        self.realtime = realtime
        self.obs: Observation | None = None
        self._wall0 = 0.0

    # --- context
    def __enter__(self) -> "Sim":
        return self

    def __exit__(self, *a) -> None:
        self.close()

    def close(self) -> None:
        self.log.save()
        self.backend.close()

    # --- episode
    def reset(self, seed: int | None = None, pose: dict | None = None) -> Observation:
        self.obs = self.backend.reset(pose=pose, seed=seed)
        self.eval.reset(self.obs)
        self.log.rows.clear()
        self._wall0 = time.monotonic() - self.obs.t
        return self.obs

    def step(self, u: dict[str, float]) -> Observation:
        if self.realtime and self.backend_name == "mock":
            lag = (self.obs.t + self.dt) - (time.monotonic() - self._wall0)
            if lag > 0:
                time.sleep(lag)
        self.obs = self.backend.step(u)
        self.eval.update(self.obs, u)
        self.log.add(self.obs, u)
        return self.obs

    @property
    def done(self) -> bool:
        return self.eval.st.done

    def report(self) -> dict[str, Any]:
        return self.eval.report(self.obs)

    # --- world edits, live
    def spawn(self, kind: str, label: str, x: float, y: float, yaw: float = 0.0, z: float = 0.0, scale=None) -> dict:
        return self.backend.spawn(kind, label, x, y, z, yaw, scale)

    def despawn(self, label: str) -> dict:
        return self.backend.despawn(label)

    def set_env(self, **kw) -> dict:
        return self.backend.set_env(**kw)

    def set_params(self, params: VehicleParams | dict) -> dict:
        self.vehicle = params if isinstance(params, VehicleParams) else self.vehicle
        return self.backend.set_params(params)

    def scene_info(self) -> dict:
        return self.backend.scene()

    def run(self, controller, seed: int | None = None, max_steps: int | None = None) -> dict[str, Any]:
        """Convenience: one full episode with a callable controller(obs) -> u."""
        obs = self.reset(seed=seed)
        if hasattr(controller, "reset"):
            controller.reset()
        n = 0
        while not self.done and (max_steps is None or n < max_steps):
            obs = self.step(controller(obs))
            n += 1
        return self.report()
