"""
mock_sim.py - pure-Python stand-in for the Unreal backend.

Not a toy: a first-order-actuated bicycle model for the car and a Fossen 3-DOF
(surge/sway/yaw) model with twin thrusters, wind, current and gusts for the boat,
both driven by the SAME vehicle_params.json the Unreal pawns read. Its job is to
let you iterate on control logic at 10,000 steps/second with no engine running,
and to serve as the model the calibration fitters fit to your real logs.

It produces the same `Observation` as the bridge, including a ray-cast 2-D lidar
against the obstacles in the scene spec, so a controller cannot tell the two
apart. When behaviour differs between mock and Unreal, that difference IS the
information you wanted: either the mock's physics is too simple, or the Unreal
setup is wrong. Both are worth knowing.
"""

from __future__ import annotations

import math
import random
from collections import deque
from typing import Any

import numpy as np

from .geometry import Obstacles, Polyline
from .specs import SceneSpec, VehicleParams
from .types import Lidar, Observation, Pose, Twist, clamp, wrap

G = 9.81

# kind -> footprint for collision/lidar (radius, or box [sx, sy]), default height
KIND_GEOM: dict[str, dict[str, Any]] = {
    "cone": {"r": 0.18}, "barrel": {"r": 0.29}, "box": {"box": [0.5, 0.5]}, "wall": {"box": [1.0, 1.0]},
    "ramp": {"box": [2.0, 1.5]}, "pole": {"r": 0.06}, "rock": {"r": 0.45}, "pedestrian_static": {"r": 0.25},
    "buoy_red": {"r": 0.3}, "buoy_green": {"r": 0.3}, "buoy_yellow": {"r": 0.3},
    "moored_boat": {"box": [3.0, 1.2]}, "dock_piece": {"box": [1.0, 1.0]}, "custom_mesh": {"box": [1.0, 1.0]},
}


class DelayLine:
    """Pure transport delay for actuator commands, in whole control steps."""

    def __init__(self, delay_s: float, dt: float, initial: dict[str, float]):
        n = max(0, int(round(delay_s / dt)))
        self.q: deque = deque([dict(initial)] * (n + 1), maxlen=n + 1)

    def push(self, u: dict[str, float]) -> dict[str, float]:
        self.q.append(dict(u))
        return self.q[0]


class Gust:
    """Ornstein-Uhlenbeck gust process around the mean wind speed."""

    def __init__(self, std: float, period: float, rng: random.Random):
        self.std, self.tau, self.rng, self.x = std, max(period, 0.1), rng, 0.0

    def step(self, dt: float) -> float:
        a = math.exp(-dt / self.tau)
        self.x = a * self.x + math.sqrt(max(1 - a * a, 0)) * self.rng.gauss(0, self.std)
        return self.x


# ---------------------------------------------------------------- the scene

class SceneWorld:
    """Static obstacle set + bounds + water/terrain lookups derived from a spec."""

    def __init__(self, spec: SceneSpec, seed: int = 0):
        self.spec = spec
        self.obs = Obstacles()
        self.dynamic: dict[str, dict] = {}
        self.sx, self.sy = spec.size
        self.centerline = Polyline(spec.centerline, spec.track.get("closed", False)) if len(spec.centerline) > 1 else None
        self.rebuild(seed)

    # --- population
    def rebuild(self, seed: int) -> None:
        rng = random.Random(seed)
        self.obs.clear()
        spec = self.spec

        for o in spec.objects:
            self._add_kind(o.get("label") or f"{o['kind']}_{len(self.obs.circles)}", o["kind"],
                           o["xy_m"], o.get("yaw_rad", 0.0), o.get("scale"))

        # track borders as cones/walls
        tr = spec.track
        if self.centerline is not None and tr.get("border", "none") in ("cones", "walls", "curbs"):
            spacing = float(tr.get("border_spacing_m", 1.5))
            half = float(tr.get("width_m", 2.0)) / 2
            s = 0.0
            while s < self.centerline.length:
                p = self.centerline.point_at(s)
                h = self.centerline.heading_at(s)
                n = np.array([-math.sin(h), math.cos(h)])
                for side in (1, -1):
                    q = p + n * half * side
                    if tr["border"] == "cones":
                        self.obs.add_circle(q[0], q[1], 0.18, "track_border")
                    else:
                        self.obs.add_box(q[0], q[1], h, spacing, 0.15, "track_border")
                s += spacing

        for i, g in enumerate(spec.gates):
            lab = g.get("label", f"gate{i + 1}")
            self.obs.add_circle(*g["left_m"], 0.3, f"{lab}_L")
            self.obs.add_circle(*g["right_m"], 0.3, f"{lab}_R")

        for sh in spec.water.get("shore", []) or []:
            self.obs.add_polygon(sh["polygon_m"], sh.get("label", "shore"))

        d = spec.dock
        if d:
            self._add_dock(d)

        for f in spec.obstacle_fields:
            self._scatter(f, rng)

        for lab, o in self.dynamic.items():
            self._add_kind(lab, o["kind"], o["xy"], o.get("yaw", 0.0), o.get("scale"))

    def _add_kind(self, label, kind, xy, yaw=0.0, scale=None) -> None:
        g = KIND_GEOM.get(kind, {"r": 0.3})
        sc = scale or [1, 1, 1]
        if "r" in g:
            self.obs.add_circle(xy[0], xy[1], g["r"] * max(sc[0], sc[1]), label)
        else:
            self.obs.add_box(xy[0], xy[1], yaw, g["box"][0] * sc[0], g["box"][1] * sc[1], label)

    def _add_dock(self, d: dict) -> None:
        cx, cy = d["xy_m"]
        yaw = float(d.get("yaw_rad", 0))
        w, L, P = float(d.get("slip_width_m", 1.6)), float(d.get("slip_length_m", 3.0)), float(d.get("pier_length_m", 8))
        c, s = math.cos(yaw), math.sin(yaw)

        def to_world(x, y):
            return cx + c * x - s * y, cy + s * x + c * y

        # main pier along +y at x = +L/2 (the end of the slip), two fingers along -x
        px, py = to_world(L / 2 + 0.3, 0)
        self.obs.add_box(px, py, yaw, 0.6, P, "pier")
        for side in (1, -1):
            fx, fy = to_world(0.0, side * (w / 2 + 0.15))
            self.obs.add_box(fx, fy, yaw, L, 0.3, "pier_finger")

    def _scatter(self, f: dict, rng: random.Random) -> None:
        x0, y0, x1, y1 = [float(v) for v in f["area_m"]]
        sep = float(f.get("min_separation_m", 1.5))
        keep = float(f.get("avoid_track_m", 1.0))
        jit = float(f.get("scale_jitter", 0.15))
        placed = []
        tries = 0
        gate_pts = [g["left_m"] for g in self.spec.gates] + [g["right_m"] for g in self.spec.gates]
        while len(placed) < int(f["count"]) and tries < int(f["count"]) * 80:
            tries += 1
            x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
            if any(math.hypot(x - a, y - b) < sep for a, b in placed):
                continue
            if keep > 0 and self.centerline is not None:
                _, ct, _ = self.centerline.project((x, y))
                if abs(ct) < keep + self.spec.track.get("width_m", 2.0) / 2:
                    continue
            if keep > 0 and any(math.hypot(x - g[0], y - g[1]) < keep + 2.0 for g in gate_pts):
                continue
            if math.hypot(x - self.spec.spawn_xy[0], y - self.spec.spawn_xy[1]) < 3.0:
                continue
            placed.append((x, y))
            s = rng.uniform(1 - jit, 1 + jit)
            self._add_kind(f"field_{f['kind']}_{len(placed)}", f["kind"], (x, y), rng.uniform(0, 2 * math.pi), [s, s, s])

    # --- runtime edits (mirror of bridge spawn/despawn)
    def spawn(self, label, kind, x, y, yaw=0.0, scale=None) -> None:
        self.dynamic[label] = {"kind": kind, "xy": (x, y), "yaw": yaw, "scale": scale}
        self._add_kind(label, kind, (x, y), yaw, scale)

    def despawn(self, label: str) -> int:
        if label == "dynamic:*":
            n = sum(self.obs.remove(l) for l in list(self.dynamic))
            self.dynamic.clear()
            return n
        self.dynamic.pop(label, None)
        return self.obs.remove(label)

    def in_bounds(self, x, y) -> bool:
        return abs(x) <= self.sx / 2 + 2 and abs(y) <= self.sy / 2 + 2

    def friction(self) -> float:
        return float(self.spec.terrain.get("surface", {}).get("friction", 0.9))


# ---------------------------------------------------------------- sensors

class SensorPack:
    def __init__(self, vp: VehicleParams, world: SceneWorld, dt: float, rng: random.Random, geo=None):
        self.vp, self.world, self.dt, self.rng, self.geo = vp, world, dt, rng, geo
        L = vp.sensors.get("lidar", {})
        self.lidar_on = bool(L.get("enabled", True))
        n = int(L.get("n_rays", 360))
        fov = float(L.get("fov_rad", 2 * math.pi))
        self.angles = np.linspace(-fov / 2, fov / 2, n)
        self.range_max = float(L.get("range_max_m", 20))
        self.lidar_noise = float(L.get("noise_std_m", 0.0))
        self.lidar_period = 1.0 / float(L.get("rate_hz", 10))
        self.mount = list(L.get("mount_m", [0, 0, 0]))
        self._last_lidar_t = -1e9
        self._lidar: Lidar | None = None
        gps = vp.sensors.get("gps", {})
        self.gps_on = bool(gps.get("enabled", False)) and geo is not None
        self.gps_noise = float(gps.get("noise_std_m", 1.0))
        self.gps_period = 1.0 / float(gps.get("rate_hz", 5))
        self._last_gps_t = -1e9
        self._gps: dict | None = None
        self.speed_noise = float(vp.sensors.get("odom", {}).get("speed_noise_std", 0.0))
        self.pose_noise = float(vp.sensors.get("odom", {}).get("pose_noise_std_m", 0.0))
        self.gyro_noise = float(vp.sensors.get("imu", {}).get("gyro_noise_std", 0.0))
        self.gyro_bias = float(vp.sensors.get("imu", {}).get("gyro_bias", 0.0))
        self.acc_noise = float(vp.sensors.get("imu", {}).get("accel_noise_std", 0.0))

    def lidar(self, t: float, x: float, y: float, yaw: float) -> Lidar | None:
        if not self.lidar_on:
            return None
        if t - self._last_lidar_t >= self.lidar_period - 1e-9 or self._lidar is None:
            ox = x + math.cos(yaw) * self.mount[0] - math.sin(yaw) * self.mount[1]
            oy = y + math.sin(yaw) * self.mount[0] + math.cos(yaw) * self.mount[1]
            r = self.world.obs.raycast(ox, oy, self.angles + yaw, self.range_max)
            if self.lidar_noise > 0:
                noise = np.array([self.rng.gauss(0, self.lidar_noise) for _ in range(len(r))])
                r = np.clip(r + noise * (r < self.range_max), 0.05, self.range_max)
            self._lidar = Lidar(float(self.angles[0]), float(self.angles[-1]), self.range_max, r.astype(np.float32))
            self._last_lidar_t = t
        return self._lidar

    def gps(self, t: float, x: float, y: float) -> dict | None:
        if not self.gps_on:
            return None
        if t - self._last_gps_t >= self.gps_period - 1e-9 or self._gps is None:
            lat0, lon0 = float(self.geo["lat"]), float(self.geo["lon"])
            nx = x + self.rng.gauss(0, self.gps_noise)
            ny = y + self.rng.gauss(0, self.gps_noise)
            self._gps = {"lat": lat0 + ny / 111_320.0,
                         "lon": lon0 + nx / (111_320.0 * math.cos(math.radians(lat0))),
                         "hdop": 1.0}
            self._last_gps_t = t
        return self._gps


# ------------------------------------------------------------------- car

class MockCar:
    """Kinematic bicycle + first-order longitudinal response + servo model + friction circle."""

    def __init__(self, vp: VehicleParams, dt: float, rng: random.Random):
        self.vp, self.dt, self.rng = vp, dt, rng
        self.set_params(vp)
        self.x = self.y = self.yaw = 0.0
        self.v = 0.0
        self.delta = 0.0
        self.yaw_rate = 0.0
        self.ax = self.ay = 0.0
        self.delay = DelayLine(self.p_common["actuator_delay_s"], dt, {"throttle": 0, "steer": 0, "brake": 0})
        self.u_applied = {"throttle": 0.0, "steer": 0.0, "brake": 0.0}

    def set_params(self, vp: VehicleParams) -> None:
        self.vp = vp
        self.p = vp.car
        self.p_common = vp.common

    def reset(self, x, y, yaw) -> None:
        self.x, self.y, self.yaw = x, y, yaw
        self.v = self.delta = self.yaw_rate = self.ax = self.ay = 0.0
        self.delay = DelayLine(self.p_common["actuator_delay_s"], self.dt, {"throttle": 0, "steer": 0, "brake": 0})

    def step(self, u: dict[str, float], mu: float) -> None:
        p, dt = self.p, self.dt
        u = self.delay.push({"throttle": clamp(u.get("throttle", 0.0)), "steer": clamp(u.get("steer", 0.0)),
                             "brake": clamp(u.get("brake", 0.0), 0, 1)})

        # --- steering servo: deadband, bias, slew limit
        sc = u["steer"] - p["steer_bias"]
        if abs(sc) < p["steer_deadband"]:
            sc = 0.0
        target = clamp(sc) * p["max_steer_rad"]
        d_max = p["steer_rate_rad_s"] * dt
        self.delta += clamp(target - self.delta, -d_max, d_max)

        # --- longitudinal: first-order toward throttle*max_speed, with accel/brake caps and drag
        if u["brake"] > 0:
            a = -math.copysign(p["max_brake_mps2"] * u["brake"], self.v) if abs(self.v) > 1e-3 else 0.0
        else:
            v_ref = u["throttle"] * (p["max_speed_mps"] if u["throttle"] >= 0 else p["max_reverse_mps"])
            a = (v_ref - self.v) / max(p["accel_time_constant_s"], 1e-3)
            a = clamp(a, -p["max_brake_mps2"], p["max_accel_mps2"])
        m = self.p_common["mass_kg"]
        a -= (p["drag_coeff"] * self.v * abs(self.v) + p["rolling_resistance"] * self.v) / m
        a = clamp(a, -mu * G, mu * G)                       # traction limit
        v_new = self.v + a * dt
        if u["brake"] > 0 and (v_new * self.v) < 0:
            v_new = 0.0
        self.ax = (v_new - self.v) / dt
        self.v = v_new

        # --- lateral: bicycle with understeer + friction circle
        L = p["wheelbase_m"]
        delta_eff = self.delta / (1.0 + p["understeer_gain"] * self.v * self.v)
        yaw_rate = self.v * math.tan(delta_eff) / L
        ay = self.v * yaw_rate
        ay_max = math.sqrt(max((mu * G) ** 2 - self.ax ** 2, 0.0))
        if abs(ay) > ay_max and abs(self.v) > 1e-3:
            yaw_rate = math.copysign(ay_max, ay) / self.v   # sliding: yaw authority saturates
            ay = self.v * yaw_rate
        self.ay, self.yaw_rate = ay, yaw_rate

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = wrap(self.yaw + yaw_rate * dt)
        self.u_applied = {"throttle": u["throttle"], "steer": u["steer"], "brake": u["brake"]}

    def half_width(self) -> float:
        return self.p_common["width_m"] / 2

    def extra(self) -> dict:
        r = max(self.p["wheel_radius_m"], 1e-3)
        rpm = self.v / (2 * math.pi * r) * 60
        return {"wheel": {"steer_angle": self.delta, "rpm": [rpm] * 4}}


# ------------------------------------------------------------------ boat

class MockBoat:
    """
    Fossen 3-DOF: (M + MA) nu_dot + C(nu) nu + D(nu) nu = tau_thr + tau_wind.
    nu = [u, v, r] body velocities; current enters kinematically. Waves add a
    small cosmetic heave/roll and a yaw disturbance proportional to amplitude.
    """

    def __init__(self, vp: VehicleParams, dt: float, rng: random.Random):
        self.vp, self.dt, self.rng = vp, dt, rng
        self.set_params(vp)
        self.x = self.y = self.yaw = 0.0
        self.u = self.v = self.r = 0.0
        self.T = [0.0, 0.0]                     # actual thrust L, R (N)
        self.delay = DelayLine(self.p_common["actuator_delay_s"], dt, {"thrust_l": 0, "thrust_r": 0})
        self.u_applied = {"thrust_l": 0.0, "thrust_r": 0.0}
        self.ax = self.ay = 0.0
        self.z = self.roll = self.pitch = 0.0

    def set_params(self, vp: VehicleParams) -> None:
        self.vp, self.p, self.p_common = vp, vp.boat, vp.common
        self.m = self.p_common["mass_kg"]
        self.Iz = vp.yaw_inertia
        self.Mu = self.m + self.p["added_mass_surge_kg"]
        self.Mv = self.m + self.p["added_mass_sway_kg"]
        self.Mr = self.Iz + self.p["added_inertia_yaw_kgm2"]

    def reset(self, x, y, yaw) -> None:
        self.x, self.y, self.yaw = x, y, yaw
        self.u = self.v = self.r = 0.0
        self.T = [0.0, 0.0]
        self.delay = DelayLine(self.p_common["actuator_delay_s"], self.dt, {"thrust_l": 0, "thrust_r": 0})

    def _thrust_cmd(self, c: float) -> float:
        if abs(c) < self.p["thrust_deadband"]:
            return 0.0
        return c * (self.p["max_thrust_fwd_n"] if c > 0 else self.p["max_thrust_rev_n"])

    def step(self, ucmd: dict[str, float], wind_vec, current_vec, wave_amp: float, t: float) -> None:
        p, dt = self.p, self.dt
        uc = self.delay.push({"thrust_l": clamp(ucmd.get("thrust_l", 0.0)), "thrust_r": clamp(ucmd.get("thrust_r", 0.0))})

        # thruster dynamics (first order)
        tau_t = max(p["thrust_time_constant_s"], 1e-3)
        for i, key in enumerate(("thrust_l", "thrust_r")):
            target = self._thrust_cmd(uc[key])
            self.T[i] += (target - self.T[i]) * (1 - math.exp(-dt / tau_t))
        TL, TR = self.T
        by = p["thruster_offset_y_m"]
        X_thr = TL + TR
        N_thr = (TR - TL) * by                  # right thruster pushes bow left (+yaw)

        # wind: relative wind in body frame, quadratic
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        wrel_e = wind_vec[0] - (c * self.u - s * self.v)
        wrel_n = wind_vec[1] - (s * self.u + c * self.v)
        wb_x = c * wrel_e + s * wrel_n
        wb_y = -s * wrel_e + c * wrel_n
        q = 0.5 * 1.225 * p["wind_coeff"] * p["wind_area_m2"]
        X_w = q * wb_x * abs(wb_x) * 0.5       # frontal area ~ half of lateral
        Y_w = q * wb_y * abs(wb_y)
        N_w = -0.1 * self.p_common["length_m"] * Y_w   # wind tends to turn bow downwind (CoP astern)

        # waves: yaw disturbance + cosmetic heave/roll
        N_wave = 0.3 * wave_amp * self.Mr * math.sin(2 * math.pi * t / 2.7 + 0.3)
        self.z = wave_amp * math.sin(2 * math.pi * t / 2.5)
        self.roll = 0.3 * wave_amp * math.sin(2 * math.pi * t / 2.5 + 1.0)
        self.pitch = 0.15 * wave_amp * math.cos(2 * math.pi * t / 2.5)

        # hydrodynamics
        X_d = -(p["drag_surge_lin"] * self.u + p["drag_surge_quad"] * self.u * abs(self.u))
        Y_d = -(p["drag_sway_lin"] * self.v + p["drag_sway_quad"] * self.v * abs(self.v))
        N_d = -(p["drag_yaw_lin"] * self.r + p["drag_yaw_quad"] * self.r * abs(self.r))
        # Coriolis / centripetal (rigid body + added mass)
        X_c = self.Mv * self.v * self.r
        Y_c = -self.Mu * self.u * self.r
        N_c = (self.Mu - self.Mv) * self.u * self.v

        du = (X_thr + X_w + X_d + X_c) / self.Mu
        dv = (Y_w + Y_d + Y_c) / self.Mv
        dr = (N_thr + N_w + N_wave + N_d + N_c) / self.Mr
        self.ax, self.ay = du - self.v * self.r, dv + self.u * self.r
        self.u += du * dt
        self.v += dv * dt
        self.r += dr * dt

        # kinematics with current
        self.x += (c * self.u - s * self.v + current_vec[0]) * dt
        self.y += (s * self.u + c * self.v + current_vec[1]) * dt
        self.yaw = wrap(self.yaw + self.r * dt)
        self.u_applied = {"thrust_l": uc["thrust_l"], "thrust_r": uc["thrust_r"]}

    def half_width(self) -> float:
        return self.p_common["width_m"] / 2

    def extra(self) -> dict:
        return {"thrust": {"l": self.T[0], "r": self.T[1]}}


# --------------------------------------------------------------- backend

class MockBackend:
    """Implements the same verbs as the UDP bridge, in-process."""

    def __init__(self, spec: SceneSpec, vp: VehicleParams | None = None, dt: float = 0.02):
        self.spec = spec
        self.vp = vp or spec.vehicle()
        self.dt = dt
        self.rng = random.Random(spec.raw.get("seed", 0))
        self.world = SceneWorld(spec, spec.raw.get("seed", 0))
        self.is_boat = spec.scenario == "boat"
        self.veh = MockBoat(self.vp, dt, self.rng) if self.is_boat else MockCar(self.vp, dt, self.rng)
        self.sensors = SensorPack(self.vp, self.world, dt, self.rng, spec.geo_origin)
        self.tick = 0
        self.t = 0.0
        self.seq = -1
        self.env = self._env_from_spec()
        self.gust = Gust(self.env["wind"]["gust_std"], self.env["wind"]["gust_period"], self.rng)
        self.time_scale = 1.0
        self._u: dict[str, float] = {}
        self._collided_prev = False

    # --- env
    def _env_from_spec(self) -> dict:
        d = self.spec.disturbance
        w, c = d.get("wind", {}), d.get("current", {})
        wv = self.spec.water.get("waves", {})
        return {"wind": {"speed": float(w.get("speed_mps", 0)), "dir": float(w.get("dir_rad", 0)),
                         "gust_std": float(w.get("gust_std_mps", 0)), "gust_period": float(w.get("gust_period_s", 8))},
                "current": {"speed": float(c.get("speed_mps", 0)), "dir": float(c.get("dir_rad", 0))},
                "waves": {"amp": float(wv.get("amp_m", 0)), "period": float(wv.get("period_s", 3)), "dir": float(wv.get("dir_rad", 0))}}

    def set_env(self, **kw) -> dict:
        for k in ("wind", "current", "waves"):
            if k in kw and kw[k]:
                self.env[k].update({kk: float(v) for kk, v in kw[k].items()})
        if "time_scale" in kw:
            self.time_scale = float(kw["time_scale"])
        self.gust = Gust(self.env["wind"]["gust_std"], self.env["wind"]["gust_period"], self.rng)
        return {"ok": True}

    def set_params(self, params: dict | VehicleParams) -> dict:
        from .specs import load_vehicle
        vp = params if isinstance(params, VehicleParams) else load_vehicle(params)
        self.vp = vp
        self.veh.set_params(vp)
        self.sensors = SensorPack(vp, self.world, self.dt, self.rng, self.spec.geo_origin)
        return {"ok": True}

    # --- episode
    def reset(self, pose: dict | None = None, seed: int | None = None, clear_dynamic: bool = True) -> Observation:
        if seed is not None:
            self.rng = random.Random(seed)
            self.world.rebuild(seed) if "obstacle_layout" in (self.spec.randomization.get("vary") or []) else None
        if clear_dynamic:
            self.world.despawn("dynamic:*")
        self._apply_randomization()
        sp = self.spec.raw["spawn"]
        x, y = self.spec.spawn_xy
        yaw = self.spec.spawn_yaw
        if pose:
            x, y, yaw = float(pose.get("x", x)), float(pose.get("y", y)), float(pose.get("yaw", yaw))
        else:
            j, jy = float(sp.get("jitter_m", 0)), float(sp.get("jitter_yaw_rad", 0))
            x += self.rng.uniform(-j, j)
            y += self.rng.uniform(-j, j)
            yaw += self.rng.uniform(-jy, jy)
        self.veh.reset(x, y, yaw)
        self.t, self.seq = 0.0, -1
        self._u = {}
        self._collided_prev = False
        self.sensors._last_lidar_t = -1e9
        return self._observe()

    def _apply_randomization(self) -> None:
        r = self.spec.randomization
        if not r.get("enabled"):
            return
        vary, rg = set(r.get("vary", [])), r.get("ranges", {})

        def roll(k, default):
            lo, hi = rg.get(k, default)
            return self.rng.uniform(float(lo), float(hi))

        if "wind" in vary:
            self.env["wind"]["speed"] = roll("wind_speed_mps", [0, 5])
            self.env["wind"]["dir"] = roll("wind_dir_rad", [-math.pi, math.pi])
        if "current" in vary:
            self.env["current"]["speed"] = roll("current_speed_mps", [0, 0.5])
            self.env["current"]["dir"] = roll("current_dir_rad", [-math.pi, math.pi])
        if "waves" in vary:
            self.env["waves"]["amp"] = roll("wave_amp_m", [0, 0.2])
        if "friction" in vary:
            self.spec.raw["world"].setdefault("terrain", {}).setdefault("surface", {})["friction"] = roll("friction", [0.5, 0.95])
        if "actuator_delay" in vary:
            self.veh.set_params(self.vp.with_overrides(**{"common.actuator_delay_s": roll("actuator_delay_s", [0, 0.1])}))
        if "mass" in vary:
            m0 = self.vp.common["mass_kg"]
            self.veh.set_params(self.vp.with_overrides(**{"common.mass_kg": m0 * roll("mass_scale", [0.85, 1.15])}))
        self.gust = Gust(self.env["wind"]["gust_std"], self.env["wind"]["gust_period"], self.rng)

    def act(self, u: dict[str, float], seq: int | None = None) -> None:
        self._u = dict(u)
        self.seq = seq if seq is not None else self.seq + 1

    def step(self, u: dict[str, float] | None = None, seq: int | None = None) -> Observation:
        if u is not None:
            self.act(u, seq)
        wspd = max(self.env["wind"]["speed"] + self.gust.step(self.dt), 0.0)
        wd = self.env["wind"]["dir"]
        wind = (wspd * math.cos(wd), wspd * math.sin(wd))
        cs, cd = self.env["current"]["speed"], self.env["current"]["dir"]
        current = (cs * math.cos(cd), cs * math.sin(cd))
        if self.is_boat:
            self.veh.step(self._u, wind, current, self.env["waves"]["amp"], self.t)
        else:
            self.veh.step(self._u, self.world.friction())
        self.t += self.dt
        self.tick += 1
        return self._observe(wind, current)

    def _observe(self, wind=(0.0, 0.0), current=(0.0, 0.0)) -> Observation:
        v = self.veh
        d, label, n = self.world.obs.nearest_point(v.x, v.y)
        collided = d < v.half_width()
        if collided:
            # Fender-style contact: push back out to the surface and kill the
            # velocity component into it. The vehicle can lean on a wall or a
            # pier finger and slide along it, which is what real ones do; it
            # cannot clip through. A car gets stopped harder - cones are not
            # fenders.
            pen = v.half_width() - d
            v.x += n[0] * pen
            v.y += n[1] * pen
            if self.is_boat:
                c, s_ = math.cos(v.yaw), math.sin(v.yaw)
                ex, ey = c * v.u - s_ * v.v, s_ * v.u + c * v.v          # earth-frame velocity
                into = ex * n[0] + ey * n[1]
                if into < 0:
                    ex -= into * n[0]
                    ey -= into * n[1]
                    ex *= 0.7; ey *= 0.7
                v.u, v.v = c * ex + s_ * ey, -s_ * ex + c * ey
                v.r *= 0.7
            else:
                v.v = 0.0
        o = Observation(
            tick=self.tick, t=self.t, dt=self.dt, seq=self.seq,
            vehicle="boat" if self.is_boat else "car",
            pose=Pose(v.x + self.rng.gauss(0, self.sensors.pose_noise), v.y + self.rng.gauss(0, self.sensors.pose_noise),
                      getattr(v, "z", 0.0), getattr(v, "roll", 0.0), getattr(v, "pitch", 0.0), v.yaw),
            vel=Twist(vx=(v.u if self.is_boat else v.v), vy=(v.v if self.is_boat else 0.0),
                      wz=(v.r if self.is_boat else v.yaw_rate) + self.sensors.gyro_bias + self.rng.gauss(0, self.sensors.gyro_noise)),
            acc=(v.ax + self.rng.gauss(0, self.sensors.acc_noise), v.ay + self.rng.gauss(0, self.sensors.acc_noise), G),
            u_applied=dict(v.u_applied),
            lidar=self.sensors.lidar(self.t, v.x, v.y, v.yaw),
            gps=self.sensors.gps(self.t, v.x, v.y),
            speed=abs((math.hypot(v.u, v.v) if self.is_boat else v.v) + self.rng.gauss(0, self.sensors.speed_noise)),
            extra=v.extra(),
            collision=collided, contacts=[label] if collided else [],
            out_of_bounds=not self.world.in_bounds(v.x, v.y), capsized=False,
            env={"wind": {"speed": math.hypot(*wind), "dir": math.atan2(wind[1], wind[0])},
                 "current": {"speed": math.hypot(*current), "dir": math.atan2(current[1], current[0])}},
        )
        if not self.is_boat:
            radius = math.hypot(self.vp.common['length_m'], self.vp.common['width_m']) / 2
            o.extra['footprint_obstacle_clearance_m'] = d - radius
        return o

    # --- live edits
    def spawn(self, kind: str, label: str, x: float, y: float, z: float = 0.0, yaw: float = 0.0,
              scale=None, movable: bool = False) -> dict:
        if kind not in KIND_GEOM and not kind.startswith("custom:"):
            return {"ok": False, "error": f"unknown kind '{kind}'"}
        self.world.spawn(label, kind if kind in KIND_GEOM else "custom_mesh", x, y, yaw, scale)
        return {"ok": True, "label": label}

    def despawn(self, label: str) -> dict:
        return {"ok": True, "removed": self.world.despawn(label)}

    def scene(self) -> dict:
        acts = [{"label": c[3], "kind": "circle", "x": c[0], "y": c[1], "r": c[2]} for c in self.world.obs.circles]
        return {"spec_name": self.spec.name, "actors": acts, "task": self.spec.task,
                "dynamic": list(self.world.dynamic)}

    def close(self) -> None:
        pass
