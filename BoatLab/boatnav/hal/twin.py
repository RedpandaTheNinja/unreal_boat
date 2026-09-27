"""Python twin of the Unreal boat: same planar dynamics and motor model as the
BoatPhysics plugin, soft contacts with buoys/shore, synthetic camera, simulated GPS
noise and payload mechanisms. Runs faster than real time for tests."""
from __future__ import annotations

import collections
import math

import numpy as np

from .. import geometry as G
from ..evaluation import simulate_actuator
from ..geo import LocalFrame
from ..model import BoatModel
from ..types import BoatState, CameraFrame
from .base import BoatInterface
from .render import Renderer

PHYS_DT = 1.0 / 120.0


class TwinBoat(BoatInterface):
    name = "twin"
    is_sim = True

    def __init__(self, boat_cfg: dict, course: dict, seed: int = 0, realtime: bool = False,
                 camera: bool = True, gps_noise_m: float | None = None, start_pose=None, **_):
        self.cfg, self.course = boat_cfg, course
        self.model = BoatModel(boat_cfg)
        self.realtime = realtime
        self.rng = np.random.default_rng(seed)
        self.frame = LocalFrame.from_course(course)
        self.renderer = Renderer(boat_cfg["camera"], course, seed=seed) if camera else None
        g = boat_cfg.get("gps", {})
        self.gps_sigma = g.get("sim_noise_m", 0.02) if gps_noise_m is None else gps_noise_m
        self.head_sigma = math.radians(g.get("sim_heading_noise_deg", 0.5))
        self.length, self.beam = boat_cfg["hull"]["length_m"], boat_cfg["hull"]["beam_m"]
        self.current = np.zeros(2)
        self.wind = np.zeros(2)
        self.hidden: set[str] = set()
        self.extra_objects: list[dict] = []
        self.contacts: list[dict] = []
        self.actions: list[dict] = []
        self.cam_seq = 0
        self._start = tuple(start_pose or course["start_pose"])
        self.reset(self._start)

    # ------------------------------------------------------------------ control
    def reset(self, pose=None):
        x, y, h = pose or self._start
        self.x, self.y, self.h = float(x), float(y), float(h)
        self.u = self.v = self.r = 0.0
        self.t = 0.0
        self.tl = self.tr = 0.0
        self.cmd = (0.0, 0.0)
        self.last_cmd_t = -100.0
        self.queue = collections.deque()
        self.bias = np.zeros(3)
        self.hidden.clear()
        self.extra_objects.clear()
        self.contacts.clear()
        self.actions.clear()

    def command(self, left: float, right: float):
        left, right = float(np.clip(left, -1, 1)), float(np.clip(right, -1, 1))
        self.cmd = (left, right)
        self.last_cmd_t = self.t
        self.queue.append((self.t, left, right))

    def set_environment(self, current=(0.0, 0.0), wind=(0.0, 0.0), waves=None, fog_visibility_m=None):
        self.current = np.asarray(current, float)
        self.wind = np.asarray(wind, float)
        if self.renderer is not None:
            self.renderer.fog_visibility = fog_visibility_m
        return {"ok": True}

    # ------------------------------------------------------------------ physics
    def tick(self, dt: float):
        n = max(1, int(round(dt / PHYS_DT)))
        for _ in range(n):
            self._step(PHYS_DT)

    def _delayed_cmd(self):
        expired = self.t - self.last_cmd_t > self.model.timeout
        while len(self.queue) > 1 and self.queue[1][0] <= self.t - self.model.delay:
            self.queue.popleft()
        if expired or not self.queue or self.queue[0][0] > self.t - self.model.delay:
            return 0.0, 0.0, expired
        _, l, r = self.queue[0]
        return l, r, expired

    def _step(self, dt):
        m = self.model
        ul, ur, _ = self._delayed_cmd()
        a = 1 - math.exp(-dt / max(1e-3, m.tau))
        self.tl += (m.motor_curve(ul) - self.tl) * a
        self.tr += (m.motor_curve(ur) - self.tr) * a
        c, s = math.cos(self.h), math.sin(self.h)
        cur_b = (c * self.current[0] + s * self.current[1], -s * self.current[0] + c * self.current[1])
        wind_b = (c * self.wind[0] + s * self.wind[1], -s * self.wind[0] + c * self.wind[1])
        du, dv, dr = m.derivatives(self.u, self.v, self.r, self.tl, self.tr, cur_b, wind_b)
        # contacts (soft penalty forces) in body frame
        fcx, fcy, ncz = self._contact_forces()
        du += fcx / (m.mass + m.max_[0])
        dv += fcy / (m.mass + m.max_[1])
        dr += ncz / (m.iz + m.naz)
        self.u += du * dt
        self.v += dv * dt
        self.r += dr * dt
        self.h = G.wrap(self.h + self.r * dt)
        c, s = math.cos(self.h), math.sin(self.h)
        self.x += (c * self.u - s * self.v) * dt
        self.y += (s * self.u + c * self.v) * dt
        self.t += dt

    def _contact_forces(self):
        poly = G.hull_polygon(self.x, self.y, self.h, self.length, self.beam)
        fx = fy = n = 0.0
        k, cdamp = 4000.0, 500.0
        c, s = math.cos(self.h), math.sin(self.h)
        vx, vy = c * self.u - s * self.v, s * self.u + c * self.v
        for ob in self.course["objects"]:
            if ob["id"] in self.hidden or ob.get("kind") == "case":
                continue
            R = ob.get("radius", 0.15)
            cxy = np.array([ob["x"], ob["y"]])
            if math.hypot(cxy[0] - self.x, cxy[1] - self.y) > self.length / 2 + R + 0.2:
                continue
            best = None
            for i in range(len(poly)):
                a_, b_ = poly[i], poly[(i + 1) % len(poly)]
                ab = b_ - a_
                t = float(np.clip(np.dot(cxy - a_, ab) / max(np.dot(ab, ab), 1e-12), 0, 1))
                q = a_ + t * ab
                d = float(np.linalg.norm(cxy - q))
                if best is None or d < best[0]:
                    best = (d, q)
            inside = G.point_in_polygon(cxy, poly)
            d, q = best
            pen = R + d if inside else R - d
            if pen <= 0:
                continue
            nvec = (q - cxy) / max(d, 1e-6)
            if inside:
                nvec = -nvec
            rx, ry = q[0] - self.x, q[1] - self.y
            pvx, pvy = vx - self.r * ry, vy + self.r * rx
            vn = pvx * nvec[0] + pvy * nvec[1]
            f = max(0.0, k * pen - cdamp * vn)
            Fx, Fy = f * nvec[0], f * nvec[1]
            fx += Fx
            fy += Fy
            n += rx * Fy - ry * Fx
            if not self.contacts or self.contacts[-1]["id"] != ob["id"] or self.t - self.contacts[-1]["t"] > 1.0:
                self.contacts.append({"id": ob["id"], "t": self.t, "speed": math.hypot(vx, vy)})
            else:
                self.contacts[-1]["t"] = self.t
        wb = self.course["water"]
        for px, py in poly:
            for val, axis, sign in ((wb["xmin"], 0, 1), (wb["xmax"], 0, -1), (wb["ymin"], 1, 1), (wb["ymax"], 1, -1)):
                p = (px, py)[axis]
                pen = (val - p) * sign
                if pen > 0:
                    F = [0.0, 0.0]
                    F[axis] = sign * k * pen
                    fx += F[0]
                    fy += F[1]
                    n += (px - self.x) * F[1] - (py - self.y) * F[0]
        # world -> body
        return c * fx + s * fy, -s * fx + c * fy, n

    # ------------------------------------------------------------------ sensing
    def state(self) -> BoatState:
        # GPS: slowly wandering bias (sigma, 120 s correlation) + small white jitter (0.1 sigma), like real receivers
        tau = 120.0
        self.bias[:2] += (-self.bias[:2] / tau) * 0.05 + self.rng.normal(0, self.gps_sigma * math.sqrt(2 * 0.05 / tau), 2)
        nx = self.x + self.bias[0] + self.rng.normal(0, self.gps_sigma * 0.1)
        ny = self.y + self.bias[1] + self.rng.normal(0, self.gps_sigma * 0.1)
        nh = G.wrap(self.h + self.rng.normal(0, self.head_sigma))
        lat, lon = self.frame.to_geo(nx, ny)
        return BoatState(self.t, nx, ny, nh, self.u, self.v, self.r, lat, lon, self.tl, self.tr, True,
                         f"twin_truth+gps({self.gps_sigma:.2f}m)")

    def camera(self) -> CameraFrame | None:
        if self.renderer is None:
            return None
        pose = (self.x, self.y, self.h)
        rgb, depth = self.renderer(pose, hidden=self.hidden, extra_objects=self.extra_objects)
        self.cam_seq += 1
        r = self.renderer
        return CameraFrame(self.t, rgb, depth if self.cfg["camera"].get("depth", True) else None, r.fx, r.fy, r.cx, r.cy,
                           tuple(self.cfg["camera"]["mount_flu_m"]), r.pitch, pose, self.cam_seq, "twin_render")

    def truth(self) -> dict:
        return {"t": self.t, "x": self.x, "y": self.y, "heading": self.h, "u": self.u, "v": self.v, "r": self.r,
                "left_thrust_n": self.tl, "right_thrust_n": self.tr, "hidden": sorted(self.hidden),
                "contacts": list(self.contacts[-20:])}

    def actuate(self, name: str, **kw) -> dict:
        res = simulate_actuator(name, (self.x, self.y, self.h), self.course, self.cfg, self.rng, hidden=self.hidden)
        res["t"] = self.t
        if res.get("hide"):
            self.hidden.add(res["hide"])
        self.actions.append(res)
        return res
