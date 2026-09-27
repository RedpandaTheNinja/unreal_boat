"""Synthetic pinhole camera for the Python twin: ray-casts the course (buoys, target, case,
detectors, dock, shoreline) over flat water and returns RGB + metric depth.

It exists so the vision pipeline can be exercised end-to-end without Unreal. It is not
photorealistic; tune HSV thresholds on Unreal and real images with tools/tune_colors.py.
"""
from __future__ import annotations

import math

import numpy as np

HULL_ORIGIN_Z = 0.125     # UE hull origin height above calm water at rest (from boat telemetry)

COLORS = {
    "red": (205, 32, 38), "green": (95, 205, 60), "blue": (30, 95, 205), "orange": (245, 125, 30),
    "purple": (135, 45, 150), "yellow": (240, 212, 35), "black": (26, 26, 30), "white": (236, 236, 236),
    "pink": (245, 110, 195), "case": (250, 196, 10), "target": (245, 100, 25), "dock": (120, 90, 60),
    "cap": (22, 22, 24), "boat": (185, 190, 198),
}
SKY_TOP, SKY_HORIZON = np.array([120, 165, 225.0]), np.array([200, 215, 230.0])
WATER_NEAR, WATER_FAR = np.array([28, 70, 78.0]), np.array([70, 110, 118.0])
SHORE = np.array([58, 82, 48.0])
SUN = np.array([0.4, -0.3, 0.87])
SUN = SUN / np.linalg.norm(SUN)


def camera_rays(width, height, hfov_deg):
    fx = width / (2 * math.tan(math.radians(hfov_deg) / 2))
    fy = fx
    cx, cy = width / 2, height / 2
    u, v = np.meshgrid(np.arange(width) + 0.5, np.arange(height) + 0.5)
    return fx, fy, cx, cy, (u - cx) / fx, (v - cy) / fy


class Renderer:
    def __init__(self, cam_cfg: dict, course: dict, seed: int = 0):
        self.w, self.h = int(cam_cfg["width"]), int(cam_cfg["height"])
        self.fx, self.fy, self.cx, self.cy, self.xn, self.yn = camera_rays(self.w, self.h, cam_cfg["hfov_deg"])
        self.mount = cam_cfg["mount_flu_m"]
        self.pitch = math.radians(cam_cfg.get("pitch_down_deg", 0.0))
        self.water = course["water"]
        self.course = course
        self.rng = np.random.default_rng(seed)
        self.fog_visibility = None

    # -------------------------------------------------------------- geometry
    def _pose(self, x, y, h):
        """Camera origin (world) and the world direction of each pixel ray (optical z = 1)."""
        c, s = math.cos(h), math.sin(h)
        mx, my, mz = self.mount
        o = np.array([x + c * mx - s * my, y + s * mx + c * my, HULL_ORIGIN_Z + mz])
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        # optical -> body FLU: forward = z, left = -x, up = -y ; then pitch down
        fwd = np.array([cp, 0.0, -sp])
        left = np.array([0.0, 1.0, 0.0])
        up = np.array([sp, 0.0, cp])
        dir_b = fwd[None, None, :] - self.xn[..., None] * left[None, None, :] - self.yn[..., None] * up[None, None, :]
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
        return o, dir_b @ R.T

    def project(self, pose, pts):
        """World points (N,3) -> pixel (u, v, depth). Used by tools and tests."""
        x, y, h = pose
        c, s = math.cos(h), math.sin(h)
        mx, my, mz = self.mount
        o = np.array([x + c * mx - s * my, y + s * mx + c * my, HULL_ORIGIN_Z + mz])
        d = np.asarray(pts, float) - o
        bx = c * d[:, 0] + s * d[:, 1]
        by = -s * d[:, 0] + c * d[:, 1]
        bz = d[:, 2]
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        zc = cp * bx - sp * bz
        up = sp * bx + cp * bz
        u = self.cx + self.fx * (-by) / zc
        v = self.cy + self.fy * (-up) / zc
        return u, v, zc

    # -------------------------------------------------------------- render
    def render(self, pose, hidden=(), extra_objects=()):
        self._last_pose = pose
        o, d = self._pose(*pose)
        H, W = self.h, self.w
        depth = np.full((H, W), np.inf)
        rgb = np.zeros((H, W, 3))
        # sky gradient
        elev = d[..., 2] / np.linalg.norm(d, axis=-1)
        k = np.clip(elev * 4, 0, 1)[..., None]
        rgb[:] = SKY_HORIZON * (1 - k) + SKY_TOP * k
        # water plane z = 0
        wmask = d[..., 2] < -1e-6
        tw = np.where(wmask, -o[2] / np.where(wmask, d[..., 2], -1), np.inf)
        px = o[0] + tw * d[..., 0]
        py = o[1] + tw * d[..., 1]
        wb = self.water
        inside = wmask & (px >= wb["xmin"]) & (px <= wb["xmax"]) & (py >= wb["ymin"]) & (py <= wb["ymax"])
        dist = np.clip(tw / 60.0, 0, 1)[..., None]
        wcol = WATER_NEAR * (1 - dist) + WATER_FAR * dist
        ripple = 6 * np.sin(px * 3.1 + py * 1.7)[..., None] + 4 * np.sin(px * 0.9 - py * 4.3)[..., None]
        rgb = np.where(inside[..., None], wcol + ripple, rgb)
        depth = np.where(inside, tw, depth)
        # shoreline walls (tree line) at the water bounds, 5 m tall
        for axis, val in ((0, wb["xmin"]), (0, wb["xmax"]), (1, wb["ymin"]), (1, wb["ymax"])):
            dd = d[..., axis]
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (val - o[axis]) / dd
            z = o[2] + t * d[..., 2]
            other = o[1 - axis] + t * d[..., 1 - axis]
            lo, hi = (wb["ymin"], wb["ymax"]) if axis == 0 else (wb["xmin"], wb["xmax"])
            m = (t > 0) & (t < depth) & (z >= -0.5) & (z <= 5.0) & (other >= lo - 30) & (other <= hi + 30)
            shade = (0.75 + 0.25 * np.sin(other * 0.7))[..., None]
            rgb = np.where(m[..., None], SHORE * shade, rgb)
            depth = np.where(m, t, depth)
        # objects
        objs = [ob for ob in self.course["objects"] if ob["id"] not in hidden] + list(extra_objects)
        for ob in self._with_dock(objs):
            rgb, depth = self._draw(ob, o, d, rgb, depth)
        # fog
        if self.fog_visibility:
            f = np.exp(-np.minimum(depth, 500) / self.fog_visibility)[..., None]
            rgb = rgb * f + np.array([205, 208, 210.0]) * (1 - f)
        rgb = rgb + self.rng.normal(0, 3.0, rgb.shape)
        img = np.clip(rgb, 0, 255).astype(np.uint8)
        # depth along optical axis == ray parameter because optical z of each ray is 1
        return img, np.where(np.isfinite(depth), depth, 0.0).astype(np.float32)

    def _with_dock(self, objs):
        out = list(objs)
        for k in self.course.get("keepout", []):
            poly = np.array(k["polygon"])
            cxy = poly.mean(axis=0)
            size = poly.max(axis=0) - poly.min(axis=0)
            color = "boat" if k.get("style") == "boat" else "dock"
            out.append(dict(id=k["id"], kind="box", color=color, x=cxy[0], y=cxy[1],
                            size=[size[0], size[1], k.get("height_m", 0.45)], yaw=0.0))
        return out

    def _bbox_mask(self, ob, o):
        """Pixel window that can contain the object (speeds up ray casting)."""
        r = float(ob.get("radius", 0.3)) + 0.3
        hgt = float(ob.get("height", 0.6)) + 0.2
        if ob.get("kind") in ("box", "case"):
            sz = ob.get("size", [0.5, 0.4, 0.2])
            r = 0.5 * math.hypot(sz[0], sz[1]) + 0.2
            hgt = sz[2] + 0.2
        pts = []
        for ang in np.linspace(0, 2 * math.pi, 12, endpoint=False):
            for z in (-0.05, hgt):
                pts.append([ob["x"] + r * math.cos(ang), ob["y"] + r * math.sin(ang), z])
        return np.array(pts)

    def _draw(self, ob, o, d, rgb, depth):
        if math.hypot(ob["x"] - o[0], ob["y"] - o[1]) > 60.0:
            return rgb, depth
        pts = self._bbox_mask(ob, o)
        pose = self._last_pose
        u, v, zc = self.project(pose, pts)
        if (zc <= 0.05).all():
            return rgb, depth
        vis = zc > 0.05
        if not vis.all():
            u0, u1, v0, v1 = 0, self.w, 0, self.h
        else:
            u0, u1 = int(max(0, math.floor(u.min()) - 2)), int(min(self.w, math.ceil(u.max()) + 2))
            v0, v1 = int(max(0, math.floor(v.min()) - 2)), int(min(self.h, math.ceil(v.max()) + 2))
        if u0 >= u1 or v0 >= v1:
            return rgb, depth
        dd = d[v0:v1, u0:u1]
        kind = ob.get("kind", "buoy")
        if kind in ("box", "case"):
            t, col = self._box(ob, o, dd)
        else:
            t, col = self._cylinder(ob, o, dd)
        sub_d = depth[v0:v1, u0:u1]
        m = t < sub_d
        if m.any():
            rgb[v0:v1, u0:u1][m] = col[m]
            sub_d[m] = t[m]
        return rgb, depth

    def _cylinder(self, ob, o, d):
        R = float(ob.get("radius", 0.135))
        h = float(ob.get("height", 0.55))
        cx, cy = ob["x"], ob["y"]
        ox, oy = o[0] - cx, o[1] - cy
        dx, dy, dz = d[..., 0], d[..., 1], d[..., 2]
        a = dx * dx + dy * dy
        b = 2 * (ox * dx + oy * dy)
        c = ox * ox + oy * oy - R * R
        disc = b * b - 4 * a * c
        t_side = np.full(a.shape, np.inf)
        ok = disc >= 0
        sq = np.sqrt(np.where(ok, disc, 0))
        t1 = (-b - sq) / (2 * np.where(a > 1e-12, a, 1e-12))
        z1 = o[2] + t1 * dz
        side_hit = ok & (t1 > 0) & (z1 >= 0) & (z1 <= h)
        t_side = np.where(side_hit, t1, np.inf)
        # top cap
        with np.errstate(divide="ignore", invalid="ignore"):
            tc = (h - o[2]) / dz
        capx, capy = ox + tc * dx, oy + tc * dy
        cap_hit = (tc > 0) & (capx * capx + capy * capy <= R * R)
        t_cap = np.where(cap_hit, tc, np.inf)
        t = np.minimum(t_side, t_cap)
        color = ob.get("color", "red")
        base = np.array(COLORS.get(color, COLORS["white" if color == "zebra" else "red"]), float)
        if ob.get("kind") == "target":
            base = np.array(COLORS["target"], float)
        col = np.broadcast_to(base, d.shape).copy()
        # lambert shading on the side using the surface normal
        hx, hy = ox + t_side * dx, oy + t_side * dy
        nrm = np.stack([hx / R, hy / R, np.zeros_like(hx)], -1)
        lam = np.clip((nrm @ SUN), 0, 1)[..., None]
        col = np.where(np.isfinite(t_side)[..., None], col * (0.55 + 0.45 * lam), col)
        zz = o[2] + t_side * dz
        kind = ob.get("kind", "buoy")
        if kind == "buoy":
            # black fender ends (top 15 %) and dark cap
            capband = np.isfinite(t_side) & (zz > h * 0.85)
            col = np.where(capband[..., None], np.array(COLORS["cap"], float), col)
            col = np.where(np.isfinite(t_cap)[..., None] & (t_cap < t_side)[..., None], np.array(COLORS["cap"], float), col)
            if color == "zebra":
                ang = np.arctan2(hy, hx)
                stripe = np.sin(zz * 2 * math.pi / 0.18 + ang * 1.0) > 0.45
                col = np.where((np.isfinite(t_side) & stripe & (zz <= h * 0.85))[..., None],
                               np.array(COLORS["black"], float), col)
        elif kind == "target":
            rr = np.sqrt(capx * capx + capy * capy)
            center = np.isfinite(t_cap) & (t_cap <= t_side) & (rr < 0.55 * R)
            col = np.where(center[..., None], np.array([30, 30, 34.0]), col)
        return t, col

    def _box(self, ob, o, d):
        sx, sy, sz = ob.get("size", [0.45, 0.35, 0.18])
        yaw = float(ob.get("yaw", 0.0))
        c, s = math.cos(yaw), math.sin(yaw)
        rx, ry = o[0] - ob["x"], o[1] - ob["y"]
        lox, loy = c * rx + s * ry, -s * rx + c * ry
        ldx = c * d[..., 0] + s * d[..., 1]
        ldy = -s * d[..., 0] + c * d[..., 1]
        ldz = d[..., 2]
        tmin = np.full(ldx.shape, -np.inf)
        tmax = np.full(ldx.shape, np.inf)
        for oo, dd, lo, hi in ((lox, ldx, -sx / 2, sx / 2), (loy, ldy, -sy / 2, sy / 2), (o[2], ldz, 0.0, sz)):
            with np.errstate(divide="ignore", invalid="ignore"):
                ta, tb = (lo - oo) / dd, (hi - oo) / dd
            t0, t1 = np.minimum(ta, tb), np.maximum(ta, tb)
            par = np.abs(dd) < 1e-12
            inside = (oo >= lo) & (oo <= hi)
            t0 = np.where(par, np.where(inside, -np.inf, np.inf), t0)
            t1 = np.where(par, np.where(inside, np.inf, -np.inf), t1)
            tmin, tmax = np.maximum(tmin, t0), np.minimum(tmax, t1)
        hit = (tmax >= tmin) & (tmin > 0)
        t = np.where(hit, tmin, np.inf)
        base = np.array(COLORS.get(ob.get("color", "case") if ob.get("kind") != "case" else "case"), float)
        zz = o[2] + t * ldz
        shade = np.where(zz > sz - 0.01, 1.0, 0.8)[..., None]
        return t, np.broadcast_to(base, d.shape) * shade

    def __call__(self, pose, hidden=(), extra_objects=()):
        with np.errstate(all="ignore"):
            return self.render(pose, hidden, extra_objects)
