"""
2-D geometry shared by the mock simulator, the task evaluator and the controllers.
Everything is numpy, metres, ENU.
"""

from __future__ import annotations

import math

import numpy as np


# ----------------------------------------------------------------- polylines

class Polyline:
    """A path with arc-length parameterisation. Closed or open."""

    def __init__(self, pts, closed: bool = False):
        p = np.asarray(pts, dtype=float).reshape(-1, 2)
        if closed and len(p) > 1 and np.linalg.norm(p[0] - p[-1]) > 1e-9:
            p = np.vstack([p, p[:1]])
        self.pts = p
        self.closed = closed
        seg = np.diff(p, axis=0)
        self.seg_len = np.linalg.norm(seg, axis=1)
        self.seg_dir = seg / np.maximum(self.seg_len[:, None], 1e-9)
        self.s = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.length = float(self.s[-1])

    def project(self, xy) -> tuple[float, float, int]:
        """Returns (arc_length_s, signed_cross_track, segment_index).
        Cross-track is positive to the LEFT of travel direction."""
        q = np.asarray(xy, dtype=float)
        a = self.pts[:-1]
        d = self.seg_dir
        t = np.einsum("ij,ij->i", q - a, d)
        t = np.clip(t, 0.0, self.seg_len)
        foot = a + t[:, None] * d
        dist = np.linalg.norm(foot - q, axis=1)
        i = int(np.argmin(dist))
        side = float(np.sign(d[i, 0] * (q[1] - foot[i, 1]) - d[i, 1] * (q[0] - foot[i, 0])))
        return float(self.s[i] + t[i]), float(dist[i] * (side if side != 0 else 1.0)), i

    def point_at(self, s: float) -> np.ndarray:
        if self.closed:
            s = s % self.length
        s = min(max(s, 0.0), self.length)
        i = int(np.searchsorted(self.s, s, side="right") - 1)
        i = min(max(i, 0), len(self.seg_len) - 1)
        return self.pts[i] + self.seg_dir[i] * (s - self.s[i])

    def project_near(self, xy, previous_s: float, window: float) -> tuple[float, float, int]:
        """Project inside a reachable arc-length window, preserving crossing branches.

        Returned progress is unwrapped across lap boundaries. A teleport is not
        silently matched to an unrelated branch; its distance remains large.
        """
        q = np.asarray(xy, float)
        starts = self.s[:-1].copy()
        if self.closed:
            mid = starts + self.seg_len / 2
            starts += np.round((previous_s - mid) / self.length) * self.length
        lo = np.maximum(0.0, previous_s - window - starts)
        hi = np.minimum(self.seg_len, previous_s + window - starts)
        valid = lo <= hi
        t = np.einsum('ij,ij->i', q - self.pts[:-1], self.seg_dir)
        t = np.maximum(lo, np.minimum(hi, t))
        foot = self.pts[:-1] + t[:, None] * self.seg_dir
        dist = np.linalg.norm(foot - q, axis=1)
        dist[~valid] = np.inf
        i = int(np.argmin(dist))
        if not np.isfinite(dist[i]):
            raise ValueError('Progress window does not intersect the route')
        side = np.sign(self.seg_dir[i, 0] * (q[1] - foot[i, 1]) -
                       self.seg_dir[i, 1] * (q[0] - foot[i, 0]))
        return float(starts[i] + t[i]), float(dist[i] * (side or 1)), i

    def heading_at(self, s: float) -> float:
        if self.closed:
            s = s % self.length
        i = int(np.searchsorted(self.s, min(max(s, 0.0), self.length), side="right") - 1)
        i = min(max(i, 0), len(self.seg_len) - 1)
        return float(math.atan2(self.seg_dir[i, 1], self.seg_dir[i, 0]))

    def lookahead(self, xy, dist: float) -> np.ndarray:
        s, _, _ = self.project(xy)
        return self.point_at(s + dist)

    def curvature_at(self, s: float, ds: float = 0.5) -> float:
        h0 = self.heading_at(s - ds)
        h1 = self.heading_at(s + ds)
        return float(((h1 - h0 + math.pi) % (2 * math.pi) - math.pi) / (2 * ds))


# ---------------------------------------------------------------- gates

def crossed_gate(p_prev, p_now, left, right) -> bool:
    """True if the segment p_prev->p_now crosses the gate segment left->right
    in the forward direction (gate normal = rotate(left->right, -90deg))."""
    p0, p1 = np.asarray(p_prev, float), np.asarray(p_now, float)
    a, b = np.asarray(left, float), np.asarray(right, float)
    return _segments_intersect(p0, p1, a, b) and _forward(p0, p1, a, b)


def _forward(p0, p1, a, b) -> bool:
    d = b - a                            # left buoy -> right buoy, i.e. pointing to starboard
    n = np.array([-d[1], d[0]])          # rotate +90 deg: forward through the gate
    return float(np.dot(p1 - p0, n)) > 0


def _segments_intersect(p, p2, q, q2) -> bool:
    def orient(a, b, c):
        v = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        return 0 if abs(v) < 1e-12 else (1 if v > 0 else -1)
    o1, o2, o3, o4 = orient(p, p2, q), orient(p, p2, q2), orient(q, q2, p), orient(q, q2, p2)
    return o1 != o2 and o3 != o4


# ------------------------------------------------------------- ray casting

class Obstacles:
    """Circles (x, y, r) and segments (x0, y0, x1, y1) for the mock lidar and collision."""

    def __init__(self):
        self.circles: list[tuple[float, float, float, str]] = []
        self.segments: list[tuple[float, float, float, float, str]] = []

    def add_circle(self, x, y, r, label="obstacle"):
        self.circles.append((float(x), float(y), float(r), label))

    def add_polygon(self, pts, label="obstacle"):
        p = np.asarray(pts, float)
        for i in range(len(p)):
            a, b = p[i], p[(i + 1) % len(p)]
            self.segments.append((a[0], a[1], b[0], b[1], label))

    def add_box(self, cx, cy, yaw, sx, sy, label="obstacle"):
        c, s = math.cos(yaw), math.sin(yaw)
        hx, hy = sx / 2, sy / 2
        corners = [(hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy)]
        self.add_polygon([(cx + c * x - s * y, cy + s * x + c * y) for x, y in corners], label)

    def remove(self, label: str) -> int:
        n0 = len(self.circles) + len(self.segments)
        self.circles = [c for c in self.circles if c[3] != label]
        self.segments = [s for s in self.segments if s[4] != label]
        return n0 - len(self.circles) - len(self.segments)

    def clear(self):
        self.circles.clear()
        self.segments.clear()

    # --- queries
    def raycast(self, ox: float, oy: float, angles: np.ndarray, range_max: float) -> np.ndarray:
        """Vectorised over angles. Returns ranges (range_max where nothing hit)."""
        dx, dy = np.cos(angles), np.sin(angles)
        best = np.full(len(angles), range_max, dtype=float)

        for (cx, cy, r, _) in self.circles:
            fx, fy = ox - cx, oy - cy
            b = 2 * (fx * dx + fy * dy)
            c = fx * fx + fy * fy - r * r
            disc = b * b - 4 * c
            ok = disc >= 0
            sq = np.sqrt(np.where(ok, disc, 0))
            t1 = (-b - sq) / 2
            t2 = (-b + sq) / 2
            t = np.where(t1 > 1e-6, t1, np.where(t2 > 1e-6, t2, np.inf))
            t = np.where(ok, t, np.inf)
            best = np.minimum(best, t)

        for (x0, y0, x1, y1, _) in self.segments:
            ex, ey = x1 - x0, y1 - y0
            denom = dx * ey - dy * ex
            ok = np.abs(denom) > 1e-12
            den = np.where(ok, denom, 1.0)
            t = ((x0 - ox) * ey - (y0 - oy) * ex) / den        # along ray
            u = ((x0 - ox) * dy - (y0 - oy) * dx) / den        # along segment
            hit = ok & (t > 1e-6) & (u >= 0) & (u <= 1)
            best = np.minimum(best, np.where(hit, t, np.inf))

        return np.minimum(best, range_max)

    def nearest(self, x: float, y: float) -> tuple[float, str]:
        """Distance to nearest obstacle surface and its label."""
        d, lab, _ = self.nearest_point(x, y)
        return d, lab

    def nearest_point(self, x: float, y: float) -> tuple[float, str, tuple[float, float]]:
        """(distance, label, outward unit normal at the closest surface point)."""
        best, lab, normal = math.inf, "", (0.0, 0.0)
        for (cx, cy, r, l) in self.circles:
            dx, dy = x - cx, y - cy
            dist = math.hypot(dx, dy)
            d = dist - r
            if d < best:
                best, lab = d, l
                normal = (dx / dist, dy / dist) if dist > 1e-9 else (1.0, 0.0)
        for (x0, y0, x1, y1, l) in self.segments:
            d, (qx, qy) = _point_seg(x, y, x0, y0, x1, y1)
            if d < best:
                best, lab = d, l
                normal = ((x - qx) / d, (y - qy) / d) if d > 1e-9 else (-(y1 - y0), (x1 - x0))
        return best, lab, normal


def _point_seg(px, py, x0, y0, x1, y1) -> tuple[float, tuple[float, float]]:
    ex, ey = x1 - x0, y1 - y0
    L2 = ex * ex + ey * ey
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((px - x0) * ex + (py - y0) * ey) / L2))
    qx, qy = x0 + t * ex, y0 + t * ey
    return math.hypot(px - qx, py - qy), (qx, qy)


def _point_seg_dist(px, py, x0, y0, x1, y1) -> float:
    return _point_seg(px, py, x0, y0, x1, y1)[0]


def point_in_polygon(x, y, poly) -> bool:
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi:
            inside = not inside
        j = i
    return inside
