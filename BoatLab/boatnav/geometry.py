"""2D geometry: angles, frames, hull footprint, paths with progress-window projection."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def body_to_world(x: float, y: float, h: float, bx: float, by: float) -> tuple[float, float]:
    c, s = math.cos(h), math.sin(h)
    return x + c * bx - s * by, y + s * bx + c * by


def world_to_body(x: float, y: float, h: float, wx: float, wy: float) -> tuple[float, float]:
    c, s = math.cos(h), math.sin(h)
    dx, dy = wx - x, wy - y
    return c * dx + s * dy, -s * dx + c * dy


# Outline of the UE hull SM_BoatHull_13x3ft (BoatDynamicsComponent::BuildCells), normalised.
_HULL = [(1.9812, 0), (1.50, .32), (.8, .4572), (-1.50, .4572), (-1.9812, .4),
         (-1.9812, -.4), (-1.5, -.4572), (.8, -.4572), (1.5, -.32)]


def hull_polygon(x: float, y: float, h: float, length: float = 3.9624, beam: float = 0.9144) -> np.ndarray:
    sx, sy = length / 3.9624, beam / 0.9144
    return np.array([body_to_world(x, y, h, px * sx, py * sy) for px, py in _HULL])


def point_segment_distance(p, a, b) -> float:
    p, a, b = (np.asarray(v, float) for v in (p, a, b))
    ab = b - a
    t = 0.0 if not ab.any() else float(np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0, 1))
    return float(np.linalg.norm(p - (a + t * ab)))


def point_in_polygon(p, poly) -> bool:
    x, y = p
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-300) + x1:
            inside = not inside
    return inside


def polygon_distance(p, poly) -> float:
    """Distance from point to polygon boundary; 0 when inside."""
    if point_in_polygon(p, poly):
        return 0.0
    return min(point_segment_distance(p, poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly)))


def segments_intersect(p1, p2, q1, q2) -> bool:
    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    d1, d2 = orient(q1, q2, p1), orient(q1, q2, p2)
    d3, d4 = orient(p1, p2, q1), orient(p1, p2, q2)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def side_of_line(a, b, p) -> float:
    """>0 when p is left of the directed line a->b."""
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])


@dataclass
class Projection:
    s: float            # arc length of the closest point
    index: int          # segment index
    point: tuple        # closest point
    lateral: float      # signed cross-track error, + = boat left of path
    tangent: float      # path heading at the closest point


class Path:
    """Polyline with per-vertex speed targets and arc-length parameterisation."""

    def __init__(self, points, speeds=None, name: str = "", reverse: bool = False):
        pts = np.asarray(points, float).reshape(-1, 2)
        keep = [0]
        for i in range(1, len(pts)):
            if np.linalg.norm(pts[i] - pts[keep[-1]]) > 1e-6:
                keep.append(i)
        self.points = pts[keep]
        if len(self.points) == 1:
            self.points = np.vstack([self.points, self.points + [1e-3, 0]])
        seg = np.diff(self.points, axis=0)
        self.seg_len = np.hypot(seg[:, 0], seg[:, 1])
        self.s = np.concatenate([[0], np.cumsum(self.seg_len)])
        self.length = float(self.s[-1])
        if speeds is None:
            speeds = 1.0
        sp = np.broadcast_to(np.asarray(speeds, float), (len(pts),)).copy()
        self.speeds = sp[keep] if len(sp) == len(pts) else np.full(len(self.points), float(sp[0]))
        self.name = name
        self.reverse = reverse

    def point_at(self, s: float) -> np.ndarray:
        s = float(np.clip(s, 0, self.length))
        i = int(np.clip(np.searchsorted(self.s, s, side="right") - 1, 0, len(self.seg_len) - 1))
        t = 0.0 if self.seg_len[i] == 0 else (s - self.s[i]) / self.seg_len[i]
        return self.points[i] + t * (self.points[i + 1] - self.points[i])

    def heading_at(self, s: float) -> float:
        s = float(np.clip(s, 0, self.length))
        i = int(np.clip(np.searchsorted(self.s, s, side="right") - 1, 0, len(self.seg_len) - 1))
        d = self.points[i + 1] - self.points[i]
        return math.atan2(d[1], d[0])

    def speed_at(self, s: float) -> float:
        return float(np.interp(np.clip(s, 0, self.length), self.s, self.speeds))

    def project(self, p, s_hint: float | None = None, window: float = 8.0) -> Projection:
        """Closest point, searched only within [s_hint - 1, s_hint + window] when a hint is given.
        The window prevents jumping to another branch where the path passes near itself (slalom, U-turns)."""
        p = np.asarray(p, float)
        best = None
        for i in range(len(self.seg_len)):
            if s_hint is not None and (self.s[i + 1] < s_hint - 1.0 or self.s[i] > s_hint + window):
                continue
            a, b = self.points[i], self.points[i + 1]
            ab = b - a
            L2 = float(np.dot(ab, ab))
            t = 0.0 if L2 == 0 else float(np.clip(np.dot(p - a, ab) / L2, 0, 1))
            q = a + t * ab
            d = float(np.linalg.norm(p - q))
            if best is None or d < best[0]:
                best = (d, i, t, q)
        if best is None:
            return self.project(p, None)
        d, i, t, q = best
        tan = math.atan2(*(self.points[i + 1] - self.points[i])[::-1])
        lat = side_of_line(self.points[i], self.points[i + 1], p)
        lat = d if lat > 0 else -d
        return Projection(float(self.s[i] + t * self.seg_len[i]), i, (float(q[0]), float(q[1])), lat, tan)

    def resampled(self, step: float = 0.5) -> "Path":
        n = max(2, int(math.ceil(self.length / step)) + 1)
        ss = np.linspace(0, self.length, n)
        pts = np.array([self.point_at(s) for s in ss])
        return Path(pts, np.interp(ss, self.s, self.speeds), self.name, self.reverse)

    def with_stop_profile(self, decel: float, v_end: float = 0.0) -> "Path":
        """Cap speeds so the boat can slow to v_end at the path end: v <= sqrt(v_end^2 + 2 a d)."""
        rp = self.resampled(0.4)
        remaining = rp.length - rp.s
        cap = np.sqrt(v_end ** 2 + 2 * decel * np.maximum(remaining, 0))
        rp.speeds = np.minimum(rp.speeds, np.maximum(cap, v_end))
        return rp

    def to_list(self, step: float = 0.5) -> list:
        rp = self.resampled(step) if self.length > step else self
        return [[round(float(x), 3), round(float(y), 3)] for x, y in rp.points]


def smooth_corners(points, radius: float = 2.0, samples: int = 6) -> np.ndarray:
    """Round polyline corners with quadratic Bezier fillets (keeps endpoints)."""
    pts = np.asarray(points, float)
    if len(pts) < 3:
        return pts
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        a, b, c = pts[i - 1], pts[i], pts[i + 1]
        da, dc = a - b, c - b
        la, lc = np.linalg.norm(da), np.linalg.norm(dc)
        if la < 1e-6 or lc < 1e-6:
            continue
        r = min(radius, la / 2, lc / 2)
        p0, p2 = b + da / la * r, b + dc / lc * r
        for t in np.linspace(0, 1, samples):
            out.append((1 - t) ** 2 * p0 + 2 * (1 - t) * t * b + t ** 2 * p2)
    out.append(pts[-1])
    return np.array(out)


def catmull_rom(points, samples_per_seg: int = 10) -> np.ndarray:
    """Centripetal-ish Catmull-Rom spline through all points (passes through each waypoint)."""
    p = np.asarray(points, float)
    if len(p) < 3:
        return p
    ext = np.vstack([2 * p[0] - p[1], p, 2 * p[-1] - p[-2]])
    out = []
    for i in range(1, len(ext) - 2):
        p0, p1, p2, p3 = ext[i - 1], ext[i], ext[i + 1], ext[i + 2]
        for t in np.linspace(0, 1, samples_per_seg, endpoint=False):
            t2, t3 = t * t, t * t * t
            out.append(0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                              + (-p0 + 3 * p1 - 3 * p2 + p3) * t3))
    out.append(p[-1])
    return np.array(out)
