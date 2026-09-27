"""Occupancy-grid A* over the lake with inflated obstacles and soft cost fields.

Obstacles come from the live buoy map (estimated positions), not simulator truth.
Inflation = object radius + half beam + clearance, so the planned centreline keeps the
hull off every buoy; a soft cost ring encourages extra clearance where there is room.
"""
from __future__ import annotations

import heapq
import math

import numpy as np

from .. import geometry as G

SQ2 = math.sqrt(2)
NB = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, SQ2), (1, -1, SQ2), (-1, 1, SQ2), (-1, -1, SQ2)]


class GridPlanner:
    def __init__(self, course: dict, boat_cfg: dict):
        p = boat_cfg["planning"]
        self.res = p["grid_res_m"]
        self.half_beam = boat_cfg["hull"]["beam_m"] / 2
        self.clear = p["hull_clearance_m"]
        self.shore = p["shore_margin_m"]
        self.det_w = p["detector_cost_weight"]
        wb = course["water"]
        self.x0, self.y0 = wb["xmin"], wb["ymin"]
        self.nx = int(math.ceil((wb["xmax"] - wb["xmin"]) / self.res)) + 1
        self.ny = int(math.ceil((wb["ymax"] - wb["ymin"]) / self.res)) + 1
        xs = self.x0 + np.arange(self.nx) * self.res
        ys = self.y0 + np.arange(self.ny) * self.res
        self.X, self.Y = np.meshgrid(xs, ys, indexing="ij")
        edge = np.minimum.reduce([self.X - wb["xmin"], wb["xmax"] - self.X, self.Y - wb["ymin"], wb["ymax"] - self.Y])
        self.base_block = edge < self.half_beam + self.shore
        for k in course.get("keepout", []):
            poly = np.array(k["polygon"])
            lo, hi = poly.min(0) - (self.half_beam + self.clear), poly.max(0) + (self.half_beam + self.clear)
            self.base_block |= (self.X >= lo[0]) & (self.X <= hi[0]) & (self.Y >= lo[1]) & (self.Y <= hi[1])
        self.blocked = self.base_block.copy()
        self.cost = np.ones_like(self.X)

    # ------------------------------------------------------------------ grid
    def cell(self, x, y):
        return (int(round((x - self.x0) / self.res)), int(round((y - self.y0) / self.res)))

    def world(self, i, j):
        return self.x0 + i * self.res, self.y0 + j * self.res

    def build(self, obstacles, detectors=(), extra_clear=0.0):
        """obstacles: iterable of (x, y, radius); detectors: iterable of (x, y, detect_radius)."""
        self.blocked = self.base_block.copy()
        self.cost = np.ones_like(self.X)
        for x, y, r in obstacles:
            infl = r + self.half_beam + self.clear + extra_clear
            soft = infl + 1.2
            i0, j0 = self.cell(x - soft, y - soft)
            i1, j1 = self.cell(x + soft, y + soft)
            i0, j0, i1, j1 = max(i0, 0), max(j0, 0), min(i1 + 1, self.nx), min(j1 + 1, self.ny)
            if i0 >= i1 or j0 >= j1:
                continue
            d = np.hypot(self.X[i0:i1, j0:j1] - x, self.Y[i0:i1, j0:j1] - y)
            self.blocked[i0:i1, j0:j1] |= d < infl
            ring = (d >= infl) & (d < soft)
            self.cost[i0:i1, j0:j1] += np.where(ring, 2.0 * (soft - d) / (soft - infl), 0.0)
        for x, y, rad in detectors:
            d2 = (self.X - x) ** 2 + (self.Y - y) ** 2
            self.cost += self.det_w * np.exp(-d2 / (rad ** 2))
        return self

    def free(self, x, y) -> bool:
        i, j = self.cell(x, y)
        return 0 <= i < self.nx and 0 <= j < self.ny and not self.blocked[i, j]

    def line_clear(self, a, b) -> bool:
        n = int(math.ceil(math.dist(a, b) / (self.res * 0.5))) + 1
        for t in np.linspace(0, 1, n):
            if not self.free(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])):
                return False
        return True

    def _clear_disk(self, x, y, r):
        i0, j0 = self.cell(x - r, y - r)
        i1, j1 = self.cell(x + r, y + r)
        i0, j0, i1, j1 = max(i0, 0), max(j0, 0), min(i1 + 1, self.nx), min(j1 + 1, self.ny)
        d = np.hypot(self.X[i0:i1, j0:j1] - x, self.Y[i0:i1, j0:j1] - y)
        self.blocked[i0:i1, j0:j1] &= ~(d < r)

    # ------------------------------------------------------------------ search
    def astar(self, start, goal, free_start_r=0.8, free_goal_r=0.0):
        if free_start_r > 0:
            self._clear_disk(*start, free_start_r)
        if free_goal_r > 0:
            self._clear_disk(*goal, free_goal_r)
        si, sj = self.cell(*start)
        gi, gj = self.cell(*goal)
        if not (0 <= si < self.nx and 0 <= sj < self.ny and 0 <= gi < self.nx and 0 <= gj < self.ny):
            return None
        if self.blocked[gi, gj]:
            return None
        blocked, cost, nx, ny = self.blocked, self.cost, self.nx, self.ny
        g = {(si, sj): 0.0}
        parent = {}
        openq = [(0.0, 0.0, si, sj)]
        closed = set()
        while openq:
            f, gc, i, j = heapq.heappop(openq)
            if (i, j) in closed:
                continue
            if (i, j) == (gi, gj):
                break
            closed.add((i, j))
            for di, dj, w in NB:
                a, b = i + di, j + dj
                if a < 0 or b < 0 or a >= nx or b >= ny or blocked[a, b] or (a, b) in closed:
                    continue
                ng = gc + w * self.res * 0.5 * (cost[i, j] + cost[a, b])
                if ng < g.get((a, b), 1e18):
                    g[(a, b)] = ng
                    parent[(a, b)] = (i, j)
                    h = math.hypot(a - gi, b - gj) * self.res
                    heapq.heappush(openq, (ng + h, ng, a, b))
        else:
            return None
        if (gi, gj) not in parent and (gi, gj) != (si, sj):
            return None
        cells = [(gi, gj)]
        while cells[-1] != (si, sj):
            cells.append(parent[cells[-1]])
        cells.reverse()
        pts = [self.world(i, j) for i, j in cells]
        pts[0], pts[-1] = tuple(start), tuple(goal)
        return self._string_pull(pts)

    def _string_pull(self, pts):
        """Remove intermediate vertices while the straight segment stays clear and cheap."""
        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1:
                if self.line_clear(pts[i], pts[j]) and self._line_cost_ok(pts, i, j):
                    break
                j -= 1
            out.append(pts[j])
            i = j
        return out

    def _line_cost_ok(self, pts, i, j):
        """Only shortcut if it does not pass through much more expensive (detector) cost."""
        n = max(2, int(math.dist(pts[i], pts[j]) / self.res))
        straight = 0.0
        for t in np.linspace(0, 1, n):
            x, y = pts[i][0] + t * (pts[j][0] - pts[i][0]), pts[i][1] + t * (pts[j][1] - pts[i][1])
            ci, cj = self.cell(x, y)
            straight += self.cost[min(max(ci, 0), self.nx - 1), min(max(cj, 0), self.ny - 1)]
        straight *= math.dist(pts[i], pts[j]) / n
        orig = 0.0
        for k in range(i, j):
            a, b = pts[k], pts[k + 1]
            ci, cj = self.cell(*a)
            orig += math.dist(a, b) * self.cost[min(max(ci, 0), self.nx - 1), min(max(cj, 0), self.ny - 1)]
        return straight <= orig * 1.05 + 0.5

    def plan(self, start, goal, obstacles, detectors=(), smooth_radius=2.0, free_goal_r=0.0):
        self.build(obstacles, detectors)
        pts = self.astar(start, goal, free_goal_r=free_goal_r)
        if pts is None:
            # retry with reduced clearance before giving up
            self.build(obstacles, detectors, extra_clear=-0.25)
            pts = self.astar(start, goal, free_goal_r=free_goal_r)
            if pts is None:
                return None
        return G.smooth_corners(pts, smooth_radius)
