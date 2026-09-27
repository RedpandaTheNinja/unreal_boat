"""Stand-off approach planning: get close enough to act on a target without driving at it.

For a mechanism at body point M that must see the target at body offset b (e.g. 1.5 m
out to starboard of the drop chute), the boat pose for heading theta is

        p(theta) = T - R(theta) b

so the target ends up abeam at the required gap. Every candidate heading is checked:
  * hull footprint at p clear of every other object (+ margin) and the shore
  * the straight final leg (pre -> p, length `lead`) clear, so the boat arrives parallel
    and already slow, rather than turning at the target
and scored by distance to reach the pre point, turning required, launch-area rules and
(optionally) facing into current/wind for steadier holding.

Contact approaches (Identify / Return) instead aim the bow at the target along a clear
line and finish at creep speed.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .. import geometry as G


@dataclass
class Approach:
    pose: tuple            # (x, y, heading) where the boat should stop
    pre: tuple             # start of the straight final leg
    heading: float
    cost: float
    kind: str
    target: tuple
    note: str = ""

    def as_dict(self):
        return {"pose": [round(v, 3) for v in self.pose], "pre": [round(v, 3) for v in self.pre],
                "heading_deg": round(math.degrees(self.heading), 1), "cost": round(self.cost, 2),
                "kind": self.kind, "target": [round(v, 3) for v in self.target], "note": self.note}


def desired_offset(boat_cfg: dict, kind: str) -> tuple[float, float]:
    """Body-frame position (FLU) where the target should sit when the mechanism fires."""
    mech = boat_cfg["mechanisms"]
    if kind == "deploy":
        m = mech["deploy"]
        # land the sensor ~0.6 m short of the buoy, well inside the 6 ft radius
        return m["mount_flu_m"][0], m["mount_flu_m"][1] - m.get("release_gap_m", 0.9) - 0.6
    if kind == "launch":
        m = mech["launch"]
        az = math.radians(m["azimuth_deg"])
        return m["mount_flu_m"][0] + m["range_m"] * math.cos(az), m["mount_flu_m"][1] + m["range_m"] * math.sin(az)
    if kind == "recover":
        m = mech["recover"]
        return m["mount_flu_m"][0], m["mount_flu_m"][1] - m.get("release_gap_m", 0.35)
    raise ValueError(kind)


def _footprint_clear(pose, objects, length, beam, margin, exclude=()):
    poly = G.hull_polygon(*pose, length, beam)
    for oid, (x, y, r) in objects.items():
        if oid in exclude:
            continue
        if G.polygon_distance((x, y), poly) < r + margin:
            return False
    return True


def plan_standoff(kind, target_xy, current_pose, objects: dict, planner, boat_cfg, *, exclude=(),
                  area_buoy=None, env_upstream=None, lead=None) -> Approach | None:
    """objects: {id: (x, y, radius)} from the buoy map. exclude: ids not treated as obstacles."""
    L, B = boat_cfg["hull"]["length_m"], boat_cfg["hull"]["beam_m"]
    lead = lead or boat_cfg["planning"]["approach_lead_m"]
    n = boat_cfg["planning"]["heading_candidates"]
    bx, by = desired_offset(boat_cfg, kind)
    tx, ty = target_xy
    cx, cy, ch = current_pose
    obst = [(x, y, r) for oid, (x, y, r) in objects.items() if oid not in exclude]
    planner.build(obst)
    best = None
    for k in range(n):
        th = -math.pi + 2 * math.pi * k / n
        c, s = math.cos(th), math.sin(th)
        px, py = tx - (c * bx - s * by), ty - (s * bx + c * by)
        pose = (px, py, th)
        if not planner.free(px, py):
            continue
        if not _footprint_clear(pose, objects, L, B, 0.25, exclude=exclude):
            continue
        pre = (px - lead * c, py - lead * s)
        if not planner.free(*pre) or not planner.line_clear(pre, (px, py)):
            continue
        # sweep the hull along the final leg
        swept_ok = all(_footprint_clear((pre[0] + t * (px - pre[0]), pre[1] + t * (py - pre[1]), th), objects, L, B,
                                        0.3, exclude=exclude) for t in np.linspace(0, 1, 8))
        if not swept_ok:
            continue
        dist = math.hypot(pre[0] - cx, pre[1] - cy)
        arrive_dir = math.atan2(pre[1] - cy, pre[0] - cx)
        turn = abs(G.wrap(th - arrive_dir))
        cost = dist + 3.0 * turn
        note = ""
        if kind == "launch" and area_buoy is not None:
            da = math.hypot(px - area_buoy[0], py - area_buoy[1])
            lim = boat_cfg["mechanisms"]["launch"].get("max_distance_from_area_buoy_m", 5.0)
            if da > lim:
                continue
            cost += 1.5 * da
            note = f"{da:.1f} m from launch-area buoy"
        if env_upstream is not None and env_upstream[1] > 0.05:
            cost += 40.0 * env_upstream[1] * (1 - math.cos(th - env_upstream[0]))
        if best is None or cost < best.cost:
            best = Approach(pose, pre, th, cost, kind, (tx, ty), note)
    return best


def plan_contact(target_xy, target_r, current_pose, objects: dict, planner, boat_cfg, *, exclude=(), lead=4.5):
    """Bow-first contact approach. Returns an Approach whose pose puts the bow just past the target surface.
    Candidate headings every 360/n degrees plus the direct line from the current position."""
    L, B = boat_cfg["hull"]["length_m"], boat_cfg["hull"]["beam_m"]
    n = boat_cfg["planning"]["heading_candidates"]
    tx, ty = target_xy
    cx, cy, ch = current_pose
    obst = [(x, y, r) for oid, (x, y, r) in objects.items() if oid not in exclude]
    planner.build(obst)
    stop_d = L / 2 + target_r - 0.12          # bow slightly inside the target radius => contact
    cands = [(-math.pi + 2 * math.pi * k / n, lead) for k in range(n)]
    direct = math.atan2(ty - cy, tx - cx)
    dist_now = math.hypot(tx - cx, ty - cy)
    if dist_now - stop_d > 2.0:
        cands.append((direct, dist_now - stop_d))
    best = None
    for th, ld in cands:
        c, s = math.cos(th), math.sin(th)
        px, py = tx - stop_d * c, ty - stop_d * s
        pre = (tx - (stop_d + ld) * c, ty - (stop_d + ld) * s)
        if not planner.free(*pre) or not planner.line_clear(pre, (px - 0.6 * c, py - 0.6 * s)):
            continue
        # the hull on the final line must not touch any other object
        ok = all(_footprint_clear((pre[0] + t * (px - pre[0]), pre[1] + t * (py - pre[1]), th), objects, L,
                                  B, 0.35, exclude=exclude) for t in np.linspace(0, 1, 8))
        if not ok:
            continue
        leg_dir = math.atan2(pre[1] - cy, pre[0] - cx)
        dist = math.hypot(pre[0] - cx, pre[1] - cy)
        # turning: rotate to the leg (only matters if the leg is long) then onto the approach line
        turn = (abs(G.wrap(leg_dir - ch)) if dist > 0.5 else 0.0) + (abs(G.wrap(th - leg_dir)) if dist > 0.5 else abs(G.wrap(th - ch)))
        clear = min((G.polygon_distance((x, y), G.hull_polygon(px, py, th, L, B)) - rr
                     for oid, (x, y, rr) in objects.items() if oid not in exclude), default=5.0)
        cost = dist + 3.0 * turn - 8.0 * min(clear, 1.2)
        if best is None or cost < best.cost:
            best = Approach((px, py, th), pre, th, cost, "contact", (tx, ty), f"neighbour clearance {clear:.2f} m")
    return best
