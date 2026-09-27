"""Rule-aware path construction for gates, slalom and the evade channel.

AIMM-ICC buoy rule (PDF p5): pass on the RIGHT of green and the LEFT of red, i.e. keep
green on the boat's port side and red on its starboard side. For a red/green pair this
fixes the crossing direction:  d = rot90ccw(red - green).
All positions come from the live buoy map, so vision corrections move the paths.
"""
from __future__ import annotations

import math

import numpy as np



def gate_frame(bmap, red_id: str, green_id: str):
    r = np.array(bmap.position(red_id))
    g = np.array(bmap.position(green_id))
    center = (r + g) / 2
    rg = r - g
    d = np.array([-rg[1], rg[0]])
    d /= np.linalg.norm(d)
    return center, d, float(np.linalg.norm(rg))


def gate_points(bmap, gate: dict, lead: float, follow: float):
    c, d, w = gate_frame(bmap, gate["red"], gate["green"])
    return [c - lead * d, c, c + follow * d], d


def slalom_points(bmap, entry_gate: dict, exit_gate: dict, buoy_ids, offset: float, lead: float, follow: float):
    cA, dA, _ = gate_frame(bmap, entry_gate["red"], entry_gate["green"])
    cB, dB, _ = gate_frame(bmap, exit_gate["red"], exit_gate["green"])
    axis = cB - cA
    axis /= np.linalg.norm(axis)
    left = np.array([-axis[1], axis[0]])
    pts = [cA - lead * dA, cA]
    ordered = sorted(buoy_ids, key=lambda b: float(np.dot(np.array(bmap.position(b)) - cA, axis)))
    for b in ordered:
        p = np.array(bmap.position(b))
        color = bmap.get(b).label
        side = 1.0 if color == "red" else -1.0     # red stays to starboard => boat passes on its left
        pts.append(p + side * offset * left)
    pts += [cB, cB + follow * dB]
    return pts


def channel_setup(bmap, entry_gate: dict, exit_gate: dict, bank_ids, lead: float):
    """Corridor, bank walls and entry/exit points for the evade channel."""
    cE, dE, wE = gate_frame(bmap, entry_gate["red"], entry_gate["green"])
    cX, dX, wX = gate_frame(bmap, exit_gate["red"], exit_gate["green"])
    pre, post = cE - lead * dE, cX + lead * dX
    # walls along each bank (red side and green side), densely sampled virtual obstacles
    walls = []
    red_bank = [bmap.position(b) for b in bank_ids if bmap.get(b).label == "red"]
    green_bank = [bmap.position(b) for b in bank_ids if bmap.get(b).label == "green"]
    for bank in (red_bank, green_bank):
        bank = sorted(bank, key=lambda p: float(np.dot(np.array(p) - cE, dE)))
        for a, b in zip(bank[:-1], bank[1:]):
            n = max(2, int(math.dist(a, b) / 0.4))
            for t in np.linspace(0, 1, n):
                walls.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), 0.1))
    # corridor: rectangle spanning the banks, extended by `lead` before the entry and after the exit
    pts = np.array(red_bank + green_bank + [tuple(pre), tuple(post)])
    lo, hi = pts.min(0), pts.max(0)
    return dict(pre=pre, post=post, entry=cE, exit=cX, d_entry=dE, d_exit=dX, walls=walls, box=(lo, hi))


def corridor_obstacles(box, planner, margin=0.0):
    """Virtual obstacles that block the planner outside an axis-aligned corridor box."""
    lo, hi = box
    lo = lo - margin
    hi = hi + margin
    blocked = ~((planner.X >= lo[0]) & (planner.X <= hi[0]) & (planner.Y >= lo[1]) & (planner.Y <= hi[1]))
    return blocked
