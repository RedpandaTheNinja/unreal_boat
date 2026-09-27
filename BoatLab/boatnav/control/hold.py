"""Station keeping at a pose with fixed stern thrusters.

A twin-motor boat cannot push sideways, so the controller corrects along-track error with
common-mode thrust (forward/reverse) and heading with differential thrust. Lateral error
is only monitored: if it grows past lateral_abort_m the mission re-runs the approach.
"""
from __future__ import annotations

import math

from .. import geometry as G
from ..types import BoatState
from .pure_pursuit import mix


class HoldController:
    def __init__(self, ctrl_cfg: dict):
        self.h = ctrl_cfg["hold"]
        self.settled_for = 0.0
        self.last = {}

    def reset(self):
        self.settled_for = 0.0

    def update(self, st: BoatState, target: tuple, dt: float, heading_tol_deg: float | None = None) -> tuple[float, float, dict]:
        h = self.h
        tx, ty, th = target
        ex, ey = G.world_to_body(st.x, st.y, st.heading, tx, ty)
        eh = G.wrap(th - st.heading)
        common = h["kp_surge"] * ex - h["kd_surge"] * st.u
        diff = h["kp_heading"] * eh - h["kd_heading"] * st.r
        lim = h["max_cmd"]
        left, right = mix(max(-lim, min(lim, common)), max(-lim, min(lim, diff)))
        tol = heading_tol_deg if heading_tol_deg is not None else h["heading_tol_deg"]
        ok = (abs(ex) < h["pos_tol_m"] and abs(eh) < math.radians(tol)
              and st.speed < h["speed_tol_mps"] and abs(st.r) < 0.08)
        self.settled_for = self.settled_for + dt if ok else 0.0
        self.last = {"ex": ex, "ey": ey, "eh_deg": math.degrees(eh), "settled_s": self.settled_for,
                     "reapproach": abs(ey) > h["lateral_abort_m"]}
        return left, right, self.last

    @property
    def settled(self) -> bool:
        return self.settled_for >= self.h["settle_s"]
