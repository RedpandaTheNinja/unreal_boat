"""Shared context handed to every behaviour and task."""
from __future__ import annotations

import math
import time

from ..control.hold import HoldController
from ..control.pure_pursuit import PurePursuit
from ..geo import LocalFrame
from ..model import BoatModel
from ..planning.grid import GridPlanner
from ..types import BoatState


class Ctx:
    def __init__(self, boat_cfg, course, bmap, hal, identify_color=None):
        self.cfg = boat_cfg
        self.course = course
        self.bmap = bmap
        self.hal = hal
        self.model = BoatModel(boat_cfg)
        self.pp = PurePursuit(self.model, boat_cfg["control"])
        self.hold_ctl = HoldController(boat_cfg["control"])
        self.planner = GridPlanner(course, boat_cfg)
        self.frame = LocalFrame.from_course(course)
        self.state: BoatState | None = None
        self.dt = 1.0 / boat_cfg["control"]["rate_hz"]
        self.t = 0.0
        self.identify_color = identify_color or course["challenges"]["4_identify"].get("default_color", "blue")
        self.events: list[dict] = []
        self.actions: list[dict] = []
        self.plan_path = None          # latest planned path (for dashboard)
        self.approach = None           # latest approach solution (for dashboard)
        self.phase = ""
        self.env_upstream = None       # (direction rad, magnitude m/s) the boat should face when holding
        self.on_action = None          # callback(result) - scorer hook

    def log(self, msg: str, level: str = "info"):
        e = {"t": round(self.t, 2), "level": level, "msg": msg, "wall": time.strftime("%H:%M:%S")}
        self.events.append(e)
        if len(self.events) > 400:
            del self.events[:100]
        print(f"[{e['t']:8.2f}] {level.upper():5s} {msg}", flush=True)

    def speed(self, name: str) -> float:
        return self.cfg["control"]["speed"][name]

    def objects(self, min_new_hits: int = 3) -> dict:
        """Obstacle dictionary {id: (x, y, radius)} from the buoy map."""
        radii = {"buoy": 0.135, "detector": 0.15}
        out = {}
        objs = {o["id"]: o for o in self.course["objects"]}
        for tid, tr in self.bmap.tracks.items():
            if tr.removed:
                continue
            if tr.prior_id is None and tr.hits < min_new_hits:
                continue
            o = objs.get(tr.prior_id or "", {})
            r = o.get("radius", radii.get(o.get("kind", "buoy"), 0.2 if tr.label not in ("target",) else 1.52))
            out[tid] = (tr.x, tr.y, r)
        return out

    def pose(self):
        s = self.state
        return (s.x, s.y, s.heading)

    def dist_to(self, xy) -> float:
        return math.hypot(xy[0] - self.state.x, xy[1] - self.state.y)
