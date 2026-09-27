"""Buoy map: fuses camera detections (already in world coordinates via GPS pose) with the
course prior. Each object keeps a 2D position estimate with a scalar variance
(Kalman update). Unmatched detections become new tracks (e.g. an unexpected buoy),
which the planner treats as obstacles.

Modes
  prior  : use course positions only (surveyed GPS points); vision is displayed but not fused
  fused  : course positions are priors (sigma = prior_sigma_m), refined by vision  [default]
  vision : priors are only coarse search hints (large sigma); positions come from vision
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass

from ..types import Detection


COMPATIBLE = {"yellow": {"case"}, "case": {"yellow"}, "orange": {"target"}, "target": {"orange"}, "black": {"zebra"}}


def label_for(obj: dict) -> str:
    k = obj.get("kind", "buoy")
    if k == "target":
        return "target"
    if k == "case":
        return "case"
    if k == "detector":
        return "pink"
    return obj["color"]


@dataclass
class Track:
    tid: str
    label: str
    x: float
    y: float
    var: float
    hits: int = 0
    last_t: float = -1.0
    prior_id: str | None = None
    px: float | None = None
    py: float | None = None
    removed: bool = False

    @property
    def sigma(self) -> float:
        return math.sqrt(self.var)


class BuoyMap:
    def __init__(self, course: dict, mode: str = "fused", prior_sigma_m: float = 1.0, vision_sigma_m: float = 4.0):
        self.mode = mode
        self.water = course["water"]
        self.tracks: dict[str, Track] = {}
        self._n = 0
        sig = vision_sigma_m if mode == "vision" else prior_sigma_m
        for o in course["objects"]:
            self.tracks[o["id"]] = Track(o["id"], label_for(o), o["x"], o["y"], sig ** 2, 0, -1, o["id"], o["x"], o["y"])
        self.last_detections: list[Detection] = []
        self.last_t: float | None = None
        self.process_noise = 0.08          # m / sqrt(s): lets estimates follow slow GPS bias drift

    def _meas_var(self, d: Detection) -> float:
        k = {"depth": 0.03, "waterline": 0.07, "size": 0.2}.get(d.range_source, 0.2)
        var = (k * d.range_m) ** 2 + 0.05 ** 2
        if d.label == "target":
            var += 0.4 ** 2 + (0.6 / max(d.range_m, 1.0)) ** 2
            if d.range_source == "depth_clipped":      # target cut by the image edge: centre estimate is biased
                var += 1.5 ** 2
        return var

    def update(self, dets: list[Detection], t: float):
        if self.last_t is not None and t > self.last_t:
            q = self.process_noise ** 2 * (t - self.last_t)      # map frame drifts with the GPS bias
            for tr in self.tracks.values():
                tr.var += q
        self.last_t = t
        wb = self.water
        keep = [d for d in dets if wb["xmin"] - 0.5 <= d.x <= wb["xmax"] + 0.5 and wb["ymin"] - 0.5 <= d.y <= wb["ymax"] + 0.5]
        self.last_detections = keep
        if self.mode == "prior":
            for d in keep:
                tr = self._associate(d, self._meas_var(d))
                if tr:
                    tr.hits += 1
                    tr.last_t = t
            return keep
        for d in keep:
            R = self._meas_var(d)
            tr = self._associate(d, R)
            if tr is None:
                self._n += 1
                tid = f"new_{d.label}_{self._n}"
                self.tracks[tid] = Track(tid, d.label, d.x, d.y, R, 1, t)
                continue
            if tr.label != d.label:
                tr.hits += 1
                tr.last_t = t
                continue
            K = tr.var / (tr.var + R)
            tr.x += K * (d.x - tr.x)
            tr.y += K * (d.y - tr.y)
            tr.var = max((1 - K) * tr.var, 0.03 ** 2)
            tr.hits += 1
            tr.last_t = t
        self._merge_duplicates()
        return keep

    def _associate(self, d: Detection, R: float) -> Track | None:
        best, best_score = None, None
        compat = COMPATIBLE.get(d.label, set())
        for tr in self.tracks.values():
            if tr.removed:
                continue
            same = tr.label == d.label
            if not same and not (tr.label in compat and tr.prior_id):
                continue
            dist = math.hypot(d.x - tr.x, d.y - tr.y)
            gate = max(1.5, 3.0 * math.sqrt(tr.var + R))
            if not same:
                gate = min(gate, 1.0)          # a mis-labelled look at a known object (e.g. case seen as yellow)
            if tr.label == "case":
                gate = max(gate, 2.5)          # the case floats and may drift
            if dist > gate:
                continue
            score = dist ** 2 / (tr.var + R)
            if best_score is None or score < best_score:
                best, best_score = tr, score
        return best

    def _merge_duplicates(self):
        news = [t for t in self.tracks.values() if t.prior_id is None and not t.removed]
        for n in news:
            for t in self.tracks.values():
                if t is n or t.removed or t.label != n.label:
                    continue
                if math.hypot(n.x - t.x, n.y - t.y) < 0.8 and (t.prior_id or t.hits >= n.hits):
                    n.removed = True
                    break

    # ------------------------------------------------------------ queries
    def get(self, object_id: str) -> Track:
        return self.tracks[object_id]

    def position(self, object_id: str) -> tuple[float, float]:
        t = self.tracks[object_id]
        return t.x, t.y

    def find_label(self, label: str, among: list[str] | None = None) -> Track | None:
        cands = [self.tracks[i] for i in among] if among else list(self.tracks.values())
        cands = [t for t in cands if t.label == label and not t.removed]
        return max(cands, key=lambda t: t.hits, default=None)

    def remove(self, object_id: str):
        if object_id in self.tracks:
            self.tracks[object_id].removed = True

    def unexpected(self, min_hits: int = 3) -> list[Track]:
        return [t for t in self.tracks.values() if t.prior_id is None and not t.removed and t.hits >= min_hits]

    def as_list(self, min_new_hits: int = 3) -> list[dict]:
        out = []
        for t in self.tracks.values():
            if t.removed or (t.prior_id is None and t.hits < min_new_hits):
                continue
            d = asdict(t)
            d["sigma"] = round(t.sigma, 3)
            d["x"], d["y"] = round(t.x, 3), round(t.y, 3)
            out.append(d)
        return out
