"""Simulation-only scoring and payload-mechanism outcomes.

Uses simulator truth (twin or Unreal), never the autonomy's estimates, so a mission
cannot score itself by believing it did well. Scoring follows the AIMM-ICC 2025 rules
as far as they can be judged geometrically; judges' calls (e.g. partial points) are
approximated and labelled as such.
"""
from __future__ import annotations

import math

import numpy as np

from . import geometry as G

CONTACT_EPS = 0.03


def _obj(course, oid):
    for o in course["objects"]:
        if o["id"] == oid:
            return o
    raise KeyError(oid)


def simulate_actuator(name, pose, course, boat_cfg, rng, hidden=()):
    x, y, h = pose
    mech = boat_cfg["mechanisms"]
    if name == "deploy":
        m = mech["deploy"]
        bx, by = m["mount_flu_m"][0], m["mount_flu_m"][1] - m.get("release_gap_m", 0.8)
        lx, ly = G.body_to_world(x, y, h, bx, by)
        lx += rng.normal(0, m.get("sim_drift_sigma_m", 0.15))
        ly += rng.normal(0, m.get("sim_drift_sigma_m", 0.15))
        z = _obj(course, course["challenges"]["5_deploy"]["target"])
        d = math.hypot(lx - z["x"], ly - z["y"])
        rad = course["challenges"]["5_deploy"].get("success_radius_m", m.get("success_radius_m", 1.8288))
        return {"action": "deploy", "ok": True, "landing": [lx, ly], "distance_m": d, "success": d <= rad,
                "limit_m": rad}
    if name == "launch":
        m = mech["launch"]
        sx, sy = G.body_to_world(x, y, h, *m["mount_flu_m"])
        az = h + math.radians(m["azimuth_deg"])
        rng_m = m["range_m"] + rng.normal(0, m.get("sim_dispersion_m", 0.3) * 0.5)
        lx = sx + rng_m * math.cos(az) + rng.normal(0, m.get("sim_dispersion_m", 0.3) * 0.5)
        ly = sy + rng_m * math.sin(az) + rng.normal(0, m.get("sim_dispersion_m", 0.3) * 0.5)
        tgt = _obj(course, course["challenges"]["6_launch"]["target"])
        area = _obj(course, course["challenges"]["6_launch"]["area_buoy"])
        d = math.hypot(lx - tgt["x"], ly - tgt["y"])
        return {"action": "launch", "ok": True, "landing": [lx, ly], "distance_m": d, "success": d <= tgt["radius"],
                "target_radius_m": tgt["radius"], "boat_to_area_buoy_m": math.hypot(x - area["x"], y - area["y"])}
    if name == "recover":
        m = mech["recover"]
        case = _obj(course, course["challenges"]["7_recover"]["target"])
        if case["id"] in hidden:
            return {"action": "recover", "ok": False, "reason": "already recovered"}
        bx, by = G.world_to_body(x, y, h, case["x"], case["y"])
        along = abs(bx - m["mount_flu_m"][0])
        outward = m["mount_flu_m"][1] - by          # >0 when the case is outboard of the mount (starboard)
        ok = along <= m["capture_halflength_m"] and -0.15 <= outward <= m["reach_m"] + case.get("radius", 0.25)
        return {"action": "recover", "ok": True, "success": ok, "along_m": along, "outward_m": outward,
                "hide": case["id"] if ok else None, "value": 2 if ok else 0}
    return {"action": name, "ok": False, "reason": "unknown actuator"}


class Scorer:
    """Feed truth poses with update(); read report() at the end."""

    def __init__(self, course: dict, boat_cfg: dict, identify_color: str | None = None):
        self.c, self.cfg = course, boat_cfg
        self.L, self.B = boat_cfg["hull"]["length_m"], boat_cfg["hull"]["beam_m"]
        self.objs = {o["id"]: o for o in course["objects"]}
        self.identify_color = identify_color or course["challenges"]["4_identify"].get("default_color", "blue")
        self.prev = None
        self.t0 = None
        self.contacts: dict[str, list] = {}
        self._touching: set[str] = set()
        self.gates = {g["id"]: dict(g, center_cross_t=None, full_t=None, direction_ok=None, _armed=False)
                      for g in course["gates"]}
        self.slalom = {bid: None for bid in course["challenges"]["2_dodge"]["buoys"]}
        self.detector_exposure_s = 0.0
        self.min_detector_m = math.inf
        self.shore_hits = 0
        self.keepouts = [(k["id"], np.array(k["polygon"], dtype=float)) for k in course.get("keepout", [])]
        self.keepout_hits: dict[str, int] = {}
        self.min_clearance_m = math.inf
        self.actions = []
        self.hidden: set[str] = set()
        self.last_t = None

    # ------------------------------------------------------------------ helpers
    def _in_channel(self):
        ch = self.c["challenges"]["3_evade"]
        e, x = self.gates[ch["entry_gate"]], self.gates[ch["exit_gate"]]
        return e["center_cross_t"] is not None and x["full_t"] is None

    def intended(self):
        ch = self.c["challenges"]
        ids = {ch["9_return"]["target"]}
        ids |= {o for o in ch["4_identify"]["candidates"] if self.objs[o]["color"] == self.identify_color}
        return ids

    def _gate_geom(self, g):
        r, gr = self.objs[g["red"]], self.objs[g["green"]]
        a, b = np.array([r["x"], r["y"]]), np.array([gr["x"], gr["y"]])
        # required travel direction keeps red on starboard: d = rot90ccw(red - green)
        rg = a - b
        d = np.array([-rg[1], rg[0]])
        return a, b, d / np.linalg.norm(d)

    def update(self, truth: dict):
        t, x, y, h = truth["t"], truth["x"], truth["y"], truth["heading"]
        self.hidden = set(truth.get("hidden", self.hidden))
        if self.t0 is None:
            self.t0 = t
        dt = 0.0 if self.last_t is None else max(0.0, t - self.last_t)
        self.last_t = t
        poly = G.hull_polygon(x, y, h, self.L, self.B)
        # contacts / clearance
        touching = set()
        for oid, o in self.objs.items():
            if oid in self.hidden or o["kind"] == "case":
                continue
            d = G.polygon_distance((o["x"], o["y"]), poly) - o.get("radius", 0.15)
            if o["kind"] != "target" and oid not in self.intended():
                self.min_clearance_m = min(self.min_clearance_m, d) if d > -1 else self.min_clearance_m
            if d <= CONTACT_EPS:
                touching.add(oid)
                if oid not in self._touching:
                    self.contacts.setdefault(oid, []).append(round(t - self.t0, 2))
            if o["kind"] == "detector" and self._in_channel():
                dc = math.hypot(x - o["x"], y - o["y"])
                self.min_detector_m = min(self.min_detector_m, dc)
                if dc <= o.get("detect_radius", 6.0):
                    self.detector_exposure_s += dt
        self._touching = touching
        wb = self.c["water"]
        if poly[:, 0].min() < wb["xmin"] or poly[:, 0].max() > wb["xmax"] or poly[:, 1].min() < wb["ymin"] or poly[:, 1].max() > wb["ymax"]:
            self.shore_hits += 1
        lo, hi = poly.min(axis=0), poly.max(axis=0)
        for kid, kp in self.keepouts:           # piers and moored boats
            if (kp.min(axis=0) > hi).any() or (kp.max(axis=0) < lo).any():
                continue
            n = len(kp)
            if (any(G.point_in_polygon(p, kp) for p in poly) or any(G.point_in_polygon(q, poly) for q in kp)
                    or any(G.segments_intersect(poly[i], poly[(i + 1) % len(poly)], kp[j], kp[(j + 1) % n])
                           for i in range(len(poly)) for j in range(n))):
                self.keepout_hits[kid] = self.keepout_hits.get(kid, 0) + 1
        # gates
        if self.prev is not None:
            px, py, ph = self.prev
            for g in self.gates.values():
                a, b, d = self._gate_geom(g)
                if G.segments_intersect((px, py), (x, y), a, b):
                    g["center_cross_t"] = round(t - self.t0, 2)
                    g["direction_ok"] = bool(np.dot([x - px, y - py], d) > 0)
                    g["_armed"] = True
                if g["_armed"] and g["full_t"] is None:
                    s = np.array([G.side_of_line(a, b, p) for p in poly])
                    s_center = G.side_of_line(a, b, (x, y))
                    if (np.sign(s) == np.sign(s_center)).all() and abs(s_center) > 1e-6:
                        g["full_t"] = round(t - self.t0, 2)
            # slalom buoy pass sides (evaluated when the boat passes abeam of each buoy)
            gA = self.gates[self.c["challenges"]["2_dodge"]["entry_gate"]]
            if gA["center_cross_t"] is not None:
                _, _, dA = self._gate_geom(gA)
                for bid in self.slalom:
                    if self.slalom[bid] is not None:
                        continue
                    bo = self.objs[bid]
                    before = np.dot([px - bo["x"], py - bo["y"]], dA)
                    after = np.dot([x - bo["x"], y - bo["y"]], dA)
                    if before < 0 <= after:
                        left = G.side_of_line((x, y), (x + dA[0], y + dA[1]), (bo["x"], bo["y"])) > 0
                        want_left = bo["color"] == "green"
                        self.slalom[bid] = {"t": round(t - self.t0, 2), "correct_side": bool(left == want_left)}
        self.prev = (x, y, h)

    def record_action(self, res: dict):
        self.actions.append(res)

    def report(self) -> dict:
        ch = self.c["challenges"]
        g = self.gates
        gA, gB = g[ch["2_dodge"]["entry_gate"]], g[ch["2_dodge"]["exit_gate"]]
        c1 = g[ch["1_gate"]["gate"]]
        gate_touch = [oid for oid in (c1["red"], c1["green"]) if oid in self.contacts]
        slalom_ok = all(v and v["correct_side"] for v in self.slalom.values())
        slalom_touch = [b for b in list(self.slalom) + [gA["red"], gA["green"], gB["red"], gB["green"]] if b in self.contacts]
        ce, cx = g[ch["3_evade"]["entry_gate"]], g[ch["3_evade"]["exit_gate"]]
        bank_touch = [b for b in ch["3_evade"]["banks"] if b in self.contacts]
        ident = [o for o in ch["4_identify"]["candidates"] if self.objs[o]["color"] == self.identify_color]
        ident_ok = bool(ident) and ident[0] in self.contacts
        wrong_ident = [o for o in ch["4_identify"]["candidates"] if o in self.contacts and o not in ident]
        last = {a["action"]: a for a in self.actions if a.get("ok")}
        rep = {
            "1_gate": {"entered": c1["center_cross_t"] is not None, "full_pass": c1["full_t"] is not None,
                       "direction_ok": c1["direction_ok"], "buoy_contacts": gate_touch,
                       "passed": c1["full_t"] is not None and bool(c1["direction_ok"])},
            "2_dodge": {"sides": self.slalom, "all_correct": slalom_ok, "contacts": slalom_touch,
                        "time_s": round(gB["center_cross_t"] - gA["center_cross_t"], 2)
                        if gA["center_cross_t"] is not None and gB["center_cross_t"] is not None else None,
                        "passed": slalom_ok and gB["full_t"] is not None},
            "3_evade": {"entered": ce["center_cross_t"] is not None, "exited_full": cx["full_t"] is not None,
                        "direction_ok": bool(ce["direction_ok"]) and bool(cx["direction_ok"]),
                        "detector_exposure_s": round(self.detector_exposure_s, 2),
                        "min_detector_distance_m": round(self.min_detector_m, 2) if math.isfinite(self.min_detector_m) else None,
                        "bank_contacts": bank_touch,
                        "time_s": round(cx["center_cross_t"] - ce["center_cross_t"], 2)
                        if ce["center_cross_t"] is not None and cx["center_cross_t"] is not None else None,
                        "passed": cx["full_t"] is not None and bool(ce["direction_ok"]) and bool(cx["direction_ok"])},
            "4_identify": {"color": self.identify_color, "contacted": ident_ok, "wrong_contacts": wrong_ident,
                           "passed": ident_ok and not wrong_ident},
            "5_deploy": {**last.get("deploy", {}), "passed": bool(last.get("deploy", {}).get("success"))},
            "6_launch": {**last.get("launch", {}), "passed": bool(last.get("launch", {}).get("success"))},
            "7_recover": {**last.get("recover", {}), "passed": bool(last.get("recover", {}).get("success"))},
            "9_return": {"contacted": ch["9_return"]["target"] in self.contacts,
                         "passed": ch["9_return"]["target"] in self.contacts},
            "contacts": self.contacts,
            "shore_contact_samples": self.shore_hits,
            "keepout_contact_samples": dict(self.keepout_hits),
            "min_clearance_to_non_target_m": round(self.min_clearance_m, 3) if math.isfinite(self.min_clearance_m) else None,
            "elapsed_s": round((self.last_t or 0) - (self.t0 or 0), 1),
            "scoring_note": "Simulation judge: geometric approximation of the AIMM-ICC 2025 rules.",
        }
        rep["passed_count"] = sum(1 for k, v in rep.items() if isinstance(v, dict) and v.get("passed"))
        return rep
