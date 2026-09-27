"""AIMM-ICC challenge tasks. Each task is a generator: yields motor commands, returns a result dict.

Phases used throughout: transit -> acquire -> approach -> hold -> act -> verify -> depart.
Success is never awarded for merely reaching a waypoint: gates are crossed completely,
stand-off tasks hold steady before acting, and contact tasks confirm the bow reached
the target (or stalled against it) before backing off.
"""
from __future__ import annotations

import math

import numpy as np

from .. import geometry as G
from ..planning import course as C
from ..planning.approach import plan_contact, plan_standoff
from .behaviors import back_off, depart, ensure_turn_room, follow, hold, idle, observe, pivot_to, transit_to


def _gate(ctx, gid):
    return next(g for g in ctx.course["gates"] if g["id"] == gid)


# ----------------------------------------------------------------------------- challenge 1
def gate_task(ctx, gate="gateA", **_):
    p = ctx.cfg["planning"]
    g = _gate(ctx, gate)
    pts, d = C.gate_points(ctx.bmap, g, p["gate_lead_m"], p["gate_follow_m"])
    far = pts[1] - (p["gate_lead_m"] + 4.0) * d          # line up well before the gate
    ctx.log(f"Gate {gate}: crossing direction {math.degrees(math.atan2(d[1], d[0])):.0f} deg (red to starboard)")
    st = yield from transit_to(ctx, tuple(far), final_leg=[tuple(pts[0]), tuple(pts[1]), tuple(pts[2])],
                               final_speed=ctx.speed("gate"), label=f"gate {gate}")
    return {"status": "done" if st == "done" else st}


# ----------------------------------------------------------------------------- challenge 2
def slalom_task(ctx, entry_gate="gateA", exit_gate="gateB", exit_turn=None, **_):
    p = ctx.cfg["planning"]
    ch = ctx.course["challenges"]["2_dodge"]
    gA, gB = _gate(ctx, entry_gate), _gate(ctx, exit_gate)
    pts = C.slalom_points(ctx.bmap, gA, gB, ch["buoys"], p["slalom_offset_m"], p["gate_lead_m"], p["gate_follow_m"])
    cA, dA, _ = C.gate_frame(ctx.bmap, gA["red"], gA["green"])
    along = float(np.dot(np.array([ctx.state.x, ctx.state.y]) - cA, dA))
    if along > 0.5:                   # already through gate A (e.g. right after challenge 1)
        pts = pts[2:]
        pts.insert(0, (ctx.state.x, ctx.state.y))
    else:
        st = yield from transit_to(ctx, tuple(pts[0]), exclude=(gA["red"], gA["green"]), label="to slalom")
        if st not in ("done",):
            return {"status": st}
    if exit_turn in ("right", "left"):
        # bend away after the exit gate (e.g. AIMM: channel green buoy sits 15 ft beyond gate B)
        cB, dB, _ = C.gate_frame(ctx.bmap, gB["red"], gB["green"])
        side = np.array([dB[1], -dB[0]]) if exit_turn == "right" else np.array([-dB[1], dB[0]])
        pts[-2] = cB + 0.45 * side               # cross gate B a little toward the turn side
        pts[-1] = cB + 1.8 * dB + 1.1 * side
        pts.append(cB + 3.2 * dB + 2.8 * side)
    spline = G.catmull_rom(pts, 12)
    path = G.Path(spline, ctx.speed("slalom"), name="slalom").resampled(0.4)
    ctx.log(f"Slalom: {len(ch['buoys'])} buoys, offset {p['slalom_offset_m']} m, path {path.length:.1f} m")
    st = yield from follow(ctx, path, stop=False, timeout=180, label="slalom", check_blocked=False)
    return {"status": st}


# ----------------------------------------------------------------------------- challenge 3
def channel_task(ctx, **_):
    ch = ctx.course["challenges"]["3_evade"]
    gE, gX = _gate(ctx, ch["entry_gate"]), _gate(ctx, ch["exit_gate"])
    setup = C.channel_setup(ctx.bmap, gE, gX, ch["banks"], ctx.cfg["planning"]["gate_lead_m"])
    dets = [(ctx.bmap.position(d)[0], ctx.bmap.position(d)[1],
             next(o for o in ctx.course["objects"] if o["id"] == d).get("detect_radius", 6.0)) for d in ch["detectors"]]
    st = yield from transit_to(ctx, tuple(setup["pre"]), detectors=dets, label="to channel")
    if st != "done":
        return {"status": st}
    obst = [v for k, v in ctx.objects().items() if k not in ch["detectors"]] + setup["walls"]
    pl = ctx.planner
    pl.build(obst, dets)
    pl.blocked |= C.corridor_obstacles(setup["box"], pl, margin=0.2)
    pts = pl.astar(tuple(setup["pre"]), tuple(setup["post"]))
    if pts is not None:
        pts = [(ctx.state.x, ctx.state.y)] + list(pts)
    if pts is None:
        ctx.log("Channel: no path inside the corridor - check bank buoy positions", "error")
        return {"status": "failed"}
    pts = G.smooth_corners(pts, 2.5)
    path = G.Path(pts, ctx.speed("channel"), name="evade channel").resampled(0.4)
    worst = min(min(math.hypot(px - dx, py - dy) for dx, dy, _ in dets) for px, py in path.points)
    ctx.log(f"Evade: channel path {path.length:.1f} m, closest planned approach to a detector {worst:.1f} m")
    st = yield from follow(ctx, path, stop=False, timeout=150, label="evade channel", check_blocked=False)
    return {"status": st, "planned_min_detector_m": round(worst, 2)}


# ----------------------------------------------------------------------------- challenge 4 / 9 (contact)
def _contact(ctx, target_id, label):
    """Bow-first contact: A* to the start of a clear straight approach line, creep along it,
    stop on contact (estimated bow gap) or stall, then reverse back out along the same line."""
    tr = ctx.bmap.get(target_id)
    objs = ctx.objects()
    r = objs.get(target_id, (0, 0, 0.135))[2]
    appr = plan_contact((tr.x, tr.y), r, ctx.pose(), objs, ctx.planner, ctx.cfg, exclude=(target_id,))
    if appr is None:
        ctx.log(f"{label}: no clear bow-on approach line", "error")
        return "failed"
    ctx.approach = appr.as_dict()
    th = appr.heading
    end = (appr.pose[0] + 2.5 * math.cos(th), appr.pose[1] + 2.5 * math.sin(th))   # overrun: stop on contact/stall
    mid = ((appr.pre[0] + appr.pose[0]) / 2, (appr.pre[1] + appr.pose[1]) / 2)
    pts = plan_points_to(ctx, appr.pre, exclude=(target_id,))
    if pts is not None and len(pts) > 1:
        moved = yield from ensure_turn_room(ctx, math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0]))
        if moved == "moved":
            pts = plan_points_to(ctx, appr.pre, exclude=(target_id,))
    if pts is None:
        return "failed"
    speeds = [ctx.speed("transit")] * len(pts) + [ctx.speed("approach"), ctx.speed("contact"), ctx.speed("contact")]
    path = G.Path(list(pts) + [mid, appr.pose[:2], end], speeds, name=label).resampled(0.3)
    ctx.pp.set_path(path, stop_at_end=True)
    ctx.plan_path = path
    ctx.phase = f"{label}: approach"
    t0, stall, contact = ctx.t, 0.0, False
    L = ctx.cfg["hull"]["length_m"]
    s_line = path.length - (math.dist(appr.pre, appr.pose[:2]) + 2.5)
    while ctx.t - t0 < 120:
        tr = ctx.bmap.get(target_id)
        hull = G.hull_polygon(ctx.state.x, ctx.state.y, ctx.state.heading, L, ctx.cfg["hull"]["beam_m"])
        gap = G.polygon_distance((tr.x, tr.y), hull) - r
        l, rr = ctx.pp.update(ctx.state, ctx.dt)
        on_line = ctx.pp.s > s_line
        if on_line:
            ctx.phase = f"{label}: creep"
        pushing = l > 0.1 and rr > 0.1
        stall = stall + ctx.dt if (on_line and pushing and ctx.state.u < 0.05 and gap < 0.6) else 0.0
        if on_line and (gap < -0.02 or stall > 1.5):          # push very slightly past the surface = contact
            contact = True
            ctx.log(f"{label}: contact (hull gap estimate {gap:.2f} m{', stalled' if stall > 1.5 else ''})")
            break
        if ctx.pp.info.done:
            contact = gap < 0.05
            ctx.log(f"{label}: reached end of approach line (gap {gap:.2f} m)", "info" if contact else "warn")
            break
        yield l, rr
    yield from idle(ctx, 0.6, f"{label}: touch")
    yield from back_off(ctx, 2.5, exclude=(target_id,))
    return "done" if contact else "no_contact"


def plan_points_to(ctx, goal, exclude=()):
    from .behaviors import plan_points
    pts = plan_points(ctx, goal, exclude=exclude)
    if pts is None:
        ctx.log(f"no route to {tuple(round(v, 1) for v in goal)}", "error")
    return pts


def free_vantage(ctx, center, radii=(6.0, 9.0), n=24):
    """Closest collision-free point 6-9 m from `center` with a clear line of sight to it."""
    ctx.planner.build(list(ctx.objects().values()))
    best = None
    for rad in np.linspace(radii[0], radii[1], 3):
        for k in range(n):
            a = 2 * math.pi * k / n
            p = (center[0] + rad * math.cos(a), center[1] + rad * math.sin(a))
            if not ctx.planner.free(*p):
                continue
            d = ctx.dist_to(p)
            if best is None or d < best[0]:
                best = (d, p)
    return best[1] if best else None


def identify_task(ctx, color=None, **_):
    color = color or ctx.identify_color
    ctx.identify_color = color
    cands = ctx.course["challenges"]["4_identify"]["candidates"]
    tr = ctx.bmap.find_label(color, among=cands)
    if tr is None or tr.hits < 2:
        # acquire: look at the cluster from a vantage point before committing
        cxy = np.mean([ctx.bmap.position(c) for c in cands], axis=0)
        vantage = free_vantage(ctx, cxy, (6.0, 9.0))
        if vantage is None:
            ctx.log("Identify: no free vantage point - using prior positions", "warn")
            vantage = (ctx.state.x, ctx.state.y)
        ctx.log(f"Identify: acquiring {color} buoy - moving to vantage point")
        yield from transit_to(ctx, vantage, stop=True, label="identify: acquire")
        yield from pivot_to(ctx, math.atan2(cxy[1] - ctx.state.y, cxy[0] - ctx.state.x))
        yield from observe(ctx, 2.5, "identify: observe")
        tr = ctx.bmap.find_label(color, among=cands)
    if tr is None:
        ctx.log(f"Identify: no {color} buoy found", "error")
        return {"status": "failed", "color": color}
    ctx.log(f"Identify: target {tr.tid} ({color}), {tr.hits} detections, sigma {tr.sigma:.2f} m")
    st = yield from _contact(ctx, tr.tid, f"identify {color}")
    return {"status": st, "color": color, "target": tr.tid}


def return_task(ctx, dock=True, **_):
    tid = ctx.course["challenges"]["9_return"]["target"]
    st = yield from _contact(ctx, tid, "return buoy")
    if dock:
        st2 = yield from dock_task(ctx)
        return {"status": st, "dock": st2.get("status")}
    return {"status": st}


def dock_task(ctx, **_):
    x, y, _h = ctx.course["dock_pose"]
    pose = (x, y, 0.0)                      # arrive facing east, bow toward the pier
    pre = (x - 6.0, y)
    st = yield from transit_to(ctx, pre, final_leg=[(x, y)], final_speed=ctx.speed("final"), stop=True, label="dock")
    st2 = yield from hold(ctx, pose, timeout=12, label="dock: hold")
    return {"status": "done" if st == "done" else st, "hold": st2}


# ----------------------------------------------------------------------------- challenges 5 / 6 / 7
def fresh_target(ctx, target_id, max_age=0.8, gate=2.5):
    """Latest camera detection of the target (world position computed with the *current* pose, so
    it shares the GPS bias of the pose: relative geometry stays exact even with plain GPS)."""
    from ..perception.mapping import COMPATIBLE
    if ctx.bmap.last_t is None or ctx.t - ctx.bmap.last_t > max_age:
        return None
    tr = ctx.bmap.get(target_id)
    labels = {tr.label} | (COMPATIBLE.get(tr.label, set()) if tr.label != "target" else set())
    best = None
    for d in ctx.bmap.last_detections:
        if d.label in labels and d.range_source != "depth_clipped":   # clipped big targets are biased
            dist = math.hypot(d.x - tr.x, d.y - tr.y)
            if dist < gate and (best is None or dist < best[0]):
                best = (dist, (d.x, d.y))
    return best[1] if best else None


def _pose_for(ctx, kind, target_xy, heading):
    from ..planning.approach import desired_offset
    bx, by = desired_offset(ctx.cfg, kind)
    c, s = math.cos(heading), math.sin(heading)
    return (target_xy[0] - (c * bx - s * by), target_xy[1] - (s * bx + c * by), heading)


class RelTarget:
    """Target position kept RELATIVE to the boat: refreshed by camera fixes (same pose frame) and
    propagated with the boat's own motion (surge/sway/yaw rate) when the target is out of view.
    GPS bias therefore cancels in the final approach and hold (the target sits abeam, outside a
    forward camera's field of view, during the hold)."""

    def __init__(self, ctx, target_id, world_xy):
        st = ctx.state
        self.id = target_id
        self.rel = G.world_to_body(st.x, st.y, st.heading, *world_xy)
        self.t = ctx.t
        self.fixes = 0

    def world(self, ctx):
        st = ctx.state
        dt = max(0.0, ctx.t - self.t)
        self.t = ctx.t
        x, y = self.rel[0] - st.u * dt, self.rel[1] - st.v * dt
        a = -st.r * dt
        self.rel = (math.cos(a) * x - math.sin(a) * y, math.sin(a) * x + math.cos(a) * y)
        ft = fresh_target(ctx, self.id)
        if ft is not None:
            m = G.world_to_body(st.x, st.y, st.heading, *ft)
            self.rel = (0.7 * self.rel[0] + 0.3 * m[0], 0.7 * self.rel[1] + 0.3 * m[1])
            self.fixes += 1
        return G.body_to_world(st.x, st.y, st.heading, *self.rel)


def final_leg(ctx, kind, target_id, appr, rel: RelTarget):
    """Straight final leg along the approach heading, re-anchored to the relative target so a GPS
    offset does not become a lateral error at the stand-off point."""
    th = appr.heading
    pose = appr.pose
    ctx.phase = f"{kind}: final leg"
    path = None
    while True:
        new_pose = _pose_for(ctx, kind, rel.world(ctx), th)
        if path is None or math.hypot(new_pose[0] - pose[0], new_pose[1] - pose[1]) > 0.12:
            pose = new_pose
            along = (pose[0] - ctx.state.x) * math.cos(th) + (pose[1] - ctx.state.y) * math.sin(th)
            start = (pose[0] - max(along, 1.0) * math.cos(th), pose[1] - max(along, 1.0) * math.sin(th))
            path = G.Path([(ctx.state.x, ctx.state.y), start, pose[:2]], ctx.speed("approach"), name=f"{kind} final")
            path = path.with_stop_profile(ctx.cfg["control"]["decel_mps2"])
            ctx.pp.set_path(path, stop_at_end=True)
            ctx.plan_path = path
        l, r = ctx.pp.update(ctx.state, ctx.dt)
        if ctx.pp.info.done:
            return
        yield l, r


def hold_locked(ctx, kind, rel: RelTarget, heading, timeout=30.0):
    """Station keeping whose set-point is the relative target (see RelTarget)."""
    ctx.phase = f"{kind}: hold"
    ctx.hold_ctl.reset()
    t0 = ctx.t
    while ctx.t - t0 < timeout:
        pose = _pose_for(ctx, kind, rel.world(ctx), heading)
        tol = ctx.cfg["mechanisms"].get(kind, {}).get("heading_tol_deg")
        l, r, info = ctx.hold_ctl.update(ctx.state, pose, ctx.dt, tol)
        if ctx.hold_ctl.settled:
            return "settled"
        if info["reapproach"] and ctx.t - t0 > 3.0:
            return "reapproach"
        yield l, r
    return "timeout"


def standoff_task(ctx, kind, target_id, area_buoy=None, retries=2):
    for attempt in range(retries + 1):
        tr = ctx.bmap.get(target_id)
        objs = ctx.objects()
        exclude = (target_id,) if kind == "recover" else ()
        appr = plan_standoff(kind, (tr.x, tr.y), ctx.pose(), objs, ctx.planner, ctx.cfg, exclude=exclude,
                             area_buoy=ctx.bmap.position(area_buoy) if area_buoy else None,
                             env_upstream=ctx.env_upstream)
        if appr is None:
            ctx.log(f"{kind}: no feasible stand-off pose around {target_id}", "error")
            return {"status": "failed"}
        ctx.approach = appr.as_dict()
        ctx.log(f"{kind}: stand-off at ({appr.pose[0]:.1f}, {appr.pose[1]:.1f}) heading "
                f"{math.degrees(appr.heading):.0f} deg, target abeam starboard {appr.note}")
        st = yield from transit_to(ctx, appr.pre, exclude=exclude or (target_id,), stop=False,
                                   final_speed=ctx.speed("approach"), label=f"{kind}: approach")
        if st != "done":
            return {"status": st}
        tr = ctx.bmap.get(target_id)
        rel = RelTarget(ctx, target_id, fresh_target(ctx, target_id) or (tr.x, tr.y))
        yield from final_leg(ctx, kind, target_id, appr, rel)
        hs = yield from hold_locked(ctx, kind, rel, appr.heading)
        if hs == "settled":
            ctx.phase = f"{kind}: act"
            res = ctx.hal.actuate(kind)
            res["attempt"] = attempt
            ctx.actions.append(res)
            if ctx.on_action:
                ctx.on_action(res)
            ok = res.get("success", res.get("ok"))
            ctx.log(f"{kind}: actuator fired -> {'success' if ok else 'not verified'} "
                    + (f"({res.get('distance_m', 0):.2f} m)" if "distance_m" in res else ""),
                    "info" if ok else "warn")
            if kind == "recover" and res.get("success"):
                ctx.bmap.remove(target_id)
            yield from idle(ctx, 0.8, f"{kind}: verify")
            yield from depart(ctx)
            return {"status": "done", "result": res}
        ctx.log(f"{kind}: hold {hs} (lateral error) - re-approaching", "warn")
        yield from back_off(ctx, 3.0, exclude=(target_id,) if kind == "recover" else ())
    return {"status": "failed"}


def deploy_task(ctx, **_):
    return (yield from standoff_task(ctx, "deploy", ctx.course["challenges"]["5_deploy"]["target"]))


def launch_task(ctx, **_):
    ch = ctx.course["challenges"]["6_launch"]
    return (yield from standoff_task(ctx, "launch", ch["target"], area_buoy=ch["area_buoy"]))


def recover_task(ctx, **_):
    return (yield from standoff_task(ctx, "recover", ctx.course["challenges"]["7_recover"]["target"]))


# ----------------------------------------------------------------------------- generic
def waypoints_task(ctx, points=None, latlon=None, speed=None, stop=True, **_):
    """Pure pursuit through GPS (lat/lon) or local (x, y) waypoints, obstacle-aware between points."""
    pts = [tuple(p) for p in (points or [])]
    if latlon:
        pts = [ctx.frame.to_local(la, lo) for la, lo in latlon]
    speed = speed or ctx.speed("transit")
    route = [(ctx.state.x, ctx.state.y)]
    start = (ctx.state.x, ctx.state.y)
    for p in pts:
        obst = list(ctx.objects().values())
        seg = ctx.planner.plan(route[-1], p, obst)
        if seg is None:
            ctx.log(f"waypoints: {p} unreachable - skipped", "warn")
            continue
        route += [tuple(q) for q in seg[1:]]
    if len(route) < 2:
        return {"status": "failed"}
    path = G.Path(route, speed, name="waypoints")
    path = path.with_stop_profile(ctx.cfg["control"]["decel_mps2"]) if stop else path.resampled(0.4)
    ctx.log(f"Waypoints: {len(pts)} points, route {path.length:.1f} m from ({start[0]:.1f}, {start[1]:.1f})")
    st = yield from follow(ctx, path, stop=stop, timeout=600, label="waypoints")
    return {"status": st}


def hold_task(ctx, seconds=5.0, **_):
    pose = ctx.pose()
    t0 = ctx.t
    while ctx.t - t0 < seconds:
        l, r, _ = ctx.hold_ctl.update(ctx.state, pose, ctx.dt)
        yield l, r
    return {"status": "done"}


REGISTRY = {
    "gate": (gate_task, 1), "slalom": (slalom_task, 2), "dodge": (slalom_task, 2), "channel": (channel_task, 3),
    "evade": (channel_task, 3), "identify": (identify_task, 4), "deploy": (deploy_task, 5), "launch": (launch_task, 6),
    "recover": (recover_task, 7), "return": (return_task, 9), "dock": (dock_task, None),
    "waypoints": (waypoints_task, None), "hold": (hold_task, None),
}
