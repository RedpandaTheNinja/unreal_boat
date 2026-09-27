"""Reusable behaviours written as generators.

Each behaviour yields (left, right) motor commands once per control tick and `return`s a
status string. Tasks compose them with `yield from`, which keeps mission logic readable:

    status = yield from transit_to(ctx, goal)
    status = yield from hold(ctx, pose)
"""
from __future__ import annotations

import math

import numpy as np

from .. import geometry as G


def make_path(points, speed, *, stop=True, decel=0.2, reverse=False, name="") -> G.Path:
    p = G.Path(points, speed, name=name, reverse=reverse)
    return p.with_stop_profile(decel) if stop else p.resampled(0.4)


def follow(ctx, path: G.Path, *, stop=True, timeout=None, label="follow", check_blocked=True, exclude=()):
    """Pure pursuit along `path`. Returns 'done' | 'timeout' | 'blocked'."""
    ctx.pp.set_path(path, stop_at_end=stop)
    ctx.plan_path = path
    ctx.phase = label
    t0 = ctx.t
    last_check = ctx.t
    while True:
        l, r = ctx.pp.update(ctx.state, ctx.dt)
        if ctx.pp.info.done:
            if stop:
                # brief active brake until nearly stopped
                t1 = ctx.t
                while ctx.state.speed > 0.08 and ctx.t - t1 < 4.0:
                    yield G_mix_brake(ctx)
            return "done"
        if timeout is not None and ctx.t - t0 > timeout:
            ctx.log(f"{label}: timeout after {timeout:.0f}s", "warn")
            return "timeout"
        if check_blocked and ctx.t - last_check > 1.0:
            last_check = ctx.t
            if path_blocked(ctx, path, ctx.pp.s, ahead=10.0, exclude=exclude):
                ctx.log(f"{label}: path blocked by a newly detected object - replanning", "warn")
                return "blocked"
        yield l, r


def G_mix_brake(ctx):
    from ..control.pure_pursuit import mix
    st = ctx.state
    return mix(max(-0.5, min(0.5, -0.9 * st.u)), max(-0.3, min(0.3, -0.6 * st.r)))


def path_blocked(ctx, path: G.Path, s0: float, ahead: float = 10.0, exclude=()) -> bool:
    """True when an unexpected (non-prior) confirmed object sits on the next `ahead` metres.
    Objects within 1.5 m of an excluded (target) object are ignored - they are usually that target."""
    news = ctx.bmap.unexpected()
    if exclude:
        tx = [ctx.bmap.position(e) for e in exclude if e in ctx.bmap.tracks]
        news = [n for n in news if all(math.hypot(n.x - a, n.y - b) > 1.5 for a, b in tx)]
    if not news:
        return False
    hb = ctx.cfg["hull"]["beam_m"] / 2
    for s in np.arange(s0, min(path.length, s0 + ahead), 0.5):
        p = path.point_at(s)
        for tr in news:
            if math.hypot(tr.x - p[0], tr.y - p[1]) < hb + 0.4:
                return True
    return False


def idle(ctx, seconds: float, label="idle"):
    ctx.phase = label
    t0 = ctx.t
    while ctx.t - t0 < seconds:
        yield 0.0, 0.0


def observe(ctx, seconds: float, label="observe"):
    """Slow to a stop and look (perception keeps running in the runner)."""
    ctx.phase = label
    t0 = ctx.t
    while ctx.t - t0 < seconds:
        yield G_mix_brake(ctx)


def pivot_to(ctx, heading: float, tol_deg=8.0, timeout=20.0):
    """Spin in place (one motor forward, the other reverse) until aligned."""
    from ..control.pure_pursuit import mix
    c = ctx.cfg["control"]
    ctx.phase = "pivot"
    t0 = ctx.t
    while ctx.t - t0 < timeout:
        e = G.wrap(heading - ctx.state.heading)
        if abs(e) < math.radians(tol_deg) and abs(ctx.state.r) < 0.08:
            return "done"
        d = c["kp_heading_pivot"] * e - c["kd_heading_pivot"] * ctx.state.r
        d = max(-c["pivot_max_cmd"], min(c["pivot_max_cmd"], d * 0.6))
        yield mix(-0.25 * ctx.state.u, d)
    return "timeout"


def plan_points(ctx, goal, *, exclude=(), detectors=(), smooth=2.0, free_goal_r=0.0):
    obst = [v for k, v in ctx.objects().items() if k not in exclude]
    start = (ctx.state.x, ctx.state.y)
    pts = ctx.planner.plan(start, tuple(goal), obst, detectors, smooth_radius=smooth, free_goal_r=free_goal_r)
    return pts


def turn_room(ctx, exclude=()):
    """Clearance between the hull's turning circle (radius L/2) and the nearest object."""
    L = ctx.cfg["hull"]["length_m"]
    x, y, _ = ctx.pose()
    return min((math.hypot(ox - x, oy - y) - r - L / 2 for k, (ox, oy, r) in ctx.objects().items() if k not in exclude),
               default=9.0)


def ensure_turn_room(ctx, heading_needed: float, exclude=(), need=0.35, max_move=4.0):
    """Before a large pivot next to buoys, slide straight forward or backward until the bow and
    stern can swing without touching anything (fixed thrusters cannot move sideways)."""
    if abs(G.wrap(heading_needed - ctx.state.heading)) < math.radians(40) or turn_room(ctx, exclude) >= need:
        return "ok"
    L, B = ctx.cfg["hull"]["length_m"], ctx.cfg["hull"]["beam_m"]
    x, y, h = ctx.pose()
    objs = {k: v for k, v in ctx.objects().items() if k not in exclude}
    best = None
    for sign in (1.0, -1.0):
        for dist in np.arange(0.5, max_move + 0.01, 0.5):
            px, py = x + sign * dist * math.cos(h), y + sign * dist * math.sin(h)
            room = min((math.hypot(ox - px, oy - py) - r - L / 2 for ox, oy, r in objs.values()), default=9.0)
            swept = all(G.polygon_distance((ox, oy), G.hull_polygon(x + sign * t * math.cos(h), y + sign * t * math.sin(h), h, L, B)) > r + 0.15
                        for t in np.linspace(0, dist, 5) for ox, oy, r in objs.values())
            if not swept or not ctx.planner.free(px, py):
                break
            if room >= need:
                if best is None or dist < best[0]:
                    best = (dist, sign)
                break
    if best is None:
        ctx.log("No room to turn safely here - proceeding carefully", "warn")
        return "none"
    dist, sign = best
    ctx.log(f"Making turning room: {'forward' if sign > 0 else 'reverse'} {dist:.1f} m before pivoting")
    pts = [(x, y), (x + sign * dist * math.cos(h), y + sign * dist * math.sin(h))]
    path = make_path(pts, ctx.speed("reverse") if sign < 0 else ctx.speed("final") + 0.1, stop=True,
                     reverse=sign < 0, name="turning room")
    yield from follow(ctx, path, stop=True, timeout=20.0, label="turning room", check_blocked=False)
    return "moved"


def transit_to(ctx, goal, *, speed=None, exclude=(), stop=False, final_leg=None, final_speed=None,
               label="transit", timeout=240.0, retries=3, detectors=()):
    """A* to `goal` (optionally continuing along `final_leg` points), replanning if blocked."""
    speed = speed or ctx.speed("transit")
    for attempt in range(retries + 1):
        pts = plan_points(ctx, goal, exclude=exclude, detectors=detectors)
        if pts is not None and attempt == 0 and len(pts) > 1:
            first = math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0])
            moved = yield from ensure_turn_room(ctx, first, exclude=exclude)
            if moved == "moved":
                pts = plan_points(ctx, goal, exclude=exclude, detectors=detectors)
        if pts is None:
            ctx.log(f"{label}: no collision-free route to {tuple(round(v, 1) for v in goal)}", "error")
            yield from observe(ctx, 1.0)
            continue
        speeds = [speed] * len(pts)
        if final_leg:
            pts = list(pts) + [tuple(p) for p in final_leg]
            speeds += [final_speed or speed] * len(final_leg)
        path = G.Path(pts, speeds, name=label)
        path = path.with_stop_profile(ctx.cfg["control"]["decel_mps2"]) if stop else path.resampled(0.4)
        status = yield from follow(ctx, path, stop=stop, timeout=timeout, label=label, exclude=exclude)
        if status != "blocked":
            return status
    return "failed"


def hold(ctx, pose, timeout=25.0, label="hold"):
    """Station-keep at pose. Returns 'settled' | 'reapproach' | 'timeout'."""
    ctx.phase = label
    ctx.hold_ctl.reset()
    t0 = ctx.t
    while ctx.t - t0 < timeout:
        l, r, info = ctx.hold_ctl.update(ctx.state, pose, ctx.dt)
        if ctx.hold_ctl.settled:
            return "settled"
        if info["reapproach"] and ctx.t - t0 > 3.0:
            return "reapproach"
        yield l, r
    return "timeout"


def back_off(ctx, distance=2.5, label="back off (reverse)", exclude=()):
    """Straight reverse leg along the current heading (both motors in reverse), shortened so the
    hull never sweeps into another object."""
    x, y, h = ctx.pose()
    L, B = ctx.cfg["hull"]["length_m"], ctx.cfg["hull"]["beam_m"]
    objs = [v for k, v in ctx.objects().items() if k not in exclude]
    safe = 0.0
    for d in np.arange(0.25, distance + 1e-6, 0.25):
        hull = G.hull_polygon(x - d * math.cos(h), y - d * math.sin(h), h, L, B)
        if any(G.polygon_distance((ox, oy), hull) < r + 0.2 for ox, oy, r in objs):
            break
        safe = d
    if safe < 0.5:
        ctx.log(f"{label}: no clear water astern - skipping", "warn")
        return "skipped"
    pts = [(x, y), (x - safe * math.cos(h), y - safe * math.sin(h))]
    path = make_path(pts, ctx.speed("reverse"), stop=True, reverse=True, name=label)
    return (yield from follow(ctx, path, stop=True, timeout=25.0, label=label, check_blocked=False))


def depart(ctx, distance=4.0, label="depart"):
    x, y, h = ctx.pose()
    ahead = (x + distance * math.cos(h), y + distance * math.sin(h))
    ctx.planner.build(list(ctx.objects().values()))
    if ctx.planner.line_clear((x + 2.0 * math.cos(h), y + 2.0 * math.sin(h)), ahead):
        path = make_path([(x, y), ahead], ctx.speed("approach"), stop=False, name=label)
        return (yield from follow(ctx, path, stop=False, timeout=20.0, label=label, check_blocked=False))
    return (yield from back_off(ctx, 2.5))
