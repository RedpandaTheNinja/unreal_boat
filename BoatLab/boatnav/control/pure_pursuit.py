"""Pure pursuit path tracking for a twin-motor (skid-steer) boat.

Geometry (classic pure pursuit):
    look-ahead point P on the path at arc length s + Ld,
    alpha = bearing of P in the body frame,
    curvature k = 2 sin(alpha) / Ld,  desired yaw rate r* = v * k.

Differential thrust:
    common-mode c (surge) and differential d (yaw) are computed from the boat model's
    feed-forward plus PI/P feedback, then mixed:  left = c - d,  right = c + d.
    Positive d turns the boat counter-clockwise (to port) because the starboard motor
    pushes harder.

Pivot turns: when |alpha| exceeds pivot_enter the boat stops advancing and spins in place
with one motor forward and the other in reverse (clockwise: left fwd + right rev;
counter-clockwise: right fwd + left rev), until |alpha| < pivot_exit.

Reverse legs (path.reverse = True): the stern leads; the same law is applied to the
stern heading and the motors run in reverse.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .. import geometry as G
from ..model import BoatModel
from ..types import BoatState


@dataclass
class PPInfo:
    mode: str = "idle"
    s: float = 0.0
    length: float = 0.0
    cross_track: float = 0.0
    alpha: float = 0.0
    lookahead: tuple = (0.0, 0.0)
    lookahead_m: float = 0.0
    u_des: float = 0.0
    r_des: float = 0.0
    remaining: float = 0.0
    done: bool = False
    extra: dict = field(default_factory=dict)


def mix(common: float, diff: float, limit: float = 1.0) -> tuple[float, float]:
    """left = c - d, right = c + d with yaw priority under saturation."""
    diff = max(-limit, min(limit, diff))
    room = limit - abs(diff)
    common = max(-room, min(room, common))
    return common - diff, common + diff


class PurePursuit:
    def __init__(self, model: BoatModel, ctrl_cfg: dict):
        self.m = model
        self.c = ctrl_cfg
        self.path: G.Path | None = None
        self.s = 0.0
        self.i_speed = 0.0
        self.pivot = False
        self.stop_at_end = True
        self.info = PPInfo()

    def set_path(self, path: G.Path, stop_at_end: bool = True):
        self.path = path
        self.s = 0.0
        self.i_speed = 0.0
        self.pivot = False
        self.stop_at_end = stop_at_end
        self.info = PPInfo(mode="track", length=path.length)

    def update(self, st: BoatState, dt: float) -> tuple[float, float]:
        p = self.path
        if p is None:
            self.info = PPInfo()
            return 0.0, 0.0
        c = self.c
        proj = p.project((st.x, st.y), self.s, window=max(2.0, 2.5 * abs(st.u) + 1.5))
        self.s = max(self.s, proj.s)
        remaining = p.length - self.s
        speed = abs(st.u)
        Ld = min(c["lookahead_max_m"], max(c["lookahead_min_m"], c["lookahead_time_s"] * speed + c["lookahead_min_m"] * 0.5))
        target_s = self.s + Ld
        if target_s <= p.length:
            la = p.point_at(target_s)
        else:  # extend beyond the end along the final tangent so the boat does not orbit the last point
            end = p.point_at(p.length)
            hd = p.heading_at(p.length)
            extra = target_s - p.length if not self.stop_at_end else 0.0
            la = (end[0] + extra * math.cos(hd), end[1] + extra * math.sin(hd))
        dist_la = math.hypot(la[0] - st.x, la[1] - st.y)
        heading_eff = st.heading + (math.pi if p.reverse else 0.0)
        alpha = G.wrap(math.atan2(la[1] - st.y, la[0] - st.x) - heading_eff)
        u_path = p.speed_at(self.s)
        done = False
        if self.stop_at_end:
            to_end = math.hypot(p.points[-1][0] - st.x, p.points[-1][1] - st.y)
            if remaining < 0.25 or to_end < 0.35:
                done = True
            u_path = min(u_path, math.sqrt(2 * c["decel_mps2"] * max(0.0, remaining)) + 0.05)
        else:
            if remaining < max(0.8, Ld * 0.5):
                done = True

        # --- pivot / normal mode selection (hysteresis)
        if not self.pivot and abs(alpha) > math.radians(c["pivot_enter_deg"]) and dist_la > 0.8:
            self.pivot = True
        elif self.pivot and abs(alpha) < math.radians(c["pivot_exit_deg"]) and abs(st.r) < 0.25:
            self.pivot = False

        if self.pivot:
            # spin in place: heading PD on alpha, surge held near zero (one motor forward, one reverse)
            d = c["kp_heading_pivot"] * alpha - c["kd_heading_pivot"] * st.r
            d = max(-c["pivot_max_cmd"], min(c["pivot_max_cmd"], d))
            common = -0.25 * st.u               # bleed off surge while spinning
            left, right = mix(common, d)
            mode, u_des, r_des = "pivot", 0.0, 0.0
        else:
            v_sign = -1.0 if p.reverse else 1.0
            u_mag = u_path * max(0.35, math.cos(alpha) ** 2)
            u_des = v_sign * u_mag
            # curvature feed-forward from the path itself (anticipates bends, zero on straights)
            s_ff = min(self.s + Ld, p.length)
            kappa_path = G.wrap(p.heading_at(s_ff) - p.heading_at(self.s)) / max(Ld, 0.5)
            r_ff = max(speed, 0.2) * kappa_path
            c_ff, d_ff = self._ff(u_des, r_ff)
            err_u = u_des - st.u
            self.i_speed = max(-0.3, min(0.3, self.i_speed + err_u * dt))
            common = c_ff + c["kp_speed"] * err_u + c["ki_speed"] * self.i_speed
            # heading PD toward the look-ahead point: kp on the pure-pursuit angle, kd on yaw rate
            d = d_ff + c["kp_heading"] * alpha - c["kd_heading"] * st.r
            dmax = c.get("turn_max_cmd", 0.9)
            d = max(-dmax, min(dmax, d))
            left, right = mix(common, d)
            r_des = r_ff + c["kp_heading"] / max(c["kd_heading"], 1e-3) * alpha
            mode = "reverse" if p.reverse else "track"
        if done and self.stop_at_end:
            left, right = mix(-0.6 * st.u, -0.3 * st.r)   # active brake using reverse thrust
        self.info = PPInfo(mode=mode, s=self.s, length=p.length, cross_track=proj.lateral, alpha=alpha,
                           lookahead=(float(la[0]), float(la[1])), lookahead_m=Ld, u_des=u_des, r_des=r_des,
                           remaining=remaining, done=done)
        return left, right

    def _ff(self, u_des: float, r_des: float) -> tuple[float, float]:
        fl, fr = self.m.feedforward(u_des, r_des)
        return (fl + fr) / 2, (fr - fl) / 2
