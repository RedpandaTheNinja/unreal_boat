"""Twin-motor boat model shared by the Python twin simulator and the controller feed-forward.

The equations mirror Plugins/BoatPhysics/.../BoatDynamicsComponent.cpp (planar part):
    (m + Xa) du = Fx + (m + Ya) v r            (rigid-body terms appear through world-frame integration)
    (m + Ya) dv = Fy - (m + Xa) u r
    (Iz + Na) dr = N - (Ya - Xa) u v           (Munk moment)
with directional linear + quadratic drag on water-relative velocity, first-order
motor lag, command delay, deadband, 60 % reverse thrust and forward-speed unloading.
All in body FLU (x forward, y port, r counter-clockwise).
"""
from __future__ import annotations

import math


class BoatModel:
    def __init__(self, boat_cfg: dict):
        d, m = boat_cfg["dynamics"], boat_cfg["motors"]
        self.mass = d["mass_kg"]
        self.xc = d.get("com_x_m", 0.0)
        self.iz = d["inertia_z_kgm2"]
        self.max_ = (d["added_mass_x_kg"], d["added_mass_y_kg"])
        self.naz = d["added_inertia_z_kgm2"]
        self.lin = d["linear_drag"]
        self.quad = d["quadratic_drag"]
        self.ylin = d["yaw_linear_drag"]
        self.yquad = d["yaw_quadratic_drag"]
        self.wind_area = d.get("wind_area_m2", [0.3, 1.2])
        self.wind_cx = d.get("wind_center_x_m", 0.0)
        self.spacing = m["spacing_m"]
        self.motor_x = m["x_m"]
        self.fmax = m["max_forward_n"]
        self.rmax = m["max_reverse_n"]
        self.deadband = m["deadband"]
        self.tau = m["time_constant_s"]
        self.delay = m["command_delay_s"]
        self.timeout = m["command_timeout_s"]
        self.unload_k = m["unload_per_mps"]

    # ---------------------------------------------------------------- motors
    def motor_curve(self, cmd: float) -> float:
        """Steady thrust (N) for a command in [-1, 1]; identical to UBoatDynamicsComponent::MotorCurve."""
        if abs(cmd) <= self.deadband:
            return 0.0
        a = (abs(cmd) - self.deadband) / (1 - self.deadband)
        return (self.fmax if cmd > 0 else -self.rmax) * min(a, 1.0)

    def cmd_for_thrust(self, f: float) -> float:
        if abs(f) < 1e-6:
            return 0.0
        full = self.fmax if f > 0 else self.rmax
        a = min(abs(f) / full, 1.0)
        return math.copysign(self.deadband + (1 - self.deadband) * a, f)

    def unload(self, thrust: float, u_rel: float) -> float:
        return min(1.0, max(0.2, 1 - self.unload_k * max(0.0, math.copysign(1, thrust) * u_rel)))

    # ---------------------------------------------------------------- hydrodynamics
    def surge_drag(self, u: float) -> float:
        return self.lin[0] * u + self.quad[0] * u * abs(u)

    def yaw_drag(self, r: float) -> float:
        return self.ylin * r + self.yquad * r * abs(r)

    def steady_speed(self, cmd_both: float) -> float:
        """Straight-line steady speed for equal commands (bisection)."""
        f = 2 * self.motor_curve(cmd_both)
        lo, hi = -3.0, 3.0
        for _ in range(60):
            u = (lo + hi) / 2
            net = f * self.unload(f, u) - self.surge_drag(u)
            lo, hi = (u, hi) if net > 0 else (lo, u)
        return (lo + hi) / 2

    def feedforward(self, u_des: float, r_des: float) -> tuple[float, float]:
        """Motor commands (left, right) that hold surge u_des and yaw rate r_des in calm water."""
        total = self.surge_drag(u_des)
        if abs(total) > 1e-9:
            total /= self.unload(total, u_des)
        diff = 2 * self.yaw_drag(r_des) / self.spacing      # F_R - F_L
        fl, fr = (total - diff) / 2, (total + diff) / 2
        return self.cmd_for_thrust(fl), self.cmd_for_thrust(fr)

    def derivatives(self, u, v, r, fl, fr, cur_b=(0.0, 0.0), wind_b=(0.0, 0.0)):
        """Body accelerations (du, dv, dr) including kinematic terms, for delivered thrusts fl, fr (N)."""
        ur, vr = u - cur_b[0], v - cur_b[1]
        fl *= self.unload(fl, ur)
        fr *= self.unload(fr, ur)
        fx = fl + fr - self.surge_drag(ur)
        fy = -(self.lin[1] * vr + self.quad[1] * vr * abs(vr))
        n = (self.spacing / 2) * (fr - fl) - self.yaw_drag(r)
        # Wind (relative air velocity in body frame), applied at wind centre.
        wx, wy = wind_b[0] - u, wind_b[1] - v
        fwx = 0.5 * 1.225 * self.wind_area[0] * wx * abs(wx)
        fwy = 0.5 * 1.225 * self.wind_area[1] * wy * abs(wy)
        fx += fwx
        fy += fwy
        n += (self.wind_cx - self.xc) * fwy
        max_, may_ = self.max_
        ax = (fx + r * vr * (may_ - max_)) / (self.mass + max_)
        ay = (fy + r * ur * (may_ - max_)) / (self.mass + may_)
        dr = (n - ur * vr * (may_ - max_)) / (self.iz + self.naz)
        # body-frame derivative = world accel expressed in body minus omega x v
        return ax + r * v, ay - r * u, dr
