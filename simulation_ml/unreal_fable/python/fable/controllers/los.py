"""
Line-of-sight guidance + heading PID + thrust allocation for the twin-thruster boat.

LOS is the standard marine path-following law (Fossen): aim at a point on the
path a lookahead distance ahead of your cross-track foot, with the lookahead
shrinking as cross-track error grows, so the boat converges without overshoot
even with current pushing it sideways. The integral term in the heading loop
is what beats a steady cross-wind - leave it in.

Tuning order: (1) kp_heading until it turns briskly without oscillating,
(2) lookahead so it settles onto the line, (3) ki_heading for the wind.
"""

from __future__ import annotations

import math

import numpy as np

from ..geometry import Polyline
from ..specs import SceneSpec, VehicleParams
from ..types import Observation, clamp, wrap
from .base import Controller
from .pid import PID


def heading_gains_from_params(vehicle: VehicleParams, wn: float = 1.2, zeta: float = 0.9) -> tuple[float, float, float]:
    """
    Pole-place the heading loop from the vehicle's own numbers.

    Yaw plant (linearised):  Mr * r_dot = N_max * yaw_cmd - Nr * r
    with yaw_cmd = kp*e - kd*r  ->  wn^2 = N_max*kp/Mr,  2*zeta*wn = (N_max*kd + Nr)/Mr

    This is the whole point of measuring the boat: once yaw inertia, yaw drag
    and thrust are calibrated, the controller re-tunes itself. Hand-tuned gains
    that worked on Tuesday stop working when you add a battery.
    """
    b = vehicle.boat
    Mr = vehicle.yaw_inertia + float(b.get("added_inertia_yaw_kgm2", 0.0))
    Nr = float(b.get("drag_yaw_lin", 1.0))
    N_max = (float(b["max_thrust_fwd_n"]) + float(b["max_thrust_rev_n"])) * float(b["thruster_offset_y_m"])
    kp = wn * wn * Mr / N_max
    kd = max((2 * zeta * wn * Mr - Nr) / N_max, 0.0)
    ki = kp * wn / 12.0
    return kp, ki, kd


def surge_gains_from_params(vehicle: VehicleParams, tau: float = 1.5) -> tuple[float, float]:
    """First-order speed loop with closed-loop time constant `tau` seconds."""
    b = vehicle.boat
    Mu = float(vehicle.common["mass_kg"]) + float(b.get("added_mass_surge_kg", 0.0))
    X_max = 2.0 * float(b["max_thrust_fwd_n"])
    kp = Mu / (tau * X_max)
    ki = kp / (2.0 * tau)
    return kp, ki


def allocate_thrust(surge_cmd: float, yaw_cmd: float, deadband: float = 0.0) -> dict[str, float]:
    """
    Map (surge in [-1,1], yaw in [-1,1]) to left/right thruster commands with
    priority to yaw: if the sum would saturate, surge is reduced, never yaw.
    Positive yaw = turn left (CCW) = more right thrust.

    `deadband` is the ESC's: commands below it produce no thrust at all. Without
    compensation the boat has NO yaw authority at low speed, which is exactly
    when docking needs it. Pass vehicle.boat["thrust_deadband"].
    """
    l, r = surge_cmd - yaw_cmd, surge_cmd + yaw_cmd
    m = max(abs(l), abs(r), 1.0)
    l, r = l / m, r / m
    if deadband > 0:
        l = 0.0 if abs(l) < 1e-3 else math.copysign(deadband + abs(l) * (1 - deadband), l)
        r = 0.0 if abs(r) < 1e-3 else math.copysign(deadband + abs(r) * (1 - deadband), r)
    return {"thrust_l": clamp(l), "thrust_r": clamp(r)}


class LOSGuidance(Controller):
    """
    Integral LOS (Fossen & Lekkas, 2014):

        chi_d = gamma_p - atan((e + kappa*e_int) / delta)
        e_int_dot = delta * e / (delta^2 + (e + kappa*e_int)^2)

    e is cross-track error, gamma_p the path tangent, delta the lookahead. The
    integral state is a current/wind estimator in disguise: it converges to the
    crab angle that cancels the drift, using nothing but position - which is
    what a real boat with GPS and a compass actually has.

    Two things that were tried and removed, so you do not re-add them:
      * sway-velocity crab compensation - the boat's own turning induces sway,
        which the compensation reads as drift, and the boat spirals;
      * a fast course-over-ground integrator - any adaptation faster than the
        heading loop's settling time (~3/wn) oscillates.

    Surge has a drag feedforward from vehicle_params so the speed loop only
    corrects model error rather than carrying the whole load.
    """

    def __init__(self, scene: SceneSpec, vehicle: VehicleParams, path=None,
                 speed: float | None = None, lookahead: float = 4.0, lookahead_min: float = 1.5,
                 kappa: float = 0.4,
                 kp_heading: float | None = None, ki_heading: float | None = None, kd_heading: float | None = None,
                 kp_speed: float | None = None, ki_speed: float | None = None, slow_radius: float = 6.0,
                 stop_at_end: bool | None = None, wn: float = 1.2, zeta: float = 0.9):
        pts = path if path is not None else (scene.centerline or scene.task.get("waypoints_m"))
        self.path = Polyline(pts, False)
        self.vehicle = vehicle
        self.v_ref = speed if speed is not None else 0.6 * float(vehicle.boat.get("max_speed_mps", 2.0))
        self.la, self.la_min, self.kappa = lookahead, lookahead_min, kappa
        kp, ki, kd = heading_gains_from_params(vehicle, wn, zeta)
        kps, kis = surge_gains_from_params(vehicle)
        self.gains = {"kp_heading": kp_heading if kp_heading is not None else kp,
                      "ki_heading": ki_heading if ki_heading is not None else ki,
                      "kd_heading": kd_heading if kd_heading is not None else kd,
                      "kp_speed": kp_speed if kp_speed is not None else kps,
                      "ki_speed": ki_speed if ki_speed is not None else kis}
        g = self.gains
        self.heading_pid = PID(g["kp_heading"], g["ki_heading"], g["kd_heading"], -1.0, 1.0, i_limit=0.6)
        self.speed_pid = PID(g["kp_speed"], g["ki_speed"], 0.0, -1.0, 1.0, i_limit=0.5)
        self.slow_radius = slow_radius
        self.stop_at_end = True if stop_at_end is None else stop_at_end
        b = vehicle.boat
        self.X_max = 2.0 * float(b["max_thrust_fwd_n"])
        self.Xu, self.Xuu = float(b.get("drag_surge_lin", 0)), float(b.get("drag_surge_quad", 0))
        self.deadband = float(b.get("thrust_deadband", 0.0))
        self.reset()

    def reset(self) -> None:
        self.heading_pid.reset()
        self.speed_pid.reset()
        self.e_int = 0.0

    def desired_heading(self, obs: Observation) -> tuple[float, float]:
        """Returns (desired heading, arc length s along the path)."""
        xy = obs.pose.xy
        s, e, _ = self.path.project(xy)
        start = self.path.pts[0]
        if s < 1e-6 and np.linalg.norm(xy - start) > self.la:
            return math.atan2(start[1] - xy[1], start[0] - xy[0]), s   # not on the path yet
        la = max(self.la_min, self.la - 0.5 * abs(e))
        gamma = self.path.heading_at(s + 0.5)
        ei = e + self.kappa * self.e_int
        self.e_int = clamp(self.e_int + obs.dt * la * e / (la * la + ei * ei), -3.0 * la, 3.0 * la)
        return wrap(gamma - math.atan2(ei, la)), s

    def __call__(self, obs: Observation) -> dict[str, float]:
        psi_d, s = self.desired_heading(obs)
        e_psi = wrap(psi_d - obs.pose.yaw)
        yaw_cmd = self.heading_pid(e_psi, obs.dt, e_dot=-obs.vel.wz)

        v_ref = self.v_ref
        if self.stop_at_end:
            remaining = self.path.length - s
            v_ref = min(v_ref, self.v_ref * clamp(remaining / self.slow_radius, 0.0, 1.0))
        v_ref *= max(0.3, math.cos(e_psi))                      # slow down when pointing wrong
        ff = (self.Xu * v_ref + self.Xuu * v_ref * abs(v_ref)) / self.X_max
        surge = clamp(ff + self.speed_pid(v_ref - obs.vel.vx, obs.dt))
        return allocate_thrust(surge, yaw_cmd, self.deadband)


class DockingController(Controller):
    """
    Two phases.
      A - approach: ILOS along [spawn -> approach point -> slip], speed scheduled
          down with distance so the boat arrives at the slip mouth at creep speed.
      B - slip: heading HOLD on the slip axis (no crabbing - there is no room),
          surge PID on the remaining distance, then station-keep.

    Cross-wind in the slip cannot be cancelled by twin fixed thrusters; the
    boat will lean on the leeward fender. The scene scores that as a contact,
    not a failure, because that is what a real fender is for. What IS a failure
    is arriving fast, arriving crooked, or never settling.
    """

    def __init__(self, scene: SceneSpec, vehicle: VehicleParams, creep_speed: float = 0.3,
                 approach_speed: float = 0.9):
        d = scene.dock or {}
        self.dock_xy = np.asarray(d.get("xy_m", [0, 0]), float)
        self.dock_yaw = float(d.get("yaw_rad", 0.0))
        self.L = float(d.get("slip_length_m", 3.0))
        self.ax = np.array([math.cos(self.dock_yaw), math.sin(self.dock_yaw)])
        self.approach = self.dock_xy - self.ax * (3.0 * self.L + 2.0)
        spawn = np.asarray(scene.spawn_xy, float)
        self.los = LOSGuidance(scene, vehicle, path=[spawn.tolist(), self.approach.tolist(), (self.dock_xy + self.ax * 0.5).tolist()],
                               speed=approach_speed, lookahead=4.5, lookahead_min=2.0, kappa=0.4, stop_at_end=False, wn=1.2)
        self.creep, self.v_fast = creep_speed, approach_speed
        kp, ki, kd = heading_gains_from_params(vehicle, wn=1.0, zeta=1.0)
        self.heading_pid = PID(kp, ki, kd, i_limit=0.4)
        kps, kis = surge_gains_from_params(vehicle, tau=2.0)
        self.pos_pid = PID(kps * 2.0, kis, kps * 3.0, -0.6, 0.6, i_limit=0.3)
        self.deadband = float(vehicle.boat.get("thrust_deadband", 0.0))
        self.Xu, self.Xuu = float(vehicle.boat.get("drag_surge_lin", 0)), float(vehicle.boat.get("drag_surge_quad", 0))
        self.X_max = 2.0 * float(vehicle.boat["max_thrust_fwd_n"])
        self.phase = "A"

    def reset(self) -> None:
        self.phase = "A"
        self.los.reset(); self.heading_pid.reset(); self.pos_pid.reset()

    def __call__(self, obs: Observation) -> dict[str, float]:
        xy = obs.pose.xy
        rel = self.dock_xy - xy
        along = float(np.dot(rel, self.ax))                        # + = dock still ahead
        cross = float(rel[0] * -self.ax[1] + rel[1] * self.ax[0])   # + = dock to the left

        if self.phase == "A":
            if along < self.L + 1.0 and abs(cross) < 0.6 and abs(wrap(self.dock_yaw - obs.pose.yaw)) < 0.35:
                self.phase = "B"
            else:
                dist = float(np.linalg.norm(rel))
                self.los.v_ref = float(clamp(0.12 * dist, self.creep, self.v_fast))
                return self.los(obs)

        # phase B: heading hold + surge on distance
        yaw_cmd = self.heading_pid(wrap(self.dock_yaw - obs.pose.yaw), obs.dt, e_dot=-obs.vel.wz)
        v_ref = clamp(0.25 * along, -0.15, self.creep)
        ff = (self.Xu * v_ref + self.Xuu * v_ref * abs(v_ref)) / self.X_max
        surge = clamp(ff + self.pos_pid(v_ref - obs.vel.vx, obs.dt), -0.6, 0.6)
        return allocate_thrust(surge, yaw_cmd, self.deadband)
