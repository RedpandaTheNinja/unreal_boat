"""
fit_boat.py - fit the twin-thruster boat model to a logged run.

What it fits, from what:

  actuator_delay_s         cross-correlation of (thrust_l + thrust_r) -> surge speed
  max_thrust_fwd_n,
  thrust_time_constant_s,
  drag_surge_lin/quad      simulation least squares on surge speed
                           (needs: full-throttle run to top speed, then thrusters to 0 and coast >30 s)
  drag_yaw_lin/quad,
  yaw_inertia_kgm2         simulation least squares on yaw rate
                           (needs: a zig-zag - alternate hard left / hard right every ~5 s at cruise)

Mass you weigh. Thruster offset you measure with a tape. The rest comes out of
a five-minute run on flat water on a calm day - calm matters, the model has no
way to know the wind was blowing.

    python -m fable.calibration.fit_boat log.csv specs/vehicles/boat_twin_thruster_default.json -o boat_measured.json
"""

from __future__ import annotations

import argparse
import json
import math
from datetime import date

import numpy as np
from scipy.optimize import least_squares

from ..specs import VehicleParams, load_vehicle
from .logs import Log, estimate_delay, load_log, shift_signal, smooth


def _thrust(cmd: np.ndarray, fwd: float, rev: float, deadband: float) -> np.ndarray:
    c = np.where(np.abs(cmd) < deadband, 0.0, cmd)
    return np.where(c >= 0, c * fwd, c * rev)


def simulate_surge(p: dict, common: dict, t: np.ndarray, tl: np.ndarray, tr: np.ndarray,
                   u0: float = 0.0, delay_s: float = 0.0) -> np.ndarray:
    dt = float(np.median(np.diff(t)))
    tl, tr = shift_signal(t, tl, delay_s), shift_signal(t, tr, delay_s)
    Mu = float(common["mass_kg"]) + float(p.get("added_mass_surge_kg", 0.0))
    tau = max(float(p["thrust_time_constant_s"]), 1e-3)
    k = 1 - math.exp(-dt / tau)
    T_l = T_r = 0.0
    u = np.empty(len(t))
    ui = u0
    tgt_l = _thrust(tl, p["max_thrust_fwd_n"], p["max_thrust_rev_n"], p["thrust_deadband"])
    tgt_r = _thrust(tr, p["max_thrust_fwd_n"], p["max_thrust_rev_n"], p["thrust_deadband"])
    for i in range(len(t)):
        T_l += (tgt_l[i] - T_l) * k
        T_r += (tgt_r[i] - T_r) * k
        X = T_l + T_r - p["drag_surge_lin"] * ui - p["drag_surge_quad"] * ui * abs(ui)
        ui += X / Mu * dt
        u[i] = ui
    return u


def simulate_yaw(p: dict, common: dict, Iz: float, t: np.ndarray, tl: np.ndarray, tr: np.ndarray,
                 r0: float = 0.0, delay_s: float = 0.0, u: np.ndarray | None = None,
                 v: np.ndarray | None = None) -> np.ndarray:
    """
    1-DOF yaw with the measured surge/sway fed in for the Coriolis coupling
    N_c = (Mu - Mv) u v. At cruise that term is the same size as the thruster
    moment, so leaving it out biases Nr badly. If your real boat has no sway
    estimate, pass v=None and only fit on the zero-speed spin test.
    """
    dt = float(np.median(np.diff(t)))
    tl, tr = shift_signal(t, tl, delay_s), shift_signal(t, tr, delay_s)
    Mr = Iz + float(p.get("added_inertia_yaw_kgm2", 0.0))
    Mu = float(common["mass_kg"]) + float(p.get("added_mass_surge_kg", 0.0))
    Mv = float(common["mass_kg"]) + float(p.get("added_mass_sway_kg", 0.0))
    Nc = (Mu - Mv) * u * v if (u is not None and v is not None) else np.zeros(len(t))
    by = float(p["thruster_offset_y_m"])
    tau = max(float(p["thrust_time_constant_s"]), 1e-3)
    k = 1 - math.exp(-dt / tau)
    T_l = T_r = 0.0
    r = np.empty(len(t))
    ri = r0
    tgt_l = _thrust(tl, p["max_thrust_fwd_n"], p["max_thrust_rev_n"], p["thrust_deadband"])
    tgt_r = _thrust(tr, p["max_thrust_fwd_n"], p["max_thrust_rev_n"], p["thrust_deadband"])
    for i in range(len(t)):
        T_l += (tgt_l[i] - T_l) * k
        T_r += (tgt_r[i] - T_r) * k
        N = (T_r - T_l) * by - p["drag_yaw_lin"] * ri - p["drag_yaw_quad"] * ri * abs(ri) + Nc[i]
        ri += N / Mr * dt
        r[i] = ri
    return r


def fit_surge(log: Log, base: VehicleParams, delay_guess: float | None = None) -> tuple[dict, float, float]:
    """Returns (boat params, delay, rms). Delay fitted jointly with the thrust lag."""
    p0, c = dict(base.boat), base.common
    u_meas = smooth(log.vx, 5)
    d0 = delay_guess if delay_guess is not None else float(c.get("actuator_delay_s", 0.0))

    def unpack(x):
        q = dict(p0)
        q["max_thrust_fwd_n"] = math.exp(x[0])
        q["thrust_time_constant_s"] = math.exp(x[1])
        q["drag_surge_lin"] = math.exp(x[2])
        q["drag_surge_quad"] = math.exp(x[3])
        q["max_thrust_rev_n"] = q["max_thrust_fwd_n"] * float(p0["max_thrust_rev_n"]) / max(float(p0["max_thrust_fwd_n"]), 1e-6)
        return q, float(x[4])

    def resid(x):
        q, d = unpack(x)
        return simulate_surge(q, c, log.t, log.thrust_l, log.thrust_r, u_meas[0], d) - u_meas

    x0 = np.array([*np.log([p0["max_thrust_fwd_n"], p0["thrust_time_constant_s"], p0["drag_surge_lin"], p0["drag_surge_quad"]]), d0])
    lo = [math.log(0.5), math.log(0.02), math.log(0.01), math.log(0.01), 0.0]
    hi = [math.log(500.0), math.log(3.0), math.log(200.0), math.log(500.0), 0.5]
    best = None
    for d_init in sorted({d0, 0.02, 0.1, 0.2}):
        x0[4] = min(max(d_init, 0.0), 0.5)
        sol = least_squares(resid, x0, bounds=(lo, hi), loss="soft_l1", f_scale=0.1, max_nfev=200)
        if best is None or sol.cost < best.cost:
            best = sol
    q, d = unpack(best.x)
    rms = float(np.sqrt(np.mean(resid(best.x) ** 2)))
    return {k: round(q[k], 4) for k in ("max_thrust_fwd_n", "max_thrust_rev_n", "thrust_time_constant_s",
                                        "drag_surge_lin", "drag_surge_quad")}, round(d, 3), rms


def fit_yaw(log: Log, base: VehicleParams) -> tuple[dict, float, float]:
    p0, c = dict(base.boat), base.common
    delay = float(c.get("actuator_delay_s", 0.0))
    r_meas = smooth(log.wz, 5)
    Iz0 = base.yaw_inertia

    def unpack(x):
        q = dict(p0)
        q["drag_yaw_lin"] = math.exp(x[0])
        q["drag_yaw_quad"] = math.exp(x[1])
        return q, Iz0 * math.exp(x[2])

    u_meas, v_meas = smooth(log.vx, 5), smooth(log.vy, 5)

    def resid(x):
        q, Iz = unpack(x)
        return simulate_yaw(q, c, Iz, log.t, log.thrust_l, log.thrust_r, r_meas[0], delay, u_meas, v_meas) - r_meas

    x0 = np.log([p0["drag_yaw_lin"], p0["drag_yaw_quad"], 1.0])
    lo, hi = [math.log(0.01), math.log(0.01), math.log(0.2)], [math.log(100.0), math.log(100.0), math.log(5.0)]
    sol = least_squares(resid, x0, bounds=(lo, hi), loss="soft_l1", f_scale=0.1, max_nfev=300)
    q, Iz = unpack(sol.x)
    rms = float(np.sqrt(np.mean(resid(sol.x) ** 2)))
    return {"drag_yaw_lin": round(q["drag_yaw_lin"], 4), "drag_yaw_quad": round(q["drag_yaw_quad"], 4)}, round(Iz, 4), rms


def straight_segment(log: Log, r_thresh: float = 0.2) -> Log:
    """
    The surge model is 1-DOF; turning couples sway and yaw into surge through
    the Coriolis terms and the fit will happily bend thrust and drag to explain
    that. So the surge fit only sees the log up to the first real turn.
    """
    turning = np.flatnonzero(np.abs(smooth(log.wz, 5)) > r_thresh)
    if len(turning) == 0:
        return log
    t_end = float(log.t[turning[0]]) - 0.5
    if t_end < 15.0:
        raise ValueError("need >= 15 s of straight running (thrust steps + coast) before any turning for the surge fit")
    return log.window(float(log.t[0]), t_end)


def fit_boat(log: Log, base: VehicleParams, source: str = "") -> VehicleParams:
    straight = straight_segment(log)
    delay_guess = estimate_delay(straight.t, straight.thrust_l + straight.thrust_r, straight.vx)
    surge, delay, rms_u = fit_surge(straight, base, delay_guess)
    base = base.with_overrides(**{f"boat.{k}": v for k, v in surge.items()}, **{"common.actuator_delay_s": delay})
    yaw, Iz, rms_r = fit_yaw(log, base)
    out = base.with_overrides(**{f"boat.{k}": v for k, v in yaw.items()}, **{"common.yaw_inertia_kgm2": Iz})
    out.raw["calibration"] = {"source": source, "date": date.today().isoformat(),
                              "rms_error": {"surge_mps": round(rms_u, 4), "yaw_rate_rad_s": round(rms_r, 4),
                                            "actuator_delay_s": round(delay, 3)}}
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("base_params")
    ap.add_argument("-o", "--out", required=True)
    a = ap.parse_args()
    fitted = fit_boat(load_log(a.log), load_vehicle(a.base_params), source=a.log)
    fitted.save(a.out)
    print(json.dumps({"calibration": fitted.raw["calibration"], "boat": fitted.boat,
                      "yaw_inertia_kgm2": fitted.common.get("yaw_inertia_kgm2")}, indent=2))


if __name__ == "__main__":
    main()
