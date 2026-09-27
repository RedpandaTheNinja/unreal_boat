"""
fit_car.py - fit the Ackermann car model to a logged drive.

What it fits, from what:

  actuator_delay_s          cross-correlation of throttle -> speed  (any log)
  max_speed_mps,
  accel_time_constant_s,
  max_brake_mps2            simulation-based least squares on speed  (needs throttle steps: 0 -> 0.3 -> 0.6 -> 1.0 -> 0)
  max_steer_rad,
  understeer_gain,
  steer_bias                least squares on yaw rate vs v*tan(delta)/L  (needs steering sweeps at 2-3 speeds)

A 3-minute drive that includes a few full-throttle launches, brakes to a stop,
and slaloms at slow and fast speed contains all of it. See docs/CALIBRATION.md.

    python -m fable.calibration.fit_car log.csv specs/vehicles/car_rc_default.json -o car_measured.json
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


# --------------------------------------------------------------- models

def simulate_speed(p: dict, t: np.ndarray, throttle: np.ndarray, brake: np.ndarray,
                   v0: float = 0.0, delay_s: float = 0.0, mass: float = 3.5, mu: float = 0.9) -> np.ndarray:
    """Same longitudinal equations as MockCar, vectorised over a log."""
    dt = float(np.median(np.diff(t)))
    thr = shift_signal(t, throttle, delay_s)
    brk = shift_signal(t, brake, delay_s)
    v = np.empty(len(t))
    vi = v0
    for i in range(len(t)):
        if brk[i] > 0:
            a = -math.copysign(p["max_brake_mps2"] * brk[i], vi) if abs(vi) > 1e-3 else 0.0
        else:
            v_ref = thr[i] * (p["max_speed_mps"] if thr[i] >= 0 else p["max_reverse_mps"])
            a = (v_ref - vi) / max(p["accel_time_constant_s"], 1e-3)
            a = max(-p["max_brake_mps2"], min(p["max_accel_mps2"], a))
        a -= (p["drag_coeff"] * vi * abs(vi) + p["rolling_resistance"] * vi) / mass
        a = max(-mu * 9.81, min(mu * 9.81, a))
        v_new = vi + a * dt
        if brk[i] > 0 and v_new * vi < 0:
            v_new = 0.0
        vi = v_new
        v[i] = vi
    return v


def yaw_rate_model(max_steer: float, understeer: float, bias: float, steer: np.ndarray,
                   v: np.ndarray, L: float) -> np.ndarray:
    delta = np.clip(steer - bias, -1, 1) * max_steer
    delta_eff = delta / (1.0 + understeer * v * v)
    return v * np.tan(delta_eff) / L


# ----------------------------------------------------------------- fits

def fit_longitudinal(log: Log, base: VehicleParams, delay_guess: float | None = None) -> tuple[dict, float, float]:
    """Returns (car params, actuator delay, rms). Delay is fitted jointly - a
    first-order lag and a pure delay look alike over a single step, so they
    must be estimated together from several steps of different size."""
    p0 = dict(base.car)
    m = float(base.common["mass_kg"])
    v_meas = smooth(log.speed, 5)
    d0 = delay_guess if delay_guess is not None else float(base.common.get("actuator_delay_s", 0.0))

    def unpack(x):
        q = dict(p0)
        q["max_speed_mps"] = math.exp(x[0])
        q["accel_time_constant_s"] = math.exp(x[1])
        q["max_brake_mps2"] = math.exp(x[2])
        q["max_accel_mps2"] = math.exp(x[3])
        return q, float(x[4])

    def resid(x):
        q, d = unpack(x)
        return simulate_speed(q, log.t, log.throttle, log.brake, v_meas[0], d, m) - v_meas

    x0 = np.array([*np.log([p0["max_speed_mps"], p0["accel_time_constant_s"], p0["max_brake_mps2"], p0["max_accel_mps2"]]), d0])
    lo = [math.log(0.2), math.log(0.05), math.log(0.5), math.log(0.5), 0.0]
    hi = [math.log(40.0), math.log(5.0), math.log(30.0), math.log(30.0), 0.5]
    best = None
    for d_init in sorted({d0, 0.02, 0.1, 0.2}):                   # a few starts: the delay axis is bumpy
        x0[4] = min(max(d_init, 0.0), 0.5)
        sol = least_squares(resid, x0, bounds=(lo, hi), loss="soft_l1", f_scale=0.3, max_nfev=150)
        if best is None or sol.cost < best.cost:
            best = sol
    q, d = unpack(best.x)
    rms = float(np.sqrt(np.mean(resid(best.x) ** 2)))
    return {k: round(q[k], 4) for k in ("max_speed_mps", "accel_time_constant_s", "max_brake_mps2", "max_accel_mps2")}, round(d, 3), rms


def fit_steering(log: Log, base: VehicleParams, min_speed: float = 0.3) -> tuple[dict, float]:
    L = float(base.car["wheelbase_m"])
    v = smooth(log.speed, 5)
    r = smooth(log.wz, 5)
    steer = log.steer
    delay = float(base.common.get("actuator_delay_s", 0.0))
    steer = shift_signal(log.t, steer, delay)
    m = v > min_speed
    if m.sum() < 50:
        raise ValueError("not enough moving samples for a steering fit (need speed > 0.3 m/s)")

    def resid(x):
        return yaw_rate_model(x[0], max(x[1], 0.0), x[2], steer[m], v[m], L) - r[m]

    x0 = [float(base.car["max_steer_rad"]), float(base.car.get("understeer_gain", 0.0)), float(base.car.get("steer_bias", 0.0))]
    sol = least_squares(resid, x0, bounds=([0.05, 0.0, -0.3], [1.2, 2.0, 0.3]), loss="soft_l1", f_scale=0.2)
    rms = float(np.sqrt(np.mean(resid(sol.x) ** 2)))
    return {"max_steer_rad": round(float(sol.x[0]), 4), "understeer_gain": round(float(sol.x[1]), 4),
            "steer_bias": round(float(sol.x[2]), 4)}, rms


def fit_car(log: Log, base: VehicleParams, source: str = "") -> VehicleParams:
    delay_guess = estimate_delay(log.t, log.throttle, log.speed)
    lon, delay, rms_v = fit_longitudinal(log, base, delay_guess)
    base = base.with_overrides(**{"common.actuator_delay_s": delay})
    lat, rms_r = fit_steering(log, base)
    out = base.with_overrides(**{f"car.{k}": v for k, v in {**lon, **lat}.items()})
    out.raw["calibration"] = {"source": source, "date": date.today().isoformat(),
                              "rms_error": {"speed_mps": round(rms_v, 4), "yaw_rate_rad_s": round(rms_r, 4),
                                            "actuator_delay_s": round(delay, 3)}}
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("base_params")
    ap.add_argument("-o", "--out", required=True)
    a = ap.parse_args()
    fitted = fit_car(load_log(a.log), load_vehicle(a.base_params), source=a.log)
    fitted.save(a.out)
    print(json.dumps({"calibration": fitted.raw["calibration"], "car": fitted.car}, indent=2))


if __name__ == "__main__":
    main()
