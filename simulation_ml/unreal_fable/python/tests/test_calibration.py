"""
The fitters are tested the only honest way: simulate a vehicle with KNOWN
parameters that differ from the defaults, log it, fit from the defaults, and
check the truth is recovered.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from fable import Sim, load_scene, load_vehicle  # noqa: E402
from fable.calibration import fit_boat, fit_car, load_log  # noqa: E402
from fable.calibration.maneuvers import DURATION, boat_sysid, car_sysid  # noqa: E402


def _open_water(scenario: str):
    """A big empty world so the open-loop manoeuvre never hits anything."""
    name = "car_track_oval" if scenario == "car" else "boat_buoy_course"
    sc = load_scene(name)
    sc.raw["objects"], sc.raw["obstacle_fields"], sc.raw["gates"] = [], [], []
    sc.raw["track"] = {"centerline_m": [], "border": "none"}
    sc.raw["world"]["size_m"] = [2000, 2000]
    sc.raw["world"]["disturbance"] = {}
    if "water" in sc.raw["world"]:
        sc.raw["world"]["water"]["waves"] = {"amp_m": 0}
        sc.raw["world"]["water"]["shore"] = []
    sc.raw["randomization"] = {"enabled": False}
    sc.raw["spawn"] = {"xy_m": [0, 0], "yaw_rad": 0, "jitter_m": 0, "jitter_yaw_rad": 0}
    sc.raw["task"] = {"type": "station_keep", "max_time_s": DURATION[scenario] + 5, "fail_on": [], "metrics": []}
    return sc


def _run_log(sc, vp, seq, duration, path):
    with Sim(sc, backend="mock", vehicle=vp, log_path=path) as sim:
        obs = sim.reset(seed=0)
        while obs.t < duration:
            obs = sim.step(seq(obs.t))
    return load_log(path)


def test_car_fit_recovers_truth(tmp_path):
    truth = load_vehicle("vehicles/car_rc_default.json").with_overrides(**{
        "car.max_speed_mps": 4.2, "car.accel_time_constant_s": 0.7, "car.max_steer_rad": 0.33,
        "car.understeer_gain": 0.05, "car.max_brake_mps2": 4.5, "common.actuator_delay_s": 0.08,
        "sensors.odom.speed_noise_std": 0.01, "sensors.imu.gyro_noise_std": 0.003})
    sc = _open_water("car")
    log = _run_log(sc, truth, car_sysid, DURATION["car"], str(tmp_path / "car.csv"))

    fitted = fit_car(log, load_vehicle("vehicles/car_rc_default.json"), source="test")
    c = fitted.car
    assert abs(fitted.common["actuator_delay_s"] - 0.08) <= 0.03
    assert abs(c["max_speed_mps"] - 4.2) / 4.2 < 0.08
    assert abs(c["accel_time_constant_s"] - 0.7) / 0.7 < 0.25
    assert abs(c["max_steer_rad"] - 0.33) / 0.33 < 0.12
    assert abs(c["max_brake_mps2"] - 4.5) / 4.5 < 0.3
    assert fitted.raw["calibration"]["rms_error"]["speed_mps"] < 0.15


def test_boat_fit_recovers_truth(tmp_path):
    truth = load_vehicle("vehicles/boat_twin_thruster_default.json").with_overrides(**{
        "boat.max_thrust_fwd_n": 26.0, "boat.thrust_time_constant_s": 0.4, "boat.drag_surge_lin": 3.0,
        "boat.drag_surge_quad": 11.0, "boat.drag_yaw_lin": 2.5, "boat.drag_yaw_quad": 2.0,
        "common.yaw_inertia_kgm2": 3.0, "common.actuator_delay_s": 0.1,
        "sensors.odom.speed_noise_std": 0.01, "sensors.imu.gyro_noise_std": 0.003})
    sc = _open_water("boat")
    log = _run_log(sc, truth, boat_sysid, DURATION["boat"], str(tmp_path / "boat.csv"))

    fitted = fit_boat(log, load_vehicle("vehicles/boat_twin_thruster_default.json"), source="test")
    b = fitted.boat
    # a pure delay and a first-order lag are barely separable from surge data alone,
    # so check their SUM; docs/CALIBRATION.md says to measure thrust lag on the bench
    assert abs((fitted.common["actuator_delay_s"] + b["thrust_time_constant_s"]) - 0.5) <= 0.08
    assert abs(b["max_thrust_fwd_n"] - 26.0) / 26.0 < 0.15
    assert abs(b["drag_surge_quad"] - 11.0) / 11.0 < 0.25
    assert abs(b["drag_yaw_lin"] - 2.5) / 2.5 < 0.35
    assert abs(fitted.common["yaw_inertia_kgm2"] - 3.0) / 3.0 < 0.35
    assert fitted.raw["calibration"]["rms_error"]["surge_mps"] < 0.08
    assert fitted.raw["calibration"]["rms_error"]["yaw_rate_rad_s"] < 0.1
