"""
sysid_run.py - drive the system-identification manoeuvre and log it.

On the sim (mock or Unreal):
    python examples/sysid_run.py car  --backend unreal --out logs/car_sim.csv
On the real vehicle: run the same manoeuvre through your ROS 2 stack and log
the EpisodeLog columns (docs/CALIBRATION.md). Then:
    python -m fable.calibration.fit_car logs/car_real.csv specs/vehicles/car_rc_default.json -o specs/vehicles/car_rc_measured.json
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from fable import Sim, load_scene, load_vehicle  # noqa: E402
from fable.calibration.maneuvers import DURATION, boat_sysid, car_sysid  # noqa: E402


def open_world(scenario: str):
    sc = load_scene("car_track_oval" if scenario == "car" else "boat_buoy_course")
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


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", choices=["car", "boat"])
    ap.add_argument("--backend", default="mock", choices=["mock", "unreal"])
    ap.add_argument("--vehicle")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    sc = open_world(a.scenario)
    vp = load_vehicle(a.vehicle) if a.vehicle else sc.vehicle()
    seq = car_sysid if a.scenario == "car" else boat_sysid
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with Sim(sc, backend=a.backend, vehicle=vp, log_path=a.out) as sim:
        obs = sim.reset(seed=0)
        while obs.t < DURATION[a.scenario]:
            obs = sim.step(seq(obs.t))
    print(f"logged {DURATION[a.scenario]} s -> {a.out}")


if __name__ == "__main__":
    main()
