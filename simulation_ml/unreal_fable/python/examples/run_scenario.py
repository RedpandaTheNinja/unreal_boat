"""
run_scenario.py - run any scene with a baseline controller, on the mock or on Unreal.

    python examples/run_scenario.py car_track_oval                      # mock, fast
    python examples/run_scenario.py boat_buoy_course --backend unreal    # live Unreal via UDP
    python examples/run_scenario.py boat_docking --seeds 10 --log runs/  # batch, CSV logs

Swap `make_controller` for your own: anything callable(obs) -> dict works.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from fable import Sim, load_scene, load_vehicle  # noqa: E402
from fable.controllers import DockingController, LOSGuidance, PurePursuit, ReactiveAvoid  # noqa: E402
from fable.controllers.racing import RacingController  # noqa: E402


def make_controller(scene, vehicle):
    task = scene.task.get("type")
    if scene.scenario == "car":
        base = PurePursuit(scene, vehicle, v_max=2.0 if scene.obstacle_fields else None)
        return ReactiveAvoid(base, "car") if (scene.objects or scene.obstacle_fields) else base
    if task == "dock":
        return DockingController(scene, vehicle)
    return LOSGuidance(scene, vehicle)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("scene")
    ap.add_argument("--backend", default="mock", choices=["mock", "unreal"])
    ap.add_argument("--vehicle", help="override vehicle_params json")
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--log", help="directory for per-episode CSV logs")
    ap.add_argument("--realtime", action="store_true", help="mock only: run at wall-clock speed")
    ap.add_argument("--port", type=int, default=9800)
    ap.add_argument('--controller', choices=['baseline', 'racing'], default='baseline')
    ap.add_argument('--max-speed', type=float, help='racing speed ceiling in m/s')
    ap.add_argument('--strict-track', action='store_true', help='invalidate collisions and footprint boundary exits')
    a = ap.parse_args()

    scene = load_scene(a.scene, validate=True)
    vehicle = load_vehicle(a.vehicle) if a.vehicle else scene.vehicle()
    if a.log:
        os.makedirs(a.log, exist_ok=True)

    results = []
    for seed in range(a.seeds):
        log_path = os.path.join(a.log, f"{scene.name}_{seed}.csv") if a.log else None
        kw = {"port": a.port} if a.backend == "unreal" else {}
        with Sim(scene, backend=a.backend, vehicle=vehicle, log_path=log_path, realtime=a.realtime,
                 strict_track=a.strict_track or a.controller == 'racing', **kw) as sim:
            controller = RacingController(scene, vehicle, v_max=a.max_speed) if a.controller == 'racing' else make_controller(scene, vehicle)
            rep = sim.run(controller, seed=seed)
        rep["seed"] = seed
        results.append(rep)
        print(json.dumps(rep))

    if len(results) > 1:
        ok = sum(r["success"] for r in results)
        print(f"\n{ok}/{len(results)} succeeded")


if __name__ == "__main__":
    main()
