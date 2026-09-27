import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from fable import Sim, load_scene  # noqa: E402
from fable.controllers import DockingController, LOSGuidance, PurePursuit, ReactiveAvoid  # noqa: E402

CASES = [
    ("car_track_oval", lambda sc, vp: PurePursuit(sc, vp)),
    ("car_track_slalom_gravel", lambda sc, vp: ReactiveAvoid(PurePursuit(sc, vp, v_max=2.0), "car")),
    ("boat_buoy_course", lambda sc, vp: LOSGuidance(sc, vp)),
    ("boat_docking", lambda sc, vp: DockingController(sc, vp)),
]


@pytest.mark.parametrize("name,make", CASES, ids=[c[0] for c in CASES])
@pytest.mark.parametrize("seed", [0, 1, 2])
def test_baseline_controller_passes(name, make, seed):
    sc = load_scene(name, validate=True)
    with Sim(sc, backend="mock") as sim:
        rep = sim.run(make(sc, sim.vehicle), seed=seed)
    assert rep["success"], rep


def test_live_world_edit_changes_outcome():
    """Spawn a wall across the oval mid-episode: the car must now hit it."""
    sc = load_scene("car_track_oval")
    with Sim(sc, backend="mock") as sim:
        ctrl = PurePursuit(sc, sim.vehicle)
        obs = sim.reset(seed=0)
        for _ in range(100):
            obs = sim.step(ctrl(obs))
        x, y, yaw = obs.pose.x, obs.pose.y, obs.pose.yaw
        import math
        sim.spawn("wall", "surprise", x + 4 * math.cos(yaw), y + 4 * math.sin(yaw), yaw=yaw + math.pi / 2, scale=[3, 0.3, 1])
        while not sim.done:
            obs = sim.step(ctrl(obs))
    rep = sim.report()
    assert not rep["success"] and rep["reason"].startswith("collision:surprise"), rep


def test_randomization_changes_layout():
    sc = load_scene("car_track_slalom_gravel")
    with Sim(sc, backend="mock") as sim:
        sim.reset(seed=1)
        a = sim.scene_info()["actors"]
        sim.reset(seed=2)
        b = sim.scene_info()["actors"]
    assert a != b


def test_vehicle_params_change_behaviour():
    sc = load_scene("car_track_oval")
    sc.raw["task"]["max_time_s"] = 400
    slow = sc.vehicle().with_overrides(**{"car.max_speed_mps": 1.5})
    with Sim(sc, backend="mock", vehicle=slow) as sim:
        rep = sim.run(PurePursuit(sc, slow), seed=0)
    assert rep["success"] and rep["time_s"] > 150, rep
