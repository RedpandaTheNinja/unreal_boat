"""
The UDP client is tested against the fake Unreal server - same protocol the C++
plugin implements - so a client bug shows up here, not in the editor.
"""
import os
import sys
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from fable import Sim, load_scene  # noqa: E402
from fable.controllers import PurePursuit  # noqa: E402
from fable.fake_unreal import FakeUnrealServer  # noqa: E402
from fable.types import Observation  # noqa: E402


def test_observation_wire_roundtrip():
    sc = load_scene("boat_buoy_course")
    with Sim(sc, backend="mock") as sim:
        obs = sim.reset(seed=0)
        obs = sim.step({"thrust_l": 0.5, "thrust_r": 0.4})
    back = Observation.from_wire(obs.to_wire())
    assert back.tick == obs.tick and abs(back.pose.x - obs.pose.x) < 1e-9
    assert back.lidar is not None and len(back.lidar.ranges) == len(obs.lidar.ranges)
    assert back.gps is not None and abs(back.gps["lat"] - obs.gps["lat"]) < 1e-12
    assert back.extra["thrust"]["l"] == obs.extra["thrust"]["l"]


def test_unreal_backend_against_fake_server():
    sc = load_scene("car_track_oval")
    srv = FakeUnrealServer(sc, port=9877, realtime=False)
    th = threading.Thread(target=srv.serve, kwargs={"stop_after_s": 30}, daemon=True)
    th.start()
    try:
        with Sim(sc, backend="unreal", port=9877, timeout=5.0) as sim:
            ctrl = PurePursuit(sc, sim.vehicle)
            obs = sim.reset(seed=0)
            t0 = obs.tick
            for _ in range(300):
                obs = sim.step(ctrl(obs))
            assert obs.tick > t0 + 250                    # frames advanced
            assert obs.speed > 0.5                        # the car actually drove
            assert obs.seq >= 0                           # our actions were applied
            r = sim.spawn("cone", "c1", obs.pose.x + 3, obs.pose.y)
            assert r["ok"]
            info = sim.scene_info()
            assert "c1" in info["dynamic"]
            assert sim.despawn("c1")["removed"] >= 1
            assert sim.set_env(wind={"speed": 2.0, "dir": 0.3})["ok"]
            assert sim.set_params(sim.vehicle.with_overrides(**{"car.max_speed_mps": 3.0}))["ok"]
    finally:
        srv.stop()
