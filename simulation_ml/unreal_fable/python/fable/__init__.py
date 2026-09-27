"""
fable - Python dev kit for testing autonomy algorithms against Unreal worlds.

    from fable import Sim, load_scene, load_vehicle
    from fable.controllers import PurePursuit, LOSGuidance

    scene = load_scene("specs/worlds/car_track_oval.json")
    with Sim(scene, backend="mock") as sim:          # or backend="unreal"
        obs = sim.reset(seed=0)
        ctrl = PurePursuit(scene, sim.vehicle)
        while not sim.done:
            obs = sim.step(ctrl(obs))
    print(sim.report())

Two backends, one interface: `mock` is a pure-Python physics stand-in so you can
iterate on the algorithm with no engine running; `unreal` is the real thing over
the UDP bridge. Swap the string and nothing else changes.
"""

from .types import Observation, Pose, Twist, Lidar
from .specs import load_scene, load_vehicle, SceneSpec, VehicleParams
from .sim import Sim

__all__ = ["Observation", "Pose", "Twist", "Lidar", "load_scene", "load_vehicle",
           "SceneSpec", "VehicleParams", "Sim"]
__version__ = "0.1.0"
