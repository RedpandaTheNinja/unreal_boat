import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fable import Sim, load_scene
from fable.controllers import RacingController
from fable.geometry import Polyline
from fable.sim import TaskEvaluator
from fable.types import Observation, Pose


def test_crossing_keeps_the_reachable_branch():
    route = Polyline([[-2, -2], [2, 2], [-2, 2], [2, -2]], False)
    expected = math.sqrt(8)
    # This point is nearer the later, descending branch.
    xy = [0.02, -0.02]
    assert route.project(xy)[0] > expected + 4
    s, ct, _ = route.project_near(xy, expected - 0.05, 0.5)
    assert s == pytest.approx(expected)
    assert abs(ct) < 0.04


def test_lap_progress_unwraps_and_teleport_is_not_a_shortcut():
    route = Polyline([[0, 0], [10, 0], [10, 10], [0, 10]], True)
    s, _, _ = route.project_near([0.1, 0], 39.9, 0.5)
    assert s == pytest.approx(40.1)
    s, ct, _ = route.project_near([10, 10], 0, 0.5)
    assert abs(ct) > 10
    assert s <= 0.5


@pytest.mark.parametrize('failure', ['collision', 'boundary', 'footprint'])
def test_invalid_finish_cannot_be_a_success(failure):
    sc = load_scene('car_track_oval')
    ev = TaskEvaluator(sc, 0.02, strict_track=True)
    obs = Observation(pose=Pose(-12, -7, yaw=0))
    ev.reset(obs)
    ev.st.progress_s = ev.line.length * 2
    if failure == 'collision':
        obs.collision = True
    elif failure == 'boundary':
        obs.pose.y = -8.0  # center inside the old tolerance; footprint outside
    else:
        obs.extra['footprint_obstacle_clearance_m'] = -0.01
    ev.update(obs, {})
    assert ev.st.done and not ev.st.success


def test_missing_and_held_stale_scan_command_a_stop():
    sc = load_scene('car_track_oval')
    ctrl = RacingController(sc, sc.vehicle())
    assert ctrl(Observation())['brake'] == 1
    with Sim(sc) as sim:
        obs = sim.reset(seed=0)
        ctrl(obs)
        obs.t += 0.5
        assert ctrl(obs)['brake'] == 1
        assert ctrl.status == 'stale_lidar'


@pytest.mark.parametrize('blocked', [False, True])
def test_racing_passes_or_stops_without_leaving_track(blocked):
    sc = load_scene('car_track_oval')
    sc.raw['track']['width_m'] = 3.2
    sc.raw['objects'] = [{'label': 'test_obstacle', 'kind': 'wall' if blocked else 'barrel',
                          'xy_m': [-3, -7], 'scale': [0.3, 3.2, 1] if blocked else [1, 1, 1]}]
    with Sim(sc, strict_track=True) as sim:
        ctrl = RacingController(sc, sim.vehicle)
        rep = sim.run(ctrl, seed=1, max_steps=1000 if blocked else None)
        assert rep['collisions'] == 0
        assert rep['min_track_clearance_m'] >= 0
        if blocked:
            assert not sim.done
            assert sim.obs.vel.vx < 0.05
            assert sim.obs.extra['footprint_obstacle_clearance_m'] > 0
        else:
            assert rep['success'], rep


def test_racing_parameters_reject_invalid_speed():
    sc = load_scene('car_track_oval')
    for speed in [-1, 0, float('nan')]:
        with pytest.raises(ValueError):
            RacingController(sc, sc.vehicle(), v_max=speed)
