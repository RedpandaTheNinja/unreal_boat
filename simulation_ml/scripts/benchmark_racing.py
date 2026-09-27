"""Compare valid lap time; also exercise passing and a fully blocked lane.

Run from the project root with the kit's Python environment. Evidence is saved
beside the previous baseline; original scenarios are not modified.
"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'unreal_fable/python'))
from fable import Sim, load_scene
from fable.controllers import PurePursuit
from fable.controllers.racing import RacingController


def scenario(case):
    sc = load_scene('car_track_oval')
    if case in ('passing', 'blocked'):
        sc.raw['track']['width_m'] = 3.2
        sc.raw['name'] = 'car_racing_' + case
        sc.raw['objects'] = [{'label': 'test_obstacle', 'kind': 'barrel' if case == 'passing' else 'wall',
                              'xy_m': [-3, -7], 'scale': [1, 1, 1] if case == 'passing' else [0.3, 3.2, 1]}]
    return sc


def run(case, seed):
    sc = scenario(case)
    ctrl_type = PurePursuit if case == 'baseline' else RacingController
    out = ROOT / 'test-results/racing'
    out.mkdir(parents=True, exist_ok=True)
    with Sim(sc, strict_track=True, log_path=str(out / f'{case}_{seed}.csv')) as sim:
        ctrl = ctrl_type(sc, sim.vehicle)
        # A blocked course is judged on holding a stop, never as a completed lap.
        rep = sim.run(ctrl, seed=seed, max_steps=1000 if case == 'blocked' else None)
        rep.update(case=case, seed=seed, controller=ctrl_type.__name__,
                   final_speed_mps=round(sim.obs.vel.vx, 4), status=getattr(ctrl, 'status', ''))
        if case == 'blocked':
            rep['safe_stop'] = (not sim.done and sim.obs.vel.vx < 0.05 and
                                sim.obs.extra['footprint_obstacle_clearance_m'] > 0 and
                                rep['min_track_clearance_m'] >= 0)
    return rep


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--seeds', type=int, default=3)
    ap.add_argument('--cases', nargs='+', default=['baseline', 'racing', 'passing', 'blocked'])
    args = ap.parse_args()
    reports = []
    for case in args.cases:
        for seed in range(args.seeds):
            rep = run(case, seed)
            reports.append(rep)
            print(json.dumps(rep), flush=True)
    (ROOT / 'test-results/racing/benchmark.json').write_text(json.dumps(reports, indent=2))
