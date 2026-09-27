"""Small reproducible diagnostic; preserves the supplied scene files."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'unreal_fable/python'))
from fable import Sim, load_scene
from fable.controllers import PurePursuit
from fable.controllers.racing import RacingController

for mode in ('pursuit', 'racing', 'racing_no_borders'):
    sc = load_scene('car_figure8_digital_twin')
    if mode.endswith('no_borders'):
        sc.raw['track']['border'] = 'none'
        sc.raw['objects'] = []
    with Sim(sc, strict_track=True) as sim:
        ctrl = PurePursuit(sc, sim.vehicle, v_max=1.5) if mode == 'pursuit' else RacingController(sc, sim.vehicle)
        rep = sim.run(ctrl, seed=0, max_steps=2500)
        print(mode, rep, 'status', getattr(ctrl, 'status', ''), 'xy', sim.obs.pose.xy,
              'offset', getattr(ctrl, 'offset', ''), 'v', sim.obs.vel.vx, flush=True)
