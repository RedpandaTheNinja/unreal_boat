"""
Calibration: fit vehicle_params.json to logged data from the real car / boat.

    python -m fable.calibration.fit_car  real_car_log.csv  specs/vehicles/car_rc_default.json  -o car_rc_measured.json
    python -m fable.calibration.fit_boat real_boat_log.csv specs/vehicles/boat_twin_thruster_default.json -o boat_measured.json

The log format is the one `fable.sim.EpisodeLog` writes (docs/CALIBRATION.md),
so a sim log and a real log are interchangeable - which is how the fitters are
tested: simulate with known parameters, fit from wrong defaults, recover them.
"""

from .logs import load_log, Log
from .fit_car import fit_car
from .fit_boat import fit_boat

__all__ = ["load_log", "Log", "fit_car", "fit_boat"]
