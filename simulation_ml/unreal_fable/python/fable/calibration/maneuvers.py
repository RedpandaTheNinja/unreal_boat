"""
System-identification manoeuvres: open-loop command sequences that excite the
parameters the fitters need. Run the SAME sequence on the real vehicle (via the
ROS 2 adapter) and in the sim; feed both logs to the fitter.

Each function maps time -> command dict. Keep the vehicle in open space.
"""

from __future__ import annotations

import math


def car_sysid(t: float) -> dict[str, float]:
    """
    0-8 s   throttle steps 0.3 / 0.6 / 1.0 straight       -> max speed, accel tau
    8-11 s  brake                                          -> max brake
    11-25 s slow slalom (0.35 throttle, +-0.8 steer, 2 s)  -> max steer
    25-40 s fast slalom (0.8 throttle, +-0.5 steer, 3 s)   -> understeer
    40-44 s coast                                          -> drag / rolling
    """
    if t < 2.0:
        return {"throttle": 0.3, "steer": 0.0, "brake": 0.0}
    if t < 5.0:
        return {"throttle": 0.6, "steer": 0.0, "brake": 0.0}
    if t < 8.0:
        return {"throttle": 1.0, "steer": 0.0, "brake": 0.0}
    if t < 11.0:
        return {"throttle": 0.0, "steer": 0.0, "brake": 1.0}
    if t < 25.0:
        return {"throttle": 0.35, "steer": 0.8 * math.copysign(1, math.sin(2 * math.pi * (t - 11) / 4.0)), "brake": 0.0}
    if t < 40.0:
        return {"throttle": 0.8, "steer": 0.5 * math.sin(2 * math.pi * (t - 25) / 6.0), "brake": 0.0}
    return {"throttle": 0.0, "steer": 0.0, "brake": 0.0}


def boat_sysid(t: float) -> dict[str, float]:
    """
    0-25 s   both thrusters 1.0                        -> top speed, thrust, drag
    25-60 s  coast (0, 0)                               -> drag lin/quad, cleanly
    60-100 s zig-zag at 0.6 cruise, +-0.5 yaw, 6 s     -> yaw drag, yaw inertia
    100-115 s spin: (+0.8, -0.8)                        -> yaw quad drag
    """
    if t < 25.0:
        return {"thrust_l": 1.0, "thrust_r": 1.0}
    if t < 60.0:
        return {"thrust_l": 0.0, "thrust_r": 0.0}
    if t < 100.0:
        y = 0.5 * math.copysign(1, math.sin(2 * math.pi * (t - 60) / 12.0))
        return {"thrust_l": 0.6 - y, "thrust_r": 0.6 + y}
    if t < 115.0:
        return {"thrust_l": 0.8, "thrust_r": -0.8}
    return {"thrust_l": 0.0, "thrust_r": 0.0}


DURATION = {"car": 44.0, "boat": 118.0}
