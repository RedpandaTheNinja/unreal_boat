"""Hardware abstraction layer. Pick a backend with make_backend(name, ...)."""
from __future__ import annotations


def make_backend(name: str, boat_cfg: dict, course: dict, **kw):
    name = name.lower()
    if name in ("twin", "mock", "python"):
        from .twin import TwinBoat
        return TwinBoat(boat_cfg, course, **kw)
    if name in ("unreal", "ue", "ue5"):
        from .unreal import UnrealBoat
        return UnrealBoat(boat_cfg, course, **kw)
    if name in ("real", "hardware", "boat"):
        from .real import RealBoat
        return RealBoat(boat_cfg, course, **kw)
    raise ValueError(f"unknown backend {name!r}: use twin | unreal | real")
