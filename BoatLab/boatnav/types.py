"""Shared data types passed between HAL, perception, planning and control."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field


@dataclass
class BoatState:
    t: float                  # seconds (backend clock)
    x: float                  # local ENU east, m
    y: float                  # local ENU north, m
    heading: float            # rad, CCW from east
    u: float = 0.0            # surge, m/s (body forward)
    v: float = 0.0            # sway, m/s (body left)
    r: float = 0.0            # yaw rate, rad/s (CCW +)
    lat: float | None = None
    lon: float | None = None
    left_thrust_n: float = 0.0
    right_thrust_n: float = 0.0
    valid: bool = True
    source: str = ""          # "twin_truth", "unreal_truth+noise", "gps_rtk", ...
    extra: dict = field(default_factory=dict)

    @property
    def speed(self) -> float:
        return (self.u ** 2 + self.v ** 2) ** 0.5

    def as_dict(self) -> dict:
        d = asdict(self)
        d["speed"] = self.speed
        return d


@dataclass
class CameraFrame:
    t: float
    rgb: object               # HxWx3 uint8 numpy array, RGB order
    depth: object | None      # HxW float32 metres along optical axis, or None
    fx: float
    fy: float
    cx: float
    cy: float
    mount_flu: tuple          # camera position in body FLU (m)
    pitch_down_rad: float
    pose: tuple               # (x, y, heading) of the boat when captured
    sequence: int = 0
    source: str = ""


@dataclass
class Detection:
    label: str                # color class: red, green, blue, orange, purple, yellow, black, zebra, pink, case, target
    u: float                  # pixel centre column
    v: float                  # pixel bottom row (waterline)
    bbox: tuple               # x0, y0, x1, y1
    bearing: float            # rad, body frame, + = left
    range_m: float            # metres
    range_source: str         # "depth" | "size"
    x: float = 0.0            # world estimate
    y: float = 0.0
    score: float = 1.0
    area_px: int = 0

    def as_dict(self) -> dict:
        return asdict(self)
