"""GPS <-> local ENU conversion (equirectangular about an origin; accurate to cm over a lake)."""
from __future__ import annotations

import math

R_EARTH = 6378137.0


class LocalFrame:
    """Local tangent plane at (lat0, lon0): x east, y north, metres."""

    def __init__(self, lat0: float, lon0: float):
        self.lat0, self.lon0 = float(lat0), float(lon0)
        self._cos = math.cos(math.radians(self.lat0))

    @classmethod
    def from_course(cls, course: dict) -> "LocalFrame":
        g = course["geo_origin"]
        return cls(g["lat"], g["lon"])

    def to_local(self, lat: float, lon: float) -> tuple[float, float]:
        x = math.radians(lon - self.lon0) * R_EARTH * self._cos
        y = math.radians(lat - self.lat0) * R_EARTH
        return x, y

    def to_geo(self, x: float, y: float) -> tuple[float, float]:
        lat = self.lat0 + math.degrees(y / R_EARTH)
        lon = self.lon0 + math.degrees(x / (R_EARTH * self._cos))
        return lat, lon


def heading_from_compass(deg_cw_from_north: float) -> float:
    """Compass/GPS course (deg, clockwise from north) -> ENU heading (rad, CCW from east)."""
    return math.atan2(math.cos(math.radians(deg_cw_from_north)), math.sin(math.radians(deg_cw_from_north)))


def compass_from_heading(heading_rad: float) -> float:
    return (90.0 - math.degrees(heading_rad)) % 360.0
