from __future__ import annotations

import csv
from dataclasses import dataclass

import numpy as np


@dataclass
class Log:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    vx: np.ndarray
    vy: np.ndarray
    wz: np.ndarray
    ax: np.ndarray
    ay: np.ndarray
    speed: np.ndarray
    throttle: np.ndarray
    steer: np.ndarray
    brake: np.ndarray
    thrust_l: np.ndarray
    thrust_r: np.ndarray

    @property
    def dt(self) -> float:
        return float(np.median(np.diff(self.t)))

    def window(self, t0: float, t1: float) -> "Log":
        m = (self.t >= t0) & (self.t <= t1)
        return Log(**{k: getattr(self, k)[m] for k in self.__dataclass_fields__})


COLUMNS = {"t": "t", "x": "x", "y": "y", "yaw": "yaw", "vx": "vx", "vy": "vy", "wz": "wz", "ax": "ax", "ay": "ay",
           "speed": "speed", "throttle": "u_throttle", "steer": "u_steer", "brake": "u_brake",
           "thrust_l": "u_thrust_l", "thrust_r": "u_thrust_r"}


def load_log(path: str) -> Log:
    """Reads the EpisodeLog CSV. Missing columns are zeros, so a real-car log
    without thrust columns (or a boat log without steer) loads fine."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"empty log: {path}")
    out = {}
    for field, col in COLUMNS.items():
        out[field] = np.array([float(r.get(col, 0) or 0) for r in rows], dtype=float)
    return Log(**out)


def smooth(x: np.ndarray, n: int = 5) -> np.ndarray:
    if n <= 1 or len(x) < n:
        return x
    k = np.ones(n) / n
    pad = np.pad(x, (n // 2, n - 1 - n // 2), mode="edge")
    return np.convolve(pad, k, mode="valid")


def derivative(x: np.ndarray, t: np.ndarray, n_smooth: int = 5) -> np.ndarray:
    return np.gradient(smooth(x, n_smooth), t)


def estimate_delay(t: np.ndarray, cmd: np.ndarray, resp: np.ndarray, max_delay: float = 0.5) -> float:
    """
    Pure delay between a command and its response by cross-correlating their
    derivatives (which removes the DC offset and any slow drift). Returns seconds.
    """
    dt = float(np.median(np.diff(t)))
    a = np.gradient(smooth(cmd, 3), t)
    b = np.gradient(smooth(resp, 3), t)
    a = (a - a.mean()) / (a.std() + 1e-9)
    b = (b - b.mean()) / (b.std() + 1e-9)
    n_max = int(max_delay / dt)
    best_k, best_c = 0, -np.inf
    for k in range(0, n_max + 1):
        c = float(np.dot(a[: len(a) - k], b[k:])) if k > 0 else float(np.dot(a, b))
        if c > best_c:
            best_c, best_k = c, k
    return best_k * dt


def shift_signal(t: np.ndarray, x: np.ndarray, delay_s: float) -> np.ndarray:
    """x delayed by a (fractional) number of samples, held at x[0] before t0."""
    if delay_s <= 0:
        return x
    return np.interp(t - delay_s, t, x, left=x[0])
