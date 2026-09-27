from __future__ import annotations

from ..types import clamp


class PID:
    def __init__(self, kp: float, ki: float = 0.0, kd: float = 0.0,
                 out_min: float = -1.0, out_max: float = 1.0, i_limit: float = 1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max, self.i_limit = out_min, out_max, i_limit
        self.reset()

    def reset(self) -> None:
        self.i = 0.0
        self.prev_e: float | None = None

    def __call__(self, e: float, dt: float, e_dot: float | None = None) -> float:
        if e_dot is None:
            e_dot = 0.0 if self.prev_e is None else (e - self.prev_e) / max(dt, 1e-6)
        self.prev_e = e
        self.i = clamp(self.i + e * dt, -self.i_limit, self.i_limit)
        return clamp(self.kp * e + self.ki * self.i + self.kd * e_dot, self.out_min, self.out_max)
