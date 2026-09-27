"""Backend interface. Everything above this line (perception, planning, control, mission)
is backend-agnostic, which is what makes the Python code portable to the real boat."""
from __future__ import annotations

import abc

from ..types import BoatState, CameraFrame


class BoatInterface(abc.ABC):
    name = "base"
    is_sim = True
    realtime = True        # False: the runner may step as fast as the backend allows (twin only)

    @abc.abstractmethod
    def state(self) -> BoatState:
        """Latest navigation state (ENU). For sims this is truth plus configured GPS noise."""

    @abc.abstractmethod
    def command(self, left: float, right: float) -> None:
        """Motor commands in [-1, 1]. Must be refreshed faster than the 0.5 s motor timeout."""

    def tick(self, dt: float) -> None:
        """Advance the backend by dt. Twin integrates physics; live backends do nothing
        (the runner paces wall-clock time)."""

    def camera(self) -> CameraFrame | None:
        return None

    def actuate(self, name: str, **kw) -> dict:
        """Trigger a payload mechanism: deploy | launch | recover."""
        return {"ok": False, "reason": "not supported"}

    def reset(self, pose=None) -> None:
        pass

    def set_environment(self, current=(0.0, 0.0), wind=(0.0, 0.0), waves=None, fog_visibility_m=None) -> dict:
        return {"ok": False, "reason": "not supported"}

    def truth(self) -> dict | None:
        """Ground truth for scoring (sims only). Never feed this into autonomy."""
        return None

    def stop(self) -> None:
        try:
            self.command(0.0, 0.0)
        except Exception:
            pass

    def close(self) -> None:
        self.stop()
