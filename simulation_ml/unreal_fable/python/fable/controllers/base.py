from __future__ import annotations

from ..types import Observation


class Controller:
    """
    Anything callable as controller(obs) -> dict works with Sim.run(). Subclass
    this to get reset() and a place to keep state. Return values are normalized
    commands: car {"throttle","steer","brake"}, boat {"thrust_l","thrust_r"}.
    """

    def reset(self) -> None:
        pass

    def __call__(self, obs: Observation) -> dict[str, float]:
        raise NotImplementedError
