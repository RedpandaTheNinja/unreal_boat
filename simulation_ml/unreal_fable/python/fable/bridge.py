"""
bridge.py - UDP client for the Unreal FableBridge plugin (docs/PROTOCOL.md).

Same verbs as `MockBackend`: reset / step / set_params / set_env / spawn /
despawn / scene. `Sim` wraps either, so user code never imports this directly
unless it wants the raw socket.
"""

from __future__ import annotations

import json
import socket
import time
from typing import Any

from .specs import SceneSpec, VehicleParams
from .types import Observation

PROTOCOL_VERSION = 1


class BridgeError(RuntimeError):
    pass


class UnrealBackend:
    def __init__(self, spec: SceneSpec, vp: VehicleParams | None = None,
                 host: str = "127.0.0.1", port: int = 9800, timeout: float = 2.0,
                 dt: float = 0.02, role: str = "controller"):
        """role: 'controller' receives observations and drives; 'editor' only edits
        (spawn/despawn/set_env/scene) and never steals the observation stream."""
        self.spec = spec
        self.vp = vp or spec.vehicle()
        self.addr = (host, port)
        self.timeout = timeout
        self.dt = dt
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))                 # any free port; UE replies to sender
        self.sock.settimeout(timeout)
        self.seq = 0
        self.role = role
        self.last: Observation | None = None
        try:
            self._hello()
            if role == "controller":
                self.set_params(self.vp)
        except Exception:
            self.sock.close()
            raise

    # --- transport
    def _send(self, msg: dict[str, Any]) -> None:
        self.sock.sendto(json.dumps(msg, separators=(",", ":")).encode("utf-8"), self.addr)

    def _recv(self, want: str | None = None, deadline: float | None = None) -> dict[str, Any]:
        deadline = deadline or (time.monotonic() + self.timeout)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise BridgeError(f"timeout waiting for '{want or 'any'}' from Unreal at {self.addr}. "
                                  "Is the level playing (PIE or -game) with a FableBridge actor?")
            self.sock.settimeout(remaining)
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            m = json.loads(data.decode("utf-8"))
            if m.get("type") == "obs":
                self.last = Observation.from_wire(m)
            if want is None or m.get("type") == want:
                return m

    def _ack(self, for_what: str) -> dict:
        while True:
            m = self._recv("ack")
            if m.get("for") == for_what:
                if not m.get("ok", False):
                    raise BridgeError(f"{for_what} failed: {m.get('error')}")
                return m

    def _hello(self) -> None:
        self._send({"type": "hello", "client": "fable-py", "version": PROTOCOL_VERSION, "role": self.role})
        try:
            self._ack("hello")
        except BridgeError as e:
            if "timeout" in str(e):
                raise
            raise BridgeError("protocol version mismatch with the FableBridge plugin") from e

    # --- verbs
    def reset(self, pose: dict | None = None, seed: int | None = None, clear_dynamic: bool = True) -> Observation:
        msg: dict[str, Any] = {"type": "reset", "clear_dynamic": clear_dynamic}
        if pose:
            msg["pose"] = pose
        if seed is not None:
            msg["seed"] = int(seed)
        self._send(msg)
        ack = self._ack("reset")
        tick = int(ack.get("tick", -1))
        # wait for the first observation after the reset tick
        while True:
            self._recv("obs")
            if self.last is not None and self.last.tick > tick:
                return self.last

    def step(self, u: dict[str, float], seq: int | None = None) -> Observation:
        self.seq = seq if seq is not None else self.seq + 1
        self._send({"type": "act", "seq": self.seq, "u": {k: float(v) for k, v in u.items()}})
        last_tick = self.last.tick if self.last else -1
        while True:
            self._recv("obs")
            if self.last is not None and self.last.tick > last_tick:
                return self.last

    def set_params(self, params: dict | VehicleParams) -> dict:
        raw = params.to_wire() if isinstance(params, VehicleParams) else params
        self._send({"type": "set_params", "params": raw})
        return self._ack("set_params")

    def set_env(self, **kw) -> dict:
        self._send({"type": "set_env", **kw})
        return self._ack("set_env")

    def spawn(self, kind: str, label: str, x: float, y: float, z: float = 0.0, yaw: float = 0.0,
              scale=None, movable: bool = False) -> dict:
        self._send({"type": "spawn", "kind": kind, "label": label, "x": x, "y": y, "z": z, "yaw": yaw,
                    "scale": scale or [1, 1, 1], "movable": movable})
        return self._ack("spawn")

    def despawn(self, label: str) -> dict:
        self._send({"type": "despawn", "label": label})
        return self._ack("despawn")

    def scene(self) -> dict:
        self._send({"type": "query", "what": "scene"})
        return self._recv("scene")

    def close(self) -> None:
        try:
            self._send({"type": "bye"})
        finally:
            self.sock.close()
