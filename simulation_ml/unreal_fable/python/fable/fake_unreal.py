"""
fake_unreal.py - a UDP server that speaks the FableBridge protocol, backed by
the mock physics. Two uses:

  1. Develop and test the bridge client and your controller's networking with
     no Unreal running:   python -m fable.fake_unreal specs/worlds/car_track_oval.json
  2. A reference implementation of the protocol for whoever ports the C++ side
     to another engine (or debugs it): every message the plugin must handle is
     handled here, in ~120 lines.

It runs the mock in real time (wall clock) at the vehicle's control rate,
exactly like the engine ticking with Use Fixed Frame Rate, and holds the last
action between steps (zero-order hold).
"""

from __future__ import annotations

import argparse
import json
import socket
import threading
import time

from .bridge import PROTOCOL_VERSION
from .mock_sim import MockBackend
from .specs import load_scene, load_vehicle


class FakeUnrealServer:
    def __init__(self, spec, vehicle=None, host: str = "127.0.0.1", port: int = 9800,
                 realtime: bool = True, dt: float | None = None):
        self.spec = spec
        self.vp = vehicle or spec.vehicle()
        self.dt = dt or 1.0 / float(self.vp.common.get("control_rate_hz", 50))
        self.backend = MockBackend(spec, self.vp, dt=self.dt)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.settimeout(0.05)
        self.client: tuple[str, int] | None = None
        self.realtime = realtime
        self.running = False
        self.u: dict[str, float] = {}
        self.seq = -1
        self.lock = threading.Lock()
        self.backend.reset()

    # --- protocol
    def _send(self, msg: dict) -> None:
        """acks/scene go to whoever asked; observations go to the controller (see _tick_loop)."""
        self._reply(getattr(self, "reply_to", None) or self.client, msg)

    def _reply(self, addr, msg: dict) -> None:
        if addr:
            self.sock.sendto(json.dumps(msg, separators=(",", ":")).encode(), addr)

    def handle(self, m: dict, addr) -> None:
        t = m.get("type")
        if t == "hello":
            ok = int(m.get("version", 0)) == PROTOCOL_VERSION
            if ok and m.get("role", "controller") != "editor":
                self.client = addr
            self._reply(addr, {"type": "ack", "for": "hello", "ok": ok, "error": None if ok else "version"})
            return
        self.reply_to = addr
        if t == "act":
            with self.lock:
                self.u = {k: float(v) for k, v in m.get("u", {}).items()}
                self.seq = int(m.get("seq", self.seq + 1))
        elif t == "reset":
            with self.lock:
                self.backend.reset(pose=m.get("pose"), seed=m.get("seed"), clear_dynamic=m.get("clear_dynamic", True))
                self.u, self.seq = {}, -1
            self._send({"type": "ack", "for": "reset", "ok": True, "tick": self.backend.tick})
        elif t == "set_params":
            try:
                with self.lock:
                    self.backend.set_params(load_vehicle(m["params"]))
                self._send({"type": "ack", "for": "set_params", "ok": True})
            except Exception as e:                                  # noqa: BLE001
                self._send({"type": "ack", "for": "set_params", "ok": False, "error": str(e)})
        elif t == "set_env":
            with self.lock:
                self.backend.set_env(**{k: v for k, v in m.items() if k != "type"})
            self._send({"type": "ack", "for": "set_env", "ok": True})
        elif t == "spawn":
            with self.lock:
                r = self.backend.spawn(m["kind"], m["label"], float(m["x"]), float(m["y"]), float(m.get("z", 0)),
                                       float(m.get("yaw", 0)), m.get("scale"), bool(m.get("movable", False)))
            self._send({"type": "ack", "for": "spawn", **r})
        elif t == "despawn":
            with self.lock:
                r = self.backend.despawn(m["label"])
            self._send({"type": "ack", "for": "despawn", **r})
        elif t == "query":
            self._send({"type": "scene", **self.backend.scene()})
        elif t == "bye":
            if addr == self.client:
                self.client = None

    # --- loops
    def serve(self, stop_after_s: float | None = None) -> None:
        self.running = True
        threading.Thread(target=self._tick_loop, daemon=True).start()
        t_end = time.monotonic() + stop_after_s if stop_after_s else None
        while self.running and (t_end is None or time.monotonic() < t_end):
            try:
                data, addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                self.handle(json.loads(data.decode()), addr)
            except Exception as e:                                  # noqa: BLE001
                self._send({"type": "ack", "for": "?", "ok": False, "error": str(e)})
        self.running = False

    def _tick_loop(self) -> None:
        next_t = time.monotonic()
        while self.running:
            with self.lock:
                obs = self.backend.step(self.u if self.u else None, self.seq)
            if self.client:
                self._reply(self.client, obs.to_wire())
            if self.realtime:
                next_t += self.dt
                lag = next_t - time.monotonic()
                if lag > 0:
                    time.sleep(lag)
                else:
                    next_t = time.monotonic()

    def stop(self) -> None:
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scene")
    ap.add_argument("--port", type=int, default=9800)
    ap.add_argument("--fast", action="store_true", help="run as fast as possible instead of real time")
    a = ap.parse_args()
    srv = FakeUnrealServer(load_scene(a.scene), port=a.port, realtime=not a.fast)
    print(f"fake Unreal on 127.0.0.1:{a.port}  scene={srv.spec.name}  dt={srv.dt}  (Ctrl-C to stop)")
    try:
        srv.serve()
    except KeyboardInterrupt:
        srv.stop()


if __name__ == "__main__":
    main()
