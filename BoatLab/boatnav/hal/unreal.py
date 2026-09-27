"""Unreal Engine 5.8 backend over the BoatPhysics plugin UDP interface (127.0.0.1:7450).

Existing plugin protocol (boat_physics_v1):
    {"left": L, "right": R}  -> motor commands in [-1, 1], expire after 0.5 s
    {}                        -> read-only state
    {"reset": true}           -> restore the Play-start pose
    {"environment": {...}}    -> current/wind/waves (UE axes)
BoatLab additions (plugin v2, see unreal/BoatPhysics):
    {"rgb": true}                              -> capture; reply has "rgb" + "depth" metadata
    {"rgb_sequence": s, "rgb_chunk": i}        -> base64 JPEG chunk
    {"depth_sequence": s, "depth_chunk": i}    -> base64 16-bit PNG (millimetres) chunk
    {"reset_pose": [x_m, y_m, yaw_deg]}        -> reset to a pose (UE axes)
    {"visual": {...}}                          -> spawn a marker / hide the recovered case

Frame conversion (the only place UE axes appear in Python):
    x = UE_X, y = -UE_Y, heading = -radians(UE yaw), r = -omega_z(UE body)
"""
from __future__ import annotations

import base64
import json
import math
import socket
import threading
import time

import cv2
import numpy as np

from .. import geometry as G
from ..evaluation import simulate_actuator
from ..geo import LocalFrame
from ..types import BoatState, CameraFrame
from .base import BoatInterface


class UnrealBoat(BoatInterface):
    name = "unreal"
    is_sim = True
    realtime = True

    def __init__(self, boat_cfg: dict, course: dict, port: int = 7450, host: str = "127.0.0.1", camera: bool = True,
                 gps_noise_m: float | None = None, seed: int = 0, timeout: float = 0.5, connect_wait_s: float = 0.0, **_):
        """connect_wait_s: keep retrying this long for the plugin to answer (it only listens during Play)."""
        self.cfg, self.course = boat_cfg, course
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)
        self.frame = LocalFrame.from_course(course)
        g = boat_cfg.get("gps", {})
        self.gps_sigma = g.get("sim_noise_m", 0.02) if gps_noise_m is None else gps_noise_m
        self.head_sigma = math.radians(g.get("sim_heading_noise_deg", 0.5))
        self.rng = np.random.default_rng(seed)
        self.bias = np.zeros(2)
        self.want_camera = camera
        self.camera_ok = None
        self.last = None
        self.last_rx = 0.0
        self.hidden: set[str] = set()
        self.actions = []
        self._warned = set()
        self.camera_period = 0.25
        self._cam_thread = None
        self._cam_frame = None
        self._cam_lock = threading.Lock()
        self._cam_stop = threading.Event()
        self.last = self._request({})
        deadline = time.monotonic() + connect_wait_s
        if self.last is None and connect_wait_s > 0:
            print(f"Waiting up to {connect_wait_s:.0f} s for Unreal on UDP {host}:{port}: open the lake level and press "
                  "Play (or Simulate)...", flush=True)
        while self.last is None and time.monotonic() < deadline:
            time.sleep(1.0)
            self.last = self._request({})
        if self.last is None:
            raise ConnectionError("No reply from Unreal on UDP %s:%d. Open the lake level, press Play, and check the "
                                  "BoatDynamics component (CommandPort)." % self.addr)
        if self.last.get("schema") != "boat_physics_v1":
            raise ConnectionError(f"Unexpected reply schema {self.last.get('schema')!r} on port {port}")

    # ------------------------------------------------------------------ transport
    def _request(self, msg: dict, accept=None, tries: int = 2):
        data = json.dumps(msg, allow_nan=False).encode()
        for _ in range(tries):
            try:
                self.sock.sendto(data, self.addr)
                deadline = time.monotonic() + self.sock.gettimeout()
                while time.monotonic() < deadline:
                    reply = json.loads(self.sock.recv(65535))
                    if accept is None or accept(reply):
                        if reply.get("schema") == "boat_physics_v1":
                            self.last, self.last_rx = reply, time.monotonic()
                        return reply
            except (socket.timeout, ConnectionResetError, OSError, ValueError):
                continue
        return None

    # ------------------------------------------------------------------ conversions
    @staticmethod
    def _to_local(rep: dict):
        X, Y, _ = rep["position_ue_m"]
        vx, vy, _ = rep["velocity_ue_mps"]
        yaw = rep["roll_pitch_yaw_deg"][2]
        h = G.wrap(-math.radians(yaw))
        ve, vn = vx, -vy
        c, s = math.cos(h), math.sin(h)
        u, v = c * ve + s * vn, -s * ve + c * vn
        r = -rep["angular_velocity_body_radps"][2]
        return X, -Y, h, u, v, r

    def truth(self) -> dict | None:
        if not self.last:
            return None
        x, y, h, u, v, r = self._to_local(self.last)
        return {"t": self.last["time_s"], "x": x, "y": y, "heading": h, "u": u, "v": v, "r": r,
                "left_thrust_n": self.last.get("left_thrust_n", 0.0), "right_thrust_n": self.last.get("right_thrust_n", 0.0),
                "hidden": sorted(self.hidden)}

    # ------------------------------------------------------------------ interface
    def state(self) -> BoatState:
        if self.last is None or time.monotonic() - self.last_rx > 0.3:
            self._request({})
        rep = self.last
        valid = rep is not None and time.monotonic() - self.last_rx < 1.0
        x, y, h, u, v, r = self._to_local(rep)
        tau = 120.0
        self.bias += (-self.bias / tau) * 0.05 + self.rng.normal(0, self.gps_sigma * math.sqrt(2 * 0.05 / tau), 2)
        nx = x + self.bias[0] + self.rng.normal(0, self.gps_sigma * 0.1)
        ny = y + self.bias[1] + self.rng.normal(0, self.gps_sigma * 0.1)
        nh = G.wrap(h + self.rng.normal(0, self.head_sigma))
        lat, lon = self.frame.to_geo(nx, ny)
        return BoatState(rep["time_s"], nx, ny, nh, u, v, r, lat, lon, rep.get("left_thrust_n", 0.0),
                         rep.get("right_thrust_n", 0.0), valid, f"unreal_truth+gps({self.gps_sigma:.2f}m)",
                         {"command_expired": rep.get("command_expired"), "deck_awash": rep.get("deck_awash")})

    def command(self, left: float, right: float):
        l, r = float(np.clip(left, -1, 1)), float(np.clip(right, -1, 1))
        self._request({"left": round(l, 4), "right": round(r, 4)}, tries=1)

    def reset(self, pose=None):
        if pose is None:
            self._request({"reset": True})
        else:
            x, y, h = pose
            self._request({"reset_pose": [x, -y, -math.degrees(h)]})
        self.hidden.clear()
        self._request({"visual": {"kind": "clear"}})

    def set_environment(self, current=(0.0, 0.0), wind=(0.0, 0.0), waves=None, fog_visibility_m=None):
        w = waves or {}
        env = {"current_x": float(current[0]), "current_y": -float(current[1]), "wind_x": float(wind[0]),
               "wind_y": -float(wind[1]), "wave_amplitude": float(w.get("amplitude", 0.0)),
               "wave_period": float(w.get("period", 3.0)), "wave_length": float(w.get("length", 8.0)),
               "wave_direction": float(w.get("direction", 0.0))}
        rep = self._request({"environment": env})
        return {"ok": bool(rep and rep.get("accepted", False)), "fog": "set fog in the level (ExponentialHeightFog)"}

    # ------------------------------------------------------------------ camera
    # Frames are fetched on a background thread with its own UDP socket so chunk transfers
    # never delay motor commands on the control loop.
    def _cam_request(self, sock, msg, accept):
        data = json.dumps(msg).encode()
        for _ in range(2):
            try:
                sock.sendto(data, self.addr)
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    reply = json.loads(sock.recv(65535))
                    if accept(reply):
                        return reply
            except (socket.timeout, ConnectionResetError, OSError, ValueError):
                continue
        return None

    def _fetch(self, sock, stream: str, seq: int, count: int) -> bytes | None:
        chunks = []
        for i in range(count):
            rep = self._cam_request(sock, {f"{stream}_sequence": seq, f"{stream}_chunk": i},
                                    lambda r, i=i: r.get(f"{stream}_chunk") == i)
            if not rep or not rep.get("valid") or rep.get(f"{stream}_sequence") != seq:
                return None
            chunks.append(base64.b64decode(rep["data_base64"]))
        return b"".join(chunks)

    def _grab(self, sock) -> CameraFrame | None:
        rep = self._cam_request(sock, {"rgb": True}, lambda r: "rgb" in r or r.get("schema") == "boat_physics_v1")
        if rep is None or "rgb" not in rep:
            if self.camera_ok is None:
                print("[unreal] Camera not available: install the BoatLab BoatPhysics plugin v2 "
                      "(BoatLab/unreal/build_install_plugin.ps1). Continuing without vision.", flush=True)
                self.camera_ok = False
            return None
        meta = rep["rgb"]
        if not meta.get("valid"):
            return None
        self.camera_ok = True
        jpeg = self._fetch(sock, "rgb", meta["sequence"], meta["chunk_count"])
        if jpeg is None:
            return None
        bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return None
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        depth = None
        dm = rep.get("depth") or {}
        if dm.get("valid"):
            png = self._fetch(sock, "depth", dm["sequence"], dm["chunk_count"])
            if png is not None:
                d16 = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED)
                if d16 is not None:
                    depth = cv2.resize(d16.astype(np.float32) / 1000.0, (rgb.shape[1], rgb.shape[0]),
                                       interpolation=cv2.INTER_NEAREST)
        W, H = meta["width"], meta["height"]
        f = W / (2 * math.tan(math.radians(meta["horizontal_fov_deg"]) / 2))
        x, y, h, *_ = self._to_local(rep)
        return CameraFrame(rep["time_s"], rgb, depth, f, f, W / 2, H / 2, tuple(meta["mount_position_body_flu_m"]),
                           math.radians(meta.get("pitch_down_deg", 0.0)), (x, y, h), meta["sequence"], "unreal_scene_capture")

    def _camera_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.5)
        try:
            while not self._cam_stop.is_set() and self.camera_ok is not False:
                t0 = time.monotonic()
                fr = self._grab(sock)
                if fr is not None:
                    with self._cam_lock:
                        self._cam_frame = fr
                self._cam_stop.wait(max(0.02, self.camera_period - (time.monotonic() - t0)))
        finally:
            sock.close()

    def camera(self) -> CameraFrame | None:
        if not self.want_camera or self.camera_ok is False:
            return None
        if self._cam_thread is None:
            self._cam_thread = threading.Thread(target=self._camera_loop, daemon=True)
            self._cam_thread.start()
        with self._cam_lock:
            fr, self._cam_frame = self._cam_frame, None
        return fr

    # ------------------------------------------------------------------ mechanisms (simulated outcome)
    def actuate(self, name: str, **kw) -> dict:
        t = self.truth()
        res = simulate_actuator(name, (t["x"], t["y"], t["heading"]), self.course, self.cfg, self.rng, hidden=self.hidden)
        res["t"] = t["t"]
        res["note"] = "outcome simulated in Python from Unreal truth pose"
        if "landing" in res:
            lx, ly = res["landing"]
            color = [1, 1, 0] if name == "deploy" else [1, 0.3, 0]
            self._request({"visual": {"kind": "marker", "x": lx, "y": -ly, "z": 0.1, "size": 0.25,
                                      "r": color[0], "g": color[1], "b": color[2]}})
        if res.get("hide"):
            self.hidden.add(res["hide"])
            self._request({"visual": {"kind": "hide_tag", "tag": "boatlab_" + res["hide"]}})
        self.actions.append(res)
        return res

    def close(self):
        self._cam_stop.set()
        if self._cam_thread is not None:
            self._cam_thread.join(timeout=2.0)
        self.stop()
        self.sock.close()
