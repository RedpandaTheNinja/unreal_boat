"""Fake Unreal BoatPhysics server: speaks the plugin's UDP protocol (v2, UE axes) using the
Python twin for physics and the synthetic renderer for the camera.

Use it to test `--backend unreal` without Unreal (CI, a laptop without the engine):
    python tools/fake_unreal.py            # terminal 1
    python -m boatnav.runner --backend unreal --mission aimm_full   # terminal 2
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import socket
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from boatnav.config import load_boat, load_course  # noqa: E402
from boatnav.hal.twin import TwinBoat  # noqa: E402

CHUNK = 24000


class FakeUnreal:
    def __init__(self, port=7450, speed=1.0, depth=True):
        self.boat, self.course = load_boat(), load_course()
        self.twin = TwinBoat(self.boat, self.course, camera=True, gps_noise_m=0.0)
        self.lock = threading.Lock()
        self.speed = speed
        self.depth_on = depth
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        self.seq = 0
        self.jpeg = b""
        self.png = b""
        self.cam_t = -1.0
        self.env = {}

    def state(self, accepted=True):
        t = self.twin
        c, s = math.cos(t.h), math.sin(t.h)
        ve, vn = c * t.u - s * t.v, s * t.u + c * t.v
        return {"schema": "boat_physics_v1", "state_source": "fake_unreal_twin", "calibration": "uncalibrated_initial_estimates",
                "time_s": t.t, "physics_dt_s": 1 / 120, "mass_kg": t.model.mass,
                "position_ue_m": [t.x, -t.y, 0.125], "velocity_ue_mps": [ve, -vn, 0.0],
                "angular_velocity_body_radps": [0.0, 0.0, -t.r], "roll_pitch_yaw_deg": [0.0, 0.0, -math.degrees(t.h)],
                "left_thrust_n": t.tl, "right_thrust_n": t.tr, "battery_voltage": 12.0,
                "command_expired": t.t - t.last_cmd_t > t.model.timeout, "deck_awash": False, "camera_available": True,
                "plugin_version": "boatlab_v2_fake", "accepted": accepted}

    def capture(self):
        if self.cam_t >= 0 and self.twin.t - self.cam_t < 0.1:
            return
        f = self.twin.camera()
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(f.rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 90])
        self.jpeg = buf.tobytes()
        if self.depth_on:
            d = cv2.resize(f.depth, (320, 240), interpolation=cv2.INTER_NEAREST)
            mm = np.where((d > 0) & (d < 65.5), d * 1000, 0).astype(np.uint16)
            ok, pbuf = cv2.imencode(".png", mm)
            self.png = pbuf.tobytes()
        self.seq += 1
        self.cam_t = self.twin.t

    def meta(self):
        cam = self.boat["camera"]
        return {"valid": True, "sequence": self.seq, "stamp_s": self.cam_t, "width": cam["width"], "height": cam["height"],
                "horizontal_fov_deg": cam["hfov_deg"], "pitch_down_deg": cam["pitch_down_deg"],
                "mount_position_body_flu_m": cam["mount_flu_m"], "encoding": "jpeg", "byte_count": len(self.jpeg),
                "chunk_count": (len(self.jpeg) + CHUNK - 1) // CHUNK}

    def depth_meta(self):
        return {"valid": self.depth_on and bool(self.png), "sequence": self.seq, "width": 320, "height": 240,
                "encoding": "png_uint16_mm", "byte_count": len(self.png), "chunk_count": (len(self.png) + CHUNK - 1) // CHUNK}

    def handle(self, m):
        tw = self.twin
        if "left" in m or "right" in m:
            l, r = m.get("left"), m.get("right")
            if not all(isinstance(v, (int, float)) and abs(v) <= 1 for v in (l, r)):
                return self.state(False)
            tw.command(l, r)
        if m.get("reset"):
            tw.reset()
        if "reset_pose" in m:
            x, y, yaw = m["reset_pose"]
            tw.reset((x, -y, -math.radians(yaw)))
        if "environment" in m:
            e = m["environment"]
            tw.set_environment((e["current_x"], -e["current_y"]), (e["wind_x"], -e["wind_y"]))
        if m.get("rgb"):
            self.capture()
            o = self.state()
            o["valid"] = True
            o["rgb"] = self.meta()
            o["depth"] = self.depth_meta()
            return o
        for stream, data in (("rgb", self.jpeg), ("depth", self.png)):
            if f"{stream}_sequence" in m:
                i = int(m[f"{stream}_chunk"])
                ok = m[f"{stream}_sequence"] == self.seq and 0 <= i * CHUNK < len(data)
                o = {f"{stream}_sequence": self.seq, f"{stream}_chunk": i, "valid": ok}
                if ok:
                    o["data_base64"] = base64.b64encode(data[i * CHUNK:(i + 1) * CHUNK]).decode()
                return o
        return self.state()

    def physics(self):
        last = time.monotonic()
        while True:
            time.sleep(0.01)
            now = time.monotonic()
            with self.lock:
                self.twin.tick((now - last) * self.speed)
            last = now

    def serve(self):
        threading.Thread(target=self.physics, daemon=True).start()
        print(f"fake Unreal BoatPhysics on udp://127.0.0.1:{self.sock.getsockname()[1]} (Ctrl+C to stop)", flush=True)
        while True:
            data, addr = self.sock.recvfrom(65535)
            try:
                m = json.loads(data)
            except ValueError:
                continue
            with self.lock:
                out = self.handle(m)
            self.sock.sendto(json.dumps(out).encode(), addr)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=7450)
    ap.add_argument("--no-depth", action="store_true")
    a = ap.parse_args()
    FakeUnreal(a.port, depth=not a.no_depth).serve()
