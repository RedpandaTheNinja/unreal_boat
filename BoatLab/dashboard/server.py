"""BoatLab dashboard server - Python standard library only (runs with Unreal's bundled Python too).

    python dashboard/server.py [--port 8770] [--runner 8772] [--ue-port 7450]

Sources, in order:
  1. Mission runner API on 127.0.0.1:<runner>  (full mission, planner, vision, score, commands)
  2. Unreal BoatPhysics UDP on 127.0.0.1:<ue-port>, read-only (boat pose/thrust + camera, no autonomy)
  3. offline
The dashboard can also start/stop the runner (twin or Unreal backend) from the browser.
Pattern follows RacingLab's F1 dashboard (simulation_ml/integrations/rapyuta/dashboard).
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CONFIG = ROOT / "config"


class UnrealDirect:
    """Read-only BoatPhysics poller used when no mission runner is active."""

    def __init__(self, port):
        self.addr = ("127.0.0.1", port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.3)
        self.lock = threading.Lock()
        self.state, self.rx = None, 0.0
        self.jpeg, self.jpeg_seq = None, -1
        self.trail = []
        threading.Thread(target=self.run, daemon=True).start()

    def req(self, msg, accept=lambda r: True):
        try:
            self.sock.sendto(json.dumps(msg).encode(), self.addr)
            end = time.monotonic() + 0.3
            while time.monotonic() < end:
                r = json.loads(self.sock.recv(65535))
                if accept(r):
                    return r
        except (OSError, ValueError):
            return None
        return None

    def run(self):
        next_cam = 0.0
        while True:
            if RUNNER.alive():
                time.sleep(0.5)
                continue
            r = self.req({}, lambda r: r.get("schema") == "boat_physics_v1")
            if r:
                with self.lock:
                    self.state, self.rx = r, time.monotonic()
                    X, Y, _ = r["position_ue_m"]
                    self.trail.append([round(X, 2), round(-Y, 2)])
                    del self.trail[:-1500]
                if time.monotonic() > next_cam and r.get("camera_available"):
                    next_cam = time.monotonic() + 0.3
                    self.grab()
            time.sleep(0.1)

    def grab(self):
        m = self.req({"rgb": True}, lambda r: "rgb" in r)
        if not m or not m["rgb"].get("valid") or m["rgb"]["sequence"] == self.jpeg_seq:
            return
        seq, parts = m["rgb"]["sequence"], []
        for i in range(m["rgb"]["chunk_count"]):
            c = self.req({"rgb_sequence": seq, "rgb_chunk": i}, lambda r, i=i: r.get("rgb_chunk") == i)
            if not c or not c.get("valid"):
                return
            parts.append(base64.b64decode(c["data_base64"]))
        with self.lock:
            self.jpeg, self.jpeg_seq = b"".join(parts), seq

    def snapshot(self):
        with self.lock:
            r, age = self.state, time.monotonic() - self.rx
            if r is None or age > 2.0:
                return None
            X, Y, _ = r["position_ue_m"]
            vx, vy, _ = r["velocity_ue_mps"]
            h = -math.radians(r["roll_pitch_yaw_deg"][2])
            ve, vn = vx, -vy
            u = math.cos(h) * ve + math.sin(h) * vn
            return {"schema": "boatlab_state_v1", "status": "ue_direct", "backend": "unreal (read-only)", "t": r["time_s"],
                    "state": {"x": X, "y": -Y, "heading": h, "u": u, "v": 0.0, "r": -r["angular_velocity_body_radps"][2],
                              "speed": math.hypot(vx, vy), "left_thrust_n": r["left_thrust_n"], "right_thrust_n": r["right_thrust_n"],
                              "source": r.get("state_source", "")},
                    "cmd": None, "trail": list(self.trail), "camera": {"sequence": self.jpeg_seq},
                    "ue": {"plugin_version": r.get("plugin_version", "boatlab_v1 (no camera)"), "command_expired": r.get("command_expired")}}


class RunnerLink:
    def __init__(self, port):
        self.base = f"http://127.0.0.1:{port}"
        self.port = port
        self.proc = None
        self.last_ok = 0.0
        self.log_path = ROOT / "runs" / "runner_console.log"

    def alive(self):
        return time.monotonic() - self.last_ok < 1.5

    def get(self, path, timeout=0.6):
        try:
            with urllib.request.urlopen(self.base + path, timeout=timeout) as r:
                data = r.read()
            self.last_ok = time.monotonic()
            return data
        except (urllib.error.URLError, OSError, TimeoutError):
            return None

    def post(self, msg):
        req = urllib.request.Request(self.base + "/api/command", data=json.dumps(msg).encode(),
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=1.0) as r:
                return json.loads(r.read())
        except (urllib.error.URLError, OSError, ValueError) as e:
            return {"ok": False, "error": f"runner not reachable: {e}"}

    def launch(self, msg):
        if self.alive():
            return {"ok": False, "error": "a runner is already active - stop it first"}
        python = msg.get("python") or SETTINGS.get("python") or sys.executable
        if not Path(python).exists():
            python = sys.executable
        args = [python, "-m", "boatnav.runner", "--backend", msg.get("backend", "twin"),
                "--mission", msg.get("mission", "aimm_full"), "--api-port", str(self.port), "--stay"]
        if msg.get("backend", "twin") == "twin":
            args.append("--realtime")
        if msg.get("color"):
            args += ["--color", msg["color"]]
        if msg.get("nav"):
            args += ["--nav", msg["nav"]]
        if msg.get("paused"):
            args.append("--start-paused")
        if msg.get("no_camera"):
            args.append("--no-camera")
        if msg.get("dry_run"):
            args.append("--dry-run")
        self.log_path.parent.mkdir(exist_ok=True)
        log = open(self.log_path, "w", encoding="utf-8")
        flags = subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0
        self.proc = subprocess.Popen(args, cwd=str(ROOT), stdout=log, stderr=subprocess.STDOUT, creationflags=flags)
        return {"ok": True, "pid": self.proc.pid, "args": args[1:], "log": str(self.log_path)}


def list_missions():
    out = []
    for p in sorted((CONFIG / "missions").glob("*.json")):
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
            out.append({"name": p.stem, "description": d.get("description", ""), "tasks": len(d.get("tasks", []))})
        except ValueError:
            pass
    return out


def main():
    global RUNNER, SETTINGS
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--runner", type=int, default=8772)
    ap.add_argument("--ue-port", type=int, default=7450)
    ap.add_argument("--course", default=str(CONFIG / "course_aimm_2025.json"))
    a = ap.parse_args()
    SETTINGS = json.loads((ROOT / "settings.json").read_text()) if (ROOT / "settings.json").exists() else {}
    RUNNER = RunnerLink(a.runner)
    ue = UnrealDirect(a.ue_port)
    course = json.loads(Path(a.course).read_text(encoding="utf-8"))
    boat = json.loads((CONFIG / "boat.json").read_text(encoding="utf-8"))

    class Handler(BaseHTTPRequestHandler):
        def send(self, code, body, mime="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/api/state":
                data = RUNNER.get("/api/state")
                if data is not None:
                    snap = json.loads(data)
                    snap["source"] = "runner"
                else:
                    snap = ue.snapshot() or {"schema": "boatlab_state_v1", "status": "offline"}
                    snap["source"] = "unreal_direct" if snap.get("status") == "ue_direct" else "none"
                snap["dashboard"] = {"runner_port": a.runner, "ue_port": a.ue_port,
                                     "runner_process": RUNNER.proc.poll() if RUNNER.proc else None}
                self.send(200, json.dumps(snap).encode())
            elif path == "/api/course":
                self.send(200, json.dumps({"course": course, "boat": {"hull": boat["hull"], "mechanisms": boat["mechanisms"],
                                                                       "camera": boat["camera"]},
                                           "missions": list_missions()}).encode())
            elif path == "/camera.jpg":
                data = RUNNER.get("/camera.jpg") if RUNNER.alive() else None
                if data is None:
                    with ue.lock:
                        data = ue.jpeg
                if data is None:
                    self.send(503, b'{"error":"no camera frame"}')
                else:
                    self.send(200, data, "image/jpeg")
            elif path in ("/", "/index.html"):
                self.send(200, (HERE / "index.html").read_bytes(), "text/html; charset=utf-8")
            elif path == "/dashboard.js":
                self.send(200, (HERE / "dashboard.js").read_bytes(), "text/javascript; charset=utf-8")
            else:
                self.send(404, b'{"error":"not found"}')

        def do_POST(self):
            if urlsplit(self.path).path != "/api/command":
                self.send(404, b'{"error":"not found"}')
                return
            n = int(self.headers.get("Content-Length", "0") or 0)
            try:
                msg = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                self.send(400, b'{"ok":false,"error":"bad json"}')
                return
            if msg.get("cmd") == "launch":
                res = RUNNER.launch(msg)
            elif msg.get("cmd") == "stop_runner":
                res = RUNNER.post({"cmd": "quit"})
            else:
                res = RUNNER.post(msg)
            self.send(200, json.dumps(res).encode())

        def log_message(self, *_):
            pass

    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    print(f"BoatLab dashboard http://127.0.0.1:{a.port}  (runner api :{a.runner}, unreal udp :{a.ue_port})", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


RUNNER = None
SETTINGS = {}

if __name__ == "__main__":
    main()
