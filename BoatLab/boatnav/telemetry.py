"""Small localhost HTTP API exposed by the mission runner (stdlib only).

GET  /api/state    latest snapshot (JSON)
GET  /camera.jpg   latest annotated camera frame
POST /api/command  {"cmd": "pause" | "resume" | "skip" | "goto" | "abort" | "estop" | "set_color" |
                    "waypoints" | "reset" | "env" | "quit", ...}
The dashboard server (dashboard/server.py) proxies these, so the browser only talks to one port.
"""
from __future__ import annotations

import json
import queue
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class TelemetryHub:
    def __init__(self):
        self.lock = threading.Lock()
        self.snapshot: dict = {"schema": "boatlab_state_v1", "status": "starting"}
        self.jpeg: bytes | None = None
        self.commands: "queue.Queue[dict]" = queue.Queue()

    def publish(self, snap: dict, jpeg: bytes | None = None):
        with self.lock:
            self.snapshot = snap
            if jpeg is not None:
                self.jpeg = jpeg

    def get(self):
        with self.lock:
            return self.snapshot, self.jpeg


def serve(hub: TelemetryHub, port: int = 8772):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, body: bytes, mime="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            snap, jpeg = hub.get()
            if self.path.startswith("/api/state"):
                self._send(200, json.dumps(snap, default=float).encode())
            elif self.path.startswith("/camera.jpg"):
                if jpeg is None:
                    self._send(503, b'{"error":"no frame"}')
                else:
                    self._send(200, jpeg, "image/jpeg")
            else:
                self._send(404, b'{"error":"not found"}')

        def do_POST(self):
            if not self.path.startswith("/api/command"):
                self._send(404, b'{"error":"not found"}')
                return
            n = int(self.headers.get("Content-Length", "0") or 0)
            try:
                msg = json.loads(self.rfile.read(n) or b"{}")
                if not isinstance(msg, dict) or "cmd" not in msg:
                    raise ValueError("expected {\"cmd\": ...}")
            except ValueError as e:
                self._send(400, json.dumps({"ok": False, "error": str(e)}).encode())
                return
            hub.commands.put(msg)
            self._send(200, b'{"ok": true, "queued": true}')

        def log_message(self, *_):
            pass

    srv = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    return srv
