"""Start Play (Simulate by default) in the open Unreal editor through its MCP server, so the BoatPhysics
plugin opens its UDP port for the Unreal backend. Used by 08_Unreal_Mission / 13_Tune_Colors_Unreal.

    python tools/unreal_play.py              # Simulate-In-Editor (no player pawn, the viewport stays free)
    python tools/unreal_play.py --play       # Play-In-Editor in the viewport
    python tools/unreal_play.py --stop       # stop the play session

Exit codes: 0 playing, 2 editor / MCP server not reachable, 3 another level is open, 4 the editor refused.
Standard library only.
"""
from __future__ import annotations

import argparse
import json
import sys
import urllib.error
import urllib.request
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[2]
LEVEL = "/Game/BoatCourse/Lake_300x80ft"
APP = "EditorToolset.EditorAppToolset"


def mcp_url() -> str:
    try:
        return json.loads((PROJECT / ".mcp.json").read_text(encoding="utf-8"))["mcpServers"]["unreal-mcp"]["url"]
    except (OSError, KeyError, ValueError):
        return "http://127.0.0.1:8000/mcp"


class Mcp:
    def __init__(self, url: str):
        self.url, self.session, self.id = url, None, 0
        self._rpc("initialize", {"protocolVersion": "2025-06-18", "capabilities": {},
                                 "clientInfo": {"name": "boatlab-unreal-play", "version": "1"}})
        self._post({"jsonrpc": "2.0", "method": "notifications/initialized"})

    def _post(self, payload: dict):
        headers = {"Content-Type": "application/json", "Accept": "application/json, text/event-stream"}
        if self.session:
            headers["Mcp-Session-Id"] = self.session
        req = urllib.request.Request(self.url, json.dumps(payload).encode(), headers, method="POST")
        with urllib.request.urlopen(req, timeout=120) as r:
            self.session = r.headers.get("Mcp-Session-Id") or self.session
            body = r.read().decode("utf-8", "replace")
        if "data:" in body:                                   # server-sent events framing
            body = next((line[5:] for line in body.splitlines() if line.startswith("data:") and '"id"' in line), "")
        return json.loads(body) if body.strip() else None

    def _rpc(self, method: str, params: dict):
        self.id += 1
        reply = self._post({"jsonrpc": "2.0", "id": self.id, "method": method, "params": params})
        if reply and "error" in reply:
            raise RuntimeError(reply["error"])
        return reply["result"] if reply else None

    def tool(self, toolset: str, name: str, args: dict | None = None):
        res = self._rpc("tools/call", {"name": "call_tool",
                                       "arguments": {"toolset_name": toolset, "tool_name": name, "arguments": args or {}}})
        text = "".join(c.get("text", "") for c in res.get("content", []))
        if res.get("isError"):
            raise RuntimeError(text)
        return json.loads(text).get("returnValue") if text.strip().startswith("{") else text


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--play", action="store_true", help="Play-In-Editor in the viewport instead of Simulate")
    ap.add_argument("--stop", action="store_true")
    ap.add_argument("--url", default=mcp_url())
    a = ap.parse_args()
    try:
        mcp = Mcp(a.url)
        playing = mcp.tool(APP, "IsPIERunning")
    except (urllib.error.URLError, OSError, RuntimeError, ValueError) as e:
        print(f"Unreal editor not reachable at {a.url} ({e}).\n"
              "Open the project (03_Open_Unreal.cmd) or press Play in the lake level yourself.")
        return 2
    if a.stop:
        if playing:
            mcp.tool(APP, "StopPIE")
        print("Play stopped.")
        return 0
    if playing:
        print("Unreal: Play is already running.")
        return 0
    level = mcp.tool("editor_toolset.toolsets.scene.SceneTools", "get_current_level")
    if level != LEVEL:
        print(f"Unreal has {level} open; open {LEVEL} and press Play.")
        return 3
    mode = {"bSimulate": False, "playMode": "PlayMode_InViewPort"} if a.play else \
        {"bSimulate": True, "playMode": "PlayMode_Simulate"}
    try:
        mcp.tool(APP, "StartPIE", {"options": dict(mode, warmupSeconds=2)})
    except RuntimeError as e:
        print(f"Unreal refused to start Play: {e}")
        return 4
    print(f"Unreal: {'Play' if a.play else 'Simulate'} started in {LEVEL}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
