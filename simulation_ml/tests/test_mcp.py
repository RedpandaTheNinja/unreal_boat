"""Regression coverage for the Codex adapter, file integrity and live UDP channel."""
from __future__ import annotations

import asyncio
import copy
import importlib.util
import json
import shutil
import sys
import threading
from datetime import timedelta
from pathlib import Path

import httpx
import pytest
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

ROOT = Path(__file__).resolve().parents[1]
module_spec = importlib.util.spec_from_file_location("codex_mcp", ROOT / "unreal_fable/mcp/codex_mcp.py")
server = importlib.util.module_from_spec(module_spec)
module_spec.loader.exec_module(server)


@pytest.fixture
def isolated(monkeypatch, tmp_path):
    specs = tmp_path / "Kevin's worlds" / "specs"
    shutil.copytree(ROOT / "unreal_fable/specs", specs)
    monkeypatch.setattr(server, "SPECS", str(specs))
    monkeypatch.setattr(server, "_current", {"spec": None, "path": ""})

    def offline(*args, **kwargs):
        raise RuntimeError("editor offline")

    monkeypatch.setattr(server, "ue_python", offline)
    server.load_world("car_track_oval")
    return specs


def test_invalid_edit_preserves_disk_and_selection(isolated):
    target = Path(server._current["path"])
    original = target.read_bytes()
    selected = copy.deepcopy(server._current)
    with pytest.raises(Exception):
        server.add_obstacle("not_a_kind", 1, 2)
    assert target.read_bytes() == original
    assert server._current == selected
    with pytest.raises(ValueError):
        server.set_route("[[0, 0]]")
    assert target.read_bytes() == original


def test_save_world_and_invalid_name(isolated):
    spec = json.loads(server.read_world())
    spec["name"] = "my_test_world"
    saved = json.loads(server.save_world(json.dumps(spec)))
    assert saved["applied"] == "spec only"
    assert json.loads(Path(saved["path"]).read_text())["name"] == "my_test_world"
    with pytest.raises(ValueError, match="already exists"):
        server.save_world(json.dumps(spec))
    spec["name"] = "../escaped"
    with pytest.raises(ValueError, match="World name"):
        server.save_world(json.dumps(spec))
    assert not (isolated / "escaped.json").exists()


def test_read_does_not_change_edit_target(isolated):
    selected = copy.deepcopy(server._current)
    assert json.loads(server.read_world("boat_docking"))["scenario"] == "boat"
    assert json.loads(server.validate_world("boat_docking"))["valid"]
    assert server._current == selected


def test_editor_offline_is_explicit(isolated):
    result = json.loads(server.add_obstacle("cone", 2, 3, label="added"))
    assert result["applied"].startswith("spec only")
    assert json.loads(Path(server._current["path"]).read_text())["objects"][-1]["label"] == "added"
    result = json.loads(server.add_gate(0, 0, 1, 1, label="gate's label"))
    assert result["applied"].startswith("spec only")


def test_generated_editor_code_handles_quotes(isolated, monkeypatch):
    calls = []

    def editor(code, **kwargs):
        compile(code, "editor-command", "exec")
        calls.append(code)
        return {"ReturnValue": True, "LogOutput": [{"Output": "FABLE_RESULT {}"}]}

    monkeypatch.setattr(server, "ue_python", editor)
    monkeypatch.setattr(server, "UE_PY", "C:/Kevin's project/Content/Python")
    server.build_world("car_track_oval")
    server.set_route("[[0, 0], [3, 4]]")
    server.add_gate(0, 0, 1, 1, label="gate's label")
    assert len(calls) == 3


def test_remote_failures_are_actionable(monkeypatch):
    def failed(*args, **kwargs):
        return httpx.Response(503, request=httpx.Request("PUT", "http://127.0.0.1/remote/object/call"))

    monkeypatch.setattr(server.httpx, "put", failed)
    with pytest.raises(RuntimeError, match="WebControl.StartServer"):
        server.ue_python("pass")
    with pytest.raises(RuntimeError, match="did not contain"):
        server._pull({"LogOutput": []}, "FABLE_RESULT")


def test_run_test_uses_custom_port_and_reports_failures(isolated, monkeypatch):
    calls = []

    class Process:
        returncode = 2

        async def communicate(self):
            return b"", b"intentional runner failure"

    async def spawn(*args, **kwargs):
        calls.append(args)
        return Process()

    monkeypatch.setattr(server.asyncio, "create_subprocess_exec", spawn)
    monkeypatch.setattr(server, "BRIDGE_PORT", 9981)
    with pytest.raises(RuntimeError, match="intentional runner failure"):
        asyncio.run(server.run_test(backend="unreal"))
    assert calls[0][-2:] == ("--port", "9981")
    with pytest.raises(ValueError):
        asyncio.run(server.run_test(seeds=0))


def test_stdio_tools_and_live_editor_preserve_controller(tmp_path):
    from fable import load_scene
    from fable.bridge import UnrealBackend
    from fable.fake_unreal import FakeUnrealServer

    scene = load_scene("car_track_oval")
    fake = FakeUnrealServer(scene, port=0)
    port = fake.sock.getsockname()[1]
    thread = threading.Thread(target=fake.serve, daemon=True)
    thread.start()

    async def exercise():
        params = StdioServerParameters(
            command=sys.executable, args=[str(ROOT / "unreal_fable/mcp/codex_mcp.py")],
            cwd=str(tmp_path), env={"FABLE_BRIDGE_PORT": str(port), "PYTHONUTF8": "1"})
        async with stdio_client(params) as (read, write):
            async with ClientSession(read, write, read_timeout_seconds=timedelta(seconds=30)) as client:
                await client.initialize()
                tools = (await client.list_tools()).tools
                assert len(tools) == 20
                for tool in tools:
                    server.jsonschema.Draft202012Validator.check_schema(tool.inputSchema)
                assert next(t for t in tools if t.name == "read_world").annotations.readOnlyHint
                result = await client.call_tool("list_worlds", {})
                assert not result.isError
                assert len(json.loads(result.content[0].text)) >= 4
                error = await client.call_tool("load_world", {"name_or_path": "does_not_exist"})
                assert error.isError
                spawned = await client.call_tool("live_spawn", {"kind": "cone", "label": "mcp_cone", "x_m": 2, "y_m": 3})
                assert not spawned.isError
                info = await client.call_tool("live_scene", {})
                assert "mcp_cone" in info.content[0].text
                result = await client.call_tool("live_set_env", {"wind_speed_mps": 2})
                assert not result.isError
                removed = await client.call_tool("live_despawn", {"label": "mcp_cone"})
                assert not removed.isError
                # Exercise a real child process under MCP's Windows stdin pipe.
                report = await client.call_tool("run_test", {"name_or_path": "boat_buoy_course"})
                assert not report.isError
                assert json.loads(report.content[0].text)["succeeded"] == 1
                assert controller.step({"throttle": 0.1}).tick > first_tick

    controller = None
    try:
        controller = UnrealBackend(scene, port=port)
        first_tick = controller.reset().tick
        asyncio.run(exercise())
    finally:
        if controller:
            controller.close()
        fake.stop()
        thread.join(timeout=2)
