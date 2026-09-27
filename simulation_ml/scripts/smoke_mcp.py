"""Exercise the actual stdio MCP server. No scene files are changed."""
from __future__ import annotations

import argparse
import asyncio
import json
import sys
import tomllib
from datetime import timedelta
from pathlib import Path

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

ROOT = Path(__file__).resolve().parents[1]


async def smoke(
    check_connections: bool = False,
    run_worlds: bool = False,
    config_path: Path | None = None,
    worlds_filter: list[str] | None = None,
    backend: str = "mock",
    seeds: int = 1,
) -> dict:
    params = StdioServerParameters(
        command=sys.executable,
        args=[str(ROOT / "unreal_fable/mcp/codex_mcp.py")],
        cwd=str(ROOT),
        env={"FABLE_DEVKIT": str(ROOT / "unreal_fable"), "PYTHONUTF8": "1"},
    )
    if config_path:
        config = tomllib.loads(config_path.read_text(encoding="utf-8"))["mcp_servers"]["codex_mcp"]
        params = StdioServerParameters(command=config["command"], args=config.get("args", []),
                                       cwd=config.get("cwd"), env=config.get("env"))
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write, read_timeout_seconds=timedelta(seconds=660)) as client:
            init = await client.initialize()
            tools = (await client.list_tools()).tools

            async def call(name, args=None):
                result = await client.call_tool(name, args or {})
                texts = [item.text for item in result.content if item.type == "text"]
                if result.isError:
                    raise RuntimeError(f"{name}: {' '.join(texts)}")
                return json.loads(texts[0])

            status = await call("server_status", {"check_connections": check_connections})
            worlds = await call("list_worlds")
            if worlds_filter:
                available = {w["name"]: w for w in worlds}
                missing = [w for w in worlds_filter if w not in available]
                if missing:
                    raise RuntimeError(f"Unknown world(s): {', '.join(missing)}")
                selected_worlds = [available[w] for w in worlds_filter]
            else:
                selected_worlds = worlds
            for world in worlds:
                await call("validate_world", {"spec_json_or_name": world["name"]})
            await call("load_world", {"name_or_path": "car_track_oval"})
            current = await call("read_world")
            assert current["name"] == "car_track_oval"
            summary = {"server": init.serverInfo.name, "protocol": init.protocolVersion,
                       "tools": [tool.name for tool in tools], "worlds_validated": len(worlds),
                       "status": status}
            if run_worlds:
                summary["simulations"] = [
                    await call("run_test", {"name_or_path": world["name"], "backend": backend, "seeds": seeds})
                    for world in selected_worlds
                ]
            return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-connections", action="store_true")
    parser.add_argument("--run-worlds", action="store_true")
    parser.add_argument("--world", action="append", default=None,
                        help="World name to include; repeatable; omit to run all")
    parser.add_argument("--backend", choices=["mock", "unreal"], default="mock",
                        help="Run MCP simulation backend")
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--config", type=Path, help="Test the exact launch command from a Codex config")
    args = parser.parse_args()
    result = asyncio.run(smoke(
        args.check_connections,
        args.run_worlds,
        args.config,
        args.world,
        args.backend,
        args.seeds,
    ))
    output = json.dumps(result, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output + "\n", encoding="utf-8")
    print(output)
