# Codex MCP for Unreal + GPT-5.6 Sol

For the car racing objective (fast valid laps, track containment, and obstacle
avoidance), see [RACING.md](RACING.md) for the new controller and verified runs.

Adapted from your `unreal_fable.zip` and accompanying README. The working kit is
in `unreal_fable/`; the original ZIP and README are kept as `unreal_fable.source.zip`
and `README.source.md` for reference. Instructions in those original documents
describe the source kit; the Codex setup below replaces their Claude setup.

Codex runs **`gpt-5.6-sol`** and calls the local **`codex_mcp`** server over stdio.
The server entry point is `unreal_fable/mcp/codex_mcp.py`.
The server provides Unreal editing and simulation tools. No separate OpenAI API
key is needed for this integration; use your existing Codex sign-in.

## Use in this Codex app

1. Open this `unreal_gpt` folder as the project and reload Codex after installation.
2. Select **GPT-5.6 Sol** in the model picker for an existing task. The generated
   project configuration sets `gpt-5.6-sol` as the project default; an existing
   task's explicit model selection can override that default.
3. Try: **“Use codex_mcp to check server status, list the worlds, and run
   car_track_oval on the mock simulator.”**

Codex starts and stops the MCP process. You do not need to keep a terminal running.
`server_status` reports editor and live-simulator readiness separately. Mock
simulation and JSON world editing work with Unreal closed.

## Setup or reinstall on Windows

Use Python 3.11 or newer. From this folder in PowerShell:

```powershell
.\setup.ps1 -Python 'C:\path\to\python.exe' -InstallConfig
```

The script creates `.venv`, installs the kit and MCP dependencies, tests the
stdio connection, and writes **this project's** `.codex/config.toml`. Your global
Codex model and other servers are not changed. Omit `-InstallConfig` to generate
`codex-config.toml` for review without installing it. Rerun setup after moving
the folder because the server configuration uses absolute paths.

To install directly into an **already activated conda environment** (Python 3.11+),
run this instead of the default `.venv` setup:

```powershell
conda activate your_env_name
.\setup.ps1 -UseActiveEnvironment -InstallConfig
```

This installs packages into the active environment and configures Codex to launch
it using `conda run --no-capture-output`, so the app need not inherit your terminal's
activation. See [conda run](https://docs.conda.io/projects/conda/en/stable/commands/run.html).
The setup checks that the selected Python belongs to the activated conda environment
and tests the generated launch command before installing the configuration.

For an Unreal project with Python scripts in a different folder:

```powershell
.\setup.ps1 -EditorPython 'D:\UE\FableSim\Content\Python' -InstallConfig
```

Full Unreal connection steps and the additional UE 5.8 Remote Control settings
are in [CODEX_SETUP.md](unreal_fable/docs/CODEX_SETUP.md).

## Available tools

| Purpose | Tools |
|---|---|
| Inspect and validate | `server_status`, `list_worlds`, `read_world`, `validate_world` |
| Select/create a spec | `load_world`, `save_world` |
| Build/edit levels | `build_world`, `add_obstacle`, `add_obstacle_line`, `add_gate`, `set_route`, `remove_object`, `describe_scene` |
| Conditions and sketches | `set_environment`, `trace_track_from_sketch` |
| Evaluate | `run_test` (`mock` or `unreal`, 1–20 seeds) |
| Edit a running simulator | `live_spawn`, `live_despawn`, `live_set_env`, `live_scene` |

Coordinates use metres, radians, and ENU (x east, y north). `save_world` creates
validated JSON without Unreal and requires `overwrite=true` to replace a file.
Incremental edits save the spec first, then try the editor; read their `applied`
or `editor` result to see which succeeded. `set_environment` changes only the spec;
use `build_world` with `fresh_level=false` to update the editor, or `live_set_env`
to change the running simulator. Runtime edits disappear on the next normal reset.

`run_test` returns complete per-seed scorecards. A failed scenario is reported as
`success=false`; a broken runner raises an MCP tool error. Mock tests do not
validate Unreal rendering, C++ compilation, or real-vehicle performance.

## Verification

```powershell
.\.venv\Scripts\python.exe -m pytest tests unreal_fable/python/tests -q
.\.venv\Scripts\python.exe scripts/smoke_mcp.py --check-connections --run-worlds
```

The regression suite covers the original physics/controllers/calibration, actual
MCP initialization and calls, atomic validated saves, quoted Windows paths,
simulation subprocesses, and live UDP editing alongside a controller.
The original C++ FableBridge plugin still needs to be built in your Unreal project.

### MCP-driven test loop

Use this as your default dev cycle:

```powershell
# 1) health check
.\.venv\Scripts\python.exe scripts/smoke_mcp.py --check-connections

# 2) edit worlds with MCP (or via Codex) and keep them in spec
.\.venv\Scripts\python.exe scripts/smoke_mcp.py --world car_figure8_digital_twin --world boat_channel_gates_digital_twin --run-worlds --backend mock --seeds 5

# 3) run against Unreal once the level is built and Play is running
.\.venv\Scripts\python.exe scripts/smoke_mcp.py --world car_figure8_digital_twin --world boat_channel_gates_digital_twin --run-worlds --backend unreal --seeds 3
```

For online disturbances while driving, use `live_*` tools (`live_spawn`,
`live_set_env`, `live_despawn`) during a running `run_test`.

## References

- [GPT-5.6 Sol model](https://developers.openai.com/api/docs/models/gpt-5.6-sol)
- [Codex MCP configuration](https://learn.chatgpt.com/docs/extend/mcp?surface=cli)
- [Official MCP Python SDK](https://github.com/modelcontextprotocol/python-sdk)

This adapter uses the SDK's maintained 1.x FastMCP API with an explicit `<2`
dependency bound, matching the source kit. The tested dependency versions are
recorded in `requirements-tested.txt`.
