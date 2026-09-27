# Connect Codex MCP to Unreal with GPT-5.6 Sol

The `codex_mcp` server runs outside Unreal and works with the Codex app.
The model is selected in Codex, not inside the MCP server. This integration
does not require the original Claude client or Unreal's built-in MCP toolset.

## Local server

Run `setup.ps1 -InstallConfig` from the outer `unreal_gpt` directory. It creates
a project-scoped `.codex/config.toml` with the Python executable, server path,
environment, `model = "gpt-5.6-sol"`, a 30-second startup timeout, and a 660-second
tool timeout. The scenario runner itself has a 600-second timeout.

Reload the Codex app/project. Select GPT-5.6 Sol in an existing task's model
picker. Run `server_status` and `list_worlds`. Four bundled worlds should appear.

## Unreal editor connection

1. Create/open your Unreal C++ project and follow the plugin, vehicle Blueprint,
   and physics setup in `UNREAL_SETUP.md` sections 1–3. The original C++ source
   has not been compiled as part of this Codex adapter work.
2. Copy `unreal/Content/Python/fable_build.py` into the Unreal project's
   `Content/Python` directory. Copy the rest of the supplied Python helpers if
   you also use the original in-editor toolset.
3. Enable **Python Editor Script Plugin** and **Remote Control API**. Restart
   the editor when requested. The standalone adapter does not need the built-in
   Unreal MCP plugin or the Claude client configuration command.
4. In **Project Settings → Remote Control → Security**, enable **Restrict Server
   Access**, use a localhost-only allowlist, and enable **Remote Python Execution**.
   Add this entry under **Custom Allowed Remote Function Calls**:

   - Class path: `/Script/PythonScriptPlugin.PythonScriptLibrary`
   - Function name: `ExecutePythonCommandEx`
   - Allow child classes: false

   These settings are required by the installed UE 5.8 source. The default
   allowlist does not include the Python function. There is no need to enable
   “Allow Any Remote Function Call.” Keep this local service bound to localhost.
5. In the Unreal console, run `WebControl.StartServer`. The adapter connects to
   `http://127.0.0.1:30010` by default.
6. Run `server_status` in Codex. `editor.ready=true` means the server can execute
   Python and import `fable_build` inside Unreal. Then try **“Build the
   car_track_oval world with Fable.”** Creating a fresh level can replace the
   currently open level, so save your editor work first.

If Python scripts live elsewhere, pass `-EditorPython` to `setup.ps1` or set
`UE_PROJECT_PYTHON` under `[mcp_servers.codex_mcp.env]`. The default uses the editor's
own `Content/Python` directory.

The editor's Python is separate from `.venv`: installing the server dependencies
does not install them into Unreal. The basic world builder uses the standard
library and Unreal's Python module. Additional sketch/terrain tools may need the
dependencies and Python paths described in the original kit.

## Running simulator connection

After the FableBridge plugin and vehicle Blueprints are built and the world is
loaded, press **Play**. The bridge listens on UDP port 9800. Now `live.ready`
should be true. `live_*` tools connect with `role=editor` so they do not take
over the controller's observation stream.

`run_test(backend="unreal")` drives and resets the simulator with the baseline
controller. Use it when that is the intended test; `backend="mock"` is the default.
The `FABLE_BRIDGE_PORT` setting is used by both live editing and the Unreal runner.

You can test the UDP connection without an Unreal project in a separate terminal:

```powershell
.\.venv\Scripts\python.exe -m fable.fake_unreal car_track_oval
```

That is a Python protocol simulator, not the Unreal engine. Stop it before
starting Unreal on the same port.

## Troubleshooting

| Result | Action |
|---|---|
| `codex_mcp` missing in Codex | Open the outer `unreal_gpt` folder, reload the app, and check project trust and `.codex/config.toml`. |
| Wrong model | Select GPT-5.6 Sol in the task model picker; the MCP server cannot change the active model. |
| Connection refused on 30010 | Open the editor, enable Remote Control API, and run `WebControl.StartServer`. |
| HTTP error / Python object unavailable | Check Remote Python Execution and the custom allowed function entry above. |
| `No module named fable_build` | Copy the builder to `Content/Python`, or set `UE_PROJECT_PYTHON` and reload Codex. |
| Live connection refused / UDP 10054 / timeout | Press Play with a FableBridge actor, or start the fake server for protocol testing. Check the configured port. |
| Edit returns `spec only` | The JSON was saved; reconnect the editor and rebuild the world to synchronize it. |

Use `scripts/smoke_mcp.py --check-connections --run-worlds` for a reproducible
connection and four-world mock check. Connection probes never start a level,
reset a simulator, or apply driving commands.
