# Editing worlds with Claude

Two ways to give Claude the tools, pick one (both work at once):

| | Built-in (UE 5.8) | Standalone `mcp/codex_mcp.py` |
|---|---|---|
| Runs in | the editor process | your Python |
| Setup | `fable_toolset.py` as a Python startup script; `ModelContextProtocol.GenerateClientConfig ClaudeCode` | `claude mcp add codex_mcp -- python mcp/codex_mcp.py` (+ Remote Control plugin for editor tools) |
| Edits the editor level | yes | yes (via Remote Control) |
| Edits the **running** sim | no | **yes** (`live_*` tools over UDP, role=editor) |
| Runs a test and reports the score | no | yes (`run_test`) |

Either way, the **scene spec is the source of truth**: every tool edits the JSON in
`specs/worlds/` first and then mirrors the change into Unreal. Your Python tests reload
the JSON, so what Claude built is exactly what the evaluator scores.

## Prompts that work

Start Claude Code from the Unreal project folder (built-in) or anywhere (standalone), then:

**Build**
> Build the boat_buoy_course world.
> Build a car world: 30 × 20 m asphalt, a figure-8 track 2 m wide with cones, spawn at the crossing. Name it car_figure8.

**Obstacles**
> Add a row of barrels from (5, −3) to (5, 3) spaced 1.2 m.
> Put a 4 m wall across the back straight at x = −12, leaving a 2.5 m gap on the inside.
> Remove everything from the obstacle field and scatter 10 rocks between x 0..20, y −8..8, keeping 3 m off the track.

**Routes and gates**
> Trace the track in G:/…/sketch.jpg as 25 m wide and make it the route with curbs.
> Add three more gates continuing the course north-east, 10 m apart, 4 m wide.
> Move the dock to (15, −5) facing north.

**Conditions**
> Set the wind to 5 m/s from the south-west with 1.5 m/s gusts and 0.15 m waves.
> Make the surface wet: friction 0.55, overcast light.

**While a test is running** (standalone only)
> The car is on lap 2 — drop a cone on the racing line 8 m ahead of it.
> Kill the wind for 20 seconds then bring it back at 6 m/s.

**Verify**
> Describe the scene. Then run the baseline controller on it for 5 seeds and tell me the cross-track RMS.

## Coordinates Claude uses

Metres, radians, **ENU**: x east, y north, yaw counter-clockwise from east, 0 = facing +x.
Same as your Python. Claude never sees Unreal centimetres.

## Tool list (both servers)

`list_worlds` · `load_world` / `load_spec` · `read_world` · `build_world` ·
`add_obstacle` · `add_obstacle_line` · `add_gate` · `set_route` · `remove_object` ·
`move_object` · `set_environment` · `randomize_obstacles` · `describe_scene` ·
`trace_track_from_sketch` · `generate_terrain` · `clear_world`
Standalone adds: `run_test` · `live_spawn` · `live_despawn` · `live_set_env` · `live_scene`

## What Claude should NOT do (put this in your CLAUDE.md)

- Do not edit `vehicle_params.json` to make a test pass. Those are measurements.
- Do not change `task.tolerance_m` or `max_time_s` to make a test pass. Change the world or the controller.
- Every world edit must go through the tools (so the spec stays in sync), never by
  placing actors in the editor by hand and forgetting.
- After editing a world, run `run_test` on the mock before declaring it done.
