# CLAUDE.md — conventions for Claude working in this dev kit

- Units everywhere in Python, specs, docs and on the wire: metres, radians, seconds, ENU (x east, y north,
  yaw CCW from east), body frame x forward / y left. Unreal centimetres/degrees exist only inside
  `unreal/Plugins/FableBridge/.../FableConv.h` and `unreal/Content/Python/fable_build.py:to_ue()`.
- The scene spec (`specs/worlds/*.json`, schema v2) is the source of truth for a world. Edit it through the
  MCP tools or `specs/make_worlds.py`, never by hand-placing actors that the JSON does not know about.
- `specs/vehicles/*.json` are measurements. Never change them to make a test pass. Never change
  `task.tolerance_m` / `max_time_s` to make a test pass either — change the world or the controller.
- After editing a world, run `python python/examples/run_scenario.py <world> --seeds 3` (mock) before saying it is done.
- `python -m pytest python/tests -q` must stay green. Add a test when adding a controller or a spec field.
- Protocol changes: update `docs/PROTOCOL.md`, `python/fable/bridge.py`, `python/fable/fake_unreal.py` and
  `FableBridgeActor.cpp` together, and bump `PROTOCOL_VERSION` if units/frames/meanings change.
