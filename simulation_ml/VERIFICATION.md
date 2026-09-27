# Verification record

- Conda setup: completed in ungpt (Python 3.11) using Windows PowerShell 5.1. Fixed inline-Python quoting and empty environment detection. An independent MCP launch without conda activation passed, with all 20 tools and all four mock scenarios; explicit TMP/TEMP settings allow Windows conda activation under MCP.

- After renaming to codex_mcp: 8 adapter tests and all four MCP mock simulations passed.

- Original Python suite: 19 passed.
- Codex MCP regression suite: 8 passed.
- Actual stdio MCP initialization, tool listing, schema validation and calls: passed.
- Four bundled worlds: validated and successful through MCP on the mock backend (seed 0).
- Live spawn, despawn, environment and scene tools: passed against the Python UDP reference server alongside an active controller.
- Windows simulation subprocess stdin regression: passed.
- Dependency consistency: pip check passed.
- PowerShell setup script: syntax checked.
- Project registration: codex mcp get codex_mcp --json confirms enabled stdio server, executable, paths and timeouts.
- Project model default: gpt-5.6-sol. An existing task may still need its model picker changed.

Unreal Remote Control and the live bridge were offline during verification. The supplied Unreal C++ plugin was not compiled or run. Live engine behavior remains unverified.
