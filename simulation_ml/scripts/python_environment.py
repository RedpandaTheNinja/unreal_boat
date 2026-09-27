"""Report the selected interpreter without PowerShell native-argument quoting."""
import json
import sys

if __name__ == "__main__":
    if sys.version_info < (3, 11):
        raise SystemExit("Python 3.11+ is required for Codex MCP setup.")
    print(json.dumps({"executable": sys.executable, "prefix": sys.prefix}))
