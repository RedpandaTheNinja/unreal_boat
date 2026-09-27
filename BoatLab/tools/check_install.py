"""BoatLab installation / connection check. Never commands the motors.

    python tools/check_install.py            # files, Python packages, configs
    python tools/check_install.py --unreal   # also query the BoatPhysics UDP port (Play must be running)
"""
from __future__ import annotations

import argparse
import importlib
import json
import socket
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT.parent


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--unreal", action="store_true")
    a = ap.parse_args()
    report = {"python": sys.version.split()[0], "executable": sys.executable, "ok": True, "problems": []}

    def bad(msg):
        report["ok"] = False
        report["problems"].append(msg)

    if sys.version_info < (3, 10):
        bad("Python 3.10+ required (3.11/3.12 recommended)")
    for mod in ("numpy", "cv2", "matplotlib", "serial", "pytest"):
        try:
            m = importlib.import_module(mod)
            report[mod] = getattr(m, "__version__", "ok")
        except ImportError:
            (bad if mod in ("numpy", "cv2") else report["problems"].append)(f"missing package: {mod}")
    for rel in ("settings.json", "config/boat.json", "config/course_aimm_2025.json", "config/vision_colors.json",
                "config/missions/aimm_full.json", "dashboard/server.py", "unreal/Content/Python/boatlab_course.py"):
        if not (ROOT / rel).exists():
            bad(f"missing file: BoatLab/{rel}")
    settings = json.loads((ROOT / "settings.json").read_text())
    eng = Path(settings.get("engine_root", ""))
    report["engine_root"] = str(eng)
    if not (eng / "Engine/Binaries/Win64/UnrealEditor.exe").exists():
        report["problems"].append("UnrealEditor.exe not found under engine_root (fine on a Jetson / Linux box)")
    plug = PROJECT / "Plugins/BoatPhysics"
    if plug.exists():
        up = json.loads((plug / "BoatPhysics.uplugin").read_text(encoding="utf-8-sig"))
        report["installed_plugin_version"] = up.get("VersionName")
        src = (plug / "Source/BoatPhysics/Private/BoatCameraSensor.cpp").exists()
        report["plugin_has_camera_source"] = src
        if not src:
            report["problems"].append("BoatPhysics plugin is v1 (no camera). Run 04_Build_Plugin.cmd for vision in Unreal.")
    if a.unreal:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(1.0)
        try:
            s.sendto(b"{}", ("127.0.0.1", settings.get("ue_udp_port", 7450)))
            r = json.loads(s.recv(65535))
            report["unreal"] = {"schema": r.get("schema"), "time_s": r.get("time_s"), "position_ue_m": r.get("position_ue_m"),
                                "plugin_version": r.get("plugin_version", "boatlab_v1 (no camera)"),
                                "camera_available": r.get("camera_available", False)}
        except OSError as e:
            bad(f"no reply from Unreal UDP {settings.get('ue_udp_port', 7450)}: open the lake level and press Play ({e})")
        finally:
            s.close()
    print(json.dumps(report, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
