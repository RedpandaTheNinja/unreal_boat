"""Load JSON configuration (boat, course, vision, missions). Standard library only."""
from __future__ import annotations

import copy
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config"


def load_json(path) -> dict:
    p = Path(path)
    if not p.is_absolute() and not p.exists():
        p = CONFIG / p
    return json.loads(p.read_text(encoding="utf-8"))


def load_boat(path="boat.json") -> dict:
    return load_json(path)


def load_course(path="course_aimm_2025.json") -> dict:
    return load_json(path)


def load_vision(path="vision_colors.json") -> dict:
    return load_json(path)


def load_mission(name_or_path) -> dict:
    p = Path(name_or_path)
    if not p.suffix:
        p = CONFIG / "missions" / (p.name + ".json")
    elif not p.is_absolute() and not p.exists():
        p = CONFIG / "missions" / p
    return json.loads(p.read_text(encoding="utf-8"))


def load_settings() -> dict:
    p = ROOT / "settings.json"
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else {}


def deep_update(base: dict, override: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_update(out[k], v)
        else:
            out[k] = v
    return out


def objects_by_id(course: dict) -> dict:
    return {o["id"]: o for o in course["objects"]}
