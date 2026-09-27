"""
fable_toolset.py - Claude's world-editing tools, exposed through UE 5.8's built-in
Unreal MCP server (plugins: Unreal MCP + Toolset Registry). Put next to
fable_build.py in <Project>/Content/Python/, then in the editor console:

    ModelContextProtocol.RefreshTools

Design rule: the SCENE SPEC is the source of truth, not the level. Every tool
edits the in-memory spec first, then mirrors the change into the level, then
writes the spec back to disk. So whatever Claude does in the editor, the Python
side (mock sim, evaluator, calibration) sees the identical world by reloading
the JSON. If the two ever disagree, run build_world again.

Set FABLE_SPECS_DIR (or edit SPECS_DIR below) to your dev-kit specs folder.
"""

from __future__ import annotations

import json
import math
import os
import random
import sys

import unreal

import fable_build as fb

try:
    import toolset_registry
except ImportError:
    unreal.log_warning("[fable] toolset_registry not importable - enable 'Unreal MCP' + 'Toolset Registry' (UE 5.8+), "
                       "or use mcp/codex_mcp.py from the dev kit instead.")
    raise

SPECS_DIR = os.environ.get("FABLE_SPECS_DIR", r"G:/My Drive/applications/unreal_fable/specs")
TOOLS_DIR = os.path.join(os.path.dirname(SPECS_DIR), "tools")

_spec: dict = {}
_spec_path: str = ""


def _water_level():
    return float(_spec.get("world", {}).get("water", {}).get("level_m", 0.0)) if _spec else 0.0


def _save():
    if _spec and _spec_path:
        with open(_spec_path, "w") as f:
            json.dump(_spec, f, indent=2)


def _require():
    if not _spec:
        raise RuntimeError("no world loaded - call build_world or load_spec first")


@unreal.uclass()
class FableWorldToolset(unreal.ToolsetDefinition):
    """Build and edit car-track and boat test worlds from scene specs."""

    @toolset_registry.tool_call
    def list_worlds(self) -> str:
        """Lists the scene specs available in the dev kit's specs/worlds folder.

        Returns:
            JSON list of {name, scenario, notes, path}.
        """
        out = []
        d = os.path.join(SPECS_DIR, "worlds")
        for fn in sorted(os.listdir(d)) if os.path.isdir(d) else []:
            if fn.endswith(".json"):
                with open(os.path.join(d, fn)) as f:
                    s = json.load(f)
                out.append({"name": s.get("name"), "scenario": s.get("scenario"), "notes": s.get("notes", "")[:160],
                            "path": os.path.join(d, fn)})
        return json.dumps(out)

    @toolset_registry.tool_call
    def build_world(self, spec_path_or_json: str, fresh_level: bool = True) -> str:
        """Builds a complete test world from a scene spec (v2) into Unreal.

        Prefer editing the spec (a JSON document: terrain/water, track, gates, dock,
        objects, obstacle_fields, task, randomization - all metres/radians, ENU) and
        rebuilding over placing actors one by one: one spec = one reproducible
        test case shared with the Python side.

        Args:
            spec_path_or_json: Absolute path to a scene spec JSON, a name from
                list_worlds (e.g. "car_track_oval"), or the spec itself as a JSON string.
            fresh_level: Create a new empty level named after the spec before building.

        Returns:
            JSON summary of what was built and where the spec was saved.
        """
        global _spec, _spec_path
        s = spec_path_or_json.strip()
        if s.startswith("{"):
            _spec = json.loads(s)
            _spec_path = os.path.join(SPECS_DIR, "worlds", f"{_spec['name']}.json")
            _save()
        else:
            p = s if os.path.isfile(s) else os.path.join(SPECS_DIR, "worlds", s if s.endswith(".json") else s + ".json")
            with open(p) as f:
                _spec = json.load(f)
            _spec_path = p
        if fresh_level:
            fb._level().new_level(f"/Game/Fable/Maps/{_spec['name']}")
            summary = fb.build_from_spec(_spec, os.path.dirname(_spec_path), clear=False)
        else:
            summary = fb.build_from_spec(_spec, os.path.dirname(_spec_path), clear=True)
        summary["spec_path"] = _spec_path
        return json.dumps(summary)

    @toolset_registry.tool_call
    def load_spec(self, spec_path: str) -> str:
        """Loads a spec as the current world WITHOUT rebuilding (the level is already built).

        Args:
            spec_path: Absolute path or a name from list_worlds.

        Returns:
            JSON with name, scenario and object counts.
        """
        global _spec, _spec_path
        p = spec_path if os.path.isfile(spec_path) else os.path.join(SPECS_DIR, "worlds", spec_path if spec_path.endswith(".json") else spec_path + ".json")
        with open(p) as f:
            _spec = json.load(f)
        _spec_path = p
        return json.dumps({"name": _spec["name"], "scenario": _spec["scenario"], "objects": len(_spec.get("objects", [])),
                           "gates": len(_spec.get("gates", [])), "track_points": len((_spec.get("track") or {}).get("centerline_m", []))})

    @toolset_registry.tool_call
    def add_obstacle(self, kind: str, x_m: float, y_m: float, yaw_rad: float = 0.0, label: str = "",
                     scale_x: float = 1.0, scale_y: float = 1.0, scale_z: float = 1.0, tag: str = "obstacle") -> str:
        """Adds one obstacle to the current world (level + spec).

        Args:
            kind: cone, barrel, wall, box, ramp, pole, rock, pedestrian_static,
                buoy_red, buoy_green, buoy_yellow, moored_boat, dock_piece.
            x_m: East position, metres (ENU world frame).
            y_m: North position, metres.
            yaw_rad: Heading, radians, counter-clockwise from east.
            label: Optional unique name (used by remove/move and reported in collisions).
            scale_x: Size multiplier along the object's length (walls: 1.0 = 1 m).
            scale_y: Size multiplier across.
            scale_z: Height multiplier.
            tag: Gameplay tag: obstacle (default), goal_marker, decoration.

        Returns:
            JSON with the label used.
        """
        _require()
        lab = fb.add_object(kind, x_m, y_m, yaw_rad, label or None, [scale_x, scale_y, scale_z], tag, _water_level())
        _spec.setdefault("objects", []).append({"label": lab, "kind": kind, "xy_m": [round(x_m, 3), round(y_m, 3)],
                                                "yaw_rad": round(yaw_rad, 4), "scale": [scale_x, scale_y, scale_z], "tag": tag})
        _save()
        return json.dumps({"ok": True, "label": lab})

    @toolset_registry.tool_call
    def add_obstacle_line(self, kind: str, x0_m: float, y0_m: float, x1_m: float, y1_m: float,
                          spacing_m: float = 1.5, label_prefix: str = "line") -> str:
        """Adds a straight line of obstacles (a row of cones, a wall of barrels, a chicane edge).

        Args:
            kind: Obstacle kind, see add_obstacle.
            x0_m: Start east, m.
            y0_m: Start north, m.
            x1_m: End east, m.
            y1_m: End north, m.
            spacing_m: Distance between objects. For kind=wall the segments are joined.
            label_prefix: Labels become prefix_0, prefix_1, ...

        Returns:
            JSON with the number added and the labels.
        """
        _require()
        dx, dy = x1_m - x0_m, y1_m - y0_m
        L = math.hypot(dx, dy)
        n = max(1, int(L / max(spacing_m, 0.1)))
        yaw = math.atan2(dy, dx)
        labels = []
        for i in range(n + 1):
            t = i / n if n else 0
            x, y = x0_m + dx * t, y0_m + dy * t
            if kind == "wall":
                if i == n:
                    break
                x, y = x0_m + dx * (i + 0.5) / n, y0_m + dy * (i + 0.5) / n
                sc = [L / n, 0.3, 1.0]
            else:
                sc = [1, 1, 1]
            lab = f"{label_prefix}_{i}"
            fb.add_object(kind, x, y, yaw, lab, sc, "obstacle", _water_level())
            _spec.setdefault("objects", []).append({"label": lab, "kind": kind, "xy_m": [round(x, 3), round(y, 3)],
                                                    "yaw_rad": round(yaw, 4), "scale": sc, "tag": "obstacle"})
            labels.append(lab)
        _save()
        return json.dumps({"ok": True, "added": len(labels), "labels": labels})

    @toolset_registry.tool_call
    def add_gate(self, left_x_m: float, left_y_m: float, right_x_m: float, right_y_m: float, label: str = "") -> str:
        """Adds a red/green buoy gate to a boat world. Gates are passed in the order added.

        Args:
            left_x_m: Port (red) buoy east position, m, when heading through the gate.
            left_y_m: Port buoy north, m.
            right_x_m: Starboard (green) buoy east, m.
            right_y_m: Starboard buoy north, m.
            label: Optional gate name (default gateN).

        Returns:
            JSON with the label and the gate's centre/heading.
        """
        _require()
        gates = _spec.setdefault("gates", [])
        lab = label or f"gate{len(gates) + 1}"
        g = {"label": lab, "left_m": [round(left_x_m, 3), round(left_y_m, 3)], "right_m": [round(right_x_m, 3), round(right_y_m, 3)]}
        gates.append(g)
        lvl = _water_level()
        fb.spawn_mesh("sphere", left_x_m, left_y_m, lvl + 0.1, 0, fb.KINDS["buoy_red"][1], f"{lab}_L", "gate", "buoy_red", "obstacle", color=fb.COLORS["buoy_red"])
        fb.spawn_mesh("sphere", right_x_m, right_y_m, lvl + 0.1, 0, fb.KINDS["buoy_green"][1], f"{lab}_R", "gate", "buoy_green", "obstacle", color=fb.COLORS["buoy_green"])
        cx, cy = (left_x_m + right_x_m) / 2, (left_y_m + right_y_m) / 2
        heading = math.atan2(-(right_x_m - left_x_m), (right_y_m - left_y_m))
        if _spec.get("task", {}).get("type") == "gates":
            _spec["task"].setdefault("waypoints_m", []).append([round(cx, 3), round(cy, 3)])
            tr = _spec.setdefault("track", {"centerline_m": [], "closed": False, "border": "none"})
            tr.setdefault("centerline_m", []).append([round(cx, 3), round(cy, 3)])
        _save()
        return json.dumps({"ok": True, "label": lab, "centre_m": [cx, cy], "heading_rad": heading})

    @toolset_registry.tool_call
    def set_route(self, points_json: str, width_m: float = 2.0, border: str = "cones", closed: bool = False,
                  spacing_m: float = 1.5) -> str:
        """Replaces the drivable route (car track / boat route) and rebuilds its borders.

        Args:
            points_json: JSON array of [x, y] metres, e.g. "[[0,0],[10,0],[10,8]]".
                For a car this becomes the coned/walled track and the waypoints;
                for a boat it is the scored route.
            width_m: Track width.
            border: none, cones, walls, curbs, paint.
            closed: True for a loop (lap tasks).
            spacing_m: Border object spacing.

        Returns:
            JSON with route length and border count.
        """
        _require()
        pts = json.loads(points_json)
        fb.remove_by_label("track_border")
        for a in fb._actors().get_all_level_actors():
            if a.tags and unreal.Name("track_border") in a.tags:
                fb._actors().destroy_actor(a)
        _spec["track"] = {"centerline_m": pts, "closed": closed, "width_m": width_m, "border": border, "border_spacing_m": spacing_m}
        if _spec.get("task", {}).get("type") in ("waypoints", "lap"):
            _spec["task"]["waypoints_m"] = pts
        n = len(fb.build_track(_spec))
        length = sum(math.dist(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts) if closed else len(pts) - 1))
        _save()
        return json.dumps({"ok": True, "length_m": round(length, 2), "border_objects": n})

    @toolset_registry.tool_call
    def remove_object(self, label: str) -> str:
        """Removes an object (or gate, or 'field:*' for all scattered obstacles) from level and spec.

        Args:
            label: The label from add_obstacle / describe_scene, or 'field:*'.

        Returns:
            JSON with the number of actors removed.
        """
        _require()
        if label == "field:*":
            n = 0
            for a in fb._actors().get_all_level_actors():
                if a.tags and unreal.Name("field") in a.tags:
                    fb._actors().destroy_actor(a); n += 1
            _spec["obstacle_fields"] = []
        else:
            n = fb.remove_by_label(label) + fb.remove_by_label(label + "_L") + fb.remove_by_label(label + "_R")
            _spec["objects"] = [o for o in _spec.get("objects", []) if o.get("label") != label]
            _spec["gates"] = [g for g in _spec.get("gates", []) if g.get("label") != label]
        _save()
        return json.dumps({"ok": True, "removed": n})

    @toolset_registry.tool_call
    def move_object(self, label: str, x_m: float, y_m: float, yaw_rad: float = -999.0) -> str:
        """Moves an object to a new position (and heading if given).

        Args:
            label: Object label.
            x_m: New east position, m.
            y_m: New north position, m.
            yaw_rad: New heading, radians; leave at -999 to keep the current heading.

        Returns:
            JSON ok/notfound.
        """
        _require()
        yaw = None if yaw_rad == -999.0 else yaw_rad
        ok = fb.move_by_label(label, x_m, y_m, yaw)
        for o in _spec.get("objects", []):
            if o.get("label") == label:
                o["xy_m"] = [round(x_m, 3), round(y_m, 3)]
                if yaw is not None:
                    o["yaw_rad"] = round(yaw, 4)
        _save()
        return json.dumps({"ok": ok})

    @toolset_registry.tool_call
    def set_environment(self, wind_speed_mps: float = -1, wind_dir_rad: float = -999, current_speed_mps: float = -1,
                        current_dir_rad: float = -999, wave_amp_m: float = -1, friction: float = -1,
                        lighting_preset: str = "") -> str:
        """Changes wind, current, waves, surface friction or lighting in the spec (and the bridge actor).

        Args:
            wind_speed_mps: Mean wind speed; -1 keeps current.
            wind_dir_rad: Direction the wind blows TOWARD, radians CCW from east; -999 keeps.
            current_speed_mps: Water current speed (boat); -1 keeps.
            current_dir_rad: Current direction; -999 keeps.
            wave_amp_m: Wave amplitude (boat); -1 keeps.
            friction: Ground friction coefficient (car); -1 keeps.
            lighting_preset: noon, overcast, dusk, night, indoor; empty keeps.

        Returns:
            JSON of the resulting disturbance block.
        """
        _require()
        w = _spec["world"]
        d = w.setdefault("disturbance", {})
        if wind_speed_mps >= 0: d.setdefault("wind", {})["speed_mps"] = wind_speed_mps
        if wind_dir_rad != -999: d.setdefault("wind", {})["dir_rad"] = wind_dir_rad
        if current_speed_mps >= 0: d.setdefault("current", {})["speed_mps"] = current_speed_mps
        if current_dir_rad != -999: d.setdefault("current", {})["dir_rad"] = current_dir_rad
        if wave_amp_m >= 0: w.setdefault("water", {}).setdefault("waves", {})["amp_m"] = wave_amp_m
        if friction >= 0: w.setdefault("terrain", {}).setdefault("surface", {})["friction"] = friction
        if lighting_preset: w.setdefault("lighting", {})["preset"] = lighting_preset
        _save()
        # mirror onto the bridge actor in the level
        for a in fb._actors().get_all_level_actors():
            if a.tags and unreal.Name("bridge") in a.tags:
                try:
                    env = a.get_editor_property("env")
                    if wind_speed_mps >= 0: env.set_editor_property("wind_speed", wind_speed_mps)
                    if wind_dir_rad != -999: env.set_editor_property("wind_dir", wind_dir_rad)
                    if current_speed_mps >= 0: env.set_editor_property("current_speed", current_speed_mps)
                    if current_dir_rad != -999: env.set_editor_property("current_dir", current_dir_rad)
                    if wave_amp_m >= 0: env.set_editor_property("wave_amp", wave_amp_m)
                    a.set_editor_property("env", env)
                except Exception as e:                  # noqa: BLE001
                    unreal.log_warning(f"[fable] env mirror: {e}")
        return json.dumps({"disturbance": d, "waves": w.get("water", {}).get("waves"), "surface": w.get("terrain", {}).get("surface"), "lighting": w.get("lighting")})

    @toolset_registry.tool_call
    def randomize_obstacles(self, seed: int) -> str:
        """Re-scatters every obstacle_field with a new seed (level + spec.seed).

        Args:
            seed: Random seed; the Python side reproduces the same layout with Sim.reset(seed).

        Returns:
            JSON with the number of obstacles placed.
        """
        _require()
        for a in fb._actors().get_all_level_actors():
            if a.tags and unreal.Name("field") in a.tags:
                fb._actors().destroy_actor(a)
        _spec["seed"] = int(seed)
        n = len(fb.build_fields(_spec, random.Random(int(seed))))
        _save()
        return json.dumps({"ok": True, "placed": n, "seed": seed})

    @toolset_registry.tool_call
    def describe_scene(self) -> str:
        """Lists what is in the current world: counts by tag and every generated actor with position.

        Returns:
            JSON {spec_name, task, tag_counts, actors:[{label,x,y,yaw,tags}]}.
        """
        d = fb.describe()
        d["spec_name"] = _spec.get("name") if _spec else None
        d["task"] = _spec.get("task") if _spec else None
        d["spec_path"] = _spec_path
        return json.dumps(d)

    @toolset_registry.tool_call
    def trace_track_from_sketch(self, image_path: str, world_m: float, width_m: float = 2.0) -> str:
        """Traces a drawn line (photo of a marker loop, or any dark stroke on light background)
        into route points in metres. Feed the result to set_route.

        Args:
            image_path: Absolute path to the image.
            world_m: What the full image width represents, in metres.
            width_m: Desired track width.

        Returns:
            JSON {centerline_m, closed, width_m, length_m}.
        """
        if TOOLS_DIR not in sys.path:
            sys.path.append(TOOLS_DIR)
        from dataclasses import asdict
        import sketch_to_heightmap as s2h
        return json.dumps(asdict(s2h.trace_track(image_path, world_m, width_m)))

    @toolset_registry.tool_call
    def generate_terrain(self, world_m: float, roughness_m: float = 0.05, feature_size_m: float = 8.0,
                         slope_max_deg: float = 8.0, seed: int = 0, sketch_image_path: str = "") -> str:
        """Generates a terrain mesh for a car world and points the current spec at it
        (world.terrain.mode = procedural/heightmap). Rebuild afterwards.

        Args:
            world_m: Terrain side length, metres.
            roughness_m: RMS height deviation; 0.02 smooth, 0.1 rough gravel, 0.3 off-road.
            feature_size_m: Bump wavelength.
            slope_max_deg: Slopes above this are flattened.
            seed: Noise seed.
            sketch_image_path: Optional drawing/photo used as elevation (bright = high).

        Returns:
            JSON with heightmap and mesh paths.
        """
        _require()
        if TOOLS_DIR not in sys.path:
            sys.path.append(TOOLS_DIR)
        import sketch_to_heightmap as s2h
        out_dir = os.path.join(os.path.dirname(_spec_path), "terrain")
        os.makedirs(out_dir, exist_ok=True)
        if sketch_image_path:
            hm, z = s2h.heightmap_from_image(sketch_image_path, 1009, world_m, roughness_m, 4.0, False, slope_max_deg)
        else:
            hm, z = s2h.procedural_heightmap(1009, world_m, roughness_m, feature_size_m, seed, 4, slope_max_deg, 3.0)
        png = os.path.join(out_dir, f"{_spec['name']}_terrain.png")
        obj = os.path.join(out_dir, f"{_spec['name']}_terrain.obj")
        s2h.save_heightmap(hm, png)
        s2h.heightmap_to_obj(hm, world_m, z, obj)
        t = _spec["world"].setdefault("terrain", {})
        t.update({"mode": "heightmap" if sketch_image_path else "procedural", "heightmap_path": png,
                  "terrain_obj_path": obj, "z_scale_m": round(z, 4)})
        _save()
        return json.dumps({"heightmap_path": png, "terrain_obj_path": obj, "z_scale_m": round(z, 4)})

    @toolset_registry.tool_call
    def clear_world(self) -> str:
        """Deletes every actor the builder created (hand-placed actors are kept).

        Returns:
            JSON with the count removed.
        """
        return json.dumps({"removed": fb.clear_generated()})
