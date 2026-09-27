"""
fable_build.py - builds a scene spec (specs/schema/scene_spec.schema.json, v2) into
the open Unreal level. Runs inside the editor's Python.

    import fable_build as fb
    fb.build_from_spec_file(r"G:/My Drive/applications/unreal_fable/specs/worlds/car_track_oval.json")

Everything it creates carries the actor tag `fable`, plus a role tag
(terrain, ground, water, track_border, gate, obstacle, dock, shore, spawn, bridge...),
so a rebuild removes only its own actors and the FableBridge plugin can report
the scene by tag. The vehicle pawn and the bridge actor are placed too, with the
spawn pose and environment copied from the spec, so a built level is playable
against Python immediately.

Coordinates: the spec is ENU metres / radians (ROS). Unreal is centimetres,
degrees, Y flipped. `to_ue()` / `yaw_ue()` are the only conversions.
"""

from __future__ import annotations

import json
import math
import os
import random

import unreal

TAG = unreal.Name("fable")
GEN = "/Game/Fable/Generated"
M2U = 100.0

BASIC = {
    "cube": "/Engine/BasicShapes/Cube.Cube",
    "cylinder": "/Engine/BasicShapes/Cylinder.Cylinder",
    "cone": "/Engine/BasicShapes/Cone.Cone",
    "sphere": "/Engine/BasicShapes/Sphere.Sphere",
    "plane": "/Engine/BasicShapes/Plane.Plane",
}

# kind -> (shape, size metres [x, y, z], floats on water?)
KINDS = {
    "cone":              ("cone",     [0.36, 0.36, 0.55], False),
    "barrel":            ("cylinder", [0.58, 0.58, 0.90], False),
    "wall":              ("cube",     [1.00, 1.00, 1.00], False),
    "box":               ("cube",     [0.50, 0.50, 0.50], False),
    "ramp":              ("cube",     [2.00, 1.50, 0.30], False),
    "pole":              ("cylinder", [0.12, 0.12, 2.00], False),
    "rock":              ("sphere",   [0.90, 0.90, 0.70], False),
    "pedestrian_static": ("cylinder", [0.50, 0.50, 1.75], False),
    "buoy_red":          ("sphere",   [0.60, 0.60, 0.60], True),
    "buoy_green":        ("sphere",   [0.60, 0.60, 0.60], True),
    "buoy_yellow":       ("sphere",   [0.60, 0.60, 0.60], True),
    "moored_boat":       ("cube",     [3.00, 1.20, 0.80], True),
    "dock_piece":        ("cube",     [1.00, 1.00, 0.40], False),
}
COLORS = {"buoy_red": (0.9, 0.05, 0.05), "buoy_green": (0.05, 0.8, 0.1), "buoy_yellow": (0.95, 0.85, 0.05),
          "cone": (1.0, 0.45, 0.0), "barrel": (0.2, 0.3, 0.8), "moored_boat": (0.9, 0.9, 0.9), "pier": (0.45, 0.3, 0.15)}


# ------------------------------------------------------------------ helpers

def _actors():
    return unreal.get_editor_subsystem(unreal.EditorActorSubsystem)


def _level():
    return unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)


def _log(m):
    unreal.log(f"[fable] {m}")


def to_ue(x_m, y_m, z_m=0.0):
    return unreal.Vector(float(x_m) * M2U, -float(y_m) * M2U, float(z_m) * M2U)


def yaw_ue(yaw_rad):
    return unreal.Rotator(0.0, 0.0, -math.degrees(float(yaw_rad)))


def _load(path):
    return unreal.EditorAssetLibrary.load_asset(path)


def _tag(a, *tags):
    t = list(a.tags) if a.tags else []
    for x in (TAG,) + tuple(unreal.Name(str(s)) for s in tags):
        if x not in t:
            t.append(x)
    a.tags = t


def _set_mobility(a, mob):
    try:
        a.root_component.set_mobility(mob)
    except Exception as e:                          # noqa: BLE001
        unreal.log_warning(f"[fable] mobility: {e}")


def _editor_world():
    try:
        return unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem).get_editor_world()
    except Exception:                               # noqa: BLE001
        return unreal.EditorLevelLibrary.get_editor_world()


def ground_z(x_m, y_m, default=0.0):
    """Trace down so props sit ON the terrain."""
    try:
        res = unreal.SystemLibrary.line_trace_single(
            _editor_world(), to_ue(x_m, y_m, 1000.0), to_ue(x_m, y_m, -1000.0),
            unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, True, [], unreal.DrawDebugTrace.NONE, True,
            unreal.LinearColor.RED, unreal.LinearColor.GREEN, 0.0)
    except Exception:                               # noqa: BLE001
        return default
    ok, hit = res if isinstance(res, tuple) else (bool(res), None)
    return float(hit.impact_point.z) / M2U if ok and hit is not None else default


_MAT_CACHE = {}


def _colored_material(rgb):
    """A MaterialInstanceDynamic-free approach: one constant material instance per colour, cached."""
    key = tuple(round(c, 2) for c in rgb)
    if key in _MAT_CACHE:
        return _MAT_CACHE[key]
    parent = _load("/Engine/EngineMaterials/DefaultMaterial.DefaultMaterial")
    name = "MI_fable_%02x%02x%02x" % tuple(int(c * 255) for c in key)
    path = f"{GEN}/{name}.{name}"
    mi = _load(path)
    if mi is None:
        try:
            factory = unreal.MaterialInstanceConstantFactoryNew()
            mi = unreal.AssetToolsHelpers.get_asset_tools().create_asset(name, GEN, unreal.MaterialInstanceConstant, factory)
            mi.set_editor_property("parent", parent)
            unreal.MaterialEditingLibrary.set_material_instance_vector_parameter_value(
                mi, "Color", unreal.LinearColor(*key, 1.0))
            unreal.EditorAssetLibrary.save_loaded_asset(mi)
        except Exception as e:                      # noqa: BLE001
            unreal.log_warning(f"[fable] material colour skipped: {e}")
            mi = None
    _MAT_CACHE[key] = mi
    return mi


def spawn_mesh(shape_or_path, x_m, y_m, z_m, yaw_rad, size_m, label, *tags, color=None, physics=False):
    path = BASIC.get(shape_or_path, shape_or_path)
    mesh = _load(path)
    if mesh is None:
        raise RuntimeError(f"mesh not found: {path}")
    a = _actors().spawn_actor_from_class(unreal.StaticMeshActor, to_ue(x_m, y_m, z_m), yaw_ue(yaw_rad))
    a.static_mesh_component.set_static_mesh(mesh)
    ext = mesh.get_bounds().box_extent            # half extents, cm
    a.set_actor_scale3d(unreal.Vector(size_m[0] * M2U / max(2 * ext.x, 1), size_m[1] * M2U / max(2 * ext.y, 1),
                                      size_m[2] * M2U / max(2 * ext.z, 1)))
    a.set_actor_label(label)
    _set_mobility(a, unreal.ComponentMobility.MOVABLE if physics else unreal.ComponentMobility.STATIC)
    if physics:
        a.static_mesh_component.set_simulate_physics(True)
    if color:
        mi = _colored_material(color)
        if mi:
            a.static_mesh_component.set_material(0, mi)
    _tag(a, label, *tags)
    return a


def clear_generated():
    n = 0
    for a in _actors().get_all_level_actors():
        if a.tags and TAG in a.tags:
            _actors().destroy_actor(a)
            n += 1
    _log(f"cleared {n}")
    return n


# ------------------------------------------------------------------ terrain / water

def build_ground(spec, spec_dir):
    w = spec["world"]
    sx, sy = w["size_m"]
    if spec["scenario"] == "boat":
        water = w.get("water", {})
        lvl = float(water.get("level_m", 0.0))
        a = spawn_mesh("cube", 0, 0, lvl - 0.5, 0, [sx * 1.2, sy * 1.2, 1.0], "FB_Water", "water", "ground",
                       color=(0.05, 0.25, 0.45))
        # water must not collide with the boat (buoyancy is done in C++) but should block lidar? No - it is flat.
        a.static_mesh_component.set_collision_profile_name("NoCollision")
        # seabed far below for anything that sinks
        spawn_mesh("cube", 0, 0, lvl - 6.0, 0, [sx * 1.2, sy * 1.2, 1.0], "FB_Seabed", "seabed", color=(0.2, 0.2, 0.15))
        for sh in water.get("shore", []) or []:
            build_shore(sh)
        return a

    t = w.get("terrain", {})
    mode = t.get("mode", "flat")
    obj = t.get("terrain_obj_path") or os.path.join(spec_dir, "terrain.obj")
    if mode in ("heightmap", "procedural") and os.path.isfile(obj):
        asset = import_terrain_mesh(obj)
        a = _actors().spawn_actor_from_class(unreal.StaticMeshActor, unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0))
        a.static_mesh_component.set_static_mesh(_load(asset))
        a.set_actor_scale3d(unreal.Vector(M2U, -M2U, M2U))      # OBJ authored in ENU metres
        a.set_actor_label("FB_Terrain")
        _set_mobility(a, unreal.ComponentMobility.STATIC)
        _tag(a, "terrain", "ground")
    else:
        if mode != "flat":
            unreal.log_warning(f"[fable] no terrain mesh at {obj} - run tools/sketch_to_heightmap.py --obj; using flat ground")
        color = {"asphalt": (0.12, 0.12, 0.13), "gravel": (0.45, 0.4, 0.35), "grass": (0.2, 0.45, 0.15),
                 "sand": (0.8, 0.7, 0.45)}.get(t.get("surface", {}).get("material_preset", "asphalt"), (0.3, 0.3, 0.3))
        a = spawn_mesh("cube", 0, 0, -0.5, 0, [sx * 1.2, sy * 1.2, 1.0], "FB_Ground", "terrain", "ground", color=color)
    apply_surface(a, t.get("surface", {}))
    return a


def import_terrain_mesh(obj_path, name="SM_FableTerrain"):
    task = unreal.AssetImportTask()
    task.filename = obj_path
    task.destination_path = GEN
    task.destination_name = name
    task.replace_existing = True
    task.automated = True
    task.save = True
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    path = f"{GEN}/{name}.{name}"
    mesh = _load(path)
    if mesh is None:
        raise RuntimeError(f"terrain import failed: {obj_path}")
    body = mesh.get_editor_property("body_setup")
    if body:
        body.set_editor_property("collision_trace_flag", unreal.CollisionTraceFlag.CTF_USE_COMPLEX_AS_SIMPLE)
    unreal.EditorAssetLibrary.save_asset(path)
    return path


def apply_surface(actor, surface):
    preset = surface.get("material_preset", "asphalt")
    name = f"PM_fable_{preset}"
    path = f"{GEN}/{name}.{name}"
    pm = _load(path)
    if pm is None:
        pm = unreal.AssetToolsHelpers.get_asset_tools().create_asset(name, GEN, unreal.PhysicalMaterial,
                                                                     unreal.PhysicalMaterialFactoryNew())
    pm.set_editor_property("friction", float(surface.get("friction", 0.9)))
    pm.set_editor_property("restitution", float(surface.get("restitution", 0.1)))
    unreal.EditorAssetLibrary.save_loaded_asset(pm)
    try:
        comp = actor.static_mesh_component
        bi = comp.get_editor_property("body_instance")
        bi.set_editor_property("phys_material_override", pm)
        comp.set_editor_property("body_instance", bi)
    except Exception as e:                          # noqa: BLE001
        unreal.log_warning(f"[fable] phys material: {e}")


def build_shore(sh):
    """Island / shoreline: a polygon extruded as a ring of wall segments (blocks lidar + collides)."""
    pts = sh["polygon_m"]
    h = float(sh.get("height_m", 1.5))
    lab = sh.get("label", "shore")
    made = []
    n = len(pts)
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        made.append(spawn_mesh("cube", (a[0] + b[0]) / 2, (a[1] + b[1]) / 2, h / 2 - 0.3, math.atan2(dy, dx),
                               [L, 0.8, h], f"{lab}_{i}", "shore", "obstacle", color=(0.35, 0.3, 0.2)))
    # fill: a flat slab so it reads as land from above
    cx = sum(p[0] for p in pts) / n
    cy = sum(p[1] for p in pts) / n
    r = max(math.hypot(p[0] - cx, p[1] - cy) for p in pts)
    made.append(spawn_mesh("cylinder", cx, cy, 0.1, 0, [r * 1.4, r * 1.4, 0.4], f"{lab}_fill", "shore", color=(0.3, 0.45, 0.2)))
    return made


# ------------------------------------------------------------------ track / gates / dock / objects

def build_track(spec):
    tr = spec.get("track") or {}
    pts = tr.get("centerline_m") or []
    if len(pts) < 2 or tr.get("border", "none") == "none":
        return []
    closed = bool(tr.get("closed", False))
    half = float(tr.get("width_m", 2.0)) / 2
    spacing = float(tr.get("border_spacing_m", 1.5))
    border = tr["border"]
    made, acc = [], 0.0
    n = len(pts)
    for i in range(n if closed else n - 1):
        a, b = pts[i], pts[(i + 1) % n]
        dx, dy = b[0] - a[0], b[1] - a[1]
        seg = math.hypot(dx, dy)
        if seg < 1e-6:
            continue
        nx, ny = -dy / seg, dx / seg
        yaw = math.atan2(dy, dx)
        acc += seg
        while acc >= spacing:
            acc -= spacing
            t = 1 - acc / seg
            cx, cy = a[0] + dx * t, a[1] + dy * t
            for side in (1, -1):
                px, py = cx + nx * half * side, cy + ny * half * side
                z = ground_z(px, py)
                k = len(made)
                if border == "cones":
                    made.append(spawn_mesh("cone", px, py, z + 0.275, 0, [0.36, 0.36, 0.55], f"cone_{k}", "track_border", "obstacle", color=COLORS["cone"]))
                elif border == "walls":
                    made.append(spawn_mesh("cube", px, py, z + 0.35, yaw, [spacing, 0.12, 0.7], f"wall_{k}", "track_border", "obstacle"))
                elif border == "curbs":
                    made.append(spawn_mesh("cube", px, py, z + 0.05, yaw, [spacing, 0.25, 0.1], f"curb_{k}", "track_border", color=(0.9, 0.9, 0.9)))
                elif border == "paint":
                    made.append(spawn_mesh("cube", px, py, z + 0.005, yaw, [spacing, 0.1, 0.01], f"paint_{k}", "track_paint", color=(1, 1, 1)))
    _log(f"track border: {len(made)}")
    return made


def build_gates(spec):
    made = []
    lvl = float(spec["world"].get("water", {}).get("level_m", 0.0))
    for i, g in enumerate(spec.get("gates") or []):
        lab = g.get("label", f"gate{i + 1}")
        made.append(spawn_mesh("sphere", g["left_m"][0], g["left_m"][1], lvl + 0.1, 0, KINDS["buoy_red"][1], f"{lab}_L", "gate", "buoy_red", "obstacle", color=COLORS["buoy_red"]))
        made.append(spawn_mesh("sphere", g["right_m"][0], g["right_m"][1], lvl + 0.1, 0, KINDS["buoy_green"][1], f"{lab}_R", "gate", "buoy_green", "obstacle", color=COLORS["buoy_green"]))
    return made


def build_dock(spec):
    d = spec.get("dock")
    if not d:
        return []
    cx, cy = d["xy_m"]
    yaw = float(d.get("yaw_rad", 0))
    w, L, P = float(d.get("slip_width_m", 1.6)), float(d.get("slip_length_m", 3.0)), float(d.get("pier_length_m", 8))
    lvl = float(spec["world"].get("water", {}).get("level_m", 0.0))
    c, s = math.cos(yaw), math.sin(yaw)

    def tw(x, y):
        return cx + c * x - s * y, cy + s * x + c * y

    made = []
    px, py = tw(L / 2 + 0.3, 0)
    made.append(spawn_mesh("cube", px, py, lvl + 0.15, yaw, [0.6, P, 0.6], "pier", "dock", "obstacle", color=COLORS["pier"]))
    for k, side in enumerate((1, -1)):
        fx, fy = tw(0.0, side * (w / 2 + 0.15))
        made.append(spawn_mesh("cube", fx, fy, lvl + 0.15, yaw, [L, 0.3, 0.6], f"pier_finger_{k}", "dock", "obstacle", color=COLORS["pier"]))
    gx, gy = tw(0, 0)
    made.append(spawn_mesh("cylinder", gx, gy, lvl - 0.05, yaw, [0.4, 0.4, 0.02], "dock_target", "goal", color=(0.1, 0.9, 0.9)))
    return made


def build_objects(spec):
    made = []
    lvl = float(spec["world"].get("water", {}).get("level_m", 0.0))
    for i, o in enumerate(spec.get("objects") or []):
        kind = o["kind"]
        lab = o.get("label") or f"{kind}_{i}"
        x, y = o["xy_m"]
        yaw = float(o.get("yaw_rad", 0))
        tag = o.get("tag", "obstacle")
        if kind == "custom_mesh":
            if not o.get("asset_path"):
                continue
            made.append(spawn_mesh(o["asset_path"], x, y, o.get("z_m", ground_z(x, y)), yaw, o.get("scale") or [1, 1, 1], lab, tag, kind))
            continue
        shape, size, floats = KINDS.get(kind, ("cube", [0.5, 0.5, 0.5], False))
        sc = o.get("scale") or [1, 1, 1]
        sz = [size[j] * sc[j] for j in range(3)]
        z = o.get("z_m")
        if z is None:
            z = (lvl + sz[2] * 0.2) if floats else (ground_z(x, y) + sz[2] / 2)
        made.append(spawn_mesh(shape, x, y, z, yaw, sz, lab, tag, kind, color=COLORS.get(kind), physics=bool(o.get("movable"))))
    _log(f"objects: {len(made)}")
    return made


def build_fields(spec, rng):
    made = []
    lvl = float(spec["world"].get("water", {}).get("level_m", 0.0))
    tr = spec.get("track") or {}
    cl = tr.get("centerline_m") or []
    half = float(tr.get("width_m", 2.0)) / 2
    gates = [g["left_m"] for g in spec.get("gates") or []] + [g["right_m"] for g in spec.get("gates") or []]
    sx, sy = spec["spawn"]["xy_m"]

    def near_line(x, y, keep):
        for i in range(len(cl) - (0 if tr.get("closed") else 1)):
            a, b = cl[i], cl[(i + 1) % len(cl)]
            ex, ey = b[0] - a[0], b[1] - a[1]
            L2 = ex * ex + ey * ey or 1e-9
            t = max(0, min(1, ((x - a[0]) * ex + (y - a[1]) * ey) / L2))
            if math.hypot(x - (a[0] + t * ex), y - (a[1] + t * ey)) < keep + half:
                return True
        return False

    for f in spec.get("obstacle_fields") or []:
        kind = f["kind"]
        shape, size, floats = KINDS.get(kind, ("cube", [0.5, 0.5, 0.5], False))
        x0, y0, x1, y1 = [float(v) for v in f["area_m"]]
        sep = float(f.get("min_separation_m", 1.5))
        keep = float(f.get("avoid_track_m", 1.0))
        jit = float(f.get("scale_jitter", 0.15))
        placed, tries = [], 0
        while len(placed) < int(f["count"]) and tries < int(f["count"]) * 80:
            tries += 1
            x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
            if any(math.hypot(x - a, y - b) < sep for a, b in placed):
                continue
            if keep > 0 and (near_line(x, y, keep) or any(math.hypot(x - g[0], y - g[1]) < keep + 2 for g in gates)):
                continue
            if math.hypot(x - sx, y - sy) < 3.0:
                continue
            placed.append((x, y))
            s = rng.uniform(1 - jit, 1 + jit)
            sz = [d * s for d in size]
            z = (lvl + sz[2] * 0.2) if floats else (ground_z(x, y) + sz[2] / 2)
            made.append(spawn_mesh(shape, x, y, z, rng.uniform(0, 2 * math.pi), sz, f"field_{kind}_{len(placed)}",
                                   "obstacle", "field", kind, color=COLORS.get(kind)))
    _log(f"fields: {len(made)}")
    return made


# ------------------------------------------------------------------ lighting, vehicle, bridge

LIGHT = {"noon": (110000, 5800, 1.0), "overcast": (18000, 6800, 1.4), "dusk": (2500, 3200, 0.7),
         "night": (120, 9000, 0.25), "indoor": (0, 4200, 1.8)}


def build_lighting(spec):
    lg = spec["world"].get("lighting", {})
    lux, temp, sky = LIGHT.get(lg.get("preset", "noon"), LIGHT["noon"])
    sun = _actors().spawn_actor_from_class(unreal.DirectionalLight, unreal.Vector(0, 0, 1000),
                                           unreal.Rotator(0, math.degrees(float(lg.get("sun_pitch_rad", -0.9))),
                                                          -math.degrees(float(lg.get("sun_yaw_rad", -0.8)))))
    c = sun.directional_light_component
    c.set_intensity(lux)
    c.set_editor_property("use_temperature", True)
    c.set_editor_property("temperature", temp)
    _set_mobility(sun, unreal.ComponentMobility.MOVABLE)
    sun.set_actor_label("FB_Sun")
    _tag(sun, "lighting")
    sl = _actors().spawn_actor_from_class(unreal.SkyLight, unreal.Vector(0, 0, 500), unreal.Rotator(0, 0, 0))
    sl.light_component.set_intensity(sky)
    _set_mobility(sl, unreal.ComponentMobility.MOVABLE)
    sl.set_actor_label("FB_Sky")
    _tag(sl, "lighting")
    fog = _actors().spawn_actor_from_class(unreal.ExponentialHeightFog, unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0))
    fog.get_component_by_class(unreal.ExponentialHeightFogComponent).set_editor_property("fog_density", float(lg.get("fog_density", 0.02)))
    fog.set_actor_label("FB_Fog")
    _tag(fog, "lighting")
    try:
        sky_atm = _actors().spawn_actor_from_class(unreal.SkyAtmosphere, unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0))
        sky_atm.set_actor_label("FB_Atmosphere")
        _tag(sky_atm, "lighting")
    except Exception:                               # noqa: BLE001
        pass


def build_vehicle_and_bridge(spec, vehicle_params: dict | None):
    """Places the pawn and the FableBridge actor with the spec's spawn pose and environment."""
    sp = spec["spawn"]
    x, y = sp["xy_m"]
    yaw = float(sp.get("yaw_rad", 0))
    is_boat = spec["scenario"] == "boat"
    cls_name = "FableBoatPawn" if is_boat else "FableCarPawn"
    bp_path = f"/Game/Fable/BP_{cls_name}.BP_{cls_name}"
    pawn = None
    bp = _load(bp_path)
    z = (float(spec["world"].get("water", {}).get("level_m", 0)) + 0.2) if is_boat else (ground_z(x, y) + 0.3)
    if bp is not None:
        pawn = _actors().spawn_actor_from_object(bp, to_ue(x, y, z), yaw_ue(yaw))
    else:
        cls = getattr(unreal, cls_name, None)
        if cls is not None:
            pawn = _actors().spawn_actor_from_class(cls, to_ue(x, y, z), yaw_ue(yaw))
    if pawn is None:
        unreal.log_warning(f"[fable] no {bp_path} and no plugin class {cls_name}; placing a spawn marker")
        spawn_mesh("sphere", x, y, z, yaw, [0.3, 0.3, 0.3], "spawn_marker", "spawn", color=(1, 0, 1))
    else:
        pawn.set_actor_label("FB_Vehicle")
        _tag(pawn, "vehicle")
        try:
            pawn.set_editor_property("auto_possess_player", unreal.AutoReceiveInput.PLAYER0)
        except Exception:                           # noqa: BLE001
            pass

    bridge_cls = getattr(unreal, "FableBridgeActor", None)
    bridge = None
    if bridge_cls is not None:
        bp_b = _load("/Game/Fable/BP_FableBridge.BP_FableBridge")
        bridge = (_actors().spawn_actor_from_object(bp_b, unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0)) if bp_b
                  else _actors().spawn_actor_from_class(bridge_cls, unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0)))
        bridge.set_actor_label("FB_Bridge")
        _tag(bridge, "bridge")
        try:
            bridge.set_editor_property("spawn_xym", unreal.Vector2D(float(x), float(y)))
            bridge.set_editor_property("spawn_yaw_rad", yaw)
            bridge.set_editor_property("spawn_jitter_m", float(sp.get("jitter_m", 0.3)))
            bridge.set_editor_property("spawn_jitter_yaw_rad", float(sp.get("jitter_yaw_rad", 0.1)))
            bridge.set_editor_property("bounds_m", unreal.Vector2D(float(spec["world"]["size_m"][0]), float(spec["world"]["size_m"][1])))
            if pawn is not None:
                bridge.set_editor_property("vehicle", pawn)
            geo = spec["world"].get("geo_origin")
            if geo:
                bridge.set_editor_property("geo_lat", float(geo["lat"]))
                bridge.set_editor_property("geo_lon", float(geo["lon"]))
            env = bridge.get_editor_property("env")
            d = spec["world"].get("disturbance", {})
            w, c = d.get("wind", {}), d.get("current", {})
            wv = spec["world"].get("water", {}).get("waves", {})
            env.set_editor_property("wind_speed", float(w.get("speed_mps", 0)))
            env.set_editor_property("wind_dir", float(w.get("dir_rad", 0)))
            env.set_editor_property("gust_std", float(w.get("gust_std_mps", 0)))
            env.set_editor_property("gust_period", float(w.get("gust_period_s", 8)))
            env.set_editor_property("current_speed", float(c.get("speed_mps", 0)))
            env.set_editor_property("current_dir", float(c.get("dir_rad", 0)))
            env.set_editor_property("wave_amp", float(wv.get("amp_m", 0)))
            env.set_editor_property("wave_period", float(wv.get("period_s", 3)))
            env.set_editor_property("wave_dir", float(wv.get("dir_rad", 0)))
            env.set_editor_property("water_level_m", float(spec["world"].get("water", {}).get("level_m", 0)))
            bridge.set_editor_property("env", env)
        except Exception as e:                      # noqa: BLE001
            unreal.log_warning(f"[fable] bridge properties: {e} (set them by hand on FB_Bridge)")
    else:
        unreal.log_warning("[fable] FableBridge plugin not loaded - level has no bridge actor")
    return pawn, bridge


# ------------------------------------------------------------------ entry points

def build_from_spec(spec: dict, spec_dir: str = "", clear: bool = True, vehicle_params: dict | None = None) -> dict:
    if spec.get("spec_version") != "2.0":
        raise ValueError("scene spec_version must be '2.0'")
    rng = random.Random(int(spec.get("seed", 0)))
    if clear:
        clear_generated()
    build_lighting(spec)
    build_ground(spec, spec_dir)
    n_track = len(build_track(spec))
    n_gates = len(build_gates(spec))
    n_dock = len(build_dock(spec))
    n_obj = len(build_objects(spec))
    n_field = len(build_fields(spec, rng))
    build_vehicle_and_bridge(spec, vehicle_params)
    try:
        _level().save_current_level()
    except Exception as e:                          # noqa: BLE001
        unreal.log_warning(f"[fable] save: {e}")
    s = {"name": spec["name"], "scenario": spec["scenario"], "seed": spec.get("seed", 0), "track_border": n_track,
         "gate_buoys": n_gates, "dock_pieces": n_dock, "objects": n_obj, "field_obstacles": n_field,
         "size_m": spec["world"]["size_m"]}
    _log(f"built {s}")
    return s


def build_from_spec_file(path: str, clear: bool = True) -> dict:
    with open(path) as f:
        spec = json.load(f)
    vp = None
    if spec.get("vehicle_params"):
        p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(path))), spec["vehicle_params"])
        if os.path.isfile(p):
            with open(p) as f:
                vp = json.load(f)
    return build_from_spec(spec, os.path.dirname(os.path.abspath(path)), clear, vp)


def new_level_and_build(spec_path: str, level_dir: str = "/Game/Fable/Maps") -> dict:
    with open(spec_path) as f:
        name = json.load(f)["name"]
    _level().new_level(f"{level_dir}/{name}")
    return build_from_spec_file(spec_path, clear=False)


# ------------------------------------------------------------------ incremental edits (used by the MCP toolset)

def add_object(kind, x_m, y_m, yaw_rad=0.0, label=None, scale=None, tag="obstacle", water_level=0.0):
    shape, size, floats = KINDS.get(kind, ("cube", [0.5, 0.5, 0.5], False))
    sc = scale or [1, 1, 1]
    sz = [size[j] * sc[j] for j in range(3)]
    z = (water_level + sz[2] * 0.2) if floats else (ground_z(x_m, y_m) + sz[2] / 2)
    label = label or f"{kind}_{random.randint(1000, 9999)}"
    spawn_mesh(shape, x_m, y_m, z, yaw_rad, sz, label, tag, kind, "added", color=COLORS.get(kind))
    return label


def remove_by_label(label):
    n = 0
    for a in _actors().get_all_level_actors():
        if a.tags and unreal.Name(label) in a.tags:
            _actors().destroy_actor(a)
            n += 1
    return n


def move_by_label(label, x_m, y_m, yaw_rad=None):
    for a in _actors().get_all_level_actors():
        if a.tags and unreal.Name(label) in a.tags:
            loc = a.get_actor_location()
            a.set_actor_location(unreal.Vector(x_m * M2U, -y_m * M2U, loc.z), False, False)
            if yaw_rad is not None:
                a.set_actor_rotation(yaw_ue(yaw_rad), False)
            return True
    return False


def describe() -> dict:
    counts, items = {}, []
    for a in _actors().get_all_level_actors():
        if not a.tags or TAG not in a.tags:
            continue
        loc = a.get_actor_location()
        tags = [str(t) for t in a.tags if str(t) != "fable"]
        for t in tags:
            counts[t] = counts.get(t, 0) + 1
        items.append({"label": a.get_actor_label(), "x": round(loc.x / M2U, 2), "y": round(-loc.y / M2U, 2),
                      "yaw": round(-math.radians(a.get_actor_rotation().yaw), 3), "tags": tags})
    return {"tag_counts": counts, "actors": items}
