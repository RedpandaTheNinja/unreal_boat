"""
boatlab_course.py - builds the AIMM-ICC 2025 course into the open Unreal level (editor Python).

Run it from the editor (Output Log -> Python, or Tools -> Execute Python Script):

    import boatlab_course as bc
    bc.build()                 # fit the lake to the course, place buoys/target/case/detectors/marina, save level
    bc.clear()                 # remove everything BoatLab placed (tag "boatlab")
    bc.describe()              # list placed objects in BoatLab ENU metres

or through the Unreal MCP server (boatlab_toolset.py registers the same functions as tools).

The course JSON (BoatLab/config/course_aimm_2025.json) is the single source of truth shared with
the Python autonomy stack; this script only mirrors it into the level.
Frames: BoatLab ENU (x east, y north, metres) -> UE: X = 100 x, Y = -100 y, Z up (cm).
"""
from __future__ import annotations

import json
import math
import os

import unreal

TAG = "boatlab"
FOLDER = "BoatLab/Course"
DOCK_FOLDER = "BoatLab/Dock"
SHORE_W, GRASS_W = 1.8, 6.5     # shore and grass strip widths (m) around the water, as in Lake_300x80ft
LEGACY_DOCK = ("Dock_Plank_", "Dock_Post")   # the level's original stub pier, replaced by the marina
MAT_DIR = "/Game/BoatLab/Materials"
BASIC = {"cylinder": "/Engine/BasicShapes/Cylinder.Cylinder", "cube": "/Engine/BasicShapes/Cube.Cube",
         "sphere": "/Engine/BasicShapes/Sphere.Sphere"}
BASE_MATERIAL = "/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"
COLORS = {  # linear colours (BasicShapeMaterial "Color")
    "red": (0.75, 0.02, 0.02), "green": (0.18, 0.75, 0.05), "blue": (0.02, 0.12, 0.7), "orange": (0.95, 0.25, 0.02),
    "purple": (0.3, 0.03, 0.4), "yellow": (0.9, 0.7, 0.02), "black": (0.01, 0.01, 0.01), "white": (0.85, 0.85, 0.85),
    "pink": (0.95, 0.18, 0.55), "case": (0.95, 0.72, 0.01), "cap": (0.012, 0.012, 0.012),
    "wood": (0.32, 0.22, 0.12), "boat": (0.72, 0.73, 0.75), "boat_cover": (0.03, 0.05, 0.08),
}


def _project_dir():
    return unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_dir())


def course_path():
    return os.environ.get("BOATLAB_COURSE") or os.path.join(_project_dir(), "BoatLab", "config", "course_aimm_2025.json")


def load_course(path=None):
    with open(path or course_path(), encoding="utf-8") as f:
        return json.load(f)


def _actors():
    return unreal.get_editor_subsystem(unreal.EditorActorSubsystem)


def _ue(x, y, z=0.0):
    return unreal.Vector(float(x) * 100.0, -float(y) * 100.0, float(z) * 100.0)


def _log(msg):
    unreal.log(f"[boatlab] {msg}")


_TOUCHED = set()


def _touch(actor):
    """Call before editing an existing actor. Python transform setters do not mark the actor dirty,
    so in this World Partition level the edit would be lost on the next load."""
    actor.modify()
    _TOUCHED.add(actor.get_package().get_name())


def _save():
    unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).save_current_level()
    pkgs = [p for p in (unreal.find_package(n) for n in sorted(_TOUCHED)) if p is not None]
    if pkgs and not unreal.EditorLoadingAndSavingUtils.save_packages(pkgs, True):
        raise RuntimeError("[boatlab] saving edited actor packages failed")
    _TOUCHED.clear()


_MATS = {}


def material(name):
    if name in _MATS:
        return _MATS[name]
    path = f"{MAT_DIR}/MI_BoatLab_{name}"
    mi = unreal.EditorAssetLibrary.load_asset(path) if unreal.EditorAssetLibrary.does_asset_exist(path) else None
    want = unreal.LinearColor(*COLORS[name], 1.0)
    if mi is None:
        tools = unreal.AssetToolsHelpers.get_asset_tools()
        mi = tools.create_asset(f"MI_BoatLab_{name}", MAT_DIR, unreal.MaterialInstanceConstant,
                                unreal.MaterialInstanceConstantFactoryNew())
        mi.set_editor_property("parent", unreal.EditorAssetLibrary.load_asset(BASE_MATERIAL))
        unreal.MaterialEditingLibrary.set_material_instance_vector_parameter_value(mi, "Color", want)
        unreal.EditorAssetLibrary.save_loaded_asset(mi)
    elif not unreal.MaterialEditingLibrary.get_material_instance_vector_parameter_value(mi, "Color").is_near_equal(want, 1e-3):
        unreal.MaterialEditingLibrary.set_material_instance_vector_parameter_value(mi, "Color", want)   # COLORS changed
        unreal.EditorAssetLibrary.save_loaded_asset(mi)
    _MATS[name] = mi
    return mi


def _spawn(shape, x, y, z_bottom, size_m, color, label, tags, collide=True, yaw_deg=0.0, folder=FOLDER):
    """Spawn a basic shape with its BOTTOM at z_bottom (m). size_m = (dx, dy, dz) metres."""
    mesh = unreal.EditorAssetLibrary.load_asset(BASIC[shape])
    a = _actors().spawn_actor_from_class(unreal.StaticMeshActor, _ue(x, y, z_bottom + size_m[2] / 2),
                                         unreal.Rotator(0.0, 0.0, -yaw_deg))
    smc = a.static_mesh_component
    smc.set_static_mesh(mesh)
    a.set_actor_scale3d(unreal.Vector(size_m[0], size_m[1], size_m[2]))     # basic shapes are 1 m
    smc.set_material(0, material(color))
    smc.set_collision_profile_name("BlockAll" if collide else "NoCollision")
    a.set_actor_label(label)
    a.set_folder_path(folder)
    a.tags = [unreal.Name(TAG)] + [unreal.Name(t) for t in tags]
    return a


def clear():
    n = 0
    for a in _actors().get_all_level_actors():
        if a.tags and unreal.Name(TAG) in a.tags:
            _touch(a)
            _actors().destroy_actor(a)
            n += 1
    _log(f"cleared {n} actors")
    return n


def _find(label):
    for a in _actors().get_all_level_actors():
        if a.get_actor_label() == label:
            return a
    return None


def fit_lake(course=None, save=True):
    """Size the lake level to the course JSON water rectangle: water surface and lake bed, with the
    shore and grass strips around them. Placement is absolute (idempotent); heights are kept."""
    course = course or load_course()
    w = course["water"]
    x0, x1, y0, y1 = w["xmin"], w["xmax"], w["ymin"], w["ymax"]
    cx, cy, lx, ly = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
    water = next((a for a in _actors().get_all_level_actors()
                  if a.tags and unreal.Name("BoatWaterSurface") in a.tags), None)
    if water is None:
        raise RuntimeError("[boatlab] no actor tagged BoatWaterSurface in the open level")
    ring = SHORE_W + GRASS_W
    adjusted = []

    def place(a, x, y, sx, sy):
        _touch(a)
        z = a.get_actor_location().z
        a.set_actor_location(unreal.Vector(x * 100.0, -y * 100.0, z), False, False)
        a.set_actor_scale3d(unreal.Vector(sx, sy, a.get_actor_scale3d().z))     # cube / wave grid are 1 m
        adjusted.append(a.get_actor_label())

    for a in _actors().get_all_level_actors():
        lab = a.get_actor_label()
        if a == water or lab.startswith("LakeBed"):
            place(a, cx, cy, lx, ly)
        elif lab.startswith(("Shore_", "Grass_")):
            loc = a.get_actor_location()
            shore = lab.startswith("Shore_")
            t = SHORE_W if shore else GRASS_W
            off = t / 2 if shore else SHORE_W + t / 2
            if abs(loc.x) > abs(loc.y):                                     # east / west strip
                place(a, (x1 + off) if loc.x > 0 else (x0 - off), cy, t, ly + 2 * SHORE_W)
            else:                                                           # UE -Y is ENU north
                place(a, cx, (y1 + off) if loc.y < 0 else (y0 - off), lx if shore else lx + 2 * ring, t)
    lft, wft = round(lx / 0.3048), round(ly / 0.3048)
    water.tags = [unreal.Name(f"LengthFt={lft}") if str(t).startswith("LengthFt=") else
                  unreal.Name(f"WidthFt={wft}") if str(t).startswith("WidthFt=") else t for t in water.tags]
    water.set_actor_label(f"Lake_Water_{lft}ft_x_{wft}ft")
    if save:
        _save()
    _log(f"lake fitted to {lft} x {wft} ft ({len(adjusted)} actors adjusted)")
    return {"length_ft": lft, "width_ft": wft, "adjusted": adjusted}


def remove_legacy_dock():
    """Delete the level's original stub pier (Dock_Plank_* / Dock_Post); the marina replaces it."""
    n = 0
    for a in _actors().get_all_level_actors():
        if a.get_actor_label().startswith(LEGACY_DOCK) and not (a.tags and unreal.Name(TAG) in a.tags):
            _touch(a)
            _actors().destroy_actor(a)
            n += 1
    return n


def _marina(course):
    """Piers and moored boats from the course keepouts (style pier / boat), all colliding."""
    for k in course.get("keepout", []):
        xs = [p[0] for p in k["polygon"]]
        ys = [p[1] for p in k["polygon"]]
        x, y, sx, sy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, max(xs) - min(xs), max(ys) - min(ys)
        h = k.get("height_m", 0.5)
        tags = ["boatlab_dock", f"boatlab_{k['id']}"]
        if k.get("style") == "boat":
            _spawn("cube", x, y, -0.3, (sx, sy, h), "boat", f"BL_{k['id']}", tags, folder=DOCK_FOLDER)
            cover = (sx * 0.55, sy * 0.7, 0.3) if sx > sy else (sx * 0.7, sy * 0.55, 0.3)
            _spawn("cube", x, y, h - 0.3, cover, "boat_cover", f"BL_{k['id']}_cover", tags, folder=DOCK_FOLDER)
        else:
            _spawn("cube", x, y, h - 0.5, (sx, sy, 0.3), "wood", f"BL_{k['id']}", tags, folder=DOCK_FOLDER)


def _buoy(o, zebra=False):
    d = 2 * o.get("radius", 0.135)
    h = o.get("height", 0.55)
    sub = 0.2                                   # submerged part (keeps the waterline natural)
    tags = [f"boatlab_{o['id']}", o.get("role", "buoy")]
    body_color = "white" if zebra else o["color"]
    body = _spawn("cylinder", o["x"], o["y"], -sub, (d, d, h + sub - 0.08), body_color, f"BL_{o['id']}", tags)
    _spawn("cylinder", o["x"], o["y"], h - 0.08, (d * 0.92, d * 0.92, 0.08), "cap", f"BL_{o['id']}_cap", tags)
    if zebra:
        # 2 in black tape wound around the white fender: approximated by tilted rings
        for i, z in enumerate((0.08, 0.2, 0.32, 0.44)):
            ring = _spawn("cylinder", o["x"], o["y"], z, (d * 1.04, d * 1.04, 0.05), "black", f"BL_{o['id']}_tape{i}", tags,
                          collide=False)
            ring.set_actor_rotation(unreal.Rotator(12.0, 0.0, 35.0 * i), False)
    return body


def _target(o):
    """VEVOR 10 ft trampoline (PDF p11): orange tube with a white top, black mat inside."""
    d, t = 2 * o["radius"], o.get("height", 0.2)
    mat = 2 * o.get("mat_radius", 0.8 * o["radius"])
    tags = [f"boatlab_{o['id']}", "target"]
    _spawn("cylinder", o["x"], o["y"], -t / 2, (d, d, t), "orange", f"BL_{o['id']}", tags)
    _spawn("cylinder", o["x"], o["y"], t / 2, (d * 0.97, d * 0.97, 0.01), "white", f"BL_{o['id']}_rim", tags, collide=False)
    _spawn("cylinder", o["x"], o["y"], t / 2 + 0.005, (mat, mat, 0.01), "black", f"BL_{o['id']}_mat", tags, collide=False)


def build(course_file=None, fit=True, save=True):
    course = load_course(course_file)
    summary = {"course": course.get("name"), "placed": []}
    if fit:
        summary["lake"] = fit_lake(course=course, save=False)
    summary["legacy_dock_removed"] = remove_legacy_dock()
    clear()
    for o in course["objects"]:
        k = o["kind"]
        if k == "buoy":
            _buoy(o, zebra=(o["color"] == "zebra"))
        elif k == "detector":
            _spawn("cylinder", o["x"], o["y"], -0.2, (0.6, 0.6, 0.35), "white", f"BL_{o['id']}_float", [f"boatlab_{o['id']}"])
            _spawn("cylinder", o["x"], o["y"], 0.15, (0.3, 0.3, o.get("height", 0.9)), "pink", f"BL_{o['id']}",
                   [f"boatlab_{o['id']}", "detector"])
        elif k == "target":
            _target(o)
        elif k == "case":
            sx, sy, sz = o.get("size", [0.33, 0.246, 0.147])
            _spawn("cube", o["x"], o["y"], -0.3 * sz, (sx, sy, sz), "case", f"BL_{o['id']}", [f"boatlab_{o['id']}", "recover"],
                   collide=False)
        summary["placed"].append(o["id"])
    _marina(course)
    summary["marina_parts"] = len(course.get("keepout", []))
    x, y, h = course["start_pose"]
    summary["boat_start"] = set_boat_start(x, y, math.degrees(h), save=False)
    if save:
        _save()
    _log(f"built {len(summary['placed'])} course objects and {summary['marina_parts']} marina parts from "
         f"{course_file or course_path()}")
    return summary


def describe():
    out = []
    for a in _actors().get_all_level_actors():
        if not a.tags or unreal.Name(TAG) not in a.tags:
            continue
        loc = a.get_actor_location()
        out.append({"label": a.get_actor_label(), "x": round(loc.x / 100, 3), "y": round(-loc.y / 100, 3),
                    "z": round(loc.z / 100, 3), "tags": [str(t) for t in a.tags]})
    boat = _find("Boat_13ft_x_3ft_TwinMotors")
    if boat is not None:
        loc, rot = boat.get_actor_location(), boat.get_actor_rotation()
        out.append({"label": "Boat_13ft_x_3ft_TwinMotors", "x": round(loc.x / 100, 3), "y": round(-loc.y / 100, 3),
                    "heading_deg": round(-rot.yaw, 1)})
    return out


def move_object(object_id, x, y, course_file=None):
    """Move one course object (updates the JSON first, then the level)."""
    path = course_file or course_path()
    course = load_course(path)
    for o in course["objects"]:
        if o["id"] == object_id:
            dx, dy = x - o["x"], y - o["y"]
            o["x"], o["y"] = x, y
            break
    else:
        raise KeyError(object_id)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(course, f, indent=2)
    n = 0
    for a in _actors().get_all_level_actors():
        if a.tags and unreal.Name(f"boatlab_{object_id}") in a.tags:
            _touch(a)
            loc = a.get_actor_location()
            a.set_actor_location(unreal.Vector(loc.x + dx * 100, loc.y - dy * 100, loc.z), False, False)
            n += 1
    _save()
    return {"moved": object_id, "actors": n, "x": x, "y": y}


def set_boat_start(x, y, heading_deg, save=True):
    """Move the boat actor (editor, not during Play) to an ENU pose."""
    boat = _find("Boat_13ft_x_3ft_TwinMotors")
    if boat is None:
        raise RuntimeError("Boat_13ft_x_3ft_TwinMotors not found in the open level")
    _touch(boat)
    loc = boat.get_actor_location()
    boat.set_actor_location_and_rotation(unreal.Vector(x * 100, -y * 100, loc.z),
                                         unreal.Rotator(0.0, 0.0, -heading_deg), False, False)
    if save:
        _save()
    return {"x": x, "y": y, "heading_deg": heading_deg}


if __name__ == "__main__":
    print(json.dumps(build(), indent=2))
