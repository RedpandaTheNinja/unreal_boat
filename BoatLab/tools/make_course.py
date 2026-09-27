"""Generate config/course_aimm_2025.json from the AIMM-ICC 2025 course map.

The PDF map (page 2) gives distances in feet. It is NOT to scale for gate
widths or small offsets, so every number here is a named, editable parameter.
Re-run this script after editing, or edit the generated JSON directly.

Frames
------
local ENU metres: x = east, y = north, heading CCW from east (radians).
In Unreal this is x = UE_X/100, y = -UE_Y/100 (UE is left-handed, Y = right).

Reading of the map (dock on the east/right, course runs west then returns):
  top line   : start buoy -> 45 ft -> Gate A (C1 gate + slalom start)
               -> slalom red/green/red/green every 30 ft -> Gate B (30 ft)
               -> 15 ft -> channel entry (green) -> 40 ft -> red corner
  channel    : 40 ft wide, 60 ft long, runs south. Reds on the west bank,
               greens on the east bank, pink LiDAR detectors on each bank (C3)
  bottom line: back east: identify cluster (C4), zebra (C5), black buoy +
               launch target 15 ft north (C6), floating case (C7),
               blue return buoy (C9), dock
The blue dot drawn between each gate pair is treated as a timing-line
marker, not a physical buoy.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

FT = 0.3048
ROOT = Path(__file__).resolve().parents[1]

PARAMS = dict(
    anchor_ft=[115.0, 32.0],      # start buoy position in local frame (feet)
    gate_width_ft=10.0,           # red-to-green centre spacing (user confirmed 10 ft)
    buoy_diameter_m=0.27,         # Taylor Made Super Gard fender, 10.5 x 30 in (PDF p4)
    buoy_height_above_water_m=0.55,
    # Identify cluster as drawn on the map (300 dpi, ~7.6 px/ft): four fenders ~3.2 ft apart, ~2.2 ft
    # north of the bottom line, centred on the course-line marker 15 ft east of the channel exit green.
    identify_spacing_ft=3.2,
    identify_row_offset_ft=2.2,
    identify_center_dx_ft=-195.0,
    launch_target_diameter_ft=10.0,   # VEVOR 10 ft trampoline: 10 ft tube, 8 ft black mat, 8 in thick (PDF p11)
    launch_target_offset_ft=15.0,
    case_size_in=[13.0, 9.7, 5.8],    # Jack Boss waterproof case, outside dimensions (PDF p12)
    return_dx_ft=-15.0,               # "~30'" east of the case
    detector_radius_m=6.0,        # assumed LiDAR detector reach; real value unknown
    # Lake 350 x 200 ft. The PDF course sits in an open lake: the map shows roughly 55-80 ft of water
    # north of the course, >=60 ft west of the channel and >=20 ft south (photo cropped). The east
    # edge leaves room for the marina.
    lake_x_ft=[-190.0, 160.0],
    lake_y_ft=[-100.0, 100.0],
    shore_margin_m=1.2,
    geo_origin=dict(lat=41.70238808, lon=-85.02280627,
                    note="Google Earth view centre from the PDF, assigned to local (0, 0). NOT surveyed."),
    # Boat slot beside the north face of the marina's east-west pier, bow west towards the course.
    start_pose_local=[100.0 * FT, 14.5 * FT, math.pi],
)


def marina(start_ft):
    """Marina from the PDF satellite map (L-shaped pier east of the start buoy), as keepout boxes in
    feet relative to the start buoy: a north-south pier with slips on both sides running to the north
    shore, and an east-west pier with four boats moored on its south side (the return buoy lies about
    8 ft south-west of them) and a pontoon boat at its west end."""
    sx, sy = start_ft
    items = []

    def box(iid, style, x0, x1, y0, y1, height):
        items.append(dict(id=iid, style=style, height_m=height,
                          polygon=[[round((sx + x) * FT, 3), round((sy + y) * FT, 3)] for x, y in
                                   ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]))

    box("pier_ns", "pier", 16, 22, -25, 68, 0.5)
    box("pier_ew", "pier", -27, 40, -33, -25, 0.5)
    for i, y in enumerate(range(-12, 59, 10)):                      # fingers on both sides of pier_ns
        box(f"finger_w{i}", "pier", 4, 16, y - 0.75, y + 0.75, 0.45)
        box(f"finger_e{i}", "pier", 22, 34, y - 0.75, y + 0.75, 0.45)
    for i, y in enumerate(range(-7, 54, 10)):
        box(f"boat_w{i}", "boat", 3, 15.5, y - 2.5, y + 2.5, 0.9)
        box(f"boat_e{i}", "boat", 22.5, 36, y - 2.5, y + 2.5, 0.9)
    for i, x in enumerate((-12, 4, 20)):                            # fingers south of pier_ew
        box(f"finger_s{i}", "pier", x - 0.75, x + 0.75, -52, -33, 0.45)
    for i, x in enumerate((-20, -4, 12, 28)):
        box(f"boat_s{i}", "boat", x - 4.5, x + 4.5, -52, -33.5, 1.0)
    box("pontoon_w", "boat", -38, -27, -42, -23, 1.1)
    return items


def build(p=PARAMS):
    ax, ay = p["anchor_ft"]
    r = p["buoy_diameter_m"] / 2
    h = p["buoy_height_above_water_m"]
    objs = []

    def add(oid, kind, color, dx, dy, **extra):
        o = dict(id=oid, kind=kind, color=color, x=round((ax + dx) * FT, 4), y=round((ay + dy) * FT, 4))
        if kind == "buoy":
            o.update(radius=r, height=h)
        o.update(extra)
        objs.append(o)

    g = p["gate_width_ft"] / 2
    add("start_blue", "buoy", "blue", 0, 0, role="start_marker")
    add("gateA_red", "buoy", "red", -45, +g, role="gate", group="gateA")
    add("gateA_green", "buoy", "green", -45, -g, role="gate", group="gateA")
    for i, (c, dx) in enumerate([("red", -75), ("green", -105), ("red", -135), ("green", -165)], 1):
        add(f"slalom{i}_{c}", "buoy", c, dx, 0, role="slalom", group="slalom")
    add("gateB_red", "buoy", "red", -195, +g, role="gate", group="gateB")
    add("gateB_green", "buoy", "green", -195, -g, role="gate", group="gateB")
    # Channel (C3): west bank reds, east bank greens, boat travels south.
    add("ch_green_top", "buoy", "green", -210, 0, role="channel", group="channel_east")
    add("ch_green_mid", "buoy", "green", -210, -30, role="channel", group="channel_east")
    add("ch_green_bot", "buoy", "green", -210, -60, role="channel", group="channel_east")
    add("ch_red_top", "buoy", "red", -250, 0, role="channel", group="channel_west")
    add("ch_red_mid", "buoy", "red", -250, -30, role="channel", group="channel_west")
    add("ch_red_bot", "buoy", "red", -250, -60, role="channel", group="channel_west")
    add("detector_east", "detector", "pink", -210, -14, radius=0.15, height=0.9,
        detect_radius=p["detector_radius_m"], role="detector")
    add("detector_west", "detector", "pink", -250, -45, radius=0.15, height=0.9,
        detect_radius=p["detector_radius_m"], role="detector")
    # Identify cluster (C4)
    sp = p["identify_spacing_ft"]
    for i, c in enumerate(["blue", "orange", "purple", "yellow"]):              # west to east, as drawn
        add(f"identify_{c}", "buoy", c, p["identify_center_dx_ft"] + (i - 1.5) * sp, -60 + p["identify_row_offset_ft"],
            role="identify")
    add("zebra", "buoy", "zebra", -145, -60, role="deploy")
    add("black", "buoy", "black", -95, -60, role="launch_area")
    add("launch_target", "target", "orange", -95, -60 + p["launch_target_offset_ft"],
        radius=p["launch_target_diameter_ft"] * FT / 2, height=0.2, mat_radius=4.0 * FT, role="launch_target")
    case = [round(v * 0.0254, 3) for v in p["case_size_in"]]
    add("case", "case", "yellow", -45, -60, size=case, radius=round(math.hypot(case[0], case[1]) / 2, 3),
        height=case[2], role="recover")
    add("return_blue", "buoy", "blue", p["return_dx_ft"], -60, role="return")

    ids = {o["id"]: o for o in objs}
    m = p["shore_margin_m"]
    (x0, x1), (y0, y1) = p["lake_x_ft"], p["lake_y_ft"]
    course = dict(
        schema="boatlab_course_v1",
        name="aimm_icc_2025",
        source="boat_course/AIMM-ICC Competition Course Details 12-13 2025.pdf (page 2 map, pages 5-14 rules)",
        units="metres, radians; local ENU x=east y=north; heading CCW from east",
        unreal_mapping=dict(ue_x_cm="100*x", ue_y_cm="-100*y", ue_yaw_deg="-degrees(heading)", water_z_cm=0.0),
        geo_origin=p["geo_origin"],
        params=p,
        water=dict(xmin=round(x0 * FT, 4), xmax=round(x1 * FT, 4), ymin=round(y0 * FT, 4), ymax=round(y1 * FT, 4), margin=m),
        # Marina piers and moored boats (unreal/Content/Python/boatlab_course.py builds them in the level).
        keepout=marina(p["anchor_ft"]),
        start_pose=p["start_pose_local"],
        dock_pose=p["start_pose_local"],
        objects=objs,
        gates=[dict(id="gateA", red="gateA_red", green="gateA_green"),
               dict(id="gateB", red="gateB_red", green="gateB_green"),
               dict(id="channel_entry", red="ch_red_top", green="ch_green_top"),
               dict(id="channel_exit", red="ch_red_bot", green="ch_green_bot")],
        challenges={
            "1_gate": dict(gate="gateA"),
            "2_dodge": dict(entry_gate="gateA", exit_gate="gateB",
                            buoys=[o["id"] for o in objs if o.get("role") == "slalom"]),
            "3_evade": dict(entry_gate="channel_entry", exit_gate="channel_exit",
                            banks=[o["id"] for o in objs if o.get("role") == "channel"],
                            detectors=["detector_east", "detector_west"]),
            "4_identify": dict(candidates=[o["id"] for o in objs if o.get("role") == "identify"], default_color="blue"),
            "5_deploy": dict(target="zebra", success_radius_m=6 * FT),
            "6_launch": dict(area_buoy="black", target="launch_target"),
            "7_recover": dict(target="case"),
            "9_return": dict(target="return_blue"),
        },
        rules=dict(green_on="port", red_on="starboard",
                   note="PDF p5: pass on the RIGHT side of green buoys and LEFT side of red buoys, "
                        "i.e. green stays on the boat's port side, red on the starboard side."),
    )
    assert all(ids[g[c]] for g in course["gates"] for c in ("red", "green"))
    return course


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=ROOT / "config" / "course_aimm_2025.json")
    a = ap.parse_args()
    course = build()
    a.out.write_text(json.dumps(course, indent=2) + "\n")
    print(f"wrote {a.out} ({len(course['objects'])} objects)")


if __name__ == "__main__":
    main()
