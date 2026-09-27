"""Update course object positions from a GPS survey (real lake).

CSV columns: id,lat,lon   (id must match config/course_aimm_2025.json, e.g. gateA_red, zebra)
Optional first row id=origin sets the local frame origin (e.g. the end of the dock).

    python tools/survey_to_course.py survey.csv --out config/course_lake.json
Then run missions with:  python -m boatnav.runner --backend real --course course_lake.json ...
Buoys you do not survey keep their map-derived positions relative to the new origin - check them.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from boatnav.config import load_course  # noqa: E402
from boatnav.geo import LocalFrame  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--course", default="course_aimm_2025.json")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    course = load_course(a.course)
    rows = list(csv.DictReader(open(a.csv, newline="")))
    origin = next((r for r in rows if r["id"] == "origin"), None)
    if origin:
        course["geo_origin"] = {"lat": float(origin["lat"]), "lon": float(origin["lon"]), "note": "surveyed origin"}
    fr = LocalFrame.from_course(course)
    objs = {o["id"]: o for o in course["objects"]}
    for r in rows:
        if r["id"] == "origin":
            continue
        if r["id"] not in objs:
            print(f"skip unknown id {r['id']}")
            continue
        x, y = fr.to_local(float(r["lat"]), float(r["lon"]))
        objs[r["id"]].update(x=round(x, 3), y=round(y, 3), surveyed=True)
        print(f"{r['id']:16s} -> x {x:8.2f}  y {y:8.2f}")
    Path(a.out if Path(a.out).is_absolute() else ROOT / a.out).write_text(json.dumps(course, indent=2))
    print("written", a.out)


if __name__ == "__main__":
    main()
