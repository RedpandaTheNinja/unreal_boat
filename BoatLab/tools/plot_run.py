"""Plot a mission run log (runs/*.jsonl): course, planned paths, driven track, actions.

    python tools/plot_run.py                 # latest run
    python tools/plot_run.py runs/<file>.jsonl --zoom channel
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from boatnav.config import load_course  # noqa: E402

COL = {"red": "#d62728", "green": "#2ca02c", "blue": "#1f77b4", "orange": "#ff7f0e", "purple": "#9467bd",
       "yellow": "#e7c31a", "black": "#111111", "zebra": "#bbbbbb", "pink": "#ff69b4"}
ZOOMS = {"gate": (12, 40, 3, 17), "slalom": (-32, 28, 3, 17), "channel": (-46, -22, -14, 17),
         "bottom": (-35, 36, -16, 0)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", nargs="?")
    ap.add_argument("--course", default="course_aimm_2025.json")
    ap.add_argument("--zoom", choices=list(ZOOMS))
    ap.add_argument("--out")
    a = ap.parse_args()
    log = Path(a.log) if a.log else sorted((ROOT / "runs").glob("*.jsonl"))[-1]
    rows = [json.loads(l) for l in log.read_text().splitlines() if l.strip()]
    course = load_course(a.course)
    fig, ax = plt.subplots(figsize=(16, 7.5))
    w = course["water"]
    ax.add_patch(plt.Rectangle((w["xmin"], w["ymin"]), w["xmax"] - w["xmin"], w["ymax"] - w["ymin"], fc="#dcecf2", ec="#5a7d8a"))
    for k in course.get("keepout", []):
        xs, ys = zip(*k["polygon"])
        ax.fill(xs, ys, fc="#8b6b4a", alpha=.7)
    for o in course["objects"]:
        c = COL.get(o["color"], "#888")
        if o["kind"] == "target":
            ax.add_patch(plt.Circle((o["x"], o["y"]), o["radius"], fc="#ffb27a", ec="k"))
        elif o["kind"] == "detector":
            ax.add_patch(plt.Circle((o["x"], o["y"]), o.get("detect_radius", 6), fc="#ff69b422", ec="#ff69b4", ls="--"))
            ax.plot(o["x"], o["y"], "D", c=c)
        elif o["kind"] == "case":
            ax.plot(o["x"], o["y"], "s", c=c, ms=8, mec="k")
        else:
            ax.plot(o["x"], o["y"], "o", c=c, ms=7, mec="k")
    for r in rows:
        if "path" in r:
            xs, ys = zip(*r["path"]["points"]) if r["path"]["points"] else ([], [])
            ax.plot(xs, ys, "-", c="#7a7a7a", lw=.8, alpha=.7)
    key = "truth" if "truth" in rows[0] else None
    tx = [(r[key] if key else r)["x"] for r in rows]
    ty = [(r[key] if key else r)["y"] for r in rows]
    ax.plot(tx, ty, "-", c="#0b3d91", lw=1.6, label="driven (truth)" if key else "driven")
    for r in rows:
        if "approach" in r and r["approach"]:
            px, py, _ = r["approach"]["pose"]
            ax.plot(px, py, "x", c="m", ms=9)
    summ = log.with_name(log.stem + "_summary.json")
    if summ.exists():
        s = json.loads(summ.read_text())
        for act in s.get("actions", []):
            if "landing" in act:
                ax.plot(*act["landing"], "*", c="gold", ms=14, mec="k")
        sc = s.get("score") or {}
        ax.set_title(f"{log.name}  passed {sc.get('passed_count')}  sim {s.get('sim_time_s')} s")
    ax.set_aspect("equal")
    if a.zoom:
        x0, x1, y0, y1 = ZOOMS[a.zoom]
        ax.set_xlim(x0, x1)
        ax.set_ylim(y0, y1)
    ax.grid(alpha=.3)
    ax.legend(loc="lower left")
    out = Path(a.out) if a.out else log.with_suffix(f".{a.zoom or 'full'}.png")
    fig.savefig(out, dpi=110, bbox_inches="tight")
    print(out)


if __name__ == "__main__":
    main()
