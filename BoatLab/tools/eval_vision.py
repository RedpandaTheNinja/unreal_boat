"""Measure detector precision/recall and localisation error against course truth.

Twin frames (default): renders N random boat poses on the course, optionally through JPEG
compression like the Unreal link, and scores every detection:
    python tools/eval_vision.py --frames 150 --jpeg 90

Recorded frames: pass a folder written by tools/tune_colors.py --save (frame_*.jpg + frame_*.json
with the boat pose) to score Unreal or real-boat images the same way:
    python tools/eval_vision.py --folder runs/frames_unreal
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from boatnav.config import load_boat, load_course, load_vision  # noqa: E402
from boatnav.hal.render import Renderer  # noqa: E402
from boatnav.perception.detector import ColorBlobDetector  # noqa: E402
from boatnav.perception.mapping import label_for  # noqa: E402
from boatnav.types import CameraFrame  # noqa: E402


def frames_twin(boat, course, n, jpeg, seed):
    rng = np.random.default_rng(seed)
    r = Renderer(boat["camera"], course, seed=seed)
    w = course["water"]
    for _ in range(n):
        pose = (rng.uniform(w["xmin"] + 3, w["xmax"] - 6), rng.uniform(w["ymin"] + 2, w["ymax"] - 2), rng.uniform(-math.pi, math.pi))
        rgb, depth = r(pose)
        if jpeg:
            ok, buf = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, jpeg])
            rgb = cv2.cvtColor(cv2.imdecode(buf, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
            d = cv2.resize(depth, (320, 240), interpolation=cv2.INTER_NEAREST)
            depth = cv2.resize(np.round(d * 1000) / 1000, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
        yield CameraFrame(0, rgb, depth, r.fx, r.fy, r.cx, r.cy, tuple(boat["camera"]["mount_flu_m"]), r.pitch, pose), pose


def frames_folder(boat, folder):
    for js in sorted(Path(folder).glob("frame_*.json")):
        meta = json.loads(js.read_text())
        bgr = cv2.imread(str(js.with_suffix(".jpg")))
        dpath = js.with_name(js.stem + "_depth.png")
        depth = None
        if dpath.exists():
            d16 = cv2.imread(str(dpath), cv2.IMREAD_UNCHANGED)
            depth = cv2.resize(d16.astype(np.float32) / 1000.0, (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        H, W = rgb.shape[:2]
        f = W / (2 * math.tan(math.radians(meta["hfov_deg"]) / 2))
        pose = tuple(meta["pose"])
        yield CameraFrame(0, rgb, depth, f, f, W / 2, H / 2, tuple(meta["mount_flu_m"]), math.radians(meta["pitch_down_deg"]),
                          pose), pose


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--frames", type=int, default=120)
    ap.add_argument("--jpeg", type=int, default=90, help="0 = no compression")
    ap.add_argument("--folder")
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--match-m", type=float, default=1.2)
    ap.add_argument("--vision", default="vision_colors.json", help="colour file to score (e.g. vision_colors_unreal.json)")
    a = ap.parse_args()
    boat, course, vision = load_boat(), load_course(), load_vision(a.vision)
    det = ColorBlobDetector(vision, boat["camera"])
    objs = [(label_for(o), o) for o in course["objects"]]
    stats = defaultdict(lambda: {"tp": 0, "fp": 0, "err": []})
    src = frames_folder(boat, a.folder) if a.folder else frames_twin(boat, course, a.frames, a.jpeg, a.seed)
    nframes = 0
    for frame, pose in src:
        nframes += 1
        for d in det.detect(frame):
            best = min((math.hypot(o["x"] - d.x, o["y"] - d.y), o) for lab, o in objs if lab == d.label) if any(lab == d.label for lab, _ in objs) else (1e9, None)
            s = stats[d.label]
            if best[0] <= a.match_m + (1.6 if d.label == "target" else 0):
                s["tp"] += 1
                s["err"].append(best[0])
            else:
                s["fp"] += 1
    print(f"{nframes} frames")
    print(f"{'label':8s} {'TP':>5s} {'FP':>5s} {'precision':>9s} {'median err m':>12s} {'p90 err m':>9s}")
    for lab, s in sorted(stats.items()):
        n = s["tp"] + s["fp"]
        e = np.array(s["err"]) if s["err"] else np.array([np.nan])
        print(f"{lab:8s} {s['tp']:5d} {s['fp']:5d} {s['tp'] / max(n, 1):9.2f} {np.nanmedian(e):12.2f} {np.nanpercentile(e, 90):9.2f}")


if __name__ == "__main__":
    main()
