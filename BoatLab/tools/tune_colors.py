"""Capture camera frames, sample the HSV of every course object where it should appear, and
propose colour thresholds for config/vision_colors.json.

Unreal lighting, exposure, fog and real cameras all shift colours, so re-tune per environment:
    python tools/tune_colors.py --backend unreal --frames 25 --save runs/frames_unreal
    python tools/tune_colors.py --backend unreal --frames 25 --apply        # write the proposal
    python tools/tune_colors.py --backend real --frames 15 --save runs/frames_lake   (boat at the course)

How it works: the boat pose (sim truth or GPS) and the course positions give where each buoy
should be in the image (pinhole projection). A small window around the projected body centre is
sampled; per colour the 5th-95th percentile H/S/V becomes the proposed range. Frames saved with
--save can be scored with tools/eval_vision.py --folder.
Drive the boat around between captures (dashboard waypoints or keyboard in Unreal) to see buoys
from several ranges and angles.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import sys
import time
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from boatnav.config import CONFIG, load_boat, load_course, load_vision  # noqa: E402
from boatnav.hal import make_backend  # noqa: E402
from boatnav.perception.detector import ColorBlobDetector  # noqa: E402


def project(frame, x, y, z):
    """World point -> pixel using the frame's pose, mount and pitch."""
    px, py, h = frame.pose
    c, s = math.cos(h), math.sin(h)
    mx, my, mz = frame.mount_flu
    ox, oy = px + c * mx - s * my, py + s * mx + c * my
    oz = 0.125 + mz
    dx, dy, dz = x - ox, y - oy, z - oz
    bx, by = c * dx + s * dy, -s * dx + c * dy
    cp, sp = math.cos(frame.pitch_down_rad), math.sin(frame.pitch_down_rad)
    zc = cp * bx - sp * dz
    up = sp * bx + cp * dz
    if zc < 0.5:
        return None
    return frame.cx + frame.fx * (-by) / zc, frame.cy + frame.fy * (-up) / zc, zc


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", default="unreal", choices=["twin", "unreal", "real"])
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--max-range", type=float, default=25.0)
    ap.add_argument("--save", help="folder for frame_*.jpg/json (+ depth png)")
    ap.add_argument("--vision", default="vision_colors.json",
                    help="colour file to start from and --apply to (config/vision_colors_unreal.json for Unreal)")
    ap.add_argument("--apply", action="store_true", help="write the proposal to the --vision file (backup kept)")
    a = ap.parse_args()
    boat, course, vision = load_boat(), load_course(), load_vision(a.vision)
    hal = make_backend(a.backend, boat, course, **({"camera": True} if a.backend != "real" else {"dry_run": True}))
    det = ColorBlobDetector(vision, boat["camera"])
    samples = defaultdict(list)
    out = Path(a.save) if a.save else None
    if out:
        out.mkdir(parents=True, exist_ok=True)
    got = 0
    t_end = time.monotonic() + a.frames * max(a.interval, 0.2) * 4 + 20
    while got < a.frames and time.monotonic() < t_end:
        if a.backend == "twin":
            hal.tick(a.interval)
        fr = hal.camera()
        if fr is None:
            time.sleep(0.2)
            continue
        got += 1
        hsv = cv2.cvtColor(fr.rgb, cv2.COLOR_RGB2HSV)
        H, W = hsv.shape[:2]
        for o in course["objects"]:
            if o["kind"] not in ("buoy", "case", "detector", "target"):
                continue
            z = 0.2 if o["kind"] == "buoy" else 0.08
            p = project(fr, o["x"], o["y"], z)
            if p is None or p[2] > a.max_range:
                continue
            u, v, zc = p
            half = max(1, int(fr.fx * o.get("radius", 0.13) * 0.5 / zc))
            if not (half <= u < W - half and half <= v < H - half):
                continue
            win = hsv[int(v) - half:int(v) + half + 1, int(u) - half:int(u) + half + 1].reshape(-1, 3)
            key = {"case": "yellow", "target": "orange", "detector": "pink"}.get(o["kind"], o["color"])
            if key == "zebra":
                win = win[win[:, 2] > 110]          # white parts only
            samples[key].append(win)
        if out:
            stem = out / f"frame_{got:04d}"
            cv2.imwrite(str(stem.with_suffix(".jpg")), cv2.cvtColor(fr.rgb, cv2.COLOR_RGB2BGR))
            if fr.depth is not None:
                cv2.imwrite(str(stem) + "_depth.png", np.clip(fr.depth * 1000, 0, 65535).astype(np.uint16))
            stem.with_suffix(".json").write_text(json.dumps({"pose": list(fr.pose), "mount_flu_m": list(fr.mount_flu),
                                                             "pitch_down_deg": math.degrees(fr.pitch_down_rad),
                                                             "hfov_deg": math.degrees(2 * math.atan(W / 2 / fr.fx)),
                                                             "source": fr.source}))
        dets = det.detect(fr)
        print(f"frame {got}: {len(dets)} detections: " + ", ".join(f"{d.label}@{d.range_m:.0f}m" for d in dets[:8]), flush=True)
        if a.backend != "twin":
            time.sleep(a.interval)
    hal.close()
    proposal = copy.deepcopy(vision)
    print("\ncolour   samples   H p5-p95      S p5-p95     V p5-p95")
    for key, wins in sorted(samples.items()):
        px = np.concatenate(wins)
        if len(px) < 30 or key not in proposal["colors"]:
            continue
        hh, ss, vv = px[:, 0].astype(float), px[:, 1], px[:, 2]
        if key == "red":                                   # red wraps around 0/180
            hh = np.where(hh > 90, hh - 180, hh)
        h5, h95 = np.percentile(hh, [5, 95])
        s5, v5 = np.percentile(ss, 5), np.percentile(vv, 5)
        s95, v95 = np.percentile(ss, 95), np.percentile(vv, 95)
        print(f"{key:8s} {len(px):7d}   {h5:5.0f}-{h95:<5.0f}    {s5:4.0f}-{s95:<4.0f}    {v5:4.0f}-{v95:<4.0f}")
        spec = proposal["colors"][key]
        if key in ("black",):
            spec["v"] = [0, int(min(90, v95 + 10))]
        elif key == "zebra":
            spec["s"] = [0, int(min(80, s95 + 10))]
            spec["v"] = [int(max(80, v5 - 15)), 255]
        else:
            lo, hi = h5 - 4, h95 + 4
            spec["h"] = [[0, int(hi)], [int(180 + lo), 180]] if (key == "red" and lo < 0) else [[int(max(0, lo)), int(min(180, hi))]]
            spec["s"] = [int(max(40, s5 - 20)), 255]
            spec["v"] = [int(max(30, v5 - 20)), 255]
    path = CONFIG / Path(a.vision).name
    prop = path.with_name(path.stem + ".proposed.json")
    prop.write_text(json.dumps(proposal, indent=1))
    print(f"\nproposal written to {prop}")
    if a.apply:
        backup = path.with_name(f"{path.stem}.backup_{time.strftime('%Y%m%d_%H%M%S')}.json")
        backup.write_text(path.read_text())
        path.write_text(json.dumps(proposal, indent=1))
        print(f"applied to {path} (backup {backup.name})")


if __name__ == "__main__":
    main()
