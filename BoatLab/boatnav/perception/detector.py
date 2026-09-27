"""Colour-blob buoy detector with metric localisation.

Pipeline per frame:
  RGB -> HSV -> per-colour masks -> morphology -> connected components
  -> shape split (e.g. tall orange = buoy, wide orange = launch target)
  -> range from depth (ZED2i / Unreal depth) else from the waterline row on flat water,
     else from apparent height -> bearing -> world (x, y) using the boat pose.

Runs on the Jetson unchanged (OpenCV). For the real course you will likely want a
trained detector (YOLO etc.); implement the same detect(frame) -> [Detection] method.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from .. import geometry as G
from ..types import CameraFrame, Detection

HULL_ORIGIN_Z = 0.125


class ColorBlobDetector:
    def __init__(self, vision_cfg: dict, cam_cfg: dict):
        self.v = vision_cfg
        self.cam = cam_cfg
        self.cam_height = cam_cfg.get("height_above_water_m", HULL_ORIGIN_Z + cam_cfg["mount_flu_m"][2])
        self.kernel = np.ones((3, 3), np.uint8)

    # ------------------------------------------------------------ geometry
    @staticmethod
    def ray_body(f: CameraFrame, u: float, v: float) -> np.ndarray:
        """Unit-optical-z ray for pixel (u, v) in body FLU."""
        xn, yn = (u - f.cx) / f.fx, (v - f.cy) / f.fy
        cp, sp = math.cos(f.pitch_down_rad), math.sin(f.pitch_down_rad)
        fwd = np.array([cp, 0.0, -sp])
        up = np.array([sp, 0.0, cp])
        left = np.array([0.0, 1.0, 0.0])
        return fwd - xn * left - yn * up

    def horizon_row(self, f: CameraFrame) -> float:
        return f.cy - f.fy * math.tan(f.pitch_down_rad)

    def _locate(self, f: CameraFrame, u_c, v_c, v_bottom, h_px, depth_med, kind):
        """Returns (bearing, horizontal range to object centre, source, body_x, body_y)."""
        mx, my, mz = f.mount_flu
        radius = self.v["object_radii_m"].get(kind, 0.15)
        cam_h = self.cam_height
        if depth_med is not None and depth_med > 0.2:
            ray = self.ray_body(f, u_c, v_c) * depth_med
            src = "depth"
            bx, by = mx + ray[0], my + ray[1]
            # depth hits the near surface; push to the centre along the horizontal ray
            n = math.hypot(ray[0], ray[1]) or 1.0
            bx += ray[0] / n * radius * 0.8
            by += ray[1] / n * radius * 0.8
        else:
            ray = self.ray_body(f, u_c, v_bottom)
            if ray[2] < -1e-3:
                t = cam_h / -ray[2]
                bx, by = mx + ray[0] * t, my + ray[1] * t
                n = math.hypot(ray[0], ray[1]) or 1.0
                bx += ray[0] / n * radius
                by += ray[1] / n * radius
                src = "waterline"
            else:
                H = self.v["object_heights_m"].get(kind, 0.55)
                z = f.fy * H / max(h_px, 1.0)
                ray = self.ray_body(f, u_c, v_c) * z
                bx, by = mx + ray[0], my + ray[1]
                src = "size"
        return math.atan2(by, bx), math.hypot(bx, by), src, bx, by

    def _locate_flat(self, f: CameraFrame, blob_mask, x0, y0):
        """Large round target (launch trampoline, radius R known): the two outermost blob columns are
        tangent rays to the rim, so the centre lies on their bisector at D = R / sin(half angle).
        Exact for a circle from any viewpoint; falls back to depth when the blob is cut by the image edge."""
        h, w = blob_mask.shape
        cols = np.nonzero(blob_mask.any(axis=0))[0]
        if len(cols) < 3:
            return 0, 0, None, 0, 0
        R = self.v["object_radii_m"].get("target", 1.5)
        W = f.rgb.shape[1]
        if x0 + cols[0] <= 1 or x0 + cols[-1] >= W - 2:
            vs, us = np.nonzero(blob_mask)
            d = f.depth[vs + y0, us + x0]
            ok = (d > 0.2) & np.isfinite(d)
            if ok.sum() < 5:
                return 0, 0, None, 0, 0
            ray = self.ray_body(f, float(np.median(us[ok] + x0)), float(np.median(vs[ok] + y0))) * float(np.median(d[ok]))
            bx, by = f.mount_flu[0] + ray[0], f.mount_flu[1] + ray[1]
            n = math.hypot(bx, by) or 1.0
            bx, by = bx + bx / n * R * 0.8, by + by / n * R * 0.8
            return math.atan2(by, bx), math.hypot(bx, by), "depth_clipped", bx, by
        angs = []
        for c in (cols[0], cols[-1]):
            rows = np.nonzero(blob_mask[:, c])[0]
            ray = self.ray_body(f, x0 + c + (0.0 if c == cols[0] else 1.0), y0 + float(np.median(rows)))
            angs.append(math.atan2(ray[1], ray[0]))
        half = abs(G.wrap(angs[0] - angs[1])) / 2
        if half < 1e-3:
            return 0, 0, None, 0, 0
        D = R / math.sin(half)
        brg = angs[1] + G.wrap(angs[0] - angs[1]) / 2
        bx, by = f.mount_flu[0] + D * math.cos(brg), f.mount_flu[1] + D * math.sin(brg)
        return math.atan2(by, bx), math.hypot(bx, by), "rim_tangent", bx, by

    # ------------------------------------------------------------ detection
    def detect(self, f: CameraFrame) -> list[Detection]:
        rgb = f.rgb
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        H, S, V = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        v_h = self.horizon_row(f) + self.v.get("horizon_margin_px", 1)
        below = np.zeros(H.shape, bool)
        below[int(max(0, math.ceil(v_h))):, :] = True
        out: list[Detection] = []
        min_area = self.v.get("min_area_px", 14)
        for cname, spec in self.v["colors"].items():
            hm = np.zeros(H.shape, bool)
            for lo, hi in spec["h"]:
                hm |= (H >= lo) & (H <= hi)
            m = hm & (S >= spec["s"][0]) & (S <= spec["s"][1]) & (V >= spec["v"][0]) & (V <= spec["v"][1]) & below
            mask = m.astype(np.uint8) * 255
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            if "merge_dark_v" in spec:
                # zebra: black tape inside/next to white regions belongs to the same buoy
                dark = ((V <= spec["merge_dark_v"]) & below).astype(np.uint8) * 255
                near = cv2.dilate(mask, np.ones((9, 5), np.uint8))
                mask = cv2.bitwise_or(mask, cv2.bitwise_and(dark, near))
            if "close_kernel" in spec:
                kw, kh = spec["close_kernel"]
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((kh, kw), np.uint8))
            n, lab, stats, cent = cv2.connectedComponentsWithStats(mask, connectivity=8)
            for i in range(1, n):
                x0, y0, w, h, area = stats[i]
                if area < min_area:
                    continue
                aspect = h / max(w, 1)
                at_border = x0 <= 1 or x0 + w >= f.rgb.shape[1] - 1
                if at_border and spec.get("allow_border") is not True:
                    continue                      # cut by the image edge: bearing/size unreliable
                label, kind = None, None
                for cl in spec["classes"]:
                    if at_border and cl["kind"] != "target":
                        continue                  # a clipped blob's aspect ratio is meaningless
                    if aspect < cl.get("aspect_min", 0) or aspect > cl.get("aspect_max", 1e9):
                        continue
                    label, kind = cl["label"], cl["kind"]
                    break
                if label is None:
                    continue
                u_c = x0 + w / 2
                v_c = y0 + h / 2
                v_b = y0 + h
                dmed = None
                if f.depth is not None:
                    dd = f.depth[y0:y0 + h, x0:x0 + w][lab[y0:y0 + h, x0:x0 + w] == i]
                    dd = dd[(dd > 0.2) & np.isfinite(dd)]
                    if dd.size:
                        dmed = float(np.median(dd))
                if kind == "target":
                    bearing, rng, src, bx, by = self._locate_flat(f, lab[y0:y0 + h, x0:x0 + w] == i, x0, y0)
                else:
                    bearing, rng, src, bx, by = self._locate(f, u_c, v_c, v_b, h, dmed, kind)
                if src is None:
                    continue
                if rng > self.v.get("max_range_m", 40.0) or rng < 0.3:
                    continue
                if kind in ("buoy", "detector"):
                    # size consistency: a real buoy at this range has a predictable pixel height
                    z_axis = max(0.5, rng * math.cos(bearing))
                    expect = f.fy * self.v["object_heights_m"].get(kind, 0.55) / z_axis
                    lo, hi = self.v.get("size_ratio", [0.35, 2.8])
                    if not lo <= h / max(expect, 1e-3) <= hi or area / float(w * h) < self.v.get("min_fill", 0.45):
                        continue
                wx, wy = G.body_to_world(*f.pose, bx, by)
                out.append(Detection(label, u_c, v_b, (int(x0), int(y0), int(x0 + w), int(y0 + h)), bearing, rng, src,
                                     wx, wy, float(min(1.0, area / 400.0)), int(area)))
        return out


def annotate(rgb: np.ndarray, dets: list[Detection], horizon_row: float | None = None) -> np.ndarray:
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if horizon_row is not None:
        cv2.line(img, (0, int(horizon_row)), (img.shape[1], int(horizon_row)), (200, 200, 200), 1)
    for d in dets:
        x0, y0, x1, y1 = d.bbox
        cv2.rectangle(img, (x0, y0), (x1, y1), (255, 255, 255), 1)
        cv2.putText(img, f"{d.label} {d.range_m:.1f}m", (x0, max(10, y0 - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                    (255, 255, 255), 1, cv2.LINE_AA)
    return img  # BGR for cv2.imencode
