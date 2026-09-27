"""
sketch_to_heightmap.py
======================
Turns words or a drawing into the two artifacts Unreal actually needs:

    1. a 16-bit greyscale heightmap PNG   (terrain)
    2. an ordered centerline polyline     (track / route)

Runs *outside* Unreal - plain numpy + Pillow. Claude calls this while writing the
scene spec, then the UE side only has to consume finished data. Keeping the
image work out of the editor is deliberate: the editor's Python is single
threaded and blocking it stalls the whole UI.

CLI
---
    python sketch_to_heightmap.py procedural  --out hm.png --size 1009 \
        --roughness-m 0.15 --feature-size-m 8 --world-m 40 --seed 7

    python sketch_to_heightmap.py from-image  --image sketch.png --out hm.png \
        --size 1009 --smooth 6 --invert

    python sketch_to_heightmap.py trace-track --image sketch.png --world-m 40 \
        --out-json track.json --simplify-m 0.35
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, asdict

import numpy as np
from PIL import Image, ImageFilter

# --------------------------------------------------------------------------
# heightmap generation
# --------------------------------------------------------------------------

U16_MAX = 65535


def _value_noise(shape: tuple[int, int], cells: int, rng: np.random.Generator) -> np.ndarray:
    """Smooth value noise on a `cells x cells` lattice, bicubically upsampled."""
    cells = max(2, int(cells))
    lattice = rng.random((cells + 1, cells + 1)).astype(np.float32)
    img = Image.fromarray((lattice * 255).astype(np.uint8), mode="L")
    img = img.resize((shape[1], shape[0]), resample=Image.BICUBIC)
    return np.asarray(img, dtype=np.float32) / 255.0


def fbm(shape: tuple[int, int], octaves: int, base_cells: float,
        seed: int = 0, persistence: float = 0.5) -> np.ndarray:
    """Fractal Brownian motion in [0, 1]."""
    rng = np.random.default_rng(seed)
    total = np.zeros(shape, dtype=np.float32)
    amp, norm, cells = 1.0, 0.0, base_cells
    for _ in range(max(1, octaves)):
        total += amp * _value_noise(shape, cells, rng)
        norm += amp
        amp *= persistence
        cells *= 2
    total /= max(norm, 1e-6)
    lo, hi = float(total.min()), float(total.max())
    return (total - lo) / max(hi - lo, 1e-6)


def clamp_slope(height_m: np.ndarray, px_per_m: float, slope_max_deg: float,
                iterations: int = 40) -> np.ndarray:
    """
    Flatten anything steeper than `slope_max_deg`.

    Matters more than it looks: a wheeled robot that can physically climb 15 deg
    will spend the first 200k steps of training stuck on a 40 deg noise spike,
    and you will blame the reward function.

    Implementation is a cone-limit sweep: enforce h(x) <= h(y) + max_step for
    every neighbour pair. It only ever lowers samples, so it is monotone and
    cannot diverge - and the one-sided bound over all ordered pairs gives the
    two-sided bound for free.

    Each axis pass is a min-plus scan done with `np.minimum.accumulate`, which
    propagates the constraint across the whole row in one go. The naive
    shift-and-min version needs one iteration per pixel of travel and is ~1000x
    slower on a 1k heightmap.
    """
    # The sweeps below constrain each axis independently, so a slope running
    # diagonally can reach sqrt(2) x the per-axis limit. Divide it out, or the
    # true gradient magnitude comes in ~35% over the number you asked for.
    max_step = (math.tan(math.radians(slope_max_deg))
                / math.sqrt(2.0) / max(px_per_m, 1e-6))
    h = height_m.astype(np.float32).copy()

    def sweep(a: np.ndarray, axis: int) -> np.ndarray:
        n = a.shape[axis]
        idx = (np.arange(n, dtype=np.float32) * max_step).reshape(
            (-1, 1) if axis == 0 else (1, -1))

        def forward(x):                       # min_{j<=i} ( x[j] + step*(i-j) )
            return np.minimum.accumulate(x - idx, axis=axis) + idx

        rev = np.flip(forward(np.flip(a, axis=axis)), axis=axis)
        return np.minimum(a, np.minimum(forward(a), rev))

    for _ in range(iterations):
        prev = h
        h = sweep(sweep(h, 0), 1)
        if float(np.max(prev - h)) < 1e-5:
            break
    return h


def _shape_to_roughness(h: np.ndarray, target_m: float, px_per_m: float,
                        slope_max_deg: float) -> np.ndarray:
    """Scale to a target RMS, slope-limit, and re-scale once to recover amplitude."""
    h = h - h.mean()
    h *= target_m / max(float(h.std()), 1e-6)
    h = clamp_slope(h, px_per_m, slope_max_deg)
    h = h - h.mean()
    got = max(float(h.std()), 1e-6)
    if got < target_m * 0.95:                     # slope limiting ate amplitude
        h *= min(target_m / got, 4.0)
        h = clamp_slope(h, px_per_m, slope_max_deg)
        h = h - h.mean()
    return h


def procedural_heightmap(size_px: int, world_m: float, roughness_m: float,
                         feature_size_m: float, seed: int = 0,
                         octaves: int = 4, slope_max_deg: float = 12.0,
                         flat_radius_m: float = 0.0,
                         flat_center_m: tuple[float, float] = (0.0, 0.0)
                         ) -> tuple[np.ndarray, float]:
    """
    Returns (uint16 heightmap, z_scale_m).

    `roughness_m` is the RMS height deviation in metres - the one terrain knob
    that maps directly onto vehicle dynamics, so it is the one worth
    randomizing over.
    """
    px_per_m = size_px / max(world_m, 1e-6)
    base_cells = max(2.0, world_m / max(feature_size_m, 0.1))
    n = fbm((size_px, size_px), octaves, base_cells, seed)

    h = _shape_to_roughness(n, roughness_m, px_per_m, slope_max_deg)

    if flat_radius_m > 0:
        ys, xs = np.mgrid[0:size_px, 0:size_px]
        cx = (flat_center_m[0] / world_m + 0.5) * size_px
        cy = (flat_center_m[1] / world_m + 0.5) * size_px
        r = np.hypot(xs - cx, ys - cy) / px_per_m
        blend = np.clip((r - flat_radius_m) / max(flat_radius_m, 1e-6), 0, 1)
        h *= blend ** 2                                    # flat pad for spawn
        # Re-clamp: multiplying by the blend ramp creates its own slope at the
        # edge of the pad, and that ring is exactly where the robot spawns.
        h = clamp_slope(h, px_per_m, slope_max_deg)

    lo, hi = float(h.min()), float(h.max())
    z_scale_m = max(hi - lo, 1e-3)
    u16 = ((h - lo) / z_scale_m * U16_MAX).astype(np.uint16)
    if float(h.std()) < roughness_m * 0.7:
        print(f"note: slope cap {slope_max_deg} deg limited roughness to "
              f"{h.std():.3f} m (asked {roughness_m} m). Raise feature_size_m "
              f"or slope_max_deg if you want it rougher.", file=__import__("sys").stderr)
    return u16, z_scale_m


def heightmap_from_image(path: str, size_px: int, world_m: float,
                         roughness_m: float, smooth_px: float = 4.0,
                         invert: bool = False, slope_max_deg: float = 12.0
                         ) -> tuple[np.ndarray, float]:
    """
    Reads a hand drawing / photo as elevation: bright = high (or the reverse
    with --invert). Blur is not cosmetic - pen strokes are 1 px cliffs and the
    physics solver hates them.
    """
    img = Image.open(path).convert("L").resize((size_px, size_px), Image.LANCZOS)
    if smooth_px > 0:
        img = img.filter(ImageFilter.GaussianBlur(radius=float(smooth_px)))
    a = np.asarray(img, dtype=np.float32) / 255.0
    if invert:
        a = 1.0 - a

    h = _shape_to_roughness(a, roughness_m, size_px / max(world_m, 1e-6), slope_max_deg)

    lo, hi = float(h.min()), float(h.max())
    z_scale_m = max(hi - lo, 1e-3)
    return ((h - lo) / z_scale_m * U16_MAX).astype(np.uint16), z_scale_m


def save_heightmap(u16: np.ndarray, out_path: str) -> None:
    """16-bit greyscale PNG - what UE's landscape importer expects."""
    Image.fromarray(u16.astype(np.uint16)).save(out_path)


# --------------------------------------------------------------------------
# heightmap -> OBJ  (the reliable terrain path, see README)
# --------------------------------------------------------------------------

def heightmap_to_obj(u16: np.ndarray, world_m: float, z_scale_m: float,
                     out_path: str, max_verts_per_side: int = 257) -> str:
    """
    Writes a triangulated terrain mesh in metres, Z-up, centred on the origin.

    Why a mesh and not a Landscape actor: Landscape creation from Python is
    fragile across UE versions, whereas `AssetImportTask` on an OBJ works
    headlessly and identically on 5.5 through 5.8, and gives you an asset you
    can version, diff and regenerate. Use a real Landscape for the hero level
    once the environment design has settled.
    """
    n = u16.shape[0]
    step = max(1, math.ceil(n / max(max_verts_per_side, 2)))
    g = u16[::step, ::step].astype(np.float32) / U16_MAX * z_scale_m
    m = g.shape[0]
    xs = np.linspace(-world_m / 2.0, world_m / 2.0, m)

    with open(out_path, "w") as f:
        f.write(f"# robotworld terrain {world_m}m z_scale={z_scale_m:.4f}m\n")
        for j in range(m):
            for i in range(m):
                f.write(f"v {xs[i]:.4f} {xs[j]:.4f} {g[j, i]:.4f}\n")
        inv = 1.0 / (m - 1)
        for j in range(m):
            for i in range(m):
                f.write(f"vt {i * inv:.5f} {j * inv:.5f}\n")
        for j in range(m - 1):
            for i in range(m - 1):
                a = j * m + i + 1
                b = a + 1
                c = a + m
                d = c + 1
                f.write(f"f {a}/{a} {c}/{c} {b}/{b}\n")
                f.write(f"f {b}/{b} {c}/{c} {d}/{d}\n")
    return out_path


# --------------------------------------------------------------------------
# sketch -> track centerline
# --------------------------------------------------------------------------

@dataclass
class Track:
    centerline_m: list[list[float]]
    closed: bool
    width_m: float
    length_m: float


def _stroke_width_px(mask: np.ndarray) -> float:
    """
    Estimate how thick the drawn line is: area / centreline-length, where
    centreline length is approximated as half the boundary pixel count.
    A marker on paper is 8-20 px in a phone photo; the caller needs this so its
    thinning cells are at least as wide as the stroke.
    """
    area = float(mask.sum())
    if area < 1:
        return 1.0
    interior = mask.copy()
    interior[1:, :] &= mask[:-1, :]
    interior[:-1, :] &= mask[1:, :]
    interior[:, 1:] &= mask[:, :-1]
    interior[:, :-1] &= mask[:, 1:]
    boundary = max(float((mask & ~interior).sum()), 1.0)
    return float(np.clip(2.0 * area / boundary, 1.0, 64.0))


def _thin(mask: np.ndarray, cell_px: float) -> np.ndarray:
    """
    Grid-centroid thinning: one point per `cell_px` cell of stroke.

    Subsampling the raw pixel list does NOT work here - a 9 px wide marker line
    leaves points scattered across the stroke's width, and the nearest-neighbour
    walk below then zig-zags side to side and reports a track three times its
    real length. Collapsing each cell to its centroid gives one point per arc
    position, which is what the walk needs.
    """
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return np.empty((0, 2), dtype=np.float32)
    c = max(2.0, float(cell_px))
    keys = (ys // c).astype(np.int64) * 100003 + (xs // c).astype(np.int64)
    order = np.argsort(keys, kind="stable")
    keys, xs, ys = keys[order], xs[order], ys[order]
    bounds = np.flatnonzero(np.diff(keys)) + 1
    pts = [[float(gx.mean()), float(gy.mean())]
           for gx, gy in zip(np.split(xs, bounds), np.split(ys, bounds))]
    return np.asarray(pts, dtype=np.float32)


def _path_len(p: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def _greedy(pts: np.ndarray, start: int) -> np.ndarray:
    """Nearest-neighbour walk from a fixed start."""
    n = len(pts)
    used = np.zeros(n, bool)
    used[start] = True
    order = [start]
    for _ in range(n - 1):
        d = np.linalg.norm(pts - pts[order[-1]], axis=1)
        d[used] = np.inf
        nxt = int(np.argmin(d))
        used[nxt] = True
        order.append(nxt)
    return pts[order]


def _two_opt(p: np.ndarray, passes: int = 6) -> np.ndarray:
    """
    Un-cross an open polyline by reversing segments.

    A pure nearest-neighbour walk on a traced stroke always leaves one or two
    long "jump back to the bit I skipped" edges; on an open path that inflated
    the measured length by ~50% in testing. 2-opt removes them, and a track
    length you can trust is the difference between a lap-time reward that works
    and one that silently rewards cutting corners.
    """
    n = len(p)
    if n < 5:
        return p
    for _ in range(passes):
        improved = False
        for i in range(1, n - 2):
            a, b = p[i - 1], p[i]
            j = np.arange(i + 1, n - 1)
            c, d = p[j], p[j + 1]
            delta = (np.linalg.norm(c - a, axis=1) + np.linalg.norm(d - b, axis=1)
                     - np.linalg.norm(b - a) - np.linalg.norm(d - c, axis=1))
            k = int(np.argmin(delta))
            if delta[k] < -1e-6:
                jj = int(j[k])
                p[i:jj + 1] = p[i:jj + 1][::-1]
                improved = True
        if not improved:
            break
    return p


def _order_points(pts: np.ndarray) -> tuple[np.ndarray, bool]:
    """
    Order scattered stroke samples into a polyline.

    Multi-start greedy + 2-opt. Cheap at these sizes (a few hundred points) and
    far more robust than a single greedy walk, which is entirely at the mercy of
    where it happens to start.
    """
    if len(pts) < 3:
        return pts, False

    c = pts.mean(axis=0)
    starts = {int(np.argmin(pts[:, 0] + pts[:, 1])),
              int(np.argmax(pts[:, 0] + pts[:, 1])),
              int(np.argmin(pts[:, 0] - pts[:, 1])),
              int(np.argmax(pts[:, 0] - pts[:, 1])),
              int(np.argmax(np.linalg.norm(pts - c, axis=1)))}

    best, best_len = None, np.inf
    for s in starts:
        cand = _two_opt(_greedy(pts, s).copy())
        length = _path_len(cand)
        if length < best_len:
            best, best_len = cand, length

    gap = float(np.linalg.norm(best[0] - best[-1]))
    step = np.median(np.linalg.norm(np.diff(best, axis=0), axis=1))
    closed = bool(gap < max(3.0 * step, 0.06 * float(np.ptp(best, axis=0).max())))
    return best, closed


def _resample(pts: np.ndarray, spacing: float, closed: bool) -> np.ndarray:
    """Uniform arc-length resampling - keeps curvature honest for the spline."""
    p = np.vstack([pts, pts[:1]]) if closed else pts
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s[-1])
    if total < 1e-6:
        return pts
    n = max(4, int(total / max(spacing, 1e-3)))
    t = np.linspace(0, total, n, endpoint=not closed)
    return np.stack([np.interp(t, s, p[:, 0]), np.interp(t, s, p[:, 1])], axis=1)


def trace_track(image_path: str, world_m: float, width_m: float = 2.0,
                simplify_m: float = 0.35, threshold: int = 128,
                dark_is_track: bool = True) -> Track:
    """
    Traces the darkest stroke in a drawing into an ordered centerline in metres.

    Assumes one dominant stroke on a light background - a marker loop on paper,
    or a line drawn in any paint program. Image centre maps to world origin,
    image +x to world +x, image +y (down) to world -y.
    """
    img = Image.open(image_path).convert("L")
    a = np.asarray(img, dtype=np.uint8)
    mask = a < threshold if dark_is_track else a > threshold
    h, w = a.shape
    px_per_m = max(w, h) / max(world_m, 1e-6)

    # Cells must be at least as wide as the stroke, or the walk zig-zags across
    # it and reports a track ~2x its true length. Resampling puts the requested
    # spacing back afterwards.
    cell = max(simplify_m * px_per_m, 1.3 * _stroke_width_px(mask))
    pts_px = _thin(mask, cell)
    if len(pts_px) < 3:
        raise ValueError("No stroke found - adjust --threshold or check the image.")

    ordered, closed = _order_points(pts_px)
    scale = world_m / max(w, h)
    xy = np.stack([(ordered[:, 0] - w / 2.0) * scale,
                   -(ordered[:, 1] - h / 2.0) * scale], axis=1)
    xy = _resample(xy, simplify_m, closed)

    seg = np.linalg.norm(np.diff(np.vstack([xy, xy[:1]]) if closed else xy, axis=0), axis=1)
    return Track(centerline_m=[[round(float(x), 3), round(float(y), 3)] for x, y in xy],
                 closed=closed, width_m=width_m, length_m=round(float(seg.sum()), 2))


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("procedural")
    p.add_argument("--out", required=True)
    p.add_argument("--obj")
    p.add_argument("--size", type=int, default=1009)
    p.add_argument("--world-m", type=float, default=40.0)
    p.add_argument("--roughness-m", type=float, default=0.15)
    p.add_argument("--feature-size-m", type=float, default=8.0)
    p.add_argument("--octaves", type=int, default=4)
    p.add_argument("--slope-max-deg", type=float, default=12.0)
    p.add_argument("--flat-radius-m", type=float, default=0.0)
    p.add_argument("--seed", type=int, default=0)

    q = sub.add_parser("from-image")
    q.add_argument("--image", required=True)
    q.add_argument("--out", required=True)
    q.add_argument("--obj")
    q.add_argument("--size", type=int, default=1009)
    q.add_argument("--world-m", type=float, default=40.0)
    q.add_argument("--roughness-m", type=float, default=0.5)
    q.add_argument("--smooth", type=float, default=4.0)
    q.add_argument("--slope-max-deg", type=float, default=12.0)
    q.add_argument("--invert", action="store_true")

    t = sub.add_parser("trace-track")
    t.add_argument("--image", required=True)
    t.add_argument("--out-json", required=True)
    t.add_argument("--world-m", type=float, default=40.0)
    t.add_argument("--width-m", type=float, default=2.0)
    t.add_argument("--simplify-m", type=float, default=0.35)
    t.add_argument("--threshold", type=int, default=128)

    a = ap.parse_args()

    if a.cmd == "procedural":
        hm, z = procedural_heightmap(a.size, a.world_m, a.roughness_m,
                                     a.feature_size_m, a.seed, a.octaves,
                                     a.slope_max_deg, a.flat_radius_m)
    elif a.cmd == "from-image":
        hm, z = heightmap_from_image(a.image, a.size, a.world_m, a.roughness_m,
                                     a.smooth, a.invert, a.slope_max_deg)
    else:
        track = trace_track(a.image, a.world_m, a.width_m, a.simplify_m, a.threshold)
        with open(a.out_json, "w") as f:
            json.dump(asdict(track), f, indent=2)
        print(f"traced {len(track.centerline_m)} pts, "
              f"{track.length_m} m, closed={track.closed} -> {a.out_json}")
        return

    save_heightmap(hm, a.out)
    print(json.dumps({"heightmap_path": a.out, "z_scale_m": round(z, 4),
                      "size_px": int(hm.shape[0]), "world_m": a.world_m}, indent=2))
    if getattr(a, "obj", None):
        heightmap_to_obj(hm, a.world_m, z, a.obj)
        print(f"mesh -> {a.obj}")


if __name__ == "__main__":
    main()
