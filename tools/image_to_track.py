"""Digitise a coloured track line from a course-map image into a length/radius segment CSV.

Example (the 2024 autocross drawing):
    python tools/image_to_track.py "course.png" fsae_lapsim/data/autocross_2024.csv \
        --start 328 45 --grid-ft 25 --preview preview.png

Steps:
  1. Mask pixels of the line colour (default: saturated red) and blur slightly.
  2. Walk along the line from the start pixel, re-centring across the line at each step.
  3. Scale pixels to metres (auto-detected grid spacing, or --m-per-px).
  4. Smooth, fit piecewise-constant curvature (Douglas-Peucker on heading vs distance),
     enforce a minimum radius (keeping each piece's turning angle) and least-squares
     adjust the turning angles so the segment chain stays on the drawn line.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import fsae_lapsim  # noqa: E402,F401  (sets up DLL paths on Windows conda)
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.ndimage import gaussian_filter, gaussian_filter1d, map_coordinates  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402
from scipy.signal import find_peaks  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402

from fsae_lapsim.track import Segment, Track  # noqa: E402


def line_mask(img, color: str, thresh: float):
    r, g, b = img[..., 0], img[..., 1], img[..., 2]
    ch = {"red": (r, g, b), "green": (g, r, b), "blue": (b, r, g)}[color]
    m = np.clip(ch[0] - np.maximum(ch[1], ch[2]), 0, 1)
    return np.where(m > thresh, m, 0.0)


def grid_spacing_px(img) -> float:
    """Median spacing of the dark grey grid lines (both axes)."""
    gray = img.mean(axis=2)
    sat = img.max(axis=2) - img.min(axis=2)
    dark = ((gray < 0.75) & (sat < 0.15)).astype(float)
    sp = []
    for prof in (dark.sum(axis=0), dark.sum(axis=1)):
        pk, _ = find_peaks(prof, height=0.3 * prof.max(), distance=5)
        d = np.diff(pk)
        if len(d):
            med = np.median(d)
            sp.extend(d[np.abs(d - med) < 0.25 * med])
    if not sp:
        raise RuntimeError("Could not detect a grid; pass --m-per-px")
    return float(np.mean(sp))


def trace(mask, start, heading_deg=0.0, step=1.0, look=4.0, max_pts=20000):
    R = gaussian_filter(mask, 1.0)

    def val(x, y):
        return map_coordinates(R, [np.atleast_1d(y), np.atleast_1d(x)], order=1, mode="constant")

    def recentre(p, h):
        nx, ny = -math.sin(h), math.cos(h)
        t = np.linspace(-look, look, 33)
        v = val(p[0] + t * nx, p[1] + t * ny)
        if v.sum() < 1e-9:
            return p
        c = (t * v).sum() / v.sum()
        return p[0] + c * nx, p[1] + c * ny

    p = recentre(tuple(map(float, start)), math.radians(heading_deg))
    h = math.radians(heading_deg)
    pts = [p]
    for _ in range(max_pts):
        th = h + np.radians(np.arange(-80, 81, 2))
        w = val(p[0] + look * np.cos(th), p[1] + look * np.sin(th))
        if w.max() < 0.08:
            break
        w = w ** 3
        hn = math.atan2((w * np.sin(th)).sum(), (w * np.cos(th)).sum())
        p = recentre((p[0] + step * math.cos(hn), p[1] + step * math.sin(hn)), hn)
        h = math.atan2(p[1] - pts[-1][1], p[0] - pts[-1][0])
        pts.append(p)
    return np.array(pts)


def _arclen(a):
    return np.r_[0.0, np.cumsum(np.hypot(*np.diff(a, axis=0).T))]


def fit_segments(xy, smooth=1.5, tol_deg=2.0, r_min=4.5, ds=0.25):
    s = _arclen(xy)
    su = np.linspace(0, s[-1], int(s[-1] / ds) + 1)
    sm = np.c_[gaussian_filter1d(np.interp(su, s, xy[:, 0]), smooth / ds, mode="nearest"),
               gaussian_filter1d(np.interp(su, s, xy[:, 1]), smooth / ds, mode="nearest")]
    s2 = _arclen(sm)
    sv = np.linspace(0, s2[-1], int(s2[-1] / ds) + 1)
    ref = np.c_[np.interp(sv, s2, sm[:, 0]), np.interp(sv, s2, sm[:, 1])]
    th = np.unwrap(np.arctan2(np.gradient(ref[:, 1]), np.gradient(ref[:, 0])))
    tol = math.radians(tol_deg)
    bps = [0]
    stack = [(0, len(sv) - 1)]
    while stack:  # iterative Douglas-Peucker on heading(s)
        i0, i1 = stack.pop()
        lin = th[i0] + (th[i1] - th[i0]) * (sv[i0:i1 + 1] - sv[i0]) / (sv[i1] - sv[i0])
        e = np.abs(th[i0:i1 + 1] - lin)
        k = int(np.argmax(e))
        if e[k] > tol and i1 - i0 > 2:
            stack += [(i0 + k, i1), (i0, i0 + k)]
        else:
            bps.append(i1)
    bps = sorted(set(bps))
    ang = np.array([th[b] - th[a] for a, b in zip(bps[:-1], bps[1:])])
    ln0 = np.array([sv[b] - sv[a] for a, b in zip(bps[:-1], bps[1:])])
    tgt = ref[bps]

    def lens(an):
        return np.maximum(ln0, np.abs(an) * r_min)

    def chain(an):
        x = y = 0.0
        t = th[0]
        out = [(x, y)]
        for a, l in zip(an, lens(an)):
            x, y, t = Track._advance(x, y, t, a / l, l)
            out.append((x, y))
        return np.array(out)

    sol = least_squares(lambda z: np.r_[(chain(ang + z) - tgt).ravel(), 3.0 * z / math.radians(1)],
                        np.zeros(len(ang)))
    ang = ang + sol.x
    segs: list[list[float]] = []
    for a, l in zip(ang, lens(ang)):
        k = a / l if abs(a) > math.radians(0.3) else 0.0
        if segs and k == 0 and segs[-1][1] == 0:
            segs[-1][0] += l
        else:
            segs.append([l, k])
    return np.array(segs), th[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image")
    ap.add_argument("out_csv")
    ap.add_argument("--start", nargs=2, type=float, required=True, metavar=("X", "Y"), help="start pixel on the line")
    ap.add_argument("--heading", type=float, default=0.0, help="initial direction in image [deg], 0 = +x (right)")
    ap.add_argument("--color", default="red", choices=["red", "green", "blue"])
    ap.add_argument("--thresh", type=float, default=0.35, help="colour dominance threshold (0..1)")
    sc = ap.add_mutually_exclusive_group(required=True)
    sc.add_argument("--grid-ft", type=float, help="real size of one grid cell [ft] (auto-detects pixel spacing)")
    sc.add_argument("--grid-m", type=float, help="real size of one grid cell [m]")
    sc.add_argument("--m-per-px", type=float)
    ap.add_argument("--smooth", type=float, default=1.5, help="Gaussian smoothing [m]")
    ap.add_argument("--tol", type=float, default=2.0, help="heading tolerance per segment [deg]")
    ap.add_argument("--min-radius", type=float, default=4.5, help="minimum centre-line radius [m]")
    ap.add_argument("--closed", action="store_true", help="track is a closed loop")
    ap.add_argument("--name", default=None)
    ap.add_argument("--preview", default=None, help="save an overlay PNG")
    a = ap.parse_args()

    img = plt.imread(a.image)[..., :3].astype(float)
    if img.max() > 1.0:
        img /= 255.0
    if a.m_per_px:
        mpp = a.m_per_px
        cell = None
    else:
        cell = grid_spacing_px(img)
        mpp = (a.grid_ft * 0.3048 if a.grid_ft else a.grid_m) / cell
    P = trace(line_mask(img, a.color, a.thresh), a.start, a.heading)
    if len(P) < 10:
        sys.exit("Tracing failed - check --start, --color and --thresh")
    xy = np.c_[P[:, 0] - P[0, 0], -(P[:, 1] - P[0, 1])] * mpp
    segs, th0 = fit_segments(xy, a.smooth, a.tol, a.min_radius)
    tr = Track([Segment(l, 1 / k if k else 0.0) for l, k in segs], closed=a.closed)
    c, s = math.cos(th0), math.sin(th0)
    fit = np.array([tr.pose_at(v)[:2] for v in np.linspace(0, tr.length, 3000)]) @ np.array([[c, s], [-s, c]])
    dev = cKDTree(fit).query(xy)[0]

    name = a.name or os.path.splitext(os.path.basename(a.out_csv))[0]
    with open(a.out_csv, "w", newline="", encoding="utf-8") as fh:
        fh.write(f"# {name}\n")
        fh.write(f"# Digitised from '{os.path.basename(a.image)}' with tools/image_to_track.py: start px {a.start}, "
                 f"{mpp:.4f} m/px" + (f" (grid {cell:.2f} px/cell)" if cell else "") + "\n")
        fh.write(f"# smooth {a.smooth} m, heading tol {a.tol} deg, min radius {a.min_radius} m; "
                 f"deviation from drawn line: mean {dev.mean():.2f} m, max {dev.max():.2f} m\n")
        fh.write("# radius: + left, - right, 0 straight\nlength_m,radius_m\n")
        for l, k in segs:
            fh.write(f"{l:.4f},{(1 / k if k else 0):.4f}\n")
    rmin = np.min(np.abs(1 / segs[segs[:, 1] != 0, 1]))
    print(f"traced {len(P)} px points, {mpp:.4f} m/px")
    print(f"{len(segs)} segments, length {tr.length:.1f} m, min radius {rmin:.2f} m")
    print(f"deviation from drawn line: mean {dev.mean():.2f} m, max {dev.max():.2f} m -> {a.out_csv}")
    if a.preview:
        fig = plt.figure(figsize=(18, 18 * img.shape[0] / img.shape[1] + 1))
        plt.imshow(img, interpolation="nearest")
        plt.plot(P[0, 0] + fit[:, 0] / mpp, P[0, 1] - fit[:, 1] / mpp, "b-", lw=1, label="fitted segments")
        plt.plot(*P[0], "go", label="start")
        plt.plot(*P[-1], "ks", label="end")
        plt.legend(loc="upper right")
        fig.savefig(a.preview, dpi=80, bbox_inches="tight")


if __name__ == "__main__":
    main()
