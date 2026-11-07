"""Parameter sweeps of QSS lap time / energy over one parameter (curve) or two (grid for iso-lap-time contours)."""
from __future__ import annotations

import math
import os
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict

import numpy as np

from .params import VehicleParams
from .qss import run_qss
from .track import Segment, Track
from .vehicle import Vehicle


def evaluate(params: dict, segments, closed: bool, standing: bool) -> tuple[float, float]:
    """QSS lap time [s] and net energy [Wh] for one parameter set (NaN if the set is invalid)."""
    try:
        veh = Vehicle(VehicleParams(**params))
        tr = Track([Segment(l, r) for l, r in segments], closed)
        q = run_qss(veh, tr, standing_start=standing)
        return float(q["lap_time"]), float(q["E"][-1] / 3600.0)
    except Exception:  # noqa: BLE001 - invalid parameter combinations become gaps in the map
        return math.nan, math.nan


def run_sweep(base: VehicleParams, track: Track, p1: str, vals1, p2: str | None = None, vals2=None,
              standing: bool | None = None, parallel: bool = True, workers: int | None = None,
              progress=None, cancel=None) -> dict:
    """Evaluate the lap on a 1-D (p1) or 2-D (p1 x p2) grid.

    Returns arrays ``T`` (lap time) and ``E`` (energy) shaped (n1,) for 1-D or (n2, n1) for 2-D,
    i.e. rows follow p2 (y axis) and columns follow p1 (x axis), ready for ``contour(vals1, vals2, T)``.
    """
    vals1 = np.asarray(vals1, float)
    vals2 = np.asarray(vals2, float) if p2 else np.array([np.nan])
    if standing is None:
        standing = not track.closed
    segs = [(sg.length, sg.radius) for sg in track.segments]
    bd = asdict(base)
    jobs = []
    for j, b in enumerate(vals2):
        for i, a in enumerate(vals1):
            d = dict(bd)
            d[p1] = float(a)
            if p2:
                d[p2] = float(b)
            jobs.append(((j, i), d))
    T = np.full((len(vals2), len(vals1)), np.nan)
    E = np.full_like(T, np.nan)
    status = "ok"
    done = 0

    def report():
        if progress:
            progress(done / len(jobs))

    finished: set = set()
    n_workers = min(workers or max(1, (os.cpu_count() or 2) - 1), len(jobs))
    if parallel and n_workers > 1 and len(jobs) > 3:
        try:
            with ProcessPoolExecutor(max_workers=n_workers) as ex:
                futs = {ex.submit(evaluate, d, segs, track.closed, standing): key for key, d in jobs}
                for f in as_completed(futs):
                    key = futs[f]
                    T[key], E[key] = f.result()
                    finished.add(key)
                    done += 1
                    report()
                    if cancel and cancel():
                        status = "cancelled"
                        for g in futs:
                            g.cancel()
                        ex.shutdown(wait=False, cancel_futures=True)
                        break
        except Exception:  # noqa: BLE001 - e.g. process start-up blocked: finish serially
            pass
    if status == "ok":
        for key, d in jobs:
            if key in finished:
                continue
            if cancel and cancel():
                status = "cancelled"
                break
            T[key], E[key] = evaluate(d, segs, track.closed, standing)
            done += 1
            report()
    if not p2:
        T, E = T[0], E[0]
    return dict(p1=p1, vals1=vals1, p2=p2, vals2=vals2 if p2 else None, T=T, E=E, status=status,
                track=track.name, standing=standing, base=bd)


def parse_levels(text: str, zmin: float, zmax: float, n_auto: int = 8) -> np.ndarray:
    """Contour levels from 'auto', 'a, b, c' or MATLAB-style 'start:step:stop'; clipped to the data range."""
    text = (text or "").strip().lower()
    if not text or text == "auto":
        from matplotlib.ticker import MaxNLocator
        lv = MaxNLocator(n_auto).tick_values(zmin, zmax)
    elif ":" in text:
        parts = [float(v) for v in text.split(":")]
        if len(parts) != 3 or parts[1] <= 0:
            raise ValueError("Use start:step:stop with a positive step, e.g. 45:0.5:50")
        a, st, b = parts
        if (b - a) / st > 500:
            raise ValueError("Too many contour levels (> 500)")
        lv = np.arange(a, b + st * 1e-9, st)
    else:
        lv = np.array([float(v) for v in text.replace(";", ",").replace(" ", ",").split(",") if v])
    lv = np.unique(lv)
    return lv[(lv >= zmin) & (lv <= zmax)]


def crossings(x, y, level: float) -> list[float]:
    """x positions where the piecewise-linear curve y(x) crosses ``level``."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    out = []
    for i in range(len(x) - 1):
        y0, y1 = y[i] - level, y[i + 1] - level
        if not (math.isfinite(y0) and math.isfinite(y1)):
            continue
        if y0 == 0:
            out.append(x[i])
        elif y0 * y1 < 0:
            out.append(x[i] + (x[i + 1] - x[i]) * y0 / (y0 - y1))
    if len(y) and math.isfinite(y[-1]) and y[-1] == level:
        out.append(x[-1])
    return out


def tradeoff(vals1, vals2, Z, x0: float, y0: float):
    """Gradients of Z at (x0, y0) and the iso-line slope dy/dx = -(dZ/dx)/(dZ/dy).

    Returns None when the point lies outside the grid.
    """
    from scipy.interpolate import RegularGridInterpolator

    vals1 = np.asarray(vals1, float)
    vals2 = np.asarray(vals2, float)
    if len(vals1) < 2 or len(vals2) < 2:
        return None
    if not (min(vals1) <= x0 <= max(vals1) and min(vals2) <= y0 <= max(vals2)):
        return None
    dZdy, dZdx = np.gradient(Z, vals2, vals1)
    pt = [[y0, x0]]
    gx = float(RegularGridInterpolator((vals2, vals1), dZdx)(pt)[0])
    gy = float(RegularGridInterpolator((vals2, vals1), dZdy)(pt)[0])
    z = float(RegularGridInterpolator((vals2, vals1), Z)(pt)[0])
    slope = -gx / gy if gy != 0 else math.inf
    return dict(z=z, dZdx=gx, dZdy=gy, slope=slope)
