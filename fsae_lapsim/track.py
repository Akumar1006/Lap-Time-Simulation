"""Track defined as a list of constant-curvature segments (length, radius).

radius > 0 : left-hand corner, radius < 0 : right-hand corner, radius == 0 : straight.
Length is always the arc length along the centre line.
"""
from __future__ import annotations

import bisect
import csv
import math
import os
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Segment:
    length: float
    radius: float = 0.0

    @property
    def curvature(self) -> float:
        return 0.0 if self.radius == 0 else 1.0 / self.radius

    @property
    def angle_deg(self) -> float:
        return math.degrees(self.length * self.curvature)


@dataclass
class Track:
    segments: list[Segment] = field(default_factory=list)
    closed: bool = True
    name: str = "track"

    def __post_init__(self):
        self.build()

    # ------------------------------------------------------------------ geometry
    def build(self, ds: float = 0.25) -> None:
        self.__dict__.pop("_seg_poses", None)
        segs = [sg for sg in self.segments if sg.length > 0]
        self._starts = []
        self._curv = []
        s = 0.0
        for sg in segs:
            self._starts.append(s)
            self._curv.append(sg.curvature)
            s += sg.length
        self.length = s
        if s <= 0:
            self.s = np.zeros(1)
            self.X = self.Y = self.theta = self.kappa = np.zeros(1)
            return
        # exact integration of piecewise-constant curvature
        npts = max(2, int(math.ceil(s / ds)) + 1)
        sg = np.linspace(0.0, s, npts)
        X = np.zeros(npts)
        Y = np.zeros(npts)
        th = np.zeros(npts)
        kap = np.zeros(npts)
        x0 = y0 = t0 = 0.0
        idx = np.searchsorted(np.array(self._starts), sg, side="right") - 1
        idx = np.clip(idx, 0, len(segs) - 1)
        seg_state = []
        for sgm in segs:
            seg_state.append((x0, y0, t0))
            x0, y0, t0 = self._advance(x0, y0, t0, sgm.curvature, sgm.length)
        self.end_pose = (x0, y0, t0)
        for k in range(npts):
            j = idx[k]
            xs, ys, ts = seg_state[j]
            ls = sg[k] - self._starts[j]
            X[k], Y[k], th[k] = self._advance(xs, ys, ts, self._curv[j], ls)
            kap[k] = self._curv[j]
        self.s, self.X, self.Y, self.theta, self.kappa = sg, X, Y, th, kap

    @staticmethod
    def _advance(x, y, th, k, ls):
        if abs(k) < 1e-12:
            return x + ls * math.cos(th), y + ls * math.sin(th), th
        th2 = th + k * ls
        return x + (math.sin(th2) - math.sin(th)) / k, y - (math.cos(th2) - math.cos(th)) / k, th2

    def closure_error(self) -> tuple[float, float]:
        """(position gap [m], heading error [deg]) between end and start of the lap."""
        x, y, th = self.end_pose
        dth = (th + math.pi) % (2 * math.pi) - math.pi
        return math.hypot(x, y), math.degrees(dth)

    # ------------------------------------------------------------------ lookups
    def wrap(self, s: float) -> float:
        if self.closed and self.length > 0:
            return s % self.length
        return s

    def curvature_at(self, s: float) -> float:
        s = self.wrap(s)
        if s < 0 or (not self.closed and s > self.length):
            return 0.0
        j = bisect.bisect_right(self._starts, s) - 1
        return self._curv[j if j >= 0 else 0]

    def pose_at(self, s: float) -> tuple[float, float, float]:
        """Centre-line (X, Y, heading) at distance s (extrapolated straight beyond an open end)."""
        if not self.closed and s > self.length:
            x, y, th = self.end_pose
            e = s - self.length
            return x + e * math.cos(th), y + e * math.sin(th), th
        s = self.wrap(s)
        j = bisect.bisect_right(self._starts, s) - 1
        j = max(j, 0)
        xs, ys, ts = self._seg_start_pose(j)
        return self._advance(xs, ys, ts, self._curv[j], s - self._starts[j])

    def _seg_start_pose(self, j):
        if not hasattr(self, "_seg_poses"):
            poses = []
            x = y = th = 0.0
            for k, st in enumerate(self._starts):
                poses.append((x, y, th))
                ln = (self._starts[k + 1] if k + 1 < len(self._starts) else self.length) - st
                x, y, th = self._advance(x, y, th, self._curv[k], ln)
            self._seg_poses = poses
        return self._seg_poses[j]

    def to_global(self, s, n, xi):
        """Curvilinear (s, n, xi) -> global (X, Y, psi). Accepts numpy arrays."""
        s = np.atleast_1d(np.asarray(s, float))
        n = np.atleast_1d(np.asarray(n, float))
        xi = np.atleast_1d(np.asarray(xi, float))
        out = np.array([self.pose_at(float(v)) for v in s])
        Xc, Yc, th = out[:, 0], out[:, 1], out[:, 2]
        return Xc - n * np.sin(th), Yc + n * np.cos(th), th + xi

    def rebuild(self) -> None:
        self.build()

    # ------------------------------------------------------------------ I/O
    def to_csv(self, path: str) -> None:
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["length_m", "radius_m"])
            for sg in self.segments:
                w.writerow([sg.length, sg.radius])

    @classmethod
    def from_csv(cls, path: str, closed: bool = True) -> "Track":
        segs = []
        with open(path, "r", encoding="utf-8") as fh:
            for row in csv.reader(fh):
                if not row:
                    continue
                try:
                    segs.append(Segment(float(row[0]), float(row[1]) if len(row) > 1 and row[1] else 0.0))
                except ValueError:
                    continue  # header
        return cls(segs, closed=closed, name=path)


def _arc(radius: float, angle_deg: float) -> Segment:
    return Segment(abs(radius) * math.radians(abs(angle_deg)), radius)


def _autocross_half():
    # total turning = +180 deg, so repeating the half yields a closed circuit
    return [Segment(40), _arc(9, 90), Segment(15), _arc(-12, 45), _arc(12, 45), Segment(20),
            _arc(20, 60), Segment(8), _arc(-10, 60), Segment(6), _arc(7, 90), Segment(25)]


def _endurance_half():
    return [Segment(60), _arc(15, 90), Segment(10), _arc(-8, 45), _arc(8, 45), Segment(35),
            _arc(-25, 30), _arc(25, 60), Segment(12), _arc(6, 60), Segment(20),
            _arc(-12, 90), Segment(15), _arc(10, 60), Segment(30), _arc(18, 30), Segment(10)]


DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def spa_francorchamps() -> Track:
    """Spa-Francorchamps F1 layout (7.0 km, clockwise) as 172 constant-radius segments."""
    tr = Track.from_csv(os.path.join(DATA_DIR, "spa_francorchamps.csv"), closed=True)
    tr.name = "Spa-Francorchamps (F1)"
    return tr


def autocross_2024() -> Track:
    """FSAE autocross digitised from the 2024 course map (red line, 878 m, open course)."""
    tr = Track.from_csv(os.path.join(DATA_DIR, "autocross_2024.csv"), closed=False)
    tr.name = "FSAE Autocross 2024 (course map)"
    return tr


PRESETS = {
    "Spa-Francorchamps (F1)": spa_francorchamps,
    "FSAE Autocross 2024 (course map)": autocross_2024,
    "FSAE autocross (sample)": lambda: Track(_autocross_half() * 2, True, "FSAE autocross (sample)"),
    "FSAE endurance lap (sample)": lambda: Track(_endurance_half() * 2, True, "FSAE endurance lap (sample)"),
    "Skidpad (R = 9.125 m circle)": lambda: Track([_arc(9.125, 360)], True, "Skidpad"),
    "Acceleration (75 m)": lambda: Track([Segment(75)], False, "Acceleration"),
    "Oval": lambda: Track([Segment(60), _arc(15, 180), Segment(60), _arc(15, 180)], True, "Oval"),
}
