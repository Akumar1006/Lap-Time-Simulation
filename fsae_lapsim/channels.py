"""Channel datasets: simulation results and imported logged/reference data (.mat).

A ``Dataset`` is a flat collection of named 1-D channels. Channels may have different lengths
(e.g. a logger recording channels at different rates); each channel can point at a time-base
channel so that any two channels can be aligned for X-Y plotting.
"""
from __future__ import annotations

import datetime as _dt
import math
import os
import re
from dataclasses import asdict, dataclass, field

import numpy as np

from .vehicle import CORNERS, STATE_NAMES, STATE_UNITS

TIME_NAMES = {"time", "t", "time_s", "times", "timestamp", "xtime", "time_sec", "sessiontime"}
VALUE_FIELDS = ("Value", "value", "Values", "values", "Data", "data", "Y", "y", "Signal", "signal")
TIME_FIELDS = ("Time", "time", "T", "t", "X", "x", "Timestamp", "timestamp")
UNIT_FIELDS = ("Units", "units", "Unit", "unit")
NAME_FIELDS = ("Name", "name", "ChannelName", "channel", "Channel")


@dataclass
class Dataset:
    name: str
    channels: dict[str, np.ndarray] = field(default_factory=dict)
    units: dict[str, str] = field(default_factory=dict)
    time_of: dict[str, str] = field(default_factory=dict)  # channel -> its time-base channel
    source: str = ""

    def add(self, name: str, data, unit: str = "", time: str | None = None) -> None:
        arr = np.asarray(data, dtype=float).ravel()
        self.channels[name] = arr
        if unit:
            self.units[name] = unit
        if time:
            self.time_of[name] = time

    def names(self) -> list[str]:
        return list(self.channels)

    def label(self, ch: str) -> str:
        u = self.units.get(ch, "")
        return f"{ch} [{u}]" if u else ch

    def copy(self, name: str) -> "Dataset":
        return Dataset(name, {k: v.copy() for k, v in self.channels.items()}, dict(self.units),
                       dict(self.time_of), self.source)

    def default_x(self) -> str | None:
        """A sensible default x-axis channel: distance first, then time."""
        prefs = ["s", "distance", "lap_distance", "lapdistance", "dist", "s_m", "t", "time"]
        low = {k.lower().split(".")[-1].replace(" ", "_"): k for k in self.channels}
        for p in prefs:
            if p in low:
                return low[p]
        for k in self.channels:
            if "distance" in k.lower():
                return k
        return next(iter(self.channels), None)

    def xy(self, xch: str, ych: str) -> tuple[np.ndarray, np.ndarray]:
        """Return aligned (x, y) arrays. Channels on different time bases are aligned by time."""
        x = self.channels[xch]
        y = self.channels[ych]
        bx, by = self._time_name(xch), self._time_name(ych)
        if len(x) == len(y) and (bx is None or by is None or bx == by):
            return x, y
        if bx is None or by is None:
            raise ValueError(f"'{xch}' ({len(x)} samples) and '{ych}' ({len(y)} samples) have different "
                             "lengths and no common time base to align them")
        tx, ty = self.channels[bx], self.channels[by]
        order = np.argsort(ty, kind="stable")
        return x, np.interp(tx, ty[order], y[order], left=np.nan, right=np.nan)

    def _time_name(self, ch: str) -> str | None:
        t = self.time_of.get(ch)
        if t in self.channels and len(self.channels[t]) == len(self.channels[ch]):
            return t
        if _is_time_name(ch):
            return ch
        return None


def _is_time_name(name: str) -> bool:
    return name.lower().split(".")[-1].replace(" ", "_") in TIME_NAMES


def _monotonic(a: np.ndarray) -> bool:
    return len(a) > 1 and bool(np.all(np.diff(a) >= 0))


# ====================================================================== simulation results
def from_qss(q: dict, name: str = "Sim QSS") -> Dataset:
    ds = Dataset(name, source="QSS lap simulation")
    t = "t"
    ds.add("t", q["t"], "s")
    ds.add("s", q["s"], "m", t)
    ds.add("v", q["v"], "m/s", t)
    ds.add("v_kph", q["v"] * 3.6, "km/h", t)
    ds.add("v_apex", q["v_apex"], "m/s", t)
    ds.add("ax", q["ax"], "m/s^2", t)
    ds.add("ay", q["ay"], "m/s^2", t)
    ds.add("ax_g", q["ax"] / 9.81, "g", t)
    ds.add("ay_g", q["ay"] / 9.81, "g", t)
    ds.add("curvature", q["kappa"], "1/m", t)
    for i, c in enumerate(CORNERS):
        ds.add(f"Fz_{c}", q["Fz"][:, i], "N", t)
    ds.add("P_el", q["P_el"], "W", t)
    ds.add("E", q["E"], "J", t)
    ds.add("X", q["X"], "m", t)
    ds.add("Y", q["Y"], "m", t)
    return ds


def from_transient(r: dict, name: str = "Sim Transient") -> Dataset:
    from .transient import Y_COLS  # local import avoids a cycle
    from .vehicle import INPUT_NAMES, INPUT_UNITS

    ds = Dataset(name, source=f"Transient lap simulation ({r.get('status', '')})")
    t = "t"
    ds.add("t", r["t"], "s")
    x, u, y = r["x"], r["u"], r["y"]
    for i, (n, un) in enumerate(zip(STATE_NAMES, STATE_UNITS)):
        ds.add(n, x[:, i], un, t)
    for i, (n, un) in enumerate(zip(INPUT_NAMES, INPUT_UNITS)):
        ds.add(n, u[:, i], un, t)
    yu = {"ax": "m/s^2", "ay": "m/s^2", "beta": "rad", "kappa": "-", "alpha": "rad", "Fx": "N", "Fy": "N", "Pel": "W"}
    for i, n in enumerate(Y_COLS):
        ds.add(n, y[:, i], yu.get(n.split("_")[0], ""), t)
    speed = np.hypot(x[:, 0], x[:, 1])
    ds.add("speed", speed, "m/s", t)
    ds.add("speed_kph", speed * 3.6, "km/h", t)
    ds.add("ax_g", y[:, 0] / 9.81, "g", t)
    ds.add("ay_g", y[:, 1] / 9.81, "g", t)
    ds.add("beta_deg", np.degrees(y[:, 2]), "deg", t)
    ds.add("delta_FL_deg", np.degrees(u[:, 0]), "deg", t)
    ds.add("delta_FR_deg", np.degrees(u[:, 1]), "deg", t)
    ds.add("P_el", y[:, -4:].sum(axis=1), "W", t)
    ds.add("v_ref", r["v_ref"], "m/s", t)
    ds.add("X", r["X"], "m", t)
    ds.add("Y", r["Y"], "m", t)
    ds.add("psi", r["psi"], "rad", t)
    return ds


# ====================================================================== .mat import
def load_mat(path: str, name: str | None = None) -> Dataset:
    """Load every numeric 1-D signal in a MATLAB file (v4/v5/v7 via scipy, v7.3 via h5py)."""
    ds = Dataset(name or os.path.splitext(os.path.basename(path))[0], source=path)
    if _is_hdf5(path):  # MATLAB v7.3
        tree = _load_h5(path)
    else:
        from scipy.io import loadmat
        raw = loadmat(path, squeeze_me=True, simplify_cells=True)
        tree = {k: v for k, v in raw.items() if not k.startswith("__")}
    _walk(tree, "", ds)
    if not ds.channels:
        raise ValueError("No numeric signals found. Store channels as numeric vectors, structs of vectors, "
                         "or per-channel structs with Time/Value fields (MATLAB tables/timetables are not "
                         "readable outside MATLAB: convert with table2struct(T,'ToScalar',true)).")
    _assign_time_bases(ds)
    return ds


def _is_hdf5(path: str) -> bool:
    """MATLAB v7.3 files are HDF5 with a 512-byte user block (signature at offset 0 or 512)."""
    sig = b"\x89HDF\r\n\x1a\n"
    with open(path, "rb") as fh:
        head = fh.read(520)
    return head[:8] == sig or head[512:520] == sig


def _load_h5(path: str) -> dict:
    try:
        import h5py
    except ImportError as e:
        raise ValueError("This is a MATLAB v7.3 (HDF5) file and h5py is not installed. "
                         "Install h5py or re-save in MATLAB with save(..., '-v7').") from e

    def conv(obj):
        if isinstance(obj, h5py.Group):
            return {k: conv(obj[k]) for k in obj if not k.startswith("#")}
        cls = obj.attrs.get("MATLAB_class", b"")
        cls = cls.decode() if isinstance(cls, bytes) else str(cls)
        if obj.attrs.get("MATLAB_empty", 0):
            return None
        data = obj[()]
        if cls == "char":
            return "".join(chr(c) for c in np.asarray(data).ravel())
        if cls in ("cell", "") and data.dtype == object:
            return None
        return np.asarray(data).T  # MATLAB is column-major

    with h5py.File(path, "r") as f:
        return {k: conv(f[k]) for k in f if not k.startswith("#")}


def _clean(name: str) -> str:
    return re.sub(r"\s+", "_", str(name).strip())


def _first(d: dict, keys):
    for k in keys:
        if k in d:
            return k
    return None


def _walk(obj, prefix: str, ds: Dataset) -> None:
    if isinstance(obj, dict):
        vk = _first(obj, VALUE_FIELDS)
        tk = _first(obj, TIME_FIELDS)
        sig_fields = {k for k, v in obj.items() if _is_signal(v)}
        if prefix and vk in sig_fields and sig_fields <= {vk, tk}:
            # per-channel struct (logger style): {Time, Value, Units, ...}
            ch = prefix
            val = _as_1d(obj[vk])
            uk = _first(obj, UNIT_FIELDS)
            unit = obj[uk] if uk and isinstance(obj[uk], str) else ""
            if tk in sig_fields and len(_as_1d(obj[tk])) == len(val):
                ds.add(ch + ".time", _as_1d(obj[tk]), "s")
                ds.add(ch, val, unit, ch + ".time")
            else:
                ds.add(ch, val, unit)
            return
        for k, v in obj.items():
            _walk(v, f"{prefix}.{_clean(k)}" if prefix else _clean(k), ds)
        units = obj.get("units")
        if isinstance(units, dict):  # sibling unit strings, as written by save_lap_mat
            for k, u in units.items():
                ch = f"{prefix}.{_clean(k)}" if prefix else _clean(k)
                if isinstance(u, str) and u and ch in ds.channels:
                    ds.units[ch] = u
    elif isinstance(obj, (list, tuple)) and obj and all(isinstance(e, dict) for e in obj):
        for i, e in enumerate(obj):  # struct array: name elements by their Name field if present
            nk = _first(e, NAME_FIELDS)
            sub = _clean(e[nk]) if nk and isinstance(e[nk], str) else str(i + 1)
            _walk(e, f"{prefix}.{sub}" if prefix else sub, ds)
    elif isinstance(obj, np.ndarray) and obj.dtype == object:
        if obj.size and all(isinstance(e, dict) for e in obj.ravel()):
            _walk(list(obj.ravel()), prefix, ds)
    elif _is_signal(obj):
        a = np.asarray(obj)
        if a.ndim == 2 and min(a.shape) > 1:
            if a.shape[0] < a.shape[1]:
                a = a.T
            if a.shape[1] <= 64:
                for j in range(a.shape[1]):
                    ds.add(f"{prefix}_{j + 1}", a[:, j])
        else:
            ds.add(prefix, a)


def _is_signal(v) -> bool:
    if not isinstance(v, np.ndarray) or v.dtype == object or np.iscomplexobj(v):
        return False
    if not (np.issubdtype(v.dtype, np.number) or v.dtype == bool):
        return False
    return v.ndim >= 1 and v.size > 1 and v.ndim <= 2


def _as_1d(v) -> np.ndarray:
    return np.asarray(v, dtype=float).ravel()


def _assign_time_bases(ds: Dataset) -> None:
    """Give every channel without a time base a same-length, monotonic time channel from its group."""
    times = [k for k in ds.channels if _is_time_name(k) and _monotonic(ds.channels[k])]
    for ch, a in ds.channels.items():
        if ch in ds.time_of or ch in times:
            continue
        grp = ch.rsplit(".", 1)[0] if "." in ch else ""
        cands = [t for t in times if len(ds.channels[t]) == len(a)]
        same_grp = [t for t in cands if (t.rsplit(".", 1)[0] if "." in t else "") == grp]
        pick = (same_grp or cands or [None])[0]
        if pick:
            ds.time_of[ch] = pick
    for t in times:
        ds.units.setdefault(t, "s")


# ====================================================================== .mat export
def _matname(n: str) -> str:
    s = re.sub(r"\W", "_", n)
    if not s or not s[0].isalpha():
        s = "c_" + s
    return s[:63]


def dataset_struct(ds: Dataset) -> dict:
    st = {_matname(k): v for k, v in ds.channels.items()}
    st["units"] = {_matname(k): ds.units.get(k, "") for k in ds.channels}
    return st


def save_lap_mat(path: str, qss: dict | None, transient: dict | None, params=None, track=None,
                 driver=None) -> None:
    """Write lap-simulation channels to a MATLAB .mat file (one struct per method)."""
    from scipy.io import savemat

    out: dict = {}
    meta = {"created": _dt.datetime.now().isoformat(timespec="seconds"), "tool": "fsae_lapsim"}
    if qss is not None:
        out["qss"] = dataset_struct(from_qss(qss))
        meta["lap_time_qss"] = float(qss["lap_time"])
    if transient is not None:
        out["transient"] = dataset_struct(from_transient(transient))
        meta["lap_time_transient"] = float(transient["lap_time"]) if math.isfinite(transient["lap_time"]) else np.nan
        meta["transient_status"] = transient.get("status", "")
    if params is not None:
        out["vehicle"] = {k: (float(v) if not isinstance(v, bool) else int(v)) for k, v in asdict(params).items()}
    if driver is not None:
        out["driver"] = {k: (float(v) if not isinstance(v, bool) else int(v)) for k, v in asdict(driver).items()}
    if track is not None:
        out["track"] = {"name": track.name, "closed": int(track.closed), "total_length_m": float(track.length),
                        "length_m": np.array([sg.length for sg in track.segments], float),
                        "radius_m": np.array([sg.radius for sg in track.segments], float)}
    out["meta"] = meta
    savemat(path, out, long_field_names=True, do_compression=True)
