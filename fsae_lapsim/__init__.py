"""FSAE 4-corner planar vehicle model, state-space tools and lap-time simulation."""
import os as _os
import sys as _sys

if _os.name == "nt":
    # Non-activated conda Pythons lack Library\bin on PATH, so LAPACK calls (e.g. numpy.linalg.inv)
    # fail with a delay-load error (0xc06d007f). Make the env's DLL folders discoverable.
    for _sub in (r"Library\bin", r"Library\mingw-w64\bin", r"Library\usr\bin"):
        _d = _os.path.join(_sys.prefix, _sub)
        if _os.path.isdir(_d) and _d.lower() not in _os.environ.get("PATH", "").lower():
            _os.environ["PATH"] = _d + _os.pathsep + _os.environ.get("PATH", "")

from .params import VehicleParams, PARAM_META  # noqa: E402
from .vehicle import Vehicle, STATE_NAMES, STATE_UNITS, INPUT_NAMES, INPUT_UNITS, OUTPUT_NAMES  # noqa: E402
from .track import Track, PRESETS  # noqa: E402
from .qss import GGV, run_qss  # noqa: E402
from .transient import DriverSettings, run_transient  # noqa: E402

__all__ = [
    "VehicleParams", "PARAM_META", "Vehicle", "STATE_NAMES", "STATE_UNITS",
    "INPUT_NAMES", "INPUT_UNITS", "OUTPUT_NAMES", "Track", "PRESETS",
    "GGV", "run_qss", "DriverSettings", "run_transient",
]
