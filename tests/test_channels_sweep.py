import math

import numpy as np
import pytest
from scipy.io import savemat

from fsae_lapsim import PRESETS, Vehicle, VehicleParams, run_qss, run_transient
from fsae_lapsim.channels import Dataset, from_qss, from_transient, load_mat, save_lap_mat
from fsae_lapsim.sweep import crossings, parse_levels, run_sweep, tradeoff

T = np.linspace(0, 10, 1001)


def test_flat_variables(tmp_path):
    f = tmp_path / "flat.mat"
    savemat(f, {"Time": T, "Speed": 20 + np.sin(T), "gear": 3.0})
    ds = load_mat(str(f))
    assert set(ds.channels) == {"Time", "Speed"}  # scalars are not channels
    assert ds.time_of["Speed"] == "Time"


def test_logger_structs_mixed_rates(tmp_path):
    t10 = np.linspace(0, 10, 101)
    f = tmp_path / "log.mat"
    savemat(f, {"Ground_Speed": {"Time": T, "Value": T * 2, "Units": "km/h"},
                "Steer": {"Time": t10, "Value": np.sin(t10), "Units": "deg"}})
    ds = load_mat(str(f))
    assert ds.units["Ground_Speed"] == "km/h"
    x, y = ds.xy("Ground_Speed", "Steer")  # aligned by time despite 1001 vs 101 samples
    assert len(x) == len(y) == 1001
    assert np.nanmax(np.abs(y - np.sin(T))) < 0.01


def test_struct_with_t_and_y_fields_is_not_a_single_channel(tmp_path):
    f = tmp_path / "s.mat"
    savemat(f, {"data": {"t": T, "y": np.sin(T), "speed": T, "acc": np.c_[T, 2 * T]}})
    ds = load_mat(str(f))
    assert {"data.t", "data.y", "data.speed", "data.acc_1", "data.acc_2"} <= set(ds.channels)
    assert ds.time_of["data.speed"] == "data.t"


def test_v73_hdf5(tmp_path):
    h5py = pytest.importorskip("h5py")
    f = tmp_path / "v73.mat"
    with h5py.File(f, "w", userblock_size=512) as h:
        g = h.create_group("log")
        for k, v in (("time", T), ("rpm", T * 100)):
            d = g.create_dataset(k, data=v[None, :])
            d.attrs["MATLAB_class"] = np.bytes_("double")
    ds = load_mat(str(f))
    assert ds.time_of["log.rpm"] == "log.time"
    assert np.allclose(ds.channels["log.rpm"], T * 100)


def test_no_signals_raises(tmp_path):
    f = tmp_path / "empty.mat"
    savemat(f, {"a": 1.0, "name": "x"})
    with pytest.raises(ValueError):
        load_mat(str(f))


def test_unaligned_channels_raise():
    ds = Dataset("d")
    ds.add("a", np.arange(5))
    ds.add("b", np.arange(7))
    with pytest.raises(ValueError):
        ds.xy("a", "b")


def test_lap_export_roundtrip(tmp_path):
    veh = Vehicle()
    tr = PRESETS["Oval"]()
    q = run_qss(veh, tr)
    r = run_transient(veh, tr, qss=q)
    f = tmp_path / "lap.mat"
    save_lap_mat(str(f), q, r, veh.p, tr)
    ds = load_mat(str(f))
    assert np.array_equal(ds.channels["qss.v"], q["v"])
    assert np.array_equal(ds.channels["transient.vx"], r["x"][:, 0])
    assert ds.units["transient.Fz_FL"] == "N"
    assert ds.time_of["transient.vx"] == "transient.t"
    x, y = ds.xy("qss.s", "transient.vx")  # different time bases -> aligned by time
    assert len(x) == len(y)
    assert set(from_qss(q).channels) and "speed_kph" in from_transient(r).channels


def test_parse_levels():
    assert list(parse_levels("45:1:47", 40, 50)) == [45, 46, 47]
    assert list(parse_levels("47, 45, 99", 40, 50)) == [45, 47]
    assert len(parse_levels("auto", 58.1, 61.9)) >= 3
    with pytest.raises(ValueError):
        parse_levels("1:0:5", 0, 10)


def test_crossings_and_tradeoff():
    x = np.array([0.0, 1.0, 2.0])
    assert crossings(x, [10.0, 12.0, 14.0], 11.0) == [0.5]
    v1, v2 = np.linspace(0, 4, 5), np.linspace(0, 2, 3)
    Z = 2.0 * v1[None, :] + 1.0 * v2[:, None]  # Z = 2x + y -> iso-line slope dy/dx = -2
    to = tradeoff(v1, v2, Z, 2.0, 1.0)
    assert to["slope"] == pytest.approx(-2.0)
    assert tradeoff(v1, v2, Z, 99.0, 1.0) is None


def test_run_sweep_1d_2d():
    tr = PRESETS["Oval"]()
    base = VehicleParams()
    r1 = run_sweep(base, tr, "m", [260, 300], parallel=False)
    assert r1["T"].shape == (2,) and r1["T"][1] > r1["T"][0]
    r2 = run_sweep(base, tr, "m", [260, 300], "wd_front", [0.47, 0.99], parallel=False)
    assert r2["T"].shape == (2, 2)
    assert np.all(np.isfinite(r2["T"][0])) and np.all(np.isnan(r2["T"][1]))  # wd_front 0.99 is invalid
    assert math.isclose(r2["T"][0, 0], r1["T"][0])
