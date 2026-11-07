import math

import numpy as np
import pytest

from fsae_lapsim import PRESETS, DriverSettings, Track, Vehicle, VehicleParams, run_qss, run_transient
from fsae_lapsim.track import Segment


@pytest.fixture(scope="module")
def veh():
    return Vehicle()


def test_presets_close():
    for name, mk in PRESETS.items():
        tr = mk()
        if tr.closed:
            gap, dth = tr.closure_error()
            assert gap < 0.05 and abs(dth) < 0.01, name


def test_autocross_2024_geometry():
    tr = PRESETS["FSAE Autocross 2024 (course map)"]()
    assert not tr.closed and len(tr.segments) > 100
    assert tr.length == pytest.approx(878, abs=15)
    assert min(abs(sg.radius) for sg in tr.segments if sg.radius) >= 4.5 - 1e-3
    x, y, th = tr.end_pose
    # finish (checkered flag) is ~47 m right of and ~29 m below the start, heading back left
    assert 40 < x < 55 and -35 < y < -22 and math.cos(th) < -0.9


def test_spa_geometry():
    tr = PRESETS["Spa-Francorchamps (F1)"]()
    assert tr.closed and len(tr.segments) > 100
    assert tr.length == pytest.approx(7004, abs=10)
    turn = sum(sg.length * sg.curvature for sg in tr.segments)
    assert math.degrees(turn) == pytest.approx(-360, abs=0.01)  # clockwise circuit
    assert min(abs(sg.radius) for sg in tr.segments if sg.radius) < 20  # La Source hairpin


def test_static_equilibrium(veh):
    x = veh.initial_state(0.0)
    assert sum(x[7:11]) == pytest.approx(veh.p.m * 9.81)
    dx = veh.deriv(x, [0.0] * 7)
    assert max(abs(v) for v in dx[:11]) < 1e-6


def test_tire_peak_location(veh):
    p = veh.p
    k = np.linspace(0.0, 0.4, 4001)
    fx = [veh.tire(v, 0.0, p.Fz0)[0] for v in k]
    assert k[int(np.argmax(fx))] == pytest.approx(p.kappa_peak, abs=1e-3)
    assert max(fx) == pytest.approx(p.mu_x * p.Fz0, rel=1e-6)


def test_straight_line_symmetry(veh):
    x = veh.initial_state(15.0)
    res = veh.simulate(x, lambda t, x: [0, 0, 10, 10, 10, 10, 0], 1.0)
    xs = res["x"]
    assert np.all(np.abs(xs[:, 1]) < 1e-9) and np.all(np.abs(xs[:, 2]) < 1e-9)
    assert xs[-1, 0] > 15.0  # accelerating
    assert xs[-1, 14] > 0  # energy used


def test_left_steer_turns_left(veh):
    x = veh.initial_state(10.0)
    res = veh.simulate(x, lambda t, x: [0.05, 0.05, 3, 3, 3, 3, 0], 1.0)
    assert res["x"][-1, 2] > 0 and res["y"][-1, 1] > 0
    fz = res["x"][-1, 7:11]
    assert fz[1] > fz[0] and fz[3] > fz[2]  # load moves to the right (outer) wheels


def test_trim_and_linearise(veh):
    x0, u0, info = veh.trim(15.0, 20.0)
    assert info["success"]
    dx = veh.deriv(x0, u0, lambda s: 1 / 20.0)
    assert max(abs(v) for v in dx[:3]) < 1e-6
    A, B, C, D = veh.linearize(x0, u0, 1 / 20.0)
    assert A.shape == (15, 15) and B.shape == (15, 7)
    assert np.all(np.linalg.eigvals(A[:11, :11]).real < 0)


def test_qss_sanity(veh):
    skid = run_qss(veh, PRESETS["Skidpad (R = 9.125 m circle)"]())
    assert 4.3 < skid["lap_time"] < 5.8
    acc = run_qss(veh, PRESETS["Acceleration (75 m)"]())
    assert 3.2 < acc["lap_time"] < 4.5
    assert acc["v"][0] == 0.0


def test_transient_close_to_qss(veh):
    tr = PRESETS["Oval"]()
    r = run_transient(veh, tr, DriverSettings())
    assert r["status"] == "ok"
    assert r["lap_time"] == pytest.approx(r["qss"]["lap_time"], rel=0.05)


def test_rwd_slower_accel():
    awd = run_qss(Vehicle(), PRESETS["Acceleration (75 m)"]())["lap_time"]
    rwd = run_qss(Vehicle(VehicleParams(awd=False)), PRESETS["Acceleration (75 m)"]())["lap_time"]
    assert rwd > awd


def test_track_csv_roundtrip(tmp_path):
    tr = Track([Segment(10), Segment(math.pi * 5, 5), Segment(10), Segment(math.pi * 5, 5)])
    f = tmp_path / "t.csv"
    tr.to_csv(str(f))
    tr2 = Track.from_csv(str(f))
    assert tr2.length == pytest.approx(tr.length)
    assert tr2.closure_error()[0] < 1e-9
