"""Transient lap simulation: closed-loop driver + torque allocation around the 15-state model."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np

from .qss import run_qss
from .track import Track
from .vehicle import Vehicle


@dataclass
class DriverSettings:
    preview_time: float = 0.45      # pure-pursuit look-ahead time [s]
    preview_min: float = 3.0        # minimum look-ahead distance [m]
    k_yaw: float = 0.08             # steer feedback on yaw-rate error [rad per rad/s]
    speed_scale: float = 1.0        # fraction of QSS speed profile used as target [-]
    speed_lookahead: float = 0.12   # time preview on speed target [s]
    k_speed: float = 4.0            # speed-error gain [1/s]
    torque_vectoring: bool = True
    k_tv: float = 8.0               # yaw-rate feedback gain for torque vectoring [1/s]
    traction_control: bool = True
    slip_target: float = 0.8        # slip-ratio target as fraction of kappa_peak [-]
    dt: float = 1e-3                # max integration step [s]
    log_dt: float = 0.01            # logging interval [s]


class Driver:
    """Path + speed tracking driver with per-wheel torque allocation."""

    def __init__(self, veh: Vehicle, track: Track, qss: dict, st: DriverSettings):
        self.veh, self.track, self.st = veh, track, st
        self.q_s = qss["s"]
        self.q_v = qss["v"]
        self.q_ax = qss["ax"]

    def _profile(self, s):
        tr = self.track
        if tr.closed:
            s = s % tr.length
        else:
            s = min(s, tr.length)
        return (float(np.interp(s, self.q_s, self.q_v)), float(np.interp(s, self.q_s, self.q_ax)))

    def control(self, x):
        veh, p, st, tr = self.veh, self.veh.p, self.st, self.track
        vx, vy, r = x[0], x[1], x[2]
        s, n, xi = x[11], x[12], x[13]
        # ---- lateral: pure pursuit on the centre line + yaw-rate feedback
        Xc, Yc, th = tr.pose_at(s)
        X, Y, psi = Xc - n * math.sin(th), Yc + n * math.cos(th), th + xi
        Lp = max(st.preview_min, st.preview_time * vx)
        Xt, Yt, _ = tr.pose_at(s + Lp)
        dx, dy = Xt - X, Yt - Y
        yl = -math.sin(psi) * dx + math.cos(psi) * dy
        ld2 = max(dx * dx + dy * dy, 1e-6)
        k_path = 2.0 * yl / ld2
        r_ref = vx * k_path
        delta = math.atan(p.wheelbase * k_path) + st.k_yaw * (r_ref - r)
        delta = max(-veh.delta_max, min(veh.delta_max, delta))
        d_fl, d_fr = veh.steer_split(delta)
        # ---- longitudinal: track the (scaled) QSS speed profile
        v_ref, a_ref = self._profile(s + vx * st.speed_lookahead)
        v_ref *= st.speed_scale
        a_ref *= st.speed_scale ** 2
        drag = veh.q * p.CdA * vx * vx
        Fz = [max(f, 1.0) for f in x[7:11]]
        F_req = p.m * (a_ref + st.k_speed * (v_ref - vx)) + drag + p.Crr * sum(Fz)
        # ---- yaw moment demand (torque vectoring)
        Mz = st.k_tv * p.Iz * (r_ref - r) if st.torque_vectoring else 0.0
        dF = Mz / (p.track_f + p.track_r)
        side = (-1.0, 1.0, -1.0, 1.0)
        R = p.R_w
        G = p.gear_ratio * p.eta_gear
        Tm = [0.0] * 4
        T_brake = 0.0
        if F_req >= 0:
            drv = [Fz[i] if veh.driven[i] else 0.0 for i in range(4)]
            sd = sum(drv)
            for i in range(4):
                if veh.driven[i]:
                    Tm[i] = (F_req * drv[i] / sd + side[i] * dF) * R / G
        else:
            tot = sum(Fz)
            Fx_des = [F_req * Fz[i] / tot + side[i] * dF for i in range(4)]
            # regen capacity per wheel, friction brake covers the rest (split by fixed bias)
            regen = [0.0] * 4
            for i in range(4):
                if veh.driven[i] and p.regen:
                    tmin, _ = veh.motor_torque_limits(x[3 + i])
                    regen[i] = -tmin * G / R
            deficit = sum(max(0.0, -Fx_des[i] - regen[i]) for i in range(4))
            T_brake = min(deficit * R, p.T_brake_max)
            for i in range(4):
                F_mech = T_brake * veh.bias[i] / R
                if veh.driven[i]:
                    Tm[i] = (Fx_des[i] + F_mech) * R / G
        # ---- traction control / slip limiting (acts on motor torque and friction brake)
        if st.traction_control:
            kt = st.slip_target * p.kappa_peak
            worst_brake = 1.0
            for i in range(4):
                vl = max(abs(vx - r * veh.yw[i]), p.v_slip_min)
                kap = (x[3 + i] * R - (vx - r * veh.yw[i])) / vl
                if kap > kt and Tm[i] > 0:
                    Tm[i] *= max(0.0, 1.0 - 4.0 * (kap - kt) / p.kappa_peak)
                elif kap < -kt:
                    f = max(0.0, 1.0 - 4.0 * (-kap - kt) / p.kappa_peak)
                    if Tm[i] < 0:
                        Tm[i] *= f
                    worst_brake = min(worst_brake, f)
            T_brake *= worst_brake
        return [d_fl, d_fr, Tm[0], Tm[1], Tm[2], Tm[3], T_brake], dict(v_ref=v_ref, delta=delta)


def run_transient(veh: Vehicle, track: Track, st: DriverSettings | None = None, qss: dict | None = None,
                  standing_start: bool | None = None, progress=None, cancel=None) -> dict:
    """Integrate one lap of the full nonlinear state-space model with the closed-loop driver.

    ``progress(frac)`` is called periodically; ``cancel()`` returning True aborts.
    """
    st = st or DriverSettings()
    if standing_start is None:
        standing_start = not track.closed
    qss = qss or run_qss(veh, track, standing_start=standing_start)
    drv = Driver(veh, track, qss, st)
    L = track.length
    v0 = 0.0 if standing_start else float(qss["v"][0]) * st.speed_scale
    x = veh.initial_state(max(v0, 0.5))
    curv = track.curvature_at
    t = 0.0
    t_max = max(3.0 * qss["lap_time"], 10.0)
    logs = {k: [] for k in ("t", "x", "u", "y", "v_ref")}
    next_log = 0.0
    status = "ok"
    wall0 = time.time()
    last_prog = 0.0
    while True:
        u, info = drv.control(x)
        if t >= next_log - 1e-12:
            aux: dict = {}
            veh.deriv(x, u, curv, aux)
            logs["t"].append(t)
            logs["x"].append(list(x))
            logs["u"].append([*aux["delta"], *aux["Tm"], aux["Tb"]])
            logs["y"].append([aux["ax"], aux["ay"], aux["beta"], *aux["kappa"], *aux["alpha"],
                              *aux["Fx"], *aux["Fy"], *aux["Pel"]])
            logs["v_ref"].append(info["v_ref"])
            next_log += st.log_dt
        h = min(st.dt, veh.stable_dt(x))
        x_new = veh.rk4_step(x, u, h, curv)
        if x_new[11] >= L:  # interpolate the finish-line crossing
            frac = (L - x[11]) / max(x_new[11] - x[11], 1e-12)
            x = [a + frac * (b - a) for a, b in zip(x, x_new)]
            t += frac * h
            break
        x = x_new
        t += h
        if progress and t - last_prog > 0.5:
            last_prog = t
            progress(min(x[11] / L, 1.0))
        if cancel and cancel():
            status = "cancelled"
            break
        if not all(math.isfinite(v) for v in x):
            status = "diverged (non-finite state)"
            break
        if abs(x[13]) > 1.3 or abs(x[12]) > 12.0:
            status = f"vehicle left the track at s = {x[11]:.1f} m (|n| or heading error too large)"
            break
        if t > t_max:
            status = "time limit exceeded"
            break
        if t > 2.0 and x[0] < 0.3:
            status = "vehicle stopped"
            break
    # final sample
    u, info = drv.control(x)
    aux = {}
    veh.deriv(x, u, curv, aux)
    logs["t"].append(t)
    logs["x"].append(list(x))
    logs["u"].append([*aux["delta"], *aux["Tm"], aux["Tb"]])
    logs["y"].append([aux["ax"], aux["ay"], aux["beta"], *aux["kappa"], *aux["alpha"],
                      *aux["Fx"], *aux["Fy"], *aux["Pel"]])
    logs["v_ref"].append(info["v_ref"])
    if progress:
        progress(1.0)
    X = np.array(logs["x"])
    gx, gy, gpsi = track.to_global(X[:, 11], X[:, 12], X[:, 13])
    return dict(method="Transient", status=status, lap_time=t if status == "ok" else float("nan"),
                t=np.array(logs["t"]), x=X, u=np.array(logs["u"]), y=np.array(logs["y"]),
                v_ref=np.array(logs["v_ref"]), X=gx, Y=gy, psi=gpsi, qss=qss,
                wall_time=time.time() - wall0)


# column indices of the transient 'y' log
Y_COLS = (["ax", "ay", "beta"] + [f"kappa_{c}" for c in ("FL", "FR", "RL", "RR")]
          + [f"alpha_{c}" for c in ("FL", "FR", "RL", "RR")]
          + [f"Fx_{c}" for c in ("FL", "FR", "RL", "RR")]
          + [f"Fy_{c}" for c in ("FL", "FR", "RL", "RR")]
          + [f"Pel_{c}" for c in ("FL", "FR", "RL", "RR")])
