"""Quasi-steady-state (QSS) lap simulation using a GGV envelope derived from the 4-corner model.

For each (speed, lateral acceleration) the four normal loads are solved together with the
longitudinal acceleration (fixed-point iteration). Each tyre gets a friction ellipse based on its
own load; lateral force is shared between axles by yaw-moment balance, longitudinal force is
limited per wheel by the remaining grip, the motor torque/power/speed curve and the accumulator
power limit.
"""
from __future__ import annotations

import math

import numpy as np

from .track import Track
from .vehicle import GRAV, Vehicle


class GGV:
    def __init__(self, veh: Vehicle, n_v: int = 45, n_ay: int = 25):
        self.veh = veh
        p = veh.p
        self.v_max = veh.v_top
        self.v = np.linspace(0.0, self.v_max, n_v)
        self.frac = np.linspace(0.0, 1.0, n_ay)
        self.ay_max = np.array([self._ay_max(v) for v in self.v])
        self.ax_acc = np.zeros((n_v, n_ay))
        self.ax_brk = np.zeros((n_v, n_ay))
        for i, v in enumerate(self.v):
            for j, f in enumerate(self.frac):
                ay = f * self.ay_max[i]
                self.ax_acc[i, j] = self.ax_limit(v, ay, +1)[0]
                self.ax_brk[i, j] = self.ax_limit(v, ay, -1)[0]
        self._m = p.m
        # plain-Python copies of the uniform tables for fast scalar lookups in the QSS passes
        self._nv, self._nay = n_v, n_ay
        self._dv = self.v_max / (n_v - 1)
        self._aym_l = self.ay_max.tolist()
        self._acc_l = self.ax_acc.tolist()
        self._brk_l = self.ax_brk.tolist()

    # ------------------------------------------------------------------ core
    def _wheel_split(self, v, ax, ay):
        veh = self.veh
        p = veh.p
        fz = [max(f, 0.0) for f in veh.fz_steady(v, ax, ay)]
        caps = [veh.tire_peak(f) for f in fz]
        Fy_tot = p.m * ay
        Fy_ax = (Fy_tot * veh.b / p.wheelbase, Fy_tot * veh.a / p.wheelbase)
        util = [0.0] * 4
        for k, (i, j) in enumerate(((0, 1), (2, 3))):
            dy = caps[i][1] + caps[j][1]
            if dy <= 0:
                return fz, caps, None
            for w in (i, j):
                util[w] = abs(Fy_ax[k]) / dy  # shared in proportion to capacity -> equal use
        return fz, caps, util

    def ax_limit(self, v, ay, sign):
        """Max acceleration (sign=+1) or max deceleration (sign=-1) at speed v and lateral acc ay.

        Returns (ax, feasible, Fz tuple).
        """
        veh = self.veh
        p = veh.p
        ax = 0.0
        drag = veh.q * p.CdA * v * v
        feasible = True
        fz = veh.fz_steady(v, 0, ay)
        for _ in range(12):
            fz, caps, util = self._wheel_split(v, ax, ay)
            if util is None or max(util) > 1.0 + 1e-9:
                feasible = False
                util = [min(u, 1.0) for u in (util or [1.0] * 4)]
            fx_av = [caps[i][0] * math.sqrt(max(0.0, 1.0 - util[i] ** 2)) for i in range(4)]
            rr = p.Crr * sum(fz)
            if sign > 0:
                w = v / p.R_w
                Fx = 0.0
                for i in range(4):
                    if not veh.driven[i]:
                        continue
                    _, tmax = veh.motor_torque_limits(w)
                    Fx += min(fx_av[i], tmax * p.gear_ratio * p.eta_gear / p.R_w)
                if v > 0.1:
                    Fx = min(Fx, p.P_total_max_kW * 1e3 * p.eta_motor * p.eta_gear / v)
            else:
                Fx = -sum(fx_av)
            ax_new = (Fx - drag - rr) / p.m
            if abs(ax_new - ax) < 1e-4:
                ax = ax_new
                break
            ax = 0.5 * ax + 0.5 * ax_new
        return ax, feasible, fz

    def _ay_max(self, v):
        lo, hi = 0.0, 5.0 * GRAV
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            _, _, util = self._wheel_split(v, 0.0, mid)
            if util is not None and max(util) <= 1.0:
                lo = mid
            else:
                hi = mid
        return lo

    # ------------------------------------------------------------------ lookups
    def ay_max_at(self, v):
        return np.interp(v, self.v, self.ay_max)

    def ax_at(self, v, ay, sign):
        """Bilinear lookup in (v, ay/ay_max) of the GGV table (uniform grids, scalar fast path)."""
        v = min(max(v, 0.0), self.v_max)
        fi = v / self._dv
        i0 = min(int(fi), self._nv - 2)
        t = fi - i0
        aym = self._aym_l
        aymax = (1 - t) * aym[i0] + t * aym[i0 + 1]
        f = min(abs(ay) / aymax, 1.0) if aymax > 0 else 1.0
        tab = self._acc_l if sign > 0 else self._brk_l
        fj = f * (self._nay - 1)
        j0 = min(int(fj), self._nay - 2)
        w = fj - j0
        r0, r1 = tab[i0], tab[i0 + 1]
        a0 = (1 - w) * r0[j0] + w * r0[j0 + 1]
        a1 = (1 - w) * r1[j0] + w * r1[j0 + 1]
        return (1 - t) * a0 + t * a1

    def apex_speed(self, kappa: np.ndarray) -> np.ndarray:
        vf = np.linspace(0.0, self.v_max, 600)
        aym = self.ay_max_at(vf)
        kabs = np.abs(np.asarray(kappa, float))
        # segment-based tracks only contain a few distinct curvatures
        uniq, inv = np.unique(kabs, return_inverse=True)
        vu = np.full(len(uniq), self.v_max)
        for k, kp in enumerate(uniq):
            if kp < 1e-9:
                continue
            idx = np.nonzero(aym - vf * vf * kp >= 0)[0]
            vu[k] = vf[idx[-1]] if len(idx) else vf[1]
        return vu[inv]


def run_qss(veh: Vehicle, track: Track, standing_start: bool | None = None, ds: float = 0.5,
            ggv: GGV | None = None) -> dict:
    """Forward/backward-integration QSS lap. Closed tracks are solved as a flying lap."""
    p = veh.p
    ggv = ggv or GGV(veh)
    if standing_start is None:
        standing_start = not track.closed
    L = track.length
    N = max(2, int(math.ceil(L / ds)) + 1)
    s = np.linspace(0.0, L, N)
    h = s[1] - s[0]
    kap = np.array([track.curvature_at(min(v, L - 1e-9)) for v in s])
    laps = 3 if (track.closed and not standing_start) else 1
    kk = np.concatenate([kap] + [kap[1:]] * (laps - 1))
    vap = ggv.apex_speed(kk)
    M = len(kk)
    v = vap.copy()
    # forward pass (acceleration limited)
    v[0] = 0.0 if standing_start else min(vap[0], ggv.v_max)
    for i in range(M - 1):
        ax = ggv.ax_at(v[i], v[i] ** 2 * kk[i], +1)
        v_next = math.sqrt(max(v[i] ** 2 + 2.0 * ax * h, 0.0))
        v[i + 1] = min(vap[i + 1], v_next)
    # backward pass (braking limited)
    for i in range(M - 1, 0, -1):
        ax = ggv.ax_at(v[i], v[i] ** 2 * kk[i], -1)
        v_prev = math.sqrt(max(v[i] ** 2 - 2.0 * ax * h, 0.0))
        if v_prev < v[i - 1]:
            v[i - 1] = v_prev
    if laps == 3:
        v = v[N - 1:2 * N - 1].copy()
        kk = kk[N - 1:2 * N - 1]
        vap = vap[N - 1:2 * N - 1]
    else:
        vap = vap[:N]
    vm = np.maximum(0.5 * (v[1:] + v[:-1]), 1e-3)
    dt = h / vm
    t = np.concatenate([[0.0], np.cumsum(dt)])
    ax = np.zeros(N)
    ax[:-1] = (v[1:] ** 2 - v[:-1] ** 2) / (2 * h)
    ax[-1] = ax[-2]
    ay = v ** 2 * kk
    fz = np.array([veh.fz_steady(v[i], ax[i], ay[i]) for i in range(N)])
    drag = veh.q * p.CdA * v ** 2
    F_trac = p.m * ax + drag + p.Crr * fz.sum(axis=1)
    P_wheel = F_trac * v
    eta = p.eta_gear * p.eta_motor
    P_el = np.where(P_wheel > 0, P_wheel / eta, (P_wheel * eta) if p.regen else 0.0)
    E = np.concatenate([[0.0], np.cumsum(0.5 * (P_el[1:] + P_el[:-1]) * dt)])
    X, Y, _ = track.to_global(s, np.zeros(N), np.zeros(N))
    return dict(method="QSS", s=s, v=v, ax=ax, ay=ay, kappa=kk, t=t, lap_time=float(t[-1]),
                Fz=fz, P_el=P_el, E=E, X=X, Y=Y, v_apex=vap, ggv=ggv)
