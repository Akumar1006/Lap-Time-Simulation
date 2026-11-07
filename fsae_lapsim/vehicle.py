"""Nonlinear 4-corner planar vehicle model in state-space form  x_dot = f(x, u, kappa(s)).

States (15)
    0  vx      longitudinal velocity (body)            [m/s]
    1  vy      lateral velocity (body)                 [m/s]
    2  r       yaw rate                                [rad/s]
    3-6  w_i   wheel spin speeds FL, FR, RL, RR        [rad/s]
    7-10 Fz_i  normal loads FL, FR, RL, RR (1st-order lag to quasi-static value) [N]
    11 s       distance along track centre line        [m]
    12 n       lateral offset from centre line (+left) [m]
    13 xi      heading relative to track tangent       [rad]
    14 E       battery energy used                     [J]

Inputs (7)
    0-1  delta_FL, delta_FR   front road-wheel steer angles  [rad]
    2-5  Tm_i                 motor shaft torques (FL, FR, RL, RR) [Nm]
    6    T_brake              total friction brake torque (split by brake bias) [Nm]

Body axes: x forward, y left, z up; yaw positive counter-clockwise.
With a zero-curvature track (s, n, xi) are simply the global (X, Y, psi).
"""
from __future__ import annotations

import math

import numpy as np

from .params import VehicleParams

GRAV = 9.81
CORNERS = ("FL", "FR", "RL", "RR")
STATE_NAMES = ["vx", "vy", "r", "w_FL", "w_FR", "w_RL", "w_RR",
               "Fz_FL", "Fz_FR", "Fz_RL", "Fz_RR", "s", "n", "xi", "E"]
STATE_UNITS = ["m/s", "m/s", "rad/s", "rad/s", "rad/s", "rad/s", "rad/s",
               "N", "N", "N", "N", "m", "m", "rad", "J"]
INPUT_NAMES = ["delta_FL", "delta_FR", "Tm_FL", "Tm_FR", "Tm_RL", "Tm_RR", "T_brake"]
INPUT_UNITS = ["rad", "rad", "Nm", "Nm", "Nm", "Nm", "Nm"]
OUTPUT_NAMES = ["ax", "ay", "beta", "kappa_FL", "kappa_FR", "kappa_RL", "kappa_RR",
                "alpha_FL", "alpha_FR", "alpha_RL", "alpha_RR"]
NX, NU = len(STATE_NAMES), len(INPUT_NAMES)
DYN_STATES = list(range(11))  # states that describe vehicle dynamics (exclude s, n, xi, E)


def _zero_curv(_s: float) -> float:
    return 0.0


class Vehicle:
    def __init__(self, params: VehicleParams | None = None):
        self.p = params or VehicleParams()
        self.update()

    # ------------------------------------------------------------------ setup
    def update(self) -> None:
        """Recompute derived constants; call after editing ``self.p``."""
        p = self.p
        errs = p.validate()
        if errs:
            raise ValueError("; ".join(errs))
        L = p.wheelbase
        self.a = L * (1.0 - p.wd_front)          # CG -> front axle
        self.b = L * p.wd_front                  # CG -> rear axle
        hf, hr = p.track_f / 2.0, p.track_r / 2.0
        self.xw = (self.a, self.a, -self.b, -self.b)
        self.yw = (hf, -hf, hr, -hr)
        self.driven = (bool(p.awd), bool(p.awd), True, True)
        G = p.gear_ratio
        self.Iw_eff = tuple(p.I_w + (p.I_motor * G * G if d else 0.0) for d in self.driven)
        self.Bxn = math.tan(math.pi / (2.0 * p.C_x))
        self.Byn = math.tan(math.pi / (2.0 * p.C_y))
        self.alpha_peak = math.radians(p.alpha_peak_deg)
        self.q = 0.5 * p.rho
        self.wm_max = p.rpm_max * 2.0 * math.pi / 60.0
        self.v_top = self.wm_max / G * p.R_w
        bf = p.brake_bias_front
        self.bias = (bf / 2, bf / 2, (1 - bf) / 2, (1 - bf) / 2)
        W = p.m * GRAV
        self.Fz_static = (W * self.b / L / 2, W * self.b / L / 2, W * self.a / L / 2, W * self.a / L / 2)
        self.delta_max = math.radians(p.delta_max_deg)

    # ------------------------------------------------------------------ tyres
    def tire_peak(self, Fz: float) -> tuple[float, float]:
        """Peak longitudinal / lateral force capacity at load Fz (load-sensitive friction)."""
        p = self.p
        if Fz <= 0.0:
            return 0.0, 0.0
        sc = max(0.1, 1.0 + p.load_sens * (Fz - p.Fz0) / p.Fz0)
        return p.mu_x * sc * Fz, p.mu_y * sc * Fz

    def tire(self, kappa: float, alpha: float, Fz: float) -> tuple[float, float]:
        """Combined-slip tyre forces in the wheel frame (normalised-slip Magic Formula).

        Pure slip: F = D sin(C atan(B_n * slip/slip_peak)), B_n chosen so the peak is at slip_peak.
        Combined slip: both curves are evaluated at the resultant normalised slip rho, which
        gives a friction ellipse at saturation.
        """
        p = self.p
        if Fz <= 0.0:
            return 0.0, 0.0
        sc = 1.0 + p.load_sens * (Fz - p.Fz0) / p.Fz0
        if sc < 0.1:
            sc = 0.1
        Dx = p.mu_x * sc * Fz
        Dy = p.mu_y * sc * Fz
        sx = kappa / p.kappa_peak
        sy = alpha / self.alpha_peak
        rho = math.sqrt(sx * sx + sy * sy)
        if rho < 1e-9:
            return Dx * p.C_x * self.Bxn * sx, Dy * p.C_y * self.Byn * sy
        fx = math.sin(p.C_x * math.atan(self.Bxn * rho)) / rho
        fy = math.sin(p.C_y * math.atan(self.Byn * rho)) / rho
        return Dx * fx * sx, Dy * fy * sy

    # ------------------------------------------------------------------ powertrain
    def motor_torque_limits(self, w_wheel: float) -> tuple[float, float]:
        """(T_min, T_max) available at the motor shaft for a given wheel speed."""
        p = self.p
        wm = abs(w_wheel) * p.gear_ratio
        tmax = p.T_motor_max
        pmax = p.P_motor_max_kW * 1e3
        if wm * tmax > pmax:
            tmax = pmax / wm
        tmin = -tmax * p.regen_frac if p.regen else 0.0
        fade = 0.98 * self.wm_max
        if wm > fade:  # torque fades to zero at max rpm
            tmax *= max(0.0, (self.wm_max - wm) / (self.wm_max - fade))
        return tmin, tmax

    def saturate_motors(self, Tcmd, w) -> tuple[list[float], list[float]]:
        """Apply per-motor torque/power/speed limits and the accumulator power limit.

        Returns (motor torques [Nm], electrical power per motor [W]).
        """
        p = self.p
        G = p.gear_ratio
        T = [0.0] * 4
        P = [0.0] * 4
        ptot = 0.0
        for i in range(4):
            if not self.driven[i]:
                continue
            tmin, tmax = self.motor_torque_limits(w[i])
            t = Tcmd[i]
            t = tmax if t > tmax else (tmin if t < tmin else t)
            T[i] = t
            pm = t * w[i] * G
            P[i] = pm / p.eta_motor if pm > 0 else pm * p.eta_motor
            if P[i] > 0:
                ptot += P[i]
        plim = p.P_total_max_kW * 1e3
        if ptot > plim:
            k = plim / ptot
            for i in range(4):
                if P[i] > 0:
                    T[i] *= k
                    P[i] *= k
        return T, P

    # ------------------------------------------------------------------ steering
    def steer_split(self, delta: float) -> tuple[float, float]:
        """Map a bicycle-model steer angle to (delta_FL, delta_FR) using the Ackermann factor."""
        if abs(delta) < 1e-12:
            return 0.0, 0.0
        L = self.p.wheelbase
        h = self.p.track_f / 2.0
        t = math.tan(delta)
        dl = math.atan(L * t / (L - h * t)) if abs(L - h * t) > 1e-9 else math.copysign(math.pi / 2, delta)
        dr = math.atan(L * t / (L + h * t))
        A = self.p.ackermann
        return delta + A * (dl - delta), delta + A * (dr - delta)

    # ------------------------------------------------------------------ loads
    def fz_steady(self, vx: float, ax: float, ay: float) -> tuple[float, float, float, float]:
        """Quasi-static normal loads incl. aero, drag pitch, longitudinal and lateral transfer."""
        p = self.p
        L = p.wheelbase
        qv = self.q * vx * vx
        down = qv * p.ClA
        drag = qv * p.CdA
        lf = down * p.aero_bal_front / 2.0
        lr = down * (1.0 - p.aero_bal_front) / 2.0
        dlong = (p.m * ax * p.h_cg + drag * p.h_cp) / L / 2.0
        mayh = p.m * ay * p.h_cg
        dyf = p.lltd_front * mayh / p.track_f
        dyr = (1.0 - p.lltd_front) * mayh / p.track_r
        s = self.Fz_static
        return (s[0] + lf - dlong - dyf, s[1] + lf - dlong + dyf,
                s[2] + lr + dlong - dyr, s[3] + lr + dlong + dyr)

    # ------------------------------------------------------------------ dynamics
    def deriv(self, x, u, curv_fn=_zero_curv, aux: dict | None = None) -> list[float]:
        """State derivative x_dot = f(x, u). ``curv_fn(s)`` gives the track curvature [1/m]."""
        p = self.p
        vx, vy, r = x[0], x[1], x[2]
        w = x[3:7]
        Fz = x[7:11]
        n, xi = x[12], x[13]
        Tm, Pel = self.saturate_motors(u[2:6], w)
        Tb = u[6]
        Tb = 0.0 if Tb < 0 else (p.T_brake_max if Tb > p.T_brake_max else Tb)
        dmax = self.delta_max
        d0 = max(-dmax, min(dmax, u[0]))
        d1 = max(-dmax, min(dmax, u[1]))
        deltas = (d0, d1, 0.0, 0.0)
        R = p.R_w
        G = p.gear_ratio * p.eta_gear
        vmin = p.v_slip_min
        sFx = sFy = Mz = 0.0
        dw = [0.0] * 4
        if aux is not None:
            aux.update(kappa=[0.0] * 4, alpha=[0.0] * 4, Fx=[0.0] * 4, Fy=[0.0] * 4,
                       Fxw=[0.0] * 4, Fyw=[0.0] * 4, Tm=Tm, Pel=Pel, delta=(d0, d1))
        for i in range(4):
            vxw = vx - r * self.yw[i]
            vyw = vy + r * self.xw[i]
            d = deltas[i]
            if d != 0.0:
                c, sn = math.cos(d), math.sin(d)
                vl = vxw * c + vyw * sn
                vt = -vxw * sn + vyw * c
            else:
                c, sn = 1.0, 0.0
                vl, vt = vxw, vyw
            den = abs(vl)
            if den < vmin:
                den = vmin
            kap = (w[i] * R - vl) / den
            alp = -math.atan(vt / den)
            fz = Fz[i] if Fz[i] > 0.0 else 0.0
            fxw, fyw = self.tire(kap, alp, fz)
            Fx = fxw * c - fyw * sn
            Fy = fxw * sn + fyw * c
            sFx += Fx
            sFy += Fy
            Mz += self.xw[i] * Fy - self.yw[i] * Fx
            wi = w[i]
            sgn = 1.0 if wi > 1.0 else (-1.0 if wi < -1.0 else wi)
            dw[i] = (Tm[i] * G - Tb * self.bias[i] * sgn - fxw * R - p.Crr * fz * R * sgn) / self.Iw_eff[i]
            if aux is not None:
                aux["kappa"][i], aux["alpha"][i] = kap, alp
                aux["Fx"][i], aux["Fy"][i] = Fx, Fy
                aux["Fxw"][i], aux["Fyw"][i] = fxw, fyw
        drag = self.q * p.CdA * vx * abs(vx)
        ax = (sFx - drag) / p.m
        ay = sFy / p.m
        fss = self.fz_steady(vx, ax, ay)
        itau = 1.0 / p.tau_fz
        curv = curv_fn(x[11])
        den = 1.0 - n * curv
        if den < 0.05:
            den = 0.05
        cx, sx = math.cos(xi), math.sin(xi)
        sdot = (vx * cx - vy * sx) / den
        if aux is not None:
            aux.update(ax=ax, ay=ay, drag=drag, Mz=Mz, curv=curv, Fz_ss=fss,
                       beta=math.atan2(vy, max(abs(vx), 1e-3)), Tb=Tb)
        return [
            ax + r * vy,
            ay - r * vx,
            Mz / p.Iz,
            dw[0], dw[1], dw[2], dw[3],
            (fss[0] - Fz[0]) * itau, (fss[1] - Fz[1]) * itau,
            (fss[2] - Fz[2]) * itau, (fss[3] - Fz[3]) * itau,
            sdot,
            vx * sx + vy * cx,
            r - curv * sdot,
            Pel[0] + Pel[1] + Pel[2] + Pel[3],
        ]

    def outputs(self, x, u, curv_fn=_zero_curv) -> list[float]:
        aux: dict = {}
        self.deriv(x, u, curv_fn, aux)
        return [aux["ax"], aux["ay"], aux["beta"], *aux["kappa"], *aux["alpha"]]

    # ------------------------------------------------------------------ helpers
    def initial_state(self, v0: float) -> list[float]:
        """Straight-line rolling state at speed v0 with settled normal loads."""
        fz = self.fz_steady(v0, 0.0, 0.0)
        w0 = v0 / self.p.R_w
        return [v0, 0.0, 0.0, w0, w0, w0, w0, *fz, 0.0, 0.0, 0.0, 0.0]

    def stable_dt(self, x) -> float:
        """Largest RK4 step that keeps the (stiff) wheel-slip modes stable."""
        p = self.p
        lam = 1.0
        v = max(abs(x[0]), p.v_slip_min)
        for i in range(4):
            dx, _ = self.tire_peak(max(x[7 + i], 0.0))
            ck = dx * p.C_x * self.Bxn / p.kappa_peak
            lam = max(lam, ck * p.R_w ** 2 / (self.Iw_eff[i] * v))
        lam = max(lam, 1.0 / p.tau_fz)
        return 2.0 / lam

    def rk4_step(self, x, u, dt, curv_fn=_zero_curv) -> list[float]:
        f = self.deriv
        k1 = f(x, u, curv_fn)
        x2 = [a + 0.5 * dt * b for a, b in zip(x, k1)]
        k2 = f(x2, u, curv_fn)
        x3 = [a + 0.5 * dt * b for a, b in zip(x, k2)]
        k3 = f(x3, u, curv_fn)
        x4 = [a + dt * b for a, b in zip(x, k3)]
        k4 = f(x4, u, curv_fn)
        return [a + dt / 6.0 * (b + 2 * c + 2 * d + e) for a, b, c, d, e in zip(x, k1, k2, k3, k4)]

    def simulate(self, x0, u_fn, t_end: float, dt: float = 1e-3, curv_fn=_zero_curv, log_dt: float = 0.01):
        """Open-loop simulation. ``u_fn(t, x)`` returns the 7-element input vector."""
        x = list(x0)
        t = 0.0
        log_t, log_x, log_u, log_y = [], [], [], []
        next_log = 0.0
        while t <= t_end + 1e-12:
            u = list(u_fn(t, x))
            if t >= next_log - 1e-12:
                aux: dict = {}
                self.deriv(x, u, curv_fn, aux)
                log_t.append(t)
                log_x.append(list(x))
                log_u.append([*aux["delta"], *aux["Tm"], aux["Tb"]])
                log_y.append([aux["ax"], aux["ay"], aux["beta"], *aux["kappa"], *aux["alpha"],
                              *aux["Fx"], *aux["Fy"], *aux["Pel"]])
                next_log += log_dt
            h = min(dt, self.stable_dt(x))
            x = self.rk4_step(x, u, h, curv_fn)
            t += h
            if not all(math.isfinite(v) for v in x):
                break
        return dict(t=np.array(log_t), x=np.array(log_x), u=np.array(log_u), y=np.array(log_y))

    # ------------------------------------------------------------------ linear analysis
    def linearize(self, x0, u0, curv: float = 0.0):
        """Central-difference Jacobians: A = df/dx, B = df/du, C = dh/dx, D = dh/du."""
        x0 = np.asarray(x0, float)
        u0 = np.asarray(u0, float)
        cf = (lambda _s: curv)

        def f(x, u):
            return np.array(self.deriv(list(x), list(u), cf))

        def h(x, u):
            return np.array(self.outputs(list(x), list(u), cf))

        A = np.zeros((NX, NX))
        C = np.zeros((len(OUTPUT_NAMES), NX))
        for j in range(NX):
            e = 1e-6 * max(1.0, abs(x0[j]))
            xp, xm = x0.copy(), x0.copy()
            xp[j] += e
            xm[j] -= e
            A[:, j] = (f(xp, u0) - f(xm, u0)) / (2 * e)
            C[:, j] = (h(xp, u0) - h(xm, u0)) / (2 * e)
        B = np.zeros((NX, NU))
        D = np.zeros((len(OUTPUT_NAMES), NU))
        for j in range(NU):
            e = 1e-6 * max(1.0, abs(u0[j]))
            up, um = u0.copy(), u0.copy()
            up[j] += e
            um[j] -= e
            B[:, j] = (f(x0, up) - f(x0, um)) / (2 * e)
            D[:, j] = (h(x0, up) - h(x0, um)) / (2 * e)
        return A, B, C, D

    def torque_split(self, T_total: float) -> list[float]:
        """Split a total motor torque over the driven motors in proportion to static load."""
        fz = [self.Fz_static[i] if self.driven[i] else 0.0 for i in range(4)]
        s = sum(fz)
        return [T_total * f / s for f in fz]

    def trim(self, v: float, radius: float):
        """Steady-state cornering (or straight-line if radius == 0) trim at forward speed v.

        Unknowns: vy, r, w_i, Fz_i, steer, total motor torque.
        Returns (x0, u0, info) where info has 'success', 'residual', 'delta', 'T_total'.
        """
        from scipy.optimize import least_squares

        p = self.p
        curv = 0.0 if radius == 0 else 1.0 / radius
        zero = _zero_curv

        def build(z):
            x = [v, z[0], z[1], *z[2:6], *z[6:10], 0.0, 0.0, 0.0, 0.0]
            dl, dr = self.steer_split(z[10])
            u = [dl, dr, *self.torque_split(z[11]), 0.0]
            return x, u

        def resid(z):
            x, u = build(z)
            dx = self.deriv(x, u, zero)
            V = math.hypot(v, z[0])
            res = [dx[0], dx[1], dx[2]]
            res += [d * 0.01 for d in dx[3:7]]
            res += [d * p.tau_fz / 100.0 for d in dx[7:11]]
            res.append((z[1] - V * curv) * 10.0)
            return res

        ay = v * v * curv
        fz0 = self.fz_steady(v, 0.0, ay)
        drag = self.q * p.CdA * v * v
        T0 = (drag + p.Crr * p.m * GRAV) * p.R_w / (p.gear_ratio * p.eta_gear)
        z0 = [0.0, v * curv, *([v / p.R_w] * 4), *fz0, p.wheelbase * curv, T0]
        sol = least_squares(resid, z0, method="lm", xtol=1e-12, ftol=1e-12, max_nfev=4000)
        x, u = build(sol.x)
        res = float(np.max(np.abs(sol.fun)))
        return x, u, dict(success=res < 1e-4, residual=res, delta=sol.x[10], T_total=sol.x[11])
