# `transient.py` – Line-by-Line Code Walkthrough

This document walks through [`fsae_lapsim/transient.py`](./fsae_lapsim/transient.py) line by line. That file runs the transient lap simulation: it puts the 15-state vehicle model in a closed loop with a driver/controller and integrates it over one lap.

For the *physics* of the vehicle model (tyres, load transfer, stiffness, why the states were chosen), see [TRANSIENT_DYNAMICS.md](./TRANSIENT_DYNAMICS.md). This document covers what the *code* does.

Line numbers refer to the current version of the file. Worked numbers use the default car parameters.

---

## Contents

1. [File layout](#1-file-layout)
2. [Imports and `DriverSettings` (L1–28)](#2-imports-and-driversettings-l128)
3. [The `Driver` class (L31–116)](#3-the-driver-class-l31116)
   - [`__init__`](#__init__-l3438)
   - [`_profile`](#_profiles-l4046)
   - [`control`: unpacking the state](#control-unpacking-the-state-l4951)
   - [`control`: steering](#control-steering-pure-pursuit--yaw-feedback-l5364)
   - [`control`: speed target](#control-speed-target-l6671)
   - [`control`: torque-vectoring yaw moment](#control-torque-vectoring-yaw-moment-l7379)
   - [`control`: allocation when driving](#control-torque-allocation-when-driving-l8085)
   - [`control`: allocation when braking](#control-torque-allocation-when-braking-l86100)
   - [`control`: traction control / ABS](#control-traction-control--abs-l102115)
4. [`run_transient`, the simulation loop (L119–197)](#4-run_transient-the-simulation-loop-l119197)
5. [`Y_COLS` (L200–205)](#5-y_cols-l200205)
6. [Known issues and limitations in this file](#6-known-issues-and-limitations-in-this-file)

---

## 1. File layout

The file has four parts:

| Part | Lines | Role |
|---|---|---|
| `DriverSettings` | [L15–28](./fsae_lapsim/transient.py#L15) | The driver's tuning knobs |
| `Driver` | [L31–116](./fsae_lapsim/transient.py#L31) | Reads the state `x`, returns the 7 inputs `u` |
| `run_transient` | [L119–197](./fsae_lapsim/transient.py#L119) | The time loop: integrates the plant over one lap with the driver in the loop |
| `Y_COLS` | [L200–205](./fsae_lapsim/transient.py#L200) | Column names for the logged outputs |

The overall data flow:

```
            ┌───────────── x (15 states) ─────────────┐
            ▼                                          │
  Driver.control(x) ──► u (7 inputs) ──► Vehicle.rk4_step(x, u, h) ──► x_new
  (steer, speed, torque allocation)        (plant, in vehicle.py)
```

---

## 2. Imports and `DriverSettings` (L1–28)

[Jump to code](./fsae_lapsim/transient.py#L1)

```python
from __future__ import annotations      # allows type hints like "DriverSettings | None"
import math, time
from dataclasses import dataclass
import numpy as np
from .qss import run_qss                 # needed for the speed plan
from .track import Track
from .vehicle import Vehicle             # the 15-state plant
```

`@dataclass class DriverSettings` is a plain container. `@dataclass` generates the constructor, so `DriverSettings(k_tv=5)` works. The GUI builds its "Driver / controller" panel by looping over these fields.

| Line | Field | Default | Meaning |
|---|---|---|---|
| 17 | `preview_time` | 0.45 s | Pure-pursuit look-ahead, as time × speed |
| 18 | `preview_min` | 3.0 m | Minimum look-ahead distance, so it doesn't shrink to zero at low speed |
| 19 | `k_yaw` | 0.08 | Extra steer (rad) per rad/s of yaw-rate error |
| 20 | `speed_scale` | 1.0 | Fraction of the QSS speed profile to aim for |
| 21 | `speed_lookahead` | 0.12 s | How far ahead to read the speed target |
| 22 | `k_speed` | 4.0 1/s | Speed-error gain: 1 m/s too slow → ask for 4 m/s² extra |
| 23 | `torque_vectoring` | on | Enable left/right torque difference |
| 24 | `k_tv` | 8.0 1/s | Yaw-rate gain for torque vectoring |
| 25 | `traction_control` | on | Enable slip limiting |
| 26 | `slip_target` | 0.8 | Start cutting torque at 80% of peak slip ratio |
| 27 | `dt` | 1 ms | Largest integration step |
| 28 | `log_dt` | 10 ms | Logging interval (100 Hz) |

---

## 3. The `Driver` class (L31–116)

### `__init__` (L34–38)

[Jump to code](./fsae_lapsim/transient.py#L34)

```python
self.veh, self.track, self.st = veh, track, st
self.q_s  = qss["s"]     # QSS distance grid (0.5 m spacing)
self.q_v  = qss["v"]     # QSS speed at each point
self.q_ax = qss["ax"]    # QSS longitudinal acceleration at each point
```

It stores references and the QSS solution. The QSS speed trace is the driver's plan; the driver never decides speeds itself.

### `_profile(s)` (L40–46)

[Jump to code](./fsae_lapsim/transient.py#L40)

```python
if tr.closed:  s = s % tr.length      # past the line on a circuit → wrap to the start
else:          s = min(s, tr.length)  # past the finish on an open course → hold the last value
return (np.interp(s, q_s, q_v), np.interp(s, q_s, q_ax))
```

It looks up the target speed and acceleration at any distance, linearly interpolating between the 0.5 m QSS grid points. The wrap matters because the look-ahead point (`s + something`) can pass the start/finish line before the car does.

### `control`: unpacking the state (L49–51)

[Jump to code](./fsae_lapsim/transient.py#L48)

`control(x)` is called once per integration step.

```python
veh, p, st, tr = self.veh, self.veh.p, self.st, self.track   # short local names
vx, vy, r = x[0], x[1], x[2]                                  # body velocities, yaw rate
s, n, xi  = x[11], x[12], x[13]                               # distance, lateral offset, heading error
```

The controller reads the true state directly, as if the car had perfect sensors. Wheel speeds `x[3:7]` and loads `x[7:11]` are read further down.

### `control`: steering, pure pursuit + yaw feedback (L53–64)

[Jump to code](./fsae_lapsim/transient.py#L53)

```python
Xc, Yc, th = tr.pose_at(s)                                   # 53
```
- Gets the centreline point and its heading at the car's current distance.

```python
X, Y, psi = Xc - n*sin(th), Yc + n*cos(th), th + xi          # 54
```
- Converts track coordinates (distance, offset, heading error) to global coordinates.
- (−sin θ, cos θ) is the unit vector pointing left of the centreline, so the car is `n` metres along it.
- The car's global heading is the track heading plus the heading error.

```python
Lp = max(st.preview_min, st.preview_time * vx)               # 55
Xt, Yt, _ = tr.pose_at(s + Lp)                               # 56
```
- Look-ahead distance: 6.75 m at 15 m/s, 13.5 m at 30 m/s.
- The target point is that far ahead on the centreline.

```python
dx, dy = Xt - X, Yt - Y                                      # 57
yl  = -sin(psi)*dx + cos(psi)*dy                             # 58
ld2 = max(dx*dx + dy*dy, 1e-6)                               # 59
```
- Line 57 is the vector from the car to the target, in global axes.
- Line 58 rotates it into the car's frame and keeps only the sideways component: how far left of the car's nose the target sits.
- Line 59 is the squared distance to the target. The floor prevents a divide-by-zero.

```python
k_path = 2.0 * yl / ld2                                      # 60
r_ref  = vx * k_path                                         # 61
```
- Line 60 is the pure-pursuit formula. It gives the curvature of the circle that starts at the car, is tangent to its heading, and passes through the target. From geometry, R² = x² + (R − y)², which rearranges to 1/R = 2y/ℓ².
- Line 61: if the car followed that circle at its current speed, its yaw rate would be v·k.

```python
delta = atan(p.wheelbase * k_path) + st.k_yaw * (r_ref - r)  # 62
```
The steer angle has two parts:
- **Feed-forward, `atan(L·k)`:** the kinematic (no-slip) steer angle for curvature k.
- **Feedback, `k_yaw·(r_ref − r)`:** if the car rotates slower than needed (understeer, or yaw still building), add steer; if faster, take some away.

```python
delta = max(-veh.delta_max, min(veh.delta_max, delta))       # 63
d_fl, d_fr = veh.steer_split(delta)                          # 64
```
- Line 63 clips the steer to ±25°.
- Line 64 splits the single steer angle into left and right road-wheel angles using the Ackermann factor. These are inputs `u[0]` and `u[1]`.

**Worked example: centred on a 20 m left-hander at 15 m/s.**

| Quantity | Value |
|---|---|
| Look-ahead `Lp` | max(3, 0.45 × 15) = 6.75 m |
| Target point in car frame | 6.62 m ahead, 1.13 m left |
| `k_path` = 2(1.13)/45.1 | 0.050 1/m = 1/20 exactly |
| Feed-forward steer atan(1.53/20) | 4.38° |
| Steer the car actually needs (from trim) | 4.66° (it understeers) |

The yaw feedback supplies the missing 0.28°. There is no integral term, so the car settles a few centimetres wide of the line. That small offset raises `yl` just enough to make up the missing steer.

### `control`: speed target (L66–71)

[Jump to code](./fsae_lapsim/transient.py#L66)

```python
v_ref, a_ref = self._profile(s + vx * st.speed_lookahead)    # 66
```
- The speed target is read about 0.12 s ahead (3.6 m at 30 m/s).
- Asking for force and getting it takes time (slip build-up, load transfer). Reading ahead makes braking start on time instead of late.

```python
v_ref *= st.speed_scale                                      # 67
a_ref *= st.speed_scale ** 2                                 # 68
```
- Scales the profile. Acceleration is a = v·dv/ds, so scaling speed by σ scales acceleration by σ².

```python
drag = veh.q * p.CdA * vx * vx                               # 69
Fz   = [max(f, 1.0) for f in x[7:11]]                        # 70
F_req = p.m*(a_ref + st.k_speed*(v_ref - vx)) + drag + p.Crr*sum(Fz)   # 71
```
- Line 69 is aero drag, ½ρC_dA·v².
- Line 70 reads the four wheel loads, floored at 1 N so a lifted wheel can't cause a divide-by-zero later.
- Line 71 is the total longitudinal force request:

| Term | Purpose |
|---|---|
| `m·a_ref` | **Feed-forward:** the plan says brake at 2 g here, so ask for that directly |
| `m·k_speed·(v_ref − vx)` | **Proportional correction:** 1 m/s too slow gives 280 × 4 = 1120 N extra |
| `+ drag + rolling resistance` | Cancels the known resistances so the feedback term only handles real error |

Nothing here checks whether `F_req` is achievable. The vehicle model limits it (`saturate_motors` and the tyre model).

### `control`: torque-vectoring yaw moment (L73–79)

[Jump to code](./fsae_lapsim/transient.py#L73)

```python
Mz = st.k_tv * p.Iz * (r_ref - r) if st.torque_vectoring else 0.0   # 73
dF = Mz / (p.track_f + p.track_r)                                    # 74
side = (-1.0, 1.0, -1.0, 1.0)                                        # 75  FL, FR, RL, RR
```

- **Line 73:** the yaw moment requested from left/right torque difference. `k_tv·Iz·error` means "fix the yaw-rate error with a time constant of 1/k_tv = 125 ms".
- **Line 74:** the force added to or removed from each wheel.
- **Line 75:** which side gets it. For a positive (leftward) moment, the right wheels (`+1`) push harder and the left wheels (`−1`) push less.

Checking the sign: a wheel at lateral position y with force F_x gives yaw moment −y·F_x.
- Front pair: −(+t_f/2)(−dF) + −(−t_f/2)(+dF) = t_f·dF
- Rear pair: t_r·dF
- Total = (t_f + t_r)·dF = Mz ✓

**Example:** a 0.05 rad/s yaw error gives Mz = 8 × 110 × 0.05 = 44 N·m, so dF = 44/2.4 = 18 N per wheel, which is 0.29 N·m at each motor. Torque vectoring makes small, continuous corrections.

```python
R = p.R_w;  G = p.gear_ratio * p.eta_gear        # 76–77: force ↔ motor torque conversion
Tm = [0.0]*4;  T_brake = 0.0                     # 78–79: outputs, default zero
```

`G` includes gearbox efficiency, so `Tm = F·R/G` is the exact inverse of how the wheel equation in `vehicle.py` turns motor torque into wheel torque.

### `control`: torque allocation when driving (L80–85)

[Jump to code](./fsae_lapsim/transient.py#L80)

```python
if F_req >= 0:
    drv = [Fz[i] if veh.driven[i] else 0.0 for i in range(4)]   # loads of the motored wheels only
    sd  = sum(drv)
    for i in range(4):
        if veh.driven[i]:
            Tm[i] = (F_req * drv[i]/sd + side[i]*dF) * R / G
```

- Each driven wheel gets a share of `F_req` in proportion to its current load. Grip is roughly μ·F_z, so every tyre uses about the same fraction of its grip and none saturates first.
- The torque-vectoring ± is then added, and the force is converted to motor shaft torque.
- In a tight corner the inner wheel's request can go negative, meaning regen on the inner wheel while driving. That is allowed.

### `control`: torque allocation when braking (L86–100)

[Jump to code](./fsae_lapsim/transient.py#L86)

**Step 1: desired braking force per wheel (L87–88).**

```python
tot = sum(Fz)
Fx_des = [F_req*Fz[i]/tot + side[i]*dF for i in range(4)]
```
This uses all four loads, because every wheel can brake through the friction brakes.

**Step 2: regen capacity per wheel (L90–94).**

```python
regen = [0.0]*4
for i in range(4):
    if veh.driven[i] and p.regen:
        tmin, _ = veh.motor_torque_limits(x[3+i])
        regen[i] = -tmin * G / R
```
This is the most braking force each motor can supply at the current wheel speed: torque limit, power limit and regen fraction, converted to force. At 20 m/s that is about 1330 N per wheel.

**Step 3: friction brake request (L95–96).**

```python
deficit = sum(max(0.0, -Fx_des[i] - regen[i]) for i in range(4))
T_brake = min(deficit * R, p.T_brake_max)
```
- For each wheel, take whatever braking regen can't cover, and add these up. That total becomes the friction brake torque request.
- `max(0, ·)` stops a wheel with spare regen from cancelling another wheel's shortfall.

**Step 4: motors make up the difference (L97–100).**

```python
for i in range(4):
    F_mech = T_brake * veh.bias[i] / R       # friction force this wheel gets (fixed bias)
    if veh.driven[i]:
        Tm[i] = (Fx_des[i] + F_mech) * R / G  # motor supplies the rest
```
- The hydraulic system sends `T_brake` to the wheels in a fixed ratio (65% front by default). Each motor then supplies the difference between its wheel's target and what the friction brake already gives.
- If the bias over-brakes a wheel, its motor can drive slightly positive to correct it. This is how real EV brake blending works.

### `control`: traction control / ABS (L102–115)

[Jump to code](./fsae_lapsim/transient.py#L102)

```python
kt = st.slip_target * p.kappa_peak                     # 103: 0.8 × 0.10 = 0.08
worst_brake = 1.0                                      # 104
for i in range(4):
    vl  = max(abs(vx - r*veh.yw[i]), p.v_slip_min)     # 106: wheel-centre forward speed (floored)
    kap = (x[3+i]*R - (vx - r*veh.yw[i])) / vl         # 107: slip ratio of wheel i
```
- Lines 106–107 compute each wheel's slip ratio. `vx − r·y_i` is the corner's forward speed: the outer wheel travels faster in a turn.

**Driving (wheelspin), L108–109:**

```python
    if kap > kt and Tm[i] > 0:
        Tm[i] *= max(0.0, 1.0 - 4.0*(kap - kt)/p.kappa_peak)
```

This is a proportional ramp on motor torque:

| Slip ratio κ | Torque factor |
|---|---|
| ≤ 0.080 | 1.0 (no intervention) |
| 0.0925 | 0.5 |
| ≥ 0.105 (= 0.08 + 0.10/4) | 0.0 (torque fully cut) |

It has no memory, so it is a pure proportional slip controller. Slip settles in about 2 ms and this runs every 1 ms, so that is enough.

**Braking (lock-up), L110–115:**

```python
    elif kap < -kt:
        f = max(0.0, 1.0 - 4.0*(-kap - kt)/p.kappa_peak)   # same ramp
        if Tm[i] < 0: Tm[i] *= f                          # scale this wheel's regen
        worst_brake = min(worst_brake, f)
T_brake *= worst_brake
```
- Motors are per wheel, so each motor's regen is cut individually.
- The friction brake is one hydraulic input, so it can't be cut per wheel. The whole brake is scaled by the worst-slipping wheel, like a basic ABS.

**Return (L116):**

```python
return [d_fl, d_fr, Tm[0], Tm[1], Tm[2], Tm[3], T_brake], dict(v_ref=v_ref, delta=delta)
```
It returns the 7-element input vector `u`, plus extra values for logging.

---

## 4. `run_transient`, the simulation loop (L119–197)

### Setup (L125–140)

[Jump to code](./fsae_lapsim/transient.py#L125)

```python
st = st or DriverSettings()                              # 125: default settings if none given
if standing_start is None:
    standing_start = not track.closed                    # 126–127: open course → standing start
qss = qss or run_qss(veh, track, standing_start=...)     # 128: GUI passes QSS in to avoid recomputing
drv = Driver(veh, track, qss, st)                        # 129
L = track.length                                         # 130: finish distance
v0 = 0.0 if standing_start else float(qss["v"][0]) * st.speed_scale   # 131
x = veh.initial_state(max(v0, 0.5))                      # 132
```
- Line 131: a flying lap starts at the QSS speed for s = 0.
- Line 132 builds the initial state:
  - on the centreline (n = 0), pointing along it (ξ = 0);
  - wheels rolling at v/R with no slip;
  - loads already settled to their static-plus-aero values.
- The 0.5 m/s floor keeps the "vehicle stopped" check from firing and keeps slip well-behaved at zero speed.

```python
curv = track.curvature_at                     # 133: function s → κ, passed into the vehicle model
t = 0.0                                       # 134
t_max = max(3.0 * qss["lap_time"], 10.0)      # 135: give up after 3× the QSS time
logs = {k: [] for k in ("t","x","u","y","v_ref")}   # 136
next_log = 0.0; status = "ok"; wall0 = time.time(); last_prog = 0.0   # 137–140
```

### Main loop (L141–179)

[Jump to code](./fsae_lapsim/transient.py#L141)

**1. Controller (L142).**

```python
u, info = drv.control(x)
```
The driver is evaluated once per step, and `u` is held constant through all four RK4 sub-steps (a "zero-order hold"). This is how a real ECU behaves: sample, compute, hold.

**2. Logging (L143–152).**

```python
if t >= next_log - 1e-12:                 # every 10 ms of simulated time
    aux = {}
    veh.deriv(x, u, curv, aux)            # 145
    logs["t"].append(t); logs["x"].append(list(x))
    logs["u"].append([*aux["delta"], *aux["Tm"], aux["Tb"]])        # 148
    logs["y"].append([ax, ay, beta, κ×4, α×4, Fx×4, Fy×4, Pel×4])   # 149–150
    logs["v_ref"].append(info["v_ref"])
    next_log += st.log_dt
```
- Line 145 runs the vehicle model only to fill `aux` with internal values (tyre forces, slips, accelerations, power). The result is thrown away.
- Line 148 logs the inputs **after** the model's limits were applied: what the motors and brakes actually delivered, not what the driver asked for. A motor torque plot that is flat at 21 N·m means the motor limit, not the driver.

**3. Integrate (L153–154).**

```python
h = min(st.dt, veh.stable_dt(x))
x_new = veh.rk4_step(x, u, h, curv)
```
- Picks the step size: the user's `dt`, or smaller if the stiff wheel modes need it (at low speed with high load). See [TRANSIENT_DYNAMICS.md §5](./TRANSIENT_DYNAMICS.md#5-time-scales-and-stiffness).
- Then takes one 4th-order Runge–Kutta step.

**4. Finish line (L155–159).**

```python
if x_new[11] >= L:                                        # this step crossed the finish line
    frac = (L - x[11]) / max(x_new[11] - x[11], 1e-12)    # fraction of the step before the line
    x = [a + frac*(b - a) for a, b in zip(x, x_new)]      # interpolate state to the line
    t += frac * h                                         # exact crossing time
    break
```
This removes up to 1 ms of rounding error from the lap time.

**5. Advance and check (L160–179).**

```python
x = x_new; t += h                                         # 160–161
if progress and t - last_prog > 0.5: progress(x[11]/L)    # 162–164: GUI progress bar
if cancel and cancel(): status = "cancelled"; break       # 165–167: GUI Cancel button
```

| Lines | Check | Status |
|---|---|---|
| 168–170 | Any state is NaN or infinite | `diverged (non-finite state)` |
| 171–173 | Heading error > 1.3 rad (75°, i.e. spinning) or more than 12 m off the centreline | `vehicle left the track` |
| 174–176 | Simulated time > 3× the QSS lap time | `time limit exceeded` |
| 177–179 | Below 0.3 m/s after the first 2 s | `vehicle stopped` |

### Wrap-up (L180–197)

[Jump to code](./fsae_lapsim/transient.py#L180)

```python
u, info = drv.control(x); aux = {}; veh.deriv(x, u, curv, aux)   # 181–183
logs[...].append(...)                                            # 184–189: log the finish-line state too
if progress: progress(1.0)                                       # 190–191
X = np.array(logs["x"])                                          # 192: list of lists → (N × 15) array
gx, gy, gpsi = track.to_global(X[:,11], X[:,12], X[:,13])        # 193: (s, n, ξ) → (X, Y, ψ) for the map
return dict(method="Transient", status=status,
            lap_time = t if status == "ok" else nan,             # an aborted run has no lap time
            t, x, u, y, v_ref, X, Y, psi, qss, wall_time)        # 194–197
```

The returned dictionary:

| Key | Shape | Content |
|---|---|---|
| `t` | (N,) | Log times [s] |
| `x` | (N, 15) | States |
| `u` | (N, 7) | Inputs **as applied** (after saturation) |
| `y` | (N, 23) | Outputs; column names in `Y_COLS` |
| `v_ref` | (N,) | Speed target the driver was chasing |
| `X, Y, psi` | (N,) | Global position and heading for the track map |
| `qss` | dict | The QSS result used as the speed plan |
| `lap_time`, `status`, `wall_time` | scalars | Summary |

---

## 5. `Y_COLS` (L200–205)

[Jump to code](./fsae_lapsim/transient.py#L200)

These are the names of the 23 columns in the `y` log:

| Columns | Names |
|---|---|
| 0–2 | `ax`, `ay`, `beta` |
| 3–6 | `kappa_FL`, `kappa_FR`, `kappa_RL`, `kappa_RR` |
| 7–10 | `alpha_FL` … `alpha_RR` |
| 11–14 | `Fx_FL` … `Fx_RR` (body frame) |
| 15–18 | `Fy_FL` … `Fy_RR` (body frame) |
| 19–22 | `Pel_FL` … `Pel_RR` (electrical power per motor) |

The GUI uses them for the CSV export header and to find columns when plotting.

---

## 6. Known issues and limitations in this file

These are real limitations of the current code, found during this walkthrough. They do not invalidate results, but you should know about them.

### 6.1 Brake blending can fall short under heavy braking (L95–100)

The friction-brake shortfall is worked out per wheel, but the fixed bias then sends part of it to wheels that didn't need it. Illustrative case with round load values:

| | Front (each) | Rear (each) |
|---|---|---|
| Desired braking force | 2017 N | 1283 N |
| Regen capacity | 1330 N | 1330 N |
| Shortfall | 687 N | 0 N |

- Total shortfall = 2 × 687 N → `T_brake` ≈ 280 N·m.
- At 65/35 bias the fronts get only 449 N each from the friction brake, and the rears get 241 N each that they didn't need.
- The front motors are asked for 2017 − 449 = 1568 N of regen, above their 1330 N limit.
- The vehicle model clips the motor torque, and about 480 N (≈7%) of the requested braking goes missing.
- The speed feedback partly makes it up on later steps.

**Fix:** size `T_brake` so that, after the bias split, no wheel's remaining demand exceeds its regen capacity. That means taking the largest of `shortfall_i / bias_i` over the wheels instead of the sum of shortfalls.

### 6.2 Torque-vectoring authority halves on a RWD car (L74)

`dF = Mz / (track_f + track_r)` assumes all four wheels share the moment. With only rear motors, about half of `Mz` is delivered. It still works because it is feedback, but the effective gain differs between AWD and RWD.

**Fix:** divide by the sum of the track widths of the driven axles only.

### 6.3 Traction control uses ideal information (L106–107)

- It reads the true vehicle speed; a real system would have to estimate it from wheel speeds and an IMU.
- It ignores the steer angle when computing front-wheel slip. The error is under 1% at FSAE steer angles.

### 6.4 The standing start begins at 0.5 m/s (L132)

This is about 0.04 s optimistic on an acceleration run or the autocross.

### 6.5 The off-track limit of 12 m is generous (L171)

An FSAE track is 3–5 m wide, so status "ok" only means the car didn't spin or wander far. Check **Max |n| offset** in the results summary to see whether it stayed within the cones.

**Fix:** make the limit a setting, or use the track half-width.

### 6.6 No integral action in steering (L62)

A small steady-state offset of a few centimetres absorbs understeer. This is harmless here, but it is why `n` never settles exactly to zero in a long corner.

**Fix:** add a small integral term on `n`, or on the yaw-rate error.
