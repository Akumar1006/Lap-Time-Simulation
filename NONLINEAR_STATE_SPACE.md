# The Nonlinear State-Space Model: from ẋ = Ax + Bu to ẋ = f(x, u)

This document explains the vehicle model as a **nonlinear state-space model**. It assumes you know linear state-space models ($\dot{\mathbf x} = A\mathbf x + B\mathbf u$, $\mathbf y = C\mathbf x + D\mathbf u$) and builds on that. It covers:

- what changes when the model becomes nonlinear;
- what stays the same;
- how this simulator implements it;
- the four ways it is used: simulation, trim, linearisation, and the closed-loop lap simulation.

Numbers come from the default car, computed with the code in this repository.

Related documents:
- [TRANSIENT_EQUATIONS.md](./TRANSIENT_EQUATIONS.md): every equation in $f$ and $h$, written out.
- [TRANSIENT_DYNAMICS.md](./TRANSIENT_DYNAMICS.md): the physics behind each one.

---

## Contents

1. [What you already know: the linear model](#1-what-you-already-know-the-linear-model)
2. [The nonlinear generalisation](#2-the-nonlinear-generalisation)
3. [What makes this vehicle model nonlinear](#3-what-makes-this-vehicle-model-nonlinear)
4. [Consequences you can see in the numbers](#4-consequences-you-can-see-in-the-numbers)
5. [How f is implemented](#5-how-f-is-implemented)
6. [How the model is used](#6-how-the-model-is-used)
7. [Practical pitfalls of nonlinear models](#7-practical-pitfalls-of-nonlinear-models)
8. [Linear ↔ nonlinear ↔ code: a mapping table](#8-linear--nonlinear--code-a-mapping-table)
9. [Using the model from your own code](#9-using-the-model-from-your-own-code)

---

## 1. What you already know: the linear model

$$\dot{\mathbf x} = A\mathbf x + B\mathbf u, \qquad \mathbf y = C\mathbf x + D\mathbf u$$

Linearity gives you a lot for free:

| Property | What it means |
|---|---|
| **The matrices *are* the model** | Four constant matrices describe the system completely, everywhere. |
| **Superposition** | Response to $\mathbf u_1 + \mathbf u_2$ = response to $\mathbf u_1$ + response to $\mathbf u_2$. Double the input, double the output. |
| **Closed-form solution** | $\mathbf x(t) = e^{At}\mathbf x_0 + \int_0^t e^{A(t-\tau)}B\,\mathbf u(\tau)\,d\tau$. |
| **Global stability** | The eigenvalues of $A$ decide stability for *every* state and input. |
| **Equilibrium at the origin** | With $\mathbf u = 0$, $\mathbf x = 0$ is an equilibrium. Others come from the linear system $0 = A\mathbf x + B\mathbf u$. |
| **Frequency domain** | Transfer functions $C(sI - A)^{-1}B + D$, Bode plots, poles and zeros. |

A real vehicle has none of these properties exactly. It has *approximately* all of them for small motions around a given operating point, which is why linearisation (§6.3) is still useful.

---

## 2. The nonlinear generalisation

Replace the matrix products with general functions:

$$\boxed{\dot{\mathbf x} = f(\mathbf x, \mathbf u, \mathbf w), \qquad \mathbf y = h(\mathbf x, \mathbf u)}$$

| | Linear | Nonlinear |
|---|---|---|
| State $\mathbf x$ | Minimal set of quantities with memory | **Same idea**, unchanged |
| Input $\mathbf u$ | What you control | **Same** |
| Output $\mathbf y$ | What you measure / care about | **Same** |
| Dynamics | $A\mathbf x + B\mathbf u$ | $f(\mathbf x, \mathbf u, \mathbf w)$: any function |
| Output map | $C\mathbf x + D\mathbf u$ | $h(\mathbf x, \mathbf u)$: any function |
| Known external signals | Usually lumped into $\mathbf u$ | $\mathbf w$: here, the track curvature $\kappa(s)$ |
| Solving it | Closed form | **Numerical integration only** |
| Equilibria | Solve a linear system | **Solve $f(\mathbf x, \mathbf u) = 0$ iteratively**; may have none, or several |
| Stability | Global, from eig($A$) | **Local**: depends on where you are |
| Superposition | Yes | **No** |

### The mental model that carries over

Even nonlinear, the state-space idea is identical. **The state is a point, and $f$ gives the velocity of that point.**
- At every instant, the 15 numbers in $\mathbf x$ are a point in a 15-dimensional space.
- $f(\mathbf x, \mathbf u)$ is an arrow at that point, saying which way and how fast the state is moving.
- Simulating means following the arrows.

In a linear system the arrows form a simple pattern ($A\mathbf x$, a linear field). In a nonlinear system the field is curved, so you can only follow it step by step, numerically.

### $f$ is a procedure, not a formula

In practice $f$ is **a function in code**: give it 15 state values and 7 input values, and it returns 15 derivatives. Inside, it runs a chain of intermediate calculations (velocities, slips, tyre forces, loads). There is no single equation you can write as "$A\mathbf x$". §5 shows exactly what happens inside.

### The extra signal $\mathbf w$: track curvature

The model needs the track curvature at the car's current distance, $\kappa(s)$. That isn't a control input, and it isn't a state. It's a *known function of a state* ($s$):

$$\dot{\mathbf x} = f\big(\mathbf x, \mathbf u, \kappa(s)\big)$$

This makes the model **parameter-varying**: its behaviour depends on where on the track it is. In the code this is the `curv_fn` argument. On a straight with $\kappa = 0$, the path states $(s, n, \xi)$ reduce to global coordinates $(X, Y, \psi)$, which is how the open-loop simulations in the GUI work.

---

## 3. What makes this vehicle model nonlinear

Every term below is in $f$, and every one breaks the linear structure. The right-hand column says what a linear model would miss.

| Nonlinearity | Example term | Type | What a linear model gets wrong |
|---|---|---|---|
| **Rotating reference frame** | $\dot v_x = a_x + r\,v_y$, $\dot v_y = a_y - r\,v_x$ | Product of two states (bilinear) | Coupling between yaw and longitudinal motion only exists when *both* are non-zero |
| **Tyre saturation** | $F = D\,\sin(C\arctan(B\rho))$ | Smooth, saturating | Force keeps growing with slip forever; there's no grip limit |
| **Combined slip** | $F_x \propto \sigma_x/\rho$, $\rho = \sqrt{\sigma_x^2 + \sigma_y^2}$ | Coupled | Braking doesn't reduce cornering grip |
| **Slip definitions** | $\kappa = (\omega R - v_l)/\max(\lvert v_l\rvert, v_{min})$ | Division by a state; abs; max | Slip sensitivity is fixed instead of scaling with $1/v$ |
| **Load sensitivity** | $D = \mu\,(1 + k_{ls}\Delta F_z/F_{z0})\,F_z$ | Quadratic in load | Load transfer has no net cost in grip |
| **Steering geometry** | $F_y = F_{xw}\sin\delta + F_{yw}\cos\delta$ | Trigonometric | Steering induces no drag; the Ackermann split is exact |
| **Aero** | $D = q C_dA\, v_x\lvert v_x\rvert$, downforce $\propto v_x^2$ | Quadratic | Downforce and drag fixed at one speed |
| **Load transfer** | $\Delta F_z \propto m\,a_y\,h$, where $a_y$ is itself a function of $\mathbf x$ | Composite | Loads fixed at one operating point |
| **Actuator limits** | $\operatorname{clip}(T_m, T_{\min}(\omega), T_{\max}(\omega))$; power cap | **Non-smooth** (kinks) | Unlimited torque and steer |
| **Logic** | Regen vs drive efficiency, `if` branches | **Discontinuous slope** | Single efficiency |
| **Path coordinates** | $\dot s = \dfrac{v_x\cos\xi - v_y\sin\xi}{1 - n\kappa}$ | Trig + division | Only valid for tiny heading errors and offsets |

There are two kinds here:
- **Smooth nonlinearities** (trig, products, the tyre curve). These can be linearised, and are well approximated near an operating point.
- **Non-smooth nonlinearities** (clip, max, abs, if). These have kinks. A linearisation taken right at a kink depends on which side you nudge, and gradient-based tools (optimisers, Newton solvers) can struggle there. See §7.

---

## 4. Consequences you can see in the numbers

### 4.1 Superposition fails

Open-loop step steer at 15 m/s from straight-line running, with motor torque held at the value that maintains 15 m/s. Yaw rate after 1.5 s:

| Steer | Yaw rate (nonlinear) | Linear prediction ($k \times$ the 1° response) | Lateral acc. (nonlinear) | Linear prediction | Error |
|---|---|---|---|---|---|
| 1° | 9.58 °/s | 9.58 °/s | 0.26 g | 0.26 g | – |
| 2° | 19.06 °/s | 19.17 °/s | 0.51 g | 0.51 g | 0.6% |
| 4° | 37.13 °/s | 38.33 °/s | 0.96 g | 1.02 g | 3% |
| 8° | 61.77 °/s | 76.66 °/s | 1.47 g | 2.04 g | **19%** |

- **Up to about 0.5 g the car is effectively linear.** Doubling the steer doubles the response.
- **Near the grip limit, the linear prediction is badly wrong.** It says 2.04 g, which this car physically cannot do; the tyres saturate.

That is the whole reason lap simulation needs the nonlinear model: a lap is spent mostly *near* the limit.

### 4.2 Equilibria are not at the origin, and may not exist

A linear system always has $\mathbf x = 0$ as an equilibrium. For this car the interesting equilibria are **steady corners**: constant speed and radius, with $\dot v_x = \dot v_y = \dot r = 0$, constant wheel speeds and settled loads. Finding one means solving $f(\mathbf x, \mathbf u) = 0$ numerically (§6.2).

They don't always exist. At 15 m/s:

| Radius | $a_y$ | Equilibrium? |
|---|---|---|
| 40 m | 0.57 g | yes, steer 2.25° |
| 20 m | 1.15 g | yes, steer 4.66° |
| 15.3 m | 1.50 g | yes, the tightest that solves |
| 15 m | 1.53 g | **none**: the car cannot hold this corner at this speed |

A linear model would happily give you a "steady state" at 3 g.

**Why the limit is 1.50 g here, while the QSS envelope says 1.59 g at 15 m/s:**
- The GGV assumes ideal balance: every tyre reaches its limit at the same time.
- A real equilibrium has a *fixed* setup (Ackermann, torque split, LLTD), so one axle saturates first.
- The 0.09 g gap is the cost of imperfect balance. Only the nonlinear model can show it.

### 4.3 The dynamics depend on where you are

Linearising at different equilibria gives different $A$ matrices, and so different eigenvalues. The table shows the **handling mode**, the lightly damped eigenvalue pair dominated by $v_y$ and $r$:

| Speed | Radius | $a_y$ | Handling-mode eigenvalue | Natural freq. | Damping ratio ζ |
|---|---|---|---|---|---|
| 8 m/s | straight | 0 | −52.9 (real) | – | 1.00 |
| 15 m/s | straight | 0 | −25.3 ± 2.9j | 4.05 Hz | 0.99 |
| 15 m/s | 40 m | 0.57 g | −22.4 ± 2.8j | 3.60 Hz | 0.99 |
| 15 m/s | 20 m | 1.15 g | −13.0 ± 5.4j | 2.24 Hz | 0.92 |
| 25 m/s | straight | 0 | −18.5 ± 6.1j | 3.09 Hz | 0.95 |
| 25 m/s | 60 m | 1.06 g | −12.8 ± 7.4j | 2.36 Hz | 0.86 |
| 25 m/s | 40 m | 1.59 g | −5.8 ± 9.5j | 1.77 Hz | **0.52** |

**The car gets less damped and slower to respond as speed and lateral acceleration rise.** At 25 m/s and 1.6 g the yaw response is oscillatory (ζ = 0.52). Every one of these linear models is correct, but only *at its own operating point*. Asking "is the car stable?" has no single answer; you have to say *where*. At the absolute limit the answer can be "no": the closed-loop experiments in [FEEDFORWARD_FEEDBACK.md](./FEEDFORWARD_FEEDBACK.md) show the car spinning at the QSS limit speed without yaw feedback.

### 4.4 Couplings that appear only away from straight-line driving

Some entries of $A$ are exactly zero when driving straight and non-zero in a corner:

| Entry | Straight, 15 m/s | 20 m corner, 15 m/s | Why |
|---|---|---|---|
| $\partial\dot v_x/\partial r$ | 0.000 | −5.82 | In a corner, extra yaw rotates lateral tyre force into the longitudinal direction |
| $\partial\dot v_y/\partial F_{z,FR}$ | 0 | +0.003 per N | Load only changes lateral force if the tyre is already producing lateral force |
| $\partial\dot r/\partial r$ | −56.9 | −37.9 | Yaw damping falls as the tyres approach saturation |

In linear terms, the **B and C matrices also change** between these points (see §6.3).

---

## 5. How f is implemented

### 5.1 The signature

[`Vehicle.deriv`](./fsae_lapsim/vehicle.py):

```python
def deriv(self, x, u, curv_fn=_zero_curv, aux=None) -> list[float]:
    """State derivative x_dot = f(x, u). curv_fn(s) gives the track curvature [1/m]."""
```

| Argument | Meaning |
|---|---|
| `x` | List of 15 numbers. The order is fixed (`STATE_NAMES`): `vx, vy, r, w_FL…w_RR, Fz_FL…Fz_RR, s, n, xi, E`. |
| `u` | List of 7 numbers (`INPUT_NAMES`): `delta_FL, delta_FR, Tm_FL…Tm_RR, T_brake`. |
| `curv_fn` | The external signal $\kappa(s)$. Defaults to a function that always returns 0 (flat ground, straight track). |
| `aux` | Optional dictionary. If given, `deriv` stores every intermediate quantity in it; this is how $h$ is obtained (§5.4). |
| **returns** | List of 15 derivatives $\dot{\mathbf x}$, in the same order as `x`. |

### 5.2 What happens inside one call

Every call computes $\dot{\mathbf x}$ from scratch, following a fixed chain of **algebraic** steps. Nothing in this chain is a state; it is all recomputed from $\mathbf x$ and $\mathbf u$ each time.

```
 x, u ─► 1. apply actuator limits       δ*, T_m*, T_b*, P_el          (§3 of TRANSIENT_EQUATIONS)
         2. wheel-centre velocities      v_x,i , v_y,i  → rotate by δ → v_l,i , v_t,i
         3. slips                        κ_i , α_i
         4. tyre model                   F_xw,i , F_yw,i   (uses F_z,i from the STATE)
         5. body forces & moment         ΣF_x , ΣF_y , M_z , drag  →  a_x , a_y
         6. load-transfer targets        F_z,i^ss(v_x, a_x, a_y)
         7. path kinematics              κ(s), ṡ, ṅ, ξ̇
         8. assemble the 15 derivatives  [v̇_x, v̇_y, ṙ, ω̇_1..4, Ḟ_z,1..4, ṡ, ṅ, ξ̇, Ė]
```

A condensed skeleton of the real code (see `vehicle.py` for the full version):

```python
Tm, Pel = self.saturate_motors(u[2:6], w)          # 1  actuator limits (nonlinear, non-smooth)
for i in range(4):
    vxw, vyw = vx - r*yw[i], vy + r*xw[i]          # 2  rigid-body kinematics (bilinear)
    vl, vt   = rotate(vxw, vyw, delta[i])          #    trig
    kap = (w[i]*R - vl) / max(abs(vl), vmin)       # 3  slip ratio (division by a state)
    alp = -atan(vt / max(abs(vl), vmin))           #    slip angle
    fxw, fyw = self.tire(kap, alp, Fz[i])          # 4  saturating tyre curve
    Fx, Fy = rotate_back(fxw, fyw, delta[i])       # 5  body-frame forces
    sFx += Fx; sFy += Fy; Mz += xw[i]*Fy - yw[i]*Fx
    dw[i] = (Tm[i]*G - Tb*bias[i]*sat(w[i]) - fxw*R - Crr*Fz[i]*R*sat(w[i])) / Iw_eff[i]
ax, ay = (sFx - drag)/m, sFy/m
fss = self.fz_steady(vx, ax, ay)                    # 6  load-transfer targets
sdot = (vx*cos(xi) - vy*sin(xi)) / max(1 - n*k, 0.05)   # 7
return [ax + r*vy, ay - r*vx, Mz/Iz, *dw,           # 8
        *((fss[i] - Fz[i])/tau for i in range(4)),
        sdot, vx*sin(xi) + vy*cos(xi), r - k*sdot, sum(Pel)]
```

### 5.3 Rule: `f` must be a pure function

`deriv` has **no memory**. It doesn't store anything between calls, and its result depends only on its arguments. That matters for two reasons:

1. **The integrator calls $f$ at trial points.** RK4 evaluates $f$ four times per step, at states that are *not* where the car ends up (§6.1). If $f$ updated some internal variable on each call, those trial evaluations would corrupt it.
2. **All memory has to live in $\mathbf x$.** If something should remember its past (a filter, an integrator, a lagged load), it must be a **state**, with its own equation in $f$. That's exactly why the normal loads are states with a lag rather than variables updated inside `deriv`.

The same rule makes the linearisation work. Finite differences call $f$ at nudged states and compare results, which only makes sense if $f$ is a pure function.

### 5.4 The output function $h$

[`Vehicle.outputs`](./fsae_lapsim/vehicle.py) is $h(\mathbf x, \mathbf u)$:

```python
def outputs(self, x, u, curv_fn=_zero_curv):
    aux = {}
    self.deriv(x, u, curv_fn, aux)        # run the chain once, keep the intermediates
    return [aux["ax"], aux["ay"], aux["beta"], *aux["kappa"], *aux["alpha"]]
```

The outputs ($a_x, a_y, \beta, \kappa_i, \alpha_i$) are intermediate values of the chain in §5.2, so $h$ just reruns it and keeps them. As in the linear case, outputs are **not integrated**: they are computed instantly from the current state and input. The transient lap simulation logs more intermediate quantities the same way: tyre forces, electrical power, and the saturated inputs.

### 5.5 Closing the loop

In the lap simulation the inputs come from a controller, so the system that is actually integrated is

$$\dot{\mathbf x} = f\big(\mathbf x,\ g(\mathbf x),\ \kappa(s)\big)$$

where $g$ is `Driver.control` in `transient.py`. The controller is evaluated **once per time step**, and its output is held constant over that step (a zero-order hold, like a digital ECU).

---

## 6. How the model is used

There are four uses, and each has a linear counterpart you already know.

### 6.1 Simulation, the replacement for $e^{At}$

There's no closed-form solution, so the state is advanced in small time steps $h$, following the arrows.
- **Simplest scheme (explicit Euler):** $\mathbf x_{k+1} = \mathbf x_k + h\, f(\mathbf x_k, \mathbf u_k)$.
- **This code uses classic 4th-order Runge–Kutta (RK4)** ([`Vehicle.rk4_step`](./fsae_lapsim/vehicle.py)). It samples the slope four times across the step and combines them:

$$\mathbf k_1 = f(\mathbf x_k), \quad \mathbf k_2 = f(\mathbf x_k + \tfrac h2\mathbf k_1), \quad \mathbf k_3 = f(\mathbf x_k + \tfrac h2\mathbf k_2), \quad \mathbf k_4 = f(\mathbf x_k + h\mathbf k_3)$$

$$\mathbf x_{k+1} = \mathbf x_k + \tfrac h6(\mathbf k_1 + 2\mathbf k_2 + 2\mathbf k_3 + \mathbf k_4)$$

The error per unit time falls with $h^4$, so halving the step cuts the error about 16×.

**How small must $h$ be?** This is where linear thinking helps. Near the current state, $f$ behaves like its Jacobian $A$, and explicit RK4 is only stable if $h\,|\lambda| \lesssim 2.8$ for **every** eigenvalue λ of that local $A$.

This car's local eigenvalues span from about −0.1 s⁻¹ (speed) to about −600 s⁻¹ (wheel spin), so it is **stiff**. The wheel eigenvalue scales like $1/v$, so it gets worse at low speed:

| Operating point | Most negative eigenvalue | Stable step `stable_dt` |
|---|---|---|
| 8 m/s straight | −857 s⁻¹ | 2.4 ms |
| 15 m/s straight | −507 s⁻¹ | 4.1 ms |
| 15 m/s, 1.15 g | −594 s⁻¹ | 3.0 ms |
| 30 m/s straight | −349 s⁻¹ | 5.9 ms |

[`Vehicle.stable_dt`](./fsae_lapsim/vehicle.py) estimates the fastest eigenvalue analytically from the current state, at every step, and the integrator uses $h = \min(1\ \text{ms},\ 2/\lambda_{max})$. This is a linear-systems idea (local eigenvalues), applied continuously to a nonlinear system.

There are two simulation drivers:
- **[`Vehicle.simulate`](./fsae_lapsim/vehicle.py)** is open loop: you supply $\mathbf u(t)$. This is the State-space tab's time response.
- **[`run_transient`](./fsae_lapsim/transient.py)** is closed loop with the driver: one lap, with finish-line interpolation and abort checks.

### 6.2 Trim: finding equilibria, the replacement for solving $0 = A\mathbf x + B\mathbf u$

[`Vehicle.trim(v, R)`](./fsae_lapsim/vehicle.py) finds the steady corner at speed $v$ and radius $R$ ($R = 0$ means straight).

**Unknowns (12):**

$$\mathbf z = [\,v_y,\ r,\ \omega_{1..4},\ F_{z,1..4},\ \delta,\ T_{total}\,]$$

- $v_x = v$ is fixed.
- Steer is one bicycle angle δ, split by the Ackermann geometry.
- Motor torque is one total, split in proportion to static load.
- The path states are irrelevant to the equilibrium and set to zero.

**Equations (12)**, solved as residuals with `scipy.optimize.least_squares` (Levenberg–Marquardt):
- $\dot v_x = 0$, $\dot v_y = 0$, $\dot r = 0$;
- $\dot\omega_i = 0$ (four equations);
- $\dot F_{z,i} = 0$ (four equations);
- the constraint that the car turns at the right rate, $r = V/R$ with $V = \sqrt{v_x^2 + v_y^2}$.

Things that differ from the linear case:
- **Iterative and needs a starting guess.** The guess is built from simple physics: kinematic steer $L/R$, drag-balancing torque, quasi-static loads.
- **Residual scaling matters.** The equations have very different sizes: $\dot\omega$ ~ hundreds of rad/s², $\dot F_z$ ~ thousands of N/s. Each residual is scaled so none dominates.
- **It can fail.** If no equilibrium exists (§4.2), the solver ends with a non-zero residual, and `trim` reports `success = False`. A failed trim is information: the requested corner is beyond what the car can hold.

### 6.3 Linearisation: getting back to A, B, C, D

Near an equilibrium $(\mathbf x_0, \mathbf u_0)$, a first-order Taylor expansion gives

$$f(\mathbf x_0 + \delta\mathbf x,\ \mathbf u_0 + \delta\mathbf u) \approx \underbrace{f(\mathbf x_0, \mathbf u_0)}_{=\,0 \text{ at a trim}} + \underbrace{\frac{\partial f}{\partial \mathbf x}}_{A}\delta\mathbf x + \underbrace{\frac{\partial f}{\partial \mathbf u}}_{B}\delta\mathbf u$$

$$h(\mathbf x_0 + \delta\mathbf x,\ \mathbf u_0 + \delta\mathbf u) \approx \mathbf y_0 + \underbrace{\frac{\partial h}{\partial \mathbf x}}_{C}\delta\mathbf x + \underbrace{\frac{\partial h}{\partial \mathbf u}}_{D}\delta\mathbf u$$

That gives back the familiar form, **in deviation variables**:

$$\delta\dot{\mathbf x} = A\,\delta\mathbf x + B\,\delta\mathbf u, \qquad \delta\mathbf y = C\,\delta\mathbf x + D\,\delta\mathbf u$$

**How the code computes the Jacobians.** [`Vehicle.linearize`](./fsae_lapsim/vehicle.py) uses **central finite differences**, so it works for whatever $f$ contains. For each state $j$:

$$A_{:,j} \approx \frac{f(\mathbf x_0 + \varepsilon_j\mathbf e_j, \mathbf u_0) - f(\mathbf x_0 - \varepsilon_j\mathbf e_j, \mathbf u_0)}{2\varepsilon_j}, \qquad \varepsilon_j = 10^{-6}\max(1, |x_{0,j}|)$$

The same is done for each input (giving $B$), and with $h$ in place of $f$ (giving $C$ and $D$). The sizes are $A$: 15×15, $B$: 15×7, $C$: 11×15, $D$: 11×7.

**What the matrices look like for this car:**
- **$A$** is dense in the dynamic block (states 0–10). It has a simple structure for the path states and energy: those are pure integrators that nothing else depends on, apart from $s$ selecting the curvature.
- **$B$:**
  - Steering enters $\dot v_y$ and $\dot r$ directly.
  - Motor torques and the brake enter **only** the wheel-speed rows ($\partial\dot\omega_i/\partial T_{mi} = G/I_{\text{eff}}$); they reach the chassis through the states.
  - Torques also enter the energy row.
- **$D$** has non-zero entries **only in the steering columns** (for $a_x$, $a_y$ and the front-wheel slips). Steering changes tyre force instantly; torque has to change wheel speed first.

**Uses** (the State-space tab does the first three):
- eigenvalues and mode shapes (participation factors);
- controllability: per-mode PBH test;
- export to MATLAB/Simulink;
- controller design (LQR, pole placement, gain scheduling);
- understanding how setup changes alter the dynamics.

**How far you can trust it:** only near $(\mathbf x_0, \mathbf u_0)$. For the 8° steer step in §4.1, a linear model built at straight-line running is off by 19%. To cover a range of conditions, linearise at several operating points (§4.3), which is the basis of gain scheduling.

### 6.4 Closed-loop lap simulation

`run_transient` combines everything:

```
x ← initial state                       (straight-line rolling at the start speed, loads settled)
loop:
    u ← g(x)                             driver: steering, speed, torque allocation
    h ← min(dt, stable_dt(x))            local-eigenvalue step limit
    x ← RK4 step of f(x, u, κ(s))        nonlinear plant
    stop at the finish line (interpolated) or if the car leaves the track / spins / stops
```

---

## 7. Practical pitfalls of nonlinear models

| Pitfall | Where it shows up here | How it's handled |
|---|---|---|
| **Division by a state that can be zero** | Slip uses $1/\lvert v_l \rvert$; path uses $1/(1 - n\kappa)$; sideslip uses $v_y/v_x$ | Floors: $v_{min}$ = 4 m/s; 0.05; $\max(\lvert v_x\rvert, 10^{-3})$ |
| **Stiffness** | Wheel modes 3000× faster than the speed mode | Automatic step limit from local eigenvalues (§6.1) |
| **Non-smooth functions** | clip, max, abs, `if regen` | Fine for simulation. Linearising right at a kink depends on which side you nudge. A gradient-based optimiser (e.g. minimum-lap-time optimal control) would need smoothed versions. |
| **Very different state scales** | $r$ ~ 1 rad/s, $F_z$ ~ 1000 N, $E$ ~ 10⁶ J | Relative finite-difference steps ($\varepsilon_j \propto \lvert x_j\rvert$); scaled trim residuals |
| **Hidden memory inside $f$** | Would corrupt the RK4 trial evaluations and the Jacobians | `deriv` is pure; anything with memory is a state |
| **Saturation inside vs outside $f$** | A controller can request impossible torque | Saturation lives in $f$ (the plant), so the simulated car can never exceed its hardware |
| **Equilibria that don't exist** | Corners beyond the limit | `trim` reports `success = False` instead of returning nonsense |

---

## 8. Linear ↔ nonlinear ↔ code: a mapping table

| Concept | Linear | Nonlinear | In this code |
|---|---|---|---|
| Dynamics | $A\mathbf x + B\mathbf u$ | $f(\mathbf x, \mathbf u, \kappa(s))$ | `Vehicle.deriv(x, u, curv_fn)` |
| Output | $C\mathbf x + D\mathbf u$ | $h(\mathbf x, \mathbf u)$ | `Vehicle.outputs(x, u, curv_fn)` |
| Solution | $e^{At}$ convolution | Numerical integration | `Vehicle.rk4_step`, `Vehicle.simulate`, `run_transient` |
| Step size | n/a (exact) | Limited by local stiffness | `Vehicle.stable_dt` |
| Equilibrium | Solve $A\mathbf x + B\mathbf u = 0$ | Solve $f = 0$ iteratively | `Vehicle.trim(v, R)` |
| Linear model | It *is* one | Jacobians at an operating point | `Vehicle.linearize(x0, u0, curv)` → `A, B, C, D` |
| Stability | eig($A$), global | eig of local $A$, varies with operating point | State-space tab → "Trim & linearise" |
| Feedback | $\mathbf u = -K\mathbf x$ | $\mathbf u = g(\mathbf x)$, any law | `Driver.control(x)` |
| Inputs | Unbounded | Saturated inside $f$ | `saturate_motors`, `clip` in `deriv` |
| State/input names | Rows/columns | Fixed list order | `STATE_NAMES`, `INPUT_NAMES`, `OUTPUT_NAMES` |

---

## 9. Using the model from your own code

Run from the project root (see the README):

```python
import math, numpy as np
from fsae_lapsim import Vehicle, STATE_NAMES, INPUT_NAMES

veh = Vehicle()                                   # default car

# --- evaluate f at a state: one arrow of the vector field
x = veh.initial_state(15.0)                       # straight-line rolling at 15 m/s
u = [0.05, 0.05, 5, 5, 5, 5, 0]                   # ~2.9° steer, 5 N·m per motor
xdot = veh.deriv(x, u)
print(dict(zip(STATE_NAMES, np.round(xdot, 3))))

# --- open-loop simulation: step steer at t = 0.5 s
res = veh.simulate(x, lambda t, x: u if t >= 0.5 else [0, 0, 5, 5, 5, 5, 0], t_end=3.0)
print("final yaw rate [deg/s]:", math.degrees(res["x"][-1, 2]))

# --- equilibrium + linearisation at a steady 20 m corner
x0, u0, info = veh.trim(15.0, 20.0)
A, B, C, D = veh.linearize(x0, u0, curv=1 / 20.0)
print("trim ok:", info["success"], " eigenvalues:", np.round(np.linalg.eigvals(A[:11, :11]), 2))

# --- check the linear model against the nonlinear one for a small perturbation
du = np.zeros(7); du[:2] = math.radians(0.5)
y_lin = np.array(veh.outputs(x0, u0, lambda s: 1/20)) + D @ du      # instant response (δx = 0)
y_nl  = np.array(veh.outputs(x0, list(np.array(u0) + du), lambda s: 1/20))
print("ay: linear", y_lin[1], " nonlinear", y_nl[1])
```

**Summary.**
- The nonlinear model keeps everything conceptually familiar from linear state space: states with memory, inputs, outputs, first-order equations.
- It replaces the four matrices with two functions, $f$ and $h$, that are evaluated rather than written down.
- In exchange it captures what matters for a racing car: grip limits, load transfer, actuator limits, and dynamics that change with speed and lateral acceleration.
- Linearisation brings back $A, B, C, D$ whenever you need linear tools, but only locally, at the operating point you choose.
