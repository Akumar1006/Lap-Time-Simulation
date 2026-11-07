# Transient Vehicle Dynamics – How the 4-Corner Model Works

This document explains the transient (time-domain) lap simulation in depth. It assumes you are comfortable with **quasi-steady-state (QSS)** lap simulation (GGV envelopes, apex speeds, forward/backward passes). It does not assume you know how a vehicle is modelled as a dynamic system.

Every equation here corresponds to code in [`fsae_lapsim/vehicle.py`](./fsae_lapsim/vehicle.py) (the plant) and [`fsae_lapsim/transient.py`](./fsae_lapsim/transient.py) (the driver and the time loop). Section 13 has a map from concept to function. Worked numbers use the default car parameters.

For a line-by-line explanation of `transient.py` itself, see [TRANSIENT_CODE_WALKTHROUGH.md](./TRANSIENT_CODE_WALKTHROUGH.md).

---

## Contents

1. [From QSS to transient: what actually changes](#1-from-qss-to-transient-what-actually-changes)
2. [State-space in one page](#2-state-space-in-one-page)
3. [Coordinate frames and sign conventions](#3-coordinate-frames-and-sign-conventions)
4. [Building the plant, step by step](#4-building-the-plant-step-by-step)
5. [Time scales and stiffness](#5-time-scales-and-stiffness)
6. [Numerical integration](#6-numerical-integration)
7. [The driver / controller](#7-the-driver--controller)
8. [Linearisation: reading the A matrix](#8-linearisation-reading-the-a-matrix)
9. [Worked examples from the default car](#9-worked-examples-from-the-default-car)
10. [Why the transient lap is slower than QSS](#10-why-the-transient-lap-is-slower-than-qss)
11. [Tuning and troubleshooting](#11-tuning-and-troubleshooting)
12. [Limitations and how to extend the model](#12-limitations-and-how-to-extend-the-model)
13. [Code map](#13-code-map)

---

## 1. From QSS to transient: what actually changes

In QSS, the car at each point on the track is assumed to be in **equilibrium**. You choose a speed and the accelerations $(a_x, a_y)$, then solve algebraically for the loads and tyre forces that would hold that state. Speed is the only quantity carried from one point to the next, and it is integrated in *distance*:
$v_{i+1}^2 = v_i^2 + 2a_x\Delta s$.

A transient simulation removes the equilibrium assumption. The car carries many quantities that have **memory**: they cannot change instantly and must be integrated in *time*.

| Concept | QSS | Transient |
|---|---|---|
| Independent variable | distance $s$ | time $t$ |
| What is integrated | $v$ only | 15 states: velocities, yaw rate, wheel speeds, loads, position, energy |
| Tyre forces | "whatever is needed", up to the friction limit | produced by **slip**, which results from wheel speeds and body velocities |
| Normal loads | algebraic function of $(a_x, a_y, v)$ | lag towards their quasi-static value |
| Path | imposed: the car is *on* the line | the car must be *steered* onto the line by a driver model |
| Inputs | none (limits only) | steer angles, 4 motor torques, brake torque |
| Result | best-case performance envelope | what a specific car + controller actually achieves |

The key mindset change is **causality**. In QSS you specify the *output* (the acceleration) and ask whether it is feasible. In a transient model you specify only the *inputs* (steer and torques), and the accelerations *emerge* from the physics. You never tell the car how fast to turn; you turn the wheels, slip angles build, lateral forces appear, and the car rotates.

That is why a transient lapsim needs a **driver model** (Section 7). Something has to choose the inputs that make the car follow the track.

---

## 2. State-space in one page

A state-space model is a set of first-order ordinary differential equations:

$$\dot{\mathbf x} = f(\mathbf x, \mathbf u, t), \qquad \mathbf y = h(\mathbf x, \mathbf u)$$

- **State $\mathbf x$:** the minimum set of numbers that fully describes the system at an instant. If you know $\mathbf x(t_0)$ and $\mathbf u(t)$ from then on, the future is determined.
- **Input $\mathbf u$:** what you control.
- **Output $\mathbf y$:** what you measure or care about. These are computed from the states and need no integration of their own.

**How to choose states.** A state is any quantity with memory, which in mechanics usually means something that stores energy:

| Energy store | State |
|---|---|
| translational kinetic energy of the chassis | $v_x, v_y$ |
| rotational kinetic energy about z | $r$ |
| rotational kinetic energy of each wheel + motor | $\omega_{FL}, \omega_{FR}, \omega_{RL}, \omega_{RR}$ |
| suspension (springs/dampers) | $F_{z,FL}, \dots, F_{z,RR}$ (simplified, see §4.8) |
| position along / across the track | $s, n, \xi$ |
| battery | $E$ |

Slip ratios, slip angles, tyre forces and accelerations are **not** states. They are algebraic functions of the states. This is an important distinction: tyre force responds *instantly* to slip in this model, and slip responds instantly to wheel speed and body velocity.

The full vectors used here:

$$\mathbf x = [v_x,\ v_y,\ r,\ \omega_{FL},\ \omega_{FR},\ \omega_{RL},\ \omega_{RR},\ F_{z,FL},\ F_{z,FR},\ F_{z,RL},\ F_{z,RR},\ s,\ n,\ \xi,\ E]^T$$

$$\mathbf u = [\delta_{FL},\ \delta_{FR},\ T_{m,FL},\ T_{m,FR},\ T_{m,RL},\ T_{m,RR},\ T_{brake}]^T$$

$$\mathbf y = [a_x,\ a_y,\ \beta,\ \kappa_{FL..RR},\ \alpha_{FL..RR}]^T$$

---

## 3. Coordinate frames and sign conventions

Three frames are used.

```
        BODY FRAME (moves & rotates with the car, origin at CG)

                 x (forward)
                 ▲
          FL ●───┼───● FR        y_FL = +t_f/2   y_FR = −t_f/2
             │   │   │           x_F  = +a
             │   │   │
     y ◄─────┼── CG  │           yaw rate r > 0 = counter-clockwise (turning left)
     (left)  │       │
             │       │           x_R  = −b
          RL ●───────● RR        y_RL = +t_r/2   y_RR = −t_r/2
```

- **Body frame (x forward, y left, z up).** All chassis equations live here. Because this frame rotates with the car, the equations pick up extra terms (see §4.6).
- **Wheel frame.** Rotated from the body frame by the steer angle $\delta_i$ (rear wheels have $\delta = 0$). Tyre slip and tyre forces are defined here.
- **Path (Frenet) frame.** Attached to the track centreline at distance $s$. The car's position is described by $s$ (distance along), $n$ (lateral offset, + left) and $\xi$ (heading relative to the centreline tangent).

Signs:
- **Steer:** positive steer, positive yaw rate and positive lateral acceleration all mean "turning left".
- **Track radius:** positive = left-hand corner, so curvature $\kappa = 1/R$.
- **Spa:** it runs clockwise, so most of its corners have negative radius.

---

## 4. Building the plant, step by step

This follows the order of operations inside `Vehicle.deriv()`: given $(\mathbf x, \mathbf u)$, compute $\dot{\mathbf x}$.

### 4.1 Inputs and actuator saturation

The controller may request any torque. The **plant** then applies what the hardware can actually deliver (`saturate_motors`, `motor_torque_limits`). For each motor, with motor speed $\omega_m = G\,|\omega_{wheel}|$:

$$T_{max} = \min\!\left(T_{peak},\ \frac{P_{motor}}{\omega_m}\right)\cdot \underbrace{\text{fade}(\omega_m)}_{\to 0 \text{ at max rpm}}, \qquad T_{min} = -T_{max}\cdot\text{regen fraction}\ \ (\text{0 if regen is off})$$

After every motor is clipped, the total electrical power is checked against the accumulator limit (80 kW by FSAE rules). If it is exceeded, all driving torques are scaled down by the same factor.

Putting saturation in the plant rather than the controller is deliberate. It means **the model can never produce physically impossible torque**, however badly the controller behaves. This is standard practice: actuators saturate, controllers request.

The friction brake input $T_{brake}$ is clipped to $[0, T_{brake,max}]$ and split by a fixed bias: wheel $i$ gets $\text{bias}_i\cdot T_{brake}$, where $\text{bias}_{FL} = \text{bias}_{FR} = b_f/2$ and $\text{bias}_{RL} = \text{bias}_{RR} = (1-b_f)/2$. This mimics a hydraulic system with a balance bar.

### 4.2 Wheel-centre velocities

The chassis is a rigid body moving with velocity $(v_x, v_y)$ at the CG and rotating at $r$. The velocity of any point at $(x_i, y_i)$ is $\mathbf v_{CG} + \mathbf r \times \mathbf p_i$:

$$v_{x,i} = v_x - r\,y_i, \qquad v_{y,i} = v_y + r\,x_i$$

This is where the **four corners become different**:
- **Front vs rear:** when the car yaws, the front wheels move sideways one way and the rears the other (the $r\,x_i$ term).
- **Left vs right:** the outer wheels travel faster than the inner ones (the $r\,y_i$ term).

A bicycle model collapses the left/right difference. A 4-corner model keeps it, and that matters for torque vectoring and load transfer.

These velocities are then rotated into each wheel's frame:

$$v_{l,i} = v_{x,i}\cos\delta_i + v_{y,i}\sin\delta_i \quad\text{(along the wheel)}, \qquad v_{t,i} = -v_{x,i}\sin\delta_i + v_{y,i}\cos\delta_i \quad\text{(across the wheel)}$$

### 4.3 Slip: the bridge between kinematics and force

A pneumatic tyre only produces force when the contact patch **slips** relative to the road. Two slip quantities are defined:

$$\kappa_i = \frac{\omega_i R - v_{l,i}}{\max(|v_{l,i}|,\ v_{min})} \qquad \text{(slip ratio; } >0 \text{ driving, } <0 \text{ braking)}$$

$$\alpha_i = -\arctan\!\frac{v_{t,i}}{\max(|v_{l,i}|,\ v_{min})} \qquad \text{(slip angle; } >0 \text{ gives a force to the left)}$$

**Why the minus sign on α.** If the wheel is sliding to the left relative to where it points ($v_t > 0$), the road pushes it back to the right. With this convention, positive α always produces positive (leftward) lateral force.

**Why $v_{min}$** (default 4 m/s). Both slips divide by speed. As $v \to 0$, a tiny wheel-speed difference becomes an enormous slip ratio and the equations become singular. Flooring the denominator turns the model into a "slip-velocity" model at low speed. It is physically reasonable, and it also bounds how stiff the equations get (§5). Real FSAE corners are rarely below 8 m/s, so the floor only matters for standing starts.

**Slip is not a state.** It is recomputed from the states at every instant. A slip ratio can only change because $\omega$ or $v$ changes. This is what couples the wheel-spin dynamics to the chassis.

### 4.4 Tyre forces (combined-slip Magic Formula)

**Pure slip.** For pure longitudinal slip, the Pacejka Magic Formula gives

$$F_x = D_x \sin\!\big(C_x \arctan(B_x \kappa)\big), \qquad D_x = \mu_x(F_z)\,F_z$$

and similarly for $F_y(\alpha)$.
- $D$ is the peak force.
- $C$ shapes the curve (between 1 and 2); it sets how much force is left once the tyre is sliding: $\sin(C\pi/2)$ of the peak.
- $B$ is the stiffness factor.

Instead of asking you for $B$, the code places the peak exactly at your chosen $\kappa_{pk}$ / $\alpha_{pk}$ by normalising the slip, $s_x = \kappa/\kappa_{pk}$, and using $B_n = \tan\!\big(\pi/(2C)\big)$. Then $\sin(C\arctan(B_n s_x))$ peaks at $s_x = 1$.

**Combined slip.** A tyre can't give peak $F_x$ and peak $F_y$ at the same time. The model uses the **normalised resultant slip**:

$$\rho = \sqrt{s_x^2 + s_y^2}, \qquad F_x = D_x\,\frac{s_x}{\rho}\sin\!\big(C_x\arctan(B_{n,x}\rho)\big), \qquad F_y = D_y\,\frac{s_y}{\rho}\sin\!\big(C_y\arctan(B_{n,y}\rho)\big)$$

How to read it:
- The total "how hard is the tyre working" is ρ. When ρ = 1 the tyre is at its peak, whatever mix of $\kappa$ and $\alpha$ produced it.
- The force is shared between the x and y directions in proportion to the normalised slips ($s_x/\rho$ and $s_y/\rho$).
- At the limit this traces a **friction ellipse** with semi-axes $D_x, D_y$.
- With only one slip present it reduces exactly to the pure-slip curve.
- The GGV/Tyre tab plots this ("Combined slip" panel).

**Load sensitivity.**

$$\mu(F_z) = \mu_0\left(1 + k_{ls}\,\frac{F_z - F_{z0}}{F_{z0}}\right), \qquad k_{ls} < 0$$

Doubling the load gives *less* than double the force. This is the physics behind most setup work. Lateral load transfer always costs total grip: the outer tyre gains less than the inner tyre loses. The front/rear split of that loss decides the balance (§4.8, §9).

**Linear (small-slip) stiffness.** The initial slope of the curve is what matters for stability:

$$C_\kappa = \frac{D_x\,C_x\,B_{n,x}}{\kappa_{pk}}\ \ [\text{N per unit slip}], \qquad C_\alpha = \frac{D_y\,C_y\,B_{n,y}}{\alpha_{pk}}\ \ [\text{N/rad}]$$

For the default car at static rear load (728 N): $C_\kappa \approx 26{,}000$ N and $C_\alpha \approx 21{,}000$ N/rad per tyre.

### 4.5 From tyre forces to body forces

Each wheel's force is rotated back into the body frame and summed:

$$F_{x,i} = F_{xw,i}\cos\delta_i - F_{yw,i}\sin\delta_i, \qquad F_{y,i} = F_{xw,i}\sin\delta_i + F_{yw,i}\cos\delta_i$$

$$M_z = \sum_i \big(x_i F_{y,i} - y_i F_{x,i}\big)$$

The yaw moment has two parts:
- **Lateral forces** acting at distance $x_i$ from the CG. This is the classic front-vs-rear balance.
- **Longitudinal forces** acting at distance $y_i$. Pushing harder on the right wheels ($y < 0$) creates a positive (left-turning) moment. **This term is torque vectoring.** It only exists in a model with separate left and right wheels.

There is also a subtle effect: under braking, the steered fronts' $F_x$ has a lateral component ($F_{xw}\sin\delta$).

### 4.6 Chassis equations: Newton–Euler in a rotating frame

The velocity vector is $\mathbf V = v_x\hat{\mathbf i} + v_y\hat{\mathbf j}$, but the unit vectors $\hat{\mathbf i}, \hat{\mathbf j}$ rotate with the car at rate $r$. Differentiating:

$$\frac{d\mathbf V}{dt} = \dot v_x\hat{\mathbf i} + \dot v_y\hat{\mathbf j} + v_x\underbrace{\dot{\hat{\mathbf i}}}_{r\hat{\mathbf j}} + v_y\underbrace{\dot{\hat{\mathbf j}}}_{-r\hat{\mathbf i}}
\quad\Rightarrow\quad a_x = \dot v_x - r\,v_y, \qquad a_y = \dot v_y + r\,v_x$$

Newton's law gives $m\,a_x = \sum F_x - D$ and $m\,a_y = \sum F_y$, so the state equations are

$$\boxed{\dot v_x = \frac{\sum F_{x,i} - D}{m} + r\,v_y, \qquad \dot v_y = \frac{\sum F_{y,i}}{m} - r\,v_x, \qquad \dot r = \frac{M_z}{I_z}}$$

with aero drag $D = \tfrac12\rho\,C_dA\,v_x|v_x|$.

> **Common confusion:** $a_y \ne \dot v_y$. In a steady corner at 1.5 g, $\dot v_y = 0$ (the sideslip is constant), yet $a_y = r v_x = v^2/R$. The $r v_x$ term is the centripetal acceleration. If you integrated $a_y$ directly into $v_y$, the car would slide sideways off the corner. The accelerations reported as outputs (and used for load transfer) are $a_x$ and $a_y$, not $\dot v_x$ and $\dot v_y$.

### 4.7 Wheel spin dynamics

Each wheel plus its motor rotor is a rotating inertia acted on by drive torque, brake torque, the tyre's reaction and rolling resistance:

$$\boxed{I_{w,\text{eff}}\ \dot\omega_i = G\eta_g\,T_{m,i}\ -\ \text{bias}_i\,T_{brake}\,\text{sgn}(\omega_i)\ -\ F_{xw,i}\,R\ -\ C_{rr}F_{z,i}R\,\text{sgn}(\omega_i)}$$

**Reflected inertia.** The motor spins $G$ times faster than the wheel. Its kinetic energy is $\tfrac12 I_m(G\omega)^2 = \tfrac12 (G^2 I_m)\omega^2$, so seen from the wheel it looks like an inertia of $G^2 I_m$. For the default car, $13.5^2 \times 2.74\times10^{-4} = 0.050$ kg·m². That is about 40% of the wheel's own 0.12 kg·m², and it noticeably slows how fast wheel speed can change.

**How a torque becomes acceleration.** The causal chain inside this equation is the heart of the transient model:

1. The motor torque spins the wheel up (ω̇ > 0).
2. The slip ratio κ grows.
3. The tyre force $F_{xw}(\kappa)$ grows, which both pushes the car forward and resists the wheel's spin-up (the $-F_{xw}R$ term).
4. The two balance within a few milliseconds at the slip where $F_{xw}R \approx G\eta_g T_m$.

If the torque exceeds what the tyre can return at its peak, there is no balance: ω runs away. That is wheelspin, and the traction control in §7.3 exists to catch it.

The `sgn(ω)` term is a saturated ramp (linear for $|\omega| < 1$ rad/s), so brake torque fades smoothly to zero if a wheel locks instead of flipping sign every time step.

### 4.8 Normal loads and the algebraic loop

Load transfer depends on acceleration, acceleration depends on tyre forces, and tyre forces depend on load:

```
   Fz ──► tyre forces ──► ax, ay ──► load transfer ──► Fz   (circular!)
```

QSS handles this loop by iterating to a fixed point at each point on the track. A time-domain model has three options:

| Option | How | Drawback |
|---|---|---|
| Iterate inside every derivative call | fixed-point solve on each call | slow, can fail to converge near the limit |
| Implicit (DAE) formulation | solve $\dot x$ and $F_z$ together | needs a DAE solver, much more complex |
| **Make $F_z$ a lagged state** ✔ | $\tau\dot F_z = F_z^{ss} - F_z$ | adds a (physically meaningful) delay |

The model uses the third option:

$$\boxed{\tau_{F_z}\,\dot F_{z,i} = F_{z,i}^{ss}(v_x, a_x, a_y) - F_{z,i}}$$

The current loads give the current forces, which give $a_x, a_y$, which give the *target* loads. The actual loads then move towards that target with time constant τ (default 30 ms). There is no circularity.

Physically, τ stands in for the suspension. A real car does not transfer load instantly: the chassis rolls and pitches over a time set by the springs, dampers and inertia. 30 ms is roughly a well-damped FSAE roll response. **Shorter τ** means more QSS-like behaviour; **longer τ** means a "lazier" car.

The target loads (`fz_steady`) are the sum of:

$$F_{z,i}^{ss} = \underbrace{F_{z,i}^{static}}_{\text{weight dist.}} + \underbrace{\tfrac12\rho C_lA v_x^2\cdot\tfrac{\text{aero bal}}{2}}_{\text{downforce}} \mp \underbrace{\frac{m a_x h + D\,h_{cp}}{2L}}_{\text{longitudinal (incl. drag pitch)}} \mp \underbrace{\frac{\text{LLTD}\cdot m a_y h}{t}}_{\text{lateral (per axle)}}$$

- **Longitudinal transfer:** take moments about the front contact patch. The inertial force $m a_x$ acts at the CG height $h$, and the drag acts at $h_{cp}$. Both shift load rearward (when accelerating, or from drag).
- **Lateral transfer:** the total overturning moment $m a_y h$ must be reacted by the two axles. **LLTD** (lateral load transfer distribution) says what fraction the front axle reacts. In a real car LLTD is set by the roll stiffness split (springs, anti-roll bars) and roll-centre heights. Here it is an input. Combined with load sensitivity (§4.4), **LLTD is the main balance tuning knob**: more front LLTD costs the front more grip, giving more understeer.

**What this model cannot do.** There is no roll angle, pitch angle or heave, so you get nothing about ride height, camber change, or how damping shapes the load-transfer transient. There is also no diagonal ("warp") load transfer. This shows up in the linear analysis as one **uncontrollable** mode: no steer or torque input can create a diagonal load pattern in a planar model.

### 4.9 Path (Frenet) coordinates

Rather than integrating global $(X, Y, \psi)$, the car's position is tracked relative to the centreline. Write the car position as $\mathbf P = \mathbf C(s) + n\,\mathbf N(s)$, where $\mathbf C$ is the centreline, $\mathbf T$ its tangent and $\mathbf N$ its left normal. Since $d\mathbf N/ds = -\kappa\mathbf T$:

$$\dot{\mathbf P} = \dot s(1 - n\kappa)\,\mathbf T + \dot n\,\mathbf N$$

The car's velocity resolved along $\mathbf T$ and $\mathbf N$ is $(v_x\cos\xi - v_y\sin\xi,\ v_x\sin\xi + v_y\cos\xi)$. Equating the two expressions:

$$\boxed{\dot s = \frac{v_x\cos\xi - v_y\sin\xi}{1 - n\kappa(s)}, \qquad \dot n = v_x\sin\xi + v_y\cos\xi, \qquad \dot\xi = r - \kappa(s)\,\dot s}$$

Why this is the right choice for a lapsim:
- **The track enters only through $\kappa(s)$,** which is exactly the length/radius segment list. Nothing else about the track is needed.
- **Lap distance is a state.** The lap ends when $s = L$, and the speed reference can be looked up by $s$.
- **Path-tracking errors are states too.** $n$ and $\xi$ are exactly what the driver needs to correct.
- **$1 - n\kappa$ is real geometry.** On the inside of a corner ($n\kappa > 0$) you cover centreline distance faster than your own speed. This is the basic reason a racing line pays off.
- **Straight track = global frame.** On a zero-curvature track, $(s, n, \xi)$ are just $(X, Y, \psi)$. The open-loop tab relies on this.

### 4.10 Energy

$$\dot E = \sum_i P_{el,i}, \qquad P_{el,i} = \begin{cases} T_{m,i}\,\omega_{m,i}/\eta_m & \text{driving} \\ T_{m,i}\,\omega_{m,i}\,\eta_m & \text{regenerating} \end{cases}$$

This state is "pure output": nothing else depends on it. It is integrated alongside the rest so you get energy per lap for free.

---

## 5. Time scales and stiffness

This section explains why transient vehicle models are numerically tricky, and it is the part most people skip.

The model contains dynamics on very different time scales. These are from the linearisation of the default car at 15 m/s on a 20 m radius:

| Mode | Eigenvalue (s⁻¹) | Time constant | Dominant states |
|---|---|---|---|
| Wheel spin (×4) | −196 … −594 | **1.7–5 ms** | $\omega_i$ |
| Load transfer (×4) | −33 … −36 | 30 ms | $F_{z,i}$ |
| Lateral/yaw ("handling") | −13 ± 5.4j | 2.2 Hz, ζ = 0.92 | $v_y, r$ |
| Speed | −0.21 | ~5 s | $v_x$ |

**Why the wheels are so fast.** Linearise the wheel equation around a rolling wheel. With $F_x \approx C_\kappa\kappa$ and $\partial\kappa/\partial\omega = R/v$:

$$\dot\omega \approx -\frac{C_\kappa R^2}{I_{w,\text{eff}}\,v}\,\delta\omega \quad\Rightarrow\quad \lambda_{wheel} \approx -\frac{C_\kappa R^2}{I_{w,\text{eff}}\,v}$$

For the default car: $26{,}000 \times 0.203^2 / (0.17 \times 10) \approx 630$ s⁻¹ at 10 m/s. That is a **1.6 ms** time constant, and it gets **faster as speed drops** (hence the $v_{min}$ floor). Physically: a stiff tyre pushing on a light wheel.

**Stiffness.** A system whose time scales differ by orders of magnitude (here ~3000× between the wheels and $v_x$) is called **stiff**. Explicit integrators (Euler, RK4) are only stable when $h|\lambda|$ is below a threshold for *every* mode, including the fastest one you don't care about:

| Method | Stable for real λ < 0 when |
|---|---|
| Explicit Euler | $h\lvert\lambda\rvert < 2$ |
| RK4 | $h\lvert\lambda\rvert < 2.785$ |

At 4 m/s the wheel mode is about $-1580$ s⁻¹, so RK4 needs $h < 1.8$ ms. If you exceed it, the wheel speeds oscillate with growing amplitude and the simulation explodes in a few steps. This isn't a bug in the physics; it is the integrator.

**How the code handles it.** `Vehicle.stable_dt(x)` re-estimates $\lambda_{wheel}$ for each corner from the current load and speed, and also accounts for the load lag $1/\tau$. Each step then uses

$$h = \min\!\left(dt_{user},\ \frac{2}{\lambda_{max}}\right)$$

Usually the 1 ms user step binds. At low speed with high load, the automatic limit takes over.

**Alternatives you will see in the literature:**
- **Implicit solvers** (BDF, Radau, e.g. `scipy.integrate.solve_ivp(method="Radau")`). These are stable at any step size but need Jacobians, so each step costs more. They are hard to combine with a controller that has discontinuities (saturation, traction-control switching).
- **Tyre relaxation length** (§12). This turns slip into a lagged state. It is more realistic at low speed and it removes the $1/v$ blow-up.
- **Remove the wheel states** by assuming a slip ratio. That is what QSS effectively does.

---

## 6. Numerical integration

`run_transient()` runs the following loop:

```
x ← initial state (QSS speed at s = 0, wheels rolling, loads settled)
repeat
    u ← Driver.control(x)                        # zero-order hold for this step
    h ← min(dt, stable_dt(x))
    k1 = f(x,          u)
    k2 = f(x + h/2·k1, u)
    k3 = f(x + h/2·k2, u)                         # classic 4th-order Runge-Kutta
    k4 = f(x + h·k3,   u)
    x_new = x + h/6·(k1 + 2k2 + 2k3 + k4)
    if s_new ≥ L: interpolate the exact crossing time; stop
    abort if |ξ| > 1.3 rad, |n| > 12 m, non-finite state, car stopped, or t > 3× QSS time
    log every log_dt
```

- **Zero-order hold.** The controller is evaluated once per step and held through all four RK4 stages. This mirrors a real digital controller (the ECU samples, then holds) and keeps the RK4 stages consistent. Track curvature, on the other hand, *is* re-evaluated at each stage's $s$, so segment boundaries are handled accurately.
- **Finish line.** Linear interpolation between the last two states removes the up-to-one-step quantisation error from the lap time.
- **Cost.** One derivative call is about 18 µs in pure Python. A 7 km Spa lap is about 230 s of simulated time ≈ 230,000 steps ≈ 1M derivative calls ≈ 25 s wall-clock.

---

## 7. The driver / controller

QSS needs no driver because the path is imposed. In a transient model the car goes wherever the physics sends it, so a controller has to close the loop. This one is deliberately simple and transparent: it is a *reasonable* driver, not an optimal one.

```
                ┌─────────────────────────── Driver.control(x) ────────────────────────────┐
  track, QSS ──►│ lateral:  pure pursuit + yaw-rate feedback ──► δ ──► Ackermann ──► δFL, δFR │
     profile    │ longit.:  v_ref(s) feed-forward + P ─────────► F_req                       │
                │ yaw:      torque vectoring ─────────────────► Mz                           │
                │ allocate: Fz-proportional split + Mz + regen/friction blend ──► Tm_i, T_b │
                │ protect:  slip-ratio traction control ───────────────────────► Tm_i, T_b  │
                └───────────────────────────────────────────────────────────────────────────┘
```

### 7.1 Steering: pure pursuit with yaw-rate feedback

1. **Pick a target point** on the centreline a distance $L_p = \max(L_{min},\ t_{p}\,v_x)$ ahead. Defaults: $t_p = 0.45$ s and $L_{min} = 3$ m, so about 7 m at 15 m/s.
2. **Express the target** in the car's frame: forward distance $x_l$, lateral $y_l$, straight-line distance $\ell$.
3. **Find the circle** that starts at the car, is tangent to its heading, and passes through the target. Its radius satisfies $R^2 = x_l^2 + (R - y_l)^2$, so

   $$k_{path} = \frac{1}{R} = \frac{2\,y_l}{\ell^2}$$

4. **Steer:**

   $$\delta = \underbrace{\arctan(L\,k_{path})}_{\text{kinematic feed-forward}} + \underbrace{k_{yaw}\,(r_{ref} - r)}_{\text{yaw-rate feedback}}, \qquad r_{ref} = v_x\,k_{path}$$

5. **Split per wheel** with the Ackermann factor and clip to ±25°.

Why this structure works:
- **Exact on arcs.** If the car is on the centreline of a constant-radius corner, the pursuit circle *is* the corner, so $k_{path} = \kappa$ and the feed-forward gives exactly the kinematic steer.
- **Self-correcting.** Any offset $n$ or heading error $\xi$ moves the target point in the car's frame, and the steer corrects for it automatically.
- **Covers what kinematics can't.** A real car needs more (understeer) or less (oversteer) steer than kinematics predicts, and needs time to build yaw. The yaw-rate term adds steer whenever the car isn't rotating as fast as the path needs.
- **Preview is the main tuning knob.** Short preview gives tight tracking but twitchy, possibly oscillating steering. Long preview is smooth but cuts corners and runs wide on exit.

### 7.2 Speed: tracking the QSS profile

The QSS result is used as the speed plan: "this is what the car could do if it were in equilibrium everywhere".

$$v_{ref} = \sigma\,v_{QSS}(s + v_x t_{look}), \qquad a_{ref} = \sigma^2 a_{QSS}(s + v_x t_{look})$$

$$F_{req} = m\big(a_{ref} + k_v (v_{ref} - v_x)\big) + D + C_{rr}\textstyle\sum F_z$$

- **Feed-forward** ($m\,a_{ref}$) does most of the work: the controller already knows the car should be braking at 2 g here. **Feedback** ($k_v = 4$ s⁻¹) only trims the error. A pure-feedback speed controller would always lag behind.
- **Look-ahead** ($t_{look}$ = 0.12 s) compensates for the delays in the chain (wheel slip, load transfer), so braking starts on time.
- **Speed scale** σ lets you run the car at, say, 97% of the QSS limit. The $\sigma^2$ on acceleration is because $a = v\,dv/ds$ scales with $v^2$. Use σ < 1 if the car runs off the track: the QSS limit assumes perfect equilibrium, which a transient car can't quite achieve.

Note that the transient lap can't be *faster* than QSS with this driver, because it chases the QSS profile. The question it answers is **"how much of the QSS potential can this car + controller actually deliver?"**

### 7.3 Torque allocation

This is where the 4-corner, 4-motor model earns its keep.

**Driving** ($F_{req} > 0$):

$$F_{x,i}^{des} = F_{req}\frac{F_{z,i}}{\sum_{driven}F_z} \pm \frac{M_z}{t_f + t_r}, \qquad T_{m,i} = \frac{F_{x,i}^{des}\,R}{G\,\eta_g}$$

- **Load-proportional split.** With roughly equal friction coefficients, every tyre ends up at the same fraction of its grip, so no single tyre saturates first. It is the simplest allocation that is near-optimal.
- **Torque vectoring.** $M_z = k_{tv}I_z(r_{ref} - r)$ adds force on one side and removes it from the other (+ on the right wheels for a left-turning moment). It acts like a fast, separate yaw actuator that doesn't depend on steering.

**Braking** ($F_{req} < 0$):

1. Compute each wheel's desired braking force (load-proportional, plus torque vectoring).
2. Each driven wheel has a regen capacity from its motor curve.
3. The total shortfall across wheels becomes a friction brake request, $T_{brake} = R\sum\max(0,\ -F^{des}_{x,i} - F^{regen,cap}_i)$.
4. The bias valve splits $T_{brake}$ front/rear. Each motor then supplies the *difference* between its wheel's target and what the friction brake already provides.

So the motors correct for the fixed hydraulic bias. This is how real EV brake blending works.

**Traction control.** If a wheel's slip ratio exceeds the target $\kappa_t = 0.8\,\kappa_{pk}$:

$$T_{m,i} \leftarrow T_{m,i}\cdot\max\!\Big(0,\ 1 - 4\,\frac{|\kappa_i| - \kappa_t}{\kappa_{pk}}\Big)$$

The same factor applies to regen. The worst wheel's factor scales the friction brake (an ABS-like effect). Because slip settles in about 2 ms (§9) and the controller runs every 1 ms, this simple proportional cut is enough to stop runaway wheelspin.

**Why controller and plant time scales matter.** A feedback loop must be slower than the dynamics it acts on, but faster than the disturbances it rejects:

| Loop | Acts on | Typical speed |
|---|---|---|
| Traction control | wheel modes | ~ms |
| Torque vectoring / yaw feedback | handling mode | ~10–100 ms |
| Steering preview | path | ~0.5 s |
| Speed feedback | $v_x$ | ~0.25 s |

If you crank a gain up until its loop is as fast as the dynamics underneath it, you get oscillation. That is the most common way to break this simulation.

---

## 8. Linearisation: reading the A matrix

The State-space tab can **trim** the car (find inputs and states where $\dot{\mathbf x} = 0$ for a given speed and radius) and then **linearise** around that point:

$$\delta\dot{\mathbf x} = A\,\delta\mathbf x + B\,\delta\mathbf u, \qquad A = \frac{\partial f}{\partial \mathbf x}, \quad B = \frac{\partial f}{\partial \mathbf u}$$

The Jacobians are computed by central finite differences, so they are exact to rounding error for whatever nonlinear model you have.

**How to read it:**
- **Eigenvalues** of $A$ (restricted to the 11 dynamic states) are the natural modes. A negative real part means stable; the time constant is $1/|\text{Re}|$. Complex pairs oscillate at $\omega_n = |\lambda|$ with damping $\zeta = -\text{Re}/|\lambda|$.
- **Participation factors** (the "dominant states" column) tell you *which states* make up each mode, so you can say "the 2.2 Hz mode is the yaw/sideslip mode".
- **Individual entries** have physical meaning. For example:
  - $\partial\dot r/\partial v_y$ is the yaw stiffness of the car (the static-margin effect);
  - $\partial\dot\omega_i/\partial\omega_i$ is the wheel mode above;
  - $B$'s $\partial\dot r/\partial T_{m,i}$ column shows how much yaw each motor can create, which is the torque-vectoring authority.
- **Controllability (PBH test)** checks whether each mode can be influenced by *some* input. Expect 10/11; the one exception is the warp load mode (§4.8).

**What you can use it for:**
- Comparing handling across setups (does more rear LLTD make the yaw mode less damped?).
- Designing controllers properly (LQR for torque vectoring, a gain-scheduled steering controller).
- Exporting to MATLAB/Simulink (`.mat` export).

**Remember:** the linear model is only valid **near the trim point**. At the limit, the tyre slope (and so every stiffness) changes sharply. Trim at several speeds and radii to see how the modes move.

---

## 9. Worked examples from the default car

These are reproducible in the GUI (State-space tab) or the Python API.

**Step steer: 2° at 15 m/s, applied at t = 0.1 s**
- Steady-state yaw rate: 19.0 °/s, giving $a_y$ = 0.50 g.
- Yaw rate reaches 90% of steady state **76 ms** after the step.
- Overshoot is only 0.8%, consistent with the handling-mode damping of ζ ≈ 0.92.
- The outer-front load reaches 63% of its final change after **~50 ms**. That is longer than τ = 30 ms, because the lateral acceleration driving it is itself still building. **Lags stack.**

**Torque step: 21 Nm on all motors from coast at 15 m/s**
- Rear slip ratio reaches 90% of its final value (κ ≈ 0.021) in about **2 ms**, consistent with the ~1.6–5 ms wheel modes.
- This is why the wheel states barely affect lap time but completely determine the integration step.

**Steady cornering trim: 15 m/s on R = 20 m ($a_y$ = 1.15 g)**
- Kinematic (Ackermann) steer would be $L/R$ = 4.38°. The trim needs **4.66°**, so the car **understeers** at this lateral acceleration.
- Yet the *linear* understeer gradient from static cornering stiffnesses is about 0 °/g (essentially neutral).
- The difference comes entirely from the nonlinearity: at 1.15 g the loads go from 645/728 N static to 360–1240 N, load sensitivity cuts the more heavily loaded tyres' effective grip, and the axles saturate at different rates.
- This is exactly the kind of result QSS (balance assumed) and linear bicycle models (no load transfer) can't show.

**Spa lap**
- QSS: 228.5 s. Transient: 229.7 s (+0.55%).
- Max path offset 1.2 m, max sideslip 5.7°.

---

## 10. Why the transient lap is slower than QSS

Each item below is a way real dynamics fall short of the QSS assumption that the car is "always at the GGV limit, instantly":

1. **Load-transfer lag.** On corner entry the outer tyres haven't been loaded yet, so the grip available lags behind what the GGV assumed.
2. **Yaw build-up.** The car needs about 50–100 ms to reach the yaw rate a corner requires. During that time it is either under-rotated (runs wide) or the driver must start turning earlier.
3. **Slip control margin.** Traction control holds κ at 80% of the peak slip, giving up a little force for stability.
4. **Combined-slip transitions.** QSS jumps along the GGV edge; the real car has to move through the interior of the g-g diagram during trail-braking. Compare the g-g scatter for the two methods in the GUI.
5. **Path error.** Running 0.5–1 m wide on exit lengthens the path ($1 - n\kappa$ works against you on the outside).
6. **Fixed brake bias.** QSS assumes ideal brake distribution at every instant; the hydraulic bias only matches it at one deceleration. Regen correction helps but is limited by motor torque.

A typical gap is **0.5–2%**. Bigger gaps mean the car or controller is poorly matched to the track: too little damping, too much LLTD swing, or badly tuned driver gains. **That is the information QSS can't give you.**

---

## 11. Tuning and troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| "Vehicle left the track" early in the lap | Speed target too ambitious for the transient car | `speed_scale` 0.95–0.98; increase `speed_lookahead` |
| Steering oscillates (δ zig-zags) | Preview too short or yaw gain too high | Increase `preview_time` or `preview_min`; reduce `k_yaw` |
| Car runs wide in every corner | Preview too long (cuts in, then washes out) or understeer | Reduce `preview_time`; increase `k_yaw` or `k_tv` |
| Yaw rate oscillates mid-corner | Torque vectoring gain too high | Reduce `k_tv` (try 3–5) |
| Slip ratio spikes / wheels run away | Traction control off or slip target above the tyre peak | Enable TC; `slip_target` ≤ 0.9 |
| Simulation diverges (NaN) | Step too large for the wheel modes; extreme parameters (very low $I_w$, very high $C_\kappa$) | Reduce `dt`; raise `v_slip_min` |
| Very slow run | Long track, small dt | `dt` 1–2 ms is fine at racing speeds; increase `log_dt` |
| Transient ≫ QSS lap time | Car can't follow the QSS plan (long τ, poor balance) | Compare speed traces to see where time is lost; check g-g scatter and slip angles there |

---

## 12. Limitations and how to extend the model

| Limitation | Consequence | Extension |
|---|---|---|
| **No tyre relaxation length** | Tyres respond instantly; stiff wheel modes; low-speed behaviour approximate | Add slip states: $\frac{\sigma_\kappa}{\lvert v\rvert}\dot\kappa' + \kappa' = \kappa$ and $\frac{\sigma_\alpha}{\lvert v\rvert}\dot\alpha' + \alpha' = \alpha$ (8 states). Forces then use κ′, α′. Relaxation lengths are ~0.1–0.3 m for FSAE tyres. |
| **No roll/pitch/heave DOF** | No camber or ride-height effects; LLTD is an input instead of a result of springs and anti-roll bars | Add roll angle φ and rate with $I_x\ddot\phi = m a_y h_{rc} - K_\phi\phi - C_\phi\dot\phi$; compute per-axle load transfer from the spring/ARB moments. Same idea for pitch. |
| **Planar (no elevation/banking)** | Eau Rouge compression and crests are ignored | Add $g\sin\theta(s)$ grade and $g\cos\theta$ load terms from an elevation profile |
| **Motor torque is instantaneous** | Infinite actuator bandwidth | First-order lag per motor, $\tau_m \dot T = T_{cmd} - T$, with τ_m ≈ 5–10 ms |
| **Centreline driving** | Lap time is conservative vs a racing line | Pre-compute a minimum-curvature line and feed its $\kappa(s)$; or use a model-predictive (MPC) driver |
| **Simple driver gains** | Not optimal; needs manual tuning | LQR on the linearised model for steering + torque vectoring; gain-schedule with speed |
| **Constant μ, no temperature or battery state** | No degradation over an endurance run | Tyre temperature states, battery SOC and voltage-sag model limiting $P_{max}$ |

---

## 13. Code map

| Concept | Where |
|---|---|
| State/input definitions | `STATE_NAMES`, `INPUT_NAMES` in [vehicle.py](./fsae_lapsim/vehicle.py) |
| Full derivative $f(\mathbf x, \mathbf u)$ | `Vehicle.deriv` |
| Tyre model | `Vehicle.tire`, `Vehicle.tire_peak` |
| Motor limits / power cap | `Vehicle.motor_torque_limits`, `Vehicle.saturate_motors` |
| Quasi-static loads | `Vehicle.fz_steady` |
| Ackermann split | `Vehicle.steer_split` |
| Step-size limit, RK4 | `Vehicle.stable_dt`, `Vehicle.rk4_step` |
| Open-loop simulation | `Vehicle.simulate` |
| Trim and linearisation | `Vehicle.trim`, `Vehicle.linearize` |
| Driver (steer, speed, allocation, TC) | `Driver.control` in [transient.py](./fsae_lapsim/transient.py) |
| Driver settings | `DriverSettings` |
| Lap loop, finish line, abort checks | `run_transient` |
| Track curvature and pose lookup | `Track.curvature_at`, `Track.pose_at` in [track.py](./fsae_lapsim/track.py) |
