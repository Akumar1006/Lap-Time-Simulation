# Transient Model – Complete Equation Reference

This is the complete set of equations implemented in the transient lap simulation, written out explicitly and **exactly as coded**, including every limit, floor and special case. Each block names the function that implements it.

| Part | File |
|---|---|
| Vehicle model (plant) $\dot{\mathbf x} = f(\mathbf x, \mathbf u, \kappa(s))$ | [`fsae_lapsim/vehicle.py`](./fsae_lapsim/vehicle.py) |
| Driver / controller $\mathbf u = g(\mathbf x)$ and the time loop | [`fsae_lapsim/transient.py`](./fsae_lapsim/transient.py) |
| Track centreline and curvature | [`fsae_lapsim/track.py`](./fsae_lapsim/track.py) |

For *why* the equations look like this, see [TRANSIENT_DYNAMICS.md](./TRANSIENT_DYNAMICS.md). Default values of every symbol are in [§13](#13-symbols-code-names-and-default-values).

---

## Contents

0. [Notation](#0-notation)
1. [State, input and output vectors](#1-state-input-and-output-vectors)
2. [Constants derived from the parameters](#2-constants-derived-from-the-parameters)
3. [Input saturation (actuator limits)](#3-input-saturation-actuator-limits)
4. [Wheel kinematics and slip](#4-wheel-kinematics-and-slip)
5. [Tyre forces](#5-tyre-forces)
6. [Forces and moment on the chassis](#6-forces-and-moment-on-the-chassis)
7. [Quasi-static normal-load targets](#7-quasi-static-normal-load-targets)
8. [The state equations](#8-the-state-equations)
9. [Outputs](#9-outputs)
10. [Track geometry](#10-track-geometry)
11. [Driver / controller](#11-driver--controller)
12. [Numerical integration and the lap loop](#12-numerical-integration-and-the-lap-loop)
13. [Symbols, code names and default values](#13-symbols-code-names-and-default-values)
14. [Simplifications as implemented](#14-simplifications-as-implemented)

---

## 0. Notation

- **Corner index:** $i \in \{1,2,3,4\} = \{\text{FL}, \text{FR}, \text{RL}, \text{RR}\}$.
- **Body frame:** origin at the CG, $x$ forward, $y$ left, $z$ up. Yaw rate $r$ is positive counter-clockwise (turning left).
- **Track sign convention:** curvature $\kappa > 0$ for a left-hand corner; lateral offset $n > 0$ to the left of the centreline.
- **Clipping:** $\operatorname{clip}(z, a, b) = \min(\max(z, a), b)$.
- **Saturated sign:** used for brake and rolling-resistance direction, with $\omega$ in rad/s:

$$\operatorname{sat}(\omega) = \operatorname{clip}(\omega,\,-1,\,1)$$

- **Positive part:** $[z]^+ = \max(z, 0)$.
- **Gravity:** $g = 9.81\ \text{m/s}^2$.

---

## 1. State, input and output vectors

$$\mathbf x = [\,v_x,\ v_y,\ r,\ \omega_1,\ \omega_2,\ \omega_3,\ \omega_4,\ F_{z1},\ F_{z2},\ F_{z3},\ F_{z4},\ s,\ n,\ \xi,\ E\,]^T \quad (15)$$

$$\mathbf u = [\,\delta_1,\ \delta_2,\ T_{m1},\ T_{m2},\ T_{m3},\ T_{m4},\ T_b\,]^T \quad (7)$$

$$\mathbf y = [\,a_x,\ a_y,\ \beta,\ \kappa_1..\kappa_4,\ \alpha_1..\alpha_4\,]^T \quad (11)$$

| Symbol | Meaning | Unit |
|---|---|---|
| $v_x, v_y$ | CG velocity in body axes | m/s |
| $r$ | yaw rate | rad/s |
| $\omega_i$ | wheel spin speed | rad/s |
| $F_{zi}$ | tyre normal load | N |
| $s, n, \xi$ | distance along centreline, lateral offset, heading relative to centreline | m, m, rad |
| $E$ | battery energy used | J |
| $\delta_1, \delta_2$ | front-left / front-right road-wheel steer (rears: $\delta_3 = \delta_4 = 0$) | rad |
| $T_{mi}$ | motor **shaft** torque, before the gearbox | N·m |
| $T_b$ | total friction-brake torque, all wheels | N·m |

---

## 2. Constants derived from the parameters

`Vehicle.update()`

**Geometry and wheel positions.**

$$a = L\,(1 - w_f), \qquad b = L\,w_f$$

$$(x_i) = (a,\ a,\ -b,\ -b), \qquad (y_i) = \left(\tfrac{t_f}{2},\ -\tfrac{t_f}{2},\ \tfrac{t_r}{2},\ -\tfrac{t_r}{2}\right)$$

**Driven wheels.** $d_i = 1$ if wheel $i$ has a motor, otherwise $0$. With AWD, $d = (1,1,1,1)$; with RWD, $d = (0,0,1,1)$.

**Effective wheel inertia** (the rotor reflected through the gearbox):

$$I_{\text{eff},i} = I_w + d_i\, G_r^2\, I_{\text{mot}}$$

**Static loads.**

$$F_{z,1}^{st} = F_{z,2}^{st} = \frac{m g\, b}{2L}, \qquad F_{z,3}^{st} = F_{z,4}^{st} = \frac{m g\, a}{2L}$$

**Brake bias per wheel.**

$$(\beta_{b,i}) = \left(\tfrac{b_f}{2},\ \tfrac{b_f}{2},\ \tfrac{1-b_f}{2},\ \tfrac{1-b_f}{2}\right)$$

**Tyre shape constants** (place the peak force at the chosen peak slip):

$$B_{nx} = \tan\!\frac{\pi}{2C_x}, \qquad B_{ny} = \tan\!\frac{\pi}{2C_y}$$

**Aero and motor constants.**

$$q = \tfrac12 \rho_{air}, \qquad \omega_{m,\max} = \frac{2\pi\, N_{\max}}{60}$$

**Total gear factor** (ratio × efficiency), used for torque at the wheel:

$$G = G_r\, \eta_g$$

---

## 3. Input saturation (actuator limits)

These are applied **inside the plant** (`Vehicle.deriv`, `Vehicle.saturate_motors`, `Vehicle.motor_torque_limits`). Whatever the controller requests, the plant only uses the clipped values below.

### 3.1 Steering and friction brake

$$\delta_i^* = \operatorname{clip}(\delta_i,\, -\delta_{\max},\, \delta_{\max}), \qquad T_b^* = \operatorname{clip}(T_b,\, 0,\, T_{b,\max})$$

### 3.2 Per-motor torque envelope

For driven wheel $i$ at motor speed $\omega_{m,i} = G_r\,|\omega_i|$:

$$T_{\max,i} = \begin{cases} P_{\text{mot}}/\omega_{m,i} & \text{if } \omega_{m,i}\,T_{\text{peak}} > P_{\text{mot}} \\ T_{\text{peak}} & \text{otherwise} \end{cases}$$

$$T_{\min,i} = \begin{cases} -\,T_{\max,i}\; f_{\text{regen}} & \text{if regen enabled} \\ 0 & \text{otherwise} \end{cases}$$

The minimum is computed **before** the fade below, so regen torque does not fade near max rpm.

**Fade near maximum rpm:**

$$\text{if } \omega_{m,i} > 0.98\,\omega_{m,\max}: \qquad T_{\max,i} \leftarrow T_{\max,i}\cdot \left[\frac{\omega_{m,\max} - \omega_{m,i}}{0.02\,\omega_{m,\max}}\right]^+$$

**Clip the commanded torque:**

$$T_{mi}^{*} = \operatorname{clip}(T_{mi},\, T_{\min,i},\, T_{\max,i}) \cdot d_i$$

### 3.3 Electrical power and the accumulator limit

**Motor mechanical power** (signed, uses the *signed* wheel speed):

$$P_{\text{mech},i} = T^*_{mi}\, \omega_i\, G_r$$

**Electrical power:**

$$P_{el,i} = \begin{cases} P_{\text{mech},i} / \eta_m & P_{\text{mech},i} > 0 \ \text{(driving)} \\ P_{\text{mech},i}\, \eta_m & \text{otherwise (regenerating)} \end{cases}$$

**Accumulator cap.** Let $P^+ = \sum_i [P_{el,i}]^+$. If $P^+ > P_{\text{tot}}$, every motor that is *drawing* power is scaled down by the same factor:

$$k_P = \frac{P_{\text{tot}}}{P^+}, \qquad T^*_{mi} \leftarrow k_P\,T^*_{mi},\quad P_{el,i} \leftarrow k_P\,P_{el,i} \quad \text{for all } i \text{ with } P_{el,i} > 0$$

---

## 4. Wheel kinematics and slip

`Vehicle.deriv`

**Velocity of each wheel centre, body frame** (rigid-body motion $\mathbf v_{CG} + \mathbf r \times \mathbf p_i$):

$$v_{x,i} = v_x - r\,y_i, \qquad v_{y,i} = v_y + r\,x_i$$

**Rotated into the wheel frame** by the steer angle, with $c_i = \cos\delta_i^*$ and $s_i = \sin\delta_i^*$:

$$v_{l,i} = v_{x,i}\,c_i + v_{y,i}\,s_i \quad\text{(along the wheel)}, \qquad v_{t,i} = -v_{x,i}\,s_i + v_{y,i}\,c_i \quad\text{(across the wheel)}$$

**Slip denominator**, floored at low speed:

$$V_i = \max(|v_{l,i}|,\ v_{\min})$$

**Slip ratio and slip angle:**

$$\boxed{\kappa_i = \frac{\omega_i R - v_{l,i}}{V_i}}, \qquad \boxed{\alpha_i = -\arctan\frac{v_{t,i}}{V_i}}$$

---

## 5. Tyre forces

`Vehicle.tire` (combined-slip, normalised Magic Formula)

**Load used by the tyre** (a negative load state gives no force):

$$\hat F_{z,i} = [F_{z,i}]^+$$

**Load-sensitive friction scale**, floored at 0.1:

$$\lambda_i = \max\!\left(0.1,\ 1 + k_{ls}\,\frac{\hat F_{z,i} - F_{z0}}{F_{z0}}\right)$$

**Peak forces:**

$$D_{x,i} = \mu_x\,\lambda_i\,\hat F_{z,i}, \qquad D_{y,i} = \mu_y\,\lambda_i\,\hat F_{z,i}$$

**Normalised slips and the combined slip:**

$$\sigma_{x,i} = \frac{\kappa_i}{\kappa_{pk}}, \qquad \sigma_{y,i} = \frac{\alpha_i}{\alpha_{pk}}, \qquad \rho_i = \sqrt{\sigma_{x,i}^2 + \sigma_{y,i}^2}$$

**Forces in the wheel frame:**

$$\boxed{F_{xw,i} = D_{x,i}\ \sigma_{x,i}\ \frac{\sin\!\big(C_x \arctan(B_{nx}\,\rho_i)\big)}{\rho_i}}, \qquad
\boxed{F_{yw,i} = D_{y,i}\ \sigma_{y,i}\ \frac{\sin\!\big(C_y \arctan(B_{ny}\,\rho_i)\big)}{\rho_i}}$$

**Small-slip limit**, used when $\rho_i < 10^{-9}$; it is the exact limit of the expressions above:

$$F_{xw,i} = D_{x,i}\,C_x B_{nx}\,\sigma_{x,i}, \qquad F_{yw,i} = D_{y,i}\,C_y B_{ny}\,\sigma_{y,i}$$

**Properties.**
- **Pure slip** ($\sigma_y = 0$): $F_x = D_x \sin(C_x \arctan(B_{nx}\sigma_x))$, which peaks at $\kappa = \kappa_{pk}$ with value $D_x$.
- **Combined slip at the peak** ($\rho = 1$): $(F_{xw}/D_x)^2 + (F_{yw}/D_y)^2 = 1$, a friction ellipse.

**Small-slip stiffness** (used in §12):

$$C_{\kappa,i} = \frac{D_{x,i}\,C_x B_{nx}}{\kappa_{pk}}, \qquad C_{\alpha,i} = \frac{D_{y,i}\,C_y B_{ny}}{\alpha_{pk}}$$

---

## 6. Forces and moment on the chassis

`Vehicle.deriv`

**Wheel forces rotated into the body frame:**

$$F_{x,i} = F_{xw,i}\,c_i - F_{yw,i}\,s_i, \qquad F_{y,i} = F_{xw,i}\,s_i + F_{yw,i}\,c_i$$

**Aero drag:**

$$D = q\, C_dA\; v_x\,|v_x|$$

**Accelerations** (inertial, in body axes):

$$a_x = \frac{\sum_i F_{x,i} - D}{m}, \qquad a_y = \frac{\sum_i F_{y,i}}{m}$$

**Yaw moment about the CG:**

$$M_z = \sum_i \big(x_i\,F_{y,i} - y_i\,F_{x,i}\big)$$

---

## 7. Quasi-static normal-load targets

`Vehicle.fz_steady(v_x, a_x, a_y)`, evaluated with the $a_x, a_y$ from §6:

**Downforce and the drag used for pitch:**

$$Q = q\,v_x^2, \qquad F_{down} = Q\,C_lA, \qquad D_p = Q\,C_dA$$

**Aero load per wheel:**

$$\ell_f = \frac{F_{down}\,\varepsilon_f}{2}, \qquad \ell_r = \frac{F_{down}\,(1-\varepsilon_f)}{2}$$

**Longitudinal transfer per wheel:**

$$\Delta_x = \frac{m\,a_x\,h + D_p\,h_{cp}}{2L}$$

**Lateral transfer per axle:**

$$\Delta_{y,f} = \frac{\Lambda_f\, m\,a_y\,h}{t_f}, \qquad \Delta_{y,r} = \frac{(1-\Lambda_f)\, m\,a_y\,h}{t_r}$$

**Targets:**

$$\begin{aligned}
F_{z,1}^{ss} &= F_{z,1}^{st} + \ell_f - \Delta_x - \Delta_{y,f} &\quad(\text{FL}) \\
F_{z,2}^{ss} &= F_{z,2}^{st} + \ell_f - \Delta_x + \Delta_{y,f} &\quad(\text{FR}) \\
F_{z,3}^{ss} &= F_{z,3}^{st} + \ell_r + \Delta_x - \Delta_{y,r} &\quad(\text{RL}) \\
F_{z,4}^{ss} &= F_{z,4}^{st} + \ell_r + \Delta_x + \Delta_{y,r} &\quad(\text{RR})
\end{aligned}$$

Signs: accelerating ($a_x > 0$) moves load rearward; a left turn ($a_y > 0$) moves load to the right-hand wheels.

---

## 8. The state equations

`Vehicle.deriv` returns these 15 derivatives. All quantities on the right-hand side come from §3–§7.

### Chassis (3)

$$\dot v_x = a_x + r\,v_y$$

$$\dot v_y = a_y - r\,v_x$$

$$\dot r = \frac{M_z}{I_z}$$

### Wheel spin (4), for $i = 1..4$

$$I_{\text{eff},i}\ \dot\omega_i = G\,T^*_{mi} \;-\; \beta_{b,i}\,T_b^*\,\operatorname{sat}(\omega_i) \;-\; F_{xw,i}\,R \;-\; C_{rr}\,\hat F_{z,i}\,R\,\operatorname{sat}(\omega_i)$$

### Normal loads (4), for $i = 1..4$

$$\dot F_{z,i} = \frac{F_{z,i}^{ss} - F_{z,i}}{\tau_{F_z}}$$

### Path coordinates (3)

With $\kappa = \kappa(s)$ from §10 and the denominator floored at 0.05:

$$\Gamma = \max(1 - n\,\kappa,\ 0.05)$$

$$\dot s = \frac{v_x\cos\xi - v_y\sin\xi}{\Gamma}$$

$$\dot n = v_x\sin\xi + v_y\cos\xi$$

$$\dot\xi = r - \kappa\,\dot s$$

### Energy (1)

Signed: regeneration reduces $E$.

$$\dot E = \sum_i P_{el,i}$$

---

## 9. Outputs

`Vehicle.outputs`:

$$\mathbf y = \big[\,a_x,\ a_y,\ \beta,\ \kappa_{1..4},\ \alpha_{1..4}\,\big], \qquad \beta = \operatorname{atan2}\!\big(v_y,\ \max(|v_x|, 10^{-3})\big)$$

The transient lap simulation also logs $F_{x,i}$, $F_{y,i}$ (body frame), $P_{el,i}$, and the *saturated* inputs $\delta^*_{1,2}$, $T^*_{m,1..4}$, $T_b^*$.

---

## 10. Track geometry

`track.py`

**Curvature.** The track is a list of segments $j$ with length $L_j$ and radius $R_j$:

$$\kappa_j = \begin{cases} 1/R_j & R_j \ne 0 \\ 0 & \text{straight} \end{cases}$$

The curvature is piecewise constant along the track:

$$\kappa(s) = \kappa_j \quad\text{for } S_j \le s < S_{j+1}, \qquad S_j = \sum_{k<j} L_k$$

- **Closed tracks:** $s$ wraps, $s \leftarrow s \bmod L_{track}$.
- **Open tracks:** $\kappa = 0$ beyond the end.

**Centreline pose** $(X_c, Y_c, \theta)(s)$. Starting from $(0,0,0)$, each segment is integrated exactly from its start pose $(X_0, Y_0, \theta_0)$ over a distance $\ell$ into the segment:

$$\kappa_j \neq 0:\quad \theta = \theta_0 + \kappa_j \ell, \quad X_c = X_0 + \frac{\sin\theta - \sin\theta_0}{\kappa_j}, \quad Y_c = Y_0 - \frac{\cos\theta - \cos\theta_0}{\kappa_j}$$

$$\kappa_j = 0:\quad \theta = \theta_0, \quad X_c = X_0 + \ell\cos\theta_0, \quad Y_c = Y_0 + \ell\sin\theta_0$$

On an open track, poses beyond the end are extrapolated in a straight line.

**Car pose in global coordinates** (used by the driver and for the track map):

$$X = X_c - n\sin\theta, \qquad Y = Y_c + n\cos\theta, \qquad \psi = \theta + \xi$$

---

## 11. Driver / controller

`Driver.control`, called once per integration step. Its output is held constant through the RK4 stages.

### 11.1 Reference from the QSS solution

The QSS lap simulation provides $v_{QSS}(s)$ and $a_{QSS}(s)$ on a 0.5 m grid, linearly interpolated. The look-up distance (closed tracks wrap; open tracks clamp to the end) is

$$s_{ref} = s + v_x\,t_{look}$$

and the references are scaled by $\sigma_v$:

$$v_{ref} = \sigma_v\, v_{QSS}(s_{ref}), \qquad a_{ref} = \sigma_v^2\, a_{QSS}(s_{ref})$$

### 11.2 Steering: pure pursuit + yaw-rate feedback

**Car pose** from §10. **Preview distance and target point:**

$$L_p = \max(L_{p,\min},\ t_p\,v_x), \qquad (X_t, Y_t) = \big(X_c, Y_c\big)(s + L_p)$$

**Target in the car's frame:**

$$\Delta X = X_t - X, \quad \Delta Y = Y_t - Y, \qquad y_\ell = -\sin\psi\,\Delta X + \cos\psi\,\Delta Y, \qquad \ell^2 = \max(\Delta X^2 + \Delta Y^2,\ 10^{-6})$$

**Pure-pursuit curvature and the yaw-rate reference:**

$$k_{path} = \frac{2\,y_\ell}{\ell^2}, \qquad r_{ref} = v_x\,k_{path}$$

**Steer command** (feedforward + feedback, then clipped):

$$\delta = \underbrace{\arctan(L\,k_{path})}_{\text{feedforward}} + \underbrace{k_{yaw}\,(r_{ref} - r)}_{\text{feedback}}, \qquad \delta \leftarrow \operatorname{clip}(\delta, -\delta_{\max}, \delta_{\max})$$

**Ackermann split** (`Vehicle.steer_split`), with $h_f = t_f/2$ and Ackermann factor $A$:

$$\delta_{L}^{Ack} = \arctan\frac{L\tan\delta}{L - h_f\tan\delta}, \qquad \delta_{R}^{Ack} = \arctan\frac{L\tan\delta}{L + h_f\tan\delta}$$

$$\delta_1 = \delta + A\,(\delta_L^{Ack} - \delta), \qquad \delta_2 = \delta + A\,(\delta_R^{Ack} - \delta)$$

- If $\delta = 0$, both angles are 0.
- If $|L - h_f\tan\delta| < 10^{-9}$, then $\delta_L^{Ack} = \pm\pi/2$.

### 11.3 Total longitudinal force request

Loads used by the controller are floored at 1 N: $\tilde F_{z,i} = \max(F_{z,i}, 1)$. Then

$$F_{req} = m\,\big(a_{ref} + k_v\,(v_{ref} - v_x)\big) + q\,C_dA\,v_x^2 + C_{rr}\sum_i \tilde F_{z,i}$$

### 11.4 Torque-vectoring yaw moment

$$M_{z,req} = \begin{cases} k_{tv}\,I_z\,(r_{ref} - r) & \text{if torque vectoring is on} \\ 0 & \text{otherwise} \end{cases}, \qquad \Delta F = \frac{M_{z,req}}{t_f + t_r}$$

The side sign is $(\varsigma_i) = (-1,\ +1,\ -1,\ +1)$ for FL, FR, RL, RR.

### 11.5 Allocation when driving ($F_{req} \ge 0$)

$$T_{mi} = d_i\,\left(F_{req}\,\frac{d_i \tilde F_{z,i}}{\sum_k d_k \tilde F_{z,k}} + \varsigma_i\,\Delta F\right)\frac{R}{G}, \qquad T_b = 0$$

This is a controller allocation rule: it converts the total requested longitudinal force and desired yaw moment into motor torque for each driven wheel. It is not a new tyre-force model; it decides how to distribute the force request.

The first term,

$$F_{req}\,\frac{d_i \tilde F_{z,i}}{\sum_k d_k \tilde F_{z,k}},$$

splits the total requested force between driven wheels in proportion to their normal loads. This is motivated by the approximate tyre limit

$$|F_{x,i}| \lesssim \mu_i F_{z,i}.$$

A wheel carrying twice as much load therefore receives roughly twice as much force, keeping the tyres at similar fractions of their available grip instead of overloading the lightly loaded wheel first. The factors $d_i$ ensure that only driven wheels participate: $d_i=1$ for a motorized wheel and $d_i=0$ otherwise.

The second term, $\varsigma_i\Delta F$, adds the left/right force difference needed for torque vectoring. With

$$\Delta F = \frac{M_{z,req}}{t_f+t_r},$$

and $(\varsigma_i)=(-1,+1,-1,+1)$ for FL, FR, RL, RR, the force additions cancel in the total longitudinal force,

$$\sum_i \varsigma_i\Delta F = 0,$$

but create a yaw moment because they act at a lateral distance from the vehicle centreline:

$$M_z = (t_f+t_r)\Delta F.$$

Thus, $\Delta F$ is the per-wheel force difference required to generate the requested yaw moment.

Finally, the quantity in parentheses is a longitudinal tyre-force request. Wheel torque is $F_{x,i}R$, and motor torque is related to wheel torque by $T_{\mathrm{wheel},i}=T_{mi}G$, where $G=G_r\eta_g$. Therefore, the conversion to motor-shaft torque is

$$T_{mi}=F_{x,i}\frac{R}{G}.$$

The plant subsequently applies the motor and tyre saturation limits.

### 11.6 Allocation when braking ($F_{req} < 0$)

**Desired force per wheel** (all four wheels can brake):

$$F_{x,i}^{des} = F_{req}\,\frac{\tilde F_{z,i}}{\sum_k \tilde F_{z,k}} + \varsigma_i\,\Delta F$$

**Regen force capacity**, from §3.2 evaluated at the current $\omega_i$:

$$F_{reg,i} = \begin{cases} -\,T_{\min,i}(\omega_i)\,G/R & d_i = 1 \text{ and regen on} \\ 0 & \text{otherwise} \end{cases}$$

**Friction brake covers the shortfall:**

$$T_b = \min\!\left(R\sum_i \big[-F_{x,i}^{des} - F_{reg,i}\big]^+,\ T_{b,\max}\right)$$

**Motors supply the remainder** after the bias-split friction brake:

$$T_{mi} = d_i\,\left(F_{x,i}^{des} + \frac{\beta_{b,i}\,T_b}{R}\right)\frac{R}{G}$$

### 11.7 Traction control and ABS (if enabled)

**Controller slip estimate** (ignores $v_y$ and the steer angle):

$$\hat\kappa_i = \frac{\omega_i R - (v_x - r\,y_i)}{\max(|v_x - r\,y_i|,\ v_{\min})}, \qquad \kappa_t = s_{tgt}\,\kappa_{pk}$$

**Reduction factor:**

$$\phi(e) = \left[1 - 4\,\frac{e}{\kappa_{pk}}\right]^+$$

**Applied per wheel:**

$$\text{if } \hat\kappa_i > \kappa_t \text{ and } T_{mi} > 0: \quad T_{mi} \leftarrow \phi(\hat\kappa_i - \kappa_t)\,T_{mi}$$

$$\text{else if } \hat\kappa_i < -\kappa_t: \quad f_i = \phi(-\hat\kappa_i - \kappa_t), \quad T_{mi} \leftarrow f_i\,T_{mi} \ \text{ if } T_{mi} < 0$$

**Friction brake** (one hydraulic input), scaled by the worst-slipping wheel:

$$T_b \leftarrow T_b \cdot \min_i f_i \qquad (f_i = 1 \text{ for wheels not locking})$$

The command $\mathbf u = [\delta_1, \delta_2, T_{m1..4}, T_b]$ then goes through the plant's saturation (§3).

---

## 12. Numerical integration and the lap loop

`Vehicle.stable_dt`, `Vehicle.rk4_step`, `run_transient`

### Initial state

$$\mathbf x_0 = \big[v_0,\ 0,\ 0,\ \tfrac{v_0}{R}\times 4,\ F^{ss}_{z,1..4}(v_0, 0, 0),\ 0,\ 0,\ 0,\ 0\big]$$

$$v_0 = \max\!\big(0.5,\ \tilde v_0\big), \qquad \tilde v_0 = \begin{cases} 0 & \text{standing start} \\ \sigma_v\, v_{QSS}(0) & \text{flying lap} \end{cases}$$

### Step size

Limited by the stiffest wheel mode and the load lag:

$$\bar v = \max(|v_x|,\ v_{\min}), \qquad \lambda_{\max} = \max\!\left(1,\ \max_i \frac{C_{\kappa,i}\,R^2}{I_{\text{eff},i}\,\bar v},\ \frac{1}{\tau_{F_z}}\right)$$

$$h = \min\!\left(\Delta t,\ \frac{2}{\lambda_{\max}}\right)$$

Here $C_{\kappa,i}$ is from §5, evaluated at $\hat F_{z,i}$.

### RK4 step (zero-order hold on $\mathbf u$)

$$\mathbf u_k = g(\mathbf x_k)$$

$$\mathbf k_1 = f(\mathbf x_k, \mathbf u_k), \quad \mathbf k_2 = f(\mathbf x_k + \tfrac h2 \mathbf k_1, \mathbf u_k), \quad \mathbf k_3 = f(\mathbf x_k + \tfrac h2 \mathbf k_2, \mathbf u_k), \quad \mathbf k_4 = f(\mathbf x_k + h\,\mathbf k_3, \mathbf u_k)$$

$$\mathbf x_{k+1} = \mathbf x_k + \frac h6\big(\mathbf k_1 + 2\mathbf k_2 + 2\mathbf k_3 + \mathbf k_4\big), \qquad t_{k+1} = t_k + h$$

$\kappa(s)$ is re-evaluated at each stage's own $s$.

### Finish line

If $s_{k+1} \ge L_{track}$:

$$\phi = \frac{L_{track} - s_k}{s_{k+1} - s_k}, \qquad \mathbf x_{fin} = \mathbf x_k + \phi\,(\mathbf x_{k+1} - \mathbf x_k), \qquad t_{lap} = t_k + \phi\,h$$

### Termination checks

After each step, the run aborts if any of these holds:
- any state is non-finite;
- $|\xi| > 1.3$ rad, or $|n| > 12$ m (left the track);
- $t > \max(3\,t_{QSS},\ 10\ \text{s})$;
- $t > 2$ s and $v_x < 0.3$ m/s (stopped).

---

## 13. Symbols, code names and default values

### Vehicle parameters (`VehicleParams`)

| Symbol | Code | Default | Unit |
|---|---|---|---|
| $m$ | `m` | 280 | kg |
| $I_z$ | `Iz` | 110 | kg m² |
| $L$ | `wheelbase` | 1.53 | m |
| $w_f$ | `wd_front` | 0.47 | – |
| $t_f,\ t_r$ | `track_f`, `track_r` | 1.22, 1.18 | m |
| $h$ | `h_cg` | 0.28 | m |
| $\Lambda_f$ | `lltd_front` | 0.50 | – |
| $\tau_{F_z}$ | `tau_fz` | 0.03 | s |
| $\rho_{air}$ | `rho` | 1.20 | kg/m³ |
| $C_lA,\ C_dA$ | `ClA`, `CdA` | 3.2, 1.3 | m² |
| $\varepsilon_f$ | `aero_bal_front` | 0.45 | – |
| $h_{cp}$ | `h_cp` | 0.35 | m |
| $R$ | `R_w` | 0.203 | m |
| $I_w$ | `I_w` | 0.12 | kg m² |
| $\mu_x,\ \mu_y$ | `mu_x`, `mu_y` | 1.55, 1.50 | – |
| $F_{z0}$ | `Fz0` | 700 | N |
| $k_{ls}$ | `load_sens` | −0.10 | – |
| $\kappa_{pk}$ | `kappa_peak` | 0.10 | – |
| $\alpha_{pk}$ | `alpha_peak_deg` | 8° | rad in equations |
| $C_x,\ C_y$ | `C_x`, `C_y` | 1.65, 1.45 | – |
| $C_{rr}$ | `Crr` | 0.015 | – |
| $v_{\min}$ | `v_slip_min` | 4.0 | m/s |
| $d_i$ | `awd` | AWD (True) | – |
| $G_r$ | `gear_ratio` | 13.5 | – |
| $\eta_g,\ \eta_m$ | `eta_gear`, `eta_motor` | 0.95, 0.92 | – |
| $T_{\text{peak}}$ | `T_motor_max` | 21 | N·m |
| $P_{\text{mot}}$ | `P_motor_max_kW` | 35 | kW |
| $N_{\max}$ | `rpm_max` | 20 000 | rpm |
| $I_{\text{mot}}$ | `I_motor` | 2.74×10⁻⁴ | kg m² |
| $P_{\text{tot}}$ | `P_total_max_kW` | 80 | kW |
| regen on, $f_{\text{regen}}$ | `regen`, `regen_frac` | True, 1.0 | – |
| $T_{b,\max}$ | `T_brake_max` | 2000 | N·m |
| $b_f$ | `brake_bias_front` | 0.65 | – |
| $\delta_{\max}$ | `delta_max_deg` | 25° | rad in equations |
| $A$ | `ackermann` | 0.5 | – |

### Driver settings (`DriverSettings`)

| Symbol | Code | Default | Unit |
|---|---|---|---|
| $t_p$ | `preview_time` | 0.45 | s |
| $L_{p,\min}$ | `preview_min` | 3.0 | m |
| $k_{yaw}$ | `k_yaw` | 0.08 | rad per rad/s |
| $\sigma_v$ | `speed_scale` | 1.0 | – |
| $t_{look}$ | `speed_lookahead` | 0.12 | s |
| $k_v$ | `k_speed` | 4.0 | 1/s |
| TV on, $k_{tv}$ | `torque_vectoring`, `k_tv` | True, 8.0 | –, 1/s |
| TC on, $s_{tgt}$ | `traction_control`, `slip_target` | True, 0.8 | – |
| $\Delta t$ | `dt` | 0.001 | s |
| – | `log_dt` | 0.01 | s |

---

## 14. Simplifications as implemented

These are faithful to the code. Each one is a deliberate modelling choice, or a known simplification, that you should be aware of when using the results.

| # | Simplification | Where | Effect |
|---|---|---|---|
| 1 | Tyre forces respond instantly to slip (no relaxation length) | §4–5 | Stiff wheel modes; low-speed behaviour approximate |
| 2 | No roll/pitch/heave states; loads follow a first-order lag to the quasi-static target | §7–8 | No camber, ride-height or damper effects; LLTD is an input |
| 3 | Drag acts along the body x-axis using $v_x$ only; pitch moment from drag uses $v_x^2$ | §6–7 | Negligible at small sideslip |
| 4 | Gear efficiency $\eta_g$ multiplies wheel torque in **both** directions | §8 wheel equation | Regen torque at the wheel is slightly *under*-estimated: physically a regenerating gearbox would need $T_{wheel} = G_r T_m/\eta_g$ |
| 5 | Regen limit does not fade near max rpm (only $T_{\max}$ fades) | §3.2 | Only matters within 2% of max rpm |
| 6 | Brake and rolling-resistance direction use $\operatorname{sat}(\omega)$, linear within ±1 rad/s | §8 | Avoids chatter at standstill, but the friction-brake torque fades to zero as a wheel stops, so a wheel can never stay fully locked |
| 7 | The torque-vectoring split assumes all four wheels share $\Delta F$ | §11.4 | On RWD, only about half of $M_{z,req}$ is delivered |
| 8 | Brake allocation sums per-wheel shortfalls, then applies a fixed bias | §11.6 | Under heavy braking the front motors may hit their regen limit, losing a few % of requested braking |
| 9 | Traction-control slip estimate ignores $v_y$ and steer angle, and uses true states | §11.7 | Idealised sensing |
| 10 | Standing start begins at 0.5 m/s | §12 | ≈0.04 s optimistic |
| 11 | Planar model: no elevation, banking or camber | all | e.g. Eau Rouge compression not represented |
