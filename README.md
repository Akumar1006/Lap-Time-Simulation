# FSAE 4-Corner State-Space Vehicle Model & Lap Time Simulator

Interactive (Tkinter + matplotlib) tool containing:

* a nonlinear **4-corner planar vehicle model** in state-space form `ẋ = f(x, u)`,
* trim + numerical **linearisation** (`A, B, C, D`, eigenvalues, controllability), exportable to MATLAB,
* a **track builder** that uses corner radii and segment lengths,
* a **quasi-steady-state (GGV) lap sim** and a **transient lap sim** that runs the full state-space model with a closed-loop driver and torque vectoring.

## Run

```powershell
python -m pip install -e ".[test,mat]"
python run_gui.py          # GUI
python -m pytest tests     # unit tests
```

The install command installs the required numpy, scipy and matplotlib packages,
plus pytest for tests and h5py for importing MATLAB v7.3 `.mat` files.
Tkinter is provided by the Python installation and is not installed through pip.

## State-space model

| States `x` (15) | | Inputs `u` (7) | |
|---|---|---|---|
| `vx, vy` | body velocities [m/s] | `delta_FL, delta_FR` | front steer angles [rad] |
| `r` | yaw rate [rad/s] | `Tm_FL … Tm_RR` | motor shaft torques [Nm] |
| `w_FL … w_RR` | wheel speeds [rad/s] | `T_brake` | total friction brake torque (split by bias) [Nm] |
| `Fz_FL … Fz_RR` | normal loads [N] | | |
| `s, n, xi` | distance, lateral offset, heading error relative to the track | | |
| `E` | battery energy used [J] | | |

Outputs `y`: `ax, ay, beta, kappa_i, alpha_i`.

Model features:

* **Tyres:** combined-slip Magic Formula with load-sensitive friction. You set the peak slip ratio and peak slip angle directly.
* **Normal loads:** a first-order lag (`tau_fz`) towards the quasi-static loads. These include aero downforce and balance, drag pitch, longitudinal transfer, and lateral transfer with LLTD. Making the loads states avoids the algebraic loop between load and acceleration.
* **Motors:** one per wheel. Each motor has a torque, power and rpm limit, and the accumulator power limit (80 kW) also applies. Regen braking is included. You can switch between AWD and RWD.
* **Track coordinates:** the curvilinear states `s, n, xi` let the same model run on any track. On a straight (zero curvature) they are simply `X, Y, psi`.

The full equations are listed in the GUI under **State-space model → Model equations**.

## Track definition

Each segment has a `length` (arc length, m) and a `radius` (m). Use `+` for a left turn, `−` for a right turn and `0` for a straight. You can enter a corner angle instead of a length. To import or export a track, use a CSV with `length_m,radius_m` columns. The simulation only uses curvature vs. distance, so a track still simulates if its geometry does not close perfectly.

The default track is **Spa-Francorchamps (F1 layout)**, stored in [fsae_lapsim/data/spa_francorchamps.csv](fsae_lapsim/data/spa_francorchamps.csv) as 172 constant-radius segments (7.0 km, clockwise).

* **Source:** the [TUMFTM racetrack-database](https://github.com/TUMFTM/racetrack-database) centre line (LGPL-3.0).
* **Conversion:** a piecewise-linear fit of heading vs. distance (1° tolerance), followed by a small least-squares correction so the lap closes.
* **Accuracy:** at most 4.8 m (mean 2.7 m) from the real centre line, which is well inside the ~13 m track width. The lap closes to within 5 mm.
* **Not modelled:** elevation (e.g. Eau Rouge), because the model is planar.

**FSAE Autocross 2024 (course map)** is the red line from the 2024 skidpad/autocross course drawing ([fsae_lapsim/data/autocross_2024_map.png](fsae_lapsim/data/autocross_2024_map.png)), stored in [fsae_lapsim/data/autocross_2024.csv](fsae_lapsim/data/autocross_2024.csv).

* **Shape:** 878 m, 201 segments. It is an open course: it starts at the green flag and finishes at the checkered flag, so the lap uses a standing start.
* **Scale:** taken from the drawing's 25 ft grid (≈0.667 m/px).
* **Accuracy:** the fitted segments stay within a mean of 0.36 m (max 1.5 m) of the drawn line.
* **Minimum radius:** set to 4.5 m (the FSAE minimum for a hairpin). This replaces the sharp kinks in the hand-drawn line; each corner keeps its turning angle.

### Adding a track from a course-map image

[tools/image_to_track.py](tools/image_to_track.py) traces a coloured line in an image and writes a segment CSV you can load with **Import CSV…**:

```powershell
python tools/image_to_track.py map.png my_track.csv --start 328 45 --grid-ft 25 --preview check.png
```

* **`--start X Y`:** a pixel on the line where the course starts. It walks the line from there until the line ends.
* **Scale:** use one of `--grid-ft`, `--grid-m` (auto-detects the grid pixel spacing) or `--m-per-px`.
* **Line colour:** `--color red|green|blue` selects which colour to follow.
* **Fit settings:** `--smooth`, `--tol` and `--min-radius` control the fit.
* **Checking the result:** always look at the `--preview` overlay to confirm the trace followed the right line.

The FSAE events (skidpad, acceleration) and the sample tracks are still available as presets.


## Lap simulation

1. **QSS:** builds a GGV from the 4-corner model, then computes apex speeds and runs forward and backward passes. Closed tracks are solved as a periodic flying lap.
2. **Transient:** integrates the nonlinear model with RK4. The step size is limited automatically so the stiff wheel-slip modes stay stable. The driver uses:
   * pure-pursuit steering with yaw-rate feedback and an Ackermann split,
   * tracking of the QSS speed profile,
   * load-proportional torque allocation plus a torque-vectoring yaw moment,
   * regen first, then friction brakes,
   * slip-ratio traction control.

**Exporting results:** **Export .mat…** writes one MATLAB file with these structs:

| Struct | Contents |
|---|---|
| `qss` | QSS channels (`s, t, v, v_kph, ax, ay, ax_g, ay_g, curvature, Fz_FL…RR, P_el, E, X, Y`) |
| `transient` | All 15 states, the 7 inputs as applied, the outputs (slips, tyre forces, power per motor) and derived channels (`speed_kph, ax_g, ay_g, beta_deg, delta_*_deg, v_ref, X, Y, psi`) |
| `vehicle` | The vehicle parameters used |
| `driver` | The driver settings used |
| `track` | `length_m`, `radius_m`, `name`, `closed` |
| `meta` | Lap times, transient status, creation date |

Each channel struct has a `units` sub-struct. **Export CSV…** is still available.

## Parameter sweep

The **Parameter sweep** tab runs the QSS lap simulation over a grid. It uses the track currently loaded in the Track tab; every other parameter comes from the Vehicle tab.

* **1-D (Parameter 2 = "none"):** lap time (or energy) vs one parameter. Dashed horizontal lines mark the chosen lap times, and the parameter value that gives each one is labelled.
* **2-D:** a contour map over two parameters with **lines of constant lap time**. Every point on, say, the 60 s line gives the same lap time.
  * The red star marks the current Vehicle-tab car.
  * The summary gives the **trade-off along the constant-lap-time line** at that point, e.g. "+1 kg of mass ⇔ +2.2 kW of power".
* **Which lines are drawn:** set by the *Constant-value lines* box. Use `auto`, a list such as `45, 47, 49`, or `start:step:stop` such as `45:0.5:50`.
  * Changing the levels, the metric (lap time / energy) or the colour fill redraws instantly, without re-running.
* **Speed:** grid points run in parallel processes. A 9×9 grid takes about 6 s on the autocross and about 20 s on Spa.
* **Invalid points:** combinations that break a parameter's allowed range (e.g. a weight distribution of 0.99) show as gaps.
* **Export:** results save to `.mat` or `.csv`.

## Channel plot and reference data

The **Channel plot** tab plots any channel against any other, from:

* **`Sim QSS` / `Sim Transient`:** the latest lap simulation, updated automatically after every run. Use **Snapshot copy** to keep a run before changing the car.
* **Imported `.mat` files:** e.g. logged data from the real car. **Load .mat…** accepts several files at once.

**How to plot:**

1. Pick a dataset and an X channel.
2. Select one or more Y channels. The filter box helps with large logs.
3. Click **Add** (or double-click a channel).

Options for each trace:

* **Subplot number:** traces with the same number share an axis, so you can stack speed, lateral g and steering vertically.
* **X offset:** to line up a logged lap distance with the simulation.
* **Y scale / Y offset:** for unit changes, e.g. ×3.6 for m/s → km/h, or road-wheel → steering-wheel angle.
* **Markers only:** draws points instead of lines, for g-g diagrams.

**Supported `.mat` layouts** (MATLAB v5/v7 via SciPy, and v7.3/HDF5 via h5py):

| Layout | Example |
|---|---|
| Flat variables plus a time vector | `Time`, `Speed`, `Throttle` |
| A struct of channels | `data.t`, `data.speed`, … |
| One struct per channel, logger-export style, sample rates may differ | `Ground_Speed.Time`, `Ground_Speed.Value`, `Ground_Speed.Units` |
| Matrices | Split into columns `name_1`, `name_2`, … |

**Channels with different sample rates** are aligned by time, using each channel's time base, before plotting one against another.

**MATLAB tables and timetables** can't be read outside MATLAB. Convert them first with `S = table2struct(T, 'ToScalar', true); save('file.mat', '-struct', 'S')`.

For a detailed walkthrough of the transient model, see **[TRANSIENT_DYNAMICS.md](./TRANSIENT_DYNAMICS.md)**. It covers:

* the derivation of every equation,
* why each modelling choice was made,
* stiffness and integrator stability,
* the driver/controller design,
* how to read the linearisation,
* worked numbers from the default car,
* tuning and troubleshooting.

For a line-by-line walkthrough of the transient driver and simulation loop code (`fsae_lapsim/transient.py`), see **[TRANSIENT_CODE_WALKTHROUGH.md](./TRANSIENT_CODE_WALKTHROUGH.md)**.

For a conceptual explanation of the driver's **feedforward and feedback** (what each does, why both are needed, with experiments that remove one or the other), see **[FEEDFORWARD_FEEDBACK.md](./FEEDFORWARD_FEEDBACK.md)**.

For every equation of the transient model written out explicitly, exactly as implemented (including limits and special cases), see **[TRANSIENT_EQUATIONS.md](./TRANSIENT_EQUATIONS.md)**.

For how the model works as a **nonlinear state-space model** (building on linear $\dot x = Ax + Bu$: what changes, how $f$ is implemented, and how simulation, trim and linearisation use it), see **[NONLINEAR_STATE_SPACE.md](./NONLINEAR_STATE_SPACE.md)**.
