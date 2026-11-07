"""Tkinter GUI for the FSAE 4-corner state-space model and lap-time simulation."""
from __future__ import annotations

import csv
import math
import threading
import traceback
from dataclasses import fields
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib

matplotlib.use("TkAgg")
matplotlib.rcParams["legend.loc"] = "upper right"  # "best" is very slow on long lap traces
import numpy as np  # noqa: E402
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

from .params import PARAM_META, VehicleParams  # noqa: E402
from .channels import from_qss, from_transient, load_mat, save_lap_mat  # noqa: E402
from .qss import GGV, run_qss  # noqa: E402
from .sweep import crossings, parse_levels, run_sweep, tradeoff  # noqa: E402
from .track import PRESETS, Segment, Track  # noqa: E402
from .transient import Y_COLS, DriverSettings, run_transient  # noqa: E402
from .vehicle import (CORNERS, INPUT_NAMES, INPUT_UNITS, OUTPUT_NAMES, STATE_NAMES,  # noqa: E402
                      STATE_UNITS, Vehicle)

MONO = ("Consolas", 9)
CORNER_COLORS = ("tab:blue", "tab:orange", "tab:green", "tab:red")


# ====================================================================== helpers
class PlotPanel(ttk.Frame):
    def __init__(self, master, figsize=(9, 6)):
        super().__init__(master)
        self.fig = Figure(figsize=figsize, dpi=90, layout="constrained")
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        tb = NavigationToolbar2Tk(self.canvas, self, pack_toolbar=False)
        tb.update()
        tb.pack(side=tk.BOTTOM, fill=tk.X)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def axes(self, nr, nc, **kw):
        self.fig.clf()
        return self.fig.subplots(nr, nc, **kw)

    def draw(self):
        self.canvas.draw_idle()


class ScrollFrame(ttk.Frame):
    def __init__(self, master):
        super().__init__(master)
        cv = tk.Canvas(self, highlightthickness=0)
        sb = ttk.Scrollbar(self, orient=tk.VERTICAL, command=cv.yview)
        self.inner = ttk.Frame(cv)
        self.inner.bind("<Configure>", lambda e: cv.configure(scrollregion=cv.bbox("all")))
        cv.create_window((0, 0), window=self.inner, anchor="nw")
        cv.configure(yscrollcommand=sb.set)
        cv.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        cv.bind("<Enter>", lambda e: cv.bind_all("<MouseWheel>", lambda ev: cv.yview_scroll(int(-ev.delta / 120), "units")))
        cv.bind("<Leave>", lambda e: cv.unbind_all("<MouseWheel>"))


def text_widget(master, height=10, width=60):
    fr = ttk.Frame(master)
    txt = tk.Text(fr, height=height, width=width, wrap=tk.NONE, font=MONO)
    ys = ttk.Scrollbar(fr, orient=tk.VERTICAL, command=txt.yview)
    xs = ttk.Scrollbar(fr, orient=tk.HORIZONTAL, command=txt.xview)
    txt.configure(yscrollcommand=ys.set, xscrollcommand=xs.set)
    txt.grid(row=0, column=0, sticky="nsew")
    ys.grid(row=0, column=1, sticky="ns")
    xs.grid(row=1, column=0, sticky="ew")
    fr.rowconfigure(0, weight=1)
    fr.columnconfigure(0, weight=1)
    return fr, txt


def set_text(txt: tk.Text, s: str):
    txt.configure(state=tk.NORMAL)
    txt.delete("1.0", tk.END)
    txt.insert("1.0", s)


def fmt_matrix(M, rows, cols, w=11):
    out = [" " * 10 + "".join(f"{c:>{w}s}" for c in cols)]
    for name, row in zip(rows, M):
        out.append(f"{name:>10s}" + "".join(f"{v:{w}.4g}" for v in row))
    return "\n".join(out)


# ====================================================================== vehicle tab
class VehicleTab(ttk.Frame):
    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        self.vars: dict[str, tk.Variable] = {}
        top = ttk.Frame(self)
        top.pack(fill=tk.X, padx=6, pady=4)
        ttk.Button(top, text="Apply / update derived", command=self.show_derived).pack(side=tk.LEFT)
        ttk.Button(top, text="Reset defaults", command=self.reset).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Load JSON…", command=self.load).pack(side=tk.LEFT)
        ttk.Button(top, text="Save JSON…", command=self.save).pack(side=tk.LEFT, padx=4)
        ttk.Label(top, text="All parameters are read automatically whenever a simulation is run.",
                  foreground="gray").pack(side=tk.LEFT, padx=10)
        body = ttk.Panedwindow(self, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True)
        sf = ScrollFrame(body)
        body.add(sf, weight=3)
        groups: dict[str, ttk.LabelFrame] = {}
        layout = {"Mass & Geometry": (0, 0), "Aero": (1, 0), "Tyres": (0, 1),
                  "Powertrain": (0, 2), "Brakes & Steering": (1, 2)}
        defaults = VehicleParams()
        for name, label, unit, group in PARAM_META:
            if group not in groups:
                r, c = layout[group]
                lf = ttk.LabelFrame(sf.inner, text=group, padding=4)
                lf.grid(row=r, column=c, sticky="nsew", padx=4, pady=4,
                        rowspan=2 if group in ("Tyres",) else 1)
                groups[group] = lf
            lf = groups[group]
            row = lf.grid_size()[1]
            val = getattr(defaults, name)
            if unit == "bool":
                v = tk.BooleanVar(value=val)
                ttk.Checkbutton(lf, text=label, variable=v).grid(row=row, column=0, columnspan=3, sticky="w")
            else:
                v = tk.StringVar(value=f"{val:g}")
                ttk.Label(lf, text=label).grid(row=row, column=0, sticky="w")
                ttk.Entry(lf, textvariable=v, width=10).grid(row=row, column=1, padx=2)
                ttk.Label(lf, text=unit, foreground="gray").grid(row=row, column=2, sticky="w")
            self.vars[name] = v
        dfr, self.info = text_widget(body, height=30, width=48)
        body.add(dfr, weight=2)
        self.show_derived()

    def get_params(self) -> VehicleParams:
        kw = {}
        for name, label, unit, _ in PARAM_META:
            v = self.vars[name]
            if unit == "bool":
                kw[name] = bool(v.get())
            else:
                try:
                    kw[name] = float(v.get())
                except ValueError:
                    raise ValueError(f"Invalid number for '{label}': {v.get()!r}")
        return VehicleParams(**kw)

    def set_params(self, p: VehicleParams):
        for name, _, unit, _ in PARAM_META:
            val = getattr(p, name)
            self.vars[name].set(val if unit == "bool" else f"{val:g}")
        self.show_derived()

    def reset(self):
        self.set_params(VehicleParams())

    def load(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if path:
            self.set_params(VehicleParams.from_json(path))

    def save(self):
        path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")])
        if path:
            try:
                self.get_params().to_json(path)
            except ValueError as e:
                messagebox.showerror("Parameters", str(e))

    def show_derived(self):
        try:
            veh = Vehicle(self.get_params())
        except ValueError as e:
            set_text(self.info, f"ERROR: {e}")
            return
        p = veh.p
        G = p.gear_ratio
        ck = [veh.tire_peak(f)[0] * p.C_x * veh.Bxn / p.kappa_peak for f in veh.Fz_static]
        ca = [veh.tire_peak(f)[1] * p.C_y * veh.Byn / veh.alpha_peak for f in veh.Fz_static]
        Caf, Car = ca[0] + ca[1], ca[2] + ca[3]
        Kus = p.m / p.wheelbase * (veh.b / Caf - veh.a / Car)
        lines = [
            "DERIVED QUANTITIES",
            f"CG to front axle a      : {veh.a:8.3f} m",
            f"CG to rear axle b       : {veh.b:8.3f} m",
            f"Static load front/wheel : {veh.Fz_static[0]:8.1f} N",
            f"Static load rear/wheel  : {veh.Fz_static[2]:8.1f} N",
            f"Downforce @ 20 m/s      : {veh.q * p.ClA * 400:8.1f} N",
            f"Drag @ 20 m/s           : {veh.q * p.CdA * 400:8.1f} N",
            f"Eff. wheel inertia F/R  : {veh.Iw_eff[0]:8.4f} / {veh.Iw_eff[2]:.4f} kg m²",
            f"Top speed (rpm limit)   : {veh.v_top:8.2f} m/s ({veh.v_top * 3.6:.1f} km/h)",
            f"Max wheel torque/motor  : {p.T_motor_max * G * p.eta_gear:8.1f} Nm",
            f"Max tractive F/motor    : {p.T_motor_max * G * p.eta_gear / p.R_w:8.1f} N",
            f"Driven motors           : {sum(veh.driven)}",
            f"MF B_x (peak @kappa_pk) : {veh.Bxn / p.kappa_peak:8.2f}",
            f"MF B_y (peak @alpha_pk) : {veh.Byn / veh.alpha_peak:8.2f} 1/rad",
            f"C_kappa static F/R      : {ck[0]:8.0f} / {ck[2]:.0f} N",
            f"C_alpha axle F/R        : {Caf:8.0f} / {Car:.0f} N/rad",
            f"Understeer gradient     : {math.degrees(Kus) * 9.81:8.3f} deg/g",
            "",
            "STATE VECTOR x (15)",
            *[f"  x{i:<2d} {n:<8s} [{u}]" for i, (n, u) in enumerate(zip(STATE_NAMES, STATE_UNITS))],
            "",
            "INPUT VECTOR u (7)",
            *[f"  u{i:<2d} {n:<9s} [{u}]" for i, (n, u) in enumerate(zip(INPUT_NAMES, INPUT_UNITS))],
        ]
        set_text(self.info, "\n".join(lines))


# ====================================================================== track tab
class TrackTab(ttk.Frame):
    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        self.track = PRESETS["Spa-Francorchamps (F1)"]()
        left = ttk.Frame(self, padding=6)
        left.pack(side=tk.LEFT, fill=tk.Y)
        pr = ttk.Frame(left)
        pr.pack(fill=tk.X)
        self.preset = tk.StringVar(value="Spa-Francorchamps (F1)")
        ttk.Combobox(pr, textvariable=self.preset, values=list(PRESETS), state="readonly", width=30).pack(side=tk.LEFT)
        ttk.Button(pr, text="Load preset", command=self.load_preset).pack(side=tk.LEFT, padx=4)
        cols = ("idx", "length", "radius", "angle", "type")
        self.tree = ttk.Treeview(left, columns=cols, show="headings", height=18, selectmode="browse")
        for c, t, w in zip(cols, ("#", "Length [m]", "Radius [m]", "Angle [°]", "Type"), (35, 85, 85, 75, 55)):
            self.tree.heading(c, text=t)
            self.tree.column(c, width=w, anchor="e")
        self.tree.pack(fill=tk.BOTH, expand=True, pady=4)
        self.tree.bind("<<TreeviewSelect>>", self.on_select)
        ed = ttk.LabelFrame(left, text="Segment (radius: + left, − right, 0 = straight)", padding=4)
        ed.pack(fill=tk.X)
        self.e_len, self.e_rad, self.e_ang = tk.StringVar(value="20"), tk.StringVar(value="0"), tk.StringVar()
        for i, (lbl, var) in enumerate((("Length [m]", self.e_len), ("Radius [m]", self.e_rad),
                                        ("or Angle [°] (overrides length)", self.e_ang))):
            ttk.Label(ed, text=lbl).grid(row=i, column=0, sticky="w")
            ttk.Entry(ed, textvariable=var, width=10).grid(row=i, column=1, sticky="w", padx=4)
        bf = ttk.Frame(ed)
        bf.grid(row=3, column=0, columnspan=2, pady=4)
        for i, (t, cmd) in enumerate((("Append", self.add), ("Insert before", self.insert), ("Update", self.update_seg),
                                      ("Delete", self.delete), ("Move up", lambda: self.move(-1)),
                                      ("Move down", lambda: self.move(1)), ("Clear all", self.clear))):
            ttk.Button(bf, text=t, command=cmd, width=12).grid(row=i // 4, column=i % 4, padx=1, pady=1)
        of = ttk.Frame(left)
        of.pack(fill=tk.X, pady=4)
        self.closed = tk.BooleanVar(value=True)
        ttk.Checkbutton(of, text="Closed circuit (flying lap)", variable=self.closed, command=self.refresh).pack(side=tk.LEFT)
        ttk.Button(of, text="Import CSV…", command=self.import_csv).pack(side=tk.LEFT, padx=4)
        ttk.Button(of, text="Export CSV…", command=self.export_csv).pack(side=tk.LEFT)
        self.info = ttk.Label(left, text="", justify=tk.LEFT)
        self.info.pack(fill=tk.X)
        self.plot = PlotPanel(self, (7, 6))
        self.plot.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.refresh()

    # -------------------------------------------------------- editing
    def _seg_from_fields(self) -> Segment:
        R = float(self.e_rad.get() or 0)
        ang = self.e_ang.get().strip()
        if ang and R != 0:
            L = abs(R) * math.radians(abs(float(ang)))
        else:
            L = float(self.e_len.get())
        if L <= 0:
            raise ValueError("Length must be > 0")
        return Segment(L, R)

    def _sel(self):
        s = self.tree.selection()
        return int(s[0]) if s else None

    def _edit(self, fn):
        try:
            fn()
        except ValueError as e:
            messagebox.showerror("Segment", str(e))
        self.refresh()

    def add(self):
        self._edit(lambda: self.track.segments.append(self._seg_from_fields()))

    def insert(self):
        i = self._sel()
        self._edit(lambda: self.track.segments.insert(i if i is not None else 0, self._seg_from_fields()))

    def update_seg(self):
        i = self._sel()
        if i is not None:
            self._edit(lambda: self.track.segments.__setitem__(i, self._seg_from_fields()))
            self.tree.selection_set(str(i))

    def delete(self):
        i = self._sel()
        if i is not None:
            del self.track.segments[i]
            self.refresh()

    def move(self, d):
        i = self._sel()
        segs = self.track.segments
        if i is not None and 0 <= i + d < len(segs):
            segs[i], segs[i + d] = segs[i + d], segs[i]
            self.refresh()
            self.tree.selection_set(str(i + d))

    def clear(self):
        if messagebox.askyesno("Track", "Remove all segments?"):
            self.track.segments.clear()
            self.refresh()

    def load_preset(self):
        self.track = PRESETS[self.preset.get()]()
        self.closed.set(self.track.closed)
        self.refresh()

    def import_csv(self):
        path = filedialog.askopenfilename(filetypes=[("CSV", "*.csv"), ("All", "*.*")])
        if path:
            self.track = Track.from_csv(path, closed=self.closed.get())
            self.refresh()

    def export_csv(self):
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")])
        if path:
            self.track.to_csv(path)

    def on_select(self, _e=None):
        i = self._sel()
        if i is None:
            return
        sg = self.track.segments[i]
        self.e_len.set(f"{sg.length:g}")
        self.e_rad.set(f"{sg.radius:g}")
        self.e_ang.set("")
        self.draw(highlight=i)

    def get_track(self) -> Track:
        segs = [Segment(s.length, s.radius) for s in self.track.segments]
        if not segs:
            raise ValueError("Track has no segments")
        return Track(segs, self.closed.get(), self.track.name)

    # -------------------------------------------------------- display
    def refresh(self):
        self.track.closed = self.closed.get()
        self.track.rebuild()
        self.tree.delete(*self.tree.get_children())
        for i, sg in enumerate(self.track.segments):
            typ = "Straight" if sg.radius == 0 else ("Left" if sg.radius > 0 else "Right")
            self.tree.insert("", tk.END, iid=str(i), values=(i, f"{sg.length:.2f}", f"{sg.radius:g}",
                                                             f"{sg.angle_deg:.1f}", typ))
        gap, dth = self.track.closure_error() if self.track.segments else (0, 0)
        txt = f"Total length: {self.track.length:.1f} m   Segments: {len(self.track.segments)}"
        if self.closed.get():
            txt += f"\nClosure gap: {gap:.2f} m, heading error: {dth:.1f}°"
            if gap > 1 or abs(dth) > 2:
                txt += "\n(geometry does not close – simulation still works,\n it only uses curvature vs distance)"
        self.info.configure(text=txt)
        self.draw()

    def draw(self, highlight=None):
        ax = self.plot.axes(1, 1)
        tr = self.track
        if tr.length > 0:
            starts = tr._starts + [tr.length]
            segs = [sg for sg in tr.segments if sg.length > 0]
            label_every = max(1, math.ceil(len(segs) / 40))
            for j, sg in enumerate(segs):
                m = (tr.s >= starts[j] - 1e-9) & (tr.s <= starts[j + 1] + 1e-9)
                col = "0.4" if sg.radius == 0 else ("tab:blue" if sg.radius > 0 else "tab:orange")
                lw = 2.0
                if highlight is not None and j == highlight:
                    col, lw = "red", 4.5
                ax.plot(tr.X[m], tr.Y[m], color=col, lw=lw)
                k = np.nonzero(m)[0]
                if len(k) and (j % label_every == 0 or j == highlight):
                    mid = k[len(k) // 2]
                    ax.annotate(str(j), (tr.X[mid], tr.Y[mid]), fontsize=7,
                                color="red" if j == highlight else "k",
                                xytext=(4, 4), textcoords="offset points")
            if tr.closed:
                ax.plot(0, 0, "g>", ms=11, label="start / finish")
            else:
                ax.plot(0, 0, "g>", ms=11, label="start")
                ax.plot(tr.end_pose[0], tr.end_pose[1], "ks", ms=9, label="finish")
            ax.legend(loc="upper right", fontsize=8)
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.3)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_title(f"{tr.name}   (blue = left, orange = right, grey = straight)", fontsize=9)
        self.plot.draw()


# ====================================================================== state-space tab
class ModelTab(ttk.Frame):
    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        self.sim = None
        self.lin = None
        self._guard = False
        left = ttk.Frame(self, padding=4)
        left.pack(side=tk.LEFT, fill=tk.Y)
        inp = ttk.LabelFrame(left, text="Inputs u (applied as a step at t_step)", padding=4)
        inp.pack(fill=tk.X)
        self.sv = {}
        specs = [("delta_FL", "δ FL [deg]", -25, 25, 0.1, 3.0), ("delta_FR", "δ FR [deg]", -25, 25, 0.1, 2.8),
                 ("Tm_FL", "Tm FL [Nm]", -21, 21, 0.1, 3.0), ("Tm_FR", "Tm FR [Nm]", -21, 21, 0.1, 3.0),
                 ("Tm_RL", "Tm RL [Nm]", -21, 21, 0.1, 3.0), ("Tm_RR", "Tm RR [Nm]", -21, 21, 0.1, 3.0),
                 ("T_brake", "Friction brake total [Nm]", 0, 2000, 10, 0)]
        for key, lbl, lo, hi, res, val in specs:
            var = tk.DoubleVar(value=val)
            sc = tk.Scale(inp, label=lbl, from_=lo, to=hi, resolution=res, orient=tk.HORIZONTAL, length=260,
                          variable=var, command=lambda _v, k=key: self.on_slider(k))
            sc.pack(fill=tk.X)
            sc.bind("<ButtonRelease-1>", lambda e: self.live())
            self.sv[key] = (var, sc)
        self.link_steer = tk.BooleanVar(value=True)
        self.link_torque = tk.BooleanVar(value=True)
        ttk.Checkbutton(inp, text="Ackermann link (δ FR follows δ FL)", variable=self.link_steer).pack(anchor="w")
        ttk.Checkbutton(inp, text="Same torque on all motors", variable=self.link_torque).pack(anchor="w")
        sim = ttk.LabelFrame(left, text="Simulation", padding=4)
        sim.pack(fill=tk.X, pady=4)
        self.v0, self.tend, self.tstep = tk.DoubleVar(value=12.0), tk.DoubleVar(value=4.0), tk.DoubleVar(value=0.5)
        for lbl, var, lo, hi, res in (("Initial speed [m/s]", self.v0, 0.5, 30, 0.5),
                                      ("Duration [s]", self.tend, 0.5, 15, 0.5),
                                      ("t_step [s]", self.tstep, 0, 5, 0.1)):
            sc = tk.Scale(sim, label=lbl, from_=lo, to=hi, resolution=res, orient=tk.HORIZONTAL, length=260, variable=var)
            sc.pack(fill=tk.X)
            sc.bind("<ButtonRelease-1>", lambda e: self.live())
        self.live_on = tk.BooleanVar(value=True)
        ttk.Checkbutton(sim, text="Live update on slider release", variable=self.live_on).pack(anchor="w")
        ttk.Button(sim, text="Run open-loop simulation", command=self.run).pack(fill=tk.X, pady=2)
        lin = ttk.LabelFrame(left, text="Linearisation  ẋ = A x + B u,  y = C x + D u", padding=4)
        lin.pack(fill=tk.X)
        self.lv, self.lr = tk.StringVar(value="15"), tk.StringVar(value="20")
        r = ttk.Frame(lin)
        r.pack(fill=tk.X)
        ttk.Label(r, text="Trim speed [m/s]").pack(side=tk.LEFT)
        ttk.Entry(r, textvariable=self.lv, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Label(r, text="Radius [m] (0 = straight)").pack(side=tk.LEFT)
        ttk.Entry(r, textvariable=self.lr, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Button(lin, text="Trim (steady state) & linearise", command=self.trim_lin).pack(fill=tk.X, pady=1)
        ttk.Button(lin, text="Linearise at final state of open-loop run", command=self.final_lin).pack(fill=tk.X, pady=1)
        ttk.Button(lin, text="Export A, B, C, D (.mat / .csv)…", command=self.export_lin).pack(fill=tk.X, pady=1)

        nb = ttk.Notebook(self)
        nb.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.nb = nb
        self.plot = PlotPanel(nb, (11, 7))
        nb.add(self.plot, text="Time response")
        lf = ttk.Frame(nb)
        nb.add(lf, text="Linear model")
        pw = ttk.Panedwindow(lf, orient=tk.VERTICAL)
        pw.pack(fill=tk.BOTH, expand=True)
        tfr, self.lin_txt = text_widget(pw, height=18)
        pw.add(tfr, weight=3)
        self.pole_plot = PlotPanel(pw, (10, 3))
        pw.add(self.pole_plot, weight=2)
        efr, eq = text_widget(nb, height=30)
        nb.add(efr, text="Model equations")
        set_text(eq, EQUATIONS)

    # -------------------------------------------------------- sliders
    def on_slider(self, key):
        if self._guard:
            return
        self._guard = True
        try:
            if key == "delta_FL" and self.link_steer.get():
                veh = self.app.vehicle(silent=True)
                if veh:
                    dl = math.radians(self.sv["delta_FL"][0].get())
                    lo, hi = -1.2, 1.2
                    for _ in range(60):
                        mid = 0.5 * (lo + hi)
                        if veh.steer_split(mid)[0] < dl:
                            lo = mid
                        else:
                            hi = mid
                    self.sv["delta_FR"][0].set(round(math.degrees(veh.steer_split(lo)[1]), 2))
            if key.startswith("Tm_") and self.link_torque.get():
                v = self.sv[key][0].get()
                for k in ("Tm_FL", "Tm_FR", "Tm_RL", "Tm_RR"):
                    self.sv[k][0].set(v)
        finally:
            self._guard = False

    def live(self):
        if self.live_on.get():
            self.run()

    def inputs(self):
        g = lambda k: self.sv[k][0].get()  # noqa: E731
        return [math.radians(g("delta_FL")), math.radians(g("delta_FR")), g("Tm_FL"), g("Tm_FR"),
                g("Tm_RL"), g("Tm_RR"), g("T_brake")]

    # -------------------------------------------------------- simulation
    def run(self):
        veh = self.app.vehicle()
        if veh is None:
            return
        u_step = self.inputs()
        ts = self.tstep.get()
        u0 = [0.0] * 7
        x0 = veh.initial_state(self.v0.get())
        self.sim = veh.simulate(x0, lambda t, x: u_step if t >= ts else u0, self.tend.get())
        self.sim_u = u_step
        self.draw_sim(veh)

    def draw_sim(self, veh):
        S = self.sim
        t, x, u, y = S["t"], S["x"], S["u"], S["y"]
        axs = self.plot.axes(2, 4)
        a = axs[0, 0]
        a.plot(t, x[:, 0], label="vx")
        a.plot(t, x[:, 1], label="vy")
        a.set_title("Body velocities [m/s]")
        a2 = axs[0, 1]
        a2.plot(t, np.degrees(x[:, 2]), label="r [deg/s]")
        a2.plot(t, np.degrees(y[:, 2]), label="β [deg]")
        a2.set_title("Yaw rate & sideslip")
        a3 = axs[0, 2]
        for i in range(4):
            a3.plot(t, x[:, 3 + i] * veh.p.R_w, color=CORNER_COLORS[i], label=f"ω·R {CORNERS[i]}")
        a3.plot(t, x[:, 0], "k--", lw=0.8, label="vx")
        a3.set_title("Wheel speeds ω·R [m/s]")
        a4 = axs[0, 3]
        for i in range(4):
            a4.plot(t, x[:, 7 + i], color=CORNER_COLORS[i], label=CORNERS[i])
        a4.set_title("Normal loads Fz [N]")
        a5 = axs[1, 0]
        for i in range(4):
            a5.plot(t, y[:, 3 + i], color=CORNER_COLORS[i], label=CORNERS[i])
        a5.set_title("Slip ratio κ [-]")
        a6 = axs[1, 1]
        for i in range(4):
            a6.plot(t, np.degrees(y[:, 7 + i]), color=CORNER_COLORS[i], label=CORNERS[i])
        a6.set_title("Slip angle α [deg]")
        a7 = axs[1, 2]
        a7.plot(t, y[:, 0] / 9.81, label="ax")
        a7.plot(t, y[:, 1] / 9.81, label="ay")
        a7.set_title("Accelerations [g]")
        a8 = axs[1, 3]
        a8.plot(x[:, 11], x[:, 12], "k-")
        step = max(1, len(t) // 15)
        a8.quiver(x[::step, 11], x[::step, 12], np.cos(x[::step, 13]), np.sin(x[::step, 13]),
                  color="tab:red", scale=25, width=0.006)
        a8.set_aspect("equal", adjustable="datalim")
        a8.set_title("Path X–Y [m] (→ heading)")
        for a_ in axs.flat:
            a_.grid(alpha=0.3)
            if a_ is not a8:
                a_.legend(fontsize=7, loc="upper right")
                a_.set_xlabel("t [s]", fontsize=8)
        self.plot.draw()
        self.nb.select(0)

    # -------------------------------------------------------- linear analysis
    def trim_lin(self):
        veh = self.app.vehicle()
        if veh is None:
            return
        try:
            v, R = float(self.lv.get()), float(self.lr.get())
        except ValueError:
            messagebox.showerror("Trim", "Invalid speed/radius")
            return
        x0, u0, info = veh.trim(v, R)
        hdr = [f"TRIM at vx = {v:g} m/s, R = {R:g} m  ->  {'converged' if info['success'] else 'NOT CONVERGED (beyond limit?)'}"
               f"  (max residual {info['residual']:.2e})",
               f"  bicycle steer {math.degrees(info['delta']):.3f} deg, total motor torque {info['T_total']:.3f} Nm",
               f"  ay = {v * v / R / 9.81 if R else 0:.3f} g"]
        self.linearise(veh, x0, u0, (1.0 / R) if R else 0.0, hdr)

    def final_lin(self):
        veh = self.app.vehicle()
        if veh is None or self.sim is None:
            messagebox.showinfo("Linearise", "Run an open-loop simulation first.")
            return
        x0 = list(self.sim["x"][-1])
        self.linearise(veh, x0, self.sim_u, 0.0, ["LINEARISED at final state of open-loop simulation"])

    def linearise(self, veh, x0, u0, curv, header):
        from scipy.linalg import eig

        A, B, C, D = veh.linearize(x0, u0, curv)
        self.lin = dict(A=A, B=B, C=C, D=D, x0=np.array(x0), u0=np.array(u0))
        nd = 11
        Ad = A[:nd, :nd]
        w, vl, vr = eig(Ad, left=True, right=True)
        order = np.argsort(-w.real)
        # PBH test: mode lambda is controllable iff rank [lambda I - A, B] = n
        Bd = B[:nd]
        ctrl = []
        for lam in w:
            M = np.hstack([lam * np.eye(nd) - Ad, Bd])
            sv = np.linalg.svd(M, compute_uv=False)
            ctrl.append(sv[-1] > 1e-9 * sv[0])
        rank = int(sum(ctrl))
        L = header + ["", "Operating point:"]
        L += [f"  {n:>9s} = {val:12.5g} {u}" for n, val, u in zip(STATE_NAMES, x0, STATE_UNITS)]
        L += [f"  {n:>9s} = {val:12.5g} {u}" for n, val, u in zip(INPUT_NAMES, u0, INPUT_UNITS)]
        L += ["", f"EIGENVALUES of dynamic subsystem (states 0–10; s, n, xi, E are kinematic integrators)",
              f"  controllable modes (PBH test): {rank} / {nd}",
              f"  {'#':>3s} {'real':>11s} {'imag':>11s} {'wn [Hz]':>9s} {'zeta':>7s} {'ctrb':>5s}  dominant states (participation)"]
        for k, i in enumerate(order):
            lam = w[i]
            wn = abs(lam)
            zeta = -lam.real / wn if wn > 0 else 0
            part = np.abs(vl[:, i].conj() * vr[:, i])
            part = part / part.sum() if part.sum() > 0 else part
            top = np.argsort(-part)[:3]
            dom = ", ".join(f"{STATE_NAMES[j]}({part[j]:.2f})" for j in top)
            L.append(f"  {k:3d} {lam.real:11.4g} {lam.imag:11.4g} {wn / 2 / math.pi:9.3f} {zeta:7.3f} "
                     f"{'yes' if ctrl[i] else 'NO':>5s}  {dom}")
        L += ["", "A = df/dx", fmt_matrix(A, STATE_NAMES, STATE_NAMES), "", "B = df/du",
              fmt_matrix(B, STATE_NAMES, INPUT_NAMES), "", "C = dh/dx", fmt_matrix(C, OUTPUT_NAMES, STATE_NAMES),
              "", "D = dh/du", fmt_matrix(D, OUTPUT_NAMES, INPUT_NAMES)]
        set_text(self.lin_txt, "\n".join(L))
        axs = self.pole_plot.axes(1, 2)
        for a_, lim in zip(axs, (None, 60)):
            a_.plot(w.real, w.imag, "x", ms=8, mew=2)
            a_.axvline(0, color="k", lw=0.8)
            a_.axhline(0, color="k", lw=0.8)
            a_.grid(alpha=0.3)
            a_.set_xlabel("Re [1/s]")
            a_.set_ylabel("Im [rad/s]")
        axs[0].set_title("Pole map (all)")
        axs[1].set_xlim(-60, 5)
        axs[1].set_ylim(-30, 30)
        axs[1].set_title("Pole map (zoom on chassis modes)")
        self.pole_plot.draw()
        self.nb.select(1)

    def export_lin(self):
        if self.lin is None:
            messagebox.showinfo("Export", "Linearise first.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".mat",
                                            filetypes=[("MATLAB", "*.mat"), ("CSV (one file per matrix)", "*.csv")])
        if not path:
            return
        if path.lower().endswith(".mat"):
            from scipy.io import savemat
            savemat(path, {**self.lin, "state_names": np.array(STATE_NAMES, dtype=object),
                           "input_names": np.array(INPUT_NAMES, dtype=object),
                           "output_names": np.array(OUTPUT_NAMES, dtype=object)})
        else:
            base = path[:-4]
            for k in ("A", "B", "C", "D"):
                np.savetxt(f"{base}_{k}.csv", self.lin[k], delimiter=",")
        messagebox.showinfo("Export", f"Saved to {path}")


# ====================================================================== GGV / tyre tab
class GGVTab(ttk.Frame):
    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        top = ttk.Frame(self, padding=4)
        top.pack(fill=tk.X)
        self.speeds = tk.StringVar(value="5, 10, 15, 20, 25, 30")
        self.loads = tk.StringVar(value="300, 600, 900, 1200")
        ttk.Label(top, text="GG speeds [m/s]").pack(side=tk.LEFT)
        ttk.Entry(top, textvariable=self.speeds, width=24).pack(side=tk.LEFT, padx=4)
        ttk.Label(top, text="Tyre loads [N]").pack(side=tk.LEFT)
        ttk.Entry(top, textvariable=self.loads, width=22).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Compute", command=self.compute).pack(side=tk.LEFT, padx=6)
        self.plot = PlotPanel(self, (12, 7))
        self.plot.pack(fill=tk.BOTH, expand=True)

    def compute(self):
        veh = self.app.vehicle()
        if veh is None:
            return
        try:
            speeds = [float(v) for v in self.speeds.get().split(",") if v.strip()]
            loads = [float(v) for v in self.loads.get().split(",") if v.strip()]
        except ValueError:
            messagebox.showerror("GGV", "Invalid list")
            return
        self.config(cursor="watch")
        self.update_idletasks()
        g = GGV(veh)
        self.config(cursor="")
        p = veh.p
        axs = self.plot.axes(2, 3)
        a = axs[0, 0]
        for v in speeds:
            v = min(v, g.v_max)
            aym = g.ay_max_at(v)
            ay = np.linspace(0, aym, 40)
            acc = np.array([g.ax_at(v, q, 1) for q in ay])
            brk = np.array([g.ax_at(v, q, -1) for q in ay])
            l, = a.plot(ay / 9.81, acc / 9.81, label=f"{v:g} m/s")
            a.plot(-ay / 9.81, acc / 9.81, color=l.get_color())
            a.plot(ay / 9.81, brk / 9.81, color=l.get_color())
            a.plot(-ay / 9.81, brk / 9.81, color=l.get_color())
        a.set_xlabel("ay [g]")
        a.set_ylabel("ax [g]")
        a.set_title("g-g diagram (GGV slices)")
        a.set_aspect("equal", adjustable="datalim")
        a.legend(fontsize=7)
        a = axs[0, 1]
        a.plot(g.v, g.ay_max / 9.81, label="ay max")
        a.plot(g.v, g.ax_acc[:, 0] / 9.81, label="ax max (accel)")
        a.plot(g.v, -g.ax_brk[:, 0] / 9.81, label="|ax| max (brake)")
        a.set_xlabel("v [m/s]")
        a.set_ylabel("[g]")
        a.set_title("Limits vs speed")
        a.legend(fontsize=7)
        a = axs[0, 2]
        v = np.linspace(0.1, veh.v_top * 1.02, 200)
        Fm, Pm = [], []
        for vv in v:
            _, tmax = veh.motor_torque_limits(vv / p.R_w)
            F = sum(veh.driven) * tmax * p.gear_ratio * p.eta_gear / p.R_w
            F = min(F, p.P_total_max_kW * 1e3 * p.eta_gear * p.eta_motor / vv)
            Fm.append(F)
        Fm = np.array(Fm)
        a.plot(v, Fm, label="max tractive force (motors)")
        a.plot(v, veh.q * p.CdA * v ** 2, label="aero drag")
        a.set_xlabel("v [m/s]")
        a.set_ylabel("[N]")
        a.set_title("Powertrain")
        a.legend(fontsize=7)
        kap = np.linspace(-0.4, 0.4, 300)
        alp = np.radians(np.linspace(-20, 20, 300))
        for fz in loads:
            axs[1, 0].plot(kap, [veh.tire(k, 0, fz)[0] for k in kap], label=f"{fz:g} N")
            axs[1, 1].plot(np.degrees(alp), [veh.tire(0, a_, fz)[1] for a_ in alp], label=f"{fz:g} N")
        axs[1, 0].set_title("Fx vs slip ratio κ")
        axs[1, 0].set_xlabel("κ [-]")
        axs[1, 1].set_title("Fy vs slip angle α")
        axs[1, 1].set_xlabel("α [deg]")
        a = axs[1, 2]
        kk = np.linspace(-0.4, 0.4, 200)
        for adeg in (0, 2, 4, 6, 8, 12):
            F = np.array([veh.tire(k, math.radians(adeg), p.Fz0) for k in kk])
            a.plot(F[:, 1], F[:, 0], label=f"α={adeg}°")
        a.set_xlabel("Fy [N]")
        a.set_ylabel("Fx [N]")
        a.set_title(f"Combined slip @ Fz0 = {p.Fz0:g} N")
        a.legend(fontsize=7)
        for a_ in axs.flat:
            a_.grid(alpha=0.3)
        axs[1, 0].legend(fontsize=7)
        axs[1, 1].legend(fontsize=7)
        self.plot.draw()


# ====================================================================== lap simulation tab
DRIVER_LABELS = {
    "preview_time": ("Steering preview time", "s"), "preview_min": ("Min preview distance", "m"),
    "k_yaw": ("Yaw-rate steer gain", "rad·s/rad"), "speed_scale": ("Target speed scale (of QSS)", "-"),
    "speed_lookahead": ("Speed target look-ahead", "s"), "k_speed": ("Speed error gain", "1/s"),
    "torque_vectoring": ("Torque vectoring", "bool"), "k_tv": ("Torque vectoring gain", "1/s"),
    "traction_control": ("Traction/slip control", "bool"), "slip_target": ("Slip target (×κ_peak)", "-"),
    "dt": ("Max integration step", "s"), "log_dt": ("Log interval", "s"),
}


class LapTab(ttk.Frame):
    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        self.result = None
        self._thread = None
        self._cancel = False
        self._prog = 0.0
        left = ttk.Frame(self, padding=4)
        left.pack(side=tk.LEFT, fill=tk.Y)
        mf = ttk.LabelFrame(left, text="Method", padding=4)
        mf.pack(fill=tk.X)
        self.method = tk.StringVar(value="both")
        ttk.Radiobutton(mf, text="Quasi-steady-state (GGV) only – fast", value="qss", variable=self.method).pack(anchor="w")
        ttk.Radiobutton(mf, text="QSS + transient 4-corner state-space model", value="both",
                        variable=self.method).pack(anchor="w")
        self.start = tk.StringVar(value="Auto")
        r = ttk.Frame(mf)
        r.pack(fill=tk.X)
        ttk.Label(r, text="Start:").pack(side=tk.LEFT)
        ttk.Combobox(r, textvariable=self.start, values=["Auto", "Flying lap", "Standing start"], state="readonly",
                     width=14).pack(side=tk.LEFT, padx=4)
        df = ttk.LabelFrame(left, text="Driver / controller (transient)", padding=4)
        df.pack(fill=tk.X, pady=4)
        self.dvars = {}
        for i, f in enumerate(fields(DriverSettings)):
            lbl, unit = DRIVER_LABELS[f.name]
            val = getattr(DriverSettings(), f.name)
            if unit == "bool":
                v = tk.BooleanVar(value=val)
                ttk.Checkbutton(df, text=lbl, variable=v).grid(row=i, column=0, columnspan=3, sticky="w")
            else:
                v = tk.StringVar(value=f"{val:g}")
                ttk.Label(df, text=lbl).grid(row=i, column=0, sticky="w")
                ttk.Entry(df, textvariable=v, width=8).grid(row=i, column=1)
                ttk.Label(df, text=unit, foreground="gray").grid(row=i, column=2, sticky="w")
            self.dvars[f.name] = v
        bf = ttk.Frame(left)
        bf.pack(fill=tk.X)
        self.run_btn = ttk.Button(bf, text="▶ Run lap simulation", command=self.run)
        self.run_btn.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(bf, text="Cancel", command=self.cancel).pack(side=tk.LEFT, padx=2)
        ttk.Button(bf, text="Export .mat…", command=self.export_mat).pack(side=tk.LEFT)
        ttk.Button(bf, text="Export CSV…", command=self.export).pack(side=tk.LEFT, padx=2)
        self.pbar = ttk.Progressbar(left, maximum=1.0)
        self.pbar.pack(fill=tk.X, pady=3)
        self.status = ttk.Label(left, text="Ready")
        self.status.pack(fill=tk.X)
        sfr, self.summary = text_widget(left, height=16, width=46)
        sfr.pack(fill=tk.BOTH, expand=True)

        nb = ttk.Notebook(self)
        nb.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.nb = nb
        self.p_over = PlotPanel(nb, (11, 7))
        self.p_corner = PlotPanel(nb, (11, 7))
        self.p_drv = PlotPanel(nb, (11, 7))
        nb.add(self.p_over, text="Overview")
        nb.add(self.p_corner, text="Corners (torque, load, slip)")
        nb.add(self.p_drv, text="Driver & energy")

    # -------------------------------------------------------- settings
    def settings(self) -> DriverSettings:
        kw = {}
        for f in fields(DriverSettings):
            v = self.dvars[f.name]
            if DRIVER_LABELS[f.name][1] == "bool":
                kw[f.name] = bool(v.get())
            else:
                kw[f.name] = float(v.get())
        st = DriverSettings(**kw)
        if st.dt <= 0 or st.log_dt <= 0:
            raise ValueError("Time steps must be > 0")
        return st

    def standing(self, track):
        s = self.start.get()
        return (not track.closed) if s == "Auto" else (s == "Standing start")

    # -------------------------------------------------------- run
    def run(self):
        if self._thread and self._thread.is_alive():
            return
        veh = self.app.vehicle()
        if veh is None:
            return
        try:
            track = self.app.track_tab.get_track()
            st = self.settings()
        except ValueError as e:
            messagebox.showerror("Lap simulation", str(e))
            return
        ss = self.standing(track)
        method = self.method.get()
        self._cancel = False
        self._prog = 0.0
        self._out = None
        self.run_btn.state(["disabled"])
        self.status.configure(text="Computing GGV + QSS…")

        def work():
            try:
                q = run_qss(veh, track, standing_start=ss)
                res = dict(qss=q, veh=veh, track=track, tr=None, st=st if method == "both" else None)
                if method == "both":
                    self._stage = "Transient simulation…"
                    res["tr"] = run_transient(veh, track, st, qss=q, standing_start=ss,
                                              progress=lambda f: setattr(self, "_prog", f),
                                              cancel=lambda: self._cancel)
                self._out = res
            except Exception:  # noqa: BLE001
                self._out = dict(error=traceback.format_exc())

        self._stage = "Computing GGV + QSS…"
        self._thread = threading.Thread(target=work, daemon=True)
        self._thread.start()
        self.after(100, self._poll)

    def cancel(self):
        self._cancel = True

    def _poll(self):
        self.pbar["value"] = self._prog
        self.status.configure(text=f"{self._stage} {self._prog * 100:.0f}%")
        if self._thread.is_alive():
            self.after(150, self._poll)
            return
        self.run_btn.state(["!disabled"])
        out = self._out
        if out is None or "error" in out:
            self.status.configure(text="Error")
            messagebox.showerror("Lap simulation", (out or {}).get("error", "unknown error"))
            return
        self.result = out
        tr = out["tr"]
        self.status.configure(text="Done" if tr is None else f"Done – transient status: {tr['status']}")
        self.pbar["value"] = 1.0
        self.show(out)
        self.app.publish_sim(out["qss"], tr)

    # -------------------------------------------------------- results
    def show(self, R):
        q, tr, veh, track = R["qss"], R["tr"], R["veh"], R["track"]
        p = veh.p
        L = [f"Track: {track.name}  ({track.length:.1f} m, {'closed' if track.closed else 'open'})",
             f"Start: {'standing' if self.standing(track) else 'flying'}", "",
             "QUASI-STEADY-STATE",
             f"  Lap time        : {q['lap_time']:.3f} s",
             f"  Avg speed       : {track.length / q['lap_time'] * 3.6:.1f} km/h",
             f"  Max speed       : {q['v'].max() * 3.6:.1f} km/h",
             f"  Min speed       : {q['v'][1:].min() * 3.6:.1f} km/h",
             f"  Max |ay|        : {np.abs(q['ay']).max() / 9.81:.2f} g",
             f"  Max ax / min ax : {q['ax'].max() / 9.81:.2f} / {q['ax'].min() / 9.81:.2f} g",
             f"  Energy (net)    : {q['E'][-1] / 3.6e6 * 1000:.1f} Wh"]
        if tr is not None:
            x, y = tr["x"], tr["y"]
            E_regen = -np.trapezoid(np.minimum(y[:, -4:].sum(axis=1), 0), tr["t"])
            L += ["", "TRANSIENT 4-CORNER MODEL",
                  f"  Status          : {tr['status']}",
                  f"  Lap time        : {tr['lap_time']:.3f} s"
                  + (f"  ({(tr['lap_time'] / q['lap_time'] - 1) * 100:+.2f}% vs QSS)" if tr["status"] == "ok" else ""),
                  f"  Max speed       : {x[:, 0].max() * 3.6:.1f} km/h",
                  f"  Max |ay|        : {np.abs(y[:, 1]).max() / 9.81:.2f} g",
                  f"  Max ax / min ax : {y[:, 0].max() / 9.81:.2f} / {y[:, 0].min() / 9.81:.2f} g",
                  f"  Max |n| offset  : {np.abs(x[:, 12]).max():.2f} m",
                  f"  Max |β|         : {np.degrees(np.abs(y[:, 2]).max()):.2f} deg",
                  f"  Energy (net)    : {x[-1, 14] / 3600:.1f} Wh",
                  f"  Regen recovered : {E_regen / 3600:.1f} Wh",
                  f"  Peak elec. power: {y[:, -4:].sum(axis=1).max() / 1e3:.1f} kW (limit {p.P_total_max_kW:g})",
                  f"  Wall-clock time : {tr['wall_time']:.1f} s"]
        set_text(self.summary, "\n".join(L))
        self._draw_overview(R)
        self._draw_corners(R)
        self._draw_driver(R)
        self.nb.select(0)

    def _draw_overview(self, R):
        q, tr, veh = R["qss"], R["tr"], R["veh"]
        axs = self.p_over.axes(2, 2)
        a = axs[0, 0]
        a.plot(q["s"], q["v"] * 3.6, label=f"QSS ({q['lap_time']:.2f} s)")
        a.plot(q["s"], np.minimum(q["v_apex"], veh.v_top) * 3.6, ":", color="0.6", lw=0.8, label="apex limit")
        if tr is not None:
            s = tr["x"][:, 11]
            a.plot(s, np.hypot(tr["x"][:, 0], tr["x"][:, 1]) * 3.6, label=f"Transient ({tr['lap_time']:.2f} s)")
        a.set_xlabel("s [m]")
        a.set_ylabel("speed [km/h]")
        a.set_title("Speed trace")
        a.legend(fontsize=8)
        a = axs[0, 1]
        if tr is not None:
            X, Y, V = tr["X"], tr["Y"], np.hypot(tr["x"][:, 0], tr["x"][:, 1]) * 3.6
        else:
            X, Y, V = q["X"], q["Y"], q["v"] * 3.6
        a.plot(q["X"], q["Y"], color="0.85", lw=6, zorder=0)
        sc = a.scatter(X, Y, c=V, cmap="turbo", s=5)
        self.p_over.fig.colorbar(sc, ax=a, label="km/h")
        a.set_aspect("equal", adjustable="datalim")
        a.set_title("Track map coloured by speed")
        a = axs[1, 0]
        a.plot(q["ay"] / 9.81, q["ax"] / 9.81, ".", ms=2, label="QSS")
        if tr is not None:
            a.plot(tr["y"][:, 1] / 9.81, tr["y"][:, 0] / 9.81, ".", ms=2, label="Transient")
        a.set_xlabel("ay [g]")
        a.set_ylabel("ax [g]")
        a.set_title("g-g scatter")
        a.set_aspect("equal", adjustable="datalim")
        a.legend(fontsize=8, markerscale=4)
        a = axs[1, 1]
        a.plot(q["s"], q["ax"] / 9.81, label="ax QSS")
        a.plot(q["s"], q["ay"] / 9.81, label="ay QSS")
        if tr is not None:
            a.plot(tr["x"][:, 11], tr["y"][:, 0] / 9.81, "--", label="ax transient")
            a.plot(tr["x"][:, 11], tr["y"][:, 1] / 9.81, "--", label="ay transient")
        a.set_xlabel("s [m]")
        a.set_ylabel("[g]")
        a.set_title("Accelerations")
        a.legend(fontsize=7, ncol=2)
        for a_ in axs.flat:
            a_.grid(alpha=0.3)
        self.p_over.draw()

    def _draw_corners(self, R):
        q, tr = R["qss"], R["tr"]
        axs = self.p_corner.axes(2, 2)
        if tr is None:
            for i in range(4):
                axs[0, 1].plot(q["s"], q["Fz"][:, i], color=CORNER_COLORS[i], label=CORNERS[i])
            axs[0, 1].set_title("Normal loads Fz [N] (QSS)")
            axs[0, 1].legend(fontsize=7)
            axs[0, 0].text(0.5, 0.5, "Run the transient model\nfor per-corner torques & slips", ha="center",
                           va="center", transform=axs[0, 0].transAxes)
        else:
            s = tr["x"][:, 11]
            for i in range(4):
                c = CORNER_COLORS[i]
                axs[0, 0].plot(s, tr["u"][:, 2 + i], color=c, lw=1, label=CORNERS[i])
                axs[0, 1].plot(s, tr["x"][:, 7 + i], color=c, lw=1, label=CORNERS[i])
                axs[1, 0].plot(s, tr["y"][:, 3 + i], color=c, lw=1, label=CORNERS[i])
                axs[1, 1].plot(s, np.degrees(tr["y"][:, 7 + i]), color=c, lw=1, label=CORNERS[i])
            ax2 = axs[0, 0].twinx()
            ax2.fill_between(s, tr["u"][:, 6], color="0.5", alpha=0.25, label="friction brake")
            ax2.set_ylabel("friction brake [Nm]", fontsize=8)
            axs[0, 0].set_title("Motor torques (inputs u) [Nm]")
            axs[0, 1].set_title("Normal loads Fz [N]")
            axs[1, 0].set_title("Slip ratio κ [-]")
            axs[1, 1].set_title("Slip angle α [deg]")
            for a_ in axs.flat:
                a_.legend(fontsize=7, loc="upper right")
                a_.set_xlabel("s [m]")
        for a_ in axs.flat:
            a_.grid(alpha=0.3)
        self.p_corner.draw()

    def _draw_driver(self, R):
        q, tr = R["qss"], R["tr"]
        axs = self.p_drv.axes(2, 2)
        a = axs[0, 0]
        a.plot(q["s"], q["kappa"], label="track curvature [1/m]")
        a.set_title("Curvature & steering")
        if tr is not None:
            s = tr["x"][:, 11]
            a2 = a.twinx()
            a2.plot(s, np.degrees(tr["u"][:, 0]), "tab:orange", lw=1, label="δ FL")
            a2.plot(s, np.degrees(tr["u"][:, 1]), "tab:red", lw=1, label="δ FR")
            a2.set_ylabel("steer [deg]")
            a2.legend(fontsize=7, loc="lower right")
            axs[0, 1].plot(s, tr["x"][:, 12], label="n (lateral offset) [m]")
            axs[0, 1].plot(s, np.degrees(tr["x"][:, 13]), label="ξ (heading error) [deg]")
            axs[0, 1].plot(s, np.degrees(tr["y"][:, 2]), label="β (sideslip) [deg]")
            axs[0, 1].set_title("Path tracking")
            axs[0, 1].legend(fontsize=7)
            P = tr["y"][:, -4:].sum(axis=1) / 1e3
            axs[1, 0].plot(s, P, label="transient")
            axs[1, 1].plot(s, tr["x"][:, 14] / 3600, label="transient")
        a.legend(fontsize=7, loc="upper left")
        axs[1, 0].plot(q["s"], q["P_el"] / 1e3, "--", label="QSS")
        axs[1, 0].axhline(R["veh"].p.P_total_max_kW, color="r", lw=0.8, label="limit")
        axs[1, 0].set_title("Battery power [kW]")
        axs[1, 0].legend(fontsize=7)
        axs[1, 1].plot(q["s"], q["E"] / 3600, "--", label="QSS")
        axs[1, 1].set_title("Energy used [Wh]")
        axs[1, 1].legend(fontsize=7)
        for a_ in axs.flat:
            a_.grid(alpha=0.3)
            a_.set_xlabel("s [m]")
        self.p_drv.draw()

    def export(self):
        if not self.result:
            messagebox.showinfo("Export", "Run a simulation first.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")])
        if not path:
            return
        q, tr = self.result["qss"], self.result["tr"]
        base = path[:-4] if path.lower().endswith(".csv") else path
        with open(base + "_qss.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["s", "t", "v", "ax", "ay", "kappa", "Fz_FL", "Fz_FR", "Fz_RL", "Fz_RR", "P_el", "E"])
            for i in range(len(q["s"])):
                w.writerow([q["s"][i], q["t"][i], q["v"][i], q["ax"][i], q["ay"][i], q["kappa"][i],
                            *q["Fz"][i], q["P_el"][i], q["E"][i]])
        msg = base + "_qss.csv"
        if tr is not None:
            with open(base + "_transient.csv", "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["t", *STATE_NAMES, *INPUT_NAMES, *Y_COLS, "v_ref", "X", "Y", "psi"])
                for i in range(len(tr["t"])):
                    w.writerow([tr["t"][i], *tr["x"][i], *tr["u"][i], *tr["y"][i], tr["v_ref"][i],
                                tr["X"][i], tr["Y"][i], tr["psi"][i]])
            msg += "\n" + base + "_transient.csv"
        messagebox.showinfo("Export", "Saved:\n" + msg)

    def export_mat(self):
        if not self.result:
            messagebox.showinfo("Export", "Run a simulation first.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".mat", filetypes=[("MATLAB", "*.mat")])
        if not path:
            return
        R = self.result
        try:
            save_lap_mat(path, R["qss"], R["tr"], R["veh"].p, R["track"], R.get("st"))
        except Exception as e:  # noqa: BLE001
            messagebox.showerror("Export", str(e))
            return
        messagebox.showinfo("Export", f"Saved {path}\n\nStructs: qss" + (", transient" if R["tr"] else "")
                            + ", vehicle, track, meta (each channel struct has a 'units' field)")


# ====================================================================== channel plot tab
class ChannelTab(ttk.Frame):
    """Plot any channel against any other from simulation results and imported .mat files."""

    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        self.traces: list[dict] = []
        self._ynames: list[str] = []
        left = ttk.Frame(self, padding=4)
        left.pack(side=tk.LEFT, fill=tk.Y)
        df = ttk.LabelFrame(left, text="Datasets (lap simulation results & imported .mat files)", padding=4)
        df.pack(fill=tk.X)
        self.ds_list = tk.Listbox(df, height=6, exportselection=False, width=50)
        self.ds_list.pack(fill=tk.X)
        self.ds_list.bind("<<ListboxSelect>>", lambda e: self.on_dataset())
        bf = ttk.Frame(df)
        bf.pack(fill=tk.X, pady=2)
        ttk.Button(bf, text="Load .mat…", command=self.load).pack(side=tk.LEFT)
        ttk.Button(bf, text="Snapshot copy", command=self.snapshot).pack(side=tk.LEFT, padx=2)
        ttk.Button(bf, text="Remove", command=self.remove).pack(side=tk.LEFT)
        self.ds_info = ttk.Label(df, text="", foreground="gray", wraplength=360, justify=tk.LEFT)
        self.ds_info.pack(fill=tk.X)

        cf = ttk.LabelFrame(left, text="Channels of the selected dataset", padding=4)
        cf.pack(fill=tk.BOTH, expand=True, pady=4)
        r = ttk.Frame(cf)
        r.pack(fill=tk.X)
        ttk.Label(r, text="X axis").pack(side=tk.LEFT)
        self.xvar = tk.StringVar()
        self.xcombo = ttk.Combobox(r, textvariable=self.xvar, state="readonly", width=36)
        self.xcombo.pack(side=tk.LEFT, padx=4, fill=tk.X, expand=True)
        r = ttk.Frame(cf)
        r.pack(fill=tk.X, pady=2)
        ttk.Label(r, text="Filter Y").pack(side=tk.LEFT)
        self.filt = tk.StringVar()
        self.filt.trace_add("write", lambda *a: self.fill_channels())
        ttk.Entry(r, textvariable=self.filt).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        lf = ttk.Frame(cf)
        lf.pack(fill=tk.BOTH, expand=True)
        self.ylist = tk.Listbox(lf, selectmode=tk.EXTENDED, exportselection=False, height=12, font=MONO)
        sb = ttk.Scrollbar(lf, command=self.ylist.yview)
        self.ylist.configure(yscrollcommand=sb.set)
        self.ylist.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.ylist.bind("<Double-Button-1>", lambda e: self.add_traces())
        of = ttk.Frame(cf)
        of.pack(fill=tk.X, pady=2)
        self.axis, self.xoff = tk.StringVar(value="1"), tk.StringVar(value="0")
        self.yscale, self.yoff = tk.StringVar(value="1"), tk.StringVar(value="0")
        for lbl, var, w in (("Subplot", self.axis, 3), ("X offset", self.xoff, 7),
                            ("Y scale", self.yscale, 6), ("Y offset", self.yoff, 6)):
            ttk.Label(of, text=lbl).pack(side=tk.LEFT)
            ttk.Entry(of, textvariable=var, width=w).pack(side=tk.LEFT, padx=(1, 5))
        ttk.Button(cf, text="Add selected Y channel(s) as trace(s)  ▼   (or double-click)",
                   command=self.add_traces).pack(fill=tk.X)

        tf = ttk.LabelFrame(left, text="Traces", padding=4)
        tf.pack(fill=tk.BOTH, expand=True)
        cols = ("ax", "ds", "x", "y", "mod")
        self.tree = ttk.Treeview(tf, columns=cols, show="headings", height=6)
        for c, t, w in zip(cols, ("Plot", "Dataset", "X", "Y", "x-off/scale/y-off"), (35, 105, 75, 100, 100)):
            self.tree.heading(c, text=t)
            self.tree.column(c, width=w, anchor="w")
        self.tree.pack(fill=tk.BOTH, expand=True)
        bf = ttk.Frame(tf)
        bf.pack(fill=tk.X, pady=2)
        ttk.Button(bf, text="Remove trace", command=self.remove_trace).pack(side=tk.LEFT)
        ttk.Button(bf, text="Clear all", command=self.clear_traces).pack(side=tk.LEFT, padx=2)
        self.scatter, self.sharex = tk.BooleanVar(value=False), tk.BooleanVar(value=True)
        ttk.Checkbutton(bf, text="Markers only", variable=self.scatter, command=self.plot_traces).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(bf, text="Share X", variable=self.sharex, command=self.plot_traces).pack(side=tk.LEFT)
        self.msg = ttk.Label(tf, text="", foreground="firebrick", wraplength=360, justify=tk.LEFT)
        self.msg.pack(fill=tk.X)
        self.plot = PlotPanel(self, (10, 7))
        self.plot.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.refresh_datasets()

    # -------------------------------------------------------- datasets
    def refresh_datasets(self, select: str | None = None):
        cur = select or self._selected_name()
        names = list(self.app.datasets)
        self.ds_list.delete(0, tk.END)
        for n in names:
            self.ds_list.insert(tk.END, n)
        if cur in names:
            i = names.index(cur)
        elif names:
            i = 0
        else:
            i = None
        if i is not None:
            self.ds_list.selection_set(i)
            self.ds_list.see(i)
        self.on_dataset()
        self.plot_traces()

    def _selected_name(self):
        s = self.ds_list.curselection() if hasattr(self, "ds_list") else ()
        return self.ds_list.get(s[0]) if s else None

    def current(self):
        n = self._selected_name()
        return self.app.datasets.get(n) if n else None

    def on_dataset(self):
        ds = self.current()
        if ds is None:
            self.xcombo["values"] = []
            self.xvar.set("")
            self.ds_info.configure(text="No datasets yet: run a lap simulation or load a .mat file.")
            self.fill_channels()
            return
        names = ds.names()
        self.xcombo["values"] = names
        if self.xvar.get() not in names:
            self.xvar.set(ds.default_x() or "")
        lens = sorted({len(v) for v in ds.channels.values()})
        ln = f"{lens[0]}" if len(lens) == 1 else f"{lens[0]}–{lens[-1]}"
        self.ds_info.configure(text=f"{len(names)} channels, {ln} samples\n{ds.source}")
        self.fill_channels()

    def fill_channels(self):
        ds = self.current()
        self.ylist.delete(0, tk.END)
        self._ynames = []
        if ds is None:
            return
        terms = self.filt.get().lower().split()
        for ch in ds.names():
            if all(t in ch.lower() for t in terms):
                self._ynames.append(ch)
                self.ylist.insert(tk.END, f"{ds.label(ch)}  ({len(ds.channels[ch])})")

    def load(self):
        paths = filedialog.askopenfilenames(filetypes=[("MATLAB data", "*.mat"), ("All files", "*.*")])
        last = None
        for path in paths:
            try:
                ds = load_mat(path)
            except Exception as e:  # noqa: BLE001
                messagebox.showerror("Load .mat", f"{path}\n\n{e}")
                continue
            ds.name = self.app.unique_name(ds.name)
            self.app.datasets[ds.name] = ds
            last = ds.name
        if last:
            self.refresh_datasets(last)

    def snapshot(self):
        ds = self.current()
        if ds is None:
            return
        import datetime as dt
        cp = ds.copy(self.app.unique_name(f"{ds.name} @ {dt.datetime.now():%H:%M:%S}"))
        self.app.datasets[cp.name] = cp
        self.refresh_datasets(cp.name)

    def remove(self):
        n = self._selected_name()
        if n is None:
            return
        del self.app.datasets[n]
        self.traces = [t for t in self.traces if t["ds"] != n]
        self._refill_tree()
        self.refresh_datasets()

    # -------------------------------------------------------- traces
    def add_traces(self):
        ds = self.current()
        sel = self.ylist.curselection()
        if ds is None or not sel:
            return
        xch = self.xvar.get()
        if xch not in ds.channels:
            messagebox.showinfo("Channel plot", "Choose an X-axis channel first.")
            return
        try:
            axn = max(1, min(8, int(self.axis.get())))
            xo, ys, yo = float(self.xoff.get()), float(self.yscale.get()), float(self.yoff.get())
        except ValueError:
            messagebox.showerror("Channel plot", "Subplot, offsets and scale must be numbers")
            return
        for i in sel:
            self.traces.append(dict(ds=ds.name, x=xch, y=self._ynames[i], axis=axn, xoff=xo, yscale=ys, yoff=yo))
        self._refill_tree()
        self.plot_traces()

    def _refill_tree(self):
        self.tree.delete(*self.tree.get_children())
        for i, t in enumerate(self.traces):
            self.tree.insert("", tk.END, iid=str(i), values=(t["axis"], t["ds"], t["x"], t["y"],
                                                             f"{t['xoff']:g} / {t['yscale']:g} / {t['yoff']:g}"))

    def remove_trace(self):
        idx = sorted((int(i) for i in self.tree.selection()), reverse=True)
        for i in idx:
            del self.traces[i]
        self._refill_tree()
        self.plot_traces()

    def clear_traces(self):
        self.traces.clear()
        self._refill_tree()
        self.plot_traces()

    def plot_traces(self):
        fig = self.plot.fig
        fig.clf()
        errs = []
        if not self.traces:
            ax = fig.add_subplot(111)
            ax.text(0.5, 0.5, "Select a dataset, an X channel and one or more Y channels,\nthen 'Add' to plot.",
                    ha="center", va="center", transform=ax.transAxes, color="gray")
            ax.set_axis_off()
            self.msg.configure(text="")
            self.plot.draw()
            return
        n = max(t["axis"] for t in self.traces)
        axs = fig.subplots(n, 1, sharex=self.sharex.get(), squeeze=False)[:, 0]
        ylabels = [[] for _ in range(n)]
        xlabels = []
        for t in self.traces:
            ds = self.app.datasets.get(t["ds"])
            if ds is None or t["x"] not in ds.channels or t["y"] not in ds.channels:
                errs.append(f"{t['ds']}: {t['y']} vs {t['x']} – channel or dataset no longer available")
                continue
            try:
                x, y = ds.xy(t["x"], t["y"])
            except ValueError as e:
                errs.append(f"{t['ds']}: {e}")
                continue
            x = x + t["xoff"]
            y = y * t["yscale"] + t["yoff"]
            mod = "" if (t["xoff"], t["yscale"], t["yoff"]) == (0, 1, 0) else \
                f" (x{t['xoff']:+g}, ×{t['yscale']:g}{t['yoff']:+g})"
            lbl = f"{t['ds']}: {t['y']}{mod}"
            ax = axs[t["axis"] - 1]
            if self.scatter.get():
                ax.plot(x, y, ".", ms=2, label=lbl)
            else:
                ax.plot(x, y, lw=1, label=lbl)
            yl = ds.label(t["y"]) if t["yscale"] == 1 else t["y"]
            if yl not in ylabels[t["axis"] - 1]:
                ylabels[t["axis"] - 1].append(yl)
            if ds.label(t["x"]) not in xlabels:
                xlabels.append(ds.label(t["x"]))
        for i, ax in enumerate(axs):
            ax.grid(alpha=0.3)
            if ax.has_data():
                ax.legend(fontsize=7, markerscale=4 if self.scatter.get() else 1)
            ax.set_ylabel(", ".join(ylabels[i])[:60], fontsize=8)
        axs[-1].set_xlabel(", ".join(xlabels)[:90])
        self.msg.configure(text="\n".join(errs[:4]))
        self.plot.draw()


# ====================================================================== parameter sweep tab
SWEEP_NONE = "(none – 1-D sweep)"


class SweepTab(ttk.Frame):
    """1-D or 2-D QSS parameter sweep; iso-lap-time contour lines for 2-D sweeps."""

    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        self.res = None
        self._cancel = False
        self._meta = {n: (lbl, unit) for n, lbl, unit, _ in PARAM_META if unit != "bool"}
        names = [self._disp(n) for n in self._meta]
        left = ttk.Frame(self, padding=4)
        left.pack(side=tk.LEFT, fill=tk.Y)
        self.pv = []
        specs = (("Parameter 1  (x axis)", "m", "240", "320", "9", names),
                 ("Parameter 2  (y axis)", "P_total_max_kW", "60", "80", "9", [SWEEP_NONE] + names))
        for k, (title, default, lo, hi, n, vals) in enumerate(specs):
            lf = ttk.LabelFrame(left, text=title, padding=4)
            lf.pack(fill=tk.X, pady=2)
            var = tk.StringVar(value=self._disp(default))
            cb = ttk.Combobox(lf, textvariable=var, values=vals, state="readonly", width=44)
            cb.grid(row=0, column=0, columnspan=6, sticky="ew", pady=(0, 3))
            cb.bind("<<ComboboxSelected>>", lambda e, k=k: self.autorange(k))
            lo_v, hi_v, n_v = tk.StringVar(value=lo), tk.StringVar(value=hi), tk.StringVar(value=n)
            for j, (lbl, v) in enumerate((("from", lo_v), ("to", hi_v), ("points", n_v))):
                ttk.Label(lf, text=lbl).grid(row=1, column=2 * j, sticky="e")
                ttk.Entry(lf, textvariable=v, width=9).grid(row=1, column=2 * j + 1, padx=(2, 8))
            self.pv.append((var, lo_v, hi_v, n_v))
        rf = ttk.LabelFrame(left, text="Run (QSS lap simulation at every grid point)", padding=4)
        rf.pack(fill=tk.X, pady=2)
        r = ttk.Frame(rf)
        r.pack(fill=tk.X)
        ttk.Label(r, text="Start:").pack(side=tk.LEFT)
        self.start = tk.StringVar(value="Auto")
        ttk.Combobox(r, textvariable=self.start, values=["Auto", "Flying lap", "Standing start"], state="readonly",
                     width=14).pack(side=tk.LEFT, padx=4)
        import os
        self.parallel = tk.BooleanVar(value=True)
        ttk.Checkbutton(r, text=f"Parallel ({max(1, (os.cpu_count() or 2) - 1)} processes)",
                        variable=self.parallel).pack(side=tk.LEFT, padx=6)
        ttk.Label(rf, text="Track = the Track tab's current track; every other parameter is taken from "
                           "the Vehicle parameters tab.", foreground="gray", wraplength=360,
                  justify=tk.LEFT).pack(fill=tk.X, pady=2)
        bf = ttk.Frame(rf)
        bf.pack(fill=tk.X)
        self.run_btn = ttk.Button(bf, text="▶ Run sweep", command=self.run)
        self.run_btn.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(bf, text="Cancel", command=lambda: setattr(self, "_cancel", True)).pack(side=tk.LEFT, padx=2)
        self.pbar = ttk.Progressbar(rf, maximum=1.0)
        self.pbar.pack(fill=tk.X, pady=2)
        self.status = ttk.Label(rf, text="")
        self.status.pack(fill=tk.X)
        of = ttk.LabelFrame(left, text="Display", padding=4)
        of.pack(fill=tk.X, pady=2)
        r = ttk.Frame(of)
        r.pack(fill=tk.X)
        ttk.Label(r, text="Metric").pack(side=tk.LEFT)
        self.metric = tk.StringVar(value="Lap time [s]")
        mc = ttk.Combobox(r, textvariable=self.metric, values=["Lap time [s]", "Energy per lap [Wh]"],
                          state="readonly", width=20)
        mc.pack(side=tk.LEFT, padx=4)
        mc.bind("<<ComboboxSelected>>", lambda e: self.draw())
        self.filled = tk.BooleanVar(value=True)
        ttk.Checkbutton(r, text="Colour fill", variable=self.filled, command=self.draw).pack(side=tk.LEFT, padx=4)
        r = ttk.Frame(of)
        r.pack(fill=tk.X, pady=3)
        ttk.Label(r, text="Constant-value lines").pack(side=tk.LEFT)
        self.levels = tk.StringVar(value="auto")
        e = ttk.Entry(r, textvariable=self.levels, width=18)
        e.pack(side=tk.LEFT, padx=4)
        e.bind("<Return>", lambda ev: self.draw())
        ttk.Button(r, text="Redraw", command=self.draw).pack(side=tk.LEFT)
        ttk.Label(of, text="e.g.  auto   |   45, 47, 49   |   45:0.5:50  (start:step:stop)",
                  foreground="gray").pack(anchor="w")
        ttk.Button(of, text="Export sweep results (.mat / .csv)…", command=self.export).pack(fill=tk.X, pady=2)
        sfr, self.summary = text_widget(left, height=12, width=50)
        sfr.pack(fill=tk.BOTH, expand=True)
        self.plot = PlotPanel(self, (10, 7))
        self.plot.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    def _disp(self, n):
        lbl, unit = self._meta[n]
        return f"{n}  ({lbl}, {unit})"

    @staticmethod
    def _key(s):
        return None if s == SWEEP_NONE else s.split("  ")[0]

    def _unit(self, n):
        return self._meta.get(n, ("", ""))[1]

    def autorange(self, k):
        var, lo_v, hi_v, _ = self.pv[k]
        key = self._key(var.get())
        if key is None:
            return
        try:
            base = getattr(self.app.vehicle_tab.get_params(), key)
        except ValueError:
            base = getattr(VehicleParams(), key)
        lo, hi = (base * 0.8, base * 1.2) if base else (-1.0, 1.0)
        lo, hi = min(lo, hi), max(lo, hi)
        lo_v.set(f"{lo:.4g}")
        hi_v.set(f"{hi:.4g}")

    # -------------------------------------------------------- run
    def run(self):
        if getattr(self, "_thread", None) and self._thread.is_alive():
            return
        try:
            base = self.app.vehicle_tab.get_params()
            track = self.app.track_tab.get_track()
            axes = []
            for var, lo_v, hi_v, n_v in self.pv:
                key = self._key(var.get())
                if key is None:
                    axes.append(None)
                    continue
                n = int(n_v.get())
                if not 2 <= n <= 101:
                    raise ValueError("Number of points must be between 2 and 101")
                axes.append((key, np.linspace(float(lo_v.get()), float(hi_v.get()), n)))
        except ValueError as e:
            messagebox.showerror("Sweep", str(e))
            return
        (p1, v1), second = axes
        p2, v2 = second if second else (None, None)
        if p2 == p1:
            messagebox.showerror("Sweep", "Choose two different parameters (or '(none)' for a 1-D sweep).")
            return
        total = len(v1) * (len(v2) if p2 else 1)
        if total > 400 and not messagebox.askyesno("Sweep", f"{total} lap simulations – continue?"):
            return
        s = self.start.get()
        standing = (not track.closed) if s == "Auto" else (s == "Standing start")
        self._cancel = False
        self._prog = 0.0
        self._out = None
        self.run_btn.state(["disabled"])
        par = self.parallel.get()

        def work():
            try:
                self._out = run_sweep(base, track, p1, v1, p2, v2, standing=standing, parallel=par,
                                      progress=lambda f: setattr(self, "_prog", f),
                                      cancel=lambda: self._cancel)
            except Exception:  # noqa: BLE001
                self._out = dict(error=traceback.format_exc())

        self._thread = threading.Thread(target=work, daemon=True)
        self._thread.start()
        self._t_start = __import__("time").time()
        self.after(150, lambda: self._poll(total))

    def _poll(self, total):
        el = __import__("time").time() - self._t_start
        self.pbar["value"] = self._prog
        self.status.configure(text=f"{self._prog * total:.0f} / {total} laps   ({el:.0f} s)")
        if self._thread.is_alive():
            self.after(200, lambda: self._poll(total))
            return
        self.run_btn.state(["!disabled"])
        out = self._out
        if out is None or "error" in out:
            messagebox.showerror("Sweep", (out or {}).get("error", "unknown error"))
            return
        self.res = out
        self.status.configure(text=f"{out['status']} – {total} laps in {el:.1f} s")
        self.draw()

    # -------------------------------------------------------- display
    def draw(self):
        R = self.res
        if R is None:
            return
        lap = self.metric.get().startswith("Lap")
        Z = R["T"] if lap else R["E"]
        unit = "s" if lap else "Wh"
        zname = "Lap time" if lap else "Energy per lap"
        p1, p2, v1, v2 = R["p1"], R["p2"], R["vals1"], R["vals2"]
        b1 = R["base"][p1]
        u1 = self._unit(p1)
        fin = np.isfinite(Z)
        lines = [f"Track: {R['track']}  ({'standing' if R['standing'] else 'flying'} start)",
                 f"Status: {R['status']}"]
        ax = self.plot.axes(1, 1)
        if not fin.any():
            ax.text(0.5, 0.5, "No valid results", ha="center", transform=ax.transAxes)
            self.plot.draw()
            set_text(self.summary, "\n".join(lines))
            return
        zmin, zmax = float(np.nanmin(Z)), float(np.nanmax(Z))
        try:
            levels = parse_levels(self.levels.get(), zmin, zmax)
        except ValueError as e:
            messagebox.showerror("Levels", str(e))
            levels = np.array([])
        lines.append(f"{zname}: {zmin:.3f} – {zmax:.3f} {unit}")
        if (~fin).any():
            lines.append(f"{int((~fin).sum())} grid point(s) invalid (parameter out of range) – shown as gaps")
        if p2 is None:
            ax.plot(v1, Z, "o-", label=zname)
            ax.axvline(b1, color="tab:green", ls=":", label=f"baseline {p1} = {b1:g}")
            lines.append("")
            lines.append(f"Parameter values on the constant-value lines ({p1} [{u1}]):")
            for L in levels:
                ax.axhline(L, color="0.5", ls="--", lw=0.8)
                xs = crossings(v1, Z, L)
                for xc in xs:
                    ax.plot(xc, L, "s", color="tab:red", ms=5)
                    ax.annotate(f"{xc:.4g}", (xc, L), xytext=(4, 4), textcoords="offset points", fontsize=8)
                lines.append(f"  {L:g} {unit}:  " + (", ".join(f"{x:.4g}" for x in xs) if xs else "–"))
            g = np.gradient(Z, v1)
            lines.append("")
            lines.append(f"Sensitivity at baseline: {np.interp(b1, v1, g):+.4g} {unit} per {u1}")
            ax.set_xlabel(f"{p1} [{u1}]")
            ax.set_ylabel(f"{zname} [{unit}]")
            ax.legend(fontsize=8)
            ax.set_title(f"{zname} vs {p1} – {R['track']}")
        else:
            u2 = self._unit(p2)
            b2 = R["base"][p2]
            Zm = np.ma.masked_invalid(Z)
            if self.filled.get():
                cf = ax.contourf(v1, v2, Zm, levels=40, cmap="viridis")
                self.plot.fig.colorbar(cf, ax=ax, label=f"{zname} [{unit}]")
            if len(levels):
                cs = ax.contour(v1, v2, Zm, levels=levels, colors="k", linewidths=1.3)
                ax.clabel(cs, fmt=lambda v: f"{v:g} {unit}", fontsize=8)
            X, Y = np.meshgrid(v1, v2)
            ax.plot(X.ravel(), Y.ravel(), ".", color="0.6", ms=2)
            if min(v1) <= b1 <= max(v1) and min(v2) <= b2 <= max(v2):
                ax.plot(b1, b2, "*", color="tab:red", ms=14, mec="k", label="baseline (Vehicle tab)")
                ax.legend(fontsize=8, loc="upper right")
            ax.set_xlabel(f"{p1} [{u1}]")
            ax.set_ylabel(f"{p2} [{u2}]")
            ax.set_title(f"Lines of constant {zname.lower()} – {R['track']}")
            lines.append(f"Lines drawn at: {', '.join(f'{v:g}' for v in levels) or 'none in range'} {unit}")
            to = tradeoff(v1, v2, Z, b1, b2) if fin.all() else None
            if to:
                lines += ["", f"At the baseline ({p1} = {b1:g}, {p2} = {b2:g}):",
                          f"  {zname} ≈ {to['z']:.3f} {unit}",
                          f"  d/d{p1} = {to['dZdx']:+.4g} {unit} per {u1}",
                          f"  d/d{p2} = {to['dZdy']:+.4g} {unit} per {u2}",
                          f"  Trade-off along the constant-{'lap-time' if lap else 'energy'} line:",
                          f"  +1 {u1} of {p1} ⇔ {to['slope']:+.4g} {u2} of {p2}"]
        ax.grid(alpha=0.3)
        self.plot.draw()
        set_text(self.summary, "\n".join(lines))

    def export(self):
        R = self.res
        if R is None:
            messagebox.showinfo("Export", "Run a sweep first.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".mat",
                                            filetypes=[("MATLAB", "*.mat"), ("CSV", "*.csv")])
        if not path:
            return
        if path.lower().endswith(".csv"):
            with open(path, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow([R["p1"]] + ([R["p2"]] if R["p2"] else []) + ["lap_time_s", "energy_Wh"])
                if R["p2"]:
                    for j, b in enumerate(R["vals2"]):
                        for i, a in enumerate(R["vals1"]):
                            w.writerow([a, b, R["T"][j, i], R["E"][j, i]])
                else:
                    for i, a in enumerate(R["vals1"]):
                        w.writerow([a, R["T"][i], R["E"][i]])
        else:
            from scipy.io import savemat
            savemat(path, {"param1": R["p1"], "values1": R["vals1"], "param2": R["p2"] or "",
                           "values2": R["vals2"] if R["p2"] else np.array([]),
                           "lap_time_s": R["T"], "energy_Wh": R["E"], "track": R["track"],
                           "standing_start": int(R["standing"]),
                           "baseline": {k: (float(v) if not isinstance(v, bool) else int(v))
                                        for k, v in R["base"].items()}})
        messagebox.showinfo("Export", f"Saved {path}")


# ====================================================================== app
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("FSAE 4-Corner State-Space Vehicle Model & Lap Time Simulator")
        self.geometry("1500x900")
        try:
            ttk.Style(self).theme_use("vista")
        except tk.TclError:
            pass
        nb = ttk.Notebook(self)
        nb.pack(fill=tk.BOTH, expand=True)
        self.datasets: dict = {}
        self.vehicle_tab = VehicleTab(nb, self)
        self.track_tab = TrackTab(nb, self)
        self.model_tab = ModelTab(nb, self)
        self.ggv_tab = GGVTab(nb, self)
        self.lap_tab = LapTab(nb, self)
        self.sweep_tab = SweepTab(nb, self)
        self.channel_tab = ChannelTab(nb, self)
        nb.add(self.vehicle_tab, text="  Vehicle parameters  ")
        nb.add(self.track_tab, text="  Track (radii & lengths)  ")
        nb.add(self.model_tab, text="  State-space model (open loop / linearise)  ")
        nb.add(self.ggv_tab, text="  GGV & tyre  ")
        nb.add(self.lap_tab, text="  Lap simulation  ")
        nb.add(self.sweep_tab, text="  Parameter sweep  ")
        nb.add(self.channel_tab, text="  Channel plot  ")
        self.nb = nb

    def unique_name(self, base: str) -> str:
        name, k = base, 2
        while name in self.datasets:
            name = f"{base} ({k})"
            k += 1
        return name

    def publish_sim(self, qss: dict, tr: dict | None) -> None:
        """Expose the latest lap-simulation channels as 'Sim QSS' / 'Sim Transient' datasets."""
        self.datasets["Sim QSS"] = from_qss(qss, "Sim QSS")
        if tr is not None:
            self.datasets["Sim Transient"] = from_transient(tr, "Sim Transient")
        else:
            self.datasets.pop("Sim Transient", None)
        self.channel_tab.refresh_datasets()

    def vehicle(self, silent=False) -> Vehicle | None:
        try:
            return Vehicle(self.vehicle_tab.get_params())
        except ValueError as e:
            if not silent:
                messagebox.showerror("Vehicle parameters", str(e))
            return None


EQUATIONS = r"""
4-CORNER PLANAR VEHICLE MODEL – STATE-SPACE FORM   x_dot = f(x, u, kappa_track(s))

States x (15)                                    Inputs u (7)
  vx, vy   body velocities          [m/s]          delta_FL, delta_FR  front steer angles    [rad]
  r        yaw rate                 [rad/s]        Tm_FL..Tm_RR        motor shaft torques   [Nm]
  w_i      wheel spin speeds (x4)   [rad/s]        T_brake             total friction brake  [Nm]
  Fz_i     normal loads (x4)        [N]
  s, n, xi path coordinates: distance, lateral offset, heading error
  E        battery energy used      [J]

Body (x fwd, y left, yaw CCW+), wheel i at (x_i, y_i):
  m (vx_dot - r vy) = sum Fx_i - 1/2 rho CdA vx^2
  m (vy_dot + r vx) = sum Fy_i
  Iz r_dot          = sum (x_i Fy_i - y_i Fx_i)

Wheel spin (one motor per wheel, gear ratio G, efficiency eta_g):
  Iw_eff,i w_i_dot = G eta_g Tm_i - bias_i T_brake sgn(w_i) - Fxw_i R - Crr Fz_i R sgn(w_i)
  Iw_eff = I_wheel + G^2 I_rotor (driven corners)
  Tm_i is saturated by the motor curve (T_max, P_max, rpm_max) and the accumulator power limit.

Tyre kinematics (wheel frame, steer delta_i):
  v_l, v_t = wheel-centre velocity [vx - r y_i, vy + r x_i] rotated by -delta_i
  kappa_i = (w_i R - v_l) / max(|v_l|, v_min)      alpha_i = -atan(v_t / max(|v_l|, v_min))

Tyre forces (normalised combined-slip Magic Formula, load sensitive friction):
  mu(Fz) = mu0 (1 + k_ls (Fz - Fz0)/Fz0)
  s_x = kappa/kappa_pk,  s_y = alpha/alpha_pk,  rho = sqrt(s_x^2 + s_y^2)
  Fxw = mu_x Fz sin(Cx atan(Bx rho)) s_x/rho ,  Fyw = mu_y Fz sin(Cy atan(By rho)) s_y/rho
  (B chosen so the peak sits at rho = 1 -> friction ellipse at the limit)

Normal loads (first-order suspension lag towards quasi-static value):
  tau Fz_i_dot = Fz_ss,i(vx, ax, ay) - Fz_i
  Fz_ss = static + aero downforce (balance) -/+ (m ax h + D h_cp)/(2L) -/+ LLTD * m ay h / track

Path coordinates relative to the track centre line (curvature kappa(s) from the segment list):
  s_dot  = (vx cos xi - vy sin xi) / (1 - n kappa)
  n_dot  =  vx sin xi + vy cos xi
  xi_dot =  r - kappa s_dot
  E_dot  =  sum P_elec,i   (P_elec = T w_m / eta_m when driving, T w_m eta_m when regenerating)

Outputs y (linearisation): ax, ay, beta, kappa_i, alpha_i

LAP SIMULATION
  1) QSS: a GGV envelope is built from the same 4-corner load transfer, tyre and powertrain models;
     apex speeds from ay_max(v), then forward (traction/power) and backward (braking) integration.
     Closed tracks are solved as a periodic flying lap.
  2) Transient: the full nonlinear model above is integrated (RK4, step limited for the stiff wheel
     modes) with a closed-loop driver:
       - steering: pure pursuit on the centre line + yaw-rate feedback, Ackermann split to delta_FL/FR
       - speed: tracks the QSS speed profile (scaled) with feed-forward acceleration
       - allocation: tractive/braking force split in proportion to measured Fz, plus a torque-vectoring
         yaw moment, regen first then friction brakes (fixed bias), slip-ratio traction control.
"""


def main():
    App().mainloop()


if __name__ == "__main__":
    main()
