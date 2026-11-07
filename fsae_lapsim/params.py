"""Vehicle parameter set (defaults represent a generic 4WD electric FSAE car)."""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, fields


@dataclass
class VehicleParams:
    # Mass, inertia and geometry
    m: float = 280.0               # total mass incl. driver [kg]
    Iz: float = 110.0              # yaw inertia [kg m^2]
    wheelbase: float = 1.53        # [m]
    wd_front: float = 0.47         # static front weight fraction [-]
    track_f: float = 1.22          # [m]
    track_r: float = 1.18          # [m]
    h_cg: float = 0.28             # [m]
    lltd_front: float = 0.50       # lateral load transfer distribution (front share) [-]
    tau_fz: float = 0.03           # load-transfer (suspension) time constant [s]
    # Aero
    rho: float = 1.20              # air density [kg/m^3]
    ClA: float = 3.2               # downforce coefficient * area [m^2]
    CdA: float = 1.3               # drag coefficient * area [m^2]
    aero_bal_front: float = 0.45   # front share of downforce [-]
    h_cp: float = 0.35             # height of drag centre of pressure [m]
    # Tyres (normalised combined-slip Magic Formula)
    R_w: float = 0.203             # loaded radius [m]
    I_w: float = 0.12              # wheel+tyre+hub spin inertia [kg m^2]
    mu_x: float = 1.55             # peak longitudinal friction at Fz0 [-]
    mu_y: float = 1.50             # peak lateral friction at Fz0 [-]
    Fz0: float = 700.0             # nominal load [N]
    load_sens: float = -0.10       # d(mu)/mu per dFz/Fz0 [-]
    kappa_peak: float = 0.10       # slip ratio at peak Fx [-]
    alpha_peak_deg: float = 8.0    # slip angle at peak Fy [deg]
    C_x: float = 1.65              # MF shape factor, longitudinal (1..2)
    C_y: float = 1.45              # MF shape factor, lateral (1..2)
    Crr: float = 0.015             # rolling resistance coefficient [-]
    v_slip_min: float = 4.0        # low-speed floor in slip denominators [m/s]
    # Powertrain (one motor per wheel)
    awd: bool = True               # front motors fitted (False = RWD, 2 motors)
    gear_ratio: float = 13.5       # motor:wheel reduction [-]
    eta_gear: float = 0.95         # gearbox efficiency [-]
    eta_motor: float = 0.92        # motor+inverter efficiency [-]
    T_motor_max: float = 21.0      # peak motor torque [Nm]
    P_motor_max_kW: float = 35.0   # peak power per motor [kW]
    rpm_max: float = 20000.0       # max motor speed [rpm]
    I_motor: float = 2.74e-4       # rotor inertia [kg m^2]
    P_total_max_kW: float = 80.0   # accumulator power limit (FSAE rule) [kW]
    regen: bool = True             # regenerative braking enabled
    regen_frac: float = 1.0        # regen torque limit as fraction of T_motor_max [-]
    # Brakes & steering
    T_brake_max: float = 2000.0    # max total friction brake torque (all wheels) [Nm]
    brake_bias_front: float = 0.65 # [-]
    delta_max_deg: float = 25.0    # max road-wheel steer angle [deg]
    ackermann: float = 0.5         # 0 = parallel, 1 = full Ackermann [-]

    def to_json(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(asdict(self), fh, indent=2)

    @classmethod
    def from_json(cls, path: str) -> "VehicleParams":
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
        names = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in data.items() if k in names})

    def validate(self) -> list[str]:
        errs = []
        if self.m <= 0 or self.Iz <= 0:
            errs.append("Mass and yaw inertia must be > 0")
        if not 0.05 < self.wd_front < 0.95:
            errs.append("Front weight fraction must be within 0.05..0.95")
        if not 1.0 < self.C_x < 2.0 or not 1.0 < self.C_y < 2.0:
            errs.append("Tyre shape factors C_x, C_y must be within (1, 2)")
        if self.kappa_peak <= 0 or self.alpha_peak_deg <= 0:
            errs.append("Peak slip values must be > 0")
        if self.tau_fz <= 0:
            errs.append("Load transfer time constant must be > 0")
        if self.R_w <= 0 or self.gear_ratio <= 0:
            errs.append("Wheel radius and gear ratio must be > 0")
        if self.v_slip_min <= 0:
            errs.append("Low-speed slip floor must be > 0")
        return errs


# (name, label, unit, group) - drives the GUI parameter editor
PARAM_META = [
    ("m", "Total mass", "kg", "Mass & Geometry"),
    ("Iz", "Yaw inertia", "kg m²", "Mass & Geometry"),
    ("wheelbase", "Wheelbase", "m", "Mass & Geometry"),
    ("wd_front", "Front weight fraction", "-", "Mass & Geometry"),
    ("track_f", "Front track", "m", "Mass & Geometry"),
    ("track_r", "Rear track", "m", "Mass & Geometry"),
    ("h_cg", "CG height", "m", "Mass & Geometry"),
    ("lltd_front", "LLTD front", "-", "Mass & Geometry"),
    ("tau_fz", "Load transfer time const.", "s", "Mass & Geometry"),
    ("rho", "Air density", "kg/m³", "Aero"),
    ("ClA", "Cl·A (downforce)", "m²", "Aero"),
    ("CdA", "Cd·A (drag)", "m²", "Aero"),
    ("aero_bal_front", "Aero balance front", "-", "Aero"),
    ("h_cp", "Drag CoP height", "m", "Aero"),
    ("R_w", "Tyre radius", "m", "Tyres"),
    ("I_w", "Wheel inertia", "kg m²", "Tyres"),
    ("mu_x", "Peak μx @Fz0", "-", "Tyres"),
    ("mu_y", "Peak μy @Fz0", "-", "Tyres"),
    ("Fz0", "Nominal load Fz0", "N", "Tyres"),
    ("load_sens", "Load sensitivity", "-", "Tyres"),
    ("kappa_peak", "Peak slip ratio", "-", "Tyres"),
    ("alpha_peak_deg", "Peak slip angle", "deg", "Tyres"),
    ("C_x", "Shape factor Cx", "-", "Tyres"),
    ("C_y", "Shape factor Cy", "-", "Tyres"),
    ("Crr", "Rolling resistance", "-", "Tyres"),
    ("v_slip_min", "Low-speed slip floor", "m/s", "Tyres"),
    ("awd", "AWD (front motors)", "bool", "Powertrain"),
    ("gear_ratio", "Gear ratio", "-", "Powertrain"),
    ("eta_gear", "Gear efficiency", "-", "Powertrain"),
    ("eta_motor", "Motor+inv. efficiency", "-", "Powertrain"),
    ("T_motor_max", "Motor peak torque", "Nm", "Powertrain"),
    ("P_motor_max_kW", "Motor peak power", "kW", "Powertrain"),
    ("rpm_max", "Motor max speed", "rpm", "Powertrain"),
    ("I_motor", "Rotor inertia", "kg m²", "Powertrain"),
    ("P_total_max_kW", "Accumulator power limit", "kW", "Powertrain"),
    ("regen", "Regen braking", "bool", "Powertrain"),
    ("regen_frac", "Regen torque fraction", "-", "Powertrain"),
    ("T_brake_max", "Max friction brake torque", "Nm", "Brakes & Steering"),
    ("brake_bias_front", "Brake bias front", "-", "Brakes & Steering"),
    ("delta_max_deg", "Max steer angle", "deg", "Brakes & Steering"),
    ("ackermann", "Ackermann factor", "-", "Brakes & Steering"),
]
