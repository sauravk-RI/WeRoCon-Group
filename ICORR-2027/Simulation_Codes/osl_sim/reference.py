"""Gait reference trajectories -- a faithful port of the hardware scripts' STEP 1/2/4.

The five hardware controllers in Hardware_Codes/ build their reference like this:

    STEP 1  read the dataset angle CSV and choose a stride window
              "heel_strike"  normal walk / ramp ascent / ramp descent:
                             heel strikes from <LEG>ForceY_Vertical > 20 N in the GRF CSV,
                             STRIDE_COUNT strides starting at heel strike STRIDE_START_INDEX
              "time_window"  stairs up / stairs down:
                             one stride between STRIDE_START_TIME_S and STRIDE_END_TIME_S,
                             made periodic with a linear ramp, optionally rotated to start
                             at the knee maximum, then tiled STRIDE_COUNT times
            -> relative angle (starts at 0) + absolute start angle, per joint
    sign    *_DATASET_TO_ENCODER_SIGN, then the *_TRAJECTORY_START_DEG override
    STEP 2  t_scaled = t * TIME_SCALE ; CubicSpline(t_scaled, rel, "not-a-knot")
    STEP 4  check_joint_limits(): shift start_deg to fit [LIMIT_MIN, LIMIT_MAX]
            (AUTO_FIT_TO_LIMITS) or abort; the reference is start_deg + spline(t)

This module reproduces that numerically (pandas is replaced by the csv module so the
simulation PC needs no pandas). ``tools/check_reference_parity.py`` executes the real
hardware scripts' code (with the motor/CAN/I2C libraries stubbed out) and confirms that
this port gives the same reference for all five gaits.

In sim-to-real mode the leg sends ITS OWN resolved knots/start angles in the handshake
(``GaitReference.from_dict``), so the simulation always tracks exactly what the leg
tracks even if a hardware setting was edited; the locally built reference is then only
used to print a comparison.

All angles here are in the HARDWARE ENCODER frame (knee flexion-negative). Use
``ref_sim()`` for the MuJoCo/sim frame.
"""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field

import numpy as np
from scipy.interpolate import CubicSpline

from .frames import JOINTS, SIM_FRAME_SIGN

GRF_THRESHOLD_N = 20.0


# ----------------------------------------------------------------------------- CSV
def read_csv_columns(path, columns):
    """Return {column: float array}; empty cells become NaN (like pandas.read_csv)."""
    with open(path, newline="", encoding="utf-8-sig") as f:
        reader = csv.reader(f)
        header = [h.strip() for h in next(reader)]
        missing = [c for c in columns if c not in header]
        if missing:
            raise ValueError(f"{path}: missing column(s) {missing}")
        idx = [header.index(c) for c in columns]
        out = {c: [] for c in columns}
        for row in reader:
            if not row:
                continue
            for c, i in zip(columns, idx):
                cell = row[i].strip() if i < len(row) else ""
                out[c].append(float(cell) if cell else math.nan)
    return {c: np.asarray(v, dtype=float) for c, v in out.items()}


def count_rows(path):
    with open(path, newline="", encoding="utf-8-sig") as f:
        return sum(1 for row in csv.reader(f) if row) - 1


# ----------------------------------------------------------------------------- STEP 1
def _find_heel_strikes_from_grf(grf_csv_path, angle_t, leg):
    n = count_rows(grf_csv_path)
    if n != len(angle_t):
        raise ValueError(f"{grf_csv_path} has {n} rows but the angle file has "
                         f"{len(angle_t)} rows -- they must share the same time base.")
    col = f"{leg.upper()}ForceY_Vertical"
    fz = read_csv_columns(grf_csv_path, [col])[col]
    if np.all(np.isnan(fz)):
        raise ValueError(f"{grf_csv_path}'s {col} column is entirely NaN -- no usable "
                         f"force data for LEG={leg!r}.")
    on_ground = np.nan_to_num(fz, nan=-1.0) > GRF_THRESHOLD_N
    return np.flatnonzero(np.diff(on_ground.astype(int)) == 1) + 1


def load_heel_strike_strides(angle_csv_path, grf_csv_path, leg, stride_start_index,
                             stride_count):
    """normal walk / ramp scripts' load_gait_cycle_both()."""
    cols = read_csv_columns(angle_csv_path, ["time", f"knee_angle_{leg}", f"ankle_angle_{leg}"])
    t = cols["time"]
    knee_raw, ankle_raw = cols[f"knee_angle_{leg}"], cols[f"ankle_angle_{leg}"]
    strikes = _find_heel_strikes_from_grf(grf_csv_path, t, leg)
    max_strides = len(strikes) - 1
    if (max_strides < 1 or stride_start_index < 0 or stride_count < 1
            or stride_start_index + stride_count > max_strides):
        raise ValueError(f"STRIDE_START_INDEX={stride_start_index}, STRIDE_COUNT="
                         f"{stride_count} out of range: at most {max_strides} strides.")
    i0 = int(strikes[stride_start_index])
    i1 = int(strikes[stride_start_index + stride_count])
    t_out = t[i0:i1 + 1] - t[i0]
    knee_rel = knee_raw[i0:i1 + 1] - knee_raw[i0]
    ankle_rel = ankle_raw[i0:i1 + 1] - ankle_raw[i0]
    info = f"{stride_count} stride(s) from heel strike #{stride_start_index} " \
           f"(t={t[i0]:.3f}..{t[i1]:.3f} s, {leg.upper()} leg, {max_strides} available)"
    return t_out, knee_rel, float(knee_raw[i0]), ankle_rel, float(ankle_raw[i0]), info


def load_time_window_stride(angle_csv_path, stride_start_time_s, stride_end_time_s,
                            stride_count, start_at_knee_max, leg="l"):
    """stairs scripts' load_gait_cycle_both()."""
    cols = read_csv_columns(angle_csv_path, ["time", f"knee_angle_{leg}", f"ankle_angle_{leg}"])
    t = cols["time"]
    knee_raw, ankle_raw = cols[f"knee_angle_{leg}"], cols[f"ankle_angle_{leg}"]
    half = 0.5 * float(np.median(np.diff(t)))
    if (stride_start_time_s < t[0] - half or stride_end_time_s > t[-1] + half
            or stride_end_time_s <= stride_start_time_s):
        raise ValueError(f"Stride window [{stride_start_time_s}, {stride_end_time_s}] s is "
                         f"not inside the clip [{t[0]:.3f}, {t[-1]:.3f}] s.")
    i0 = int(np.argmin(np.abs(t - stride_start_time_s)))
    i1 = int(np.argmin(np.abs(t - stride_end_time_s)))
    t_out = t[i0:i1 + 1] - t[i0]
    ramp = t_out / t_out[-1]
    knee_abs = knee_raw[i0:i1 + 1] - (knee_raw[i1] - knee_raw[i0]) * ramp
    ankle_abs = ankle_raw[i0:i1 + 1] - (ankle_raw[i1] - ankle_raw[i0]) * ramp
    if start_at_knee_max:
        m = int(np.argmax(knee_abs[:-1]))
        order = np.concatenate([np.arange(m, len(t_out) - 1), np.arange(0, m)])
        order = np.append(order, order[0])
        knee_abs, ankle_abs = knee_abs[order], ankle_abs[order]
    knee_start, ankle_start = float(knee_abs[0]), float(ankle_abs[0])
    knee_rel, ankle_rel = knee_abs - knee_abs[0], ankle_abs - ankle_abs[0]
    if stride_count > 1:
        T = t_out[-1]
        t_out = np.concatenate([t_out[:-1] + k * T for k in range(stride_count)]
                               + [[stride_count * T]])
        knee_rel = np.concatenate([knee_rel[:-1]] * stride_count + [knee_rel[-1:]])
        ankle_rel = np.concatenate([ankle_rel[:-1]] * stride_count + [ankle_rel[-1:]])
    info = f"stride t={t[i0]:.3f}..{t[i1]:.3f} s tiled x{stride_count}" \
           f"{', starting at knee max' if start_at_knee_max else ''}"
    return t_out, knee_rel, knee_start, ankle_rel, ankle_start, info


# ----------------------------------------------------------------------------- object
@dataclass
class GaitReference:
    """Reference trajectory for both joints, in the hardware encoder frame."""
    t_knots: np.ndarray                      # seconds, already multiplied by TIME_SCALE
    rel_deg: dict                            # joint -> relative angle at the knots
    start_deg: dict                          # joint -> absolute start (after override/fit)
    duration_s: float
    limits_deg: dict = field(default_factory=dict)   # joint -> (min, max), encoder frame
    source: str = ""
    notes: list = field(default_factory=list)

    def __post_init__(self):
        self.t_knots = np.asarray(self.t_knots, dtype=float)
        self.rel_deg = {j: np.asarray(v, dtype=float) for j, v in self.rel_deg.items()}
        self.pos_spline = {j: CubicSpline(self.t_knots, self.rel_deg[j], bc_type="not-a-knot")
                           for j in JOINTS}
        self.vel_spline = {j: self.pos_spline[j].derivative() for j in JOINTS}

    # hardware: reference_deg = joint.start_deg + float(joint.pos_spline(t_wall))
    def ref_enc(self, j, t):
        return self.start_deg[j] + float(self.pos_spline[j](t))

    def vel_enc(self, j, t):
        return float(self.vel_spline[j](t))

    def ref_sim(self, j, t):
        return SIM_FRAME_SIGN[j] * self.ref_enc(j, t)

    def range_enc(self, j):
        t_grid = np.linspace(0.0, self.duration_s, 2000)
        crit = self.vel_spline[j].roots(extrapolate=False)
        crit = crit[(crit >= 0.0) & (crit <= self.duration_s)]
        rel = self.pos_spline[j](np.union1d(t_grid, crit))
        return self.start_deg[j] + float(rel.min()), self.start_deg[j] + float(rel.max())

    # ---- serialisation (used by the sim<->leg handshake and the run folder) -------
    def to_dict(self):
        return dict(t_knots=self.t_knots.tolist(),
                    rel_deg={j: self.rel_deg[j].tolist() for j in JOINTS},
                    start_deg=dict(self.start_deg), duration_s=self.duration_s,
                    limits_deg={j: list(v) for j, v in self.limits_deg.items()},
                    source=self.source, notes=list(self.notes))

    @classmethod
    def from_dict(cls, d):
        return cls(np.asarray(d["t_knots"]), {j: d["rel_deg"][j] for j in JOINTS},
                   {j: float(d["start_deg"][j]) for j in JOINTS}, float(d["duration_s"]),
                   {j: tuple(v) for j, v in d.get("limits_deg", {}).items()},
                   d.get("source", ""), list(d.get("notes", [])))

    def max_difference_deg(self, other, n=4000):
        """Largest |difference| between two references over the common duration."""
        T = min(self.duration_s, other.duration_s)
        ts = np.linspace(0.0, T, n)
        diff = max(abs(self.ref_enc(j, t) - other.ref_enc(j, t)) for j in JOINTS for t in ts)
        return diff, abs(self.duration_s - other.duration_s)


# ----------------------------------------------------------------------------- STEP 4
def _fit_to_limits(ref, j, lo, hi, auto_fit, label):
    """hardware check_joint_limits(), ported."""
    abs_min, abs_max = ref.range_enc(j)
    span, limit_span = abs_max - abs_min, hi - lo
    if abs_min < lo or abs_max > hi:
        if not auto_fit:
            raise ValueError(f"{label} trajectory [{abs_min:+.1f}, {abs_max:+.1f}] deg is "
                             f"outside the limits [{lo:+.1f}, {hi:+.1f}] and "
                             f"AUTO_FIT_TO_LIMITS is False.")
        if span > limit_span:
            raise ValueError(f"{label} trajectory spans {span:.1f} deg, wider than the "
                             f"{limit_span:.1f} deg range [{lo:+.1f}, {hi:+.1f}].")
        shift = lo - abs_min if abs_min < lo else hi - abs_max
        ref.start_deg[j] += shift
        ref.notes.append(f"[auto-fit] {label}: reference shifted {shift:+.2f} deg to fit "
                         f"[{lo:+.1f}, {hi:+.1f}] deg")
        abs_min, abs_max = abs_min + shift, abs_max + shift
    ref.notes.append(f"[limit-check] {label}: range [{abs_min:+.1f}, {abs_max:+.1f}] deg "
                     f"inside [{lo:+.1f}, {hi:+.1f}] deg")


def build_reference(cfg):
    """Build the reference from a sim script's USER SETTINGS (a dict-like ``cfg``)."""
    g = cfg.get
    leg = g("LEG", "l")
    if g("STRIDE_SELECTION") == "time_window":
        t, kr, ks, ar, as_, info = load_time_window_stride(
            cfg["ANGLE_CSV_PATH"], cfg["STRIDE_START_TIME_S"], cfg["STRIDE_END_TIME_S"],
            cfg["STRIDE_COUNT"], bool(g("START_AT_KNEE_MAX", False)), leg)
    else:
        t, kr, ks, ar, as_, info = load_heel_strike_strides(
            cfg["ANGLE_CSV_PATH"], cfg["GRF_CSV_PATH"], leg, int(g("STRIDE_START_INDEX", 0)),
            int(cfg["STRIDE_COUNT"]))
    ksign, asign = cfg["KNEE_DATASET_TO_ENCODER_SIGN"], cfg["ANKLE_DATASET_TO_ENCODER_SIGN"]
    rel = {"knee": ksign * kr, "ankle": asign * ar}
    start = {"knee": ksign * ks, "ankle": asign * as_}
    notes = [info]
    for j, key in (("knee", "KNEE_TRAJECTORY_START_DEG"), ("ankle", "ANKLE_TRAJECTORY_START_DEG")):
        if g(key) is not None:
            notes.append(f"{j} start overridden {start[j]:+.2f} -> {float(g(key)):+.2f} deg")
            start[j] = float(g(key))
    scale = float(cfg["TIME_SCALE"])
    limits = {"knee": (cfg["KNEE_LIMIT_MIN_DEG"], cfg["KNEE_LIMIT_MAX_DEG"]),
              "ankle": (cfg["ANKLE_LIMIT_MIN_DEG"], cfg["ANKLE_LIMIT_MAX_DEG"])}
    ref = GaitReference(t * scale, rel, start, float(t[-1] * scale), limits,
                        source=str(cfg["ANGLE_CSV_PATH"]), notes=notes)
    for j, label in (("knee", "Knee"), ("ankle", "Ankle")):
        _fit_to_limits(ref, j, *limits[j], bool(g("AUTO_FIT_TO_LIMITS", True)), label)
    return ref
