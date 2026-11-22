"""Run folders, CSV files, sim/hardware pairing, metrics and the static comparison plot.

Every test writes ONE new folder:  results/<gait>/run_<N>_<YYYYmmdd-HHMMSS>/

    run_info.json              settings, model/engine metadata, link + timing, summary
    reference.csv              the reference both systems tracked, on the 200 Hz grid
    simulation.csv             simulation, one row per 5 ms control tick (both joints)
    simulation_full_rate.csv   simulation, one row per 0.5 ms physics step
    hardware.csv               the leg's own control ticks, streamed live (sim-to-real)
    hardware_raw.jsonl         every message received from the leg, untouched
    comparison.csv             each hardware encoder sample paired with the simulated
                               angle at the same instant (sim-to-real)
    summary.json               RMS errors etc.
    comparison.png             static overlay plot

ALL ANGLES are in the "sim frame": knee +flexion, ankle +plantarflexion -- the same frame
the hardware scripts use in their own CSV files. TIME is seconds since the common start
instant (the GO instant); hardware times are already shifted onto that axis.
"""
from __future__ import annotations

import csv
import json
import math
import re
from datetime import datetime

import numpy as np

from .frames import JOINTS, SIM_FRAME_SIGN
from .paths import RESULTS_DIR


def make_run_dir(gait):
    root = RESULTS_DIR / gait
    root.mkdir(parents=True, exist_ok=True)
    numbers = [int(m.group(1)) for p in root.iterdir()
               if (m := re.match(r"run_(\d+)", p.name))]
    n = max(numbers, default=0) + 1
    path = root / f"run_{n}_{datetime.now().strftime('%Y%m%d-%H%M%S')}"
    path.mkdir()
    return path


def write_csv(path, rows, header=None):
    if not rows:
        return
    header = header or list(rows[0])
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=header, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow({k: _fmt(r.get(k)) for k in header})


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return [{k: _parse(v) for k, v in r.items()} for r in csv.DictReader(f)]


def _fmt(v):
    if isinstance(v, float):
        return "" if not math.isfinite(v) else f"{v:.9g}"
    return "" if v is None else v


def _parse(v):
    if v in ("", None):
        return None
    if v in ("True", "False"):
        return v == "True"
    try:
        return float(v)
    except ValueError:
        return v


def write_json(path, obj):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, default=_json_default)
        f.write("\n")


def _json_default(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, tuple):
        return list(o)
    return str(o)


# ----------------------------------------------------------------------------- hardware
def hardware_rows(samples, time_shift_s=0.0):
    """Leg samples (encoder frame, one per joint per tick) -> one row per tick (sim frame)."""
    ticks = {}
    for s in samples:
        j = s.get("joint")
        if j not in JOINTS:
            continue
        sign = SIM_FRAME_SIGN[j]
        row = ticks.setdefault(int(s["tick"]), {"tick": int(s["tick"]),
                                                "time_s": s["t_s"] + time_shift_s})
        row[f"{j}_encoder_time_s"] = s["t_enc_s"] + time_shift_s
        row[f"{j}_dt_s"] = s["dt_s"]
        row[f"{j}_reference_deg"] = sign * s["reference_deg"]
        row[f"{j}_actual_deg"] = sign * s["actual_deg"]
        row[f"{j}_error_deg"] = sign * s["error_deg"]
        row[f"{j}_reference_vel_deg_s"] = sign * s["reference_vel_deg_s"]
        row[f"{j}_encoder_valid"] = s["encoder_valid"]
        for key in ("p_A", "i_A", "d_A", "current_request_A", "current_cmd_A"):
            row[f"{j}_{key}"] = s[key]
        row[f"{j}_clamped"] = s["clamped"]
        row[f"{j}_actual_encoder_frame_deg"] = s["actual_deg"]
    return [ticks[k] for k in sorted(ticks)]


HARDWARE_HEADER = ["tick", "time_s"] + [f"{j}_{c}" for j in JOINTS for c in (
    "encoder_time_s", "dt_s", "reference_deg", "actual_deg", "error_deg",
    "reference_vel_deg_s", "encoder_valid", "p_A", "i_A", "d_A", "current_request_A",
    "current_cmd_A", "clamped", "actual_encoder_frame_deg")]


# ----------------------------------------------------------------------------- pairing
def pair_with_simulation(hw_rows, sim_time, sim_angle):
    """Interpolate the simulated angle at every hardware encoder-read instant."""
    pairs = []
    for r in hw_rows:
        for j in JOINTS:
            if f"{j}_actual_deg" not in r:
                continue
            t = r[f"{j}_encoder_time_s"]
            if t < sim_time[0] or t > sim_time[-1]:
                continue
            sim = float(np.interp(t, sim_time, sim_angle[j]))
            valid = bool(r[f"{j}_encoder_valid"])
            hw = r[f"{j}_actual_deg"] if valid else None
            ref = r[f"{j}_reference_deg"]
            pairs.append(dict(joint=j, tick=r["tick"], time_s=t, reference_deg=ref,
                              hardware_deg=hw, simulation_deg=sim,
                              difference_deg=None if hw is None else sim - hw,
                              hardware_tracking_error_deg=None if hw is None else ref - hw,
                              sim_tracking_error_deg=ref - sim))
    return pairs


def _rms(x):
    return math.sqrt(sum(v * v for v in x) / len(x)) if x else None


def summarize(pairs, sim_ctrl_rows, sim_time, sim_angle):
    out = {}
    for j in JOINTS:
        res = {}
        ref = [r[f"{j}_reference_deg"] for r in sim_ctrl_rows]
        sim = [r[f"{j}_sim_deg"] for r in sim_ctrl_rows]
        res["simulation_tracking_rms_deg"] = _rms([a - b for a, b in zip(ref, sim)])
        res["simulation_tracking_peak_deg"] = max((abs(a - b) for a, b in zip(ref, sim)),
                                                  default=None)
        res["simulation_clipped_updates"] = sum(bool(r[f"{j}_clipped"]) for r in sim_ctrl_rows)
        p = [r for r in pairs if r["joint"] == j and r["difference_deg"] is not None]
        if p:
            res.update(
                paired_samples=len(p),
                sim_minus_hardware_rms_deg=_rms([r["difference_deg"] for r in p]),
                sim_minus_hardware_peak_deg=max(abs(r["difference_deg"]) for r in p),
                hardware_tracking_rms_deg=_rms([r["hardware_tracking_error_deg"] for r in p]),
                hardware_tracking_peak_deg=max(abs(r["hardware_tracking_error_deg"]) for r in p))
            t = np.array([r["time_s"] for r in p])
            hw = np.array([r["hardware_deg"] for r in p])
            use = (t >= t[0] + .2) & (t <= t[-1] - .2)
            if use.sum() > 20:
                # Reference-implementation exploratory lag fit: +/-200 ms, 5 ms steps, 200 ms edge trim
                lags = np.arange(-.2, .2001, .005)
                errs = [np.mean((np.interp(t[use] + lag, sim_time, sim_angle[j]) - hw[use]) ** 2)
                        for lag in lags]
                k = int(np.argmin(errs))
                res.update(best_fit_lag_ms=float(lags[k] * 1000), lag_fit_rms_deg=float(
                    math.sqrt(errs[k])), lag_note="positive = simulation later than hardware; "
                    "exploratory, NOT a network-latency measurement")
        out[j] = res
    return out


# ----------------------------------------------------------------------------- plot
def plot_comparison(path, sim_ctrl_rows, hw_rows, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    has_hw = bool(hw_rows)
    fig, axes = plt.subplots(2, 2 if has_hw else 1, figsize=(13 if has_hw else 9, 7),
                             squeeze=False, sharex="col")
    t = [r["time_s"] for r in sim_ctrl_rows]
    for row, j in enumerate(JOINTS):
        ax = axes[row, 0]
        ax.plot(t, [r[f"{j}_reference_deg"] for r in sim_ctrl_rows], "--", color="#777f88",
                lw=1.2, label="Reference")
        ax.plot(t, [r[f"{j}_sim_deg"] for r in sim_ctrl_rows], color="#087e82", lw=1.2,
                label="Simulation")
        if has_hw:
            th = [r[f"{j}_encoder_time_s"] for r in hw_rows if f"{j}_actual_deg" in r]
            ax.plot(th, [r[f"{j}_actual_deg"] for r in hw_rows if f"{j}_actual_deg" in r],
                    color="#d9822b", lw=1.1, label="Hardware")
            d = axes[row, 1]
            sim_t = np.array(t)
            sim_a = np.array([r[f"{j}_sim_deg"] for r in sim_ctrl_rows])
            hw = [(r[f"{j}_encoder_time_s"], r[f"{j}_actual_deg"]) for r in hw_rows
                  if f"{j}_actual_deg" in r]
            if hw:
                ht, ha = map(np.array, zip(*hw))
                d.plot(ht, np.interp(ht, sim_t, sim_a) - ha, color="#5b4b8a", lw=1)
            d.axhline(0, color="k", lw=.6, alpha=.5)
            d.set_ylabel(f"{j.capitalize()}: sim − hardware (deg)")
            d.grid(alpha=.3)
        ax.set_ylabel(f"{j.capitalize()} angle (deg)")
        ax.set_title(f"{j.capitalize()} ({'+flexion' if j == 'knee' else '+plantarflexion'})")
        ax.grid(alpha=.3)
        ax.legend(loc="best", fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("Time since start (s)")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
