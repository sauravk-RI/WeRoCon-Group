#!/usr/bin/env python3
"""
bode_noise_overlay.py -- superimpose the Bode results of several noise runs
(typically the same joint at different amplitudes: --amp 5, 10, 15) in ONE
magnitude plot and ONE phase plot.

usage:
    python bode_noise_overlay.py <run folder> <run folder> ... [--out FILE.png] [--show]
e.g.
    python bode_noise_overlay.py results/bode_noise_ankle/run_1 results/bode_noise_ankle/run_2 results/bode_noise_ankle/run_3

Each folder must contain bode_noise_summary.csv (written at the end of every
noise run). One colour per run (ordered by amplitude); the line and its markers
have the same colour; the legend gives the amplitude and that curve's -3 dB
frequency, which is also marked on both panels. The figure is saved next to
the first run folder as bode_overlay.png unless --out is given.
Needs bode_noise_tools.py in the same folder.
"""
import argparse
import math
import os

import numpy as np
import pandas as pd

import matplotlib
import bode_noise_tools as bnt


def load(run_dir):
    path = os.path.join(run_dir, bnt.SUMMARY_CSV_NAME)
    meta = dict(bnt.read_header(path))
    s = pd.read_csv(path, comment="#")
    if len(s) == 0:
        raise SystemExit(f"{path} has no frequency rows (the run had no complete trial).")
    return meta, s


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("run_dirs", nargs="+")
    ap.add_argument("--out", default=None, help="output PNG (default: bode_overlay.png next to the first run folder)")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    runs = [load(d) for d in args.run_dirs]
    runs.sort(key=lambda r: float(r[0].get("noise_peak_deg", "nan")))
    joints = {r[0].get("joint", "?") for r in runs}
    cmap = plt.get_cmap("tab10")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 8), sharex=True)
    f_lo, f_hi = [], []
    for i, (meta, s) in enumerate(runs):
        col = cmap(i % 10)
        f = s["f_hz"].to_numpy()
        f_lo.append(f.min())
        f_hi.append(f.max())
        targets = np.logspace(np.log10(f.min()), np.log10(f.max()), 50)
        mk = np.unique(np.abs(np.log(f)[None, :] - np.log(targets)[:, None]).argmin(axis=1))
        fc = bnt.first_down_crossing(f, s["magnitude_dB"], -3.0)
        ph = bnt.interp_log(fc, f, s["phase_deg"]) if not math.isnan(fc) else float("nan")
        amp = float(meta.get("noise_peak_deg", "nan"))
        label = f"{amp:g} deg" + ("" if math.isnan(fc) else f"  (f_c = {fc:.2f} Hz, phase {ph:.0f} deg)")
        for ax, c in ((ax1, "magnitude_dB"), (ax2, "phase_deg")):
            ax.plot(f, s[c], "-", color=col, lw=1.2, zorder=2)
            ax.plot(f[mk], s[c].to_numpy()[mk], "o", color=col, ms=4, zorder=3, label=label if c == "magnitude_dB" else None)
        if not math.isnan(fc):
            for ax in (ax1, ax2):
                ax.axvline(fc, color=col, ls="--", lw=0.8)
            ax1.plot([fc], [-3.0], "o", color=col, ms=8, mec="black", zorder=5)
            if not math.isnan(ph):
                ax2.plot([fc], [ph], "o", color=col, ms=8, mec="black", zorder=5)
    ax1.axhline(-3.0, color="0.4", ls="--", lw=0.9)
    ax2.axhline(-180.0, color="0.4", ls="--", lw=0.9)
    bnt._style_freq_axis(ax2, min(f_lo), max(f_hi))
    ax1.set_ylabel("Magnitude [dB]")
    ax2.set_ylabel("Phase [deg]")
    ax2.set_xlabel("Frequency [Hz]")
    ax1.legend(loc="lower left", fontsize=8)
    m0 = runs[0][0]
    ax1.set_title(f"{', '.join(sorted(joints)).capitalize()} joint | PID Kp={m0.get('kp')}, Ki={m0.get('ki')}, "
                  f"Kd={m0.get('kd')} | noise LP {m0.get('ref_lp_hz')} Hz", fontsize=10)
    fig.tight_layout()
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(args.run_dirs[0])), "bode_overlay.png")
    fig.savefig(out, dpi=bnt.FIG_DPI, bbox_inches="tight")
    print(f"saved {out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
