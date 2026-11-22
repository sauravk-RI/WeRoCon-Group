#!/usr/bin/env python3
"""
bode_noise_tools.py -- reference generation, Blackman-Tukey analysis and
plotting for the OSL-style Gaussian-noise Bode test of the OSL v2 ankle and
knee (ankle_bode_noise_PID_control_me.py, knee_bode_noise_PID_control_me.py).

Self-contained: it does not import bode_sweep_tools.py. No hardware imports, so
it runs on any machine:

    python bode_noise_tools.py analyse <run folder> [--show]
        Rebuild bode_noise_summary.csv and the three figures from
        bode_noise_data.csv alone, also after an aborted run.

    python bode_noise_tools.py compare <noise run> <sweep run>
        Overlay the stepped-sine points of <sweep run>/bode_summary.csv (grey
        squares) on the noise Bode plot (fig3b_bode_vs_stepped_sine.png, written
        into the noise run folder) and print the difference in f_c and in the
        phase at f_c. The sweep folder is only read.

    python bode_noise_tools.py selftest [--out DIR] [--no-plots]
        Offline check against known plants (see the plan, Section 7).

THE TEST (Azocar et al. 2018, 2020): the reference is Gaussian white noise
through a causal 3rd-order low-pass, zero mean, scaled to a peak of +/-A deg
about the centre; 15 s record per trial, several trials; the closed-loop
T(f) = S_xy / S_xx is estimated by Blackman-Tukey spectral analysis with the
cross-spectra SUMMED over trials.

FILES OF A RUN (results/bode_noise_<joint>/<mode>_<N>/)
    bode_noise_data.csv     one row per control tick:
                            time_s, trial, reference_deg, actual_deg, in_window
                            [, current_cmd_A when LOG_CURRENT_A]
    bode_noise_summary.csv  one row per frequency bin from F_LO_HZ up to the low-pass corner:
                            f_hz, magnitude_dB, phase_deg (unwrapped, lag negative)
    fig1_ref_vs_actual_by_trial.png, fig2_ref_vs_actual_whole_run.png,
    fig3_bode.png
Both CSVs start with '# key: value' lines. Load with
    pandas:  pd.read_csv(path, comment="#")
    MATLAB:  T = readtable(path, 'CommentStyle', '#');   (not tested)

CONVENTIONS
  - Angles are in the controller's own (encoder) frame; x = reference - centre,
    y = angle - centre. Phase = angle(S_xy / S_xx), so a lagging output has a
    NEGATIVE phase, unwrapped over frequency.
  - A trial is analysed only if its whole record window was logged (time span
    >= rec_s minus 3 ticks). The loop runs at about 982-995 Hz, so the window
    holds slightly fewer than rec_s * 1000 rows; logged and angle are
    interpolated onto an exact 1 kHz grid using the logged time_s.
"""

import argparse
import math
import os
import sys
import tempfile

import numpy as np
import pandas as pd
from scipy import signal

# ===========================================================================
#  Definitions shared by the scripts and the analysis
# ===========================================================================
FS = 1000.0                       # analysis grid and reference rate [Hz]
CSV_COLUMNS = ("time_s", "trial", "reference_deg", "actual_deg", "in_window")
CURRENT_COLUMN = "current_cmd_A"
SUMMARY_COLUMNS = ("f_hz", "magnitude_dB", "phase_deg")
DATA_CSV_NAME = "bode_noise_data.csv"
SUMMARY_CSV_NAME = "bode_noise_summary.csv"
FIG_BY_TRIAL = "fig1_ref_vs_actual_by_trial.png"
FIG_WHOLE = "fig2_ref_vs_actual_whole_run.png"
FIG_BODE = "fig3_bode.png"
FIG_COMPARE = "fig3b_bode_vs_stepped_sine.png"
FIG_DPI = 200

# analysis defaults, used when a header does not carry the value
BT_MAX_LAG_S_DEFAULT = 4.0
F_LO_HZ_DEFAULT = 0.3
NFFT = 1 << 14


# ===========================================================================
#  Reference generation (plan, Appendix A)
# ===========================================================================

def make_reference(fs=1000.0, lp_hz=20.0, peak_deg=5.0, rec_s=15.0, fade_s=1.0,
                   warm_s=4.0, seed=0, order=3):
    """Offset-from-centre reference [deg]: fade-in | record | fade-out.
    Returns (r, i0, i1): len(r) = (fade + rec + fade) * fs, record window = r[i0:i1]."""
    rng = np.random.default_rng(seed)
    n_f, n_r, n_w = int(round(fade_s * fs)), int(round(rec_s * fs)), int(round(warm_s * fs))
    x = rng.standard_normal(n_w + 2 * n_f + n_r)
    sos = signal.butter(order, lp_hz, fs=fs, output="sos")        # causal, like the papers
    y = signal.sosfilt(sos, x)[n_w:]                              # drop filter start-up
    i0, i1 = n_f, n_f + n_r
    y = y - y[i0:i1].mean()                                       # zero mean in the record window
    y = y * (peak_deg / np.abs(y[i0:i1]).max())                   # peak of the record window = peak_deg
    w = np.ones_like(y)
    ramp = 0.5 * (1 - np.cos(np.pi * np.arange(n_f) / n_f))
    w[:n_f], w[-n_f:] = ramp, ramp[::-1]
    return y * w, i0, i1


def build_run_arrays(n_trials, lp_hz, peak_deg, seed_base, rec_s=15.0, fade_s=1.0,
                     hold_s=1.0, preroll_s=1.0, order=3, fs=FS):
    """Whole-run arrays on the nominal 1 kHz grid, indexed by int(t * fs):
    ref (offset from the centre), trial (0 = pre-roll), win (1 in a record
    window). Trial k uses seed seed_base + k. Layout: pre-roll | (fade-in,
    record, fade-out, hold) x n_trials."""
    n_pre, n_hold = int(round(preroll_s * fs)), int(round(hold_s * fs))
    refs, trials, wins = [np.zeros(n_pre)], [np.zeros(n_pre, np.int8)], [np.zeros(n_pre, np.int8)]
    for k in range(1, n_trials + 1):
        r, i0, i1 = make_reference(fs, lp_hz, peak_deg, rec_s, fade_s, seed=seed_base + k, order=order)
        w = np.zeros(len(r) + n_hold, np.int8)
        w[i0:i1] = 1
        refs += [r, np.zeros(n_hold)]
        trials.append(np.full(len(r) + n_hold, k, np.int8))
        wins.append(w)
    return np.concatenate(refs), np.concatenate(trials), np.concatenate(wins)


def predicted_peak_current(r, kp, ki, kd, integral_limit, fs=FS):
    """Upper-bound estimate of the peak PID current: the PID law applied to
    e = r (backward-Euler D, integral clamp as in the PID class)."""
    dt = 1.0 / fs
    integ, prev, peak = 0.0, None, 0.0
    for e in r:
        integ = max(-integral_limit, min(integral_limit, integ + e * dt))
        d = 0.0 if prev is None else (e - prev) / dt
        prev = e
        peak = max(peak, abs(kp * e + ki * integ + kd * d))
    return peak


# ===========================================================================
#  Blackman-Tukey spectra (plan, Appendix A)
# ===========================================================================

def _xcorr(a, b, M):
    N = len(a)
    L = 1 << int(np.ceil(np.log2(2 * N)))
    c = np.fft.irfft(np.conj(np.fft.rfft(a, L)) * np.fft.rfft(b, L), L) / N   # c[m] = (1/N) sum a[k] b[k+m]
    return c[np.arange(-M, M + 1) % L]


def bt_spectra(trials, fs=1000.0, max_lag_s=4.0, nfft=1 << 14):
    """Blackman-Tukey spectra summed over trials; trials = [(x, y), ...] on a uniform grid.
    Returns f, H (= S_xy / S_xx, lag gives negative phase) and the (unused) coherence."""
    M = int(round(max_lag_s * fs))
    w = np.hanning(2 * M + 1)
    Sxx = Syy = Sxy = 0.0
    for x, y in trials:
        x = x - x.mean()
        y = y - y.mean()
        out = []
        for a, b in ((x, x), (y, y), (x, y)):
            R = _xcorr(a, b, M) * w
            z = np.zeros(nfft)
            z[:M + 1] = R[M:]
            z[-M:] = R[:M]
            out.append(np.fft.rfft(z))
        Sxx = Sxx + out[0]
        Syy = Syy + out[1]
        Sxy = Sxy + out[2]
    f = np.fft.rfftfreq(nfft, 1 / fs)
    H = Sxy / Sxx
    coh = np.abs(Sxy) ** 2 / (Sxx.real * Syy.real)
    return f, H, coh


# ===========================================================================
#  Small helpers (duplicated on purpose; see the plan, ground rule 1)
# ===========================================================================

def first_down_crossing(f, y, level):
    """First downward crossing of `level`, linear in log10(f), or NaN."""
    f = np.asarray(f, float)
    y = np.asarray(y, float)
    if len(f) < 2 or y[0] < level:
        return float("nan")
    for i in range(len(f) - 1):
        if y[i] >= level > y[i + 1]:
            x = (level - y[i]) / (y[i + 1] - y[i])
            return float(10 ** (math.log10(f[i]) + x * (math.log10(f[i + 1]) - math.log10(f[i]))))
    return float("nan")


def interp_log(f_query, f, y):
    f = np.asarray(f, float)
    if len(f) < 2 or f_query < f[0] * (1 - 1e-9) or f_query > f[-1] * (1 + 1e-9):
        return float("nan")
    return float(np.interp(math.log10(f_query), np.log10(f), y))


def _one_line(v):
    return " ".join(str(v).split())


def write_csv(path, header, df, float_format="%.9g"):
    """header: list of (key, value) written as '# key: value' lines, then the table."""
    with open(path, "w", newline="") as f:
        for k, v in header:
            f.write(f"# {k}: {_one_line(v)}\n")
        df.to_csv(f, index=False, float_format=float_format, lineterminator="\n")


def read_header(path):
    out = []
    with open(path) as f:
        for line in f:
            if not line.startswith("#"):
                break
            k, _, v = line[1:].strip().partition(":")
            out.append((k.strip(), v.strip()))
    return out


def read_data(path):
    return pd.read_csv(path, comment="#")


def _num(meta, key, default=float("nan")):
    try:
        return float(meta[key])
    except (KeyError, ValueError, TypeError):
        return default


# ===========================================================================
#  Analysis (data CSV only)
# ===========================================================================

def find_trials(df, meta):
    """[{k, lo, hi, complete}] for trials k >= 1 present in the data. `lo:hi`
    are the row positions of the record window (in_window == 1)."""
    out = []
    if len(df) == 0:
        return out
    t = df["time_s"].to_numpy(float)
    trial = df["trial"].to_numpy(int)
    win = df["in_window"].to_numpy(int) == 1
    rec_s = _num(meta, "rec_s", 15.0)
    tick = float(np.median(np.diff(t))) if len(t) > 2 else 1e-3
    for k in sorted(set(trial[trial >= 1])):
        idx = np.nonzero((trial == k) & win)[0]
        d = {"k": int(k), "lo": None, "hi": None, "complete": False}
        if len(idx):
            d["lo"], d["hi"] = int(idx[0]), int(idx[-1]) + 1
            d["complete"] = bool(t[idx[-1]] - t[idx[0]] >= rec_s - 3.0 * tick)
        out.append(d)
    return out


def _grid_trial(t, ref, act, centre, lo, hi, rec_s):
    """Interpolate one record window onto the exact 1 kHz grid."""
    t0 = t[lo]
    n = min(int(round(rec_s * FS)), int(math.floor((t[hi - 1] - t0) * FS)) + 1)
    tg = t0 + np.arange(n) / FS
    x = np.interp(tg, t[lo:hi], ref[lo:hi]) - centre
    y = np.interp(tg, t[lo:hi], act[lo:hi]) - centre
    return x, y


def analyse_data(df, meta):
    """dict(summary, trials, f, H, f_c, phase_at_fc, n_complete)."""
    trials = find_trials(df, meta)
    done = [d for d in trials if d["complete"]]
    res = {"trials": trials, "n_complete": len(done), "f_c": float("nan"),
           "phase_at_fc": float("nan"), "f": None, "H": None,
           "summary": pd.DataFrame(columns=SUMMARY_COLUMNS)}
    if not done:
        return res
    t = df["time_s"].to_numpy(float)
    ref = df["reference_deg"].to_numpy(float)
    act = df["actual_deg"].to_numpy(float)
    centre = _num(meta, "center_deg", 0.0)
    rec_s = _num(meta, "rec_s", 15.0)
    pairs = [_grid_trial(t, ref, act, centre, d["lo"], d["hi"], rec_s) for d in done]
    f, H, coh = bt_spectra(pairs, FS, _num(meta, "bt_max_lag_s", BT_MAX_LAG_S_DEFAULT), NFFT)
    res.update(f=f, H=H)
    # Every bin from F_LO_HZ up to the reference low-pass corner (above it there is no input power).
    hi = max(_num(meta, "ref_lp_hz", 20.0), 2.0)
    band = (f >= _num(meta, "f_lo_hz", F_LO_HZ_DEFAULT)) & (f <= hi)
    mag = 20.0 * np.log10(np.abs(H[band]))
    ph = np.degrees(np.unwrap(np.angle(H[band])))
    res["summary"] = pd.DataFrame({"f_hz": f[band], "magnitude_dB": mag, "phase_deg": ph}, columns=SUMMARY_COLUMNS)
    fc = first_down_crossing(f[band], mag, -3.0)
    res["f_c"] = fc
    if not math.isnan(fc):
        res["phase_at_fc"] = interp_log(fc, f[band], ph)
    return res


# ===========================================================================
#  Plots
# ===========================================================================
_state = {"gui_ok": True}
C_REF, C_ACT, C_INK, C_GRID = "tab:blue", "tab:red", "0.2", 0.3


def _plt():
    import matplotlib.pyplot as plt
    return plt


def _subplots(*a, **k):
    """plt.subplots, falling back to Agg if the GUI backend fails."""
    plt = _plt()
    try:
        return plt.subplots(*a, **k)
    except Exception as e:
        print(f"[analyse] matplotlib GUI backend failed ({type(e).__name__}: {e}); "
              f"falling back to the Agg backend -- figures are saved but not shown.")
        _state["gui_ok"] = False
        plt.switch_backend("Agg")
        return plt.subplots(*a, **k)


def _title_line(meta):
    loop = _num(meta, "loop_hz_achieved")
    if math.isnan(loop):
        loop = _num(meta, "loop_hz_nominal")
    loop_s = "" if math.isnan(loop) else f" | loop {loop:.0f} Hz"
    return (f"{str(meta.get('joint', '?')).capitalize()} joint | PID Kp={meta.get('kp')}, Ki={meta.get('ki')}, "
            f"Kd={meta.get('kd')} | noise LP {_num(meta, 'ref_lp_hz'):g} Hz, peak +/-{_num(meta, 'noise_peak_deg'):g} deg"
            f"{loop_s}")


def _style_freq_axis(ax, f_lo, f_hi):
    """Log frequency axis, 1-2-5 ticks, no minor ticks."""
    from matplotlib.ticker import FixedLocator, FormatStrFormatter, NullLocator
    ax.set_xscale("log")
    ticks = [m * 10.0 ** e for e in range(-3, 4) for m in (1, 2, 5) if f_lo / 1.001 <= m * 10.0 ** e <= f_hi * 1.001]
    ax.xaxis.set_major_locator(FixedLocator(ticks))
    ax.xaxis.set_major_formatter(FormatStrFormatter("%g"))
    ax.xaxis.set_minor_locator(NullLocator())
    ax.set_xlim(f_lo / 1.15, f_hi * 1.15)
    ax.grid(True, which="major", alpha=C_GRID)


def _mark_cutoff(ax_mag, ax_ph, fc, ph, f_hi):
    if math.isnan(fc):
        return
    for ax in (ax_mag, ax_ph):
        ax.axvline(fc, color=C_INK, ls="--", lw=0.9)
    ax_mag.plot([fc], [-3.0], "o", color=C_INK, ms=7, zorder=5)
    if not math.isnan(ph):
        ax_ph.plot([fc], [ph], "o", color=C_INK, ms=7, zorder=5)
    box = dict(fc="white", ec="0.6", alpha=0.9)
    ax_mag.annotate(f"f_c = {fc:.2f} Hz\nmagnitude -3.0 dB\nphase {ph:.0f} deg", (fc, -3.0), xytext=(10, 10),
                    textcoords="offset points", fontsize=9, color=C_INK, bbox=box)
    if not math.isnan(ph):
        ax_ph.annotate(f"f_c = {fc:.2f} Hz\nphase {ph:.0f} deg", (fc, ph), xytext=(10, -35),
                       textcoords="offset points", fontsize=9, color=C_INK, bbox=box)


def plot_by_trial(df, meta, res, out_path):
    done = [d for d in res["trials"] if d["complete"]]
    if not done:
        return None
    n = len(done)
    ncols = min(3, n)
    nrows = math.ceil(n / ncols)
    fig, axes = _subplots(nrows, ncols, figsize=(5 * ncols, 2.8 * nrows), squeeze=False)
    t = df["time_s"].to_numpy(float)
    c = _num(meta, "center_deg", 0.0)
    ref = df["reference_deg"].to_numpy(float) - c
    act = df["actual_deg"].to_numpy(float) - c
    ylim = 1.25 * _num(meta, "noise_peak_deg", 5.0)
    for ax in axes.ravel():
        ax.set_visible(False)
    for ax, d in zip(axes.ravel(), done):
        ax.set_visible(True)
        s = slice(d["lo"], d["hi"])
        x = t[s] - t[d["lo"]]
        ax.plot(x, ref[s], color=C_REF, ls="--", lw=0.8, label="reference")
        ax.plot(x, act[s], color=C_ACT, lw=0.8, label="actual")
        ax.axhline(0, color="0.6", lw=0.5)
        ax.set_ylim(-ylim, ylim)
        ax.set_xlim(0, x[-1])
        rms = float(np.sqrt(np.mean((ref[s] - act[s]) ** 2)))
        ax.set_title(f"trial {d['k']} | rms error {rms:.2f} deg", fontsize=9)
        ax.grid(True, alpha=C_GRID)
        ax.tick_params(labelsize=7)
    for ax in axes[-1]:
        ax.set_xlabel("time in record window [s]", fontsize=8)
    for ax in axes[:, 0]:
        ax.set_ylabel("angle - centre [deg]", fontsize=8)
    axes[0, 0].legend(fontsize=7, loc="upper right")
    fig.suptitle(_title_line(meta) + "\nrecord window of each trial", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out_path, dpi=FIG_DPI, bbox_inches="tight")
    return fig


def plot_whole_run(df, meta, res, out_path):
    if len(df) == 0:
        return None
    t = df["time_s"].to_numpy(float)
    fig, ax = _subplots(1, 1, figsize=(16, 5))
    ax.plot(t, df["reference_deg"], color=C_REF, ls="--", lw=0.6, label="reference", rasterized=True)
    ax.plot(t, df["actual_deg"], color=C_ACT, lw=0.6, label="actual", rasterized=True)
    trial = df["trial"].to_numpy(int)
    trans = ax.get_xaxis_transform()
    for k in sorted(set(trial[trial >= 1])):
        idx = np.nonzero(trial == k)[0]
        t0, t1 = t[idx[0]], t[idx[-1]]
        if k % 2 == 1:
            ax.axvspan(t0, t1, color="0.5", alpha=0.12, lw=0)
        ax.text(0.5 * (t0 + t1), 0.99, f"T{k}", transform=trans, fontsize=8, ha="center", va="top", color=C_INK)
    c, a = _num(meta, "center_deg", 0.0), _num(meta, "noise_peak_deg", 5.0)
    ax.set_ylim(c - 1.5 * a, c + 1.5 * a)
    ax.set_xlim(t[0], max(t[-1], t[0] + 1e-3))
    ax.set_xlabel("time [s]")
    ax.set_ylabel("joint angle [deg]")
    ax.set_title(_title_line(meta) + "\ntrials shaded alternately, labelled T1...TN", fontsize=10)
    ax.legend(loc="lower right", fontsize=8)
    ax.grid(True, alpha=C_GRID)
    fig.tight_layout()
    fig.savefig(out_path, dpi=FIG_DPI, bbox_inches="tight")
    return fig


def plot_bode(res, meta, out_path, sweep=None):
    s = res["summary"]
    if len(s) == 0:
        return None
    fig, (ax1, ax2) = _subplots(2, 1, figsize=(9, 8), sharex=True)
    amp = _num(meta, "noise_peak_deg", float("nan"))
    # green dots at ~50 log-spaced frequencies (the bins are dense at high f); the blue line has every bin
    targets = np.logspace(np.log10(s["f_hz"].min()), np.log10(s["f_hz"].max()), 50)
    mk = np.unique(np.abs(np.log(s["f_hz"].to_numpy())[None, :] - np.log(targets)[:, None]).argmin(axis=1))
    for ax, col in ((ax1, "magnitude_dB"), (ax2, "phase_deg")):
        ax.plot(s["f_hz"], s[col], "-", color="tab:blue", lw=1.2, zorder=2)
        ax.plot(s["f_hz"].iloc[mk], s[col].iloc[mk], "o", color="green", ms=4, zorder=3,
                label="noise" if col == "magnitude_dB" else None)
        if sweep is not None:
            ax.plot(sweep["f_hz"], sweep[col], "s", color="black", ms=5,
                    label="stepped sine" if col == "magnitude_dB" else None)
    ax1.axhline(-3.0, color="0.4", ls="--", lw=0.9)
    ax2.axhline(-180.0, color="0.4", ls="--", lw=0.9)
    f_lo, f_hi = float(s["f_hz"].min()), float(s["f_hz"].max())
    if sweep is not None:
        f_lo, f_hi = min(f_lo, sweep["f_hz"].min()), max(f_hi, sweep["f_hz"].max())
    _style_freq_axis(ax2, f_lo, f_hi)
    _mark_cutoff(ax1, ax2, res["f_c"], res["phase_at_fc"], f_hi)
    ax1.set_ylabel("Magnitude [dB]")
    ax2.set_ylabel("Phase [deg]")
    ax2.set_xlabel("Frequency [Hz]")
    if sweep is not None:                      # legend only to tell the two methods apart in the compare figure
        ax1.legend(loc="lower left", fontsize=8)
    ax1.set_title(_title_line(meta), fontsize=10)
    n = res["n_complete"]
    fig.text(0.5, 0.005,
             f"Closed-loop T(f) = measured / reference angle of the Pi-side PID + current-loop position controller, "
             f"estimated from {n} Gaussian-noise trial(s) (LP {_num(meta, 'ref_lp_hz'):g} Hz, peak +/-{amp:g} deg, "
             f"{_num(meta, 'rec_s', 15):g} s each) by Blackman-Tukey cross-spectra. The bandwidth belongs to this loop, not to the motor.",
             ha="center", va="bottom", fontsize=7, color=C_INK, wrap=True)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(out_path, dpi=FIG_DPI, bbox_inches="tight")
    return fig


def _make_figures(jobs, show):
    """jobs: (name, callable returning a saved figure or None); each in its own
    try/except. Save first, show last."""
    figs = []
    for name, fn in jobs:
        try:
            fg = fn()
        except Exception as e:
            print(f"[analyse] {name} failed ({type(e).__name__}: {e}); the other figures are unaffected.")
            continue
        if fg is not None:
            figs.append(fg)
            print(f"[analyse] saved {name}")
    if show and figs and _state["gui_ok"]:
        try:
            _plt().show()
        except Exception as e:
            print(f"[analyse] could not show the figures ({type(e).__name__}: {e}); they are saved.")
    for fg in figs:
        try:
            _plt().close(fg)
        except Exception:
            pass


# ===========================================================================
#  analyse / compare
# ===========================================================================

def _fmt(x, spec):
    return "not reached" if x is None or (isinstance(x, float) and math.isnan(x)) else spec.format(x)


def analyse_run(run_dir, show=False, make_plots=True, status=None, quiet=False):
    """Rebuild the summary CSV and the figures from the data CSV. Safe on a
    partial run. Returns the analysis dict."""
    path = os.path.join(run_dir, DATA_CSV_NAME)
    header = read_header(path)
    if status is not None:
        header = [(k, status if k == "status" else v) for k, v in header]
    meta = dict(header)
    df = read_data(path)
    res = analyse_data(df, meta)
    fc, ph = res["f_c"], res["phase_at_fc"]
    sum_header = [(k, v) for k, v in header if k not in ("n_trials_completed", "f_c_hz", "phase_at_fc_deg")]
    sum_header += [("n_trials_completed", res["n_complete"]),
                   ("f_c_hz", "not reached" if math.isnan(fc) else f"{fc:.4f}"),
                   ("phase_at_fc_deg", "not reached" if math.isnan(ph) else f"{ph:.2f}")]
    write_csv(os.path.join(run_dir, SUMMARY_CSV_NAME), sum_header, res["summary"], float_format="%.8g")
    if not quiet:
        print(f"[analyse] {run_dir}")
        print(f"[analyse] status: {meta.get('status', 'unknown')}")
        if res["n_complete"] == 0:
            print("[analyse] no complete trial yet, nothing to analyse.")
        else:
            s = res["summary"]
            rng = f"{s.f_hz.min():.2f}-{s.f_hz.max():.2f} Hz" if len(s) else "none"
            print(f"[analyse] {res['n_complete']} complete trial(s); bins {rng}; -3 dB cut-off f_c = {_fmt(fc, '{:.2f} Hz')}; "
                  f"magnitude there -3 dB, phase {_fmt(ph, '{:.1f} deg')}")
    if make_plots and len(df):
        _make_figures([
            ("fig1 (by trial)", lambda: plot_by_trial(df, meta, res, os.path.join(run_dir, FIG_BY_TRIAL))),
            ("fig2 (whole run)", lambda: plot_whole_run(df, meta, res, os.path.join(run_dir, FIG_WHOLE))),
            ("fig3 (Bode)", lambda: plot_bode(res, meta, os.path.join(run_dir, FIG_BODE))),
        ], show)
    return res


def compare_runs(noise_dir, sweep_dir, show=False):
    """Overlay the stepped-sine points on the noise Bode plot (fig3b) and print
    the differences. The sweep folder is only read."""
    res = analyse_run(noise_dir, make_plots=False, quiet=True)
    meta = dict(read_header(os.path.join(noise_dir, DATA_CSV_NAME)))
    sw = pd.read_csv(os.path.join(sweep_dir, "bode_summary.csv"), comment="#")
    fc_s = first_down_crossing(sw["f_hz"], sw["magnitude_dB"], -3.0)
    ph_s = interp_log(fc_s, sw["f_hz"], sw["phase_deg"]) if not math.isnan(fc_s) else float("nan")
    fc_n, ph_n = res["f_c"], res["phase_at_fc"]
    print(f"[compare] noise : f_c = {_fmt(fc_n, '{:.2f} Hz')}, phase there {_fmt(ph_n, '{:.1f} deg')}")
    print(f"[compare] sweep : f_c = {_fmt(fc_s, '{:.2f} Hz')}, phase there {_fmt(ph_s, '{:.1f} deg')}")
    if not (math.isnan(fc_n) or math.isnan(fc_s)):
        print(f"[compare] difference (noise - sweep): f_c {fc_n - fc_s:+.2f} Hz ({100 * (fc_n - fc_s) / fc_s:+.1f} %), "
              f"phase at f_c {ph_n - ph_s:+.1f} deg")
    out = os.path.join(noise_dir, FIG_COMPARE)
    _make_figures([("fig3b (noise vs stepped sine)", lambda: plot_bode(res, meta, out, sweep=sw))], show)
    return fc_n, fc_s


# ===========================================================================
#  selftest -- known plants through the real CSV and analysis path
# ===========================================================================

def _simulate(r_full, fn, zeta, delay_ms, stick_deg=0.0, fs=FS):
    """Output [deg] of a 2nd-order + delay plant driven by r_full (deg). With
    stick_deg > 0 a Coulomb friction (stick-slip) is added: the joint only
    moves once |command - angle| exceeds stick_deg."""
    wn = 2 * math.pi * fn
    d = int(round(delay_ms))
    u = np.concatenate([np.zeros(d), r_full[:len(r_full) - d]]) if d else r_full
    if stick_deg <= 0:
        num, den, _ = signal.cont2discrete(([wn ** 2], [1, 2 * zeta * wn, wn ** 2]), 1 / fs, "zoh")
        return signal.lfilter(num.ravel(), den, u)
    y = np.zeros(len(u))
    th, om, h = 0.0, 0.0, 1.0 / fs
    fc = wn ** 2 * stick_deg
    for i in range(len(u)):
        tau = wn ** 2 * (u[i] - th)                  # spring toward the (delayed) command, deg/s^2
        if abs(om) < 1e-3 and abs(tau) <= fc:
            om = 0.0
        else:
            f = fc * (1.0 if om > 0 else -1.0 if om < 0 else (1.0 if tau > 0 else -1.0))
            new = om + (tau - f - 2 * zeta * wn * om) * h
            om = 0.0 if (om != 0.0 and new * om < 0 and abs(tau) <= fc) else new
        th += om * h
        y[i] = th
    return y


def synthesize_run(run_dir, fn=7.0, zeta=0.7, delay_ms=5, n_trials=5, lp_hz=20.0, peak=5.0, center=0.0,
                   noise_deg=0.01, quant_deg=360.0 / 16384.0, seed_base=20260001, seed=0, stick_deg=0.0,
                   jitter=False, truncate_at_s=None, log_current=False, joint="ankle"):
    """Write bode_noise_data.csv (production format) for a known plant."""
    ref, trial, win = build_run_arrays(n_trials, lp_hz, peak, seed_base)
    y = _simulate(ref, fn, zeta, delay_ms, stick_deg)
    n = len(ref)
    rng = np.random.default_rng(seed)
    if jitter:
        gaps = rng.uniform(1 / 995.0, 1 / 985.0, n)
        gaps[n // 2] = 0.005
        tt = np.cumsum(gaps)
        tt = tt[tt < (n - 1) / FS]
        grid = np.arange(n) / FS
        ref_t, y_t = np.interp(tt, grid, ref), np.interp(tt, grid, y)
        idx = np.minimum((tt * FS).astype(int), n - 1)
        trial_t, win_t = trial[idx], win[idx]
    else:
        tt = np.arange(n) / FS
        ref_t, y_t, trial_t, win_t = ref, y, trial, win
    act = np.round((y_t + rng.normal(0, noise_deg, len(tt))) / quant_deg) * quant_deg + center
    out = pd.DataFrame({"time_s": tt, "trial": trial_t.astype(int), "reference_deg": ref_t + center,
                        "actual_deg": act, "in_window": win_t.astype(int)})
    if log_current:
        out[CURRENT_COLUMN] = 0.0
    if truncate_at_s is not None:
        out = out[out["time_s"] <= truncate_at_s]
    header = make_header(joint, center, peak, lp_hz, n_trials, seed_base, log_current=log_current)
    write_csv(os.path.join(run_dir, DATA_CSV_NAME), header, out)
    return out


def make_header(joint="ankle", center=0.0, peak=5.0, lp_hz=20.0, n_trials=5, seed_base=20260001,
                status="COMPLETED", log_current=False):
    return [("joint", joint), ("script", "synthetic"), ("kp", 0.6), ("ki", 0.0), ("kd", 0.015),
            ("center_deg", center), ("noise_peak_deg", peak), ("ref_lp_hz", lp_hz), ("ref_lp_order", 3),
            ("rec_s", 15.0), ("fade_s", 1.0), ("hold_s", 1.0), ("preroll_s", 1.0),
            ("n_trials_planned", n_trials), ("seed_base", seed_base), ("bt_max_lag_s", 4.0),
            ("f_lo_hz", 0.3), ("loop_hz_nominal", 1000), ("log_current_a", log_current),
            ("status", status)]


def _truth(f, fn, zeta, delay_ms):
    d = delay_ms / 1000.0 + 0.5 / FS            # the ZOH simulation adds half a sample
    r = np.asarray(f) / fn
    return np.exp(-2j * np.pi * np.asarray(f) * d) / (1 - r ** 2 + 2j * zeta * r)


def selftest(out_dir=None, make_plots=True):
    base = out_dir or tempfile.mkdtemp(prefix="bode_noise_selftest_")
    os.makedirs(base, exist_ok=True)
    print(f"[selftest] working in {base}")
    failures = []

    def check(name, ok, detail=""):
        print(f"  [{'PASS' if ok else 'FAIL'}] {name} {detail}")
        if not ok:
            failures.append(name)

    # 1. reference
    r, i0, i1 = make_reference(seed=3)
    check("reference: record-window peak = 5", abs(np.abs(r[i0:i1]).max() - 5.0) < 1e-9)
    check("reference: record-window mean ~ 0", abs(r[i0:i1].mean()) < 1e-9)
    check("reference: zero at both ends", r[0] == 0.0 and abs(r[-1]) < 1e-12)
    check("reference: no jump larger than the signal's own slew", np.abs(np.diff(r)).max() <= np.abs(np.diff(r[i0:i1])).max() * 1.01 + 1e-12)
    check("reference: same seed identical", np.array_equal(r, make_reference(seed=3)[0]))
    ref, trial, win = build_run_arrays(5, 20.0, 5.0, 20260001)
    check("run arrays: 91 s, windows of 15 s", len(ref) == 91000 and win.sum() == 75000)

    # 2. known plants through the real CSV and analysis path
    for fn, zeta, dly in ((7.0, 0.7, 5), (5.0, 0.6, 8), (9.0, 0.8, 4)):
        d = os.path.join(base, f"plant_{fn:g}_{zeta:g}_{dly}")
        os.makedirs(d, exist_ok=True)
        synthesize_run(d, fn, zeta, dly, seed=1)
        res = analyse_run(d, make_plots=make_plots and fn == 7.0, quiet=True)
        s = res["summary"]
        band = s[(s.f_hz >= 0.3) & (s.f_hz <= 10.0)]
        h = _truth(band.f_hz.to_numpy(), fn, zeta, dly)
        g_err = np.abs(band.magnitude_dB.to_numpy() - 20 * np.log10(np.abs(h))).max()
        p_err = np.abs(band.phase_deg.to_numpy() - np.degrees(np.unwrap(np.angle(h)))).max()
        ff = np.logspace(-1, 1.5, 100001)
        fc_true = first_down_crossing(ff, 20 * np.log10(np.abs(_truth(ff, fn, zeta, dly))), -3.0)
        tag = f"plant fn {fn:g} Hz, zeta {zeta:g}, {dly} ms"
        check(f"{tag}: gain within 0.5 dB, phase within 3 deg", g_err < 0.5 and p_err < 3.0, f"({g_err:.2f} dB, {p_err:.2f} deg)")
        check(f"{tag}: f_c within 2 %", abs(res['f_c'] - fc_true) / fc_true < 0.02, f"(found {res['f_c']:.3f}, true {fc_true:.3f} Hz)")
        if fn == 7.0:
            # Welch cross-check (selftest only)
            df = read_data(os.path.join(d, DATA_CSV_NAME))
            meta = dict(read_header(os.path.join(d, DATA_CSV_NAME)))
            t = df.time_s.to_numpy()
            pairs = [_grid_trial(t, df.reference_deg.to_numpy(), df.actual_deg.to_numpy(), 0.0, tr["lo"], tr["hi"], 15.0)
                     for tr in res["trials"]]
            Pxx = Pxy = 0
            for x, y in pairs:
                fw, a = signal.csd(x, x, FS, nperseg=4000, window="hann", noverlap=2000)
                _, b = signal.csd(x, y, FS, nperseg=4000, window="hann", noverlap=2000)
                Pxx, Pxy = Pxx + a, Pxy + b
            Hw = Pxy / Pxx
            m = (fw >= 0.3) & (fw <= 20)
            fcw = first_down_crossing(fw[m], 20 * np.log10(np.abs(Hw[m])), -3.0)
            check("Welch cross-check: f_c agrees within 2 %", abs(fcw - res["f_c"]) / res["f_c"] < 0.02, f"(Welch {fcw:.3f}, BT {res['f_c']:.3f} Hz)")
            check("summary covers 0.3 Hz up to the low-pass corner", s.f_hz.min() >= 0.3 and s.f_hz.max() <= 20.0 + 1e-9)
            raw = pd.read_csv(os.path.join(d, DATA_CSV_NAME), comment="#")
            check("data CSV columns", tuple(raw.columns) == CSV_COLUMNS)
            check("summary CSV columns", tuple(pd.read_csv(os.path.join(d, SUMMARY_CSV_NAME), comment="#").columns) == SUMMARY_COLUMNS)

    # 3. friction plant: still produces a curve; low-frequency gain droops vs the linear plant
    dl, dfr = os.path.join(base, "lin"), os.path.join(base, "fric")
    for d in (dl, dfr):
        os.makedirs(d, exist_ok=True)
    synthesize_run(dl, seed=2)
    synthesize_run(dfr, seed=2, stick_deg=0.5)
    rl, rf = analyse_run(dl, make_plots=False, quiet=True), analyse_run(dfr, make_plots=False, quiet=True)
    lf = lambda r: r["summary"][(r["summary"].f_hz >= 0.3) & (r["summary"].f_hz <= 1.0)].magnitude_dB.mean()
    check("friction plant: curve produced", len(rf["summary"]) > 20)
    check("friction plant: low-frequency gain droops vs linear", lf(rf) < lf(rl) - 0.1, f"(linear {lf(rl):+.2f} dB, friction {lf(rf):+.2f} dB)")

    # 4. partial runs
    d = os.path.join(base, "partial")
    os.makedirs(d, exist_ok=True)
    t_trial4 = 1 + 3 * 18 + 8.0           # inside trial 4's record window
    synthesize_run(d, seed=3, truncate_at_s=t_trial4)
    rp = analyse_run(d, make_plots=make_plots, quiet=True)
    check("partial run cut inside trial 4: trials 1-3 analysed", rp["n_complete"] == 3 and len(rp["summary"]) > 20, f"({rp['n_complete']} trials)")
    synthesize_run(d, seed=3, truncate_at_s=1 + 3 * 18 + 16.5)   # trial 4 fade-out: complete
    check("cut in trial 4's fade-out: trial 4 counts", analyse_run(d, make_plots=False, quiet=True)["n_complete"] == 4)
    d = os.path.join(base, "zero")
    os.makedirs(d, exist_ok=True)
    synthesize_run(d, seed=3, truncate_at_s=1 + 8.0)
    rz = analyse_run(d, make_plots=make_plots, quiet=True)
    s0 = pd.read_csv(os.path.join(d, SUMMARY_CSV_NAME), comment="#")
    check("zero complete trials: header-only summary, fig2 only", rz["n_complete"] == 0 and len(s0) == 0
          and os.path.exists(os.path.join(d, FIG_WHOLE)) == make_plots and not os.path.exists(os.path.join(d, FIG_BODE)))

    # 6. uneven ticks
    d = os.path.join(base, "jitter")
    os.makedirs(d, exist_ok=True)
    synthesize_run(d, seed=4, jitter=True)
    rj = analyse_run(d, make_plots=False, quiet=True)
    fc_true = first_down_crossing(np.logspace(-1, 1.5, 100001), 20 * np.log10(np.abs(_truth(np.logspace(-1, 1.5, 100001), 7.0, 0.7, 5))), -3.0)
    check("uneven ticks (985-995 Hz, one 5 ms gap): f_c within 2 %", rj["n_complete"] == 5 and abs(rj["f_c"] - fc_true) / fc_true < 0.02,
          f"(found {rj['f_c']:.3f}, true {fc_true:.3f} Hz, {rj['n_complete']} trials)")

    print(f"[selftest] {'ALL PASSED' if not failures else 'FAILED: ' + ', '.join(failures)}")
    return not failures


# ===========================================================================
#  CLI
# ===========================================================================

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("analyse", help="rebuild summary and figures of one run folder")
    a.add_argument("run_dir")
    a.add_argument("--show", action="store_true")
    c = sub.add_parser("compare", help="overlay the stepped-sine result on the noise Bode plot")
    c.add_argument("noise_run")
    c.add_argument("sweep_run")
    c.add_argument("--show", action="store_true")
    s = sub.add_parser("selftest", help="offline check against known plants")
    s.add_argument("--out", default=None)
    s.add_argument("--no-plots", action="store_true")
    args = ap.parse_args(argv)
    import matplotlib
    if not getattr(args, "show", False):
        matplotlib.use("Agg")
    if args.cmd == "analyse":
        analyse_run(args.run_dir, show=args.show)
    elif args.cmd == "compare":
        compare_runs(args.noise_run, args.sweep_run, show=args.show)
    else:
        sys.exit(0 if selftest(args.out, make_plots=not args.no_plots) else 1)


if __name__ == "__main__":
    main()