#!/usr/bin/env python3
"""Turn a logged run folder into a LIVE-GRAPH video: reference, simulation and hardware
angles drawn together as time advances, with a moving time cursor and live readouts.

    python make_live_plot_video.py results/normal_walk/run_3_20261117-163513
    python make_live_plot_video.py <run folder> --joints ankle          # ankle only
    python make_live_plot_video.py <run folder> --speed 0.5             # slow motion
    python make_live_plot_video.py <run folder> --show                  # live window too

Output: <run folder>/live_plot_<joints>.mp4   (or .gif if no ffmpeg is available)

Works for standalone simulation runs too (no hardware trace; the Hardware line is
simply absent).

Options
    --joints both|knee|ankle   which angle graphs to draw (default both)
    --fps N                    video frame rate (default 30)
    --speed X                  playback speed, 1 = real time (default 1)
    --window S                 show a sliding S-second window instead of the whole run
    --show                     also open an interactive window playing the animation
    --out PATH                 output file (.mp4 or .gif)
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from osl_sim.frames import JOINTS                                    # noqa: E402
from osl_sim.results import read_csv                                 # noqa: E402

COLORS = dict(reference="#777f88", simulation="#087e82", hardware="#d9822b")


def load_run(run_dir):
    sim = read_csv(run_dir / "simulation.csv")
    full = read_csv(run_dir / "simulation_full_rate.csv")
    hw_path = run_dir / "hardware.csv"
    hw = read_csv(hw_path) if hw_path.exists() else []
    info = json.loads((run_dir / "run_info.json").read_text(encoding="utf-8"))
    data = dict(info=info, t_ref=np.array([r["time_s"] for r in sim]))
    for j in JOINTS:
        data[f"ref_{j}"] = np.array([r[f"{j}_reference_deg"] for r in sim])
        t0 = sim[0][f"{j}_sim_deg"]
        data["t_sim"] = np.array([0.0] + [r["time_s"] for r in full])
        data[f"sim_{j}"] = np.array([t0] + [r[f"{j}_sim_deg"] for r in full])
        rows = [r for r in hw if r.get(f"{j}_actual_deg") is not None]
        data[f"t_hw_{j}"] = np.array([r[f"{j}_encoder_time_s"] for r in rows])
        data[f"hw_{j}"] = np.array([r[f"{j}_actual_deg"] for r in rows])
    data["has_hw"] = any(len(data[f"hw_{j}"]) for j in JOINTS)
    data["duration"] = float(max(data["t_ref"][-1], data["t_sim"][-1]))
    return data


def interp(t, ts, ys):
    if len(ts) == 0 or t < ts[0]:
        return None
    return float(np.interp(min(t, ts[-1]), ts, ys))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--joints", choices=("both", "knee", "ankle"), default="both")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--window", type=float, default=None)
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    run_dir = args.run_dir.resolve()
    d = load_run(run_dir)
    joints = JOINTS if args.joints == "both" else (args.joints,)

    import matplotlib
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import animation

    fig, axes_list = plt.subplots(len(joints), 1, figsize=(11, 4.5 * len(joints)), dpi=100,
                                  squeeze=False)
    axes = {j: axes_list[r, 0] for r, j in enumerate(joints)}

    T = d["duration"]
    lines, cursors, readouts = {}, {}, {}
    for j, ax in axes.items():
        vals = [d[f"ref_{j}"], d[f"sim_{j}"]] + ([d[f"hw_{j}"]] if len(d[f"hw_{j}"]) else [])
        lo = min(float(np.min(v)) for v in vals)
        hi = max(float(np.max(v)) for v in vals)
        pad = 0.08 * (hi - lo + 1e-6)
        ax.set_ylim(lo - pad, hi + pad)
        ax.set_xlim(0, args.window or T)
        ax.set_ylabel(f"{j.capitalize()} angle (deg)\n"
                      f"{'+flexion' if j == 'knee' else '+plantarflexion'}")
        ax.grid(alpha=0.3)
        lines[j] = {
            "reference": ax.plot([], [], "--", color=COLORS["reference"], lw=1.6, label="Reference")[0],
            "simulation": ax.plot([], [], color=COLORS["simulation"], lw=2.0, label="Simulation")[0]}
        if len(d[f"hw_{j}"]):
            lines[j]["hardware"] = ax.plot([], [], color=COLORS["hardware"], lw=2.0,
                                           label="Hardware")[0]
        cursors[j] = ax.axvline(0, color="k", lw=0.8, alpha=0.5)
        readouts[j] = ax.text(0.01, 0.97, "", transform=ax.transAxes, va="top", family="monospace",
                              fontsize=10, bbox=dict(fc="white", ec="0.8", alpha=0.85))
        ax.legend(loc="upper right", fontsize=10)
    list(axes.values())[-1].set_xlabel("Time since start (s)")
    info = d["info"]
    s = info.get("summary", {}).get("joints", {})
    subtitle = "   ".join(
        f"{j}: sim−hw RMS {s[j]['sim_minus_hardware_rms_deg']:.2f}°"
        for j in joints if s.get(j, {}).get("sim_minus_hardware_rms_deg") is not None)
    title = fig.suptitle("", fontsize=14)
    mode = "sim-to-real" if d["has_hw"] else "simulation"
    fig.tight_layout(rect=(0, 0, 1, 0.95))

    n_frames = int(math.ceil(T / args.speed * args.fps)) + 1

    def draw(k):
        t = min(k * args.speed / args.fps, T)
        title.set_text(f"OSL V2 — {info.get('gait')} — {mode}   t = {t:5.2f} s"
                       + (f"\n{subtitle}" if subtitle else ""))
        for j in joints:
            m = d["t_ref"] <= t
            lines[j]["reference"].set_data(d["t_ref"][m], d[f"ref_{j}"][m])
            m = d["t_sim"] <= t
            lines[j]["simulation"].set_data(d["t_sim"][m], d[f"sim_{j}"][m])
            text = [f"ref {interp(t, d['t_ref'], d[f'ref_{j}']):+7.2f}°",
                    f"sim {interp(t, d['t_sim'], d[f'sim_{j}']):+7.2f}°"]
            if "hardware" in lines[j]:
                th = d[f"t_hw_{j}"]
                m = th <= t
                lines[j]["hardware"].set_data(th[m], d[f"hw_{j}"][m])
                h = interp(t, th, d[f"hw_{j}"])
                text.append(f"hw  {h:+7.2f}°" if h is not None else "hw     --")
            readouts[j].set_text("\n".join(text))
            cursors[j].set_xdata([t, t])
            if args.window:
                axes[j].set_xlim(max(0.0, t - args.window), max(args.window, t))
        return []

    anim = animation.FuncAnimation(fig, draw, frames=n_frames, interval=1000 / args.fps,
                                   blit=False, repeat=False)
    out = args.out or run_dir / f"live_plot_{args.joints}.mp4"
    writer = None
    if out.suffix.lower() == ".mp4":
        try:
            import imageio_ffmpeg
            matplotlib.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
            writer = animation.FFMpegWriter(fps=args.fps, bitrate=6000)
        except Exception as exc:                                # noqa: BLE001
            print(f"[video] ffmpeg not available ({exc}); writing a GIF instead.")
            out = out.with_suffix(".gif")
    if writer is None:
        writer = animation.PillowWriter(fps=min(args.fps, 20))
    print(f"[video] Rendering {n_frames} frames ({T:.2f} s of data at {args.speed}x) -> {out}")

    def progress(i, n):
        if i % max(1, n // 10) == 0:
            print(f"[video]   {100 * i // n:3d}%", flush=True)

    anim.save(str(out), writer=writer, progress_callback=progress)
    print(f"[video] Saved {out}")
    if args.show:
        anim = animation.FuncAnimation(fig, draw, frames=n_frames, interval=1000 / args.fps,
                                       blit=False, repeat=True)
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
