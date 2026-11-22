#!/usr/bin/env python3
"""Turn a logged run folder into a 3-D MuJoCo RENDER video, recreated from the logged
simulation angles -- this is an offscreen re-render after the fact, not a live capture,
so it can be made from any past run, any time, on any computer with MuJoCo installed.

    python make_3d_render_video.py results/normal_walk/run_3_20261117-163513
    python make_3d_render_video.py <run folder> --speed 0.5      # slow motion
    python make_3d_render_video.py <run folder> --show           # live window too

Output: <run folder>/render_3d.mp4   (or .gif if no ffmpeg is available)

Always one panel, the SIMULATED leg, re-posed from simulation_full_rate.csv -- for a
standalone run and for a sim-to-real run alike. The real leg's own encoder data
(hardware.csv, present only for a sim-to-real run) is never rendered here.

For the live-graph angle video (reference/simulation/hardware curves, no 3-D render),
use make_live_plot_video.py instead; the two tools are independent and can both be run
from the same run folder.

Options
    --fps N        video frame rate (default 30)
    --speed X      playback speed, 1 = real time (default 1)
    --show         also open an interactive window playing the animation
    --out PATH     output file (.mp4 or .gif)

Headless Linux (no monitor): set MUJOCO_GL=egl (or osmesa) before running this script.
Windows / desktop Linux need nothing extra.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from osl_sim.frames import JOINTS                                    # noqa: E402
from osl_sim.paths import MODEL_PATH                                 # noqa: E402
from osl_sim.results import read_csv                                 # noqa: E402


def load_run(run_dir):
    sim = read_csv(run_dir / "simulation.csv")        # only for each joint's t=0 pose
    full = read_csv(run_dir / "simulation_full_rate.csv")
    info = json.loads((run_dir / "run_info.json").read_text(encoding="utf-8"))
    data = dict(info=info)
    data["t_sim"] = np.array([0.0] + [r["time_s"] for r in full])
    for j in JOINTS:
        t0 = sim[0][f"{j}_sim_deg"]
        data[f"sim_{j}"] = np.array([t0] + [r[f"{j}_sim_deg"] for r in full])
    data["duration"] = float(data["t_sim"][-1]) if len(data["t_sim"]) else 0.0
    return data


def interp(t, ts, ys):
    return float(np.interp(min(max(t, ts[0]), ts[-1]), ts, ys))


class LegRenderer:
    """Offscreen MuJoCo render of the leg at a given joint pose (sim frame, degrees)."""

    def __init__(self, width=480, height=480):
        import mujoco
        from osl_sim.visual import brighten, set_side_camera
        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.model.joint("ankle").range[:] = np.radians([-31, 31])
        brighten(self.model)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height, width)
        self.cam = mujoco.MjvCamera()
        set_side_camera(self.cam, self.model)
        self.qadr = {j: int(self.model.joint(j).qposadr[0]) for j in JOINTS}

    def render(self, angles_deg):
        for j in JOINTS:
            self.data.qpos[self.qadr[j]] = math.radians(angles_deg[j])
        self.mj.mj_forward(self.model, self.data)
        self.renderer.update_scene(self.data, self.cam)
        return self.renderer.render()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    run_dir = args.run_dir.resolve()
    d = load_run(run_dir)
    if d["duration"] <= 0:
        print("[render] This run has no simulation_full_rate.csv data to render.")
        return 1

    try:
        renderer = LegRenderer()
    except Exception as exc:                                        # noqa: BLE001
        print(f"[render] MuJoCo offscreen rendering is unavailable ({exc}).\n"
              f"  Headless Linux: prefix the command with MUJOCO_GL=egl (or osmesa).")
        return 1

    import matplotlib
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import animation

    fig, ax = plt.subplots(figsize=(5.2, 5.8), dpi=100)
    ax.set_axis_off()
    image = ax.imshow(np.zeros((480, 480, 3), dtype=np.uint8))

    T = d["duration"]
    title = fig.suptitle("", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    n_frames = int(math.ceil(T / args.speed * args.fps)) + 1

    def draw(k):
        t = min(k * args.speed / args.fps, T)
        title.set_text(f"OSL V2 — {d['info'].get('gait')} — simulation   t = {t:5.2f} s")
        image.set_data(renderer.render(
            {j: interp(t, d["t_sim"], d[f"sim_{j}"]) for j in JOINTS}))
        return []

    anim = animation.FuncAnimation(fig, draw, frames=n_frames, interval=1000 / args.fps,
                                   blit=False, repeat=False)
    out = args.out or run_dir / "render_3d.mp4"
    writer = None
    if out.suffix.lower() == ".mp4":
        try:
            import imageio_ffmpeg
            matplotlib.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
            writer = animation.FFMpegWriter(fps=args.fps, bitrate=6000)
        except Exception as exc:                                    # noqa: BLE001
            print(f"[render] ffmpeg not available ({exc}); writing a GIF instead.")
            out = out.with_suffix(".gif")
    if writer is None:
        writer = animation.PillowWriter(fps=min(args.fps, 20))
    print(f"[render] Rendering {n_frames} frames ({T:.2f} s of data at {args.speed}x) -> {out}")

    def progress(i, n):
        if i % max(1, n // 10) == 0:
            print(f"[render]   {100 * i // n:3d}%", flush=True)

    anim.save(str(out), writer=writer, progress_callback=progress)
    print(f"[render] Saved {out}")
    if args.show:
        anim = animation.FuncAnimation(fig, draw, frames=n_frames, interval=1000 / args.fps,
                                       blit=False, repeat=True)
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
