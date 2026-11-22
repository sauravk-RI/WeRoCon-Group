# OSL V2 — MuJoCo simulation and sim-to-real pipeline

This folder simulates the Open-Source Leg V2 bench leg in MuJoCo for the five gait tests
(normal walk, ramp ascent, ramp descent, stairs down, stairs up).

You can use it in two ways:

* **Standalone** (`SIM_2_REAL = 0`): the simulation runs on its own on any computer.
* **Sim-to-real** (`SIM_2_REAL = 1`): you press **START** in the simulation and the
  **real leg** starts the same trajectory at the same instant. The leg streams every
  control tick back, and the simulation logs the hardware data and the simulation data
  into **one run folder**.

From a run folder, `make_live_plot_video.py` makes a video of the reference, simulation
and hardware angles being drawn together as time advances, and `make_3d_render_video.py`
makes a separate video re-rendering the leg's 3-D motion in MuJoCo from the same logged
data.

```
  SIMULATION COMPUTER (Windows / Linux / Pi)              LEG RASPBERRY PI
  Simulation_Codes/sim_knee_ankle_trajectory_<gait>.py    Hardware_Codes/knee_ankle_trajectory_PID_control_me_<gait>.py
  ┌──────────────────────────────┐   TCP (port 8795)     ┌──────────────────────────────┐
  │ MuJoCo leg + 2 current PIDs  │ ───────────────────▶  │ waits, limp, for the sim     │
  │ live 3-D viewer              │  HELLO / ARM / GO /   │ ARM → home + settle → READY  │
  │ logs sim + hardware          │  heartbeat / STOP     │ GO  → PID trajectory @200 Hz │
  │ results/<gait>/run_N_…/      │ ◀───────────────────  │ streams every tick back      │
  └──────────────────────────────┘  reference, READY,    └──────────────────────────────┘
                                    200 Hz samples, DONE
```

---------------------------------------------------------------------------------------

## Contents

1. [What is where](#1-what-is-where)
2. [Install on Windows](#2-install-on-windows)
3. [Install on Linux (Ubuntu/Debian PC)](#3-install-on-linux-ubuntudebian-pc)
4. [Install on a Raspberry Pi](#4-install-on-a-raspberry-pi-simulation-computer)
5. [Check the installation](#5-check-the-installation)
6. [Prepare the leg's Raspberry Pi](#6-prepare-the-legs-raspberry-pi)
7. [Run a simulation only (`SIM_2_REAL = 0`)](#7-run-a-simulation-only-sim_2_real--0)
8. [Run a sim-to-real test (`SIM_2_REAL = 1`)](#8-run-a-sim-to-real-test-sim_2_real--1)
9. [Connecting the leg and the simulation (same network or different networks)](#9-connecting-the-leg-and-the-simulation)
10. [The run folder](#10-the-run-folder)
11. [Live-graph video](#11-live-graph-video)
12. [3D render video](#12-3d-render-video)
13. [Safety](#13-safety)
14. [How the simulation works](#14-how-the-simulation-works-and-what-it-assumes)
15. [Troubleshooting](#15-troubleshooting)

---------------------------------------------------------------------------------------

## 1. What is where

```
Simulation_Codes/                      ← copy this WHOLE folder to the simulation computer
├── sim_knee_ankle_trajectory_normal_walk.py     ┐
├── sim_knee_ankle_trajectory_ramp_ascent.py     │ one script per gait test; settings at the top
├── sim_knee_ankle_trajectory_ramp_descent.py    │ (SIM_2_REAL, HARDWARE_HOST, token, gait
├── sim_knee_ankle_trajectory_stairs_down.py     │  reference settings, PID gains, viewer)
├── sim_knee_ankle_trajectory_stairs_up.py       ┘
├── make_live_plot_video.py            run folder → live-graph video (angle curves only)
├── make_3d_render_video.py            run folder → 3-D MuJoCo re-render video
├── requirements.txt                   Python packages for the simulation computer
├── osl_sim/                           the simulation package (every file below is loaded
│   │                                   by every real test run; nothing here is unused)
│   ├── engine.py                      MuJoCo leg + drivetrains + PIDs (the final paper model)
│   ├── reference.py                   gait reference = port of the hardware scripts' logic
│   ├── runner.py                      runs one test (standalone or sim-to-real) and logs it
│   ├── sim_link.py                    network link (identical copy in Hardware_Codes/)
│   ├── results.py, visual.py, frames.py, paths.py
│   └── reference_control/             control + drivetrain modules from the reference research code (verbatim)
├── models/osl_v2_bench_aluminium_2810.xml    MuJoCo model (the final paper gait plant)
├── osl_v2_0_assembly/meshes/          CAD meshes the model needs
├── datasets/                          the five gait datasets (same files as Hardware_Codes/datasets)
└── results/                           every run creates results/<gait>/run_<N>_<date-time>/

Hardware_Codes/                        ← copy to the leg's Raspberry Pi (as before)
├── knee_ankle_trajectory_PID_control_me_<gait>.py   five gait controllers, now with SIM_2_REAL
├── sim_link.py                        network link (standard library only)
├── README.md                          leg-side setup, including the sim-to-real pipeline
└── datasets/                          the five gait datasets
```

Every path inside the code is **relative** to the file that uses it (see `osl_sim/paths.py`
and each sim script's `_HERE`/`SIM_ROOT`). Copy the whole `Simulation_Codes` folder — and
separately the whole `Hardware_Codes` folder — anywhere, on any computer or operating
system, and both always find their own `datasets/` next to themselves. Nothing needs
editing, and nothing outside the copied folder is read.

---------------------------------------------------------------------------------------

## 2. Install on Windows

These steps were checked against the exact package versions in `requirements.txt`. All of
them have ready-made Windows wheels for Python 3.10–3.13, so nothing has to be compiled.

**Step 1 — Install Python 3.12 (64-bit).**

1. Download it from <https://www.python.org/downloads/windows/>. Choose the
   *Windows installer (64-bit)* for a 3.12.x release.
2. In the first installer screen, tick **"Add python.exe to PATH"**, then click
   **Install Now**.
3. Open a new **PowerShell** window and check the installed versions:

   ```powershell
   py --version          # should print Python 3.12.x
   py -0                 # lists every Python installed; 3.12 must be there
   ```

   Python 3.10–3.13 all work. Prefer 3.11 or later: from 3.11, Windows sleeps are
   accurate to about 1 ms, so the real-time pacing is smoother.

**Step 2 — Copy the folder.** Copy the whole `Simulation_Codes` folder, for example to
`C:\OSL\Simulation_Codes`. Avoid OneDrive-synced folders: syncing can lock files while
results are being written.

**Step 3 — Create a virtual environment and install the packages.** A virtual
environment (`.venv`) keeps these packages separate from everything else on the computer.

```powershell
cd C:\OSL\Simulation_Codes
py -3.12 -m venv .venv
.venv\Scripts\python.exe -m pip install --upgrade pip
.venv\Scripts\python.exe -m pip install -r requirements.txt
```

**Step 4 (optional) — Activate the environment.** After activating it you can type
`python` instead of `.venv\Scripts\python.exe`:

```powershell
Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned
.venv\Scripts\Activate.ps1
```


Alternatively, skip activation and always type `.venv\Scripts\python.exe`. Every command
in this guide works either way.

**Step 5 — Check everything.** Follow [section 5](#5-check-the-installation).

Notes for Windows:

* **Graphics:** the MuJoCo viewer needs OpenGL, which every normal Windows graphics driver
  provides. On a remote-desktop session without a GPU the viewer may not open. The
  simulation then continues without it; you can also set `SHOW_VIEWER = False`.
* **Firewall:** the simulation only makes *outgoing* connections to the leg. Windows
  Firewall therefore needs **no rule**, and you will not normally see a firewall prompt.
* **Video:** `imageio-ffmpeg` brings its own `ffmpeg.exe`, so there is nothing else to
  install for videos.

---------------------------------------------------------------------------------------

## 3. Install on Linux (Ubuntu/Debian PC)

```bash
# system packages: Python venv support + OpenGL/EGL libraries for the viewer and video
sudo apt update
sudo apt install -y python3 python3-venv python3-pip libgl1 libegl1 libglfw3

cd ~/OSL/Simulation_Codes            # wherever you copied the folder
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
source .venv/bin/activate            # optional: then just type "python"
```

The Python version must be 3.10–3.13 (`python3 --version`). The viewer needs a desktop
session; over plain SSH it cannot open, and the simulation continues without it. The
live-graph video (see [section 11](#11-live-graph-video)) needs no OpenGL and runs the
same with or without a monitor.

---------------------------------------------------------------------------------------

## 4. Install on a Raspberry Pi

This is how the simulation runs on the lab's Raspberry Pi 5 (64-bit Raspberry Pi OS
Bookworm or newer, Python 3.11–3.13). `mujoco==3.12.0` has an ARM64 wheel, so nothing is
compiled. Use a **64-bit** OS: run `uname -m`, and it must print `aarch64`.

```bash
sudo apt update
sudo apt install -y python3-venv libgl1 libegl1 libglfw3

cd ~/system_identification_and_characterization_project/Simulation_Codes
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
```

Measured on this project's Raspberry Pi 5:

* The physics runs at about **5.8× real time**, so live sim-to-real is easy.
* The 3-D viewer runs in its own thread, so the slow Pi GPU (about 40 ms per frame) never
  delays the simulation. The worst pacing lag measured was 11 ms, and the median was
  0.06 ms.
* Under Wayland, GLFW warns *"The platform does not provide the window position"*. This
  warning is harmless.

> The **leg's** Raspberry Pi does **not** need MuJoCo or anything new. See section 6.

---------------------------------------------------------------------------------------

## 5. Check the installation

Run these from the `Simulation_Codes` folder with the virtual environment's Python. On
Windows that is `.venv\Scripts\python.exe`; on Linux and the Pi it is `.venv/bin/python`,
or just `python` once the environment is activated.

```bash
python -c "import mujoco, numpy, scipy, matplotlib; print('MuJoCo', mujoco.__version__)"

@'
import time
from osl_sim.engine import LegSimulation

targets = {'knee': 5.0, 'ankle': 2.0}
sim = LegSimulation({'knee': 0.0, 'ankle': 0.0}, targets)
t0 = time.perf_counter()
for i in range(400):
    sim.control_tick(i, targets)
speed = sim.time / (time.perf_counter() - t0)
print(f"Engine OK: simulated {sim.time:.2f} s at {speed:.1f}x real time, knee={sim.angles()['knee']:.2f} deg")
'@ | python -


python -m mujoco.viewer --mjcf=models/osl_v2_bench_aluminium_2810.xml
```

* The first command must print `MuJoCo 3.12.0`.
* The second command runs two seconds of the physics/controller engine on its own (no
  dataset or network needed) and must print `Engine OK`, with `speed` more than 1.0× —
  the live sim-to-real mode needs the physics to run faster than real time.
* The last command opens the MuJoCo viewer with the leg (it falls under gravity: no
  controller is running there). Close the window to finish.

---------------------------------------------------------------------------------------

## 6. Prepare the leg's Raspberry Pi

The leg Pi keeps its existing hardware environment: the `epicallypowerful` driver, `can`,
`smbus2`, `numpy`, `scipy`, `pandas` and `matplotlib`. **Nothing new is installed.**
`sim_link.py` uses only the Python standard library.

1. Copy the whole `Hardware_Codes` folder to the leg Pi for example to
   `/home/legpi/test_codes/`. The folder must contain:
   * the five `knee_ankle_trajectory_PID_control_me_<gait>.py` scripts
   * `sim_link.py`
   * the `datasets/` folder

   
2. Find the leg Pi's IP address with `hostname -I`. When a hardware script starts with
   `SIM_2_REAL = 1`, it also prints its addresses.
3. If a firewall is enabled on the leg Pi (it is not by default on Raspberry Pi OS),
   allow the port: `sudo ufw allow 8795/tcp`.

Each hardware script now has these extra settings next to its other USER SETTINGS:

| Setting | Shipped value | Meaning |
|---|---|---|
| `SIM_2_REAL` | `1` | `0` = run standalone, exactly as before. `1` = wait for the simulation |
| `SIM_LINK_GAIT` | the gait name | Must equal `GAIT_NAME` in the simulation script (checked on connect) |
| `SIM_LINK_PORT` | `8795` | TCP port the leg listens on |
| `SIM_LINK_TOKEN` | `"osl-sim2real-change-me"` | Shared secret, must equal the simulation's. **Change it** in both, or set the environment variable `OSL_SIM2REAL_TOKEN` on both computers |
| `SIM_LINK_HEARTBEAT_TIMEOUT_S` | `1.0` | The leg stops if the simulation is silent this long. Use `2.0` over the internet |

All five hardware scripts and all five simulation scripts are shipped with matching
`SIM_2_REAL = 1`, matching `SIM_LINK_PORT`/`SIM_LINK_TOKEN`, and matching gait names, so
each pair is ready to connect as soon as you set the simulation script's `HARDWARE_HOST`
— see [section 9](#9-connecting-the-leg-and-the-simulation) for exactly how, for both a
shared network and separate networks. With `SIM_2_REAL = 0`, `sim_link.py` is never
imported and the script behaves exactly as it did before this pipeline existed. The
control loop, gains, guards, homing and logging are all unchanged either way.

---------------------------------------------------------------------------------------

## 7. Run a simulation only (`SIM_2_REAL = 0`)

Every script ships with `SIM_2_REAL = 1` (ready to connect to the leg — see
[section 9](#9-connecting-the-leg-and-the-simulation)). For a simulation-only run with no
leg and no network at all, set `SIM_2_REAL = 0` at the top of the script first.

```bash
python sim_knee_ankle_trajectory_normal_walk.py
```

(or `..._ramp_ascent.py`, `..._ramp_descent.py`, `..._stairs_down.py`,
`..._stairs_up.py`)

1. The script builds the reference from `datasets/` with the settings at the top of the
   script. It prints the same stride, start-override and auto-fit messages as the
   hardware script.
2. The MuJoCo viewer opens with the leg at the start pose. **Press Enter** in the terminal
   to start.
3. The leg follows the trajectory in real time.
4. The run folder is saved in `results/<gait>/`, and the tracking RMS is printed.
5. Close the viewer window to exit.

Useful settings at the top of each simulation script:

| Setting | Meaning |
|---|---|
| `SHOW_VIEWER` | `False` = no 3-D window (for example over SSH) |
| `REALTIME` | `False` = compute as fast as possible (standalone only) |
| `WAIT_FOR_ENTER_TO_START` | `False` = start immediately |
| `KEEP_VIEWER_OPEN` | `False` = close the viewer as soon as the run ends |
| `STRIDE_*`, `TIME_SCALE`, `*_TRAJECTORY_START_DEG`, `*_LIMIT_*_DEG` | Same meaning as in the hardware script |
| `KNEE_KP` … `ANKLE_KD` | PID gains (A/deg, A/(deg·s), A/(deg/s)) |

The hardware script's own `SIM_2_REAL = 0` mode is simply the standalone hardware test you
already know.

---------------------------------------------------------------------------------------

## 8. Run a sim-to-real test (`SIM_2_REAL = 1`)

**Before the first real run**, do a dry run with the leg's motors disconnected or
secured, and the leg operator ready at the power switch, so you can watch the whole
ARM → home → READY → START → run sequence once before any motion matters.

**On the leg Pi.** In the gait script you want, set:

```python
SIM_2_REAL = 1
SIM_LINK_TOKEN = "your-secret"          # same as the simulation script
```

**On the simulation computer.** In the simulation script for the **same gait**, set:

```python
SIM_2_REAL = 1
HARDWARE_HOST = "192.168.1.50"          # the leg Pi's IP (or Tailscale IP / name, see section 9)
SIM_LINK_TOKEN = "your-secret"
```

**Then, in either order:**

1. On the leg Pi, run `python knee_ankle_trajectory_PID_control_me_normal_walk.py`.
   It loads its trajectory, opens **nothing** (no CAN, no I2C, motors limp) and prints
   `waiting for the simulation to connect`.
2. On the simulation computer, run `python sim_knee_ankle_trajectory_normal_walk.py`.
   It connects (retrying until the leg is up) and receives the leg's resolved reference
   and gains, then prints how they compare with its own settings.

**Then:**

3. Press **Enter** in the simulation terminal to **ARM**. The leg wakes its motors, homes
   to the start pose and settles. The simulation terminal shows `[leg] HOMING`,
   `[leg] SETTLING` and then `Leg READY`. The simulated leg is placed at the leg's
   *measured* start angles, and the viewer opens.
4. Press **Enter** again to **START**. The simulation measures the clock offset between the
   two computers (best of 30 round trips) and schedules a common start instant
   `GO_LEAD_S` (1 s) ahead. Both start at that instant. With a camera running, you will
   see the 3-D leg and the real leg move together.
5. The leg streams its data during the run. After the trajectory, the leg sends `DONE`.
   The simulation then writes **one run folder containing both datasets**. The leg still
   saves its own CSVs and plots, as before.

**To stop early:** press **Ctrl+C** in the simulation terminal. That sends STOP, and the leg
releases its motors. Ctrl+C on the leg Pi also stops the leg and tells the simulation.
Closing the simulation, losing the network, or the simulation freezing all stop the leg
within `SIM_LINK_HEARTBEAT_TIMEOUT_S`.

What is used from which side:

* **Reference, gains and limits:** the **leg's** values are authoritative
  (`USE_HARDWARE_SETTINGS = True`), so the simulation always tracks exactly what the leg
  tracks. If the simulation script's own settings differ, a warning is printed and the
  difference is saved in `run_info.json`.
* **Gait:** you choose it by which hardware script you launch. The simulation script must
  be for the same gait, otherwise the leg refuses the connection with a clear message.
* **Joints:** `KNEE_CONTROL` / `ANKLE_CONTROL` still work on the leg. The simulation always
  simulates both joints and logs whichever joints the leg reports.

---------------------------------------------------------------------------------------

## 9. Connecting the leg and the simulation

Both computers' scripts already ship with matching port, token and gait name (see
[section 6](#6-prepare-the-legs-raspberry-pi)). The simulation script's `HARDWARE_HOST`
is a three-line toggle — exactly one of the `HARDWARE_HOST = ...` lines should be
uncommented, the other(s) commented out with `#`:

```python
# ---- SAME network (lab Wi-Fi/Ethernet): this leg Pi's current LAN IP ------
HARDWARE_HOST = "172.31.59.123"
# HARDWARE_HOST = "rpi1.local"        # mDNS hostname alternative, same network only
# ---- DIFFERENT network / over the internet: this leg Pi's Tailscale IP ----
# HARDWARE_HOST = "100.x.x.x"         # `tailscale ip -4` on the leg Pi, once installed
```

Pick **Case A** or **Case B** below depending on where the simulation computer actually
is relative to the leg Pi, and nothing else in either folder needs to change.

### Case A — same network (lab Wi-Fi or Ethernet)

**Install:** nothing beyond the base setup already covered — the hardware packages from
[Hardware_Codes/README.md](../Hardware_Codes/README.md) on the leg Pi, and
`requirements.txt` from [section 2–4](#2-install-on-windows) on the simulation computer.
Plain TCP over the existing network needs no extra package on either side.

**Connect:**

1. Make sure both computers are joined to the **same** Wi-Fi network or switch.
2. **On the leg Pi**, find its address — either works:
   * `hostname -I` (prints the current LAN IP, e.g. `172.31.59.123`; this can change after
     a reboot or a DHCP lease renewal unless the router is set to always give this Pi the
     same IP)
   * `hostname -f` plus `.local` (e.g. `rpi1.local`) — works without knowing the IP, as
     long as the simulation computer also supports mDNS (Windows 10/11 and macOS do out
     of the box; Linux needs `avahi-daemon`, `sudo apt install avahi-daemon` if missing)

   A hardware script started with `SIM_2_REAL = 1` also prints its own addresses at
   startup, as a reminder.
3. **On the simulation computer**, in the gait script you'll run, uncomment the matching
   `HARDWARE_HOST` line under "SAME network" and put in that address.
4. **Test the link**: `ping <that address>` from the simulation computer. If it doesn't
   reply, you're not actually on the same network (check both computers'
   Wi-Fi/Ethernet connection), or a firewall is blocking ICMP — try step 5 anyway, since
   some networks block `ping` but still allow the TCP connection sim_link.py uses.
5. If the leg Pi runs a firewall (`ufw`, not installed by default on Raspberry Pi OS),
   allow the port: `sudo ufw allow 8795/tcp`.
6. Start both scripts (either order) and follow [section 8](#8-run-a-sim-to-real-test-sim_2_real--1)
   from "Press **Enter** ... to **ARM**" onward.

### Case B — different networks / over the internet

Use this when the simulation computer is anywhere else — a different building, a
different Wi-Fi, home broadband, a different country — not sharing a network with the
leg Pi. Direct connection is not possible there: home and university routers block
incoming connections by default (NAT), so the leg Pi cannot simply be reached by its LAN
IP from outside. **[Tailscale](https://tailscale.com)** solves this: it is free for
personal and education use, and it gives every device running it a private, fixed
address that reaches the others directly, encrypted end to end, from anywhere. **No code
changes beyond `HARDWARE_HOST` are needed.**

**Install (on *both* computers):**

```bash
curl -fsSL https://tailscale.com/install.sh | sh     # leg Pi and any Linux/Pi simulation computer
sudo tailscale up                                     # prints a login link: open it, approve the device
sudo systemctl enable --now tailscaled                # start automatically after reboots
```

On Windows, install from <https://tailscale.com/download> instead and sign in there; on
macOS, from the Mac App Store or <https://tailscale.com/download>. Sign in with the
**same Tailscale account** on both computers (or have the leg's owner share the leg Pi's
device with the remote person's account from the Tailscale admin console).

**Connect:**

1. **On the leg Pi:** `tailscale ip -4` (e.g. `100.101.102.103`) — this is the address
   that goes in `HARDWARE_HOST`. With MagicDNS enabled in the Tailscale admin console,
   the Pi's Tailscale machine name (e.g. `rpi1`) works as `HARDWARE_HOST` too.
2. **On the simulation computer**, in the gait script you'll run, uncomment the
   `HARDWARE_HOST` line under "DIFFERENT network" and put in that address.
3. **Test the link**: `ping 100.101.102.103` (or the name) from the simulation computer,
   and confirm both devices show *Connected* in the Tailscale app/`tailscale status`.
4. **Raise the timing margins for internet latency** (typically 50–300 ms round trip) in
   both scripts:

   | Where | Setting | Same network | Internet |
   |---|---|---|---|
   | simulation script | `GO_LEAD_S` | 1.0 | 2.0 |
   | simulation script | `LINK_TIMEOUT_S` | 3.0 | 5.0 |
   | hardware script | `SIM_LINK_HEARTBEAT_TIMEOUT_S` | 1.0 | 2.0 |

5. Start both scripts (either order) and follow [section 8](#8-run-a-sim-to-real-test-sim_2_real--1)
   from "Press **Enter** ... to **ARM**" onward — exactly the same procedure as Case A.

How well the two starts line up over the internet: the clock synchronisation keeps the
start error below half the best round-trip time. The exact figure is saved in
`run_info.json` as `clock_sync.uncertainty_s`, and it is usually a few ms to 20 ms. The
start instant is scheduled in advance, so it does **not** depend on the one-way network
delay. The hardware samples arrive slightly later than real time, but they are logged with
the leg's own timestamps, so the comparison is unaffected.

Alternatives: [ZeroTier](https://www.zerotier.com) works the same way. Do **not** expose
port 8795 on a router with port forwarding, in either case. The shared token is not
encryption, and this is a powered robot.

> **Remote operation of a powered leg:** someone must be physically next to the leg with the
> power switch or emergency stop whenever it is armed from another location. See section 13.

---------------------------------------------------------------------------------------

## 10. The run folder

Each test makes `results/<gait>/run_<N>_<YYYYmmdd-HHMMSS>/`.

**Angles** use the *sim frame*: knee **+flexion**, ankle **+plantarflexion**. This is the
same convention as the hardware scripts' own CSV files. **Time** is seconds since the
common START instant, and hardware times are already placed on that axis.

| File | Contents |
|---|---|
| `run_info.json` | All settings, model/engine metadata, the leg's handshake info, clock sync, start lateness, statuses, summary |
| `reference.csv` | The reference both systems tracked, on the 200 Hz control grid |
| `reference_knots.json` | The exact spline knots (from the leg in sim-to-real mode) |
| `simulation.csv` | Simulation, one row per 5 ms control tick: reference, angle, current command, P/I/D, joint torque, belt deflection, real-time lag |
| `simulation_full_rate.csv` | Simulation, one row per 0.5 ms physics step: angles, targets, currents, joint/actuator/friction torques, belt deflection, velocities |
| `hardware.csv` | *(sim-to-real)* The leg's own ticks: encoder time, dt, reference, encoder angle, error, P/I/D, requested and commanded current, clamp flag, encoder validity, raw encoder-frame angle |
| `hardware_raw.jsonl` | *(sim-to-real)* Every message received from the leg, unmodified |
| `comparison.csv` | *(sim-to-real)* Each hardware encoder sample paired with the simulated angle at the same instant |
| `summary.json` | Tracking RMS and peak for simulation and hardware, sim−hardware RMS and peak, exploratory best-fit lag |
| `comparison.png` | Static overlay plot |

The hardware P/I/D and current columns use the hardware's motor-command sign
(`ENCODER_TO_MOTOR_SIGN` already applied), exactly as in the leg's own CSV. The
simulation's currents use the model's positive-torque convention.

---------------------------------------------------------------------------------------

## 11. Live-graph video

```bash
python make_live_plot_video.py results/normal_walk/run_1_20261118-101500
```

This writes `live_plot_both.mp4` into the run folder: the knee and ankle angle graphs,
with **Reference**, **Simulation** and **Hardware** drawn up to the current time, a
moving time cursor, and live numeric readouts. The sim−hardware RMS is in the title.
No 3-D rendering is involved, so it needs no OpenGL/GPU and runs the same on a headless
machine.

Options:

| Option | Meaning |
|---|---|
| `--joints ankle` | Ankle only (or `knee`; default `both`) |
| `--speed 0.5` | Slow motion; `1` = real time |
| `--window 3` | A sliding 3-second window instead of the whole run |
| `--fps 30` | Frame rate |
| `--show` | Also play the animation in a window |
| `--out file.mp4` | Choose the output file; `.gif` also works |

It works for standalone simulation runs too; they just have no Hardware line, since
there is no hardware trace to draw. A 5–7 s run takes well under a minute to encode on
a Raspberry Pi 5, and faster still on a laptop.

---------------------------------------------------------------------------------------

## 12. 3D render video

```bash
python make_3d_render_video.py results/normal_walk/run_1_20261118-101500
```

This writes `render_3d.mp4` into the run folder: a single-panel offscreen MuJoCo
re-render of the **simulated** leg's motion, recreated from `simulation_full_rate.csv`
— not a live capture, so it can be made from any past run, any time, on any computer
with MuJoCo installed. This is true for a standalone run and a sim-to-real run alike;
the real leg's own encoder data (`hardware.csv`, present only for a sim-to-real run) is
never rendered here — see [section 11](#11-live-graph-video) if you want the real leg's
angles shown too, as a curve alongside the simulated one.

This is a separate tool from `make_live_plot_video.py`: run either one, or both, from
the same run folder — they don't interact and don't need each other.

Options:

| Option | Meaning |
|---|---|
| `--speed 0.5` | Slow motion; `1` = real time |
| `--fps 30` | Frame rate |
| `--show` | Also play the animation in a window |
| `--out file.mp4` | Choose the output file; `.gif` also works |

A 5–7 s run takes well under a minute to render on a Raspberry Pi 5. On a Linux machine
with no monitor, run it as `MUJOCO_GL=egl python make_3d_render_video.py <run folder>`.

---------------------------------------------------------------------------------------

## 13. Safety

* With `SIM_2_REAL = 1`, the leg opens **no** CAN or I2C bus and powers **no** motor until
  ARM is pressed in the simulation. A simulation that disconnects before ARM leaves the
  leg waiting, still limp, for a new connection.
* After ARM, the leg aborts and releases both motors through its normal shutdown code in
  any of these cases:
  * STOP or Ctrl+C in the simulation
  * the TCP connection is lost
  * no message arrives for `SIM_LINK_HEARTBEAT_TIMEOUT_S` (the simulation sends a
    heartbeat every 0.2 s)
  * Ctrl+C on the leg Pi

  All five cases were tested during development, including a frozen (SIGSTOP) and a
  killed simulation process.
* Every existing guard stays active: hardware limits, tracking-error abort, I2C failure
  and encoder-glitch rejection, and homing timeouts. The network code never blocks the
  200 Hz loop. Samples go into a memory queue, and a separate thread sends them.
* The simulation **never** sends targets or currents to the leg. The leg computes its own
  trajectory and control exactly as in standalone mode. The simulation only says
  ARM, START (when) and STOP.
* Use a non-default `SIM_LINK_TOKEN` on both sides.
* For remote operation, someone must be next to the leg with the power switch or
  emergency stop. A network link is not an emergency stop.

---------------------------------------------------------------------------------------

## 14. How the simulation works and what it assumes

The engine (`osl_sim/engine.py`) re-implements the project's validated `AssembledTracking`
MuJoCo plant, with its **final paper configuration** (`replay_paper.py` in that research
codebase) built in. Its control and drivetrain modules are used **verbatim**
(`osl_sim/reference_control/`; only five import lines differ). It was checked against all
six of the published reference gait trials before release, reproducing all 12 of the
published RMSE values to about 1e-15°.

| Item | Value |
|---|---|
| Model | `osl_v2_bench_aluminium_2810.xml`: CAD-derived, 5.069 kg, knee and ankle hinges; the upper body `knee_prox` is **fixed to the world**; no ground contact or body weight |
| Physics | MuJoCo 3.12.0, 0.5 ms step, implicitfast, gravity |
| Actuation | Authored MuJoCo position servos **disabled**. Each joint is driven only by its own compliant drivetrain torque (Best et al. 2025 actuator + nonlinear belt; belt ratio 4.61, output KT 0.7044 N·m/A, rotor KT 0.095 N·m/A, current-dependent gear friction) |
| Controller | Two independent current-domain PIDs (deg in, A out) at 200 Hz, ±60 A clip, ideal current loop, integral limit 50 deg·s. Gains knee 3.0 / 0 / 0.10, ankle 2.0 / 0 / 0.075 (from the leg in sim-to-real mode) |
| Derivative | "Configuration C": target minus gearbox-output angle referred to the joint (the final paper setting). **The hardware PID differentiates the joint-angle error.** Testing showed that the joint-error derivative makes the coupled model diverge, so the simulation and hardware share gains but not this one feedback signal |
| Ankle range | Runtime ankle range widened to ±31° (the research code's `--ankle-range hardware`); the XML on disk is unchanged |
| Start state | Standalone: the reference start, at rest. Sim-to-real: the leg's measured angles at READY, at rest |
| Reference | Bit-identical to the hardware script's: dataset, stride selection, signs, start override, auto-fit, TIME_SCALE, not-a-knot cubic spline |

These assumptions are not yet verified and should be stated with any result: the physical
encoder zero and direction relative to the model's joint frame, the mount orientation, and
the drivetrain parameters (taken from the literature and datasheets, not fitted to this
leg).

---------------------------------------------------------------------------------------

## 15. Troubleshooting

| Problem | Fix |
|---|---|
| `leg not reachable yet … retrying` forever | Is the leg script running with `SIM_2_REAL = 1`? Is `HARDWARE_HOST` right (`hostname -I` or `tailscale ip -4` on the leg)? Can you `ping` it? Is the port the same on both sides? |
| `leg rejected the simulation: bad token or protocol` | `SIM_LINK_TOKEN` (or `OSL_SIM2REAL_TOKEN`) differs between the two computers, or one side has an old `sim_link.py` |
| `gait mismatch` | You started a different gait script on the leg than in the simulation |
| `WARNING: the leg's settings differ` | A STRIDE, TIME_SCALE, start or limit setting was changed on one side only. The leg's values are used automatically; copy them into the simulation script by hand if you want the two files to agree again |
| `leg refused to start: start instant already passed` | The network is slower than `GO_LEAD_S`; increase it (2.0 over the internet) |
| Leg aborts with `heartbeat timeout` | The network stalled. Over the internet, raise `SIM_LINK_HEARTBEAT_TIMEOUT_S` to 2.0 and `LINK_TIMEOUT_S` to 5.0 |
| Viewer does not open | There is no display or OpenGL (SSH, remote desktop). The run continues; set `SHOW_VIEWER = False` |
| `Could not open … plugin directory` / GLFW Wayland warnings on a Pi | Harmless |
| `ModuleNotFoundError: mujoco` | You are not using the `.venv` Python. Activate it, or call `.venv\Scripts\python.exe` / `.venv/bin/python` |
| `pip` cannot find `mujoco==3.12.0` | Python is too old or new (use 3.10–3.13), or the OS is 32-bit (use 64-bit) |
| Section 5's engine smoke check fails or is very slow | The model, `reference_control` or the package versions were changed; restore them from this folder |
