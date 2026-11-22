# OSL V2 — leg hardware controllers

This folder runs on the Raspberry Pi bolted to the physical leg. It drives the knee and
ankle CubeMars AK80-9 actuators over CAN and reads each joint's AS5048B magnetic encoder
over I2C, under a code-level PID current-loop controller.

It optionally also runs as one half of a MuJoCo sim-to-real pipeline — see
[section 4](#4-connecting-to-a-simulation-computer) — but everything else below applies
whether or not you ever use that.

---------------------------------------------------------------------------------------

## Contents

1. [What is where](#1-what-is-where)
2. [Dataset paths are always relative to this folder](#2-dataset-paths-are-always-relative-to-this-folder)
3. [Setting up a fresh leg Raspberry Pi from scratch](#3-setting-up-a-fresh-leg-raspberry-pi-from-scratch)
4. [Connecting to a simulation computer](#4-connecting-to-a-simulation-computer)
5. [Running a test](#5-running-a-test)
6. [Results](#6-results)
7. [Safety](#7-safety)
8. [Troubleshooting](#8-troubleshooting)

---------------------------------------------------------------------------------------

## 1. What is where

```
Hardware_Codes/
├── knee_ankle_trajectory_PID_control_me_<gait>.py   five combined knee+ankle gait
│                                                     controllers (normal_walk, ramp_ascent,
│                                                     ramp_descent, stairs_down, stairs_up)
├── knee_step_test_PID_control_me*.py                knee-only step-response tests
├── ankle_step_test_PID_control_me*.py                ankle-only step-response tests
├── knee_trajectory_servoPositionControl_me_ie.py    knee-only, Servo Position Loop Mode
├── ankle_trajectory_servoPositionControl_me_ie.py   ankle-only, Servo Position Loop Mode
├── commanded_current_vs_measured_current_test.py
├── knee_bode_noise_PID_control_me.py, ankle_bode_noise_PID_control_me.py, bode_noise_*.py
├── knee_free_swing_test_me.py, knee_resistance_torque_test_PID_control_me_loadcell_d_on_meas.py
├── loadcell_test.py, as5048b_read.py
├── sim_link.py                 sim-to-real network link (standard library only)
├── requirements.txt            Python packages for this Pi
├── joint_specifications.txt    I2C addresses, encoder offsets and sign conventions
└── datasets/                   the gait datasets every script above reads from
    ├── normal_walk_1-2/
    ├── ramp_acsent_10_degrees/
    ├── ramp_descent_10degrees/
    ├── stairs_down_normal/
    └── stairs_up_normal/
```

Each script writes its own `results/<script-specific-folder>/run_<N>/` next to itself,
created the first time that script is run, with a fresh `run_<N>` every time.

---------------------------------------------------------------------------------------

## 2. Dataset paths are always relative to this folder

Every script that reads a gait dataset finds it at `datasets/<trial>/...` **next to
itself**, resolved from `os.path.dirname(os.path.abspath(__file__))` at import time —
never from a hard-coded absolute path, and never from the current working directory.

This means: **copy this whole `Hardware_Codes` folder anywhere** — a different directory
on this Pi, a different Pi, a USB stick, a laptop for a code review — and every script
still finds its own datasets, with no path to edit. The same is true of the companion
`Simulation_Codes/` folder and its own `datasets/` copy; the two folders are independent
and neither reads from the other's.

If you only need some of the five gait trials on the leg Pi (to save space), you can
delete the datasets subfolders you don't need — the matching gait script will fail
clearly, instead of silently reading from somewhere else, if its own folder is missing.

---------------------------------------------------------------------------------------

## 3. Setting up a fresh leg Raspberry Pi from scratch

### Step 1 — Copy this folder and install the Python packages

Copy the whole `Hardware_Codes` folder onto the Pi, anywhere you like (for example
`/home/pi/Hardware_Codes` or the historical `/home/legpi/test_codes/`) — see
[section 2](#2-dataset-paths-are-always-relative-to-this-folder). Then:

```bash
cd Hardware_Codes
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
```

The Python version must be 3.11–3.13 (`python3 --version`); check with
`uname -m` that the OS is 64-bit (`aarch64`, not `armv7l`) — `epicallypowerful` only
ships a 64-bit ARM wheel. `pip` resolves everything from [PyPI](https://pypi.org) and
[piwheels.org](https://www.piwheels.org) (the Raspberry Pi wheel mirror); nothing is
compiled from source.

Leave `.venv` **out** of the folder if you later copy it elsewhere — recreate the virtual
environment on each new machine instead of copying it, since it contains compiled,
machine-specific binaries.

### Step 2 — Verify everything before connecting to the actual actuators

```bash
.venv/bin/python -c "import can, smbus2, epicallypowerful, opensourceleg, numpy, scipy, pandas, matplotlib; print('OK')"
ip -details link show can0             # state UP, bitrate 1000000
i2cdetect -y 1                         # with both encoders wired: 0x40 and 0x41 should show
```

`i2cdetect` shows `--` everywhere if no AS5048B is wired/powered yet; `0x40` and `0x41`
appearing confirms the knee and ankle encoders are both detected (see
`joint_specifications.txt` for the address-to-joint mapping, offsets and sign
conventions). A permission error on `/dev/i2c-1` or `can0` means step 6 was skipped or
the Pi was not rebooted/re-logged-in since.

---------------------------------------------------------------------------------------

## 4. Connecting to a simulation computer

The five `knee_ankle_trajectory_PID_control_me_<gait>.py` scripts can run together with
the MuJoCo simulation on another computer or
the same one: the leg waits, with its motors untouched, until the simulation ARMs it,
then runs the trajectory in step with the simulation and streams its data back. All five
scripts here ship ready for this (`SIM_2_REAL = 1`), matching the five scripts in
`Simulation_Codes` on port, token and gait name — only the network address needs
pointing at each other, in [Network setup](#network-setup) below.

### The switch

At the top of each script, next to the other USER SETTINGS:

```python
SIM_2_REAL = 1      # 0 = standalone, exactly as before;  1 = run together with the simulation
SIM_LINK_GAIT = "normal_walk"             # must equal GAIT_NAME in the simulation script
SIM_LINK_PORT = 8795
SIM_LINK_TOKEN = "osl-sim2real-change-me"   # same secret as the simulation (or env OSL_SIM2REAL_TOKEN)
SIM_LINK_HEARTBEAT_TIMEOUT_S = 1.0        # 2.0 when the simulation is on another network
```

* **`SIM_2_REAL = 0`:** nothing changes. `sim_link.py` is not even imported. Homing,
  settle, PID, guards, CSVs and plots work exactly as before this pipeline existed — see
  [section 5](#5-running-a-test).
* **`SIM_2_REAL = 1`:** the run goes like this.
  1. The script loads and checks its trajectory as usual, then **opens nothing** (no CAN,
     no I2C, motors limp). It prints its own IP addresses and waits for the simulation
     to connect.
  2. The simulation connects and receives this script's resolved reference, gains and
     limits. If the simulation disconnects now, the leg just waits for it again.
  3. **ARM** in the simulation: wake motors → home → settle (the same code as standalone),
     then report READY.
  4. Hold the start pose (zero velocity, all guards active) until **START** in the
     simulation. Both computers start at the same pre-scheduled instant.
  5. Run the trajectory exactly as in standalone mode, streaming every control tick to
     the simulation. Streaming never blocks the 200 Hz loop.
  6. Shut down and save the CSVs and plots as usual, and report DONE to the simulation.

**What stops the leg, once ARMed:** STOP or Ctrl+C in the simulation; the network
connection dropping; nothing heard from the simulation for `SIM_LINK_HEARTBEAT_TIMEOUT_S`;
Ctrl+C on this Pi; or any existing guard (hard limit, tracking error, I2C, encoder
glitches, homing timeout) — exactly as in standalone mode. The simulation never sends
targets or currents; it only ever says ARM, START (when) and STOP.

**Files this needs on the leg Pi**, besides the gait script itself: `sim_link.py`
(standard library only — **must be the identical file** as
`Simulation_Codes/osl_sim/sim_link.py`) and `datasets/` (see
[section 2](#2-dataset-paths-are-always-relative-to-this-folder)). No extra Python
packages beyond [section 3](#3-setting-up-a-fresh-leg-raspberry-pi-from-scratch)'s are
needed for this.

**The SIM-LINK code blocks.** Every change this pipeline made to a gait script is marked
`SIM-LINK n/5` in that script, so it is easy to find or remove:

| Block | Where | What |
|---|---|---|
| settings | USER SETTINGS | `SIM_2_REAL` and `SIM_LINK_*` |
| 1/5 | after the pre-flight limit check | start the link server, send the reference, wait for ARM |
| 2/5 | homing and settle loops | stop if the simulation says so or the link is lost |
| 3/5 | `sim_link_hold_until_go()`, before the HANDOFF | READY, then hold until the START instant |
| 4/5 | PID loop | publish each tick; add link problems to the abort reasons |
| 5/5 | shutdown, after the motors are limp | report the outcome to the simulation and flush the stream |

**Before the first real run:** do a dry run with the leg's motors disconnected or
secured, and the leg operator ready at the power switch, so the whole
ARM → home → READY → START → run sequence has been watched once end to end before any
motion matters.

**If you change any reference setting** here (`STRIDE_*`, `TIME_SCALE`, start angle,
limits), copy the same change into the matching
`sim_knee_ankle_trajectory_<gait>.py` script in `Simulation_Codes`. The leg's own
reference and gains are what the simulation actually tracks in sim-to-real mode
(`USE_HARDWARE_SETTINGS` there), and the simulation prints a warning at connect time if
the two scripts' settings no longer agree.

### Network setup

The only thing left before the two sides can actually reach each other is pointing the
simulation computer's `HARDWARE_HOST` at this Pi's address. **The full procedure for both
cases, including exactly what to set on the simulation side, is in
`Simulation_Codes/README.md`, section 9 ("Connecting the leg and the simulation").** This
is what it means for this Pi:

### Case A — simulation computer on the same network (lab Wi-Fi or Ethernet)

Nothing extra to install here. Just find this Pi's address so it can be typed into the
simulation script's `HARDWARE_HOST`:

```bash
hostname -I          # this Pi's current LAN IP, e.g. 172.31.59.123 -- can change after
                      # a reboot/DHCP renewal unless the router gives this Pi a fixed IP
hostname -f           # this Pi's hostname; <that>.local also reaches it from most
                      # computers (Windows/macOS out of the box; Linux needs avahi-daemon)
```

A gait script run with `SIM_2_REAL = 1` also prints these addresses itself at startup.
If this Pi runs a firewall (`ufw`, not installed by default on Raspberry Pi OS), open
the port: `sudo ufw allow 8795/tcp`.

### Case B — simulation computer on a different network / over the internet

Install **[Tailscale](https://tailscale.com)** on this Pi (and have the simulation
computer's owner install it on theirs — see the simulation README for that side):

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up                        # prints a login link: open it, approve the device
sudo systemctl enable --now tailscaled   # start automatically after reboots
tailscale ip -4                          # this Pi's Tailscale address, e.g. 100.101.102.103
```

That address (or, with MagicDNS on, this Pi's Tailscale machine name) is what goes into
the simulation script's `HARDWARE_HOST`. Both computers must be signed into the **same**
Tailscale account (or this Pi shared with the other account from the Tailscale admin
console). Raise `SIM_LINK_HEARTBEAT_TIMEOUT_S` to `2.0` in the gait script for internet
latency. Never port-forward 8795 on a router instead of using Tailscale — the shared
token is not encryption.

Whichever case applies, once `HARDWARE_HOST` is set correctly on the simulation side,
start this script and the matching simulation script (either order) — "The switch"
above describes exactly what happens next.

---------------------------------------------------------------------------------------

## 5. Running a test

```bash
cd Hardware_Codes
.venv/bin/python knee_ankle_trajectory_PID_control_me_normal_walk.py
```

(or any other script in this folder). Each script prints its own startup checks (CAN
IDs, PID gains, hardware limits, I2C addresses), homes the joint(s), then runs its test
and saves CSVs/plots into its own `results/.../run_<N>/` folder. Press **Ctrl+C** at any
time — the motors are always released and whatever was logged is still saved.

The five `knee_ankle_trajectory_PID_control_me_<gait>.py` scripts ship with
`SIM_2_REAL = 1`, so by default they wait for the simulation to connect and ARM them —
see [section 4](#4-connecting-to-a-simulation-computer). Set `SIM_2_REAL = 0` at the top
of a script to run it completely on its own, with no network involved at all; it then
behaves exactly as it did before this pipeline existed, and `sim_link.py` is never
imported.

---------------------------------------------------------------------------------------

## 6. Results

Every script's results live in a `results/` folder it creates next to itself, with a new,
auto-incremented `run_<N>` subfolder each time it is run — nothing is ever overwritten.
Nothing is written outside this `Hardware_Codes` folder.

---------------------------------------------------------------------------------------

## 7. Safety

* Every script homes the joint(s) before engaging its PID/position controller, and holds
  position briefly before handing off, so control begins from a known, settled state.
* Hard position limits, a tracking-error abort, an I2C-failure abort and a glitch-jump
  rejection on the magnetic-encoder reading are all checked **every tick**, before any
  command is computed from a fresh reading. A violation on either joint stops **both**
  motors immediately.
* Ctrl+C always runs the same cleanup as a normal finish: both motors are commanded to
  zero and then released, the CAN bus and I2C bus are closed, and whatever was logged up
  to that point is still saved.
* **Before the first run on real hardware**, verify by hand: the physical wiring matches
  the CAN IDs and I2C addresses in the script (`joint_specifications.txt`), the joint can
  move freely through its full configured range without colliding with anything, and
  `MAGNETIC_ENCODER_SIGN`/`ENCODER_TO_MOTOR_SIGN` have been confirmed as described in each
  script's own module docstring — a wrong motor sign is the one failure mode that is **not**
  caught by the guards above.

---------------------------------------------------------------------------------------

## 8. Troubleshooting

| Problem | Fix |
|---|---|
| `FileNotFoundError` on a dataset CSV | The matching `datasets/<trial>/` subfolder is missing from this copy of `Hardware_Codes`; copy it back in (see [section 2](#2-dataset-paths-are-always-relative-to-this-folder)) |
| `PermissionError` opening `/dev/i2c-1` | `$USER` is not in the `i2c` group yet, or you have not logged out/rebooted since adding it (step 6) |
| `can.exceptions.CanInitializationError` / no `can0` | Steps 2–4 not done, HAT not seated, or the overlay line doesn't match your HAT. Check `dmesg \| grep -i mcp251x` and `ip link show can0` |
| Script hangs at startup doing `sudo ip link set can0 ...` | The user has no passwordless `sudo` for `ip` (step 4's sudoers snippet) |
| `i2cdetect -y 1` shows nothing at `0x40`/`0x41` | Encoder not wired/powered, wrong I2C bus number, or a loose connection — check before trusting any reading |
| `ModuleNotFoundError` for `can`, `smbus2`, `epicallypowerful` or `opensourceleg` | Not using this folder's `.venv` Python; call `.venv/bin/python`, not the system `python3` |
| `pip` cannot find a wheel for `epicallypowerful` | OS is 32-bit (`uname -m` prints `armv7l`); reflash with the 64-bit Raspberry Pi OS image |
| Motor spins the wrong way the instant it's commanded | **Stop immediately (Ctrl+C or power switch).** `ENCODER_TO_MOTOR_SIGN` is wrong for that joint — re-verify it by hand with the motor passive, per that script's docstring, before trying again |
