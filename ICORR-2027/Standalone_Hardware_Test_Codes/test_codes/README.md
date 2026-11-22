# Standalone hardware test codes — Raspberry Pi setup

These scripts drive the knee and ankle CubeMars AK80-9 actuators over CAN, read the
AS5048B magnetic encoders over I2C, and (for two scripts) read an SPI load cell. They are
meant to run **on the Raspberry Pi mounted on the leg**. This README explains how to copy
the folder to a Pi, create the Python environment from `requirements.txt`, and run a script.

## 1. What is in this folder

| Files | Purpose |
|---|---|
| `knee_ankle_trajectory_PID_control_me_<gait>.py` | Combined knee + ankle gait tracking (`normal_walk`, `ramp_ascent`, `ramp_descent`, `stairs_down`, `stairs_up`) |
| `knee_step_test_*.py`, `ankle_step_test_*.py` | Step-response tests |
| `knee_trajectory_servoPositionControl_me_ie.py`, `ankle_trajectory_servoPositionControl_me_ie.py` | Single-joint trajectories in servo position mode |
| `knee_bode_noise_*.py`, `ankle_bode_noise_*.py`, `bode_noise_tools.py`, `bode_noise_overlay.py` | Bode / noise identification and analysis |
| `commanded_current_vs_measured_current_test.py`, `knee_free_swing_test_me.py` | Current-tracking and free-swing tests |
| `knee_resistance_torque_test_PID_control_me_loadcell_d_on_meas.py`, `loadcell_test.py` | Load-cell tests (SPI1) |
| `as5048b_read.py` | Quick encoder read-out (I2C) |
| `normal_walk_*/`, `ramp_*/`, `stairs_*/` | Gait datasets (CSV) the trajectory scripts read |
| `joint_specifications.txt` | I2C addresses, encoder offsets, sign conventions |

The scripts find their dataset folders next to themselves, so copy the **whole folder**
and keep its structure. Each script creates its own `results/...` folder when run.

## 2. Requirements

- Raspberry Pi running **64-bit** Raspberry Pi OS (Bookworm or Trixie) — check `uname -m` prints `aarch64`. `epicallypowerful` only ships a 64-bit ARM wheel.
- **Python 3.11 – 3.13** — check `python3 --version`.
- Internet access for the install (packages come from PyPI and [piwheels](https://www.piwheels.org); nothing is compiled).
- Hardware: CAN HAT (`can0`), AS5048B encoders on I2C bus 1, and for the load cell an SPI1 connection.

## 3. Copy the folder to the Pi

From your computer (replace `pi@raspberrypi.local` and the destination as needed):

```bash
scp -r test_codes pi@raspberrypi.local:~/test_codes
```

or copy it with a USB stick. Do **not** copy a `.venv` folder between machines — create it
fresh on the Pi (next step).

## 4. Create the Python environment and install the packages

On the Pi:

```bash
cd ~/test_codes
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
```

Verify the install:

```bash
.venv/bin/python -c "import can, smbus2, epicallypowerful, opensourceleg, numpy, scipy, pandas, matplotlib; print('OK')"
```

Optionally activate the environment so you can type `python` instead of `.venv/bin/python`:

```bash
source .venv/bin/activate
```

(Use `deactivate` to leave it.) The rest of this README uses `python` assuming it is activated.

## 5. One-time Raspberry Pi system setup

`pip` only installs Python packages. The hardware interfaces must also be enabled in the OS:

1. **I2C** (encoders): `sudo raspi-config` → Interface Options → I2C → Enable. Then check
   `sudo apt install -y i2c-tools && i2cdetect -y 1` — the knee encoder is `0x40`, the ankle `0x41`.
2. **CAN** (motors): add the overlay for your CAN HAT to `/boot/firmware/config.txt`, e.g.
   ```
   dtparam=spi=on
   dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
   ```
   (set `oscillator` to your HAT's crystal), then reboot. See
   `Motor Control/Documentation/CubeMars_AK80-9_V30_Basic_Tutorials/13_Controlling_the_AK80-9_from_a_Raspberry_Pi_5.md`
   for the full walkthrough.
3. **Load cell** (only for `loadcell_test.py` and the resistance-torque test): the scripts
   use SPI1 (`/dev/spidev1.0`), which must be enabled in `config.txt` (e.g. `dtoverlay=spi1-1cs`) and verified with `ls /dev/spidev1.*`.
4. **Bring up the CAN interface** — needed after **every boot** unless you set up the
   systemd service from the tutorial above:
   ```bash
   sudo ip link set can0 up type can bitrate 1000000
   sudo ip link set can0 txqueuelen 65536
   ip -details link show can0      # should show state UP, bitrate 1000000
   ```

## 6. Running the scripts

From the `test_codes` folder, with the environment activated:

```bash
python as5048b_read.py                    # encoder sanity check (Ctrl+C to stop)
python loadcell_test.py --tare            # load cell, with no load on it
python bode_noise_tools.py selftest       # offline analysis check, no hardware
python knee_step_test_PID_control_me.py   # a knee test
python knee_ankle_trajectory_PID_control_me_normal_walk.py
```

Without activating the environment, use `.venv/bin/python <script>.py` instead. Each script
documents its options (`--dry-run`, `--pilot`, etc.) in the header comment — read it before
the first run. Scripts that move motors need `can0` up and the encoders connected.

Plots are saved to files; over SSH or without a display the scripts use the headless
`Agg` matplotlib backend automatically.

## 7. Safety

These scripts command real actuators. Before the first run of any script: mount the leg
so it can move freely, keep the motor power E-stop within reach, start with
`--dry-run` / `--pilot` where offered, and stop with **Ctrl+C**.

## 8. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `No matching distribution found for epicallypowerful` | OS is 32-bit (`uname -m` ≠ `aarch64`) or Python is outside 3.11–3.13 |
| `OSError: [Errno 19] No such device` on CAN / `Network is down` | `can0` is not up — redo step 4 of section 5 |
| `FileNotFoundError: /dev/i2c-1` or `PermissionError` | I2C not enabled, or user not in the `i2c` group: `sudo usermod -aG i2c,spi $USER`, then log out/in |
| `Remote I/O error` reading the encoder | Encoder not powered/wired; `i2cdetect -y 1` should list `0x40`/`0x41` |
| Load-cell script fails to open SPI | SPI1 overlay not enabled (`ls /dev/spidev1.0`) |
| No CAN traffic from the motor | Power-cycle the motor while the Pi stays on, then `candump can0` |
| `ModuleNotFoundError` | The environment is not activated — `source .venv/bin/activate` or use `.venv/bin/python` |
