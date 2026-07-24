# Thigh IMU Logger — MicroStrain 3DM-GV7-AR

Hardware mounting → SensorConnect configuration → Python environment →
relative-quaternion CSV logger → MATLAB/Python plotting.

This is the **standalone logging** pipeline (`micro_strain_front_mount.py`), which
writes straight to CSV per trial. It is a sibling to a WebSocket/live-dashboard
version (`imu_dashboard_ws.py`) — same math, different output path. If you
just want to run a walking trial and get a CSV + plots, this is the one to use.

The sensor is mounted **vertically on the thigh** for this pipeline.

This README has three parts:

- **[Part 1 — Hardware & SensorConnect](#part-1--hardware--sensorconnect)**:
  mounting the sensor and getting it configured and streaming the right
  channels via MicroStrain's official tool. Do this once per sensor/mount.
- **[Part 2 — One-Time Calibration](#part-2--one-time-calibration)**:
  using `calibrate_imu.py` to save a reference quaternion to a JSON file so
  `micro_strain_front_mount.py` doesn't need to re-calibrate every run.
- **[Part 3 — Logger Software](#part-3--logger-software)**: installing the
  Python environment, understanding how `micro_strain_front_mount.py` works, running
  it, plotting results, and troubleshooting. Do this per machine / per trial.

---

# Part 1 — Hardware & SensorConnect

*Do this once per sensor/mount — get the hardware physically set up and
the right channels streaming from SensorConnect. Section numbers below
restart at 1 in each part, so each part can be followed independently.*

## 1. What you need

- MicroStrain 3DM-GV7-AR sensor + its official cable/connector
- A PC (Windows recommended) for initial configuration via SensorConnect
- A Raspberry Pi (or any Linux machine) to run the logger
- USB cable (sensor → adapter → USB) or RS-232/RS-422 wiring
- Python 3.9+ on the logging machine

---

## 2. Mount the sensor correctly

- Mounted **vertically on the thigh** for this pipeline.
- **Vent must NOT face up.** Mounting it vent-up can compromise the
  IP68 seal or clog airflow to the internal pressure sensor.
- Use the correct MicroStrain cable — the IP68 rating only holds if
  the cable is installed per the mechanical spec.
- Only power it within its rated voltage range. Reversed polarity can
  permanently damage the unit.
- Default axis convention: **X = direction of travel, Z = down**. Exact
  mounting angle on the thigh doesn't matter for this logger — see
  Part 3, Section 3, since it calibrates against whatever pose you're standing in.

---

## 3. Install SensorConnect (on your PC first)

1. Download it: **microstrain.com/software/sensorconnect**
2. Install and launch it.
3. Plug the GV7-AR into your PC via USB. SensorConnect should
   auto-detect it and show the model name and serial number.
4. Check the **Firmware/Downloads** section on the GV7-AR product page
   and upgrade firmware if needed — SensorConnect handles this in-app.

---

## 4. Configure the sensor in SensorConnect

This logger needs **one specific channel enabled** that a generic Euler-angle
setup won't give you by default:

1. **Sampling tile** — enable:
   - **Estimation Filter → Attitude (Quaternion)** — `estAttitudeQuaternion`
     (this is the channel `micro_strain_front_mount.py` actually reads; plain
     roll/pitch/yaw Euler output is not used by this script)
   - **Estimated Angular Rate**, specifically the axis you'll set as
     `THIGH_AXIS` in the script (default `y` — see Part 3, Section 2)
   Set your sample rate (up to 1000 Hz on the GV7-AR; 100 Hz is the
   default this script's loop timing assumes).
2. **Mounting transform** — not required for this script. Because it
   works in relative-quaternion space (Part 3, Section 3), an arbitrary mount
   angle is corrected out automatically at calibration time.
3. **Capture gyro bias** — do this once after mounting, sensor and leg
   completely stationary. Save to non-volatile memory so it survives
   power cycles.
4. Watch the live graphs for a minute and confirm the quaternion
   channel is populated and angular rate looks sane at rest (~0).
5. **Disconnect SensorConnect** before running the Python script — only
   one program can hold the serial port at a time.

---

---

# Part 2 — One-Time Calibration

*Run this once (per session, per remount) to save a reference quaternion to
a JSON file. `micro_strain_front_mount.py` can then load it with
`--calibration <file>.json` instead of prompting you to stand still and
re-calibrating every single run.*

## 1. What it does

`calibrate_imu.py` connects to the sensor, asks you to **stand neutral and
hold still**, buffers incoming quaternions, and once it detects a
sufficiently still window (max angular deviation within the buffer under
`CALIBRATION_STILL_STD_DEG * 3`), it saves that quaternion as the reference
pose. If it never detects stillness within `CALIBRATION_TIMEOUT_S`, it falls
back to the last reading and warns you.

## 2. Running it

```bash
pip install python-mscl --break-system-packages

python3 calibrate_imu.py
# or, with a custom output filename:
python3 calibrate_imu.py --out my_calibration.json
```

Stand neutral and hold still when prompted. Re-run this any time you want a
fresh zero (new session, sensor remounted, etc).

## 3. Output file format

The script writes a JSON file (default name `imu_side_mounted_calibration.json`)
shaped like this:

```json
{
  "reference_quaternion": {
    "w": 0.7305218577384949,
    "x": -0.03269679844379425,
    "y": 0.6811332702636719,
    "z": 0.036416616290807724
  },
  "calibrated_at": "2026-07-13 12:47:45"
}
```

- `reference_quaternion` — the `(w, x, y, z)` orientation captured while you
  stood still and neutral; this is what every subsequent quaternion is
  measured relative to.
- `calibrated_at` — local timestamp (`%Y-%m-%d %H:%M:%S`) of when the
  calibration was captured, for traceability across trials/sessions.

## 4. Using the saved calibration

Pass the file straight to the logger to skip live calibration:

```bash
python3 micro_strain_front_mount.py --calibration my_calibration.json
```

---

---

# Part 3 — Logger Software

*Do this per logging machine (once) and per trial (running the script).
Assumes Part 1 is already done: sensor is mounted, SensorConnect is
configured with the quaternion channel enabled, gyro bias is captured, and
SensorConnect itself is closed so the serial port is free.*

## 1. Set up the Raspberry Pi (or Linux logging machine)

```bash
mkdir gv7_project
cd gv7_project

# 1. Create a virtual environment
python3 -m venv sensor_env
source sensor_env/bin/activate

# 2. Install Python dependencies
pip install python-mscl matplotlib --break-system-packages

# 3. Give your user permission to access the serial port
sudo usermod -a -G dialout $USER
# then log out and back in (or reboot) for the group change to apply

# 4. Plug in the sensor and confirm the device shows up
ls -l /dev/ttyACM0
```

If your sensor enumerates as a different port (e.g. `/dev/ttyUSB0` or
`COM5` on Windows), update `SERIAL_PORT` near the top of
`micro_strain_front_mount.py` (and in `calibrate_imu.py` if you use it).

> **Note on MSCL install methods.** MicroStrain's official MSCL is
> archived (last release `v68.1.0`) and traditionally ships as a
> platform-specific `.deb` package (see the general GV7-AR setup guide
> for that path — download from
> [MSCL releases](https://github.com/LORD-MicroStrain/MSCL/releases),
> `dpkg -i`, then `sys.path.append('/usr/share/python3-mscl/')` before
> `import mscl`). **This script does not use that path.** It imports
> `from python_mscl import mscl`, i.e. the `python-mscl` **PyPI**
> package, installed directly into your venv with `pip install
> python-mscl`. Don't mix the two — if you already have the `.deb`
> version installed system-wide, that's a separate, unrelated install
> from the one this script needs. If `pip install python-mscl` isn't
> available for your architecture, fall back to the `.deb` +
> `sys.path.append` approach and change the import line accordingly.

---

## 2. Configuration you may need to change

These live near the top of `micro_strain_front_mount.py`:

| Constant | Meaning | Default |
|---|---|---|
| `SERIAL_PORT` | Serial device path | `/dev/ttyACM0` |
| `BAUD_RATE` | Serial baud rate | `115200` |
| `THIGH_AXIS` | Which quaternion-relative axis is thigh flexion/extension | `"y"` (confirmed 2026‑07‑07: swing axis is ~97% Y) |
| `PITCH_SIGN` | Flips sign so flexion is positive/negative as you expect | `-1.0` |
| `CALIBRATION_SECONDS` | Target length of the "hold still" calibration window | `3.0` |
| `CALIBRATION_STILL_STD_DEG` | Max allowed wobble (×3) during calibration to accept it as "still" | `0.5°` |
| `CALIBRATION_TIMEOUT_S` | Give up waiting for stillness after this long, use best-effort reference | `15.0` |
| `ALPHA_GYRO` | Low-pass filter strength on the *derived* angular rate | `0.04` |
| `A_PITCH`, `B_GYRO` | Phase-portrait normalization constants | `23.7°`, `150.8°/s` |
| `MIN_RADIUS` | Below this phase-portrait radius, phase is undefined (near-zero motion) | `0.05` |

**Important:** `A_PITCH` and `B_GYRO` are carried over from an existing,
known-good **MPU9250** pipeline. They are a placeholder, not a
calibrated value for the GV7-AR — the GV7-AR's actual pitch range and
noise characteristics are different, so re-derive these constants from
a real GV7-AR walking trial before trusting `phase_var` for anything
quantitative.

If `THIGH_AXIS` is changed to `"x"` or `"z"`, note that `roll_deg` /
`yaw_deg` in the output are just "whichever of the other two axes isn't
the thigh axis" — they are not general-purpose roll/yaw, they're
leftover/secondary axes for reference only.

---

## 3. How the logger actually works

This is the part that differs most from a "typical" IMU logger, so it's
worth walking through explicitly.

### 3.1 Why quaternions, and why *relative*

A naive logger reads the sensor's Euler roll/pitch/yaw directly. That's fine
near level orientations, but it has two problems for a thigh-mounted sensor:
1. **Gimbal lock** — Euler angles become unstable/discontinuous near ±90°
   pitch, which is easy to hit if the sensor isn't mounted perfectly
   axis-aligned, or during large leg swings.
2. **Mount angle sensitivity** — the sensor's raw orientation depends on
   exactly how it's strapped to the leg, which varies trial to trial.

This script sidesteps both by:
1. Reading the sensor's **quaternion** attitude estimate (`estAttitudeQuaternion`
   or equivalent, via `read_estimation_filter()`), never the sensor's own
   Euler output.
2. **Calibrating a reference quaternion** (`calibrate_reference_quat()`, or
   loading one saved earlier by `calibrate_imu.py` — see Part 2) while you
   stand still in a neutral pose, whatever that pose's absolute orientation
   happens to be.
3. At every timestep, computing the **relative rotation** between the
   current quaternion and that reference (`quat_relative_euler_deg()`), and
   converting *that* to Euler angles.

Because the relative rotation only needs to represent normal gait
range-of-motion (tens of degrees), it stays far away from the ±90° pitch
singularity regardless of how the sensor is actually strapped on — that's
the "avoids gimbal lock regardless of mount angle" behavior mentioned in
the script's docstring.

### 3.2 Calibration loop

`calibrate_reference_quat()` (used live by `micro_strain_front_mount.py`, and
by `calibrate_imu.py` in Part 2 to produce a reusable JSON file):
- Buffers incoming quaternions for `CALIBRATION_SECONDS` worth of samples.
- Once the buffer is full, checks the **maximum angular deviation** between
  the most recent sample and every other sample in the window (via
  quaternion difference → `2 * acos(|w|)`).
- If that max deviation is within `CALIBRATION_STILL_STD_DEG * 3`, it accepts
  the most recent sample as the reference pose ("you were standing still
  enough").
- If you never hold still enough within `CALIBRATION_TIMEOUT_S`, it gives up
  and uses whatever the last reading was — printing a warning rather than
  hanging forever.
- There's no live re-zero mid-run in this version (unlike
  `imu_dashboard_ws.py`) — it's meant for one calibration, then one clean
  trial. Re-run the script per trial for a fresh calibration, or reuse a
  saved JSON file via `--calibration`.

You can skip live calibration entirely with `--calibration path.json`, which
loads a previously saved reference quaternion (as written by
`calibrate_imu.py`, Part 2) instead of prompting you to stand still.

### 3.3 Main logging loop

`log_session()` runs a **fixed-cadence** loop (default 100 Hz / 10 ms period):
- Reads the latest quaternion + this axis's angular rate.
- Converts quaternion → relative Euler angles via the calibration reference.
- Maps whichever axis is `THIGH_AXIS` to `pitch_deg` (with `PITCH_SIGN`
  applied), and stashes the other two axes into `roll_deg`/`yaw_deg` for
  reference.
- Computes **two independent angular rate estimates** for the same axis:
  - `sensor_ang_vel_deg_s` — straight from the sensor's own angular rate
    channel (e.g. `estAngularRateY`).
  - `derived_ang_vel_deg_s` — numerically differentiated from
    consecutive `pitch_deg` values (`Δpitch / Δt`), then smoothed with an
    exponential filter (`ALPHA_GYRO`).
  These are logged **side by side, deliberately**, so you can compare which
  one is more trustworthy for the GV7-AR rather than assuming one is
  correct — the docstring is explicit that this is an open question being
  tested, not settled.
- Computes a **phase variable** (`compute_phase_var()`) from the
  pitch/derived-rate pair, normalized into a "phase portrait" circle using
  `A_PITCH`/`B_GYRO`, then mapped to `atan2` → 0–1. This is meant to look
  like a repeating sawtooth over a normal walking gait cycle. If the
  portrait radius is too small (`< MIN_RADIUS`, i.e. near-zero pitch and
  rate — standing still), phase is left undefined and just holds the last
  valid value rather than jittering.
- The loop uses **absolute scheduling** (`next_tick += loop_period_s`, sleep
  only the remainder) rather than a flat `sleep()`, so timing doesn't drift
  and doesn't try to "catch up" in a burst if a single iteration runs long.
- Writes are buffered by Python's file object and explicitly `flush()`ed
  every 200 rows, so data survives even if you kill the process, and the
  file is always flushed and closed in the `finally` block — including on
  Ctrl+C.

### 3.4 Output columns

```
t, roll_deg, pitch_deg, yaw_deg, sensor_ang_vel_deg_s, derived_ang_vel_deg_s, phase_var
```

- `t` — raw Unix timestamp (seconds), **not zeroed** in the CSV itself;
  both the built-in plotter and the MATLAB script zero it at load time.
- `pitch_deg` — thigh flexion/extension relative to the calibrated neutral
  pose (sign per `PITCH_SIGN`).
- `roll_deg` / `yaw_deg` — the two non-thigh axes, for reference/debugging.
- `sensor_ang_vel_deg_s` — sensor's own rate channel for the thigh axis.
- `derived_ang_vel_deg_s` — filtered numerical derivative of `pitch_deg`.
- `phase_var` — 0–1 gait phase estimate; empty/held-over where undefined.

---

## 4. Running it

```bash
source sensor_env/bin/activate

# Default: 10-second trial, auto-named CSV, plots afterward
python3 micro_strain_front_mount.py

# Custom filename and duration
python3 micro_strain_front_mount.py --out walking_trial_1.csv --seconds 15

# Skip live calibration, reuse a saved reference pose (see Part 2)
python3 micro_strain_front_mount.py --calibration ref_pose.json

# Skip auto-plotting
python3 micro_strain_front_mount.py --no-plot
```

What happens, in order:
1. Connects to the sensor (`setup_imu()`).
2. Calibrates (or loads) the reference quaternion — **stand neutral and
   hold still when prompted**.
3. Runs a fixed 3-second stabilization window (discarding samples) before
   logging starts — lets the estimator settle right after calibration.
4. Logs at the configured cadence until `--seconds` elapses or Ctrl+C.
5. Unless `--no-plot`, generates a 4-panel PNG (`plot_csv()`):
   pitch vs. time, sensor-vs-derived angular rate, phase portrait, and
   phase variable vs. time — saved as `<csv_name>_plots.png` next to the CSV.

If the output file already exists, you'll be prompted before it's
overwritten — no silent data loss.

---

## 5. Plotting in MATLAB instead

If you'd rather use MATLAB (e.g. to integrate with existing analysis
scripts), `microstrain_matlab_plot.m` reads the same CSV and produces the
same four plots as separate figures:

```matlab
analyze_walking_trial('walking_trial_20260710_145719.csv')
```

It zeros the timestamp the same way the Python plotter does
(`t = T.t - T.t(1)`), and expects the exact column names this logger
writes (`pitch_deg`, `sensor_ang_vel_deg_s`, `derived_ang_vel_deg_s`,
`phase_var`). If you rename columns or change `FIELDNAMES` in the Python
script, update the MATLAB script's field references to match.

---

## 6. Troubleshooting

| Symptom | Likely cause |
|---|---|
| `ModuleNotFoundError: No module named 'python_mscl'` | `pip install python-mscl --break-system-packages` wasn't run, or you're not inside `sensor_env` |
| Script hangs at "Calibrating: stand neutral..." | No quaternion data arriving — confirm `Attitude (Quaternion)` is enabled in SensorConnect's Sampling tile, and that SensorConnect itself is disconnected (only one program can hold the serial port) |
| Calibration warns "never found a still window" | You moved during the 3s calibration window, or `CALIBRATION_STILL_STD_DEG` is too tight — hold genuinely still, or loosen the threshold |
| `pitch_deg` seems to swing the wrong axis | `THIGH_AXIS` doesn't match how the sensor is actually mounted — re-check against a known movement and swap to `"x"`/`"z"` if needed |
| `phase_var` looks nothing like a sawtooth | `A_PITCH`/`B_GYRO` are still MPU9250-derived placeholders — recalibrate them against real GV7-AR trial data (Part 3, Section 2 above) |
| `sensor_ang_vel_deg_s` and `derived_ang_vel_deg_s` disagree a lot | Expected during initial GV7-AR bring-up — this is exactly why both are logged; compare across a few trials before picking one to trust |
| CSV has gaps or looks noisy at first | Gyro bias not captured in SensorConnect yet, or sensor still settling — the 3s post-calibration stabilization window helps but isn't a substitute for capturing gyro bias in Part 1, Section 4 |
| Existing file silently missing after re-run | It isn't silent — you're prompted `[y/N]` before overwrite; answering anything but `y` aborts without touching the file |

---

## 7. Reference links

- SensorConnect: https://microstrain.com/software/sensorconnect
- MSCL (archived, `.deb`-based, v68.1.0 final): https://github.com/LORD-MicroStrain/MSCL/releases
- `python-mscl` on PyPI (what this script actually uses): `pip install python-mscl`
- MIP SDK (lower-level C/C++, MicroStrain's forward-looking recommendation): https://github.com/LORD-MicroStrain/mip_sdk
- GV7 User Manual: https://s3.amazonaws.com/files.microstrain.com/GV7_User_Manual/user_manual_content/

---

## Appendix A: `calibrate_imu.py` (full source)

```python
"""
Standalone calibration - run this once, save the reference quaternion to a
file. micro_strain_front_mount.py can then load it instead of re-calibrating every
single run.

Requires:
    pip install python-mscl --break-system-packages

Usage:
    python3 calibrate_imu.py
    python3 calibrate_imu.py --out my_calibration.json

Stand neutral and hold still when prompted. Re-run this any time you want
a fresh zero (new session, sensor remounted, etc).
"""

import argparse
import json
import math
import os
import time

from python_mscl import mscl

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

RAD_TO_DEG = 180.0 / math.pi

CALIBRATION_SECONDS = 3.0
CALIBRATION_STILL_STD_DEG = 0.5
CALIBRATION_TIMEOUT_S = 15.0

DEFAULT_CALIBRATION_FILE = "imu_side_mounted_calibration.json"


def _vector_to_wxyz(vec):
    if hasattr(vec, "as_floatAt"):
        return (vec.as_floatAt(0), vec.as_floatAt(1), vec.as_floatAt(2), vec.as_floatAt(3))
    if hasattr(vec, "as_doubleAt"):
        return (vec.as_doubleAt(0), vec.as_doubleAt(1), vec.as_doubleAt(2), vec.as_doubleAt(3))
    if hasattr(vec, "data"):
        d = vec.data()
        return (d[0], d[1], d[2], d[3])
    return (vec[0], vec[1], vec[2], vec[3])


def setup_imu():
    connection = mscl.Connection.Serial(SERIAL_PORT, BAUD_RATE)
    node = mscl.InertialNode(connection)
    node.resume()
    return node


def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def quat_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def read_quat(node):
    packets = node.getDataPackets(500)
    qw = qx = qy = qz = None

    QUAT_VECTOR_NAMES = ("estAttitudeQuaternion", "estOrientQuaternion", "estQuaternion")
    QUAT_SCALAR_NAMES = {
        "w": ("estQuaternionW", "estAttitudeQuaternionW"),
        "x": ("estQuaternionX", "estAttitudeQuaternionX"),
        "y": ("estQuaternionY", "estAttitudeQuaternionY"),
        "z": ("estQuaternionZ", "estAttitudeQuaternionZ"),
    }

    for packet in packets:
        for point in packet.data():
            name = point.channelName()
            if name in QUAT_VECTOR_NAMES:
                try:
                    q = point.as_Vector()
                    qw, qx, qy, qz = _vector_to_wxyz(q)
                except (AttributeError, TypeError):
                    pass
            elif name in QUAT_SCALAR_NAMES["w"]:
                qw = point.as_float()
            elif name in QUAT_SCALAR_NAMES["x"]:
                qx = point.as_float()
            elif name in QUAT_SCALAR_NAMES["y"]:
                qy = point.as_float()
            elif name in QUAT_SCALAR_NAMES["z"]:
                qz = point.as_float()

    if None not in (qw, qx, qy, qz):
        return (qw, qx, qy, qz)
    return None


def calibrate_reference_quat(node, duration_s):
    print(f"Calibrating: stand neutral and hold still for {duration_s:.0f}s...")
    buffer = []
    start = time.time()

    while True:
        elapsed = time.time() - start
        if elapsed > CALIBRATION_TIMEOUT_S:
            if buffer:
                print(
                    f"Warning: never found a still window within {CALIBRATION_TIMEOUT_S:.0f}s. "
                    f"Using last reading as best-effort reference."
                )
                return buffer[-1]
            print("Warning: no samples received during calibration, using identity reference.")
            return (1.0, 0.0, 0.0, 0.0)

        quat = read_quat(node)
        if quat is None:
            continue

        buffer.append(quat)

        max_len = max(int(duration_s / 0.005), 10)
        if len(buffer) > max_len:
            buffer.pop(0)

        if len(buffer) >= max_len:
            q_last = buffer[-1]
            max_dev_deg = 0.0
            for q in buffer:
                q_rel = quat_mul(quat_conj(q_last), q)
                w = max(-1.0, min(1.0, q_rel[0]))
                dev = 2 * math.acos(abs(w)) * RAD_TO_DEG
                max_dev_deg = max(max_dev_deg, dev)

            if max_dev_deg <= CALIBRATION_STILL_STD_DEG * 3:
                print(
                    f"Calibration complete. Reference quaternion = "
                    f"({q_last[0]:.4f}, {q_last[1]:.4f}, {q_last[2]:.4f}, {q_last[3]:.4f}) "
                    f"(max deviation in window: {max_dev_deg:.2f} deg)."
                )
                return q_last


def main():
    parser = argparse.ArgumentParser(description="Calibrate IMU and save reference quaternion to a file.")
    parser.add_argument("--out", default=DEFAULT_CALIBRATION_FILE, help=f"Output file (default: {DEFAULT_CALIBRATION_FILE})")
    args = parser.parse_args()

    node = setup_imu()
    ref_quat = calibrate_reference_quat(node, CALIBRATION_SECONDS)

    data = {
        "reference_quaternion": {"w": ref_quat[0], "x": ref_quat[1], "y": ref_quat[2], "z": ref_quat[3]},
        "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }

    with open(args.out, "w") as f:
        json.dump(data, f, indent=2)

    print(f"\nSaved calibration to: {os.path.abspath(args.out)}")
    print("Use this with: python3 micro_strain_front_mount.py --calibration " + args.out)


if __name__ == "__main__":
    main()
```

---

## Appendix B: `micro_strain_front_mount.py` (full source)

```python
"""
Thigh IMU logger - direct to CSV, using the relative-quaternion pipeline.

Same math as imu_dashboard_ws.py (relative rotation from calibration pose,
not fixed-frame Euler - avoids gimbal lock regardless of mount angle), but
writes straight to CSV instead of streaming over WebSocket.

Requires:
    pip install python-mscl --break-system-packages

Sensor Connect setup:
    Enable the Estimation Filter "Attitude (Quaternion)" channel.

Usage:
    python3 micro_strain_front_mount.py
    python3 micro_strain_front_mount.py --out walking_trial_1.csv
    python3 micro_strain_front_mount.py --seconds 15

Calibration:
    Stand neutral and hold still when prompted. Re-run the script for a
    fresh calibration each trial (no live re-zero here, unlike the
    dashboard version - this is meant for quick standalone logging runs).

Stop early any time with Ctrl+C - the file is flushed as you go.

Output columns:
    t,roll_deg,pitch_deg,yaw_deg,ang_vel_deg_s,phase_var
"""

import argparse
import csv
import json
import math
import os
import time

import matplotlib
matplotlib.use("Agg")  # no display over SSH - save PNG only, don't try to open a window
import matplotlib.pyplot as plt
from python_mscl import mscl

# ---- Configuration (mirrors imu_dashboard_ws.py) ----
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

RAD_TO_DEG = 180.0 / math.pi

PITCH_SIGN = -1.0

# Which axis is thigh flexion/extension. CONFIRMED 2026-07-07: swing axis is ~97% Y.
THIGH_AXIS = "y"

ANGULAR_RATE_CHANNEL = {
    "x": "estAngularRateX",
    "y": "estAngularRateY",
    "z": "estAngularRateZ",
}[THIGH_AXIS]

CALIBRATION_SECONDS = 3.0
CALIBRATION_STILL_STD_DEG = 0.5
CALIBRATION_TIMEOUT_S = 15.0

ALPHA_GYRO = 0.04

# Phase portrait normalization - matches known-good MPU9250 pipeline values.
A_PITCH = 23.7     # deg
B_GYRO = 150.8     # deg/s

MIN_RADIUS = 0.05

FIELDNAMES = ["t", "roll_deg", "pitch_deg", "yaw_deg", "sensor_ang_vel_deg_s", "derived_ang_vel_deg_s", "phase_var"]


def _vector_to_wxyz(vec):
    if hasattr(vec, "as_floatAt"):
        return (vec.as_floatAt(0), vec.as_floatAt(1), vec.as_floatAt(2), vec.as_floatAt(3))
    if hasattr(vec, "as_doubleAt"):
        return (vec.as_doubleAt(0), vec.as_doubleAt(1), vec.as_doubleAt(2), vec.as_doubleAt(3))
    if hasattr(vec, "data"):
        d = vec.data()
        return (d[0], d[1], d[2], d[3])
    return (vec[0], vec[1], vec[2], vec[3])


def setup_imu():
    connection = mscl.Connection.Serial(SERIAL_PORT, BAUD_RATE)
    node = mscl.InertialNode(connection)
    node.resume()
    return node


def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def quat_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_relative_euler_deg(q_ref, q_now):
    """Full independent roll/pitch/yaw (deg) of q_now relative to q_ref."""
    q_rel = quat_mul(quat_conj(q_ref), q_now)
    w, x, y, z = q_rel

    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def read_estimation_filter(node):
    packets = node.getDataPackets(500)
    qw = qx = qy = qz = None
    rate = None

    QUAT_VECTOR_NAMES = ("estAttitudeQuaternion", "estOrientQuaternion", "estQuaternion")
    QUAT_SCALAR_NAMES = {
        "w": ("estQuaternionW", "estAttitudeQuaternionW"),
        "x": ("estQuaternionX", "estAttitudeQuaternionX"),
        "y": ("estQuaternionY", "estAttitudeQuaternionY"),
        "z": ("estQuaternionZ", "estAttitudeQuaternionZ"),
    }

    for packet in packets:
        for point in packet.data():
            name = point.channelName()
            if name in QUAT_VECTOR_NAMES:
                try:
                    q = point.as_Vector()
                    qw, qx, qy, qz = _vector_to_wxyz(q)
                except (AttributeError, TypeError):
                    pass
            elif name in QUAT_SCALAR_NAMES["w"]:
                qw = point.as_float()
            elif name in QUAT_SCALAR_NAMES["x"]:
                qx = point.as_float()
            elif name in QUAT_SCALAR_NAMES["y"]:
                qy = point.as_float()
            elif name in QUAT_SCALAR_NAMES["z"]:
                qz = point.as_float()
            elif name == ANGULAR_RATE_CHANNEL:
                rate = point.as_float()

    quat = None
    if None not in (qw, qx, qy, qz):
        quat = (qw, qx, qy, qz)

    return quat, rate


def calibrate_reference_quat(node, duration_s):
    print(f"Calibrating: stand neutral and hold still for {duration_s:.0f}s...")
    buffer = []
    start = time.time()

    while True:
        elapsed = time.time() - start
        if elapsed > CALIBRATION_TIMEOUT_S:
            if buffer:
                print(
                    f"Warning: never found a still window within {CALIBRATION_TIMEOUT_S:.0f}s. "
                    f"Using last reading as best-effort reference."
                )
                return buffer[-1]
            print("Warning: no samples received during calibration, using identity reference.")
            return (1.0, 0.0, 0.0, 0.0)

        quat, _ = read_estimation_filter(node)
        if quat is None:
            continue

        buffer.append(quat)

        max_len = max(int(duration_s / 0.005), 10)
        if len(buffer) > max_len:
            buffer.pop(0)

        if len(buffer) >= max_len:
            q_last = buffer[-1]
            max_dev_deg = 0.0
            for q in buffer:
                q_rel = quat_mul(quat_conj(q_last), q)
                w = max(-1.0, min(1.0, q_rel[0]))
                dev = 2 * math.acos(abs(w)) * RAD_TO_DEG
                max_dev_deg = max(max_dev_deg, dev)

            if max_dev_deg <= CALIBRATION_STILL_STD_DEG * 3:
                print(
                    f"Calibration complete. Reference quaternion = "
                    f"({q_last[0]:.4f}, {q_last[1]:.4f}, {q_last[2]:.4f}, {q_last[3]:.4f}) "
                    f"(max deviation in window: {max_dev_deg:.2f} deg)."
                )
                return q_last


def compute_phase_var(pitch_centered_deg, rate_deg_s):
    x = pitch_centered_deg / A_PITCH
    y = rate_deg_s / B_GYRO

    radius = math.sqrt(x * x + y * y)
    if radius < MIN_RADIUS:
        return None

    angle = math.atan2(y, x)
    phase = (angle + math.pi) / (2 * math.pi)
    return 1-phase


def load_calibration(path):
    """Loads a reference quaternion saved by calibrate_imu.py."""
    with open(path) as f:
        data = json.load(f)
    q = data["reference_quaternion"]
    print(f"Loaded calibration from {path} (calibrated at {data.get('calibrated_at', 'unknown')})")
    return (q["w"], q["x"], q["y"], q["z"])


def plot_csv(csv_path):
    """
    Reads the CSV just written and plots pitch, BOTH angular velocity
    sources (sensor vs. derived) overlaid for direct comparison, plus the
    phase portrait and phase variable.
    """
    t, pitch, sensor_rate, derived_rate, phase = [], [], [], [], []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["t"]))
            pitch.append(float(row["pitch_deg"]))
            sensor_rate.append(float(row["sensor_ang_vel_deg_s"]) if row["sensor_ang_vel_deg_s"] else float("nan"))
            derived_rate.append(float(row["derived_ang_vel_deg_s"]) if row["derived_ang_vel_deg_s"] else float("nan"))
            phase.append(float(row["phase_var"]) if row["phase_var"] else float("nan"))

    t0 = t[0]
    t = [ti - t0 for ti in t]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(f"Thigh IMU trial - {os.path.basename(csv_path)}")

    axes[0, 0].plot(t, pitch, color="tab:blue")
    axes[0, 0].set_title("Pitch over time")
    axes[0, 0].set_xlabel("Time (s)")
    axes[0, 0].set_ylabel("Pitch (deg)")
    axes[0, 0].grid(True)

    axes[0, 1].plot(t, sensor_rate, color="tab:red", label="Sensor (estAngularRateY)", alpha=0.7)
    axes[0, 1].plot(t, derived_rate, color="tab:green", label="Derived (d(pitch)/dt, filtered)", alpha=0.7)
    axes[0, 1].set_title("Angular velocity: sensor vs. derived")
    axes[0, 1].set_xlabel("Time (s)")
    axes[0, 1].set_ylabel("Ang. vel (deg/s)")
    axes[0, 1].legend(fontsize=8)
    axes[0, 1].grid(True)

    axes[1, 0].plot(pitch, derived_rate, ".", markersize=2, color="tab:orange")
    axes[1, 0].set_title("Phase portrait (pitch vs. derived ang. vel.)")
    axes[1, 0].set_xlabel("Pitch (deg)")
    axes[1, 0].set_ylabel("Ang. vel (deg/s)")
    axes[1, 0].axis("equal")
    axes[1, 0].grid(True)

    axes[1, 1].plot(t, phase, "-", markersize=2, color="tab:cyan")
    axes[1, 1].set_title("Phase variable over time")
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].set_ylabel("Phase (0-1)")
    axes[1, 1].set_ylim(0, 1)
    axes[1, 1].grid(True)

    plt.tight_layout()

    png_path = os.path.splitext(csv_path)[0] + "_plots.png"
    plt.savefig(png_path, dpi=150)
    print(f"Saved plots to: {os.path.abspath(png_path)}")
    print("(No display available over SSH - copy this PNG to your laptop with scp to view it.)")


def default_filename():
    return f"walking_trial_{time.strftime('%Y%m%d_%H%M%S')}.csv"


def log_session(node, ref_quat, out_path, max_seconds=None, loop_period_s=0.01):
    """
    loop_period_s: target fixed time between samples (default 10ms = 100Hz).
    Uses a fixed-cadence loop (sleeps for whatever time remains after
    processing, rather than a flat sleep(0.005)) to test whether the
    sensor's own estAngularRateY channel becomes trustworthy under more
    uniform timing than the previous logger used.

    Logs BOTH the sensor's reported rate and our own pitch-derivative rate
    side by side, so they can be compared directly instead of guessing
    which one to trust.
    """
    abs_out_path = os.path.abspath(out_path)
    print("=" * 60)
    print(f"  SAVING TO: {abs_out_path}")
    print(f"  Target loop period: {loop_period_s*1000:.0f} ms")
    print("=" * 60)
    print("Press Ctrl+C to stop." + (f" (auto-stops after {max_seconds}s)" if max_seconds else ""))

    last_phase = None
    gy_filt = 0.0
    have_gy_filt = False
    prev_t = None
    prev_pitch = None
    row_count = 0
    start = time.time()

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(FIELDNAMES)

        try:
            next_tick = time.time()
            while True:
                if max_seconds is not None and (time.time() - start) > max_seconds:
                    break

                quat, sensor_rate_rad_s = read_estimation_filter(node)
                now = time.time()

                if quat is not None:
                    roll_raw, pitch_raw, yaw_raw = quat_relative_euler_deg(ref_quat, quat)
                    angle_by_axis = {"x": roll_raw, "y": pitch_raw, "z": yaw_raw}
                    pitch_deg = PITCH_SIGN * angle_by_axis[THIGH_AXIS]
                    roll_deg = angle_by_axis["x"] if THIGH_AXIS != "x" else angle_by_axis["y"]
                    yaw_deg = angle_by_axis["z"] if THIGH_AXIS != "z" else angle_by_axis["y"]

                    sensor_rate_deg_s = (
                        PITCH_SIGN * sensor_rate_rad_s * RAD_TO_DEG
                        if sensor_rate_rad_s is not None else ""
                    )

                    derived_rate_deg_s = ""
                    if prev_t is not None:
                        dt = now - prev_t
                        if dt > 0:
                            derived_rate_deg_s = (pitch_deg - prev_pitch) / dt
                            if not have_gy_filt:
                                gy_filt = derived_rate_deg_s
                                have_gy_filt = True
                            else:
                                gy_filt = ALPHA_GYRO * derived_rate_deg_s + (1 - ALPHA_GYRO) * gy_filt

                    prev_t = now
                    prev_pitch = pitch_deg

                    phase = ""
                    if derived_rate_deg_s != "":
                        p = compute_phase_var(pitch_deg, gy_filt)
                        if p is None:
                            p = last_phase
                        else:
                            last_phase = p
                        phase = p if p is not None else ""

                    writer.writerow([
                        time.time(), roll_deg, pitch_deg, yaw_deg,
                        sensor_rate_deg_s, derived_rate_deg_s, phase,
                    ])
                    row_count += 1
                    if row_count % 200 == 0:
                        f.flush()
                        print(f"  ...{row_count} samples logged", end="\r")

                next_tick += loop_period_s
                sleep_time = next_tick - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    # We're behind schedule - don't sleep, and resync next_tick
                    # so we don't try to "catch up" with a burst of fast loops.
                    next_tick = time.time()

        except KeyboardInterrupt:
            pass
        finally:
            f.flush()
            print(f"\nDone. {row_count} samples written to {abs_out_path}")


def main():
    parser = argparse.ArgumentParser(description="Log thigh IMU data (relative-quaternion pipeline) to CSV.")
    parser.add_argument("--out", default=None, help="Output CSV path (default: timestamped filename)")
    parser.add_argument("--seconds", type=float, default=10.0, help="Auto-stop after N seconds (default: 10s)")
    parser.add_argument("--calibration", default=None, help="Path to a saved calibration file from calibrate_imu.py (skips live calibration)")
    parser.add_argument("--no-plot", action="store_true", help="Skip plotting after logging finishes")
    args = parser.parse_args()

    out_path = args.out or default_filename()
    abs_out_path = os.path.abspath(out_path)

    if os.path.exists(out_path):
        answer = input(f"WARNING: {abs_out_path} already exists and will be overwritten. Continue? [y/N]: ")
        if answer.strip().lower() != "y":
            print("Aborted - no data was overwritten.")
            return

    node = setup_imu()

    if args.calibration:
        ref_quat = load_calibration(args.calibration)
    else:
        ref_quat = calibrate_reference_quat(node, CALIBRATION_SECONDS)

    STABILIZE_SECONDS = 3.0
    print(f"Stabilizing for {STABILIZE_SECONDS:.0f}s - start walking now...")
    t_stab = time.time()
    while time.time() - t_stab < STABILIZE_SECONDS:
        read_estimation_filter(node)

    log_session(node, ref_quat, out_path, max_seconds=args.seconds)

    if not args.no_plot:
        plot_csv(out_path)


if __name__ == "__main__":
    main()
```

---

## Appendix C: `microstrain_matlab_plot.m` (full source)

```matlab
% microstrain_matlab_plot.m
%
% Loads a walking_trial_*.csv (t, roll_deg, pitch_deg, yaw_deg,
% sensor_ang_vel_deg_s, derived_ang_vel_deg_s, phase_var), zeros the
% timestamp so it starts at 0s, and plots pitch, angular velocity
% (sensor vs derived), phase portrait, and phase variable over time.
%
% Usage:
%   analyze_walking_trial('walking_trial_20260710_145719.csv')
function analyze_walking_trial(csv_file)
if nargin < 1
    csv_file = 'walking_trial_20260713_150826.csv';
end
T = readtable(csv_file);
t = T.t - T.t(1);   % zero the timestamp - raw t is a Unix timestamp
pitch = T.pitch_deg;
sensor_rate = T.sensor_ang_vel_deg_s;
derived_rate = T.derived_ang_vel_deg_s;
phase = T.phase_var;
%% Pitch over time
figure('Name', 'pitch over time');
plot(t, pitch, 'b-');
xlabel('Time (s)');
ylabel('Pitch (deg)');
title('Thigh angle over session');
grid on;
%% Angular velocity: sensor vs derived
figure('Name', 'Angular velocity: sensor vs derived');
plot(t, sensor_rate, 'r-', 'DisplayName', 'Sensor (estAngularRateY)');
hold on;
plot(t, derived_rate, 'g-', 'DisplayName', 'Derived (d(pitch)/dt, filtered)');
hold off;
xlabel('Time (s)');
ylabel('Angular velocity (deg/s)');
title('Angular velocity: sensor channel vs derived');
legend('show');
grid on;
%% Phase portrait
figure('Name', 'Phase portrait');
plot(pitch, derived_rate, '.', 'MarkerSize', 4);
xlabel('Pitch (deg)');
ylabel('Derived angular velocity (deg/s)');
title('Phase portrait (pitch vs derived ang. vel.)');
axis equal;
grid on;
%% Phase variable over time
figure('Name', 'Phase variable over time');
plot(t, phase, 'b-', 'MarkerSize', 4);
xlabel('Time (s)');
ylabel('Phase variable (0-1)');
title('Phase variable - should look like a repeating sawtooth');
ylim([0 1]);
grid on;
end
```
