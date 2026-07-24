# Thigh IMU — Side Mount Logging & Calibration

This README covers the **offline CSV logging pipeline** for a side-mounted
MicroStrain IMU on the thigh:

- `calibrate_imu.py` — stand-alone calibration, saves a reference quaternion to JSON
- `micro_strain_side_mount.py` — main logger, reads the sensor's Estimation Filter output, computes thigh angle relative to the calibration pose, and writes a CSV (with an optional plot at the end)
- `imu_side_mounted_calibration.json` — an example saved calibration file
- `analyze_walking_trial.m` — MATLAB script to re-plot a saved CSV

It is a companion to `README_live_dashboard.md`, which covers the real-time
WebSocket/HTML dashboard version of the same math.

---

## 1. Why relative quaternions, not Euler angles

The sensor's Estimation Filter outputs an absolute attitude quaternion
`(w, x, y, z)` referenced to the world frame (via internal sensor fusion).
Two problems if you used that directly:

1. It reports **world-frame** orientation, not orientation relative to how you
   stood at calibration — so a `+1.0` change in yaw depends on which way you
   happened to be facing when you turned the sensor on.
2. Converting to fixed-frame Euler angles (roll/pitch/yaw about world axes)
   suffers **gimbal lock** — near ±90° pitch, roll and yaw become degenerate
   and the numbers can jump or blow up.

Both scripts solve this the same way: capture a reference quaternion while
you stand still, then for every new sample compute the relative rotation

```
q_rel = conjugate(q_ref) * q_now
```

and convert `q_rel` to Euler angles. Because `q_rel` represents "how far have
we rotated *since calibration*", not "orientation in the world", this stays
well-behaved regardless of how the sensor is mounted (side vs front) or which
way you were facing when you calibrated.

### The core math

```python
def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )

def quat_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def quat_relative_euler_deg(q_ref, q_now):
    q_rel = quat_mul(quat_conj(q_ref), q_now)
    w, x, y, z = q_rel

    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sinp  = max(-1.0, min(1.0, 2*(w*y - z*x)))
    pitch = math.asin(sinp)
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
```

Standard quaternion Hamilton product + conjugate, then a standard
quaternion→Euler (roll-pitch-yaw, ZYX convention) conversion, applied to the
*relative* quaternion instead of the raw one. `pitch` still uses `asin` and is
therefore still technically capable of gimbal lock at ±90° — but because it's
now measured from a neutral standing pose instead of from the world frame,
you'd need the thigh itself to rotate 90° from vertical relative to
calibration to hit it, which is far outside normal gait range.

---

## 2. Calibration (`calibrate_imu.py`)

Stand-alone script — run it once per session/remount, save the result, reuse
it across multiple trials so you don't have to recalibrate every single run.

### What it does

1. Opens the serial connection to the sensor (`/dev/ttyACM0` @ 115200 baud)
   and resumes streaming.
2. Buffers incoming quaternion samples for `CALIBRATION_SECONDS = 3.0`s worth
   of samples (buffer length ≈ `duration_s / 0.005`, i.e. assumes ~200 Hz).
3. Checks whether you actually held still: for every quaternion in the
   buffer, it computes the angular deviation from the *last* sample in the
   buffer via

   ```python
   q_rel = quat_mul(quat_conj(q_last), q)
   w = clamp(q_rel[0], -1, 1)
   dev_deg = 2 * acos(|w|) * RAD_TO_DEG
   ```

   (this is the standard "angle of the relative rotation" formula — the `w`
   component of a rotation quaternion is `cos(θ/2)`, so `2*acos(w)` recovers
   `θ`). If the *worst* deviation across the whole window is under
   `CALIBRATION_STILL_STD_DEG * 3 = 1.5°`, the window counts as "still" and
   the last sample in it becomes the reference quaternion.
4. If you never hold still enough within `CALIBRATION_TIMEOUT_S = 15s`, it
   falls back to the last sample it saw (with a warning) rather than hanging
   forever. If it received literally nothing, it falls back to the identity
   quaternion `(1, 0, 0, 0)`.
5. Saves the result to JSON:

   ```json
   {
     "reference_quaternion": {"w": ..., "x": ..., "y": ..., "z": ...},
     "calibrated_at": "2026-07-13 16:16:49"
   }
   ```

### Usage

```bash
pip install python-mscl --break-system-packages

python3 calibrate_imu.py                       # writes imu_side_mounted_calibration.json
python3 calibrate_imu.py --out my_calibration.json
```

Stand neutral and hold still when prompted. Re-run any time you want a fresh
zero — new session, sensor remounted, strap re-tightened, etc.

> **Note on the printed usage hint:** the script's final print statement says
> `Use this with: python3 micro_strain_front_mount.py --calibration ...`, but
> the actual logger script in this pipeline is named
> `micro_strain_side_mount.py`. That's a leftover filename from an earlier
> front-mount variant — use the real filename when you run the logger.

### Example saved file

`imu_side_mounted_calibration.json` in this repo is a real example of the
output shape:

```json
{
  "reference_quaternion": {
    "w": 0.7402328848838806,
    "x": -0.1662135124206543,
    "y": -0.6349981427192688,
    "z": -0.14562182128429413
  },
  "calibrated_at": "2026-07-13 16:16:49"
}
```

---

## 3. Logging (`micro_strain_side_mount.py`)

This is the main data-collection script for a trial. It does **not** do live
re-calibration inside a run — each run either calibrates fresh at startup or
loads a saved calibration file, then logs until it hits the time limit or you
hit Ctrl+C.

### Configuration block

```python
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE   = 115200

PITCH_SIGN  = +1.0
THIGH_AXIS  = "z"          # <-- see note below
CALIBRATION_SECONDS       = 3.0
CALIBRATION_STILL_STD_DEG = 0.5
CALIBRATION_TIMEOUT_S     = 15.0
ALPHA_GYRO  = 0.04          # EMA smoothing factor for the derived rate

A_PITCH   = 13.0    # deg, phase-portrait normalization
B_GYRO    = 127.0   # deg/s, phase-portrait normalization
MIN_RADIUS = 0.15
```

> **⚠️ Axis mismatch to double check before trusting the output:**
> The comment above `THIGH_AXIS` says *"CONFIRMED 2026-07-07: swing axis is
> ~97% Y"*, but `THIGH_AXIS` is set to `"z"`, not `"y"`. That means
> `ANGULAR_RATE_CHANNEL` resolves to `estAngularRateZ`, and the code labels
> `yaw_raw` (rotation about the relative Z axis) as `pitch_deg` for the whole
> rest of the script — including feeding it into the phase-portrait math and
> the CSV's `pitch_deg` column. If the confirmed swing axis really is Y for
> this mount, this constant needs to be changed back to `"y"` (which is what
> the live-dashboard script uses — see the other README) before the numbers
> in `pitch_deg` can be trusted as actual sagittal-plane thigh angle. This is
> a one-line fix (`THIGH_AXIS = "y"`) but worth confirming against your
> current mount orientation, since side-mount vs front-mount changes which
> physical axis corresponds to flexion/extension.
>
> Also note `A_PITCH`/`B_GYRO`/`MIN_RADIUS` here (13.0 / 127.0 / 0.15) differ
> from the live-dashboard script's values (23.7 / 150.8 / 0.05) — these are
> normalization constants tuned per-pipeline, so keep them intentionally
> different only if you've actually re-tuned them for this axis/mount; otherwise
> it's worth checking they weren't just left over from a different sensor setup.

### Per-sample pipeline

For every packet read off the serial connection (`node.getDataPackets(500)`,
i.e. wait up to 500 ms for new data):

1. **Read quaternion + sensor's own angular rate channel**
   (`read_estimation_filter`) — pulls `estAttitudeQuaternion` (or the
   per-axis scalar channel fallbacks `estQuaternionW/X/Y/Z`) plus whichever
   `estAngularRate{X,Y,Z}` channel matches `THIGH_AXIS`.

2. **Convert to relative Euler angles** via `quat_relative_euler_deg(ref_quat, quat)`,
   then pick out `pitch_deg` as `PITCH_SIGN * angle_by_axis[THIGH_AXIS]`
   (sign flip lets you match the physical convention you want — positive
   angle = flexion, for example — without touching the math itself).

3. **Sensor-reported angular rate**: converts the fused-filter rate channel
   from rad/s to deg/s (`sensor_ang_vel_deg_s` in the CSV). This is the IMU's
   own internal estimate.

4. **Derived angular rate**: independently computed as `Δpitch / Δt` between
   consecutive samples, then smoothed with an exponential moving average:

   ```python
   gy_filt = ALPHA_GYRO * derived_rate + (1 - ALPHA_GYRO) * gy_filt
   ```

   `ALPHA_GYRO = 0.04` is a fairly heavy smoothing factor — it takes ~25
   samples for a step change to mostly settle. This filtered value is what
   feeds `derived_ang_vel_deg_s` and the phase-variable calculation, *not*
   the raw per-sample derivative — logging both means you can directly
   compare "trust the sensor's gyro channel" vs "differentiate our own
   quaternion-derived angle" after the fact rather than betting on one at
   collection time.

5. **Phase variable**: normalizes pitch and filtered rate into a unit-ish
   circle (`x = pitch/A_PITCH`, `y = rate/B_GYRO`), computes the polar angle
   with `atan2(y, x)`, and remaps to `[0, 1)`:

   ```python
   phase = (angle + pi) / (2*pi)
   return 1 - phase
   ```

   If the point falls inside `MIN_RADIUS` of the origin (i.e. near-zero pitch
   *and* near-zero rate — the "standing still" region), the phase is
   considered undefined for that sample and the **last valid phase value is
   held over** rather than reported as garbage or zero. This avoids a
   division-by-noise blowup right in the middle of stance/near-neutral
   moments.

### Fixed-cadence sampling loop

Rather than a flat `time.sleep(x)` after every iteration (which drifts as
processing time varies), the logger targets a fixed period (`loop_period_s =
0.01`, i.e. 100 Hz) using a running `next_tick` clock:

```python
next_tick += loop_period_s
sleep_time = next_tick - time.time()
if sleep_time > 0:
    time.sleep(sleep_time)
else:
    next_tick = time.time()   # fell behind — resync instead of bursting to catch up
```

This is deliberately being tested against the sensor's own reported rate
channel — the docstring notes this logger exists partly to check "whether the
sensor's own `estAngularRateY` channel becomes trustworthy under more uniform
timing than the previous logger used."

### CSV output

```
t,roll_deg,pitch_deg,yaw_deg,sensor_ang_vel_deg_s,derived_ang_vel_deg_s,phase_var
```

- `t` — Unix timestamp (not zeroed; `analyze_walking_trial.m` and `plot_csv()`
  both zero it to start at 0s when plotting)
- `roll_deg`, `pitch_deg`, `yaw_deg` — relative Euler angles, with whichever
  axis is `THIGH_AXIS` reported as `pitch_deg` (see axis note above)
- `sensor_ang_vel_deg_s` — IMU's own fused angular rate estimate, deg/s
- `derived_ang_vel_deg_s` — our own filtered `Δangle/Δt`, deg/s
- `phase_var` — normalized gait-cycle phase, 0–1 (empty string if undefined
  for that row, e.g. before the first `Δt` can be computed)

File is flushed every 200 rows and on exit, so a `Ctrl+C` mid-trial doesn't
lose your data.

### Auto-plotting

Unless `--no-plot` is passed, `plot_csv()` runs after logging and produces a
4-panel PNG (`<output>_plots.png`) next to the CSV: pitch vs. time, sensor
vs. derived angular velocity overlay, phase portrait (pitch vs. derived
rate), and phase variable vs. time. Uses the `Agg` backend so it works
headless over SSH — it only writes the PNG, it never tries to open a window.

### Usage

```bash
pip install python-mscl --break-system-packages

python3 micro_strain_side_mount.py                          # 10s trial, auto-named CSV, live calibration
python3 micro_strain_side_mount.py --out walking_trial_1.csv
python3 micro_strain_side_mount.py --seconds 15
python3 micro_strain_side_mount.py --calibration imu_side_mounted_calibration.json
python3 micro_strain_side_mount.py --no-plot
```

Flow on each run:

1. Connects to the sensor.
2. Calibrates live (3s, stand neutral) *or* loads a saved calibration JSON if
   `--calibration` is passed.
3. **3-second stabilize window** — reads and discards samples so you have
   time to start walking before logging actually begins.
4. Logs for `--seconds` (default 10) or until Ctrl+C, writing rows as it goes.
5. Plots the result unless `--no-plot`.

If the output file already exists, you'll be prompted to confirm overwrite.

### Re-plotting later

`analyze_walking_trial.m` (MATLAB) reproduces the same 4 plots from a saved
CSV, independent of the Python plotting path:

```matlab
analyze_walking_trial('walking_trial_20260713_150826.csv')
```

It zeros the timestamp the same way (`t = T.t - T.t(1)`) and plots pitch,
sensor-vs-derived rate, the phase portrait, and the phase variable in
separate figure windows.

---

## 4. Sensor Connect setup

Both `calibrate_imu.py` and `micro_strain_side_mount.py` require the
Estimation Filter's **"Attitude (Quaternion)"** channel to be enabled in
MicroStrain Sensor Connect before running either script — that's the source
of `estAttitudeQuaternion` (or the scalar `estQuaternionW/X/Y/Z` fallback)
that `read_estimation_filter()` looks for.

---

## 5. Dependencies

```bash
pip install python-mscl --break-system-packages
```

Also uses `matplotlib` (with the non-interactive `Agg` backend) for the
end-of-trial PNG plot, and the Python standard library (`argparse`, `csv`,
`json`, `math`, `os`, `time`).

---

## 6. Full source listings

### `calibrate_imu.py`

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

### `imu_side_mounted_calibration.json` (example output)

```json
{
  "reference_quaternion": {
    "w": 0.7402328848838806,
    "x": -0.1662135124206543,
    "y": -0.6349981427192688,
    "z": -0.14562182128429413
  },
  "calibrated_at": "2026-07-13 16:16:49"
}
```

### `micro_strain_side_mount.py`

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
    python3 micro_strain_side_mount.py
    python3 micro_strain_side_mount.py --out walking_trial_1.csv
    python3 micro_strain_side_mount.py --seconds 15

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

PITCH_SIGN = +1.0

# Which axis is thigh flexion/extension. CONFIRMED 2026-07-07: swing axis is ~97% Y.
THIGH_AXIS = "z"

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
A_PITCH = 13.0     # deg
B_GYRO = 127.0     # deg/s

MIN_RADIUS = 0.15

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

    axes[1, 1].plot(t, phase, "_", markersize=4, color="tab:cyan")
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

### `analyze_walking_trial.m`

```matlab
% analyze_walking_trial.m
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
