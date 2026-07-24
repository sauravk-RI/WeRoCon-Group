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
