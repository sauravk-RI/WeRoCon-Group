# Thigh IMU Logger — MPU9250 (Kalman Filter + Gait Phase Variable)

Wiring → Raspberry Pi I2C setup → counterfeit check → Python library
installs → calibration → Kalman tuning → running the logger → plotting
the CSV in MATLAB.

Written so someone starting from zero can follow it top to bottom and
end up with a working thigh-angle + gait-phase logger and MATLAB plots.

This README has two parts:

- **[Part 1 — Hardware & Raspberry Pi Setup](#part-1--hardware--raspberry-pi-setup)**:
  wiring, I2C, checking your board isn't a counterfeit, and installing
  Python dependencies. Do this once per sensor/Pi.
- **[Part 2 — Logger Software](#part-2--logger-software)**: calibration,
  Kalman filter tuning, understanding how `phase_variable_mpu_9250.py` actually
  works, running it, plotting in MATLAB, and troubleshooting. Do this
  per calibration cycle / per trial.

---

# Part 1 — Hardware & Raspberry Pi Setup

## 1. What you need

- MPU9250 breakout board (MPU6500 accel/gyro + AK8963 magnetometer)
- Raspberry Pi (any model with I2C; this guide assumes Pi 5)
- 4 jumper wires (breadboard optional but recommended)
- MATLAB (any recent version — no toolboxes required for plotting)

**Files in this setup:**

| File | Purpose |
|---|---|
| `verify_mpu9250.py` | Checks whether your board is a genuine MPU9250/GY-91 or a fake/relabeled clone (Section 4) |
| `phase_variable_mpu_9250.py` | Main Kalman-filter logger + gait phase variable, writes `phase_variable_mpu_9250.csv` |
| `matlab_phase-variable_mpu_9250.m` | MATLAB script to verify/plot the phase variable output |

**Everything in this guide happens inside one project folder.** Create
it first and keep all three files above inside it — every command
below assumes you're `cd`'d into this folder:

```bash
mkdir imu_project
cd imu_project
```

Copy `verify_mpu9250.py` and `phase_variable_mpu_9250.py` into this folder
(and later copy the output CSV out of it into wherever your MATLAB
project lives, or just also drop `matlab_phase-variable_mpu_9250.m` in here and run
MATLAB pointed at this folder). The virtual environment (`sensor_env`)
also gets created inside this same folder — see Part 1, Section 5.

---

## 2. Wiring

The MPU9250 talks over I2C — only 4 wires needed:

| MPU9250 pin | Raspberry Pi pin      |
|-------------|------------------------|
| VCC         | 3.3V (Pin 1)           |
| GND         | GND (Pin 6 or any GND) |
| SCL         | GPIO3 / SCL (Pin 5)    |
| SDA         | GPIO2 / SDA (Pin 3)    |

Reference for the full 40-pin header (this guide only uses pins 1, 3,
5, and 6, but keep this handy if you add other sensors later): see the
Raspberry Pi 5 GPIO pinout diagram in your board's documentation.
<img width="1024" height="593" alt="image" src="https://github.com/user-attachments/assets/f20f7164-6334-4657-a98f-b3ceb97c2308" />


- Leave **AD0 tied to GND** — this keeps the accel/gyro at I2C address
  `0x68`. If AD0 is pulled high instead, the address becomes `0x69`
  (this script assumes `0x68`).
- The MPU9250 module wires both the MPU6050 (accel/gyro) and the
  AK8963 (magnetometer) to the same I2C port internally via an
  auxiliary I2C bus — you don't wire the magnetometer separately, it
  rides on the same SDA/SCL lines once bypass mode is enabled (the
  script does this for you).

---

## 3. Enable I2C on the Raspberry Pi

```bash
sudo raspi-config
```
Navigate to **Interface Options → I2C → Enable**, then reboot:
```bash
sudo reboot
```

Install the I2C tools, then `cd` into your project folder and scan
the bus:
```bash
sudo apt update
sudo apt install -y i2c-tools
cd imu_project
i2cdetect -y 1
```

### What each address in the scan means

`i2cdetect` just shows *which addresses respond* — it doesn't tell
you what's plugged in. Use this table to read the result:

| Address shown | What it means |
|---|---|
| `68` | MPU9250/MPU9255/MPU6500 accel+gyro die, with **AD0 tied to GND** (the default wiring in Section 2) |
| `69` | Same accel+gyro die, but with **AD0 tied to 3.3V** instead — if you see this instead of `68`, update `MPU_ADDR` in every script to `0x69` |
| `0C` | AK8963 magnetometer — **only appears after bypass mode is enabled**, which only happens once you run `phase_variable_mpu_9250.py` or `verify_mpu9250.py`. A bare `i2cdetect` right after wiring, before running any script, will normally show `68` alone with `0C` absent — that's expected, not a fault |
| `76` or `77` | BMP280 barometer — present on genuine GY-91 boards. Its presence does **not** confirm the IMU part is genuine (see Section 4) |
| Nothing at all | Check wiring (Section 2), check the Pi is powered, and check you actually rebooted after enabling I2C |
| `UU` instead of an address | A kernel driver has already claimed that address — usually harmless for our purposes, but means you can't talk to it directly over raw I2C without unloading that driver first |

A typical **first scan**, right after wiring and before running any
script, looks like this:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- 76 --
```
That's a genuine board with only `68` (accel/gyro) and `76` (BMP280)
visible — the `0C` (magnetometer) hasn't shown up yet because bypass
mode isn't enabled until the Python script runs. This is normal, not
a sign of a fake board — don't judge authenticity from `i2cdetect`
alone; use `verify_mpu9250.py` (Section 4) instead, since it actively
enables bypass mode before checking for the magnetometer.

**If `i2cdetect` shows nothing at all:** check wiring and that the Pi
is powered; recheck 3.3V vs 5V (the MPU9250 is a 3.3V part — some
breakout boards have an onboard regulator that tolerates 5V, check
your board's silkscreen/datasheet before assuming).

---

## 4. Check whether your MPU9250/GY-91 is genuine (do this first!)

A lot of cheap "GY-91" or "MPU9250" boards sold online are counterfeit
in a specific, sneaky way: the seller relabels a bare **MPU6500**
(accelerometer + gyroscope only — no magnetometer die at all) and
sells it as if it were a 9-axis MPU9250. Because the MPU6500 and
MPU9250 are register-compatible for accel/gyro, **everything appears
to work normally** — you'll get real roll/pitch data — right up until
you try to read the magnetometer, which either returns nothing,
returns zeros, or (on real GY-91 boards with a genuine BMP280 but no
real AK8963) silently fails while the barometer still works fine.

This matters because the logger script in this guide falls back
gracefully to gyro-only yaw when no magnetometer is found — which
means a fake board can run for weeks without you ever noticing your
yaw has been drifting uncorrected the whole time.

**Run the verification script before doing anything else:**

```bash
cd imu_project   # if not already there
source sensor_env/bin/activate
pip install smbus2 --break-system-packages   # if not already installed
python3 verify_mpu9250.py
```

It checks two things via I2C `WHO_AM_I` registers:

1. **The main chip ID** — genuine MPU9250 reports `0x71`, MPU9255
   reports `0x73`. If instead you get `0x70` (MPU6500) or `0x68`
   (MPU6050), your board physically cannot do 9-axis sensing no
   matter what firmware or calibration you apply — it's a 6-axis part
   labeled as something it isn't.
2. **Whether a genuine AK8963 magnetometer responds** at `0x0C` with
   the correct WHO_AM_I value `0x48`, after enabling bypass mode.

Example output for a genuine chip:
```
Accel/Gyro die WHO_AM_I : 0x71
  → MPU9250  (genuine — has AK8963 magnetometer on the die)

Magnetometer probe at 0x0C:
  WHO_AM_I = 0x48  (expected 0x48 for genuine AK8963)
  → AK8963 magnetometer responded correctly.

✅ GENUINE 9-axis MPU9250/MPU9255 — accel, gyro, AND magnetometer
   all confirmed present and responding correctly.
```

Example output for a relabeled fake:
```
Accel/Gyro die WHO_AM_I : 0x70
  → MPU6500  (accel+gyro ONLY — this is what most fakes actually are)

Magnetometer probe at 0x0C:
  → NO response. No magnetometer chip is present/reachable at all.

❌ THIS IS LIKELY A FAKE / MISLABELED BOARD.
```

**Note:** a genuine GY-91 board also carries a separate BMP280
barometer chip (pressure/temperature) at I2C address `0x76` or `0x77`.
That chip working correctly does **not** prove the IMU part is
genuine — counterfeiters typically keep the real BMP280 and only
fake or omit the motion-sensing die, since the barometer is cheap and
buyers rarely check it.

If your board comes back as fake, you have two options: keep using it
as a 6-axis sensor (accel+gyro only, accepting gyro-only yaw with
drift) by explicitly relying on the script's existing fallback path,
or return/replace it if you specifically need real magnetometer-based
yaw correction.

---

## 5. Install Python libraries

Stay inside `imu_project` for all of this:

```bash
cd imu_project      # skip if you're already here

# Create a virtual environment (recommended)
python3 -m venv sensor_env
source sensor_env/bin/activate

# Install dependencies — smbus2 for I2C, numpy for the Kalman filter
pip install smbus2 numpy --break-system-packages
```

No separate MPU9250-specific driver library is needed — the script
in this setup talks to the chip directly over I2C using raw register
addresses (see the register map at the top of the script), which
avoids depending on third-party driver packages that vary in quality
and maintenance.

---

## 6. A known AK8963 gotcha (read this before you run anything)

On some Pi/kernel combinations, even with bypass mode correctly
enabled (`INT_PIN_CFG = 0x02`), the AK8963 magnetometer refuses to
respond and raises:
```
OSError: [Errno 121] Remote I/O error
```
This can happen even when the MPU9250 registers correctly indicate
bypass mode is enabled — `i2cdetect` still won't show a device at
`0x0C` in that case.

**This script already handles it gracefully** — if the AK8963 isn't
found, it prints a warning and falls back to gyro-only yaw
(integrated, no magnetometer correction) instead of crashing. You'll
see:
```
[AK8963]  WARNING: magnetometer not found at 0x0C — yaw = gyro only.
```
If you need working yaw with magnetometer correction, first try
power-cycling the sensor (remove and reapply VCC) — this resolves it
in most cases. If it persists, it may indicate a kernel driver
claiming the I2C bus underneath you, or a faulty module.

---

# Part 2 — Logger Software

*Do this per calibration cycle (Section 1) and per trial (running the
script, Section 4). Assumes Part 1 is already done: sensor is wired,
I2C is enabled, the board is confirmed genuine (or you've accepted the
gyro-only fallback), and Python dependencies are installed.*

## 1. Apply calibration offsets (do this before trusting any output)

The script ships with **placeholder calibration values already filled
in** from a prior calibration run — but they're for whoever ran that
calibration, on their sensor, in their mounting. Treat them as
starting points to overwrite, not values to trust blindly for a new
sensor or mount:

```python
gyro_off  = {'gx': -0.8146, 'gy': 1.9096, 'gz': -0.6619}
accel_off = {'x':  0.0, 'y':  0.0, 'z':  0.0}
accel_scl = {'x':  1.0, 'y':  1.0, 'z':  1.0}
mag_hi    = {'x':  0.0, 'y':  0.0, 'z':  0.0}
mag_si    = {'x':  1.0, 'y':  1.0, 'z':  1.0}
```

Note that in the current script, `accel_off`/`accel_scl`/`mag_hi`/`mag_si`
are still at their **no-correction defaults** (0.0 offset, 1.0 scale) —
only the gyro offsets have been calibrated so far. To fill in the rest:

1. **Gyro bias** (`gyro_off`) — with the sensor completely still on a
   flat, level surface, log a few seconds of gyro data and average it.
   That average is `gx`/`gy`/`gz`. *(Already done in the current
   script — the values above came from this step.)*
2. **Accelerometer offset/scale** (`accel_off`, `accel_scl`) — the
   standard 6-position calibration (each axis up and down, flat)
   gives you offset and scale per axis. *(Still at defaults — not
   yet calibrated in the current script.)*
3. **Magnetometer hard/soft iron** (`mag_hi`, `mag_si`) — rotate the
   sensor through a figure-eight motion while logging raw mag
   readings, then fit an ellipsoid to get hard-iron offset and
   soft-iron scale. A typical calibration routine has you wave the
   device in a figure eight until enough samples are collected to
   compute these biases and scale factors. *(Still at defaults.)*

Edit these dictionaries directly in `main()` before logging real data
you intend to trust. Skipping accel/mag calibration means roll/pitch
will carry a small constant offset and yaw may drift or be distorted
by nearby metal/magnets — the gyro-bias correction alone (already
applied) fixes the biggest source of drift, but not the rest.

---

## 2. Understand and tune the two Kalman filters

This script runs **two separate Kalman filters**, not one — that's
easy to miss on a skim, so it's worth being explicit:

### 2.1 Angle Kalman filter (`KalmanFilter` class)

One instance each for roll, pitch, and yaw. Standard 2-state
(angle, gyro-bias) filter: predicts the angle forward using the
gyro rate, corrects it against the accelerometer- (or magnetometer-,
for yaw) derived angle, and tracks gyro bias as a side effect.

```python
R_STILL        = 0.4371
R_MOVING       = 21.8550
R_HOLD_CYCLES  = 21
BLEND_G        = 0.060
Q_ANGLE        = 0.00003840
Q_BIAS         = 0.0000003840
```

- `R_STILL` / `R_MOVING` — measurement noise (trust in the
  accelerometer) when the sensor is still vs. actively moving. Higher
  R = trust the accelerometer less, lean on the gyro prediction more.
- `BLEND_G` — how far `|accel| ` has to deviate from 1g before the
  filter decides you're "moving" rather than "still" and switches
  toward `R_MOVING`.
- `R_HOLD_CYCLES` — once motion is detected (e.g. a heel strike),
  stay in the high-R "moving" state for this many samples afterward,
  even if the accelerometer briefly looks still again. This is a
  deliberate hold-off (see `compute_R_dynamic()`) so a brief lull
  mid-stride doesn't cause the filter to suddenly over-trust a noisy
  accelerometer reading.
- `Q_ANGLE` / `Q_BIAS` — process noise for the angle state and the
  bias state respectively. Smaller values mean the filter believes
  its own gyro-integrated prediction more and moves more slowly to
  correct against new measurements.

These specific values are **already tuned** (they're described in the
script's header as "pasted from `noise_profiler` output"), not the
generic defaults from an earlier revision. If you have a
`noise_profiler.py` / `tuning_constant.py` script for your own sensor
and mounting, run that first and paste its output values in here
instead — every mounting location and motion pattern (walking gait
vs. general robotics vs. vehicle IMU) has different noise
characteristics, and these particular numbers were tuned for a
specific thigh-mounted walking-gait setup.

### 2.2 Gyro-rate Kalman filter (`GyroKalman` class) — separate from the above

This is a **second, independent filter** that smooths the
*derivative* of pitch (i.e., angular velocity), not the angle itself.
It replaces a simpler exponential-moving-average approach used in
earlier revisions:

```python
GK_Q_OMEGA     = 0.25
GK_Q_OMEGADOT  = 1.0
GK_R           = 90.0
```

It tracks two states — angular velocity (`omega`) and angular
acceleration (`omegadot`) — and feeds off the numerically
differentiated pitch signal (`gyro_pitch_dps`, computed as
`(pitch - prev_pitch) / dt`), not off the raw gyro register directly.
Its smoothed output (`gyro_pitch_filt`) is what actually feeds the
gait phase variable below — so if the phase variable looks off, this
filter's tuning is one of the first places to check, separately from
the angle-filter tuning above.

---

## 3. How the logger actually works

### 3.1 Per-loop pipeline

Each iteration of the main loop (target rate: `LOOP_HZ = 100`):

1. Read raw accel + gyro over I2C; apply gyro bias and accel
   offset/scale corrections.
2. Compute `accel_norm` (`|accel|` in g) and feed it into
   `compute_R_dynamic()` to decide whether roll/pitch's angle Kalman
   filters should currently be in "still" or "moving" mode (with the
   hold-off described in Section 2.1).
3. Compute accelerometer-derived roll/pitch (`accel_angles()`), then
   update the two angle Kalman filters with the gyro rate + this
   accelerometer angle + the dynamic R.
4. If a magnetometer reading is available this cycle, compute
   tilt-compensated yaw (`tilt_yaw()`) and update the yaw Kalman
   filter with it. If not (either no magnetometer at all, or a
   transient read failure this cycle), yaw is instead integrated
   open-loop from the gyro (`last_yaw += gz * dt`), which will drift
   without magnetometer correction — this is the same fallback
   behavior described in Part 1, Section 6.
5. Center pitch around a fixed offset (`pitch_centered = pitch -
   DC_OFFSET_PITCH`, currently `-3.587°`, itself derived from a prior
   `noise_profiler` walking trial — recalibrate this if your mounting
   or subject changes) and numerically differentiate pitch to get
   `gyro_pitch_dps`, then smooth it through the gyro-rate Kalman
   filter (Section 2.2) to get `gyro_pitch_filt`.
6. Compute the **gait phase variable** from `pitch_centered` and
   `gyro_pitch_filt` (Section 3.2).
7. Write one CSV row; print a progress line every 10 rows; flush the
   file every 100 rows.
8. Sleep for whatever time remains in the loop period to hold
   `LOOP_HZ`.

### 3.2 The gait phase variable, and why it needs a safeguard

The phase variable is meant to sweep smoothly from 0 → 1 once per gait
cycle (a "sawtooth" when plotted against time). It's built as a
normalized phase portrait:

```python
phi_raw   = atan2(gyro_pitch_filt / PHASE_B, pitch_centered / PHASE_A)  # -π to π
phase_var = 1 - ((phi_raw + π) / (2π))                                  #  0 to 1
```

`PHASE_A` (30°) and `PHASE_B` (130°/s) are **fixed scaling constants**
carried over from a prior gait cycle's amplitude — not live-normalized
per trial. If a new trial's pitch or angular-velocity range is
meaningfully different (different subject, different walking speed,
different mounting), the phase portrait won't fill the expected
circle and `phase_var` may not actually reach 0 or 1 — see the
Troubleshooting table.

Because `atan2`-based phase can jitter backward slightly on noisy
samples even during genuine forward progress, the script adds an
explicit **monotonicity safeguard** that isn't mentioned anywhere in
the plain-English sections of a typical guide, so it's worth calling
out on its own:

- If `phase_var` would go *down* from the previous sample, and it's
  not a legitimate wraparound (i.e. previous value was already near 1
  and new value is near 0 — a real new gait cycle starting), the
  script **clamps** it to `prev_phase_var + MIN_INCREMENT` instead of
  letting it dip backward.
- `clamp_count` tracks how many consecutive samples got clamped this
  way. If that streak exceeds `MAX_CONSEC_CLAMP` (50 samples), the
  script sets `sensor_fault_flag = True` and prints `fault=True` in
  the live progress line — a signal that phase isn't genuinely
  progressing and something upstream (noisy signal, wrong `PHASE_A`/
  `PHASE_B`, or a real sensor problem) needs attention, not that the
  clamp itself is fixing anything.

### 3.3 Output file and columns

**The script writes `phase_variable_mpu_9250.csv`** in the current
directory (hardcoded as `csv_file` in `main()`) — not `imu_data.csv`.
If you're following along with a general prose description that
mentions `imu_data.csv`, that refers to an earlier/simpler revision;
this script's actual output filename is `phase_variable_mpu_9250.csv`.
Rename it after the run, or edit the `csv_file` variable, if you need
a specific filename for a downstream tool.

Columns actually written, in order:

```
time_s, roll_deg, pitch_deg, yaw_deg, temp_c,
bias_roll, bias_pitch, accel_norm_g,
K_roll_angle, K_roll_bias, K_pitch_angle, K_pitch_bias,
P_roll_00, P_roll_11, P_pitch_00, P_pitch_11,
gyro_pitch_dps, pitch_centered_deg, R_dynamic, gyro_pitch_filt,
phase_var
```

The first 16 columns are unchanged from an earlier revision (raw
angles, Kalman gains, and covariances — everything needed to inspect
angle-filter behavior). The last 5 (`gyro_pitch_dps` onward) are new
in this revision and specifically support the gait phase variable:
raw differentiated pitch rate, centered pitch, the dynamic R value
actually used that sample, the Kalman-filtered pitch rate, and the
final phase variable.

---

## 4. Run the logger

```bash
cd imu_project   # if not already there
source sensor_env/bin/activate
python3 phase_variable_mpu_9250.py
```

Expected output:
```
Initialising MPU9250…
[MPU9250] WHO_AM_I = 0x71  (0x71=MPU9250  0x47=ICM-20689)
[AK8963]  WHO_AM_I = 0x48  (expected 0x48)
[AK8963]  Sensitivity adj = (1.21, 1.22, 1.17)

Logging to : phase_variable_mpu_9250.csv
Duration   : 10.0 s   (Ctrl+C to stop early)
Gyro range : ±500 °/s   DLPF: 20 Hz   R_HOLD: 21 cycles
Phase scaling : A=30.0 deg   B=130.0 deg/s

Stabilizing for 5s — keep IMU still…
Stabilization done — starting logging.

  100% | roll=  +2.14° | pitch=  -1.03° | R=0.437 | hold=0 | φ=0.812 | fault=False

Saved 1000 rows → phase_variable_mpu_9250.csv
```

- **There's a mandatory 5-second stabilization window** before logging
  starts (`STABILIZE_DURATION = 5.0`) — keep the sensor still through
  this; it reads the sensor but discards the data, just to let
  readings settle before the run officially begins.
- `RUN_DURATION` (default 10.0s) controls how long it logs after
  stabilization — increase this for real trials.
- Press `Ctrl+C` to stop early — the CSV is flushed and closed safely
  either way (`f_csv.flush()` / `f_csv.close()` run after the loop
  regardless of how it exits).
- Watch the `fault=` field in the live progress line — `fault=True`
  means the phase-variable monotonicity safeguard has been clamping
  for more than 50 consecutive samples (Section 3.2), which is worth
  investigating even though the run will keep going.

---

## 5. Plot the CSV in MATLAB

`matlab_phase-variable_mpu_9250.m` is built specifically to verify the phase variable
(not a general-purpose orientation plotter) — it checks that
`phase_var` genuinely sweeps 0 → 1 and looks like a sawtooth, and
overlays the two raw signals that feed it.

1. Copy your output CSV into the same folder as `matlab_phase-variable_mpu_9250.m`,
   **or edit the filename inside the script** — as written, it loads
   a specific hardcoded file:
   ```matlab
   data = readtable('imu_phase_15june_7th.csv');
   ```
   which will not match `phase_variable_mpu_9250.csv` (the name the
   Python script actually writes, per Section 3.3) unless you rename
   one to match the other, or edit this line.
2. Open MATLAB, navigate to that folder, and run:
   ```matlab
   plot_phase_var
   ```

This produces 2 figures:

| Figure | Shows |
|---|---|
| **Phase Variable vs Time** (3 stacked subplots) | `phase_var` over time with 0/1 reference lines; `pitch_centered_deg` with ±`PHASE_A` reference lines; `gyro_pitch_filt` with ±`PHASE_B` reference lines |
| **Phase Portrait coloured by φ** | `pitch_centered_deg` vs. `gyro_pitch_filt`, scatter-colored by `phase_var` (HSV colormap) — a good visual gut-check that the portrait actually traces a full circle |

A summary is also printed to the MATLAB command window:

```
── Phase variable stats ──
  Min  : ...
  Max  : ...
  Range: ...
  Mean : ...

  If range ≈ 1.0 and plot shows sawtooth → phase is working correctly.
  If range is small → A or B needs adjustment (signal not reaching limits).
```

That last line is the practical diagnostic: if `phase_var`'s range
comes back well under 1.0, it's telling you `PHASE_A`/`PHASE_B` in
the Python script (Section 3.2) don't match this trial's actual
amplitude — re-tune them rather than trusting the phase values as-is.

No MATLAB toolboxes are required — it only uses `readtable` and base
plotting/scatter functions.

---

## 6. Troubleshooting

| Symptom | Likely cause |
|---|---|
| `[FATAL] Could not init MPU9250` | Wrong wiring, I2C not enabled, or address conflict — recheck Part 1, Section 3 |
| `WHO_AM_I` prints something other than `0x71` | Wrong sensor variant, bad wiring, or address mismatch — check AD0 pin |
| AK8963 `Remote I/O error` | See Part 1, Section 6 — script falls back gracefully, but try power-cycling the sensor |
| Roll/pitch have a constant offset | Accel/mag calibration not applied yet — only gyro bias is calibrated by default (Section 1) |
| Yaw drifts steadily even when still | No magnetometer correction active (gyro-only fallback), or mag hard/soft-iron calibration not applied |
| Values look noisy/jittery | Check DLPF settings weren't changed, and confirm the sensor is mounted rigidly (loose mounting = vibration noise) |
| `fault=True` appears in the live output | Phase variable has been clamped for 50+ consecutive samples (Section 3.2) — check `PHASE_A`/`PHASE_B` against this trial's actual pitch/rate range, or re-check sensor mounting |
| `phase_var` range comes back well under 1.0 in MATLAB | `PHASE_A`/`PHASE_B` don't match this trial's amplitude — re-tune in the Python script and re-run, per the MATLAB script's own printed guidance |
| MATLAB `readtable` errors, or loads the wrong/old file | `matlab_phase-variable_mpu_9250.m` has a hardcoded filename (`imu_phase_15june_7th.csv`) that won't automatically match the Python script's actual output (`phase_variable_mpu_9250.csv`) — rename one to match the other, or edit the script (Section 5) |
| CSV missing or looks truncated | Check the run wasn't killed harder than Ctrl+C (e.g. `kill -9`, power loss) — the `finally`-style flush/close only runs on normal exit or a caught `SIGINT` |
| Magnetometer never works, no matter what you try | Run `verify_mpu9250.py` (Part 1, Section 4) — you may have a fake/relabeled MPU6500 board with no magnetometer die at all |

---

## 7. Reference links

- MPU-9250 register map / datasheet: search "InvenSense MPU-9250 Register Map"
- smbus2 docs: https://pypi.org/project/smbus2/
- Maker Portal MPU9250 + Raspberry Pi tutorial (wiring diagrams, background): https://makersportal.com/blog/2019/11/11/raspberry-pi-python-accelerometer-gyroscope-magnetometer
- Kalman filter for IMU tilt estimation (background reading): search "complementary vs Kalman filter IMU tilt"

---

## Appendix A: `verify_mpu9250.py` (full source)

```python
"""
verify_mpu9250.py  —  Genuine vs Fake MPU9250/GY-91 checker
============================================================
Many cheap "GY-91" / "MPU9250" boards sold online are actually a bare
MPU6500 (accel + gyro ONLY — no magnetometer die at all) relabeled and
sold as a 9-axis sensor. This script checks the WHO_AM_I registers of
both chips on the bus and tells you plainly what you actually have.

Usage:
    pip install smbus2 --break-system-packages
    python3 verify_mpu9250.py
"""

import smbus2
import time
import sys

MPU_ADDR = 0x68     # or 0x69 if AD0 is pulled high
AK_ADDR  = 0x0C

REG_WHO_AM_I     = 0x75
REG_PWR_MGMT_1   = 0x6B
REG_USER_CTRL    = 0x6A
REG_INT_PIN_CFG  = 0x37

AK_WIA = 0x00

# Known WHO_AM_I values for the accel/gyro die
KNOWN_IDS = {
    0x71: "MPU9250  (genuine — has AK8963 magnetometer on the die)",
    0x73: "MPU9255  (genuine — near-identical to MPU9250, has AK8963)",
    0x70: "MPU6500  (accel+gyro ONLY — this is what most fakes actually are)",
    0x68: "MPU6050  (older accel+gyro only chip, no magnetometer)",
    0x47: "ICM-20689 (accel+gyro only, sometimes substituted on clones)",
}


def main():
    print("=" * 60)
    print("  MPU9250 / GY-91 Authenticity Check")
    print("=" * 60)

    bus = smbus2.SMBus(1)

    # ── Step 1: wake the chip and read its WHO_AM_I ────────────────
    try:
        bus.write_byte_data(MPU_ADDR, REG_PWR_MGMT_1, 0x00)
        time.sleep(0.05)
        bus.write_byte_data(MPU_ADDR, REG_PWR_MGMT_1, 0x01)
        time.sleep(0.10)
        who = bus.read_byte_data(MPU_ADDR, REG_WHO_AM_I)
    except OSError as e:
        print(f"\n[FAIL] Could not talk to any chip at address 0x{MPU_ADDR:02X}.")
        print(f"       {e}")
        print("       Check wiring, power, and that AD0 matches the address above.")
        sys.exit(1)

    print(f"\nAccel/Gyro die WHO_AM_I : 0x{who:02X}")
    chip_desc = KNOWN_IDS.get(who, f"UNKNOWN chip ID (0x{who:02X}) — not a recognized InvenSense part")
    print(f"  → {chip_desc}")

    is_9axis_capable_die = who in (0x71, 0x73)

    # ── Step 2: enable bypass mode and probe for AK8963 ────────────
    bus.write_byte_data(MPU_ADDR, REG_USER_CTRL, 0x00)
    time.sleep(0.02)
    bus.write_byte_data(MPU_ADDR, REG_INT_PIN_CFG, 0x02)   # BYPASS_EN
    time.sleep(0.10)

    mag_found = False
    mag_wia = None
    try:
        mag_wia = bus.read_byte_data(AK_ADDR, AK_WIA)
        mag_found = True
    except OSError:
        mag_found = False

    print(f"\nMagnetometer probe at 0x{AK_ADDR:02X}:")
    if mag_found:
        print(f"  WHO_AM_I = 0x{mag_wia:02X}  (expected 0x48 for genuine AK8963)")
        mag_genuine = (mag_wia == 0x48)
        if mag_genuine:
            print("  → AK8963 magnetometer responded correctly.")
        else:
            print("  → A device responded, but the ID doesn't match a genuine AK8963.")
    else:
        print("  → NO response. No magnetometer chip is present/reachable at all.")
        mag_genuine = False

    # ── Step 3: verdict ─────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("  VERDICT")
    print("=" * 60)

    if is_9axis_capable_die and mag_genuine:
        print("✅ GENUINE 9-axis MPU9250/MPU9255 — accel, gyro, AND magnetometer")
        print("   all confirmed present and responding correctly.")
    elif is_9axis_capable_die and not mag_genuine:
        print("⚠️  Accel/gyro die reports as MPU9250/MPU9255 (correct ID), but the")
        print("   magnetometer did NOT respond correctly. Possible causes:")
        print("     - Bypass mode wiring issue (rare, since bypass is set above)")
        print("     - Kernel driver holding the bus (see README troubleshooting)")
        print("     - This specific unit's AK8963 die is faulty/absent despite the")
        print("       main chip ID being correct (less common, but seen on some")
        print("       clone batches that relabel the main chip but omit the mag die)")
    else:
        print("❌ THIS IS LIKELY A FAKE / MISLABELED BOARD.")
        print(f"   The main chip identifies as: {chip_desc}")
        print("   This is NOT a 9-axis part. It has no magnetometer capability")
        print("   at all, regardless of what the AK8963 probe above found.")
        print()
        print("   This is the single most common counterfeit pattern for")
        print("   'GY-91' / 'MPU9250' boards sold cheaply online: an MPU6500")
        print("   (6-axis only) is relabeled and sold as if it were a 9-axis")
        print("   MPU9250. The two chips are pin- and register-compatible for")
        print("   accel/gyro, so everything except magnetometer reads will")
        print("   appear to work fine — which is exactly what makes the fake")
        print("   easy to miss until you specifically check WHO_AM_I.")

    print("\nNote: a genuine GY-91 board also includes a separate BMP280")
    print("barometer chip (pressure/temperature) at I2C address 0x76 or 0x77.")
    print("That chip's presence does NOT confirm the MPU part is genuine —")
    print("counterfeiters keep the real BMP280 and only fake/omit the IMU die.")


if __name__ == "__main__":
    main()

```

## Appendix B: `phase_variable_mpu_9250.py` (full source)

```python
import math, struct, time, signal, sys
import smbus2
import numpy as np

# ─── Run settings ─────────────────────────────────────────────────────
LOOP_HZ      = 100
RUN_DURATION = 10.0     # seconds — change as needed

# ─── MPU9250 Register Map ─────────────────────────────────────────────
MPU_ADDR         = 0x68
AK_ADDR          = 0x0C

REG_SMPLRT_DIV   = 0x19
REG_CONFIG       = 0x1A
REG_GYRO_CONFIG  = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_CFG2   = 0x1D
REG_INT_PIN_CFG  = 0x37
REG_USER_CTRL    = 0x6A
REG_PWR_MGMT_1   = 0x6B
REG_ACCEL_XOUT   = 0x3B
REG_GYRO_XOUT    = 0x43
REG_TEMP_OUT     = 0x41
REG_WHO_AM_I     = 0x75

AK_WIA    = 0x00
AK_ST1    = 0x02
AK_XOUT_L = 0x03
AK_ST2    = 0x09
AK_CNTL1  = 0x0A
AK_ASAX   = 0x10

ACCEL_SCALE = 16384.0   # ±2 g  (unchanged)
GYRO_SCALE  = 65.5      # ±500 °/s  (was 131.0 for ±250 °/s)

# ─── TUNING CONSTANTS ─────────────────────────────────────────────────
R_STILL        = 0.4371
R_MOVING       = 21.8550
R_HOLD_CYCLES  = 21
BLEND_G        = 0.060
Q_ANGLE        = 0.00003840
Q_BIAS         = 0.0000003840

# ─── Phase variable pre-processing ────────────────────────────────────
DC_OFFSET_PITCH = -3.587  # degrees — from noise_profiler walking trial

# ─── Phase variable normalization — fixed scaling from prior gait cycle ──
PHASE_A = 30.0    # max amplitude of pitch_centered_deg  (degrees)
PHASE_B = 130.0   # max amplitude of gyro_pitch_filt     (deg/s)

# ─── Gyro Kalman tuning ───────────────────────────────────────────────
GK_Q_OMEGA     = 0.25
GK_Q_OMEGADOT  = 1.0
GK_R           = 90.0

# ─── Phase variable monotonicity safeguard ────────────────────────────
MIN_INCREMENT     = 0.0005   # forced minimum forward step per loop
WRAP_THRESHOLD     = 0.85    # prev > this and new < (1-this) => real cycle wrap
MAX_CONSEC_CLAMP   = 50      # consecutive clamps before flagging sensor fault


# ─── Kalman Filter (with dynamic R and hold-off) ──────────────────────
class KalmanFilter:
    def __init__(self, Q_angle=Q_ANGLE, Q_bias=Q_BIAS, R_still=R_STILL):
        self.Q_angle = Q_angle
        self.Q_bias  = Q_bias
        self.R_still = R_still
        self.x = np.zeros((2, 1))
        self.P = np.eye(2)
        self.K = np.zeros((2, 1))
        self.H = np.array([[1.0, 0.0]])
        self._t = None

    def update(self, gyro_rate: float, meas_angle: float, R_dynamic: float) -> float:
        now = time.monotonic()
        if self._t is None:
            self._t = now
            self.x[0, 0] = meas_angle
            return meas_angle
        dt = now - self._t
        self._t = now
        F = np.array([[1.0, -dt], [0.0, 1.0]])
        B = np.array([[dt], [0.0]])
        Q = np.diag([self.Q_angle, self.Q_bias])
        self.x = F @ self.x + B * gyro_rate
        self.P = F @ self.P @ F.T + Q
        y = meas_angle - float((self.H @ self.x)[0, 0])
        S = float((self.H @ self.P @ self.H.T)[0, 0]) + R_dynamic
        self.K = (self.P @ self.H.T) / S
        self.x = self.x + self.K * y
        self.P = (np.eye(2) - self.K @ self.H) @ self.P
        return float(self.x[0, 0])

    @property
    def bias(self):
        return float(self.x[1, 0])


# ─── Gyro Kalman Filter ───────────────────────────────────────────────
class GyroKalman:
    def __init__(self, dt, q_omega, q_omegadot, r_meas):
        self.dt = dt
        self.omega     = 0.0
        self.omegadot  = 0.0
        self.P00 = 1.0;  self.P01 = 0.0
        self.P10 = 0.0;  self.P11 = 1.0
        self.q0  = q_omega
        self.q1  = q_omegadot
        self.R   = r_meas

    def update(self, omega_raw):
        dt = self.dt
        omega_pred    = self.omega    + self.omegadot * dt
        omegadot_pred = self.omegadot
        P00 = self.P00 + dt*(self.P10 + self.P01) + dt*dt*self.P11 + self.q0
        P01 = self.P01 + dt*self.P11
        P10 = self.P10 + dt*self.P11
        P11 = self.P11 + self.q1
        y  = omega_raw - omega_pred
        S  = P00 + self.R
        K0 = P00 / S
        K1 = P10 / S
        self.omega    = omega_pred + K0 * y
        self.omegadot = omegadot_pred + K1 * y
        self.P00 = (1 - K0) * P00
        self.P01 = (1 - K0) * P01
        self.P10 = P10 - K1 * P00
        self.P11 = P11 - K1 * P01
        return self.omega


# ─── MPU9250 Driver ───────────────────────────────────────────────────
class MPU9250:
    def __init__(self):
        self.bus     = smbus2.SMBus(1)
        self.mag_asa = (1.0, 1.0, 1.0)
        self._setup()

    def _w(self, reg, val, addr=MPU_ADDR):
        self.bus.write_byte_data(addr, reg, val)

    def _r(self, reg, n, addr=MPU_ADDR):
        return bytes(self.bus.read_i2c_block_data(addr, reg, n))

    def _r1(self, reg, addr=MPU_ADDR):
        return self.bus.read_byte_data(addr, reg)

    def _setup(self):
        self._w(REG_PWR_MGMT_1, 0x00);  time.sleep(0.05)
        self._w(REG_PWR_MGMT_1, 0x01);  time.sleep(0.10)
        who = self._r1(REG_WHO_AM_I)
        print(f"[MPU9250] WHO_AM_I = 0x{who:02X}  (0x71=MPU9250  0x47=ICM-20689)")
        self._w(REG_SMPLRT_DIV,  0x00)
        self._w(REG_CONFIG,      0x05)
        self._w(REG_GYRO_CONFIG, 0x08)
        self._w(REG_ACCEL_CONFIG, 0x00)
        self._w(REG_ACCEL_CFG2,  0x05)
        self._w(REG_USER_CTRL,   0x00);  time.sleep(0.02)
        self._w(REG_INT_PIN_CFG, 0x02);  time.sleep(0.10)
        self._init_ak8963()

    def _init_ak8963(self):
        try:
            wia = self._r1(AK_WIA, AK_ADDR)
            print(f"[AK8963]  WHO_AM_I = 0x{wia:02X}  (expected 0x48)")
        except OSError:
            print("[AK8963]  WARNING: magnetometer not found at 0x0C — yaw = gyro only.")
            self.mag_asa = None
            return
        self._w(AK_CNTL1, 0x00, AK_ADDR);  time.sleep(0.02)
        self._w(AK_CNTL1, 0x0F, AK_ADDR);  time.sleep(0.02)
        asa = self._r(AK_ASAX, 3, AK_ADDR)
        self.mag_asa = tuple((v - 128) / 256.0 + 1.0 for v in asa)
        self._w(AK_CNTL1, 0x00, AK_ADDR);  time.sleep(0.02)
        self._w(AK_CNTL1, 0x16, AK_ADDR);  time.sleep(0.02)
        print(f"[AK8963]  Sensitivity adj = {tuple(round(a,4) for a in self.mag_asa)}")

    def accel(self):
        d = struct.unpack('>hhh', self._r(REG_ACCEL_XOUT, 6))
        return tuple(v / ACCEL_SCALE for v in d)

    def gyro(self):
        d = struct.unpack('>hhh', self._r(REG_GYRO_XOUT, 6))
        return tuple(v / GYRO_SCALE for v in d)

    def mag(self):
        if self.mag_asa is None:
            return None
        try:
            if not (self._r1(AK_ST1, AK_ADDR) & 0x01):
                return None
            raw = self._r(AK_XOUT_L, 7, AK_ADDR)
            if raw[6] & 0x08:
                return None
            vals = struct.unpack('<hhh', raw[:6])
            return tuple(vals[i] * self.mag_asa[i] * 0.15 for i in range(3))
        except OSError:
            return None

    def temp(self):
        d = struct.unpack('>h', self._r(REG_TEMP_OUT, 2))[0]
        return d / 333.87 + 21.0


# ─── Angle helpers ────────────────────────────────────────────────────
def accel_angles(ax, ay, az):
    roll  = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))
    return roll, pitch

def tilt_yaw(mx, my, mz, roll_r, pitch_r):
    cr, sr = math.cos(roll_r),  math.sin(roll_r)
    cp, sp = math.cos(pitch_r), math.sin(pitch_r)
    Bx =  mx*cp + my*sr*sp + mz*cr*sp
    By =          my*cr    - mz*sr
    y  = math.degrees(math.atan2(-By, Bx))
    return y + 360 if y < 0 else y

def compute_R_dynamic(accel_mag: float, hold_counter: int) -> tuple:
    accel_error = abs(accel_mag - 1.0)
    if accel_error > BLEND_G:
        return R_MOVING, R_HOLD_CYCLES
    if hold_counter > 0:
        return R_MOVING, hold_counter - 1
    blend = min(accel_error / BLEND_G, 1.0)
    R_dyn = R_STILL + blend * (R_MOVING - R_STILL)
    return R_dyn, 0


# ─── Main logger ──────────────────────────────────────────────────────
def main():
    print("Initialising MPU9250…")
    try:
        imu = MPU9250()
    except Exception as e:
        print(f"[FATAL] Could not init MPU9250: {e}")
        sys.exit(1)

    kf_roll  = KalmanFilter()
    kf_pitch = KalmanFilter()
    kf_yaw   = KalmanFilter()

    gyro_off  = {'gx': -0.8146, 'gy': 1.9096, 'gz': -0.6619}
    accel_off = {'x':  0.0, 'y':  0.0, 'z':  0.0}
    accel_scl = {'x':  1.0, 'y':  1.0, 'z':  1.0}
    mag_hi    = {'x':  0.0, 'y':  0.0, 'z':  0.0}
    mag_si    = {'x':  1.0, 'y':  1.0, 'z':  1.0}

    gx_o, gy_o, gz_o = gyro_off['gx'], gyro_off['gy'], gyro_off['gz']
    mag_available = imu.mag_asa is not None

    # ── CSV setup ─────────────────────────────────────────────────
    csv_file = "phase_variable_mpu_9250.csv"
    f_csv    = open(csv_file, 'w')
    header = ("time_s,roll_deg,pitch_deg,yaw_deg,temp_c,bias_roll,bias_pitch,accel_norm_g,"
              "K_roll_angle,K_roll_bias,K_pitch_angle,K_pitch_bias,"
              "P_roll_00,P_roll_11,P_pitch_00,P_pitch_11,"
              "gyro_pitch_dps,pitch_centered_deg,R_dynamic,gyro_pitch_filt,"
              "phase_var\n")
    f_csv.write(header)
    f_csv.flush()

    print(f"\nLogging to : {csv_file}")
    print(f"Duration   : {RUN_DURATION} s   (Ctrl+C to stop early)")
    print(f"Gyro range : ±500 °/s   DLPF: 20 Hz   R_HOLD: {R_HOLD_CYCLES} cycles")
    print(f"Phase scaling : A={PHASE_A} deg   B={PHASE_B} deg/s\n")

    # ── Stabilization delay ───────────────────────────────────────
    STABILIZE_DURATION = 5.0
    print(f"Stabilizing for {STABILIZE_DURATION:.0f}s — keep IMU still…")
    t_stab_start = time.monotonic()
    while time.monotonic() - t_stab_start < STABILIZE_DURATION:
        try:
            ax, ay, az = imu.accel()
            gx, gy, gz = imu.gyro()
        except OSError:
            pass
        remaining = STABILIZE_DURATION - (time.monotonic() - t_stab_start)
        print(f"\r  {remaining:4.1f} s remaining…", end='', flush=True)
        time.sleep(1.0 / LOOP_HZ)
    print("\nStabilization done — starting logging.\n")

    # ── GyroKalman init ───────────────────────────────────────────
    dt_nominal = 1.0 / LOOP_HZ
    gyro_kf = GyroKalman(
        dt         = dt_nominal,
        q_omega    = GK_Q_OMEGA,
        q_omegadot = GK_Q_OMEGADOT,
        r_meas     = GK_R
    )

    running = [True]
    def _stop(sig, frame):
        running[0] = False
    signal.signal(signal.SIGINT, _stop)

    last_yaw      = 0.0
    dt_target     = 1.0 / LOOP_HZ
    t_start       = time.monotonic()
    row_count     = 0
    hold_roll     = 0
    hold_pitch    = 0
    gyro_pitch_filt = 0.0
    prev_pitch    = None
    t_prev        = None

    # ── Phase variable monotonicity safeguard state ────────────────
    prev_phase_var   = None
    clamp_count      = 0
    sensor_fault_flag = False

    while running[0]:
        t0    = time.monotonic()
        t_rel = t0 - t_start

        if t_rel >= RUN_DURATION:
            break

        try:
            ax, ay, az = imu.accel()
            gx, gy, gz = imu.gyro()
        except OSError:
            time.sleep(0.01)
            continue

        gx -= gx_o;  gy -= gy_o;  gz -= gz_o
        ax  = (ax - accel_off['x']) * accel_scl['x']
        ay  = (ay - accel_off['y']) * accel_scl['y']
        az  = (az - accel_off['z']) * accel_scl['z']

        accel_norm = math.sqrt(ax**2 + ay**2 + az**2)

        R_dyn, hold_roll  = compute_R_dynamic(accel_norm, hold_roll)
        _,     hold_pitch = compute_R_dynamic(accel_norm, hold_pitch)

        a_roll, a_pitch = accel_angles(ax, ay, az)

        roll  = kf_roll.update(gx,  a_roll,  R_dyn)
        pitch = kf_pitch.update(gy, a_pitch, R_dyn)

        if mag_available:
            m = imu.mag()
            if m:
                mx = (m[0] - mag_hi['x']) * mag_si['x']
                my = (m[1] - mag_hi['y']) * mag_si['y']
                mz = (m[2] - mag_hi['z']) * mag_si['z']
                mag_yaw  = tilt_yaw(mx, my, mz,
                                    math.radians(roll), math.radians(pitch))
                last_yaw = kf_yaw.update(gz, mag_yaw, R_STILL)
            else:
                last_yaw += gz * dt_target
                if last_yaw >  180: last_yaw -= 360
                if last_yaw < -180: last_yaw += 360
        else:
            last_yaw += gz * dt_target
            if last_yaw >  180: last_yaw -= 360
            if last_yaw < -180: last_yaw += 360

        pitch_centered  = pitch - DC_OFFSET_PITCH
        if prev_pitch is None or t_prev is None:
            gyro_pitch_dps = 0.0
        else:
            dt_actual      = t0 - t_prev
            gyro_pitch_dps = (pitch - prev_pitch) / dt_actual
        prev_pitch = pitch
        t_prev     = t0
        gyro_pitch_filt = gyro_kf.update(gyro_pitch_dps)

        # ── Phase variable — fixed A and B, atan2 → 0 to 1 ──────
        phi_raw  = math.atan2(gyro_pitch_filt / PHASE_B, pitch_centered / PHASE_A)  # -π to π
        phase_var = 1 - ((phi_raw + math.pi) / (2 * math.pi))                              #  0 to 1

        # ── Monotonicity safeguard ──────────────────────────────
        if prev_phase_var is None:
            clamp_count = 0
        else:
            is_wraparound = (prev_phase_var > WRAP_THRESHOLD) and (phase_var < (1 - WRAP_THRESHOLD))
            if is_wraparound:
                clamp_count = 0
            elif phase_var <= prev_phase_var:
                phase_var = prev_phase_var + MIN_INCREMENT
                clamp_count += 1
                if clamp_count > MAX_CONSEC_CLAMP:
                    sensor_fault_flag = True
            else:
                clamp_count = 0
        prev_phase_var = phase_var

        # ── Write CSV row ──────────────────────────────────────────
        f_csv.write(
            f"{round(t_rel,  4)},"
            f"{round(roll,   4)},"
            f"{round(pitch,  4)},"
            f"{round(last_yaw, 4)},"
            f"{round(imu.temp(), 2)},"
            f"{round(kf_roll.bias,  6)},"
            f"{round(kf_pitch.bias, 6)},"
            f"{round(accel_norm, 4)},"
            f"{round(float(kf_roll.K[0,0]),  6)},"
            f"{round(float(kf_roll.K[1,0]),  6)},"
            f"{round(float(kf_pitch.K[0,0]), 6)},"
            f"{round(float(kf_pitch.K[1,0]), 6)},"
            f"{round(float(kf_roll.P[0,0]),  6)},"
            f"{round(float(kf_roll.P[1,1]),  6)},"
            f"{round(float(kf_pitch.P[0,0]), 6)},"
            f"{round(float(kf_pitch.P[1,1]), 6)},"
            f"{round(gyro_pitch_dps, 4)},"
            f"{round(pitch_centered, 4)},"
            f"{round(R_dyn,          4)},"
            f"{round(gyro_pitch_filt, 4)},"
            f"{round(phase_var,       4)}\n"
        )
        row_count += 1

        if row_count % 100 == 0:
            f_csv.flush()

        if row_count % 10 == 0:
            print(
                f"\r  {int((t_rel/RUN_DURATION)*100):3d}%"
                f" | roll={roll:+7.2f}°"
                f" | pitch={pitch:+7.2f}°"
                f" | R={R_dyn:.3f}"
                f" | hold={hold_roll}"
                f" | φ={phase_var:.3f}"
                f" | fault={sensor_fault_flag}",
                end='', flush=True
            )

        spare = dt_target - (time.monotonic() - t0)
        if spare > 0:
            time.sleep(spare)

    f_csv.flush()
    f_csv.close()
    print(f"\n\nSaved {row_count} rows → {csv_file}")


if __name__ == "__main__":
    main()
```

---

## Appendix C: `matlab_phase-variable_mpu_9250.m` (full source)

```matlab
% matlab_phase-variable_mpu_9250.m
% Verifies that phase_var moves from 0 to 1 correctly over time
% Load CSV output from imu_thigh_angle_phase.py and plot phase_var vs time
clear; clc; close all;
%% ── Load data ────────────────────────────────────────────────────────
data = readtable('imu_phase_15june_7th.csv');
t         = data.time_s;
phase_var = data.phase_var;
pitch     = data.pitch_centered_deg;
gyro_filt = data.gyro_pitch_filt;
%% ── Plot 1: phase_var vs time ────────────────────────────────────────
figure('Name', 'Phase Variable vs Time', 'NumberTitle', 'off');
subplot(3,1,1);
plot(t, phase_var, 'b', 'LineWidth', 1.2);
yline(0, 'k--', 'LineWidth', 0.8);
yline(1, 'k--', 'LineWidth', 0.8);
ylim([-0.1 1.1]);
xlabel('Time (s)');
ylabel('\phi (0 \rightarrow 1)');
title('Phase Variable over Time');
grid on;
subplot(3,1,2);
plot(t, pitch, 'r', 'LineWidth', 1.0);
xlabel('Time (s)');
ylabel('Pitch centered (deg)');
title('pitch\_centered\_deg  [A = 20 deg]');
yline(20,  'r--', '+A');
yline(-20, 'r--', '-A');
grid on;
subplot(3,1,3);
plot(t, gyro_filt, 'm', 'LineWidth', 1.0);
xlabel('Time (s)');
ylabel('Gyro filtered (deg/s)');
title('gyro\_pitch\_filt  [B = 100 deg/s]');
yline(100,  'm--', '+B');
yline(-100, 'm--', '-B');
grid on;
sgtitle('Phase Variable Verification  (A=20 deg, B=100 deg/s)');
%% ── Plot 2: phase portrait with phase colour ─────────────────────────
figure('Name', 'Phase Portrait coloured by phi', 'NumberTitle', 'off');
scatter(pitch, gyro_filt, 8, phase_var, 'filled');
colormap(hsv);
cb = colorbar;
cb.Label.String = '\phi (0 \rightarrow 1)';
xlabel('pitch\_centered\_deg');
ylabel('gyro\_pitch\_filt (deg/s)');
title('Phase Portrait — colour = phase variable \phi');
xline(0, 'k--'); yline(0, 'k--');
grid on;
%% ── Print summary stats ──────────────────────────────────────────────
fprintf('\n── Phase variable stats ──\n');
fprintf('  Min  : %.4f\n', min(phase_var));
fprintf('  Max  : %.4f\n', max(phase_var));
fprintf('  Range: %.4f\n', max(phase_var) - min(phase_var));
fprintf('  Mean : %.4f\n', mean(phase_var));
fprintf('\n  If range ≈ 1.0 and plot shows sawtooth → phase is working correctly.\n');
fprintf('  If range is small → A or B needs adjustment (signal not reaching limits).\n\n');
```
