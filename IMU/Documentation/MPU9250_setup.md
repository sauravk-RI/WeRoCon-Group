# MPU9250 — Full Setup Guide
Wiring → Raspberry Pi I2C setup → Python library installs →
calibration → running the logger → plotting the CSV in MATLAB.

Written so someone starting from zero can follow it top to bottom and
end up with a working thigh-angle logger and MATLAB plots.

---

## 1. What you need

- MPU9250 breakout board (MPU6500 accel/gyro + AK8963 magnetometer)
- Raspberry Pi (any model with I2C; this guide assumes Pi 5)
- 4 jumper wires (breadboard optional but recommended)
- MATLAB (any recent version — no toolboxes required for plotting)

**Files in this setup:**
| File | Purpose |
|---|---|
| `verify_mpu9250.py` | Checks whether your board is a genuine MPU9250/GY-91 or a fake/relabeled clone (Section 6) |
| `imu_thigh_angle.py` | Main Kalman-filter logger, writes `imu_data.csv` |
| `plot_imu.m` | MATLAB script to plot the CSV output |

**Everything in this guide happens inside one project folder.** Create
it first and keep all three files above inside it — every command
below assumes you're `cd`'d into this folder:

```bash
mkdir imu_project
cd imu_project
```

Copy `verify_mpu9250.py` and `imu_thigh_angle.py` into this folder
(and later copy `imu_data.csv` out of it into wherever your MATLAB
project lives, or just also drop `plot_imu_data.m` in here and run
MATLAB pointed at this folder). The virtual environment
(`sensor_env`) also gets created inside this same folder — see
Section 4.

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
5, and 6, but keep this handy if you add other sensors later):

![Raspberry Pi 5 GPIO pinout]<img width="2016" height="1168" alt="image" src="https://github.com/user-attachments/assets/eec8f2a8-98b5-4abe-bb5c-a0de12762ce7" />


- Leave **AD0 tied to GND** — this keeps the accel/gyro at I2C address
  `0x68`. If AD0 is pulled high instead, the address becomes `0x69`
  (this script assumes `0x68`).
- The MPU9250 module wires both the MPU6050 (accel/gyro) and the AK8963 (magnetometer) to the same I2C port internally via an auxiliary I2C bus — you don't wire the magnetometer separately, it rides on the same SDA/SCL lines once bypass mode is enabled (the script does this for you).

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
you what's plugged in. Use this table to read the result for the
sensors covered in this guide (and the GV7-AR guide, if you're
running both on the same Pi):

| Address shown | What it means |
|---|---|
| `68` | MPU9250/MPU9255/MPU6500 accel+gyro die, with **AD0 tied to GND** (the default wiring in Section 2) |
| `69` | Same accel+gyro die, but with **AD0 tied to 3.3V** instead — if you see this instead of `68`, update `MPU_ADDR` in every script to `0x69` |
| `0C` | AK8963 magnetometer — **only appears after bypass mode is enabled**, which only happens once you run `imu_thigh_angle.py` or `verify_mpu9250.py`. A bare `i2cdetect` right after wiring, before running any script, will normally show `68` alone with `0C` absent — that's expected, not a fault |
| `76` or `77` | BMP280 barometer — present on genuine GY-91 boards. Its presence does **not** confirm the IMU part is genuine (see Section 6) |
| Nothing at all | Check wiring (Section 2), check the Pi is powered, and check you actually rebooted after enabling I2C |
| `UU` instead of an address | A kernel driver has already claimed that address — usually harmless for our purposes, but means you can't talk to it directly over raw I2C without unloading that driver first |

![If MPU9250 has all sensors]<img width="2171" height="724" alt="image" src="https://github.com/user-attachments/assets/b3e2b86b-8b4a-4af0-a6bb-4101da4f20ab" />


A typical **first scan**, right after wiring and before running any
script, looks like this:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- 76 --
`
That's a genuine board with only `68` (accel/gyro) and `76` (BMP280)
visible — the `0C` (magnetometer) hasn't shown up yet because bypass
mode isn't enabled until the Python script runs. This is normal, not
a sign of a fake board — don't judge authenticity from `i2cdetect`
alone; use `verify_mpu9250.py` (Section 6) instead, since it actively
enables bypass mode before checking for the magnetometer.

**If `i2cdetect` shows nothing at all:** check wiring and that the Pi
is powered; recheck 3.3V vs 5V (the MPU9250 is a 3.3V part — some
breakout boards have an onboard regulator that tolerates 5V, check
your board's silkscreen/datasheet before assuming).

---

## 4. Install Python libraries

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

## 5. A known AK8963 gotcha (read this before you run anything)

On some Pi/kernel combinations, even with bypass mode correctly
enabled (`INT_PIN_CFG = 0x02`), the AK8963 magnetometer refuses to
respond and raises:
```
OSError: [Errno 121] Remote I/O error
```
This can happen even when the MPU9250 registers correctly indicate bypass mode is enabled — i2cdetect still won't show a device at 0x0C in that case.

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

## 6. Check whether your MPU9250/GY-91 is genuine (do this first!)

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



The script has placeholder calibration values (all zeros/ones) that
apply **no correction**. For real use you need:

1. **Gyro bias** — with the sensor completely still on a flat, level
   surface, log a few seconds of gyro data and average it. That
   average is your `gyro_off` for gx/gy/gz.
2. **Accelerometer offset/scale** — the standard 6-position
   calibration (each axis up and down, flat) gives you `accel_off`
   and `accel_scl` per axis.
3. **Magnetometer hard/soft iron** — rotate the sensor through a
   figure-eight motion while logging raw mag readings, then fit an
   ellipsoid to get `mag_hi` (hard-iron offset) and `mag_si`
   (soft-iron scale). A typical calibration routine has you wave the device in a figure eight until enough samples are collected to compute these biases and scale factors.

Paste all of these values into the **Calibration offsets** section
near the top of `main()` in the script before logging real data.
Skipping this step means your roll/pitch will have a constant offset
and yaw will drift or be distorted by nearby metal/magnets.

---

## 8. Tune the Kalman filter

Look at the **TUNING CONSTANTS** section near the top of the script:

```python
R_STILL        = 0.08     # Kalman R when sensor is still
R_MOVING       = 2.5      # Kalman R during movement
R_HOLD_CYCLES  = 12       # Samples to block accel after heel strike
BLEND_G        = 0.15     # Accel deviation (g) that triggers R_MOVING
Q_ANGLE        = 0.005    # Process noise — angle
Q_BIAS         = 0.003    # Process noise — bias
```

These defaults work reasonably for a leg-mounted sensor doing gait
analysis. If you have a `tuning_constant.py` / noise-profiler script
for your specific mounting and motion pattern, run that first and
paste its output values here instead of using the defaults — every
mounting location and use case (walking gait vs. general robotics vs.
vehicle IMU) has different noise characteristics.

---

## 9. Run the logger

```bash
cd imu_project   # if not already there
source sensor_env/bin/activate
python3 imu_thigh_angle.py
```

Expected output:
```
Initialising MPU9250…
[MPU9250] WHO_AM_I = 0x71  (0x71=MPU9250  0x47=ICM-20689)
[AK8963]  WHO_AM_I = 0x48  (expected 0x48)
[AK8963]  Sensitivity adj = (1.21, 1.22, 1.17)

Logging to : imu_data.csv
Duration   : 10.0 s   (Ctrl+C to stop early)
Gyro range : ±500 °/s   DLPF: 20 Hz   R_HOLD: 12 cycles

  100% | roll=  +2.14° | pitch=  -1.03° | R=0.080 | hold=0

Saved 1000 rows → imu_data.csv
```

- `RUN_DURATION` (default 10.0s) controls how long it logs — increase
  this for real trials.
- Press `Ctrl+C` to stop early — the CSV is flushed and closed safely
  either way.
- The CSV columns are: `time_s, roll_deg, pitch_deg, yaw_deg, temp_c,
  bias_roll, bias_pitch, accel_norm_g, K_roll_angle, K_roll_bias,
  K_pitch_angle, K_pitch_bias, P_roll_00, P_roll_11, P_pitch_00,
  P_pitch_11` — everything needed to inspect filter behavior, not just
  final angles.

---

## 10. Plot the CSV in MATLAB

1. Copy `imu_data.csv` into the same folder as `plot_imu_data.m`
   (or edit the `filename` variable at the top of the script).
2. Open MATLAB, navigate to that folder, and run:
   ```matlab
   plot_imu_data
   ```

This produces 5 figures:

| Figure | Shows |
|---|---|
| **Orientation** | Roll/pitch/yaw vs time — the main result |
| **Accel Norm** | `\|accel\|` in g, with a dashed line at 1g — spikes above/below show motion events (e.g. heel strikes) |
| **Gyro Bias Estimates** | The Kalman filter's live estimate of gyro drift for roll/pitch |
| **Kalman Gain** | How much the filter trusts new accel measurements vs. the gyro prediction, over time |
| **Covariance** | Filter confidence (P matrix diagonal) for roll and pitch |

A summary is also printed to the MATLAB command window: sample count,
duration, average sample rate, mean temperature, and final bias
estimates.

No MATLAB toolboxes are required — it only uses `readtable` and base
plotting functions.

---

## 11. Troubleshooting

| Symptom | Likely cause |
|---|---|
| `[FATAL] Could not init MPU9250` | Wrong wiring, I2C not enabled, or address conflict — recheck Section 3 |
| `WHO_AM_I` prints something other than `0x71` | Wrong sensor variant, bad wiring, or address mismatch — check AD0 pin |
| AK8963 `Remote I/O error` | See Section 5 — script falls back gracefully, but try power-cycling the sensor |
| Roll/pitch have a constant offset | Calibration not applied yet — see Section 6 |
| Yaw drifts steadily even when still | No magnetometer correction active (gyro-only fallback), or mag calibration not applied |
| Values look noisy/jittery | Check `DLPF` settings weren't changed, and confirm the sensor is mounted rigidly (loose mounting = vibration noise) |
| MATLAB `readtable` errors | Confirm `imu_data.csv` is in the working directory and wasn't truncated by a mid-run crash — reopen and check the last line isn't cut off |
| Magnetometer never works, no matter what you try | Run `verify_mpu9250.py` (Section 6) — you may have a fake/relabeled MPU6500 board with no magnetometer die at all |

---

## 12. Reference links

- MPU-9250 register map / datasheet: search "InvenSense MPU-9250 Register Map"
- smbus2 docs: https://pypi.org/project/smbus2/
- Maker Portal MPU9250 + Raspberry Pi tutorial (wiring diagrams, background): https://makersportal.com/blog/2019/11/11/raspberry-pi-python-accelerometer-gyroscope-magnetometer
- Kalman filter for IMU tilt estimation (background reading): search "complementary vs Kalman filter IMU tilt"
