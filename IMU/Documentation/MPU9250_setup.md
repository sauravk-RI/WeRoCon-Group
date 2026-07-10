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

![Raspberry Pi 5 GPIO pinout]<img width="2016" height="1168" alt="image" src="https://github.com/user-attachments/assets/3abd1738-c3a5-4fe1-8ebb-aabffc17d5b5" />


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

Install the I2C tools and confirm the sensor is detected:
```bash
sudo apt update
sudo apt install -y i2c-tools
i2cdetect -y 1
```

You should see a device at `0x68`. A correctly working setup shows the MPU9250 at address 0x68 and the AK8963 magnetometer at 0x0C — but note the magnetometer only appears at `0x0C` **after** bypass mode is enabled by the script; a bare `i2cdetect` before running any code typically shows only `0x68`, and that's normal.

**If `i2cdetect` shows nothing at all:** check wiring and that the Pi is powered; recheck 3.3V vs 5V (the MPU9250 is a 3.3V part — some breakout boards have an onboard regulator that tolerates 5V, check your board's silkscreen/datasheet before assuming).

---

## 4. Install Python libraries

```bash
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

## 6. Calibrate the sensor (do this before trusting any output)

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

## 7. Tune the Kalman filter

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

## 8. Run the logger

```bash
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

## 9. Plot the CSV in MATLAB

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

## 10. Troubleshooting

| Symptom | Likely cause |
|---|---|
| `[FATAL] Could not init MPU9250` | Wrong wiring, I2C not enabled, or address conflict — recheck Section 3 |
| `WHO_AM_I` prints something other than `0x71` | Wrong sensor variant, bad wiring, or address mismatch — check AD0 pin |
| AK8963 `Remote I/O error` | See Section 5 — script falls back gracefully, but try power-cycling the sensor |
| Roll/pitch have a constant offset | Calibration not applied yet — see Section 6 |
| Yaw drifts steadily even when still | No magnetometer correction active (gyro-only fallback), or mag calibration not applied |
| Values look noisy/jittery | Check `DLPF` settings weren't changed, and confirm the sensor is mounted rigidly (loose mounting = vibration noise) |
| MATLAB `readtable` errors | Confirm `imu_data.csv` is in the working directory and wasn't truncated by a mid-run crash — reopen and check the last line isn't cut off |

---

## 11. Reference links

- MPU-9250 register map / datasheet: search "InvenSense MPU-9250 Register Map"
- smbus2 docs: https://pypi.org/project/smbus2/
- Maker Portal MPU9250 + Raspberry Pi tutorial (wiring diagrams, background): https://makersportal.com/blog/2019/11/11/raspberry-pi-python-accelerometer-gyroscope-magnetometer
- Kalman filter for IMU tilt estimation (background reading): search "complementary vs Kalman filter IMU tilt"
