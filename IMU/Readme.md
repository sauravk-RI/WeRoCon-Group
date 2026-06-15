# MPU9250 IMU — Prosthetic Leg Orientation Tracking

![Python](https://img.shields.io/badge/Python-3.9%2B-3776AB?style=flat-square&logo=python&logoColor=white)
![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-5-C51A4A?style=flat-square&logo=raspberrypi&logoColor=white)
![MPU9250](https://img.shields.io/badge/Sensor-MPU9250-00ADD8?style=flat-square)
![Kalman Filter](https://img.shields.io/badge/Filter-Kalman-FF6B35?style=flat-square)
![WebSocket](https://img.shields.io/badge/Stream-WebSocket-4ADE80?style=flat-square&logo=websocket&logoColor=white)
![MATLAB](https://img.shields.io/badge/Analysis-MATLAB-E16737?style=flat-square&logo=mathworks&logoColor=white)
![Status](https://img.shields.io/badge/Status-Active-brightgreen?style=flat-square)
![PRs](https://img.shields.io/badge/PRs-welcome-blueviolet?style=flat-square)

Real-time roll/pitch/yaw estimation for a prosthetic leg using an
MPU9250 (accelerometer + gyroscope + AK8963 magnetometer) on a
Raspberry Pi 5, fused with a discrete Kalman filter.

This repo contains two tracks:

| File              | Purpose                                                              |
|-------------------|-----------------------------------------------------------------------|
| `imu_server.py`   | Live WebSocket server — streams filtered roll/pitch/yaw to a browser dashboard |
| `dashboard.html`  | Browser dashboard with live gauges, charts, footstrike detection, and customizable scale |
| `imu_logger.py`   | Standalone CSV logger (10 s capture) for offline analysis in MATLAB   |
| `imu_analysis.m`  | MATLAB script to load the CSV and generate angle / diagnostic plots   |
| `calibration.json`| Optional calibration file (gyro offsets, accel offsets/scale, mag hard/soft iron) used by the **extended** logger |

---

## 1. Hardware Setup

- **Board:** Raspberry Pi 5
- **Sensor:** MPU9250 (with onboard AK8963 magnetometer)
- **Connection:** I2C (SDA → GPIO2, SCL → GPIO3, VCC → 3.3V, GND → GND)
- **Mounting:** Rigid mount strongly recommended (see [Mounting Notes](#mounting-notes) below) — loose mounting is the single biggest source of noise on a prosthetic leg.

### Enable I2C on the Pi

```bash
sudo raspi-config
# → Interface Options → I2C → Enable
sudo reboot
```

Verify the sensor is detected:

```bash
sudo apt install -y i2c-tools
i2cdetect -y 1
```

You should see `0x68` (MPU9250) and `0x0C` (AK8963, once bypass mode is enabled by the script).

---

## 2. Software Setup

```bash
git clone <this-repo-url>
cd <repo-folder>

pip install smbus2 numpy websockets --break-system-packages
```

| Package      | Used by                          |
|--------------|-----------------------------------|
| `smbus2`     | I2C communication with MPU9250/AK8963 |
| `numpy`      | Kalman filter matrix math          |
| `websockets` | Live streaming to the dashboard (`imu_server.py` only) |

---

## 3. Quick Start — Live Dashboard

```bash
python3 imu_server.py
```

On first run, the script will:

1. Wake the MPU9250 and read `WHO_AM_I` (should print `0x71`)
2. Enable I2C bypass mode so the Pi can talk to the AK8963 magnetometer directly
3. Read the AK8963 `WHO_AM_I` (should print `0x48`)
4. **Calibrate the gyro** — keep the sensor completely still for 2 seconds
5. **Calibrate the magnetometer** — rotate the sensor through all orientations for 5 seconds
6. Start streaming filtered roll/pitch/yaw over WebSocket at `ws://<pi-ip>:8765`

Open `dashboard.html` in any browser (on the Pi or another device on the same network), enter the Pi's IP in the connect bar, and click **Connect**.

---

## 4. The Kalman Filter

Both `imu_server.py` and `imu_logger.py` use the same 2-state discrete
Kalman filter to fuse gyroscope and accelerometer data into a stable
angle estimate.

### State vector

```
x = [angle, gyro_bias]ᵗ
```

### Prediction step

```
x̂_k|k-1 = F · x̂_{k-1} + B · u_k
P_k|k-1  = F · P · Fᵗ + Q

F = [[1, -dt], [0, 1]]
B = [[dt], [0]]
Q = diag(Q_angle, Q_bias)
```

### Update step

```
y = z - H · x̂_k|k-1          (H = [1, 0])
S = H · P · Hᵗ + R
K = P · Hᵗ / S
x̂ = x̂_k|k-1 + K · y
P  = (I - K · H) · P_k|k-1
```

- **`Q_angle`** — how much you trust the gyro-predicted angle to drift between steps. Higher = filter follows fast motion more readily but is noisier.
- **`Q_bias`** — how fast the estimated gyro bias is allowed to change.
- **`R_measure`** — how much you trust the accelerometer-derived angle. Higher = filter relies more on the gyro and less on (noisy) accelerometer readings.

### Tuned parameter sets

| Use case                          | Q_angle | Q_bias | R_measure |
|------------------------------------|---------|--------|-----------|
| Hand-held / slow motion (original) | 0.001   | 0.003  | 0.03      |
| **Prosthetic leg (recommended)**   | 0.003   | 0.005  | 0.15      |

The prosthetic-leg values trust the accelerometer less, since footstrike
vibration spikes the accelerometer reading well above 1 g during gait.

---

## 5. Why It's Noisy On a Real Leg (and how the fixes work)

If the filter looks smooth when you wave the sensor by hand but noisy
once strapped to a leg, the cause is almost always one or more of:

1. **Footstrike shock** — 50–200 Hz vibration spike at heel-strike
2. **Loose mounting** — Velcro/tape allows micro-movement between the IMU and the limb
3. **Structural resonance** — the pylon/socket vibrates at its own frequency
4. **High loop rate** — 100 Hz captures vibration content a slower loop would average out

### Fixes implemented in `imu_server.py`

- **EMA pre-filter** (`EMA_ALPHA = 0.15`) — smooths accel/gyro before the Kalman filter sees them
- **Retuned Kalman gains** — see table above
- **Footstrike detection** — when accel magnitude `|a|` exceeds `FOOTSTRIKE_THRESHOLD` (default `1.4 g`), the Kalman update step is skipped for that cycle (gyro-only propagation), preventing the heel-strike spike from corrupting the angle estimate
- **Reduced loop rate** — `LOOP_HZ = 50` and `SMPLRT_DIV = 0x13` (50 Hz hardware sample rate)

### Mounting Notes

No filter fully compensates for a sensor that physically moves relative
to the limb. In order of effectiveness:

1. 3D-printed rigid cuff bolted/clipped to the pylon (best)
2. Velcro strap over a rigid backing plate, pressing the IMU flat against the limb
3. Kinesiology (KT) tape wrapped directly over the unit — cheap interim fix, much better than Velcro alone

### Tuning the footstrike threshold

`dashboard.html` shows a live **Accel |g|** chart with a dashed red line
at the current `FOOTSTRIKE_THRESHOLD`. During normal walking:

- Mid-stance / swing phases should sit near `1.0 g`
- Heel-strike peaks typically reach `1.5–2.5 g`

Set the threshold just above the quiet-phase ceiling for your patient's
gait, then update `FOOTSTRIKE_THRESHOLD` in `imu_server.py`.

---

## 6. Offline Logging for MATLAB

```bash
python3 imu_logger.py
```

- Runs for **10 seconds** (`RUN_DURATION = 10.0`, edit this constant to change)
- Press `Ctrl+C` to stop early — partial data is saved safely
- Output: `imu_data_YYYYMMDD_HHMMSS.csv`

### CSV columns

```
timestamp_s, roll_deg, pitch_deg, yaw_deg, temp_c, bias_roll, bias_pitch
```

### Analyzing in MATLAB

```matlab
>> imu_analysis
```

`imu_analysis.m` automatically:

- Loads the most recent `imu_data_*.csv` in the folder (or prompts you to pick one)
- Prints summary stats (mean / std / min / max for roll, pitch, yaw, temperature)
- Plots roll, pitch, and yaw individually and combined
- Plots Kalman gyro-bias estimates, temperature, pitch angular rate, and sample-interval distribution (to verify the loop is running at a steady rate)
- Exports all figures as PNGs

---

## 7. Extended Logger — Calibration Support

`imu_logger.py` supports an optional `calibration.json` file in the same
directory. If present, it applies:

- **Gyro offset correction** — `gx_o, gy_o, gz_o` subtracted from raw gyro readings (in addition to the runtime gyro calibration)
- **Accelerometer offset + scale correction** — `corrected = (raw - offset) * scale` per axis
- **Magnetometer hard-iron + soft-iron correction** — `corrected = (raw - hard_iron) * soft_iron` per axis

If `calibration.json` is **not found**, the logger prints a warning and
proceeds with raw sensor data (gyro is still self-calibrated for 2 s at
startup as in the basic version).

### `calibration.json` format

```json
{
  "gyro": {
    "gx": 0.0123,
    "gy": -0.0087,
    "gz": 0.0041
  },
  "accel": {
    "offset": { "x": 0.012, "y": -0.005, "z": 0.031 },
    "scale":  { "x": 1.002, "y": 0.998,  "z": 1.005 }
  },
  "mag": {
    "hard_iron": { "x": 12.4, "y": -8.1, "z": 3.7 },
    "soft_iron": { "x": 1.03, "y": 0.97, "z": 1.01 }
  }
}
```

### How to generate calibration values

1. **Gyro offsets** — keep the sensor perfectly still and average raw gyro readings over several seconds (the basic logger already does this automatically at runtime, so this field is mainly for persisting a known-good offset across runs).
2. **Accel offset/scale** — place the sensor flat on each of its 6 faces in turn and record the accelerometer reading at rest (should read ±1g on the gravity axis, 0 on the others). Compute offset and scale per axis from these six readings using the standard 6-position calibration method.
3. **Magnetometer hard/soft iron** — rotate the sensor through as many orientations as possible while logging raw magnetometer values, then fit an ellipsoid to the data; the ellipsoid center gives hard-iron offsets and the axis ratios give soft-iron scale factors. (Tools like MATLAB's `magcal()` function or open-source ellipsoid-fit scripts work well for this.)

Once you have a `calibration.json`, drop it next to `imu_logger.py` —
no code changes needed, it's picked up automatically.

---

## 8. Troubleshooting

| Symptom                                  | Likely cause / fix |
|-------------------------------------------|---------------------|
| `WHO_AM_I = 0x00` or `OSError`            | Check I2C wiring, confirm `i2cdetect -y 1` shows `0x68` |
| `[AK8963] WARNING: cannot reach magnetometer` | Bypass mode failed to enable — power-cycle the sensor and retry; yaw will fall back to gyro-only integration |
| Smooth on desk, noisy on leg              | See [Section 5](#5-why-its-noisy-on-a-real-leg-and-how-the-fixes-work) — mounting and footstrike detection |
| MATLAB error `Unrecognized table variable name 'timestamp_s'` | Make sure you're using the CSV header from this version (`timestamp_s,...`), not an older logger version |
| Dashboard shows "⚠ Cannot connect"        | Confirm the Pi's IP address, that `imu_server.py` is running, and both devices are on the same network/subnet |

