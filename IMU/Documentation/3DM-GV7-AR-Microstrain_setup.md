# MicroStrain 3DM-GV7-AR — Full Setup Guide
Hardware mounting → SensorConnect configuration → Python environment →
live dashboard + CSV logging.

Anyone starting from zero should be able to follow this top to bottom
and end up with live roll/pitch/yaw in a browser and data saved to CSV.

---

## 1. What you need

- MicroStrain 3DM-GV7-AR sensor + its official cable/connector
- A PC (Windows recommended) for initial configuration via SensorConnect
- A Raspberry Pi (or any Linux machine) to run the logger continuously
- USB cable (sensor → adapter → USB) or RS-232/RS-422 wiring
- Python 3.9+ on the Pi

---

## 2. Mount the sensor correctly

- **Vent must NOT face up.** Mounting it vent-up can compromise the
  IP68 seal or clog airflow to the internal pressure sensor.
- Use the correct MicroStrain cable — the IP68 rating only holds if
  the cable is installed properly per the mechanical spec.
- Only power it within its rated voltage range. Reversed polarity can
  permanently damage the unit.
- Default axis convention: **X = direction of travel, Z = down**. If
  your mounting orientation differs, you'll set a sensor-to-vehicle
  transform in step 4.

---

## 3. Install SensorConnect (on your PC first)

SensorConnect is MicroStrain's official configuration + visualization
tool for the GV7 family.

1. Download it: **microstrain.com/software/sensorconnect**
2. Install and launch it.
3. Plug the GV7-AR into your PC via USB. SensorConnect should
   auto-detect it and show the model name and serial number.
4. Go to the **Firmware/Downloads** section on the GV7-AR product page
   and check you're running the latest firmware. Upgrade if needed —
   SensorConnect handles this in-app.

---

## 4. Configure the sensor in SensorConnect

1. **Sampling tile** — enable the data channels you'll stream:
   - Estimated Orientation (Euler: roll/pitch/yaw)
   - Estimated Angular Rate (gyro X/Y/Z)
   Set your sample rate (the GV7-AR supports up to 1000 Hz; 100 Hz is
   a good default for most logging).
2. **Mounting transform** (if the sensor isn't mounted axis-aligned
   with your vehicle/body frame) — set this under the device
   configuration so orientation output is corrected automatically.
3. **Capture gyro bias** — do this once, after mounting, with the
   sensor completely stationary (motors off too). This removes drift
   introduced during shipping/installation. Save it to non-volatile
   memory so it survives power cycles.
4. Watch the live graphs in SensorConnect for a minute and confirm
   roll/pitch/yaw values look sane (near 0 at rest, respond correctly
   when you tilt the sensor).
5. **Disconnect SensorConnect** before running the Python script —
   only one program can hold the serial port at a time.

---

## 5. Set up the Raspberry Pi (or Linux logging machine)

```bash
mkdir gv7_project
cd gv7_project

# 1. Create a virtual environment
python3 -m venv sensor_env
source sensor_env/bin/activate

# 2. Install Python dependencies
pip install websockets

# 4. Give your user permission to access the serial port
sudo usermod -a -G dialout $USER
# then log out and back in (or reboot) for the group change to apply

# 5. Plug in the sensor and confirm the device shows up
ls -l /dev/ttyACM0
```

If your sensor enumerates as a different port (e.g. `/dev/ttyUSB0` or
`COM5` on Windows), update `PORT` in `gv7_live_logger.py` accordingly.

---

## 6. Install MSCL (the library your script actually depends on)

**Important:** MicroStrain archived the MSCL GitHub repository —
the last release is v68.1.0 and no further releases will be made.
MSCL still works fine for the GV7-AR today, but for new projects
MicroStrain now points people toward their newer **MIP SDK**
(lighter-weight C/C++ library) instead. This guide sticks with MSCL
since your existing script (`import python_mscl` / `mscl`) is built
against it.

MSCL is **not** a pip package — it ships as prebuilt `.deb` packages
with the Python bindings included. Here's the real install path:

### 6a. Find the right package for your machine

Go to the releases page and pick the asset matching your CPU
architecture:

**https://github.com/LORD-MicroStrain/MSCL/releases**

| Your machine | Package to download |
|---|---|
| Raspberry Pi 4/5 (64-bit Raspberry Pi OS — most common today) | `MSCL-Cpp-Shared-aarch64-v68.1.0.deb` |
| Raspberry Pi running 32-bit OS | `MSCL-Cpp-Shared-armv8l-v68.1.0.deb` |
| x86_64 Linux PC/laptop | `MSCL-Cpp-Shared-x86_64-v68.1.0.deb` |

Check which one you have with:
```bash
uname -m
# aarch64  → 64-bit ARM  (use the aarch64 .deb)
# armv7l/armv8l → 32-bit ARM (use the armv8l .deb)
# x86_64   → 64-bit Intel/AMD (use the x86_64 .deb)
```

### 6b. Download and install it

```bash
cd gv7_project

# Replace the URL below with whichever .deb matches your architecture
wget https://github.com/LORD-MicroStrain/MSCL/releases/download/latest/MSCL-Cpp-Shared-aarch64-v68.1.0.deb

sudo dpkg -i MSCL-Cpp-Shared-aarch64-v68.1.0.deb

# If dpkg complains about missing dependencies, this pulls them in:
sudo apt install -f
```

This installs MSCL as a system package — it is **not** installed
inside your `sensor_env` virtual environment, because it's a compiled
`.deb`, not a Python wheel. The Python bindings get placed under
`/usr/share/python3-mscl/` (or a similarly named path — check with
`dpkg -L <package-name> | grep mscl` if you're unsure).

### 6c. Make Python able to find it from inside your venv

Since `sensor_env` doesn't know about system packages by default, add
the MSCL path at the top of any script that imports it — this is
already effectively what your logger needs, so add this near the top
of `gv7_live_logger.py` **before** the `import mscl` line if it isn't
already resolving:

```python
import sys
sys.path.append('/usr/share/python3-mscl/')
import mscl
```

(Your current script does `from python_mscl import mscl` — if that
exact import doesn't resolve after installing the `.deb`, switch to
the `sys.path.append` + `import mscl` pattern above, since that's the
path MicroStrain's own installer actually uses.)

### 6d. Verify it worked

```bash
source sensor_env/bin/activate
python3 -c "import sys; sys.path.append('/usr/share/python3-mscl/'); import mscl; print(mscl.MSCL_VERSION)"
```

If that prints a version number instead of an `ImportError`, MSCL is
correctly installed and reachable.

---

## 7. Files in this setup

| File                    | Purpose                                              |
|-------------------------|-------------------------------------------------------|
| `gv7_live_logger.py`    | Connects to the GV7-AR, streams EKF data, logs to CSV, broadcasts live readings over WebSocket |
| `dashboard_gv7.html`    | Browser dashboard — connects to the Pi's WebSocket server and shows live gauges + a rolling chart |

### What the logger does
- Streams orientation (roll/pitch/yaw) and angular rate (gx/gy/gz) at
  `SAMPLE_HZ` (default 100 Hz)
- Writes every sample to a timestamped CSV file, e.g.
  `gv7_log_20260710_143201.csv`
- Runs a WebSocket server on port `8765` so any browser on the same
  network can watch live readings
- Computes a smoothed gyro value and a "phase variable" — tune
  `PHASE_A`, `PHASE_B`, and `DC_OFFSET_PITCH` for your application
  (these came from your existing MPU9250 tuning; recalibrate them for
  the GV7-AR's actual pitch range and noise characteristics — don't
  assume the same constants transfer directly)
- Stops automatically after `RUN_DURATION` seconds (default 60s). Set
  `RUN_DURATION = None` in the script to log until you press Ctrl+C

---

## 8. Run it

**On the Raspberry Pi:**
```bash
source sensor_env/bin/activate
python3 gv7_live_logger.py
```

You should see:
```
[IMU] Connected: 3DM-GV7-AR  S/N: xxxxxxxx
[IMU] Streaming at 100Hz — EKF enabled
[IMU] Logging to: gv7_log_20260710_143201.csv
[WS]  WebSocket server started on port 8765
      Open dashboard_gv7.html and connect to this Pi's IP
      Find Pi IP with: hostname -I
```

**Find the Pi's IP address** (run this in a separate terminal on the Pi):
```bash
hostname -I
```

**On your laptop:**
1. Open `dashboard_gv7.html` in any browser (double-click it, or serve
   it however you like — it's a static file).
2. Enter the Pi's IP address in the input box.
3. Click **Connect**.
4. You'll see live roll/pitch/yaw/gyro readings updating, plus a
   rolling chart of roll/pitch/yaw.

**Stop logging:** press `Ctrl+C` on the Pi terminal, or let it run
until `RUN_DURATION` elapses. The CSV is flushed and closed
automatically either way, and the terminal prints how many rows were
saved.

---

## 9. Troubleshooting

| Symptom | Likely cause |
|---|---|
| `[IMU] FATAL: ...` on startup | Wrong `PORT`, sensor not powered, or another program (SensorConnect) already holds the serial port |
| Dashboard shows "disconnected" | Wrong IP entered, Pi and laptop not on the same network, or firewall blocking port 8765 |
| Values all read 0 | Channels not enabled/configured — reconnect in SensorConnect and check the Sampling tile |
| CSV has gaps or looks noisy at first | Gyro bias not captured yet, or sensor still settling right after power-on — wait a few seconds before trusting readings |
| `getDataPackets` error | Check your MSCL Python binding version matches this script — the timeout argument must be passed positionally, not as a keyword |
| `ModuleNotFoundError: No module named 'mscl'` (or `python_mscl`) | MSCL isn't installed, or your venv can't see it — revisit Section 6, and make sure the `sys.path.append('/usr/share/python3-mscl/')` line runs before `import mscl` |
| `dpkg -i` fails with dependency errors | Run `sudo apt install -f` right after, which pulls in whatever MSCL's `.deb` needs |

---

## 10. Reference links

- SensorConnect: https://microstrain.com/software/sensorconnect
- MSCL (Python/C++/MATLAB library, archived — v68.1.0 is final): https://github.com/LORD-MicroStrain/MSCL/releases
- MIP SDK (lower-level C/C++, recommended for production use going
  forward since MSCL may eventually be phased out for inertial
  products): https://github.com/LORD-MicroStrain/mip_sdk
- GV7 User Manual: https://s3.amazonaws.com/files.microstrain.com/GV7_User_Manual/user_manual_content/
