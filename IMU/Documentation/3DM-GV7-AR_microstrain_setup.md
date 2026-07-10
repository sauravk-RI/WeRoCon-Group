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
# 1. Create a virtual environment
python3 -m venv sensor_env
source sensor_env/bin/activate

# 2. Install Python dependencies
pip install websockets

# 3. Install MSCL Python bindings
#    Get the correct package for your Pi's architecture (armhf/aarch64)
#    and Python version from:
#    https://github.com/LORD-MicroStrain/MSCL/releases
#    (Follow MicroStrain's install instructions for that release —
#    it installs a system package that the "python_mscl" import
#    in the script relies on.)

# 4. Give your user permission to access the serial port
sudo usermod -a -G dialout $USER
# then log out and back in (or reboot) for the group change to apply

# 5. Plug in the sensor and confirm the device shows up
ls -l /dev/ttyACM0
```

If your sensor enumerates as a different port (e.g. `/dev/ttyUSB0` or
`COM5` on Windows), update `PORT` in `gv7_live_logger.py` accordingly.

---

## 6. Files in this setup

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

## 7. Run it

**Find the Pi's IP address** (note it down, used to connect dashboard to terminal):
```bash
hostname -I
```

**On the Raspberry Pi:**
```bash
source sensor_env/bin/activate
nano gv7_live_logger.py
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

## 8. Troubleshooting

| Symptom | Likely cause |
|---|---|
| `[IMU] FATAL: ...` on startup | Wrong `PORT`, sensor not powered, or another program (SensorConnect) already holds the serial port |
| Dashboard shows "disconnected" | Wrong IP entered, Pi and laptop not on the same network, or firewall blocking port 8765 |
| Values all read 0 | Channels not enabled/configured — reconnect in SensorConnect and check the Sampling tile |
| CSV has gaps or looks noisy at first | Gyro bias not captured yet, or sensor still settling right after power-on — wait a few seconds before trusting readings |
| `getDataPackets` error | Check your MSCL Python binding version matches this script — the timeout argument must be passed positionally, not as a keyword |

---

## 9. Reference links

- SensorConnect: https://microstrain.com/software/sensorconnect
- MSCL (Python/C++/MATLAB library): https://github.com/LORD-MicroStrain/MSCL
- MIP SDK (lower-level C/C++, recommended for production use going
  forward since MSCL may eventually be phased out for inertial
  products): https://github.com/LORD-MicroStrain/mip_sdk
- GV7 User Manual: https://s3.amazonaws.com/files.microstrain.com/GV7_User_Manual/user_manual_content/
