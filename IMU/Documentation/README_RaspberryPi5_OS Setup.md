# Raspberry Pi 5 — OS Setup Guide

> Simple step-by-step guide to flash Raspberry Pi OS and connect via SSH.

---

## What You Need

| Item | Details |
|---|---|
| Raspberry Pi 5 | 4GB or 8GB |
| microSD Card | Minimum 16GB, Class 10 |
| microSD Card Reader | USB adapter |
| Power Supply | USB-C 5V 5A (official Pi 5 adapter) |
| Laptop/PC | Windows / macOS / Linux |
| WiFi | 2.4GHz or 5GHz network |

---

## Step 1 — Download Raspberry Pi Imager

Go to this link on your laptop and download the imager:

```
https://www.raspberrypi.com/software/
```

Install and open it.

---

## Step 2 — Insert microSD Card

Plug your microSD card into the card reader, then plug it into your laptop.

---

## Step 3 — Select Device, OS & Storage

In Raspberry Pi Imager:

| Field | What to Choose |
|---|---|
| **Choose Device** | Raspberry Pi 5 |
| **Choose OS** | Raspberry Pi OS (other) → **Raspberry Pi OS Lite (64-bit)** |
| **Choose Storage** | Your microSD card |

Click **Next**.

---

## Step 4 — Edit Settings (IMPORTANT)

When asked to apply custom settings → click **Edit Settings**.

**General Tab:**
```
Hostname:       raspberrypi
Username:       pi
Password:       [choose a password]
WiFi SSID:      [your WiFi name]
WiFi Password:  [your WiFi password]
```

**Services Tab:**
```
✅ Enable SSH  ← Must turn this ON
```

Click **Save → Yes → Yes**.

> ⏳ Flashing takes about 5–10 minutes.

---

## Step 5 — Boot the Pi

1. Eject the microSD card from your laptop
2. Insert it into the **bottom slot** of the Raspberry Pi 5
3. Plug in the power cable
4. Wait **60 seconds**

---

## Step 6 — Connect via SSH

Open Terminal (Mac/Linux) or Command Prompt (Windows):

```bash
ssh pi@raspberrypi.local
```

Type your password when asked. You should see:

```
pi@raspberrypi:~ $
```

> If it doesn't connect, find your Pi's IP from your WiFi router and try:
> ```bash
> ssh pi@192.168.x.x
> ```

---

## Step 7 — Update the System

Once connected, run:

```bash
sudo apt update && sudo apt upgrade -y
```

Done! Your Raspberry Pi 5 is ready to use. ✅

---

## Quick Summary

```
Download Raspberry Pi Imager on laptop
            │
   Insert microSD into laptop
            │
  Imager → Pi 5 → OS Lite 64-bit → microSD
            │
  Edit Settings → WiFi + SSH + Username
            │
     Click Write → Wait 10 mins
            │
  Insert microSD into Pi 5 → Power ON
            │
   ssh pi@raspberrypi.local ✅
```

---

# MPU6250 — Wiring & Setup Guide

> Connect and read accelerometer + gyroscope data from the MPU6250 using I2C on Raspberry Pi 5.

---

## What You Need (Extra)

| Item | Details |
|---|---|
| MPU6250 Module | 6-axis IMU (Accelerometer + Gyroscope) |
| Jumper Wires | 4x Female-to-Female |
| Breadboard | Optional, for easy wiring |

---

## Step 8 — Enable I2C on Raspberry Pi 5

The MPU6250 communicates via **I2C**, which is disabled by default. Enable it:

```bash
sudo raspi-config
```

Navigate using arrow keys:

```
Interface Options → I2C → Yes → OK → Finish
```

Then reboot:

```bash
sudo reboot
```

SSH back in after reboot:

```bash
ssh pi@raspberrypi.local
```

Verify I2C is active:

```bash
ls /dev/i2c*
# Should show: /dev/i2c-1
```

---

## Step 9 — Wire the MPU6250 to Raspberry Pi 5

Connect using **4 jumper wires** only:

| MPU6250 Pin | Raspberry Pi 5 Pin | Description |
|---|---|---|
| **VCC** | Pin 1 — 3.3V | Power |
| **GND** | Pin 6 — GND | Ground |
| **SDA** | Pin 3 — GPIO 2 | I2C Data |
| **SCL** | Pin 5 — GPIO 3 | I2C Clock |

> ⚠️ Always use **3.3V** for VCC — never 5V. The MPU6250 is a 3.3V sensor and 5V will damage it.

### Pi 5 GPIO Diagram (relevant pins)

```
[Pin 1]  3.3V  ← VCC
[Pin 2]  5V
[Pin 3]  GPIO2 (SDA) ← SDA
[Pin 4]  5V
[Pin 5]  GPIO3 (SCL) ← SCL
[Pin 6]  GND   ← GND
```

### AD0 Pin — I2C Address

| AD0 | I2C Address |
|---|---|
| GND (or unconnected) | `0x68` ← default |
| 3.3V | `0x69` |

Leave AD0 unconnected to use the default address `0x68`.

---

## Step 10 — Install Required Libraries

### Install I2C Tools

```bash
sudo apt install -y i2c-tools python3-smbus
```

### Create a Virtual Environment

```bash
python3 -m venv .venv
source .venv/bin/activate
```

> You will see `(.venv)` at the start of your terminal — this means it is active.

### Install Python Libraries

```bash
pip install smbus2
pip install mpu6050-raspberrypi
```

### Library Overview

| Library | Purpose |
|---|---|
| `smbus2` | Communicate with I2C devices from Python |
| `mpu6050-raspberrypi` | Ready-made driver to read MPU6250/6050 data |
| `i2c-tools` | Scan and detect I2C devices on the bus |
| `python3-smbus` | System-level I2C support |

---

## Step 11 — Detect MPU6250 on I2C Bus

Run a scan to confirm the sensor is wired correctly and detected:

```bash
sudo i2cdetect -y 1
```

Expected output:

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

> `68` appearing = MPU6250 successfully detected ✅
> Nothing showing = recheck your wiring

---

## Step 12 — Read IMU Data

Create a Python script:

```bash
nano read_mpu.py
```

Paste this code:

```python
import smbus2
import time

MPU6250_ADDR = 0x68   # Change to 0x69 if AD0 is connected to 3.3V
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43
TEMP_OUT_H   = 0x41

bus = smbus2.SMBus(1)

# Wake up the MPU6250
bus.write_byte_data(MPU6250_ADDR, PWR_MGMT_1, 0x00)
time.sleep(0.1)

def read_word(reg):
    high  = bus.read_byte_data(MPU6250_ADDR, reg)
    low   = bus.read_byte_data(MPU6250_ADDR, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 65536
    return value

def get_accel():
    ax = read_word(ACCEL_XOUT_H)     / 16384.0
    ay = read_word(ACCEL_XOUT_H + 2) / 16384.0
    az = read_word(ACCEL_XOUT_H + 4) / 16384.0
    return ax, ay, az

def get_gyro():
    gx = read_word(GYRO_XOUT_H)     / 131.0
    gy = read_word(GYRO_XOUT_H + 2) / 131.0
    gz = read_word(GYRO_XOUT_H + 4) / 131.0
    return gx, gy, gz

def get_temp():
    raw = read_word(TEMP_OUT_H)
    return (raw / 340.0) + 36.53

print("MPU6250 Live Data — Press Ctrl+C to stop\n")

try:
    while True:
        ax, ay, az = get_accel()
        gx, gy, gz = get_gyro()
        temp       = get_temp()

        print(f"Accel (g)    X:{ax:+.3f}  Y:{ay:+.3f}  Z:{az:+.3f}")
        print(f"Gyro (°/s)   X:{gx:+.3f}  Y:{gy:+.3f}  Z:{gz:+.3f}")
        print(f"Temp (°C)    {temp:.2f}")
        print("-" * 45)
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopped.")
```

Save with `Ctrl+X → Y → Enter`.

Run it:

```bash
python3 read_mpu.py
```

### Expected Output

```
Accel (g)    X:-0.012  Y:+0.004  Z:+1.001
Gyro (°/s)   X:+0.213  Y:-0.152  Z:+0.031
Temp (°C)    28.54
---------------------------------------------
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `i2cdetect` shows nothing | Recheck wiring — SDA→Pin3, SCL→Pin5, VCC→3.3V, GND→GND |
| Permission denied on I2C | Run `sudo usermod -aG i2c pi` then log out and back in |
| `ModuleNotFoundError: smbus2` | Run `source .venv/bin/activate` then `pip install smbus2` |
| All readings are 0 | Reinstall: `pip uninstall smbus2 && pip install smbus2` |
| Address shows `69` not `68` | Change `MPU6250_ADDR = 0x69` in the script |

---

## Full Flow Summary

```
Enable I2C → sudo raspi-config
            │
  Wire MPU6250:
  VCC→Pin1(3.3V) | GND→Pin6 | SDA→Pin3 | SCL→Pin5
            │
  sudo i2cdetect -y 1 → confirm "68"
            │
  source .venv/bin/activate
  pip install smbus2 mpu6050-raspberrypi
            │
  python3 read_mpu.py  ✅  Live IMU readings!
```
