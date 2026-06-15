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
 Enable SSH  ← Must turn this ON
```

Click **Save → Yes → Yes**.

>  Flashing takes about 5–10 minutes.

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

Done! Your Raspberry Pi 5 is ready to use. 

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
   ssh pi@raspberrypi.local 
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

<img width="1900" height="900" alt="image" src="https://github.com/user-attachments/assets/c3178f22-eb6b-4f78-b594-0fc0bf74391c" />



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

> `68` appearing = MPU6250 successfully detected ,
>  I2C Address Of MPU6250 = 0x68 (hexadecimal)
> Nothing showing = recheck your wiring

---
