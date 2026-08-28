# Lesson 7: Using the AMS OSRAM AS5048B Magnetic Position Sensor Board with Raspberry Pi 5

@FirstAuthor: Pritam Ranjan Kalita, Project Assistant, WeRoCon Laboratory, August 2026. <br>
@Disclaimer: This tutorial was written and reviewed by the author. AI-assisted tools were used to support drafting, editing, and language refinement, with all technical content verified by the author.

[← Back to Contents](00_Contents.md)

---

# What is the AS5048B?

The **AS5048B** (Mouser part **AS5048B_TS_EK_AB**) is a contactless magnetic rotary position sensor from ams OSRAM. It measures the absolute angular position of a small diametrically magnetized magnet mounted above the chip, using the Hall-effect principle, and reports a **14-bit absolute angle value (0–16383, corresponding to 0°–360°)** over an **I2C** interface.

Because the magnet and the sensor never touch, this type of sensor is very popular in robotics for:

- Joint angle feedback on robot arms
- Steering angle sensing
- Wheel/motor shaft position feedback (as an alternative to optical encoders)
- Contactless knobs and dials

> 💡 **Note:** The AS5048 chip also comes in an SPI variant, the **AS5048A**. This lesson covers the **AS5048B**, which uses I2C — the interface most convenient for the Raspberry Pi.

---

# Board Overview

The breakout board exposes a small header with the following pins:

| Pin | Function |
|-----|----------|
| 5V | Supply voltage input (regulated internally to 3.3V) |
| 3.3V | Regulated 3.3V output/input |
| PWM | Optional PWM output proportional to angle (not used in this lesson) |
| SDA | I2C data line (shared with SPI CSn on the chip) |
| SCL | I2C clock line (shared with SPI SCK on the chip) |
| A1 | I2C address select bit 0 (shared with SPI MOSI on the chip) |
| A2 | I2C address select bit 1 (shared with SPI MISO on the chip) |
| GND | Ground |

> 💡 **Note:** The AS5048B accepts a supply of **3.0V–5.5V**. It is safe to power it directly from the Raspberry Pi 5's 3.3V pin, or from 5V if your specific breakout regulates it down internally. **Always confirm your board's silkscreen/documentation before connecting power**, since incorrectly wiring 5V into a 3.3V-only input can damage the sensor.

The default 7-bit I2C address is built from the A1 and A2 pins:

| A2 | A1 | 7-bit Address (hex) |
|----|----|----------------------|
| 0 | 0 | 0x40 |
| 0 | 1 | 0x41 |
| 1 | 0 | 0x42 |
| 1 | 1 | 0x43 |

Leaving A1 and A2 unconnected (or tied low) gives the default address **0x40**, which is what this lesson uses.

---

# Wiring the AS5048B to the Raspberry Pi 5

The Raspberry Pi 5's 40-pin GPIO header exposes a dedicated I2C bus (**I2C1**) on physical pins 3 (SDA) and 5 (SCL).

| AS5048B Pin | Raspberry Pi 5 Pin | Description |
|-------------|---------------------|--------------|
| 3.3V | Pin 1 (3.3V) | Power |
| GND | Pin 6 (GND) | Ground |
| SDA | Pin 3 (GPIO 2 / SDA1) | I2C data |
| SCL | Pin 5 (GPIO 3 / SCL1) | I2C clock |

> 💡 **Note:** GPIO pins 3 and 5 already have onboard pull-up resistors on the Raspberry Pi, so external pull-ups are usually not required for a single sensor on a short cable.

## The A1 / A2 Address Pins (Not Connected to the Raspberry Pi)

**A1 and A2 do not connect to the Raspberry Pi at all.** They are address-select pins local to the sensor board itself — you wire each one to the *same* board's own GND or 3.3V pin (or simply leave them unconnected, which behaves the same as tying them low). For a single sensor, leaving both unconnected — or tying both to that board's own GND — gives the default address **0x40**, which is what this lesson uses.

Refer back to **Lesson 1** for the full GPIO pinout diagram if you need to double check pin numbering.

---

# Enabling I2C on the Raspberry Pi 5

I2C is disabled by default on Raspberry Pi OS and must be enabled first.

## Step 1: Open the Configuration Tool

```bash
sudo raspi-config
```

Navigate to:

```text
3 Interface Options → I5 I2C → Enable
```

Confirm **Yes** when asked whether to enable the I2C interface, then select **Finish** and reboot if prompted.

## Step 2: Reboot (if required)

```bash
sudo reboot
```

## Step 3: Install I2C Tools

```bash
sudo apt update
sudo apt install -y i2c-tools python3-smbus2
```

- `i2c-tools` provides the `i2cdetect` and `i2cget` utilities used to scan and query the bus.
- `python3-smbus2` provides the Python library used later in this lesson to read the sensor over I2C.

---

# Detecting the Sensor on the I2C Bus

With the sensor wired up and the Raspberry Pi powered on, scan the I2C bus:

```bash
sudo i2cdetect -y 1
```

Example Output:

```text
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
```

Here,

- `1` in the command refers to **I2C bus 1**, the bus exposed on the 40-pin header of the Raspberry Pi 5.
- `40` appearing in the grid confirms that the sensor was detected at address **0x40**.

> ⚠️ **Warning:** If nothing shows up in the grid, double check the wiring (especially SDA/SCL not being swapped), confirm the sensor is powered, and re-check that I2C was actually enabled and the Pi was rebooted.

---

# Understanding the AS5048B Angle Registers

The AS5048B stores its measured angle as a 14-bit value split across two 8-bit registers:

| Register (hex) | Contents |
|-----------------|----------|
| 0xFE | Angle value — upper 8 bits |
| 0xFF | Angle value — lower 6 bits (bits 5:0) |
| 0xFA | AGC value (magnetic field strength indicator) |
| 0xFB | Diagnostic flags (e.g., magnet too weak/too strong) |
| 0xFC | Magnitude of the internal CORDIC vector |

The 14-bit raw angle is reconstructed as:

```text
raw_angle = (high_byte << 6) | (low_byte & 0x3F)
```

This gives a value from **0 to 16383**, which is then converted to degrees:

```text
angle_degrees = (raw_angle / 16384) * 360
```

---

# Reading the Angle with Python

Create a new script:

```bash
nano as5048b_read.py
```

Add the following code:

```python
#!/usr/bin/env python3

import time
from smbus2 import SMBus

I2C_BUS = 1
AS5048B_ADDR = 0x40

ANGLE_HIGH_REG = 0xFE
ANGLE_LOW_REG = 0xFF


def read_angle(bus):
    high_byte = bus.read_byte_data(AS5048B_ADDR, ANGLE_HIGH_REG)
    low_byte = bus.read_byte_data(AS5048B_ADDR, ANGLE_LOW_REG)

    raw_angle = (high_byte << 6) | (low_byte & 0x3F)
    angle_degrees = (raw_angle / 16384.0) * 360.0

    return raw_angle, angle_degrees


def main():
    with SMBus(I2C_BUS) as bus:
        print("Reading AS5048B angle. Press Ctrl+C to stop.\n")
        try:
            while True:
                raw_angle, angle_degrees = read_angle(bus)
                print(f"Raw: {raw_angle:5d}    Angle: {angle_degrees:6.2f} deg")
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
```

Save the file (**Ctrl+O**, **Enter**, **Ctrl+X** in `nano`), then run it:

```bash
python3 as5048b_read.py
```

Example Output:

```text
Reading AS5048B angle. Press Ctrl+C to stop.

Raw:  8192    Angle: 180.00 deg
Raw:  8210    Angle: 180.40 deg
Raw:  8256    Angle: 181.41 deg
```

Rotate the magnet above the sensor slowly and watch the angle value update in real time.

> 💡 **Tip:** If every reading returns `0` or the value never changes, check that the magnet is centered above the sensor die and within the recommended air gap specified in the AS5048B datasheet (typically around 0.5–3 mm depending on magnet grade).

---

# Reading the Diagnostics and Magnitude Registers (Optional)

The AGC and diagnostic registers are useful for verifying that the magnet is placed correctly.

```python
AGC_REG = 0xFA
DIAG_REG = 0xFB

agc = bus.read_byte_data(AS5048B_ADDR, AGC_REG)
diag = bus.read_byte_data(AS5048B_ADDR, DIAG_REG)

print(f"AGC: {agc}  (0 = strong field, 255 = weak field)")
print(f"Diagnostics byte: {bin(diag)}")
```

A very low or very high AGC value, or diagnostic flags indicating "magnet too weak" or "magnet too strong," usually means the magnet needs to be repositioned closer to or farther from the sensor.

---

# Connecting Two AS5048B Encoders to One Raspberry Pi 5

Many robotics projects need more than one angle sensor — for example, two joints of an arm, or left/right wheel feedback. Since the AS5048B communicates over I2C, and every device on an I2C bus must have a unique address, there are two common ways to connect more than one sensor: giving each sensor a different address using its **A1**/**A2** pins (Option A), or using an **I2C multiplexer** if the addresses cannot be changed (Option B).

## Option A: Different Addresses on the Same Bus (Recommended)

> ⚠️ **Important:** A1 and A2 are **not** connected to the Raspberry Pi. They are local address-select pins on each encoder board — each one is wired to that *same* encoder's own GND or 3.3V pin using a short jumper. Only SDA, SCL, 3.3V, and GND go back to the Raspberry Pi, and those four lines are shared by both encoders.

### Connections to the Raspberry Pi (shared by both encoders)

Both encoders connect to the **same** four Raspberry Pi pins:

| Signal | Raspberry Pi 5 Pin |
|--------|----------------------|
| 3.3V | Pin 1 (3.3V) |
| GND | Pin 6 (GND) |
| SDA | Pin 3 (GPIO 2 / SDA1) |
| SCL | Pin 5 (GPIO 3 / SCL1) |

### Address-Select Jumpers (local to each encoder board — not wired to the Pi)

| Encoder | A1 wired to | A2 wired to | Resulting Address |
|---------|-------------|-------------|----------------------|
| Encoder 1 | Encoder 1's own GND | Encoder 1's own GND | **0x40** |
| Encoder 2 | Encoder 2's own 3.3V | Encoder 2's own GND | **0x41** |

> 💡 **Note:** Refer back to the address table earlier in this lesson if you want a different combination (e.g., 0x42 or 0x43 for a third or fourth sensor).

## Reading Both Sensors in Python

Only the address argument changes between reads — the rest of the reading logic is identical to the single-sensor example:

```python
#!/usr/bin/env python3

import time
from smbus2 import SMBus

I2C_BUS = 1

ENCODER_1_ADDR = 0x40
ENCODER_2_ADDR = 0x41

ANGLE_HIGH_REG = 0xFE
ANGLE_LOW_REG = 0xFF


def read_angle(bus, address):
    high_byte = bus.read_byte_data(address, ANGLE_HIGH_REG)
    low_byte = bus.read_byte_data(address, ANGLE_LOW_REG)

    raw_angle = (high_byte << 6) | (low_byte & 0x3F)
    angle_degrees = (raw_angle / 16384.0) * 360.0

    return raw_angle, angle_degrees


def main():
    with SMBus(I2C_BUS) as bus:
        print("Reading two AS5048B encoders. Press Ctrl+C to stop.\n")
        try:
            while True:
                raw1, deg1 = read_angle(bus, ENCODER_1_ADDR)
                raw2, deg2 = read_angle(bus, ENCODER_2_ADDR)
                print(f"Encoder 1: {deg1:6.2f} deg    Encoder 2: {deg2:6.2f} deg")
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
```

Run `sudo i2cdetect -y 1` after wiring both sensors to confirm that **both 0x40 and 0x41** appear in the grid before running the script.

> 💡 **Note:** The A1/A2 pins only give four possible addresses (0x40–0x43), so this method works for up to four sensors on one bus.

## Option B: Same Address, Separate Channels via a Multiplexer

If your boards have A1/A2 hardwired and cannot be changed (so every board is stuck at the same address, e.g., **0x40**), or you need more than four sensors, use an **I2C multiplexer** such as the **TCA9548A**. It sits between the Raspberry Pi and the sensors, and lets multiple same-address devices share one physical bus by switching between eight isolated channels in software — only one channel is active at a time, so each connected sensor can be addressed as if it were alone on the bus.

### Wiring

The multiplexer connects to the Raspberry Pi's normal I2C pins, and each encoder connects to its own numbered channel on the multiplexer instead of directly to the Pi:

| Signal | Raspberry Pi 5 Pin | TCA9548A Pin |
|--------|----------------------|----------------|
| 3.3V | Pin 1 (3.3V) | VIN |
| GND | Pin 6 (GND) | GND |
| SDA | Pin 3 (GPIO 2 / SDA1) | SDA |
| SCL | Pin 5 (GPIO 3 / SCL1) | SCL |

Only **SDA and SCL** are per-channel and pass through the multiplexer — **3.3V and GND are not wired through the multiplexer's channel pins**. Instead, every encoder's 3.3V and GND connect directly to the shared 3.3V and GND rails (the same Pin 1 and Pin 6 the multiplexer itself uses):

| Encoder | TCA9548A Channel | SDA connects to | SCL connects to | 3.3V connects to | GND connects to |
|---------|---------------------|--------------------|--------------------|----------------------|----------------------|
| Encoder 1 | Channel 0 | SD0 | SC0 | Shared 3.3V rail | Shared GND rail |
| Encoder 2 | Channel 1 | SD1 | SC1 | Shared 3.3V rail | Shared GND rail |

Both encoders can keep their **A1/A2 pins tied the same way** (e.g., both at 0x40), since they are now on separate, isolated channels and never see each other's traffic.

> 💡 **Note:** The TCA9548A itself appears on the main I2C bus at its own address, typically **0x70** by default (adjustable with its own address pins if you need multiple multiplexers). This is the address your Raspberry Pi talks to when selecting a channel — it is separate from the AS5048B's address.

### Selecting a Channel and Reading a Sensor in Python

Before talking to a sensor, you first write a single byte to the multiplexer to activate the channel that sensor is connected to. Each bit in that byte corresponds to one channel (bit 0 = channel 0, bit 1 = channel 1, and so on):

```python
#!/usr/bin/env python3

import time
from smbus2 import SMBus

I2C_BUS = 1

MUX_ADDR = 0x70          # TCA9548A default address
ENCODER_ADDR = 0x40      # Same address on both encoders

CHANNEL_ENCODER_1 = 0
CHANNEL_ENCODER_2 = 1

ANGLE_HIGH_REG = 0xFE
ANGLE_LOW_REG = 0xFF


def select_channel(bus, channel):
    bus.write_byte(MUX_ADDR, 1 << channel)


def read_angle(bus):
    high_byte = bus.read_byte_data(ENCODER_ADDR, ANGLE_HIGH_REG)
    low_byte = bus.read_byte_data(ENCODER_ADDR, ANGLE_LOW_REG)

    raw_angle = (high_byte << 6) | (low_byte & 0x3F)
    angle_degrees = (raw_angle / 16384.0) * 360.0

    return raw_angle, angle_degrees


def main():
    with SMBus(I2C_BUS) as bus:
        print("Reading two AS5048B encoders via TCA9548A. Press Ctrl+C to stop.\n")
        try:
            while True:
                select_channel(bus, CHANNEL_ENCODER_1)
                _, deg1 = read_angle(bus)

                select_channel(bus, CHANNEL_ENCODER_2)
                _, deg2 = read_angle(bus)

                print(f"Encoder 1: {deg1:6.2f} deg    Encoder 2: {deg2:6.2f} deg")
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
```

> ⚠️ **Warning:** Only one channel should be active at a time when two devices share the same address. Always call `select_channel()` immediately before communicating with a sensor on that channel — otherwise you risk talking to the wrong device or getting a bus conflict if two active channels both hold a device at the same address.

Run `sudo i2cdetect -y 1` first to confirm the multiplexer itself appears at **0x70**. Then, with a channel selected (for example by running `i2cset -y 1 0x70 0x01` to select channel 0), run `i2cdetect` again — you should now see the encoder's address (e.g., 0x40) appear, since only that channel's device is currently visible on the bus.

---

# Setting a Custom Zero Position

By default, the AS5048B's zero position is fixed at the factory and usually will not line up with whatever position you want to treat as "zero" on your motor shaft (for example, a specific joint angle or a centered steering position). There are two ways to handle this: a **permanent hardware zero** burned into the sensor's memory, and a **software offset** applied in your own code. For most projects, the software offset is the simpler and safer choice.

## Option A: Software Zero Offset (Recommended)

This method never touches the sensor itself — it is fully reversible, requires no special registers, and works immediately.

1. Rotate the motor shaft to the exact position you want to call "zero."
2. Read the raw angle at that position and note it down (this is your **zero offset**).
3. In your script, subtract the offset from every subsequent raw reading, wrapping the result back into the 0–16383 range.

```python
#!/usr/bin/env python3

import time
from smbus2 import SMBus

I2C_BUS = 1
AS5048B_ADDR = 0x40

ANGLE_HIGH_REG = 0xFE
ANGLE_LOW_REG = 0xFF

RAW_MAX = 16384  # 14-bit resolution (0-16383)

# Raw angle reading captured while the shaft was at your desired zero position.
# Replace this with the value you measured for your own setup.
ZERO_OFFSET = 8192


def read_raw_angle(bus):
    high_byte = bus.read_byte_data(AS5048B_ADDR, ANGLE_HIGH_REG)
    low_byte = bus.read_byte_data(AS5048B_ADDR, ANGLE_LOW_REG)
    return (high_byte << 6) | (low_byte & 0x3F)


def read_angle_degrees(bus, offset=ZERO_OFFSET):
    raw_angle = read_raw_angle(bus)
    adjusted = (raw_angle - offset) % RAW_MAX
    return (adjusted / RAW_MAX) * 360.0


def main():
    with SMBus(I2C_BUS) as bus:
        print("Reading AS5048B angle relative to custom zero. Press Ctrl+C to stop.\n")
        try:
            while True:
                angle_degrees = read_angle_degrees(bus)
                print(f"Angle from zero: {angle_degrees:6.2f} deg")
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
```

To find `ZERO_OFFSET` for your own setup, run the single-sensor reading script from earlier in this lesson, rotate the shaft to your desired zero position, and note the **Raw** value printed at that moment — that raw value is your offset.

The `% RAW_MAX` (modulo) operation handles wraparound, so the angle correctly rolls over from 359° back to 0° instead of jumping to a negative number.

> 💡 **Note:** Because this offset lives in your script, it resets to nothing if you forget to reapply it in a new program. Keep `ZERO_OFFSET` as a named constant (or load it from a config file) so every script that reads this sensor stays consistent.

## Option B: Permanent Hardware Zero (OTP Programming)

The AS5048B also supports burning a zero position directly into the chip's **One-Time Programmable (OTP)** memory, using registers **0x16** (zero position high byte) and **0x17** (zero position low 6 bits) together with the programming control register (**0x03**).

> ⚠️ **Warning:** OTP programming is **permanent and can only be performed once per device**. There is no way to factory-reset it afterward. Only proceed if you are certain about the position, and consider testing thoroughly with the volatile (non-burned) write first.

The general procedure is:

1. Rotate the shaft to the desired zero position.
2. Read the current raw angle from 0xFE/0xFF.
3. Write that raw value into 0x16 and 0x17. This step is **volatile** — it takes effect immediately but resets on power-cycle, so you can safely verify it first.
4. Confirm the sensor now reports approximately 0° at that shaft position, and check for any wraparound issues.
5. Only if satisfied, permanently commit the value by setting the Program-Enable bit, then the Burn bit, in register 0x03, followed by the Verify bit to confirm the value was written correctly to OTP.

Given the irreversible nature of this method, most users are better served by the software offset in **Option A** unless the zero position must be guaranteed correct even when the sensor is read by different hardware or firmware that you do not control.

---

# Summary

The AS5048B is a simple and precise way to add contactless angle sensing to a Raspberry Pi 5 based robotics project. In this lesson you learned how the sensor connects to the GPIO I2C pins, how to enable and verify the I2C interface on Raspberry Pi OS, how to detect the sensor's address using `i2cdetect`, and how to read its 14-bit angle value in Python using `smbus2`. You also learned how to wire and read multiple encoders on the same I2C bus, either by giving each one a unique A1/A2 address or by using a TCA9548A multiplexer when addresses can't be changed, and how to set a custom zero position — either safely in software with an offset, or permanently on the chip using OTP programming. With this foundation, you can integrate the sensor into larger projects such as joint feedback for a robot arm or a contactless rotary control knob.

---

**Happy Learning!** 😊

[← Back to Contents](00_Contents.md)
