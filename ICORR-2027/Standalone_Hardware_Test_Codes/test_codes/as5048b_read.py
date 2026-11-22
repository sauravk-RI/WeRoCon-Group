#!/usr/bin/env python3

import time
from smbus2 import SMBus

I2C_BUS = 1
AS5048B_ADDR = 0x40
ANGLE_HIGH_REG = 0xFE
ANGLE_LOW_REG = 0xFF

AGC_REG = 0xFA
DIAG_REG = 0xFB


def read_angle(bus):
    high_byte = bus.read_byte_data(AS5048B_ADDR, ANGLE_HIGH_REG)
    low_byte = bus.read_byte_data(AS5048B_ADDR, ANGLE_LOW_REG)
    agc = bus.read_byte_data(AS5048B_ADDR, AGC_REG)
    diag = bus.read_byte_data(AS5048B_ADDR, DIAG_REG)

    raw_angle = (high_byte << 6) | (low_byte & 0x3F)
    angle_degrees = (raw_angle / 16384.0) * 360.0

    return raw_angle, angle_degrees, agc, diag


def main():
    with SMBus(I2C_BUS) as bus:
        print("Reading AS5048B angle. Press Ctrl+C to stop.\n")
        try:
            while True:
                raw_angle, angle_degrees, agc, diag = read_angle(bus)
                print(f"Raw: {raw_angle:5d}    Angle: {angle_degrees:6.2f} deg")
                print(f"AGC: {agc}  (0 = strong field, 255 = weak field)")
                print(f"Diagnostics byte: {bin(diag)}")
                time.sleep(0.02)  # 50 Hz update rate
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
