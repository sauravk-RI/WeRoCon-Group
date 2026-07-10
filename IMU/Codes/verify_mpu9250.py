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
