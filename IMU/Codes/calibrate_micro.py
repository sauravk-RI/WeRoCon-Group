"""
Standalone calibration - run this once, save the reference quaternion to a
file. imu_logger_csv_v2.py can then load it instead of re-calibrating every
single run.

Requires:
    pip install python-mscl --break-system-packages

Usage:
    python3 calibrate_imu.py
    python3 calibrate_imu.py --out my_calibration.json

Stand neutral and hold still when prompted. Re-run this any time you want
a fresh zero (new session, sensor remounted, etc).
"""

import argparse
import json
import math
import os
import time

from python_mscl import mscl

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

RAD_TO_DEG = 180.0 / math.pi

CALIBRATION_SECONDS = 3.0
CALIBRATION_STILL_STD_DEG = 0.5
CALIBRATION_TIMEOUT_S = 15.0

DEFAULT_CALIBRATION_FILE = "imu_side_mounted_calibration.json"


def _vector_to_wxyz(vec):
    if hasattr(vec, "as_floatAt"):
        return (vec.as_floatAt(0), vec.as_floatAt(1), vec.as_floatAt(2), vec.as_floatAt(3))
    if hasattr(vec, "as_doubleAt"):
        return (vec.as_doubleAt(0), vec.as_doubleAt(1), vec.as_doubleAt(2), vec.as_doubleAt(3))
    if hasattr(vec, "data"):
        d = vec.data()
        return (d[0], d[1], d[2], d[3])
    return (vec[0], vec[1], vec[2], vec[3])


def setup_imu():
    connection = mscl.Connection.Serial(SERIAL_PORT, BAUD_RATE)
    node = mscl.InertialNode(connection)
    node.resume()
    return node


def quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def quat_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def read_quat(node):
    packets = node.getDataPackets(500)
    qw = qx = qy = qz = None

    QUAT_VECTOR_NAMES = ("estAttitudeQuaternion", "estOrientQuaternion", "estQuaternion")
    QUAT_SCALAR_NAMES = {
        "w": ("estQuaternionW", "estAttitudeQuaternionW"),
        "x": ("estQuaternionX", "estAttitudeQuaternionX"),
        "y": ("estQuaternionY", "estAttitudeQuaternionY"),
        "z": ("estQuaternionZ", "estAttitudeQuaternionZ"),
    }

    for packet in packets:
        for point in packet.data():
            name = point.channelName()
            if name in QUAT_VECTOR_NAMES:
                try:
                    q = point.as_Vector()
                    qw, qx, qy, qz = _vector_to_wxyz(q)
                except (AttributeError, TypeError):
                    pass
            elif name in QUAT_SCALAR_NAMES["w"]:
                qw = point.as_float()
            elif name in QUAT_SCALAR_NAMES["x"]:
                qx = point.as_float()
            elif name in QUAT_SCALAR_NAMES["y"]:
                qy = point.as_float()
            elif name in QUAT_SCALAR_NAMES["z"]:
                qz = point.as_float()

    if None not in (qw, qx, qy, qz):
        return (qw, qx, qy, qz)
    return None


def calibrate_reference_quat(node, duration_s):
    print(f"Calibrating: stand neutral and hold still for {duration_s:.0f}s...")
    buffer = []
    start = time.time()

    while True:
        elapsed = time.time() - start
        if elapsed > CALIBRATION_TIMEOUT_S:
            if buffer:
                print(
                    f"Warning: never found a still window within {CALIBRATION_TIMEOUT_S:.0f}s. "
                    f"Using last reading as best-effort reference."
                )
                return buffer[-1]
            print("Warning: no samples received during calibration, using identity reference.")
            return (1.0, 0.0, 0.0, 0.0)

        quat = read_quat(node)
        if quat is None:
            continue

        buffer.append(quat)

        max_len = max(int(duration_s / 0.005), 10)
        if len(buffer) > max_len:
            buffer.pop(0)

        if len(buffer) >= max_len:
            q_last = buffer[-1]
            max_dev_deg = 0.0
            for q in buffer:
                q_rel = quat_mul(quat_conj(q_last), q)
                w = max(-1.0, min(1.0, q_rel[0]))
                dev = 2 * math.acos(abs(w)) * RAD_TO_DEG
                max_dev_deg = max(max_dev_deg, dev)

            if max_dev_deg <= CALIBRATION_STILL_STD_DEG * 3:
                print(
                    f"Calibration complete. Reference quaternion = "
                    f"({q_last[0]:.4f}, {q_last[1]:.4f}, {q_last[2]:.4f}, {q_last[3]:.4f}) "
                    f"(max deviation in window: {max_dev_deg:.2f} deg)."
                )
                return q_last


def main():
    parser = argparse.ArgumentParser(description="Calibrate IMU and save reference quaternion to a file.")
    parser.add_argument("--out", default=DEFAULT_CALIBRATION_FILE, help=f"Output file (default: {DEFAULT_CALIBRATION_FILE})")
    args = parser.parse_args()

    node = setup_imu()
    ref_quat = calibrate_reference_quat(node, CALIBRATION_SECONDS)

    data = {
        "reference_quaternion": {"w": ref_quat[0], "x": ref_quat[1], "y": ref_quat[2], "z": ref_quat[3]},
        "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }

    with open(args.out, "w") as f:
        json.dump(data, f, indent=2)

    print(f"\nSaved calibration to: {os.path.abspath(args.out)}")
    print("Use this with: python3 imu_logger_csv_v2.py --calibration " + args.out)


if __name__ == "__main__":
    main()
