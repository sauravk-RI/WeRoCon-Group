Thigh IMU Phase Variable Estimation (MicroStrain 3DM-GV7-AR)
What is a phase variable?
A normalized 0-to-1 representation of the gait cycle, where 0 = start of stride, 1 = end of stride.
Repeats once per gait cycle regardless of walking speed:
Faster walking -> same 0-to-1 progression in less time -> steeper slope vs. time
Slower walking -> shallower slope
Used instead of elapsed time because gait cadence is not constant, and a prosthetic leg needs a cadence-independent way to know where it is in the cycle.
Where it comes from
Derived from two signals: thigh angle and its angular velocity.
Plotting these against each other over one gait cycle traces a closed, roughly elliptical loop — the phase portrait.
The angular position around this loop (via atan2) progresses smoothly from 0 to 1 per cycle — this angular position is the phase variable.

Three requirements for this to work correctly:

Normalize both axes to comparable scales — otherwise the loop is a stretched ellipse, distorting phase spacing within a cycle.
Center the signal on the origin — an offset mean shifts the loop off-origin, causing the same kind of distortion.
Exclude points near the origin — atan2 is unstable near (0,0); since both signals approach zero when stationary, any stationary period produces spurious rapid phase cycling unless excluded.
Getting thigh angle from the IMU
Sensor reports orientation as a quaternion (estOrientQuaternion), not a direct joint angle.
Process:
Record a reference quaternion during neutral standing (calibration).
At each sample, compute rotation relative to that reference.
Decompose into roll/pitch/yaw Euler angles.
Relative rotation (not raw orientation) is used because the sensor's raw output is relative to gravity/north, not the wearer's stance — this removes dependency on mounting angle.
Calibration requires genuine stillness in neutral stance for the full capture window; otherwise the reference is offset and every angle downstream carries a fixed, non-physical error.
This offset isn't obvious from the angles alone — verify by decoding the reference quaternion to Euler angles (should be a plausible mounted orientation) or comparing repeated calibrations via quaternion dot product (should agree closely).
Reference quaternion is computed as an average over a still window, not a single sample, to reduce sensitivity to noise.
python
"""
Calibrate reference quaternion for the MicroStrain 3DM-GV7-AR thigh IMU.
"""

import argparse
import json
import math
import time

from python_mscl import mscl

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
RAD_TO_DEG = 180.0 / math.pi

CALIBRATION_STILL_STD_DEG = 0.5
CALIBRATION_TIMEOUT_S = 20.0


def _vector_to_wxyz(vec):
    if hasattr(vec, "as_floatAt"):
        return (vec.as_floatAt(0), vec.as_floatAt(1), vec.as_floatAt(2), vec.as_floatAt(3))
    if hasattr(vec, "as_doubleAt"):
        return (vec.as_doubleAt(0), vec.as_doubleAt(1), vec.as_doubleAt(2), vec.as_doubleAt(3))
    if hasattr(vec, "data"):
        d = vec.data()
        return (d[0], d[1], d[2], d[3])
    return (vec[0], vec[1], vec[2], vec[3])


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


def quat_to_euler_deg(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def quat_normalize(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (w / n, x / n, y / n, z / n)


def quat_average(quats):
    """Simple averaging + renormalize."""
    w = sum(q[0] for q in quats) / len(quats)
    x = sum(q[1] for q in quats) / len(quats)
    y = sum(q[2] for q in quats) / len(quats)
    z = sum(q[3] for q in quats) / len(quats)
    return quat_normalize((w, x, y, z))


def read_estimation_filter(node):
    packets = node.getDataPackets(500)
    qw = qx = qy = qz = None

    QUAT_VECTOR_NAMES = ("estOrientQuaternion", "estAttitudeQuaternion", "estQuaternion")

    for packet in packets:
        for point in packet.data():
            name = point.channelName()
            if name in QUAT_VECTOR_NAMES:
                try:
                    q = point.as_Vector()
                    qw, qx, qy, qz = _vector_to_wxyz(q)
                except (AttributeError, TypeError):
                    pass

    if None not in (qw, qx, qy, qz):
        return (qw, qx, qy, qz)
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="calibration.json", help="Output JSON path")
    parser.add_argument("--seconds", type=float, default=3.0, help="Still-window duration (default 3s)")
    args = parser.parse_args()

    connection = mscl.Connection.Serial(SERIAL_PORT, BAUD_RATE)
    node = mscl.InertialNode(connection)
    node.resume()

    print("Warming up (let the filter settle)...")
    time.sleep(2.0)

    print(f"\nStand neutral and hold still. Capturing for {args.seconds:.0f}s once stable...")

    buffer = []
    start = time.time()
    max_len = max(int(args.seconds / 0.02), 10)

    while True:
        elapsed = time.time() - start
        if elapsed > CALIBRATION_TIMEOUT_S:
            print("\nTimed out waiting for stillness. Try again - check sensor connection.")
            return

        q = read_estimation_filter(node)
        if q is None:
            continue

        buffer.append(q)
        if len(buffer) > max_len:
            buffer.pop(0)

        roll, pitch, yaw = quat_to_euler_deg(q)
        print(f"\r  t={elapsed:5.1f}s  roll={roll:7.2f}  pitch={pitch:7.2f}  yaw={yaw:7.2f}   (window: {len(buffer)}/{max_len})", end="")

        if len(buffer) >= max_len:
            q_last = buffer[-1]
            max_dev_deg = 0.0
            for q_i in buffer:
                q_rel = quat_mul(quat_conj(q_last), q_i)
                w = max(-1.0, min(1.0, q_rel[0]))
                dev = 2 * math.acos(abs(w)) * RAD_TO_DEG
                max_dev_deg = max(max_dev_deg, dev)

            if max_dev_deg <= CALIBRATION_STILL_STD_DEG * 3:
                q_ref = quat_average(buffer)
                print(f"\n\nCalibration complete.")
                print(f"  Max deviation in still window: {max_dev_deg:.3f} deg")
                r, p, y = quat_to_euler_deg(q_ref)
                print(f"  World-frame roll={r:.2f} pitch={p:.2f} yaw={y:.2f}")

                out = {
                    "reference_quaternion": {
                        "w": q_ref[0], "x": q_ref[1], "y": q_ref[2], "z": q_ref[3]
                    },
                    "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "still_window_max_dev_deg": max_dev_deg,
                    "num_samples_averaged": len(buffer),
                }
                with open(args.out, "w") as f:
                    json.dump(out, f, indent=2)
                print(f"\nSaved to {args.out}")
                return

        time.sleep(0.02)


if __name__ == "__main__":
    main()
Identifying the thigh-flexion axis
Only one of roll/pitch/yaw matches thigh flexion/extension, depending on physical mounting — must be confirmed against real walking data, not assumed.
Confirmation method: record all three during walking, identify which shows a clean gait-rate oscillation in the expected range (~-10° to +30° here).
A wrong-axis selection can look identical to a calibration fault — check calibration first.
Angular velocity
