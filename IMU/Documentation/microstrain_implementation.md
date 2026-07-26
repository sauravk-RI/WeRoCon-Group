# MicroStrain 3DM-GV7-AR Implementation

## Getting thigh angle from the IMU

- Sensor reports orientation as a quaternion (`estOrientQuaternion`), not a direct joint angle.
- Process:
  1. Record a **reference quaternion** during neutral standing (calibration).
  2. At each sample, compute rotation **relative to** that reference.
  3. Decompose into roll/pitch/yaw Euler angles.
- Relative rotation (not raw orientation) is used because the sensor's raw output is relative to gravity/north, not the wearer's stance — this removes dependency on mounting angle.
- Calibration requires genuine stillness in neutral stance for the full capture window; otherwise the reference is offset and every angle downstream carries a fixed, non-physical error.
- This offset isn't obvious from the angles alone — verify by decoding the reference quaternion to Euler angles (should be a plausible mounted orientation) or comparing repeated calibrations via quaternion dot product (should agree closely).
- Reference quaternion is computed as an **average over a still window**, not a single sample, to reduce sensitivity to noise.

```python
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
```

## Identifying the thigh-flexion axis

- Only one of roll/pitch/yaw matches thigh flexion/extension, depending on physical mounting — must be confirmed against real walking data, not assumed.
- Confirmation method: record all three during walking, identify which shows a clean gait-rate oscillation in the expected range (~-10° to +30° here).
- A wrong-axis selection can look identical to a calibration fault — check calibration first.

## Angular velocity

- Two sources: sensor's own angular rate channel, or numerically derived (`d(pitch)/dt`), smoothed with an EMA.
- **Only the derived rate feeds into the phase variable.**
- Using both interchangeably without checking sign/magnitude agreement makes comparisons between them unreliable, though the phase variable itself is unaffected.

## Computing the phase variable

```python
def compute_phase_var(pitch_centered_deg, rate_deg_s):
    x = pitch_centered_deg / A_PITCH
    y = rate_deg_s / B_GYRO

    radius = math.sqrt(x * x + y * y)
    if radius < MIN_RADIUS:
        return None

    angle = math.atan2(y, x)
    phase = (angle + math.pi) / (2 * math.pi)
    return 1 - phase
```

- `A_PITCH`, `B_GYRO`: normalization constants, set from this sensor's actual walking-trial amplitude (see the checklist above).
- `MIN_RADIUS`: excludes near-origin points where `atan2` is unstable; phase holds its last valid value through such periods instead of being recomputed.

## Complete pipeline: CSV logger

```python
"""
Thigh IMU logger - direct to CSV, using the relative-quaternion pipeline.

Same math as imu_dashboard_ws.py (relative rotation from calibration pose,
not fixed-frame Euler - avoids gimbal lock regardless of mount angle), but
writes straight to CSV instead of streaming over WebSocket.

Requires:
    pip install python-mscl --break-system-packages

Sensor Connect setup:
    Enable the Estimation Filter "Attitude (Quaternion)" channel.

Usage:
    python3 imu_logger_csv_v2.py
    python3 imu_logger_csv_v2.py --out walking_trial_1.csv
    python3 imu_logger_csv_v2.py --seconds 15

Calibration:
    Stand neutral and hold still when prompted. Re-run the script for a
    fresh calibration each trial (no live re-zero here, unlike the
    dashboard version - this is meant for quick standalone logging runs).

Stop early any time with Ctrl+C - the file is flushed as you go.

Output columns:
    t,roll_deg,pitch_deg,yaw_deg,ang_vel_deg_s,phase_var
"""

import argparse
import csv
import json
import math
import os
import time

import matplotlib
matplotlib.use("Agg")  # no display over SSH - save PNG only, don't try to open a window
import matplotlib.pyplot as plt
from python_mscl import mscl

# ---- Configuration (mirrors imu_dashboard_ws.py) ----
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

RAD_TO_DEG = 180.0 / math.pi

PITCH_SIGN = -1.0

# Which axis is thigh flexion/extension. CONFIRMED 2026-07-07: swing axis is ~97% Y.
THIGH_AXIS = "y"

ANGULAR_RATE_CHANNEL = {
    "x": "estAngularRateX",
    "y": "estAngularRateY",
    "z": "estAngularRateZ",
}[THIGH_AXIS]

CALIBRATION_SECONDS = 3.0
CALIBRATION_STILL_STD_DEG = 0.5
CALIBRATION_TIMEOUT_S = 15.0

ALPHA_GYRO = 0.04

# Phase portrait normalization - matches known-good MPU9250 pipeline values.
A_PITCH = 23.7     # deg
B_GYRO = 150.8     # deg/s

MIN_RADIUS = 0.05

FIELDNAMES = ["t", "roll_deg", "pitch_deg", "yaw_deg", "sensor_ang_vel_deg_s", "derived_ang_vel_deg_s", "phase_var"]


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


def quat_relative_euler_deg(q_ref, q_now):
    """Full independent roll/pitch/yaw (deg) of q_now relative to q_ref."""
    q_rel = quat_mul(quat_conj(q_ref), q_now)
    w, x, y, z = q_rel

    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def read_estimation_filter(node):
    packets = node.getDataPackets(500)
    qw = qx = qy = qz = None
    rate = None

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
            elif name == ANGULAR_RATE_CHANNEL:
                rate = point.as_float()

    quat = None
    if None not in (qw, qx, qy, qz):
        quat = (qw, qx, qy, qz)

    return quat, rate


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

        quat, _ = read_estimation_filter(node)
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


def compute_phase_var(pitch_centered_deg, rate_deg_s):
    x = pitch_centered_deg / A_PITCH
    y = rate_deg_s / B_GYRO

    radius = math.sqrt(x * x + y * y)
    if radius < MIN_RADIUS:
        return None

    angle = math.atan2(y, x)
    phase = (angle + math.pi) / (2 * math.pi)
    return 1-phase


def load_calibration(path):
    """Loads a reference quaternion saved by calibrate_imu.py."""
    with open(path) as f:
        data = json.load(f)
    q = data["reference_quaternion"]
    print(f"Loaded calibration from {path} (calibrated at {data.get('calibrated_at', 'unknown')})")
    return (q["w"], q["x"], q["y"], q["z"])


def plot_csv(csv_path):
    """
    Reads the CSV just written and plots pitch, BOTH angular velocity
    sources (sensor vs. derived) overlaid for direct comparison, plus the
    phase portrait and phase variable.
    """
    t, pitch, sensor_rate, derived_rate, phase = [], [], [], [], []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row["t"]))
            pitch.append(float(row["pitch_deg"]))
            sensor_rate.append(float(row["sensor_ang_vel_deg_s"]) if row["sensor_ang_vel_deg_s"] else float("nan"))
            derived_rate.append(float(row["derived_ang_vel_deg_s"]) if row["derived_ang_vel_deg_s"] else float("nan"))
            phase.append(float(row["phase_var"]) if row["phase_var"] else float("nan"))

    t0 = t[0]
    t = [ti - t0 for ti in t]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(f"Thigh IMU trial - {os.path.basename(csv_path)}")

    axes[0, 0].plot(t, pitch, color="tab:blue")
    axes[0, 0].set_title("Pitch over time")
    axes[0, 0].set_xlabel("Time (s)")
    axes[0, 0].set_ylabel("Pitch (deg)")
    axes[0, 0].grid(True)

    axes[0, 1].plot(t, sensor_rate, color="tab:red", label="Sensor (estAngularRateY)", alpha=0.7)
    axes[0, 1].plot(t, derived_rate, color="tab:green", label="Derived (d(pitch)/dt, filtered)", alpha=0.7)
    axes[0, 1].set_title("Angular velocity: sensor vs. derived")
    axes[0, 1].set_xlabel("Time (s)")
    axes[0, 1].set_ylabel("Ang. vel (deg/s)")
    axes[0, 1].legend(fontsize=8)
    axes[0, 1].grid(True)

    axes[1, 0].plot(pitch, derived_rate, ".", markersize=2, color="tab:orange")
    axes[1, 0].set_title("Phase portrait (pitch vs. derived ang. vel.)")
    axes[1, 0].set_xlabel("Pitch (deg)")
    axes[1, 0].set_ylabel("Ang. vel (deg/s)")
    axes[1, 0].axis("equal")
    axes[1, 0].grid(True)

    axes[1, 1].plot(t, phase, "-", markersize=2, color="tab:cyan")
    axes[1, 1].set_title("Phase variable over time")
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].set_ylabel("Phase (0-1)")
    axes[1, 1].set_ylim(0, 1)
    axes[1, 1].grid(True)

    plt.tight_layout()

    png_path = os.path.splitext(csv_path)[0] + "_plots.png"
    plt.savefig(png_path, dpi=150)
    print(f"Saved plots to: {os.path.abspath(png_path)}")
    print("(No display available over SSH - copy this PNG to your laptop with scp to view it.)")


def default_filename():
    return f"walking_trial_{time.strftime('%Y%m%d_%H%M%S')}.csv"


def log_session(node, ref_quat, out_path, max_seconds=None, loop_period_s=0.01):
    """
    loop_period_s: target fixed time between samples (default 10ms = 100Hz).
    Uses a fixed-cadence loop (sleeps for whatever time remains after
    processing, rather than a flat sleep(0.005)) to test whether the
    sensor's own estAngularRateY channel becomes trustworthy under more
    uniform timing than the previous logger used.

    Logs BOTH the sensor's reported rate and our own pitch-derivative rate
    side by side, so they can be compared directly instead of guessing
    which one to trust.
    """
    abs_out_path = os.path.abspath(out_path)
    print("=" * 60)
    print(f"  SAVING TO: {abs_out_path}")
    print(f"  Target loop period: {loop_period_s*1000:.0f} ms")
    print("=" * 60)
    print("Press Ctrl+C to stop." + (f" (auto-stops after {max_seconds}s)" if max_seconds else ""))

    last_phase = None
    gy_filt = 0.0
    have_gy_filt = False
    prev_t = None
    prev_pitch = None
    row_count = 0
    start = time.time()

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(FIELDNAMES)

        try:
            next_tick = time.time()
            while True:
                if max_seconds is not None and (time.time() - start) > max_seconds:
                    break

                quat, sensor_rate_rad_s = read_estimation_filter(node)
                now = time.time()

                if quat is not None:
                    roll_raw, pitch_raw, yaw_raw = quat_relative_euler_deg(ref_quat, quat)
                    angle_by_axis = {"x": roll_raw, "y": pitch_raw, "z": yaw_raw}
                    pitch_deg = PITCH_SIGN * angle_by_axis[THIGH_AXIS]
                    roll_deg = angle_by_axis["x"] if THIGH_AXIS != "x" else angle_by_axis["y"]
                    yaw_deg = angle_by_axis["z"] if THIGH_AXIS != "z" else angle_by_axis["y"]

                    sensor_rate_deg_s = (
                        PITCH_SIGN * sensor_rate_rad_s * RAD_TO_DEG
                        if sensor_rate_rad_s is not None else ""
                    )

                    derived_rate_deg_s = ""
                    if prev_t is not None:
                        dt = now - prev_t
                        if dt > 0:
                            derived_rate_deg_s = (pitch_deg - prev_pitch) / dt
                            if not have_gy_filt:
                                gy_filt = derived_rate_deg_s
                                have_gy_filt = True
                            else:
                                gy_filt = ALPHA_GYRO * derived_rate_deg_s + (1 - ALPHA_GYRO) * gy_filt

                    prev_t = now
                    prev_pitch = pitch_deg

                    phase = ""
                    if derived_rate_deg_s != "":
                        pitch_centered = pitch_deg - (-2.3)   # i.e. pitch_deg + 2.3
                        p = compute_phase_var(pitch_centered, gy_filt)
                        if p is None:
                            p = last_phase
                        else:
                            last_phase = p
                        phase = p if p is not None else ""

                    writer.writerow([
                        time.time(), roll_deg, pitch_deg, yaw_deg,
                        sensor_rate_deg_s, derived_rate_deg_s, phase,
                    ])
                    row_count += 1
                    if row_count % 200 == 0:
                        f.flush()
                        print(f"  ...{row_count} samples logged", end="\r")

                next_tick += loop_period_s
                sleep_time = next_tick - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    # We're behind schedule - don't sleep, and resync next_tick
                    # so we don't try to "catch up" with a burst of fast loops.
                    next_tick = time.time()

        except KeyboardInterrupt:
            pass
        finally:
            f.flush()
            print(f"\nDone. {row_count} samples written to {abs_out_path}")


def main():
    parser = argparse.ArgumentParser(description="Log thigh IMU data (relative-quaternion pipeline) to CSV.")
    parser.add_argument("--out", default=None, help="Output CSV path (default: timestamped filename)")
    parser.add_argument("--seconds", type=float, default=10.0, help="Auto-stop after N seconds (default: 10s)")
    parser.add_argument("--calibration", default=None, help="Path to a saved calibration file from calibrate_imu.py (skips live calibration)")
    parser.add_argument("--no-plot", action="store_true", help="Skip plotting after logging finishes")
    args = parser.parse_args()

    out_path = args.out or default_filename()
    abs_out_path = os.path.abspath(out_path)

    if os.path.exists(out_path):
        answer = input(f"WARNING: {abs_out_path} already exists and will be overwritten. Continue? [y/N]: ")
        if answer.strip().lower() != "y":
            print("Aborted - no data was overwritten.")
            return

    node = setup_imu()

    if args.calibration:
        ref_quat = load_calibration(args.calibration)
    else:
        ref_quat = calibrate_reference_quat(node, CALIBRATION_SECONDS)

    STABILIZE_SECONDS = 3.0
    print(f"Stabilizing for {STABILIZE_SECONDS:.0f}s - start walking now...")
    t_stab = time.time()
    while time.time() - t_stab < STABILIZE_SECONDS:
        read_estimation_filter(node)

    log_session(node, ref_quat, out_path, max_seconds=args.seconds)

    if not args.no_plot:
        plot_csv(out_path)


if __name__ == "__main__":
    main()
```

## Complete pipeline: live WebSocket streamer

```python
"""
Thigh IMU real-time WebSocket streamer - relative-quaternion pipeline.

Same math as imu_phase_13july.py (relative rotation from calibration pose),
but streams JSON over a WebSocket instead of writing to CSV. No file
logging, no matplotlib plotting - live dashboard only.

Requires:
    pip install python-mscl websockets --break-system-packages

Sensor Connect setup:
    Enable the Estimation Filter "Attitude (Quaternion)" channel.

Usage:
    python3 imu_dashboard_ws.py
    python3 imu_dashboard_ws.py --calibration calibration.json

Then open imu_dashboard.html in a browser (on the same machine, or update
the WS_URL in the HTML file to point at the Pi's IP).
"""

import argparse
import asyncio
import json
import math
import time

import websockets
from python_mscl import mscl

# ---- Configuration (unchanged from imu_phase_13july.py) ----
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

RAD_TO_DEG = 180.0 / math.pi

PITCH_SIGN = -1.0

# Which axis is thigh flexion/extension. CONFIRMED 2026-07-07: swing axis is ~97% Y.
THIGH_AXIS = "y"

ANGULAR_RATE_CHANNEL = {
    "x": "estAngularRateX",
    "y": "estAngularRateY",
    "z": "estAngularRateZ",
}[THIGH_AXIS]

CALIBRATION_SECONDS = 3.0
CALIBRATION_STILL_STD_DEG = 0.5
CALIBRATION_TIMEOUT_S = 15.0

ALPHA_GYRO = 0.04

# Phase portrait normalization - matches known-good MPU9250 pipeline values.
A_PITCH = 23.7     # deg
B_GYRO = 150.8     # deg/s

MIN_RADIUS = 0.05

STABILIZE_SECONDS = 3.0

WS_HOST = "0.0.0.0"
WS_PORT = 8765


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


def quat_relative_euler_deg(q_ref, q_now):
    """Full independent roll/pitch/yaw (deg) of q_now relative to q_ref."""
    q_rel = quat_mul(quat_conj(q_ref), q_now)
    w, x, y, z = q_rel

    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def read_estimation_filter(node):
    packets = node.getDataPackets(500)
    qw = qx = qy = qz = None
    rate = None

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
            elif name == ANGULAR_RATE_CHANNEL:
                rate = point.as_float()

    quat = None
    if None not in (qw, qx, qy, qz):
        quat = (qw, qx, qy, qz)

    return quat, rate


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

        quat, _ = read_estimation_filter(node)
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


def compute_phase_var(pitch_centered_deg, rate_deg_s):
    x = pitch_centered_deg / A_PITCH
    y = rate_deg_s / B_GYRO

    radius = math.sqrt(x * x + y * y)
    if radius < MIN_RADIUS:
        return None

    angle = math.atan2(y, x)
    phase = (angle + math.pi) / (2 * math.pi)
    return 1 - phase


def load_calibration(path):
    """Loads a reference quaternion saved by calibrate_imu.py."""
    with open(path) as f:
        data = json.load(f)
    q = data["reference_quaternion"]
    print(f"Loaded calibration from {path} (calibrated at {data.get('calibrated_at', 'unknown')})")
    return (q["w"], q["x"], q["y"], q["z"])


class StreamState:
    """Holds the same running variables log_session used to keep locally."""
    def __init__(self):
        self.last_phase = None
        self.gy_filt = 0.0
        self.have_gy_filt = False
        self.prev_t = None
        self.prev_pitch = None


def compute_sample(node, ref_quat, state):
    """
    Runs the exact same per-sample computation as the loop body inside
    log_session() in imu_phase_13july.py, just returning a dict instead
    of writing a CSV row.
    """
    quat, sensor_rate_rad_s = read_estimation_filter(node)
    now = time.time()

    if quat is None:
        return None

    roll_raw, pitch_raw, yaw_raw = quat_relative_euler_deg(ref_quat, quat)
    angle_by_axis = {"x": roll_raw, "y": pitch_raw, "z": yaw_raw}
    pitch_deg = PITCH_SIGN * angle_by_axis[THIGH_AXIS]
    roll_deg = angle_by_axis["x"] if THIGH_AXIS != "x" else angle_by_axis["y"]
    yaw_deg = angle_by_axis["z"] if THIGH_AXIS != "z" else angle_by_axis["y"]

    sensor_rate_deg_s = (
        PITCH_SIGN * sensor_rate_rad_s * RAD_TO_DEG
        if sensor_rate_rad_s is not None else None
    )

    derived_rate_deg_s = None
    if state.prev_t is not None:
        dt = now - state.prev_t
        if dt > 0:
            derived_rate_deg_s = (pitch_deg - state.prev_pitch) / dt
            if not state.have_gy_filt:
                state.gy_filt = derived_rate_deg_s
                state.have_gy_filt = True
            else:
                state.gy_filt = ALPHA_GYRO * derived_rate_deg_s + (1 - ALPHA_GYRO) * state.gy_filt

    state.prev_t = now
    state.prev_pitch = pitch_deg

    phase = None
    if derived_rate_deg_s is not None:
        p = compute_phase_var(pitch_deg, state.gy_filt)
        if p is None:
            p = state.last_phase
        else:
            state.last_phase = p
        phase = p

    return {
        "t": now,
        "roll_deg": roll_deg,
        "pitch_deg": pitch_deg,
        "yaw_deg": yaw_deg,
        "sensor_ang_vel_deg_s": sensor_rate_deg_s,
        "derived_ang_vel_deg_s": derived_rate_deg_s,
        "phase_var": phase,
    }


async def _safe_send(client, message):
    try:
        await client.send(message)
    except Exception:
        pass  # client likely disconnected - cleanup handled in ws_handler


async def imu_broadcaster(node, ref_quat, connected_clients, loop_period_s=0.01):
    """
    Same fixed-cadence timing approach as log_session()'s while loop, but
    broadcasts each sample as JSON to all connected WebSocket clients
    instead of writing it to a CSV file.
    """
    state = StreamState()
    next_tick = time.time()

    while True:
        sample = compute_sample(node, ref_quat, state)

        if sample is not None and connected_clients:
            message = json.dumps(sample)
            for client in list(connected_clients):
                asyncio.create_task(_safe_send(client, message))

        next_tick += loop_period_s
        sleep_time = next_tick - time.time()
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)
        else:
            next_tick = time.time()


async def ws_handler(websocket, connected_clients):
    connected_clients.add(websocket)
    print(f"Client connected. Total clients: {len(connected_clients)}")
    try:
        async for _ in websocket:
            pass  # dashboard doesn't send anything back, just keep connection open
    finally:
        connected_clients.discard(websocket)
        print(f"Client disconnected. Total clients: {len(connected_clients)}")


async def main_async(args):
    node = setup_imu()

    if args.calibration:
        ref_quat = load_calibration(args.calibration)
    else:
        ref_quat = calibrate_reference_quat(node, CALIBRATION_SECONDS)

    print(f"Stabilizing for {STABILIZE_SECONDS:.0f}s - start walking now...")
    t_stab = time.time()
    while time.time() - t_stab < STABILIZE_SECONDS:
        read_estimation_filter(node)

    connected_clients = set()

    async with websockets.serve(
        lambda ws: ws_handler(ws, connected_clients), WS_HOST, WS_PORT
    ):
        print(f"WebSocket server running at ws://{WS_HOST}:{WS_PORT}")
        print("Open imu_dashboard.html in a browser to view the live plots.")
        await imu_broadcaster(node, ref_quat, connected_clients)


def main():
    parser = argparse.ArgumentParser(description="Stream thigh IMU data (relative-quaternion pipeline) over WebSocket.")
    parser.add_argument("--calibration", default=None, help="Path to a saved calibration file from calibrate_imu.py (skips live calibration)")
    args = parser.parse_args()

    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
```

## Live dashboard

```html
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Thigh IMU - Live Dashboard</title>
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.1/chart.umd.min.js"></script>
<style>
  :root {
    --bg: #0d1117;
    --panel: #161b22;
    --border: #30363d;
    --text: #c9d1d9;
    --muted: #8b949e;
    --pitch: #58a6ff;
    --phase: #d29922;
    --ok: #3fb950;
    --bad: #f85149;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0;
    background: var(--bg);
    color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    padding: 20px;
  }
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 16px;
    flex-wrap: wrap;
    gap: 10px;
  }
  h1 {
    font-size: 18px;
    font-weight: 600;
    margin: 0;
    letter-spacing: 0.2px;
  }
  .status {
    display: flex;
    align-items: center;
    gap: 8px;
    font-size: 13px;
    color: var(--muted);
  }
  .dot {
    width: 9px;
    height: 9px;
    border-radius: 50%;
    background: var(--bad);
    transition: background 0.2s;
  }
  .dot.connected { background: var(--ok); }
  .controls {
    display: flex;
    align-items: center;
    gap: 8px;
    font-size: 13px;
  }
  input[type="text"] {
    background: var(--panel);
    border: 1px solid var(--border);
    color: var(--text);
    padding: 5px 8px;
    border-radius: 6px;
    font-size: 13px;
    width: 220px;
  }
  button {
    background: var(--panel);
    border: 1px solid var(--border);
    color: var(--text);
    padding: 5px 12px;
    border-radius: 6px;
    font-size: 13px;
    cursor: pointer;
  }
  button:hover { border-color: var(--muted); }
  .grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 14px;
  }
  @media (max-width: 900px) {
    .grid { grid-template-columns: 1fr; }
  }
  .panel {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 14px 16px;
  }
  .panel h2 {
    font-size: 13px;
    font-weight: 600;
    color: var(--muted);
    margin: 0 0 10px 0;
    text-transform: uppercase;
    letter-spacing: 0.4px;
  }
  .chart-wrap {
    position: relative;
    height: 240px;
  }
  .readouts {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 10px;
    margin-bottom: 14px;
  }
  .readout {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 10px;
    padding: 10px 14px;
  }
  .readout .label {
    font-size: 11px;
    color: var(--muted);
    text-transform: uppercase;
    letter-spacing: 0.4px;
  }
  .readout .value {
    font-size: 22px;
    font-weight: 600;
    font-variant-numeric: tabular-nums;
    margin-top: 2px;
  }
  .readout .unit {
    font-size: 12px;
    color: var(--muted);
    font-weight: 400;
  }
</style>
</head>
<body>

<header>
  <h1>Thigh IMU &mdash; Live Dashboard</h1>
  <div class="controls">
    <input type="text" id="wsUrl" value="ws://raspberrypi.local:8765">
    <button id="connectBtn">Connect</button>
  </div>
  <div class="status">
    <span class="dot" id="statusDot"></span>
    <span id="statusText">Disconnected</span>
  </div>
</header>

<div class="readouts">
  <div class="readout">
    <div class="label">Pitch</div>
    <div class="value" id="rPitch">&mdash;<span class="unit">&deg;</span></div>
  </div>
  <div class="readout">
    <div class="label">Phase Variable</div>
    <div class="value" id="rPhase">&mdash;</div>
  </div>
  <div class="readout">
    <div class="label">Samples / sec</div>
    <div class="value" id="rHz">&mdash;<span class="unit">Hz</span></div>
  </div>
</div>

<div class="grid">
  <div class="panel">
    <h2>Thigh Pitch Angle</h2>
    <div class="chart-wrap"><canvas id="pitchChart"></canvas></div>
  </div>
  <div class="panel">
    <h2>Phase Variable</h2>
    <div class="chart-wrap"><canvas id="phaseChart"></canvas></div>
  </div>
</div>

<script>
const WINDOW_SECONDS = 10;

let ws = null;
let t0 = null;
let sampleCount = 0;
let lastHzCheck = performance.now();
let hzCounter = 0;
let redrawCounter = 0;
const REDRAW_EVERY_N = 4;

const pitchData = [];
const phaseData = [];

const chartDefaults = {
  animation: false,
  responsive: true,
  maintainAspectRatio: false,
  scales: {
    x: {
      type: "linear",
      ticks: { color: "#8b949e", maxTicksLimit: 6 },
      grid: { color: "#21262d" },
      title: { display: true, text: "Time (s)", color: "#8b949e" }
    },
    y: {
      ticks: { color: "#8b949e" },
      grid: { color: "#21262d" }
    }
  },
  plugins: { legend: { labels: { color: "#c9d1d9", boxWidth: 12, font: { size: 11 } } } }
};

const pitchChart = new Chart(document.getElementById("pitchChart"), {
  type: "line",
  data: { datasets: [{ label: "Pitch (deg)", data: pitchData, borderColor: "#58a6ff", backgroundColor: "transparent", pointRadius: 0, borderWidth: 1.5 }] },
  options: JSON.parse(JSON.stringify(chartDefaults))
});

const phaseChart = new Chart(document.getElementById("phaseChart"), {
  type: "line",
  data: { datasets: [{ label: "Phase (0-1)", data: phaseData, borderColor: "#d29922", backgroundColor: "transparent", pointRadius: 0, borderWidth: 1.5 }] },
  options: (() => {
    const o = JSON.parse(JSON.stringify(chartDefaults));
    o.scales.y.min = 0;
    o.scales.y.max = 1;
    o.scales.y.reverse = true;
    return o;
  })()
});

function pruneWindow(arr, tNow) {
  while (arr.length && (tNow - arr[0].x) > WINDOW_SECONDS) arr.shift();
}

function handleSample(msg) {
  if (t0 === null) t0 = msg.t;
  const tRel = msg.t - t0;

  hzCounter++;
  const now = performance.now();
  if (now - lastHzCheck >= 1000) {
    document.getElementById("rHz").innerHTML = `${hzCounter}<span class="unit">Hz</span>`;
    hzCounter = 0;
    lastHzCheck = now;
  }

  document.getElementById("rPitch").innerHTML = `${msg.pitch_deg.toFixed(1)}<span class="unit">&deg;</span>`;
  document.getElementById("rPhase").textContent = msg.phase_var !== null ? msg.phase_var.toFixed(3) : "\u2014";

  pitchData.push({ x: tRel, y: msg.pitch_deg });
  pruneWindow(pitchData, tRel);

  if (msg.phase_var !== null) {
    phaseData.push({ x: tRel, y: msg.phase_var });
    pruneWindow(phaseData, tRel);
  }

  const xMin = tRel - WINDOW_SECONDS;
  [pitchChart, phaseChart].forEach(c => {
    c.options.scales.x.min = xMin;
    c.options.scales.x.max = tRel;
  });

  redrawCounter++;
  if (redrawCounter >= REDRAW_EVERY_N) {
    redrawCounter = 0;
    pitchChart.update("none");
    phaseChart.update("none");
  }
}

function setStatus(connected, text) {
  document.getElementById("statusDot").classList.toggle("connected", connected);
  document.getElementById("statusText").textContent = text;
}

function connect() {
  const url = document.getElementById("wsUrl").value.trim();
  if (!url) return;

  if (ws) { ws.close(); }

  setStatus(false, "Connecting...");
  ws = new WebSocket(url);

  ws.onopen = () => setStatus(true, "Connected");
  ws.onclose = () => setStatus(false, "Disconnected");
  ws.onerror = () => setStatus(false, "Connection error");
  ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      handleSample(msg);
    } catch (e) {
      console.error("Bad message:", e);
    }
  };
}

document.getElementById("connectBtn").addEventListener("click", connect);
</script>

</body>
</html>
```

## Parameter reference

| Parameter | Purpose | Determination method |
|---|---|---|
| `THIGH_AXIS` | Which Euler angle is thigh flexion/extension | Confirmed against real walking data |
| `PITCH_SIGN` | Sign convention | Set per physiological convention |
| `A_PITCH` | Normalizes thigh angle | Amplitude of centered pitch from a real trial: 23.7° |
| `B_GYRO` | Normalizes angular velocity | Amplitude of centered derived rate: 150.8°/s |
| `MIN_RADIUS` | Excludes unstable near-origin points | Large enough to reject noise, small enough to keep real low-velocity gait phases |
| `ALPHA_GYRO` | Smoothing for derived rate | Balances noise rejection vs. loop distortion |

## Files

- `calibrate_imu.py` — calibration utility
- `imu_phase_13july.py` — CSV logger + summary plot
- `imu_dashboard_ws.py` — WebSocket streaming version
- `imu_dashboard.html` — browser dashboard

## Requirements

```bash
pip install python-mscl websockets --break-system-packages
```

## Usage

```bash
# Calibrate once, save reference pose
python3 calibrate_imu.py --out calibration.json

# CSV logging + summary plot
python3 imu_phase_13july.py --calibration calibration.json --seconds 10

# Live dashboard
python3 imu_dashboard_ws.py --calibration calibration.json
# then open imu_dashboard.html in a browser and connect to ws://<pi-ip>:8765
```
