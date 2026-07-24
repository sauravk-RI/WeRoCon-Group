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
    python3 micro_strain_side_mount.py
    python3 micro_strain_side_mount.py --out walking_trial_1.csv
    python3 micro_strain_side_mount.py --seconds 15

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

PITCH_SIGN = +1.0

# Which axis is thigh flexion/extension. CONFIRMED 2026-07-07: swing axis is ~97% Y.
THIGH_AXIS = "z"

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
A_PITCH = 13.0     # deg
B_GYRO = 127.0     # deg/s

MIN_RADIUS = 0.15

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

    axes[1, 1].plot(t, phase, "_", markersize=4, color="tab:cyan")
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
                        p = compute_phase_var(pitch_deg, gy_filt)
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
