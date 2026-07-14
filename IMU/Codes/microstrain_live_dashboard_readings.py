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
