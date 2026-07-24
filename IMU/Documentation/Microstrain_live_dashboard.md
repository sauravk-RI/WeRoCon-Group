# Thigh IMU — Live WebSocket Dashboard

This README covers the **real-time streaming pipeline**: a Python WebSocket
server (`imu_dashboard_ws.py`) that reads the MicroStrain IMU and broadcasts
JSON samples, plus an HTML/JS dashboard (`imu_dashboard.html`) that connects
to it and plots pitch, angular velocity, the phase portrait, and the phase
variable live using Chart.js.

It shares its core math with the CSV logger described in
`README_side_mount_logging.md` — same relative-quaternion approach, no file
writing, no matplotlib, just live broadcast + live plotting.

---

## 1. Architecture

```
MicroStrain IMU (serial, /dev/ttyACM0)
        │
        ▼
imu_dashboard_ws.py  (Python, asyncio + websockets)
        │  JSON over ws://<host>:8765
        ▼
imu_dashboard.html   (browser, Chart.js)
```

The server computes one sample per loop tick and pushes it as JSON to every
connected browser client. The HTML page is a static file you open directly —
no separate web server needed for the frontend, it just opens a WebSocket
connection out to wherever the Python script is running.

---

## 2. Server: `imu_dashboard_ws.py`

### Same relative-quaternion math as the CSV logger

Identical `quat_mul`, `quat_conj`, and `quat_relative_euler_deg` functions to
`micro_strain_side_mount.py` — see the other README for the full explanation
of why relative quaternions (rotation since calibration, not world-frame
Euler angles) are used instead of directly converting the sensor's absolute
attitude quaternion. Short version: it avoids gimbal lock and makes the
numbers meaningful regardless of which way you were facing when you
calibrated.

### Configuration differences from the CSV logger

```python
PITCH_SIGN = -1.0        # CSV logger uses +1.0
THIGH_AXIS = "y"          # CSV logger uses "z"

A_PITCH    = 23.7   # deg   — CSV logger uses 13.0
B_GYRO     = 150.8  # deg/s — CSV logger uses 127.0
MIN_RADIUS = 0.05         # CSV logger uses 0.15

WS_HOST = "0.0.0.0"
WS_PORT = 8765
```

> **Note:** this script sets `THIGH_AXIS = "y"`, matching the comment above it
> ("swing axis is ~97% Y"). The CSV logger script (`micro_strain_side_mount.py`)
> has the same comment but sets `THIGH_AXIS = "z"` instead — see the
> corresponding note in `README_side_mount_logging.md`. If both scripts are
> meant to be measuring the same physical thing off the same mount, it's
> worth reconciling which axis is actually correct before comparing data
> between the two pipelines directly, since right now they're computing
> `pitch_deg` from two different physical axes.
>
> The phase-portrait normalization constants (`A_PITCH`, `B_GYRO`,
> `MIN_RADIUS`) also differ between the two scripts. Since these are used to
> scale pitch/rate into a roughly unit circle for the phase-angle
> calculation, using different constants across pipelines means a given
> `phase_var` value isn't directly comparable between a live-dashboard
> session and a CSV-logged session unless that's intentional (e.g. re-tuned
> per session/subject).

### `StreamState` — carrying loop state across async calls

The CSV logger keeps `last_phase`, `gy_filt`, `prev_t`, `prev_pitch` as plain
local variables inside one long `while` loop. Since the dashboard version
needs to hand samples off between an async broadcaster and (potentially)
future refactors, that state is wrapped in a small class instead:

```python
class StreamState:
    def __init__(self):
        self.last_phase = None
        self.gy_filt = 0.0
        self.have_gy_filt = False
        self.prev_t = None
        self.prev_pitch = None
```

`compute_sample(node, ref_quat, state)` is the direct async-friendly
equivalent of the per-iteration body inside the CSV logger's `log_session()`
loop — same derived-rate EMA filtering (`ALPHA_GYRO = 0.04`), same
phase-variable computation with phase held over when the portrait radius is
below `MIN_RADIUS`. It returns a dict instead of writing a CSV row:

```python
{
    "t": now,
    "roll_deg": roll_deg,
    "pitch_deg": pitch_deg,
    "yaw_deg": yaw_deg,
    "sensor_ang_vel_deg_s": sensor_rate_deg_s,
    "derived_ang_vel_deg_s": derived_rate_deg_s,
    "phase_var": phase,
}
```

Note fields use `None` here (JSON `null`) where the CSV version used empty
strings — this matters for the HTML side, which checks `!== null` before
using a value.

### Broadcasting: fixed-cadence loop, same pattern as the CSV logger

```python
async def imu_broadcaster(node, ref_quat, connected_clients, loop_period_s=0.01):
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
```

Same "target 100 Hz, resync instead of bursting if we fall behind" pattern as
the CSV logger, just using `asyncio.sleep` instead of `time.sleep` so it
doesn't block the event loop (which also needs to service incoming
WebSocket connections).

Each send is wrapped so a slow/dead client can't stall the whole broadcast
loop:

```python
async def _safe_send(client, message):
    try:
        await client.send(message)
    except Exception:
        pass  # cleanup handled in ws_handler when the connection actually drops
```

Using `asyncio.create_task(...)` per client per sample means sends to
multiple clients happen concurrently rather than one after another — a slow
client doesn't hold up delivery to a fast one.

### Connection handling

```python
async def ws_handler(websocket, connected_clients):
    connected_clients.add(websocket)
    try:
        async for _ in websocket:
            pass   # dashboard never sends anything back — just keep the socket open
    finally:
        connected_clients.discard(websocket)
```

Any number of browsers can connect simultaneously (e.g. a phone and a
laptop watching the same trial); each gets an independent copy of every
sample.

### Startup sequence (`main_async`)

1. Connect to the sensor.
2. Calibrate live (3s) or load a saved calibration file via `--calibration`
   (same `calibrate_imu.py`-produced JSON as the CSV pipeline).
3. **3-second stabilize window** — read-and-discard samples so you have time
   to get moving before anything meaningful streams out.
4. Start the WebSocket server and the broadcaster loop together; runs forever
   until you Ctrl+C.

### Usage

```bash
pip install python-mscl websockets --break-system-packages

python3 imu_dashboard_ws.py
python3 imu_dashboard_ws.py --calibration imu_side_mounted_calibration.json
```

Then, on the same machine (or any machine that can reach it over the
network):

```bash
# open imu_dashboard.html in a browser
```

Console output tells you the exact address to use: `WebSocket server running
at ws://0.0.0.0:8765`. From another device, replace `0.0.0.0` with the
sensor host's actual IP or `.local` hostname.

---

## 3. Frontend: `imu_dashboard.html`

Single static HTML file — dark-themed dashboard, no build step, Chart.js
pulled from a CDN. Open it directly in a browser (double-click, or
`file://` — no server needed for the HTML itself).

### Layout

- **Header**: title, a WebSocket URL text box (defaults to
  `ws://raspberrypi.local:8765`) + Connect button, and a connection-status dot
  (red = disconnected, green = connected).
- **Readout row** (4 live numeric tiles): current pitch, current derived
  angular velocity, current phase variable, and measured samples/sec.
- **2×2 chart grid**:
  1. Pitch over time (line)
  2. Angular velocity — sensor vs. derived, overlaid (2-line chart)
  3. Phase portrait — pitch (x) vs. derived angular velocity (y), scatter
  4. Phase variable over time (line, y-axis locked to 0–1)

### Connecting

```javascript
function connect() {
  const url = document.getElementById("wsUrl").value.trim();
  if (ws) ws.close();
  ws = new WebSocket(url);
  ws.onopen    = () => setStatus(true, "Connected");
  ws.onclose   = () => setStatus(false, "Disconnected");
  ws.onerror   = () => setStatus(false, "Connection error");
  ws.onmessage = (event) => handleSample(JSON.parse(event.data));
}
```

Edit the URL field to point at wherever `imu_dashboard_ws.py` is actually
running (its own machine's `localhost`, a `.local` mDNS hostname, or a raw
IP), then click Connect. If you're viewing the dashboard from a different
device than the one running the Python script, update the default URL in the
`<input>`'s `value` attribute, or just type the correct address into the
box each time.

### Rolling time window

Each time-series chart only keeps the last `WINDOW_SECONDS = 10` seconds of
data — older points are dropped every sample:

```javascript
function pruneWindow(arr, tNow) {
  while (arr.length && (tNow - arr[0].x) > WINDOW_SECONDS) arr.shift();
}
```

The phase-portrait scatter plot instead caps by **point count**
(`MAX_PORTRAIT_POINTS = 1500`), since it's not time-indexed on either axis —
old points are shifted out once the cap is hit, giving a rolling "trail"
effect on the portrait shape.

### Handling an incoming sample

```javascript
function handleSample(msg) {
  if (t0 === null) t0 = msg.t;      // first sample sets the time origin
  const tRel = msg.t - t0;          // everything after is relative to it

  // update the 4 numeric readout tiles
  // push into pitchData / rateData.sensor / rateData.derived / phaseData
  // prune each to the rolling window
  // push into portraitData if derived rate is available, cap at MAX_PORTRAIT_POINTS
  // shift the chart x-axis window to [tRel - WINDOW_SECONDS, tRel]
}
```

Each field from the server (`sensor_ang_vel_deg_s`, `derived_ang_vel_deg_s`,
`phase_var`) is checked against `!== null` before being plotted or used to
update a readout — matches the server sending JSON `null` (rather than the
CSV pipeline's empty string) for "not available yet" values, e.g. before the
first `Δt` can be computed.

### Throttled redraws

Charts aren't redrawn on every single incoming sample — only every
`REDRAW_EVERY_N = 4` samples:

```javascript
redrawCounter++;
if (redrawCounter >= REDRAW_EVERY_N) {
  redrawCounter = 0;
  pitchChart.update("none");
  rateChart.update("none");
  phaseChart.update("none");
  portraitChart.update("none");
}
```

Data is still recorded into the underlying arrays every sample (so nothing
is lost from the rolling window), only the actual canvas repaint is
throttled — keeps the browser responsive at 100 Hz input without redrawing
4 charts 100 times a second. `chart.update("none")` skips Chart.js's
built-in animation, which would otherwise fight with the rolling window.

### Samples/sec readout

A simple 1-second tumbling counter, independent of the chart redraw
throttling:

```javascript
hzCounter++;
if (performance.now() - lastHzCheck >= 1000) {
  document.getElementById("rHz").innerHTML = `${hzCounter}Hz`;
  hzCounter = 0;
  lastHzCheck = performance.now();
}
```

Useful as a sanity check that the server's target 100 Hz loop is actually
keeping pace end-to-end (serial read → compute → WebSocket → browser), not
just that the server-side loop timer is hitting its target.

---

## 4. Sensor Connect setup

Same requirement as the CSV pipeline: the Estimation Filter's **"Attitude
(Quaternion)"** channel must be enabled in MicroStrain Sensor Connect before
running `imu_dashboard_ws.py`.

---

## 5. Dependencies

**Server:**
```bash
pip install python-mscl websockets --break-system-packages
```

**Client:** nothing to install — `imu_dashboard.html` loads Chart.js from
`cdnjs.cloudflare.com` at load time, so the viewing machine needs internet
access (or you'd need to vendor the Chart.js file locally for a fully
offline setup).

---

## 6. Relationship to the CSV pipeline

| | CSV logger (`micro_strain_side_mount.py`) | Dashboard (`imu_dashboard_ws.py`) |
|---|---|---|
| Output | CSV file + end-of-trial PNG | Live JSON over WebSocket |
| Calibration | Live 3s or `--calibration <file>` | Live 3s or `--calibration <file>` |
| Loop rate target | 100 Hz (`loop_period_s=0.01`) | 100 Hz (`loop_period_s=0.01`) |
| `THIGH_AXIS` | `"z"` | `"y"` |
| `PITCH_SIGN` | `+1.0` | `-1.0` |
| `A_PITCH` / `B_GYRO` / `MIN_RADIUS` | `13.0` / `127.0` / `0.15` | `23.7` / `150.8` / `0.05` |
| Plotting | matplotlib PNG (`Agg` backend) or MATLAB (`analyze_walking_trial.m`) | Chart.js, live in-browser |

Both share the same calibration file format (from `calibrate_imu.py`) and
the same core relative-quaternion math — they diverge only in the
configuration constants above and in what they do with each computed sample
(write a row vs. broadcast JSON). See the axis/constant mismatch notes in
both READMEs before treating data from the two pipelines as directly
comparable.

---

## 7. Full source listings

### `imu_dashboard_ws.py`

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

### `imu_dashboard.html`

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
    --sensor-rate: #f85149;
    --derived-rate: #3fb950;
    --phase: #d29922;
    --portrait: #ff7b72;
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
    grid-template-columns: repeat(4, 1fr);
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
    <div class="label">Derived Ang. Vel.</div>
    <div class="value" id="rRate">&mdash;<span class="unit">&deg;/s</span></div>
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
    <h2>Angular Velocity &mdash; Sensor vs Derived</h2>
    <div class="chart-wrap"><canvas id="rateChart"></canvas></div>
  </div>
  <div class="panel">
    <h2>Phase Portrait (Pitch vs Derived Ang. Vel.)</h2>
    <div class="chart-wrap"><canvas id="portraitChart"></canvas></div>
  </div>
  <div class="panel">
    <h2>Phase Variable</h2>
    <div class="chart-wrap"><canvas id="phaseChart"></canvas></div>
  </div>
</div>

<script>
const WINDOW_SECONDS = 10;
const MAX_PORTRAIT_POINTS = 1500;

let ws = null;
let t0 = null;
let sampleCount = 0;
let lastHzCheck = performance.now();
let hzCounter = 0;
let redrawCounter = 0;
const REDRAW_EVERY_N = 4;

const pitchData = [];
const rateData = { sensor: [], derived: [] };
const phaseData = [];
const portraitData = [];

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

const rateChart = new Chart(document.getElementById("rateChart"), {
  type: "line",
  data: {
    datasets: [
      { label: "Sensor (estAngularRateY)", data: rateData.sensor, borderColor: "#f85149", backgroundColor: "transparent", pointRadius: 0, borderWidth: 1.2 },
      { label: "Derived (filtered)", data: rateData.derived, borderColor: "#3fb950", backgroundColor: "transparent", pointRadius: 0, borderWidth: 1.2 }
    ]
  },
  options: JSON.parse(JSON.stringify(chartDefaults))
});

const phaseChart = new Chart(document.getElementById("phaseChart"), {
  type: "line",
  data: { datasets: [{ label: "Phase (0-1)", data: phaseData, borderColor: "#d29922", backgroundColor: "transparent", pointRadius: 0, borderWidth: 1.5 }] },
  options: (() => {
    const o = JSON.parse(JSON.stringify(chartDefaults));
    o.scales.y.min = 0;
    o.scales.y.max = 1;
    return o;
  })()
});

const portraitChart = new Chart(document.getElementById("portraitChart"), {
  type: "scatter",
  data: { datasets: [{ label: "Pitch vs Derived Ang. Vel.", data: portraitData, borderColor: "#ff7b72", backgroundColor: "#ff7b72", pointRadius: 1.5, showLine: false }] },
  options: {
    animation: false,
    responsive: true,
    maintainAspectRatio: false,
    scales: {
      x: { title: { display: true, text: "Pitch (deg)", color: "#8b949e" }, ticks: { color: "#8b949e" }, grid: { color: "#21262d" } },
      y: { title: { display: true, text: "Derived Ang. Vel. (deg/s)", color: "#8b949e" }, ticks: { color: "#8b949e" }, grid: { color: "#21262d" } }
    },
    plugins: { legend: { display: false } }
  }
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
  if (msg.derived_ang_vel_deg_s !== null) {
    document.getElementById("rRate").innerHTML = `${msg.derived_ang_vel_deg_s.toFixed(1)}<span class="unit">&deg;/s</span>`;
  }
  document.getElementById("rPhase").textContent = msg.phase_var !== null ? msg.phase_var.toFixed(3) : "\u2014";

  pitchData.push({ x: tRel, y: msg.pitch_deg });
  pruneWindow(pitchData, tRel);

  if (msg.sensor_ang_vel_deg_s !== null) {
    rateData.sensor.push({ x: tRel, y: msg.sensor_ang_vel_deg_s });
    pruneWindow(rateData.sensor, tRel);
  }
  if (msg.derived_ang_vel_deg_s !== null) {
    rateData.derived.push({ x: tRel, y: msg.derived_ang_vel_deg_s });
    pruneWindow(rateData.derived, tRel);
  }

  if (msg.phase_var !== null) {
    phaseData.push({ x: tRel, y: msg.phase_var });
    pruneWindow(phaseData, tRel);
  }

  if (msg.derived_ang_vel_deg_s !== null) {
    portraitData.push({ x: msg.pitch_deg, y: msg.derived_ang_vel_deg_s });
    if (portraitData.length > MAX_PORTRAIT_POINTS) portraitData.shift();
  }

  const xMin = tRel - WINDOW_SECONDS;
  [pitchChart, rateChart, phaseChart].forEach(c => {
    c.options.scales.x.min = xMin;
    c.options.scales.x.max = tRel;
  });

  redrawCounter++;
  if (redrawCounter >= REDRAW_EVERY_N) {
    redrawCounter = 0;
    pitchChart.update("none");
    rateChart.update("none");
    phaseChart.update("none");
    portraitChart.update("none");
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
