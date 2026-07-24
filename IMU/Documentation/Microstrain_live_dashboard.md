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
