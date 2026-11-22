"""sim_link.py -- TCP link between the MuJoCo simulation and the leg's Raspberry Pi.

ONE FILE, TWO IDENTICAL COPIES
    Hardware_Codes/sim_link.py            (imported by the five gait controllers)
    Simulation_Codes/osl_sim/sim_link.py  (imported by the simulation runner)
Keep them byte-identical (tools/check_reference_parity.py also checks this).
Python standard library only -- nothing extra to install on the leg Pi.

ROLES
    SimLinkServer   runs inside the hardware controller on the leg Pi. It LISTENS on
                    a TCP port, so the simulation PC only ever connects OUT (no Windows
                    firewall rule needed) and the two programs can be started in either
                    order.
    SimLinkClient   runs inside the simulation. It connects (retrying) to the leg.

PROTOCOL  (one JSON object per line, UTF-8)
    sim -> leg   HELLO{proto, token, gait}   HB{}   PING{id, t_sim}   ARM{}
                 GO{t_go_hw}   STOP{reason}
    leg -> sim   WELCOME{info} | REJECT{reason}   HB{}   PONG{id, t_sim, t_hw}
                 STATUS{phase, msg}   READY{angles_deg, t_hw}
                 GO_ACK{t_go_hw} | GO_REJECTED{reason}   STARTED{t0_hw, late_s}
                 S{samples:[...]}   DONE{status, reason, published, dropped}

SEQUENCE
    1. leg script loads its trajectory, opens NOTHING (CAN/I2C closed, motors limp),
       and waits for the simulation              -> SimLinkServer.wait_for_client()
    2. sim connects, authenticates with the shared token, receives the leg's resolved
       reference/gains/limits (WELCOME.info)
    3. operator presses ARM in the sim           -> leg wakes motors, homes, settles
       (a disconnect BEFORE arm just makes the leg wait for a new simulation)
    4. leg holds the start pose and sends READY
    5. operator presses START: the sim measures the clock offset between the two
       computers (PING/PONG, best of N round trips) and sends GO with a start instant
       in the LEG's clock, ~GO_LEAD_S in the future. Both sides start at that instant.
    6. leg streams every control tick back (S batches); sim logs them
    7. leg sends DONE after the trajectory (or after an abort)

SAFETY (after ARM)
    The leg aborts -- both motors released by the controller's own finally block --
    if the sim sends STOP, the TCP connection drops, or no message has arrived for
    heartbeat_timeout_s. The sim sends HB every 0.2 s. All existing hardware limits,
    tracking-error and I2C guards in the controllers remain in force.

TIMING
    The control thread never touches the network: publish()/send_*() only append to
    in-memory queues; a writer thread sends them. Clocks are time.perf_counter() on both
    sides (the hardware scripts' own t0_pid clock).
"""
import json
import os
import queue
import socket
import threading
import time
from collections import deque

PROTOCOL = "osl-sim2real-v1"
DEFAULT_PORT = 8795
TOKEN_ENV = "OSL_SIM2REAL_TOKEN"
_HB_PERIOD_S = 0.2
_BATCH_PERIOD_S = 0.02
_MAX_LINE = 16 * 1024 * 1024


class SimLinkAbort(RuntimeError):
    """Raised in the hardware controller when the link says the leg must stop."""


def resolve_token(token):
    """Environment variable OSL_SIM2REAL_TOKEN (if set) overrides the script setting."""
    env = os.environ.get(TOKEN_ENV)
    return env if env else token


def _encode(msg):
    return (json.dumps(msg, allow_nan=False, separators=(",", ":")) + "\n").encode("utf-8")


def _clean(value):
    """Make a value JSON-safe (NaN/inf -> None, numpy scalars -> float)."""
    if isinstance(value, bool) or value is None or isinstance(value, (int, str)):
        return value
    try:
        f = float(value)
    except (TypeError, ValueError):
        return str(value)
    return f if f == f and f not in (float("inf"), float("-inf")) else None


class _LineSocket:
    """Line-oriented reading on a socket (works the same on Linux and Windows)."""

    def __init__(self, sock):
        self.sock = sock
        self.buf = b""

    def readline(self):
        while b"\n" not in self.buf:
            chunk = self.sock.recv(65536)
            if not chunk:
                return None
            self.buf += chunk
            if len(self.buf) > _MAX_LINE:
                raise ValueError("line too long")
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line.decode("utf-8"))


# =========================================================================== SERVER
class SimLinkServer:
    """Hardware side. Construct BEFORE opening CAN/I2C; motors stay untouched until ARM."""

    def __init__(self, port=DEFAULT_PORT, token="", heartbeat_timeout_s=1.0, info=None,
                 gait="", host="0.0.0.0", log=print):
        self.token = resolve_token(token)
        if not self.token:
            raise ValueError("sim_link: a non-empty SIM_LINK_TOKEN (or env var "
                             f"{TOKEN_ENV}) is required")
        self.port, self.gait, self.info = int(port), gait, dict(info or {})
        self.heartbeat_timeout_s = float(heartbeat_timeout_s)
        self.log = log
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()   # reader (PONG) and writer threads share it
        self._client = None             # active socket
        self._client_id = 0
        self._authenticated = threading.Event()
        self._armed = False
        self._arm_event = threading.Event()
        self._stop_reason = None
        self._lost_reason = None
        self._last_rx = time.monotonic()
        self._go_hw = None
        self._ready = False
        self._out = queue.Queue()
        self._samples = deque()
        self.published = 0
        self.dropped = 0
        self._closing = False
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind((host, self.port))
        self._listener.listen(2)
        self._listener.settimeout(0.5)
        threading.Thread(target=self._accept_loop, name="sim_link_accept", daemon=True).start()
        threading.Thread(target=self._writer_loop, name="sim_link_writer", daemon=True).start()
        self.log(f"[sim-link] Listening on TCP port {self.port} for the simulation "
                 f"(this Pi's addresses: {', '.join(_local_addresses()) or 'unknown'}).")

    # -------------------------------------------------------------- network threads
    def _accept_loop(self):
        while not self._closing:
            try:
                sock, addr = self._listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with self._lock:
                busy = self._client is not None
            if busy:
                try:
                    sock.sendall(_encode({"type": "REJECT", "reason": "leg already has a "
                                          "simulation connected"}))
                finally:
                    sock.close()
                continue
            threading.Thread(target=self._client_loop, args=(sock, addr),
                             name="sim_link_client", daemon=True).start()

    def _client_loop(self, sock, addr):
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        reader = _LineSocket(sock)
        try:
            sock.settimeout(5.0)
            hello = reader.readline()
            if (not hello or hello.get("type") != "HELLO" or hello.get("proto") != PROTOCOL
                    or hello.get("token") != self.token):
                why = "bad token or protocol" if hello else "no HELLO"
                sock.sendall(_encode({"type": "REJECT", "reason": why}))
                self.log(f"[sim-link] Rejected connection from {addr[0]}: {why}.")
                sock.close()
                return
            if self.gait and hello.get("gait") and hello["gait"] != self.gait:
                why = (f"gait mismatch: the leg is running '{self.gait}' but the simulation "
                       f"script is '{hello['gait']}'")
                sock.sendall(_encode({"type": "REJECT", "reason": why}))
                self.log(f"[sim-link] Rejected {addr[0]}: {why}.")
                sock.close()
                return
            sock.settimeout(None)
            with self._lock:
                self._client = sock
                self._client_id += 1
                self._last_rx = time.monotonic()
            with self._send_lock:
                sock.sendall(_encode({"type": "WELCOME", "info": self.info,
                                      "t_hw": time.perf_counter()}))
            self._authenticated.set()
            self.log(f"[sim-link] Simulation connected from {addr[0]}.")
            while True:
                msg = reader.readline()
                if msg is None:
                    raise ConnectionError("simulation closed the connection")
                self._last_rx = time.monotonic()
                self._handle(sock, msg)
        except (OSError, ValueError, ConnectionError) as exc:
            reason = f"link to simulation lost ({exc})"
        else:
            reason = "link to simulation lost"
        with self._lock:
            if self._client is sock:
                self._client = None
                self._authenticated.clear()
                if self._armed and self._lost_reason is None:
                    self._lost_reason = reason
        try:
            sock.close()
        except OSError:
            pass
        if not self._closing:
            self.log(f"[sim-link] {reason}.")

    def _handle(self, sock, msg):
        kind = msg.get("type")
        if kind == "PING":
            with self._send_lock:
                sock.sendall(_encode({"type": "PONG", "id": msg.get("id"),
                                      "t_sim": msg.get("t_sim"), "t_hw": time.perf_counter()}))
        elif kind == "ARM":
            with self._lock:
                self._armed = True
            self._arm_event.set()
        elif kind == "GO":
            t_go = float(msg.get("t_go_hw", 0.0))
            now = time.perf_counter()
            if not self._ready:
                self._out.put({"type": "GO_REJECTED", "reason": "leg is not READY yet"})
            elif self._go_hw is not None:
                self._out.put({"type": "GO_REJECTED", "reason": "already started"})
            elif t_go < now + 0.005:
                self._out.put({"type": "GO_REJECTED", "reason": f"start instant already "
                               f"passed by {now - t_go:.3f} s (increase GO_LEAD_S)"})
            elif t_go > now + 30.0:
                self._out.put({"type": "GO_REJECTED", "reason": "start instant >30 s ahead"})
            else:
                self._go_hw = t_go
                self._out.put({"type": "GO_ACK", "t_go_hw": t_go})
        elif kind == "STOP":
            with self._lock:
                self._stop_reason = "simulation requested STOP" + (
                    f" ({msg['reason']})" if msg.get("reason") else "")
            if not self._armed:
                self._arm_event.set()     # wake wait_for_arm so it can report the stop
        # HB and unknown messages only refresh _last_rx

    def _writer_loop(self):
        last_hb = 0.0
        while True:
            try:
                msgs = [self._out.get(timeout=_BATCH_PERIOD_S)]
            except queue.Empty:
                msgs = []
            while True:
                try:
                    msgs.append(self._out.get_nowait())
                except queue.Empty:
                    break
            batch = []
            while self._samples:
                batch.append(self._samples.popleft())
            if batch:
                msgs.insert(0, {"type": "S", "samples": batch})
            now = time.monotonic()
            if not msgs and now - last_hb >= _HB_PERIOD_S:
                msgs.append({"type": "HB"})
            with self._lock:
                sock = self._client
            if msgs and sock is not None:
                try:
                    data = b"".join(_encode(m) for m in msgs)
                    with self._send_lock:
                        sock.sendall(data)
                    last_hb = now
                except OSError:
                    pass                     # the reader thread reports the loss
            if self._closing and self._out.empty() and not self._samples:
                return

    # -------------------------------------------------------------- controller API
    def wait_for_client(self):
        """Block until an authenticated simulation is connected (Ctrl+C to quit)."""
        self.log("[sim-link] SIM_2_REAL=1 -- waiting for the simulation to connect "
                 "(motors are NOT powered; Ctrl+C to quit)...")
        while not self._authenticated.wait(0.2):
            pass

    def wait_for_arm(self):
        """Block until the sim sends ARM. Before ARM a lost simulation is simply waited
        for again -- nothing on the leg is energised yet."""
        self.log("[sim-link] Waiting for ARM from the simulation (leg stays limp)...")
        while True:
            if not self._authenticated.is_set():
                self._stop_reason = None
                self._arm_event.clear()
                self.wait_for_client()
                self.log("[sim-link] Waiting for ARM from the simulation (leg stays limp)...")
            if self._arm_event.wait(0.2):
                if self._armed:
                    self._last_rx = time.monotonic()
                    self.log("[sim-link] ARM received -- powering motors and homing.")
                    self.send_status("ARMED", "arm received; waking motors")
                    return
                # STOP before ARM: forget this simulation, wait for another one
                self._arm_event.clear()
                self._stop_reason = None

    def abort_reason(self):
        """None while it is safe to keep driving the leg, else a reason string."""
        if self._stop_reason:
            return self._stop_reason
        if self._lost_reason:
            return self._lost_reason
        if self._armed and time.monotonic() - self._last_rx > self.heartbeat_timeout_s:
            with self._lock:
                if self._lost_reason is None:
                    self._lost_reason = (f"no message from the simulation for "
                                         f"{self.heartbeat_timeout_s:.2f} s (heartbeat timeout)")
            return self._lost_reason
        return None

    def check(self):
        reason = self.abort_reason()
        if reason:
            raise SimLinkAbort("[sim-link] " + reason + " -- stopping the leg")

    def send_status(self, phase, msg=""):
        self._out.put({"type": "STATUS", "phase": phase, "msg": msg})

    def send_ready(self, angles_deg):
        self._ready = True
        self._out.put({"type": "READY", "t_hw": time.perf_counter(),
                       "angles_deg": {k: _clean(v) for k, v in angles_deg.items()}})

    def go_time(self):
        """The accepted start instant (time.perf_counter() domain), or None."""
        return self._go_hw

    def notify_started(self, t0_hw):
        self._out.put({"type": "STARTED", "t0_hw": t0_hw,
                       "late_s": (t0_hw - self._go_hw) if self._go_hw is not None else None})

    def publish(self, **sample):
        """Queue one joint's control-tick sample. Never blocks."""
        if len(self._samples) > 200000:
            self.dropped += 1
            return
        self._samples.append({k: _clean(v) for k, v in sample.items()})
        self.published += 1

    def finish(self, status="COMPLETE", reason=None, drain_s=3.0):
        """Send DONE, give the writer a bounded time to drain, then close."""
        self._out.put({"type": "DONE", "status": status, "reason": reason,
                       "published": self.published, "dropped": self.dropped})
        deadline = time.monotonic() + drain_s
        while (not self._out.empty() or self._samples) and time.monotonic() < deadline:
            time.sleep(0.02)
        time.sleep(0.2)
        self.close()

    def close(self):
        self._closing = True
        try:
            self._listener.close()
        except OSError:
            pass
        with self._lock:
            sock, self._client = self._client, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()


def _local_addresses():
    addrs = set()
    try:
        for res in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            addrs.add(res[4][0])
    except OSError:
        pass
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))     # no packet is sent
        addrs.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    addrs.discard("127.0.0.1")
    return sorted(addrs)


# =========================================================================== CLIENT
class SimLinkClient:
    """Simulation side."""

    def __init__(self, host, port=DEFAULT_PORT, token="", gait="", log=print,
                 raw_log_path=None, link_timeout_s=3.0):
        self.host, self.port, self.gait = host, int(port), gait
        self.token = resolve_token(token)
        self.log = log
        self.link_timeout_s = float(link_timeout_s)
        self.sock = None
        self.info = None
        self.samples = []
        self.events = queue.Queue()
        self._pongs = queue.Queue()
        self._send_lock = threading.Lock()
        self._closed = False
        self.lost_reason = None
        self._last_rx = time.monotonic()
        self.done = None
        self.started = None
        self._raw = open(raw_log_path, "a", encoding="utf-8") if raw_log_path else None

    # -------------------------------------------------------------- connection
    def connect(self, retry_forever=True):
        if not self.token:
            raise ValueError("sim_link: SIM_LINK_TOKEN must not be empty")
        self.log(f"[sim-link] Connecting to the leg at {self.host}:{self.port} "
                 f"(start the hardware script with SIM_2_REAL = 1; Ctrl+C to quit)...")
        last_note = 0.0
        while True:
            try:
                sock = socket.create_connection((self.host, self.port), timeout=3.0)
                break
            except OSError as exc:
                if not retry_forever:
                    raise
                if time.monotonic() - last_note > 5.0:
                    self.log(f"[sim-link]   leg not reachable yet ({exc}); retrying...")
                    last_note = time.monotonic()
                time.sleep(0.5)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.sendall(_encode({"type": "HELLO", "proto": PROTOCOL, "token": self.token,
                              "gait": self.gait}))
        reader = _LineSocket(sock)
        sock.settimeout(10.0)
        reply = reader.readline()
        if not reply:
            raise ConnectionError("leg closed the connection during the handshake")
        if reply.get("type") == "REJECT":
            raise ConnectionError("leg rejected the simulation: " + str(reply.get("reason")))
        if reply.get("type") != "WELCOME":
            raise ConnectionError(f"unexpected handshake reply {reply.get('type')}")
        sock.settimeout(None)
        self.sock, self.info = sock, reply.get("info", {})
        self._write_raw(reply)
        self._last_rx = time.monotonic()
        threading.Thread(target=self._reader, args=(reader,), daemon=True).start()
        threading.Thread(target=self._heartbeat, daemon=True).start()
        self.log(f"[sim-link] Connected to the leg ({self.info.get('script', '?')}).")
        return self.info

    def _write_raw(self, msg):
        if self._raw:
            self._raw.write(json.dumps({"rx_t_sim": time.perf_counter(), "msg": msg}) + "\n")

    def _reader(self, reader):
        try:
            while True:
                msg = reader.readline()
                if msg is None:
                    raise ConnectionError("leg closed the connection")
                self._last_rx = time.monotonic()
                kind = msg.get("type")
                if kind == "PONG":
                    msg["t_rx"] = time.perf_counter()
                    self._pongs.put(msg)
                    continue
                if kind == "HB":
                    continue
                self._write_raw(msg)
                if kind == "S":
                    self.samples.extend(msg.get("samples", []))
                    continue
                if kind == "DONE":
                    self.done = msg
                elif kind == "STARTED":
                    self.started = msg
                self.events.put(msg)
        except (OSError, ValueError, ConnectionError) as exc:
            # the leg closing the connection AFTER its DONE message is the normal ending
            if not self._closed and self.lost_reason is None and self.done is None:
                self.lost_reason = f"link to the leg lost ({exc})"
                self.events.put({"type": "LOST", "reason": self.lost_reason})

    def _heartbeat(self):
        while not self._closed and self.lost_reason is None:
            try:
                self.send({"type": "HB"})
            except OSError:
                return
            if time.monotonic() - self._last_rx > self.link_timeout_s and self.done is None:
                self.lost_reason = (f"no message from the leg for {self.link_timeout_s:.1f} s")
                self.events.put({"type": "LOST", "reason": self.lost_reason})
                return
            time.sleep(_HB_PERIOD_S)

    def send(self, msg):
        with self._send_lock:
            self.sock.sendall(_encode(msg))

    # -------------------------------------------------------------- operations
    def wait_event(self, kinds, timeout=None, on_other=None):
        """Wait for one of the message types in ``kinds``; LOST/DONE always return."""
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            remaining = None if deadline is None else max(0.0, deadline - time.monotonic())
            if remaining == 0.0:
                return None
            try:
                msg = self.events.get(timeout=min(0.2, remaining) if remaining else 0.2)
            except queue.Empty:
                continue
            if msg["type"] in kinds or msg["type"] in ("LOST", "DONE"):
                return msg
            if on_other:
                on_other(msg)

    def arm(self):
        self.send({"type": "ARM"})

    def sync_clocks(self, n=30):
        """Return (offset, rtt): t_hw = t_sim + offset, from the lowest-RTT round trip."""
        best = None
        for i in range(n):
            t_send = time.perf_counter()
            self.send({"type": "PING", "id": i, "t_sim": t_send})
            try:
                pong = self._pongs.get(timeout=2.0)
            except queue.Empty:
                continue
            if pong.get("id") != i:
                continue
            rtt = pong["t_rx"] - t_send
            offset = pong["t_hw"] - (t_send + pong["t_rx"]) / 2.0
            if best is None or rtt < best[1]:
                best = (offset, rtt)
            time.sleep(0.01)
        if best is None:
            raise ConnectionError("clock synchronisation failed (no PONG replies)")
        return best

    def go(self, offset, lead_s):
        """Schedule the start lead_s from now. Returns t_go in THIS computer's clock."""
        t_go_sim = time.perf_counter() + lead_s
        self.send({"type": "GO", "t_go_hw": t_go_sim + offset})
        reply = self.wait_event(("GO_ACK", "GO_REJECTED"), timeout=max(lead_s, 1.0))
        if reply is None:
            raise ConnectionError("no reply to GO")
        if reply["type"] != "GO_ACK":
            raise ConnectionError("leg refused to start: " + str(reply.get("reason")))
        return t_go_sim

    def stop(self, reason=""):
        try:
            self.send({"type": "STOP", "reason": reason})
        except OSError:
            pass

    def close(self):
        self._closed = True
        if self.sock is not None:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.sock.close()
        if self._raw:
            self._raw.close()
            self._raw = None
