#!/usr/bin/env python3
"""
knee_resistance_torque_test_PID_control_me_loadcell_d_on_meas.py -- Resistance-based
torque test for the KNEE joint: the knee is stepped to a goal angle by the
same code-level PID current-loop position controller used in
knee_step_test_PID_control_me.py (Servo Current Loop mode, amperes), then
HELD there indefinitely. While it is held you push on the knee by hand; the
PID resists by adding correction current. The load cell measures the torque
you actually apply, the PID current gives the torque the motor commands, and
both are logged and plotted together.
"_me" = Magnetic Encoder (AS5048B) -- feedback comes only from it, as in the
step test.

THIS COPY differs from knee_resistance_torque_test_PID_control_me_loadcell.py in
ONE thing only: the DERIVATIVE TERM. As in knee_bode_noise_PID_control_me.py, the
D term is taken on the MEASURED angle through a first-order low-pass
(D_ON_MEASUREMENT, D_FILTER_HZ) instead of on the error. A hand push changes the
measurement, not the reference, so the behaviour while held is nearly the same,
but the arrival step no longer produces a derivative kick and the encoder
quantization noise is filtered. Set D_ON_MEASUREMENT = False to get the original
derivative-on-error behaviour back. Results go to a separate folder,
results/resistance_torque_test_knee_loadcell_d_on_meas/. Control law, glitch guard, ENCODER_TO_MOTOR_SIGN and hard joint
limits are carried over unchanged from knee_step_test_PID_control_me.py.

DIFFERENCES FROM THE KNEE STEP TEST
  1. HOLD FOREVER: there is no POST_STEP_S timeout. The PID keeps holding the
     goal angle until you press Ctrl+C.
  2. LOAD CELL: an SRI M3564F read through the OSL NBLoadcellDAQ class (same
     LoadcellReader as knee_ankle_trajectory_PID_control_me_normal_walk_loadcell.py).
     It is TARED ONCE at the very start (you press Enter with nothing loading
     it), before the motor wakes.
  3. LOGGING STARTS ON ARRIVAL: homing, the step transit and the settle are
     not logged. Once the knee has stayed within ARRIVE_TOLERANCE_DEG of the
     goal for ARRIVE_SETTLE_S, t=0 and logging begin. From then on every tick
     logs the usual PID columns plus the load cell Fy and Mx, the knee torque
     computed from them (KNEE_LOADCELL_TORQUE_SIGN * (Mx + KNEE_ARM_M * Fy)),
     and the commanded torque from the PID current (GEAR * KT * current).
  4. ABORT_ERROR_DEG is raised to 60 deg so a hard push doesn't stop the test.
     The hard joint limits still apply unconditionally.

SEQUENCE
  1. Load cell tare (Enter), then motor wake-up.
  2. Home to BASELINE_ANGLE_DEG (velocity mode), then PID steps to
     STEP_ANGLE_DEG (PID engages with fresh integral/derivative state).
  3. Arrival detected -> logging starts. Push on the knee by hand.
  4. Ctrl+C -> motor released, CSV + plots saved to
     results/resistance_torque_test_knee_loadcell/run_<N>/ (N auto-increments),
     including the commanded-vs-measured torque plot.

If the load cell cannot be started or stops sending data, the run carries on
without it (its columns are NaN) and says so at the end.
"""

# ===========================================================================
#  USER SETTINGS
# ===========================================================================

# --- CAN ID ------------------------------------------------------------
KNEE_CAN_ID = 100

# --- Step / hold -------------------------------------------------------
STEP_ANGLE_DEG = -10.0       # goal angle held until Ctrl+C (0 = full extension, flexion NEGATIVE)
BASELINE_ANGLE_DEG = 0.0    # homing target before the step

ARRIVE_TOLERANCE_DEG = 0.5  # |error| within this counts as "arrived"
ARRIVE_SETTLE_S = 0.3       # ...for this long continuously, then logging starts
ARRIVE_TIMEOUT_S = 200.0     # abort if the goal isn't reached within this long of the step

# --- PID gains (unchanged from the knee step test) ---------------------
KP = 3.0
KI = 0.0
KD = 0.1
INTEGRAL_LIMIT_DEG_S = 10   # was 50 in the step test -- smaller so stored I-current stays <= KI*10 = 1.5 A
INTEGRAL_ACTIVE_BAND_DEG = 2.0   # integrate only while |error| <= this (anti-windup against hand pushes)
# Derivative on the MEASURED angle (not on the error), low-passed -- same as
# knee_bode_noise_PID_control_me.py. False = original derivative-on-error (no filter).
D_ON_MEASUREMENT = True
D_FILTER_HZ = 100.0   # first-order low-pass on the D term (D_ON_MEASUREMENT only)
MAX_CURRENT_A = 60.0

# --- Control-loop rate ---------------------------------------------------
LOOP_HZ = 1000

# --- Joint hardware limits, degrees (unconditional) ------------------------
KNEE_LIMIT_MIN_DEG = -110.0
KNEE_LIMIT_MAX_DEG = 3.0

# --- Runaway guard (raised so hand pushes don't trip it) -------------------
ABORT_ERROR_DEG = 60.0

# --- Homing (velocity-mode-to-target) ---------------------------------------
HOMING_VELOCITY_RAD_S = 0.4
HOME_LOOP_HZ = 1000
HOME_TOLERANCE_DEG = 0.2
HOME_TIMEOUT_S = 30.0
NUM_HOME_STOP_COMMANDS = 15
HOME_STOP_COMMAND_INTERVAL_S = 0.001

# --- Implausible-jump guard ----------------------------------------------
GLITCH_JUMP_DEG = 15.0
MAX_CONSECUTIVE_WRAP_REJECTS = 10

# --- Torque conversion (same values as the loadcell trajectory script) -----
GEAR_RATIO_FACTOR = 4.61
KT = 0.7044   # N*m/A, output-shaft torque constant; joint torque = GEAR_RATIO_FACTOR * KT * current

# --- Magnetic encoder (AS5048B) settings -----------------------------------
I2C_BUS = 1
AS5048B_ADDR = 0x40
ANGLE_HIGH_REG = 0xFE
ANGLE_LOW_REG = 0xFF
AGC_REG = 0xFA
DIAG_REG = 0xFB
MAGNETIC_ENCODER_OFFSET = 120.0   # deg
ENCODER_TO_MOTOR_SIGN = -1.0
MAX_CONSECUTIVE_I2C_FAILS = 25

# --- Load cell (SRI M3564F via the OSL NBLoadcellDAQ class on SPI1) ---------
LOADCELL_ENABLE = True
LOADCELL_TARE_AT_START = True     # zero the sensor before the motor wakes (asks you to press Enter)
LOADCELL_EXCITATION_V = 5.007
LOADCELL_STALE_S = 0.1            # a sample older than this counts as "no data" for that tick
KNEE_ARM_M = 0.151                # load cell centre -> knee axis, metres
KNEE_LOADCELL_TORQUE_SIGN = 1.0   # flip to -1.0 if load cell torque has the opposite sign to the motor torque
LOADCELL_MATRIX = [
    (-8.344189, -1912.95886, 31.095542, 5.691295, -6.311593, 1814.362687),
    (-38.275154, 1121.206361, -0.012994, -2127.946099, 12.118408, 1056.375086),
    (-984.814447, -11.163688, -962.72409, -23.826888, -949.723252, 4.760664),
    (19.884734, 0.16967, -0.459657, -0.155463, -19.741508, 0.18944),
    (-11.650934, -0.613806, 22.981705, 0.478966, -11.244318, 0.654913),
    (-0.178507, -26.357561, -0.167943, -26.511556, 0.246647, -26.257032)]

# --- Plotting -------------------------------------------------------------
GENERATE_PLOTS = True
SHOW_PLOTS = True
DISPLAY_SIGN = -1.0   # cosmetic: knee shown flexion-positive on the angle columns/plot

# --- Smoothed-comparison plot (commanded mean line + spike-free measured) ---
MEAN_WINDOW_S = 0.5        # centered moving-average window for the commanded-torque mean line
SPIKE_WINDOW_S = 0.2       # rolling window used to judge whether a measured sample is a spike
SPIKE_MAD_THRESHOLD = 6.0  # |x - rolling median| > this many robust sigmas (1.4826*MAD) => spike
SPIKE_MIN_DEV_NM = 0.5     # ...and also at least this many N*m away (guards against a tiny MAD)

# ===========================================================================
#  END OF USER SETTINGS
# ===========================================================================

import os
import csv
import math
import threading
import time
from dataclasses import dataclass, field

import can
import numpy as np
from smbus2 import SMBus

import matplotlib
if SHOW_PLOTS and os.environ.get("DISPLAY"):
    try:
        matplotlib.use("TkAgg")
    except Exception:
        matplotlib.use("Agg")
else:
    matplotlib.use("Agg")          # safe for headless / SSH sessions
import matplotlib.pyplot as plt

from epicallypowerful.actuation import ActuatorGroup
from epicallypowerful.toolbox import TimedLoop

# Detach and remove the motorlog.log file the actuation library creates on import.
import logging as _logging
_motorlog = _logging.getLogger('motorlog')
for _h in _motorlog.handlers[:]:
    _motorlog.removeHandler(_h)
    _h.close()
if os.path.exists('motorlog.log'):
    os.remove('motorlog.log')

# Load cell library. A missing/broken install must NOT stop the run.
try:
    from opensourceleg.logging import LOGGER as _OSL_LOGGER
    from opensourceleg.sensors.loadcell import NBLoadcellDAQ
    _OSL_LOGGER._ensure_file_handler = lambda: None
    _LOADCELL_IMPORT_ERROR = None
except Exception as _e:
    NBLoadcellDAQ = None
    _LOADCELL_IMPORT_ERROR = f"{type(_e).__name__}: {_e}"

MOTOR_TYPE = 'AK80-9-V3-servo'   # Servo operating mode

print("=" * 78)
print("knee_resistance_torque_test_PID_control_me_loadcell_d_on_meas.py")
print("=" * 78)

for _name, _val in (("STEP_ANGLE_DEG", STEP_ANGLE_DEG),
                    ("BASELINE_ANGLE_DEG", BASELINE_ANGLE_DEG)):
    if _val < KNEE_LIMIT_MIN_DEG or _val > KNEE_LIMIT_MAX_DEG:
        raise ValueError(
            f"{_name}={_val:+.1f} deg falls outside the knee's hardware "
            f"limits [{KNEE_LIMIT_MIN_DEG:+.1f}, {KNEE_LIMIT_MAX_DEG:+.1f}] "
            f"deg -- aborting before the motor moves."
        )
print("[startup] Pre-flight limit check passed.")


# ---------------------------------------------------------------------------
# Results directory: results/resistance_torque_test_knee_loadcell/run_<N>/
# ---------------------------------------------------------------------------
_script_dir = os.path.dirname(os.path.abspath(__file__))
_results_root = os.path.join(_script_dir, "results", "resistance_torque_test_knee_loadcell_d_on_meas")
os.makedirs(_results_root, exist_ok=True)
_existing_run_numbers = []
for _name in os.listdir(_results_root):
    if _name.startswith("run_") and _name[len("run_"):].isdigit():
        _existing_run_numbers.append(int(_name[len("run_"):]))
RUN_NUMBER = max(_existing_run_numbers, default=0) + 1
RUN_DIR = os.path.join(_results_root, f"run_{RUN_NUMBER}")
os.makedirs(RUN_DIR)
print(f"[startup] This is run #{RUN_NUMBER}. Results (CSV + plots) will be "
      f"saved to: {RUN_DIR}")


# ---------------------------------------------------------------------------
# PID CONTROLLER -- same P/I as the knee step test; D on the measurement
# ---------------------------------------------------------------------------
class PID:
    """Standard textbook discrete PID (output in amperes; error in degrees)."""

    def __init__(self, kp, ki, kd, integral_limit_deg_s):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit_deg_s = integral_limit_deg_s
        self.integral_deg_s = 0.0
        self.prev_error_deg = None
        self.prev_meas_deg = None
        self.d_filt_deg_s = 0.0

    def reset(self):
        self.integral_deg_s = 0.0
        self.prev_error_deg = None
        self.prev_meas_deg = None
        self.d_filt_deg_s = 0.0

    def update(self, reference_deg, measurement_deg, dt):
        error_deg = reference_deg - measurement_deg

        # Conditional integration (anti-windup): only accumulate when the
        # knee is near the goal, so a long hand push can't store up current
        # that then overshoots / rings when the push is released.
        if abs(error_deg) <= INTEGRAL_ACTIVE_BAND_DEG:
            self.integral_deg_s += error_deg * dt
        if self.integral_deg_s > self.integral_limit_deg_s:
            self.integral_deg_s = self.integral_limit_deg_s
        elif self.integral_deg_s < -self.integral_limit_deg_s:
            self.integral_deg_s = -self.integral_limit_deg_s

        if D_ON_MEASUREMENT:
            if self.prev_meas_deg is None:
                raw_deg_s = 0.0
            else:
                raw_deg_s = -(measurement_deg - self.prev_meas_deg) / dt   # = d(-y)/dt
            self.prev_meas_deg = measurement_deg
            a = math.exp(-2.0 * math.pi * D_FILTER_HZ * dt)
            self.d_filt_deg_s = a * self.d_filt_deg_s + (1.0 - a) * raw_deg_s
            derivative_deg_s = self.d_filt_deg_s
        else:
            if self.prev_error_deg is None:
                derivative_deg_s = 0.0
            else:
                derivative_deg_s = (error_deg - self.prev_error_deg) / dt
            self.prev_error_deg = error_deg

        p_term = self.kp * error_deg
        i_term = self.ki * self.integral_deg_s
        d_term = self.kd * derivative_deg_s

        return p_term + i_term + d_term, p_term, i_term, d_term, error_deg


# ------------------------------------------------------------------
# MAGNETIC ENCODER (AS5048B) SUPPORT
# ------------------------------------------------------------------
def _as5048b_read_angle_raw(bus):
    high_byte, low_byte = bus.read_i2c_block_data(AS5048B_ADDR,
                                                    ANGLE_HIGH_REG, 2)
    raw14 = ((high_byte << 6) | (low_byte & 0x3F)) & 0x3FFF
    return (raw14 / 16384.0) * 360.0


def _as5048b_read_health(bus):
    agc = bus.read_byte_data(AS5048B_ADDR, AGC_REG)
    diag = bus.read_byte_data(AS5048B_ADDR, DIAG_REG)
    return agc, diag


def wrap_deg(angle_deg):
    """Wrap an angle in degrees to (-180, 180]."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


def read_actual_angle_deg(bus):
    raw_deg = _as5048b_read_angle_raw(bus)
    return wrap_deg(raw_deg - MAGNETIC_ENCODER_OFFSET)


def read_actual_angle_deg_safe(bus, state):
    """A failed I2C read reuses the last good angle; raises after too many
    consecutive failures or if none was ever obtained."""
    try:
        angle = read_actual_angle_deg(bus)
        state.i2c_fail_streak = 0
        state.i2c_last_good_deg = angle
        state.i2c_has_good_reading = True
        return angle
    except OSError as e:
        state.i2c_fail_count += 1
        state.i2c_fail_streak += 1
        state.i2c_fail_max_streak = max(state.i2c_fail_max_streak,
                                         state.i2c_fail_streak)
        print(f"  [warning] Knee: AS5048B I2C read failed ({e}) -- "
              f"fail streak = {state.i2c_fail_streak}")
        if (not state.i2c_has_good_reading
                or state.i2c_fail_streak > MAX_CONSECUTIVE_I2C_FAILS):
            reason = ("no good reading has ever been obtained yet -- "
                      "check wiring / I2C address / power before "
                      "trusting this sensor for control.")
            if state.i2c_has_good_reading:
                reason = ("the magnetic encoder appears to be "
                          "disconnected or the I2C bus is stuck -- "
                          "aborting rather than running the control "
                          "loop blind on stale data.")
            raise RuntimeError(
                f"Knee: {state.i2c_fail_streak} consecutive AS5048B I2C "
                f"read failure(s) (last error: {e}) -- {reason}"
            ) from e
        return state.i2c_last_good_deg


# ---------------------------------------------------------------------------
# LOAD CELL SUPPORT -- background reader + knee torque formula
# ---------------------------------------------------------------------------
class LoadcellReader:
    """Reads the load cell continuously in a background thread and keeps only
    the latest [Fx, Fy, Fz, Mx, My, Mz] sample. NBLoadcellDAQ.update() blocks
    until a new ADC sample is ready, so it must never run in the PID loop; the
    loop only calls latest(), which never blocks. Every failure is caught and
    remembered -- nothing here raises into the control loop."""

    def __init__(self):
        self.loadcell = None
        self.setup_error = None      # why the load cell could not be started (None = OK)
        self.tared = False
        self.n_samples = 0
        self.n_errors = 0
        self.last_error = None
        self._started = False
        self._lock = threading.Lock()
        self._wrench = None
        self._t_sample = 0.0
        self._stop_evt = threading.Event()
        self._thread = None

    def setup(self):
        if not LOADCELL_ENABLE:
            self.setup_error = "disabled by LOADCELL_ENABLE = False"
            return
        if NBLoadcellDAQ is None:
            self.setup_error = f"opensourceleg could not be imported ({_LOADCELL_IMPORT_ERROR})"
            return
        try:
            self.loadcell = NBLoadcellDAQ(
                calibration_matrix=np.array(LOADCELL_MATRIX),
                excitation_voltage=LOADCELL_EXCITATION_V,
                amp_gain=[34, 34, 34, 151, 151, 151],
                spi_bus=1, spi_cs=0,        # SPI1 -> /dev/spidev1.0 (the library default is bus 0!)
                data_rate=500,
                gains=[1] * 6,
            )
            self.loadcell.start()
            self._started = True
        except Exception as e:
            self.setup_error = f"{type(e).__name__}: {e}"
            self._close()
            return
        if LOADCELL_TARE_AT_START:
            try:
                input("[loadcell] Make sure NOTHING is loading the load cell, then press Enter to zero it... ")
                print("[loadcell] Zeroing (taring)...")
                self.loadcell.calibrate(1024)
                self.tared = True
                print("[loadcell] Zeroing done.")
            except Exception as e:
                print(f"  [loadcell warning] Zeroing failed ({type(e).__name__}: {e}) -- "
                      f"continuing UNZEROED (readings will include the sensor's own offset).")
        self._thread = threading.Thread(target=self._run, name="loadcell-reader", daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop_evt.is_set():
            try:
                self.loadcell.update()
                w = np.array(self.loadcell.data, dtype=float)
                if w.shape != (6,) or not np.all(np.isfinite(w)):
                    raise ValueError(f"bad load cell sample: {w!r}")
                with self._lock:
                    self._wrench = w
                    self._t_sample = time.perf_counter()
                    self.n_samples += 1
            except Exception as e:
                self.n_errors += 1
                self.last_error = f"{type(e).__name__}: {e}"
                self._stop_evt.wait(0.2)

    def latest(self):
        """(wrench[6], sample_index) of the newest sample, or None if there is
        none yet or it is older than LOADCELL_STALE_S. Never blocks."""
        with self._lock:
            if self._wrench is None or time.perf_counter() - self._t_sample > LOADCELL_STALE_S:
                return None
            return self._wrench, self.n_samples

    def _close(self):
        try:
            if self.loadcell is not None and self._started:
                self.loadcell.stop()
        except Exception:
            pass
        self._started = False

    def stop(self):
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._close()


def centered_moving_mean(values, t, window_s):
    """Centered moving average (NaN-aware) over ~window_s seconds, computed
    from the actual mean sample rate. Edges use edge-replicated padding so the
    line stays centred on the data right up to the first/last sample."""
    x = np.asarray(values, dtype=float)
    n = len(x)
    if n < 2:
        return x.copy()
    fs = (n - 1) / (t[-1] - t[0]) if t[-1] > t[0] else float(LOOP_HZ)
    w = max(1, int(round(window_s * fs)))
    if w % 2 == 0:
        w += 1
    w = min(w, n if n % 2 == 1 else n - 1)
    half = w // 2
    padded = np.pad(x, half, mode="edge")
    ok = np.isfinite(padded)
    kernel = np.ones(w)
    num = np.convolve(np.where(ok, padded, 0.0), kernel, mode="valid")
    den = np.convolve(ok.astype(float), kernel, mode="valid")
    with np.errstate(invalid="ignore", divide="ignore"):
        return num / den


def remove_spikes(values, t, window_s, mad_threshold, min_dev):
    """Hampel-style outlier rejection: a sample is a spike if it is more than
    mad_threshold robust sigmas (1.4826 * MAD) AND more than min_dev away from
    the rolling median of its neighbours. Spikes are replaced by linear
    interpolation between the surrounding good samples.
    Returns (cleaned, n_spikes)."""
    x = np.asarray(values, dtype=float)
    n = len(x)
    if n < 3:
        return x.copy(), 0
    fs = (n - 1) / (t[-1] - t[0]) if t[-1] > t[0] else float(LOOP_HZ)
    w = max(3, int(round(window_s * fs)))
    if w % 2 == 0:
        w += 1
    w = min(w, n if n % 2 == 1 else n - 1)
    half = w // 2
    padded = np.pad(x, half, mode="edge")
    windows = np.lib.stride_tricks.sliding_window_view(padded, w)
    with np.errstate(all="ignore"):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            med = np.nanmedian(windows, axis=1)
            mad = np.nanmedian(np.abs(windows - med[:, None]), axis=1)
    sigma = 1.4826 * mad
    dev = np.abs(x - med)
    spike = np.isfinite(dev) & (dev > np.maximum(mad_threshold * sigma, min_dev))
    cleaned = x.copy()
    cleaned[spike] = np.nan
    # Bridge each removed spike with a straight line between its nearest good
    # neighbours (original NaN samples, i.e. missing load cell data, are
    # left as gaps).
    bad = spike
    good = np.isfinite(x) & ~spike
    if bad.any() and good.sum() >= 2:
        idx = np.arange(n)
        cleaned[bad] = np.interp(idx[bad], idx[good], x[good])
    return cleaned, int(spike.sum())


def loadcell_knee_torque(w):
    """Knee torque [N*m] from one wrench w = [Fx, Fy, Fz, Mx, My, Mz]:
    knee = Mx + knee_arm * Fy."""
    return KNEE_LOADCELL_TORQUE_SIGN * (w[3] + KNEE_ARM_M * w[1])


# ---------------------------------------------------------------------------
# RUNTIME STATE -- counters and log buffers
# ---------------------------------------------------------------------------
NAN = float("nan")


@dataclass
class JointState:
    clamp_count: int = 0
    i2c_fail_count: int = 0
    i2c_fail_streak: int = 0
    i2c_fail_max_streak: int = 0
    i2c_last_good_deg: float = 0.0
    i2c_has_good_reading: bool = False
    wrap_reject_count: int = 0
    wrap_reject_streak: int = 0
    wrap_reject_max_streak: int = 0
    last_agc: int = -1
    p_prev_deg: float = 0.0    # last known-good actual angle (glitch-reject reuse)
    log: dict = field(default_factory=lambda: {k: [] for k in (
        "t", "dt", "reference_deg", "actual_deg", "error_deg", "p_term",
        "i_term", "d_term", "current_cmd", "current_clamped",
        "tau_equivalent", "torque_nm",
        "lc_sample_index", "lc_fy", "lc_mx", "lc_joint_torque")})


STATE = JointState()
MOTORS = {KNEE_CAN_ID: MOTOR_TYPE}

print(f"[startup] KNEE_CAN_ID = {KNEE_CAN_ID}   AS5048B addr 0x{AS5048B_ADDR:02X} on I2C bus {I2C_BUS}")
print(f"[startup] PID gains   = KP={KP} KI={KI} KD={KD} "
      f"({'D on measurement, ' + format(D_FILTER_HZ, 'g') + ' Hz LP' if D_ON_MEASUREMENT else 'D on error'})   "
      f"MAX_CURRENT_A={MAX_CURRENT_A:.1f}")
print(f"[startup] Knee limits = [{KNEE_LIMIT_MIN_DEG:+.1f}, {KNEE_LIMIT_MAX_DEG:+.1f}] deg   "
      f"ABORT_ERROR_DEG={ABORT_ERROR_DEG:.1f}")
print(f"[startup] Test        = home to {BASELINE_ANGLE_DEG:+.1f} deg, step to "
      f"{STEP_ANGLE_DEG:+.1f} deg, HOLD until Ctrl+C")
print(f"[startup] Torque      = commanded: {GEAR_RATIO_FACTOR} * {KT} * I   "
      f"load cell: Mx + {KNEE_ARM_M:g} * Fy (sign {KNEE_LOADCELL_TORQUE_SIGN:+.1f})")


# ---------------------------------------------------------------------------
# Tare and start the load cell BEFORE the motor is powered
# ---------------------------------------------------------------------------
loadcell_reader = LoadcellReader()
loadcell_reader.setup()
if loadcell_reader.setup_error is not None:
    print(f"  [loadcell warning] Load cell not available: {loadcell_reader.setup_error}\n"
          f"  [loadcell warning] Continuing WITHOUT the load cell.")
else:
    print("[loadcell] Load cell started (SPI1).")


# ---------------------------------------------------------------------------
# Open the CAN bus and the magnetic encoder's I2C bus
# ---------------------------------------------------------------------------
actuators = ActuatorGroup.from_dict(MOTORS, exit_manually=True)
print("[startup] CAN bus opened and knee motor registered with EPICally Powerful.")
mag_bus = SMBus(I2C_BUS)
print(f"[startup] I2C bus {I2C_BUS} opened for the AS5048B magnetic encoder.")


def lifecycle(code, can_id):
    """Send the motor's power-on (0xFC) or power-off (0xFD) message."""
    actuators.bus.send(
        can.Message(arbitration_id=can_id, data=[0xFF] * 7 + [code],
                    is_extended_id=True)
    )


def enforce_hard_limits(actual_deg, context):
    """Immediately halts the motor and raises if the corrected angle is
    outside [KNEE_LIMIT_MIN_DEG, KNEE_LIMIT_MAX_DEG] -- no tolerance."""
    if KNEE_LIMIT_MIN_DEG <= actual_deg <= KNEE_LIMIT_MAX_DEG:
        return
    print(f"\n  [HARD LIMIT VIOLATION] Knee: magnetic encoder reads "
          f"{actual_deg:+.2f} deg during {context}, outside "
          f"[{KNEE_LIMIT_MIN_DEG:+.1f}, {KNEE_LIMIT_MAX_DEG:+.1f}] deg. "
          f"Commanding zero current/velocity immediately and exiting.")
    try:
        actuators.set_torque(KNEE_CAN_ID, 0.0)
        actuators.set_velocity(KNEE_CAN_ID, 0.0, 0)
    except Exception:
        pass
    raise RuntimeError(
        f"Knee hard limit violated: {actual_deg:+.2f} deg is outside "
        f"[{KNEE_LIMIT_MIN_DEG:+.1f}, {KNEE_LIMIT_MAX_DEG:+.1f}] deg "
        f"during {context} -- this must never happen; aborting immediately."
    )


def home_joint(target_deg, bus, state):
    """Drive the joint to target_deg in Servo Velocity Loop Mode, direction
    chosen from the live error every tick; magnetic encoder alone decides
    arrival; then a burst of zero-velocity stop commands."""
    print(f"  [home] Knee: homing to {target_deg:+.2f} deg "
          f"(|HOMING_VELOCITY_RAD_S|={abs(HOMING_VELOCITY_RAD_S):.2f} rad/s)...")
    clock_h = TimedLoop(rate=HOME_LOOP_HZ)
    t_home0 = time.perf_counter()
    home_tick = 0
    HOME_PRINT_EVERY = 5
    while clock_h():
        now_deg = read_actual_angle_deg_safe(bus, state)
        enforce_hard_limits(now_deg, "homing")

        err = wrap_deg(target_deg - now_deg)

        if abs(err) <= HOME_TOLERANCE_DEG:
            print(f"  [home] Knee: reached {now_deg:+.2f} deg (target "
                  f"{target_deg:+.2f} deg, error {err:+.2f} deg).")
            break

        if time.perf_counter() - t_home0 > HOME_TIMEOUT_S:
            raise RuntimeError(
                f"Knee failed to home within {HOME_TIMEOUT_S:.1f} s "
                f"(still {abs(err):.2f} deg from {target_deg:+.2f} deg per "
                f"the magnetic encoder) -- aborting before the control "
                f"loop starts."
            )

        vel_cmd = ENCODER_TO_MOTOR_SIGN * math.copysign(abs(HOMING_VELOCITY_RAD_S), err)
        actuators.set_velocity(KNEE_CAN_ID, vel_cmd, 0)

        if home_tick % HOME_PRINT_EVERY == 0:
            print(f"  [home] Knee: t={time.perf_counter() - t_home0:5.2f}s  "
                  f"mag={now_deg:+7.2f} deg  err={err:+7.2f} deg  "
                  f"vel_cmd={vel_cmd:+.2f} rad/s")
        home_tick += 1

    print(f"  [home] Knee: sending {NUM_HOME_STOP_COMMANDS} zero-velocity stop commands...")
    for _ in range(NUM_HOME_STOP_COMMANDS):
        actuators.set_velocity(KNEE_CAN_ID, 0.0, 0)
        time.sleep(HOME_STOP_COMMAND_INTERVAL_S)
    final_deg = read_actual_angle_deg_safe(bus, state)
    enforce_hard_limits(final_deg, "post-homing stop")
    state.p_prev_deg = final_deg
    print(f"  [home] Knee: stopped at {final_deg:+.2f} deg (target {target_deg:+.2f} deg).")


try:
    for motor_id in MOTORS:
        lifecycle(0xFC, motor_id)
    print("[run] Sent motor wake-up (0xFC) frame. Waiting for fresh telemetry...")

    for motor_id in MOTORS:
        t_wait0 = time.perf_counter()
        ts0 = actuators.get_data(motor_id).timestamp
        fresh = 0
        while fresh < 5 and time.perf_counter() - t_wait0 < 1.0:
            ts = actuators.get_data(motor_id).timestamp
            if ts > ts0:
                fresh, ts0 = fresh + 1, ts
            time.sleep(0.002)
    for motor_id in MOTORS:
        lifecycle(0xFC, motor_id)   # re-assert wake, belt and braces
    time.sleep(0.02)
    print("[run] Motor awake and telemetry flowing.")

    print("\n[run] --- Phase 1: homing ---")
    home_joint(BASELINE_ANGLE_DEG, mag_bus, STATE)

    pid = PID(KP, KI, KD, INTEGRAL_LIMIT_DEG_S)
    reference_deg = STEP_ANGLE_DEG

    print(f"\n[run] --- Phase 2: step to {STEP_ANGLE_DEG:+.2f} deg, then HOLD "
          f"(KP={KP} KI={KI} KD={KD}, {LOOP_HZ} Hz) --- Ctrl+C stops.")
    print("      Logging starts once the knee has arrived and settled.\n")

    clock = TimedLoop(rate=LOOP_HZ)
    t_step = time.perf_counter()   # step instant (timeout reference)
    t0_log = None                  # set at arrival; t=0 for all logged data
    t_in_tol_since = None
    t_prev = t_step
    i = 0
    abort_reason = None
    PRINT_EVERY = max(1, LOOP_HZ // 5)

    while clock():
        actual_deg_raw = read_actual_angle_deg_safe(mag_bus, STATE)
        now_t = time.perf_counter()

        enforce_hard_limits(actual_deg_raw, "PID control loop")

        # Reject an implausible single-tick jump; reuse the last good reading.
        delta_deg = actual_deg_raw - STATE.p_prev_deg
        if abs(delta_deg) > GLITCH_JUMP_DEG:
            STATE.wrap_reject_count += 1
            STATE.wrap_reject_streak += 1
            STATE.wrap_reject_max_streak = max(
                STATE.wrap_reject_max_streak, STATE.wrap_reject_streak)
            if STATE.wrap_reject_streak > MAX_CONSECUTIVE_WRAP_REJECTS:
                abort_reason = (
                    f"Knee: {STATE.wrap_reject_streak} consecutive "
                    f"implausible position jump(s) "
                    f"(> {GLITCH_JUMP_DEG:.0f} deg/tick) -- the magnetic "
                    f"encoder reading is not trustworthy right now."
                )
            actual_deg_raw = STATE.p_prev_deg
        else:
            STATE.wrap_reject_streak = 0

        STATE.p_prev_deg = actual_deg_raw
        actual_deg = actual_deg_raw

        dt = now_t - t_prev
        if dt <= 0.0:
            dt = 1.0 / LOOP_HZ
        t_prev = now_t

        current_cmd, p_term, i_term, d_term, error_deg = pid.update(
            reference_deg, actual_deg, dt)

        current_cmd *= ENCODER_TO_MOTOR_SIGN
        p_term *= ENCODER_TO_MOTOR_SIGN
        i_term *= ENCODER_TO_MOTOR_SIGN
        d_term *= ENCODER_TO_MOTOR_SIGN

        current_clamped = current_cmd
        clamped_flag = False
        if abs(current_clamped) > MAX_CURRENT_A:
            current_clamped = math.copysign(MAX_CURRENT_A, current_clamped)
            clamped_flag = True
            STATE.clamp_count += 1

        actuators.set_torque(KNEE_CAN_ID, current_clamped)   # amperes

        if ABORT_ERROR_DEG > 0.0 and abs(error_deg) > ABORT_ERROR_DEG:
            abort_reason = (
                f"Knee tracking error reached {abs(error_deg):.1f} deg "
                f"(limit {ABORT_ERROR_DEG:.1f} deg)."
            )

        # --- Arrival detection: within tolerance for ARRIVE_SETTLE_S ------
        if t0_log is None:
            if abs(error_deg) <= ARRIVE_TOLERANCE_DEG:
                if t_in_tol_since is None:
                    t_in_tol_since = now_t
                elif now_t - t_in_tol_since >= ARRIVE_SETTLE_S:
                    t0_log = now_t
                    print(f"\n  [ARRIVED] Knee at {actual_deg:+.2f} deg "
                          f"(goal {STEP_ANGLE_DEG:+.2f}). Holding; logging "
                          f"loadcell + torques from NOW (t=0). Push on the "
                          f"knee; Ctrl+C when done.\n")
            else:
                t_in_tol_since = None
                if now_t - t_step > ARRIVE_TIMEOUT_S:
                    abort_reason = (
                        f"Knee did not reach {STEP_ANGLE_DEG:+.2f} deg "
                        f"within {ARRIVE_TIMEOUT_S:.1f} s of the step "
                        f"(error {error_deg:+.2f} deg).")

        # --- Load cell sample (non-blocking) -------------------------------
        lc = loadcell_reader.latest()
        if lc is None:
            lc_idx, lc_fy, lc_mx, lc_tq = -1, NAN, NAN, NAN
        else:
            w, lc_idx = lc
            lc_fy, lc_mx = float(w[1]), float(w[3])
            lc_tq = float(loadcell_knee_torque(w))

        tau_cmd_nm = current_clamped * KT * GEAR_RATIO_FACTOR

        if t0_log is not None:
            t_wall = now_t - t0_log
            STATE.log["t"].append(t_wall)
            STATE.log["dt"].append(dt)
            STATE.log["reference_deg"].append(reference_deg)
            STATE.log["actual_deg"].append(actual_deg)
            STATE.log["error_deg"].append(error_deg)
            STATE.log["p_term"].append(p_term)
            STATE.log["i_term"].append(i_term)
            STATE.log["d_term"].append(d_term)
            STATE.log["current_cmd"].append(current_clamped)
            STATE.log["current_clamped"].append(clamped_flag)
            STATE.log["tau_equivalent"].append(tau_cmd_nm)
            STATE.log["torque_nm"].append(KT * current_clamped)
            STATE.log["lc_sample_index"].append(lc_idx)
            STATE.log["lc_fy"].append(lc_fy)
            STATE.log["lc_mx"].append(lc_mx)
            STATE.log["lc_joint_torque"].append(lc_tq)
        else:
            t_wall = now_t - t_step

        if abort_reason is not None:
            print(f"\n  [ABORT] {abort_reason}")
            print("  Stopping the test now. The CSV/plots below still show "
                  "everything logged up to this point.")
            break

        if i % PRINT_EVERY == 0:
            try:
                agc, _diag = _as5048b_read_health(mag_bus)
                STATE.last_agc = agc
            except OSError:
                pass
            agc_str = f"{STATE.last_agc}" if STATE.last_agc >= 0 else "?"
            clamp_str = " [CLAMPED]" if clamped_flag else ""
            phase = "HOLD" if t0_log is not None else "STEP"
            lc_str = (f"Fy={lc_fy:+7.2f} N  Mx={lc_mx:+6.2f}  LCtq={lc_tq:+6.2f}"
                      if lc is not None else "LC=n/a")
            print(f"[{phase}] t={t_wall:6.2f}s  ref={reference_deg:+6.2f}  "
                  f"act={actual_deg:+7.2f}  err={error_deg:+6.2f}  "
                  f"I_cmd={current_clamped:+5.2f} A{clamp_str}  "
                  f"cmdTq={tau_cmd_nm:+6.2f} N*m  {lc_str}  AGC={agc_str}")
        i += 1

        if getattr(actuators.notifier, "exception", None):
            print("RX thread died:", actuators.notifier.exception)
            break

    print("\n[run] Test finished.")

except KeyboardInterrupt:
    print("\n[run] Ctrl+C -- stopping.")

finally:
    print("\n[shutdown] Stopping and releasing the motor...")
    for _ in range(10):
        for motor_id in MOTORS:
            actuators.set_torque(motor_id, 0.0)
            actuators.set_velocity(motor_id, 0.0, 0)
        time.sleep(0.005)
    for motor_id in MOTORS:
        lifecycle(0xFD, motor_id)
    try:
        actuators.disable_actuators()
        actuators.notifier.stop()
        actuators.bus.shutdown()
    except Exception:
        pass
    try:
        mag_bus.close()
    except Exception:
        pass
    try:
        loadcell_reader.stop()
    except Exception:
        pass
    print("[shutdown] Motor limp; verify by hand before approaching.")

    n_ticks = len(STATE.log["t"])
    print(f"[shutdown] Logged {n_ticks} control tick(s) (from arrival onward).")
    if n_ticks > 1:
        elapsed = STATE.log["t"][-1] - STATE.log["t"][0]
        achieved = (n_ticks - 1) / elapsed if elapsed > 0 else float("nan")
        print(f"[shutdown] Achieved loop rate: {achieved:.1f} Hz "
              f"(nominal {LOOP_HZ} Hz, {100.0 * achieved / LOOP_HZ:.1f}%)")

    if STATE.clamp_count > 0:
        print(f"  [warning] Knee: current clamp engaged on {STATE.clamp_count} tick(s).")
    if STATE.i2c_fail_count > 0:
        print(f"  [warning] Knee: AS5048B I2C read failed {STATE.i2c_fail_count} time(s) "
              f"(worst streak: {STATE.i2c_fail_max_streak}).")
    if STATE.wrap_reject_count > 0:
        print(f"  [warning] Knee: rejected {STATE.wrap_reject_count} implausible position "
              f"jump(s) (worst streak: {STATE.wrap_reject_max_streak}).")

    # --- Load cell report -------------------------------------------------
    if loadcell_reader.setup_error is not None:
        print(f"  [loadcell] NOT used this run: {loadcell_reader.setup_error}")
    else:
        n_missing = sum(1 for v in STATE.log["lc_joint_torque"] if math.isnan(v))
        print(f"[loadcell] {loadcell_reader.n_samples} samples read, "
              f"{loadcell_reader.n_errors} read error(s), "
              f"{n_missing}/{n_ticks} logged tick(s) had no fresh sample "
              f"(tared={loadcell_reader.tared}).")
        if loadcell_reader.last_error:
            print(f"  [loadcell warning] last error: {loadcell_reader.last_error}")

    # --- CSV export (always written) --------------------------------------
    if n_ticks == 0:
        print("[shutdown] The knee never arrived at the goal / nothing was "
              "logged -- no CSV or plots were produced.")
    else:
        # Mean line of the commanded torque (= GEAR*KT*current, so it is also
        # the mean of the commanded current, scaled): centered moving average.
        STATE.log["tau_mean"] = centered_moving_mean(
            STATE.log["tau_equivalent"], STATE.log["t"], MEAN_WINDOW_S).tolist()

        csv_columns = ["t", "dt", "reference_deg", "actual_deg", "error_deg",
                        "p_term", "i_term", "d_term", "current_cmd",
                        "current_clamped", "tau_equivalent", "tau_mean",
                        "torque_nm",
                        "lc_sample_index", "lc_fy", "lc_mx", "lc_joint_torque"]
        csv_header = ["time_s", "dt_s", "reference_deg", "actual_deg",
                       "error_deg", "p_term_A", "i_term_A", "d_term_A",
                       "current_cmd_A", "current_clamped",
                       "commanded_torque_Nm", "commanded_torque_mean_Nm",
                       "torque_Nm",
                       "lc_sample_index", "lc_Fy_N", "lc_Mx_Nm",
                       "lc_knee_torque_Nm"]
        csv_path = os.path.join(RUN_DIR, "resistance_torque_data.csv")
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(csv_header)
            for row in zip(*(
                    [DISPLAY_SIGN * v for v in STATE.log[col]]
                    if col in ("reference_deg", "actual_deg")
                    else STATE.log[col] for col in csv_columns)):
                writer.writerow(row)
        print(f"[shutdown] CSV data saved to: {csv_path}")

        if not GENERATE_PLOTS:
            print("[shutdown] Plot generation disabled (GENERATE_PLOTS = False).")
        else:
            d = STATE.log
            s = DISPLAY_SIGN
            t = d["t"]

            # --- Main plot: commanded (PID current) vs. measured (load cell)
            # knee torque, together in one figure. -------------------------
            fig, ax = plt.subplots(1, 1, figsize=(11, 6))
            ax.plot(t, d["tau_equivalent"], color="blue", linewidth=1.8,
                    label=f"Commanded torque ({GEAR_RATIO_FACTOR} * k_t * PID current)")
            ax.plot(t, d["lc_joint_torque"], color="red", linewidth=1.8,
                    label="Measured torque (load cell: Mx + arm*Fy)")
            ax.axhline(0.0, color="black", linewidth=0.8, linestyle="--", alpha=0.5)
            ax.set_xlabel("Time [s]", fontsize=12)
            ax.set_ylabel("Knee joint torque [N*m]", fontsize=12)
            ax.set_title(f"Knee resistance test: commanded vs. measured torque "
                         f"(hold at {s * STEP_ANGLE_DEG:+.1f} deg)", fontsize=13)
            ax.legend(loc="best", fontsize=11)
            ax.grid(True, alpha=0.3)
            fig.tight_layout()
            p = os.path.join(RUN_DIR, "torque_commanded_vs_measured.png")
            fig.savefig(p, dpi=300, bbox_inches="tight")
            print(f"[shutdown] Plot saved to: {p}")

            # --- Extra plot: light-grey commanded torque, spike-free measured
            # torque (red), and the computed mean line of the command (green).
            measured_clean, n_spikes = remove_spikes(
                d["lc_joint_torque"], t, SPIKE_WINDOW_S,
                SPIKE_MAD_THRESHOLD, SPIKE_MIN_DEV_NM)
            print(f"[shutdown] Ignored {n_spikes} measured-torque spike(s) "
                  f"as sensor noise (linearly interpolated) in the smoothed comparison plot.")
            fig, ax = plt.subplots(1, 1, figsize=(11, 6))
            ax.plot(t, d["tau_equivalent"], color="lightgrey", linewidth=1.2,
                    label=f"Commanded torque ({GEAR_RATIO_FACTOR} * k_t * PID current)")
            ax.plot(t, measured_clean, color="red", linewidth=1.5,
                    label="Measured torque (load cell, spikes removed)")
            ax.plot(t, d["tau_mean"], color="green", linewidth=2.5,
                    label=f"Commanded torque mean ({MEAN_WINDOW_S:g} s moving average)")
            ax.axhline(0.0, color="black", linewidth=0.8, linestyle="--", alpha=0.5)
            ax.set_xlabel("Time [s]", fontsize=12)
            ax.set_ylabel("Knee joint torque [N*m]", fontsize=12)
            ax.set_title(f"Knee resistance test: commanded mean vs. measured torque "
                         f"(hold at {s * STEP_ANGLE_DEG:+.1f} deg)", fontsize=13)
            ax.legend(loc="best", fontsize=11)
            ax.grid(True, alpha=0.3)
            fig.tight_layout()
            p = os.path.join(RUN_DIR, "torque_commanded_mean_vs_measured.png")
            fig.savefig(p, dpi=300, bbox_inches="tight")
            print(f"[shutdown] Plot saved to: {p}")

            def plot_single_panel(traces, ylabel, title, filename, ref_line=None):
                fig, ax = plt.subplots(1, 1, figsize=(10, 5))
                for series, color, label in traces:
                    ax.plot(t, series, color=color, linewidth=1.5, label=label)
                if ref_line is not None:
                    ax.axhline(ref_line, color="black", linewidth=0.8,
                               linestyle="--", alpha=0.5)
                ax.set_ylabel(ylabel)
                ax.set_xlabel("Time [s]")
                ax.set_title(title)
                ax.legend(loc="best")
                ax.grid(True, alpha=0.3)
                fig.tight_layout()
                out = os.path.join(RUN_DIR, filename)
                fig.savefig(out, dpi=300, bbox_inches="tight")
                print(f"[shutdown] Plot saved to: {out}")

            plot_single_panel(
                [([s * v for v in d["reference_deg"]], "blue", "Reference (goal)"),
                 ([s * v for v in d["actual_deg"]], "red", "Actual angle (magnetic encoder)")],
                "Knee joint angle [deg]", "Knee angle while held / pushed",
                "position.png")
            plot_single_panel(
                [(d["current_cmd"], "darkorange", "Commanded current")],
                "current [A]", "PID output current", "current_command.png",
                ref_line=0.0)
            plot_single_panel(
                [(d["lc_fy"], "teal", "Fy [N]"), (d["lc_mx"], "purple", "Mx [N*m]")],
                "load cell reading", "Load cell Fy and Mx", "loadcell_fy_mx.png",
                ref_line=0.0)

            if SHOW_PLOTS:
                try:
                    plt.show()
                except Exception:
                    pass

print("\nDone.")