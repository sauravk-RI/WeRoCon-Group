#!/usr/bin/env python3
"""
ankle_bode_noise_PID_control_me.py -- OSL-style Gaussian-noise Bode test of the
ANKLE position loop. "_me" = Magnetic Encoder (the AS5048B, not the motor's own
internal one).

WHAT IS MEASURED
Closed-loop T(f) = measured ankle angle (AS5048B) / reference angle of the SAME
Pi-side loop the gait scripts run: Python PID at LOOP_HZ -> current command over
CAN -> the driver's Servo CURRENT LOOP (actuators.set_torque(), amperes) ->
motor, transmission, joint. The PID, encoder code, sign handling, 60 A clamp,
homing, hold, hard limits and guards are those of
ankle_bode_sweep_PID_control_me.py. The D TERM IS NOT: it is taken on the MEASURED
angle through a 100 Hz low-pass (D_ON_MEASUREMENT, D_FILTER_HZ), the same change as in
knee_bode_noise_PID_control_me.py. Gains and D settings are written to the CSV
headers. The papers do not state their gains and their position loop ran inside
the Dephy drive, so the numbers are not directly comparable with theirs.

THE TEST (Azocar et al. 2018 and 2020)
The reference is Gaussian white noise through a causal 3rd-order Butterworth
low-pass (REF_LP_HZ), zero mean, scaled so the peak of the 15 s RECORD window is
+/-NOISE_PEAK_DEG about CENTER_DEG. Per trial:

    fade-in 1 s | RECORD 15 s | fade-out 1 s | hold 1 s      (18 s)

with a 1 s PID-held pre-roll before trial 1; the PID is never reset between
trials. Trial k uses seed SEED_BASE + k. Every trial's reference is generated
once, BEFORE the motor wakes (bode_noise_tools.build_run_arrays), and the 1 kHz
loop only looks it up; the value actually sent is logged every tick. T(f) =
S_xy / S_xx by Blackman-Tukey spectral analysis with the cross-spectra summed
over the complete trials; the -3 dB point is the bandwidth.

WHAT TO EXPECT (differs from the stepped-sine result)
  - A peak of +/-5 deg gives an rms of only about 1.5 deg (a +/-5 deg sine has
    3.5 deg rms). Coulomb friction matters more for small motions, so the
    low-frequency gain can droop more than in the sine result. That is a real
    effect, not a bug.
  - Valid frequencies are limited by coherence. Below about F_LO_HZ = 0.3 Hz the
    4 s lag window cannot resolve the spectrum; those bins are not reported.
  - The input spectrum rolls off above the low-pass corner, so coherence falls
    there. Bins with coherence < COH_MIN are not written.

ANKLE SETUP -- a Bode plot belongs to its setup
  - Clamp the leg / unit as in your gait runs (same pylon / shank / foot, same
    orientation to gravity); the foot must touch nothing. Put what is attached in
    SETUP_NOTE below (or --note).
  - The KNEE motor is never touched by this script (it is not even on the CAN
    bus here). BLOCK the knee mechanically (strap or pin) or keep it separated,
    otherwise it swings and feeds motion back into the ankle.
  - Centre 0 deg (the encoder's mean/neutral position; plantarflexion positive,
    dorsiflexion negative); hard limits +/-29 deg.
  - Start every trial from a fresh homing (every run does: Phase 1).
  - Keep a hand on the E-stop for the first run of each joint.

RUN ORDER
  1. python bode_noise_tools.py selftest                 (offline, no hardware)
  2. python ankle_bode_noise_PID_control_me.py --dry-run
        Motor never woken (CAN not opened), nothing moves; the reference is
        generated and logged, the encoder is read, the analysis runs.
        (Tracking and swing guards are off in this mode.)
  3. python ankle_bode_noise_PID_control_me.py --pilot     (one trial, E-stop in hand)
        Acceptance: measured peak |current| within about 30 % of the pre-flight
        prediction; record-window rms tracking error a few tenths of a degree to
        about 2 deg; Bode curve starts near 0 dB (it may droop from friction).
  4. python ankle_bode_noise_PID_control_me.py             (5 trials, 40 Hz low-pass)
  5. python ankle_bode_noise_PID_control_me.py --lp 40     (the papers' corner, only
        after the pilot looks calm), then --amp 10 and --amp 15 if the 5 deg
        runs behave.
  6. python bode_noise_tools.py compare <noise run> <sweep run>

PRE-FLIGHT (before anything can move)
  1. centre +/- (peak + GUARD_MARGIN_DEG) must lie inside the hard limits.
  2. Per trial the reference rms, peak rate and the PREDICTED peak PID current
     (the PID law on e = r; an upper-bound estimate, not a measurement; real
     current is higher because of encoder quantisation noise in the D term) are
     printed. The run is refused if any trial exceeds PREFLIGHT_MAX_PRED_A.
  3. The planned duration is printed.

GUARDS (every tick): hard joint limits (no tolerance); tracking error
|r - y| > NOISE_PEAK_DEG + GUARD_MARGIN_DEG; swing |y - centre| > the same;
glitch-jump and I2C-failure guards (events counted); driver temperature above
MAX_DRIVER_TEMP_C (also refuses to start); the 60 A clamp (events counted).

STOPPING EARLY
Ctrl+C (or SIGTERM / SIGHUP) during the PID phase only sets a flag; the loop
stops at the next tick, the motor is released FIRST, then the data CSV is
written, then the complete trials are analysed and plotted. A trial interrupted
inside its record window is dropped (its samples stay in the data CSV). With no
complete trial you still get the data CSV, a header-only summary and fig2. A
signal during shutdown only prints a message; once the CSV is written the default
behaviour returns. Re-analyse any run with
    python bode_noise_tools.py analyse <run folder>

OUTPUT -- results/bode_noise_ankle/<mode>_<N>/   (mode = run / pilot / dryrun)
  bode_noise_data.csv     time_s, trial (0 = pre-roll), reference_deg (sent),
                          actual_deg (glitch-rejected, into the PID), in_window,
                          and with LOG_CURRENT_A a last column current_cmd_A
  bode_noise_summary.csv  f_hz, magnitude_dB, phase_deg, coherence
  fig1_ref_vs_actual_by_trial.png, fig2_ref_vs_actual_whole_run.png,
  fig3_bode.png, fig4_coherence.png
Both CSVs begin with '# key: value' lines; load with pd.read_csv(path,
comment="#") or MATLAB readtable(path,'CommentStyle','#') (not tested).
Angles are in the CONTROL (encoder) frame: plantarflexion positive, centre 0, the same frame as the ankle step script.
"""

# ===========================================================================
#  USER SETTINGS -- the fields you are most likely to want to change
# ===========================================================================

# --- CAN ID ------------------------------------------------------------
ANKLE_CAN_ID = 1     # CAN ID of the ankle actuator (same as the step test)
# --- Noise test --------------------------------------------------------------
CENTER_DEG = 0.0           # ankle centre, encoder frame (neutral; plantarflexion +, dorsiflexion -)
NOISE_PEAK_DEG = 5.0       # record-window peak about the centre (--amp 10, --amp 15)
REF_LP_HZ = 20.0           # low-pass corner (--lp 40 for the papers' value)
REF_LP_ORDER = 3
REC_S = 15.0               # record window per trial
FADE_S = 1.0               # raised-cosine fade-in and fade-out
HOLD_S = 1.0               # hold at the centre after each trial
PREROLL_S = 0.0            # PID-held settle at the centre before trial 1 (trial 0)
N_TRIALS = 5               # --trials N, --pilot = 1
SEED_BASE = 20260001       # trial k uses SEED_BASE + k (--seed N)
PRE_HOLD_S = 0.5           # velocity-mode hold at the centre after homing,
                           # before the PID engages (not logged)

# --- Analysis (written to the CSV headers, used by bode_noise_tools) -------------
COH_MIN = 0.9              # bins with lower ensemble coherence are not reported
BT_MAX_LAG_S = 4.0         # Blackman-Tukey lag window (resolution about 0.19 Hz)
F_LO_HZ = 0.3              # lowest reported frequency
PREFLIGHT_MAX_PRED_A = 1000.0   # refuse to start if a trial's predicted peak current exceeds this

# --- PID gains (frozen for this test) -----------------------------------------

KP = 4.5
KI = 0.0
KD = 0.085
INTEGRAL_LIMIT_DEG_S = 50.0   # anti-windup clamp on the accumulated integral
# Derivative on the MEASURED angle (not on the error), low-passed:
#     u = Kp*e + Ki*integral(e) - Kd * LP( dy/dt ),   e = r - y
# i.e. the classic law Kp*e + Ki*int(e) + Kd*(dr/dt - dy/dt) with the Kd*dr/dt (reference
# velocity feedforward) term dropped. Feedback damping, stability and disturbance rejection
# are unchanged; only the reference -> output path loses that lead, and the derivative of the
# 40 Hz-limited noise reference is no longer sent into the current command.
# Set D_ON_MEASUREMENT = False for the old derivative-on-error behaviour (no filter then).
# NOTE: the ankle gains above were tuned with derivative-on-error; re-tune them for this form.
D_ON_MEASUREMENT = True
D_FILTER_HZ = 100.0           # first-order low-pass on the derivative term (D_ON_MEASUREMENT only)

# --- Current clamp -----------------------------------------------------
MAX_CURRENT_A = 60.0    # the applied q-axis current limit from the motor manual

# --- Control-loop rate ---------------------------------------------------
LOOP_HZ = 1000

# --- Joint hardware limits (OSL v2 mechanical range), degrees --------------
# Strictly, unconditionally enforced every tick -- see enforce_hard_limits().
ANKLE_LIMIT_MIN_DEG = -30.0
ANKLE_LIMIT_MAX_DEG = 30.0
# --- Test guards -----------------------------------------------------------
GUARD_MARGIN_DEG = 10.0                          # added to the peak for the two guards below
MAX_DRIVER_TEMP_C = 60.0    # abort (and refuse to start) above this
REQUIRE_CONFIRMATION = False # ask for Enter before waking the motor (tty only)

# --- Homing (velocity-mode-to-target), identical to the step test -----------
HOMING_VELOCITY_RAD_S = 0.4
HOME_LOOP_HZ = 1000
HOME_TOLERANCE_DEG = 0.1
HOME_TIMEOUT_S = 30.0
NUM_HOME_STOP_COMMANDS = 15
HOME_STOP_COMMAND_INTERVAL_S = 0.001

# --- Implausible-jump guard ----------------------------------------------
GLITCH_JUMP_DEG = 15.0
MAX_CONSECUTIVE_WRAP_REJECTS = 10

# --- Magnetic encoder (AS5048B) settings -----------------------------------
I2C_BUS = 1
AS5048B_ADDR = 0x41
ANGLE_HIGH_REG = 0xFE
ANGLE_LOW_REG = 0xFF
MAGNETIC_ENCODER_OFFSET = 200.0   # deg; actual_deg = wrap_deg(raw - OFFSET)
ENCODER_TO_MOTOR_SIGN = -1.0      # verified value; wrong sign = positive feedback
MAX_CONSECUTIVE_I2C_FAILS = 25
# --- Logging -----------------------------------------------------------------
LOG_CURRENT_A = True       # add a last column current_cmd_A to the data CSV

# --- Run metadata (written to the CSV headers) --------------------------------
SETUP_NOTE = ("This test is conducted with the Ankle Unit hanging from a Bench using a G-Clamp and Pylon."
"The foot bottom is facing the ground. The Knee unit is separated and only Ankle unit is mounted in the Bench-top Mode.")
# --- Plotting -------------------------------------------------------------
GENERATE_PLOTS = True
SHOW_PLOTS = True

# ===========================================================================
#  END OF USER SETTINGS
# ===========================================================================

import argparse
import datetime
import math
import os
import signal
import sys
import time
from dataclasses import dataclass

_ap = argparse.ArgumentParser(description="Ankle Gaussian-noise Bode test (see the module docstring).")
_ap.add_argument("--dry-run", action="store_true",
                 help="never wake the motor (CAN bus not opened); generate, log and analyse only")
_ap.add_argument("--pilot", action="store_true", help="one trial instead of N_TRIALS")
_ap.add_argument("--trials", type=int, metavar="N", help="number of trials (default N_TRIALS)")
_ap.add_argument("--amp", type=float, metavar="DEG", help="record-window peak in degrees (default NOISE_PEAK_DEG)")
_ap.add_argument("--lp", type=float, metavar="HZ", help="reference low-pass corner (default REF_LP_HZ)")
_ap.add_argument("--seed", type=int, metavar="N", help="SEED_BASE; trial k uses N + k")
_ap.add_argument("--trial", type=int, help="trial label to record in the header (default: the run number)")
_ap.add_argument("--note", type=str, help="override SETUP_NOTE")
ARGS = _ap.parse_args()

DRY_RUN = ARGS.dry_run
if ARGS.amp is not None:
    NOISE_PEAK_DEG = ARGS.amp
if ARGS.lp is not None:
    REF_LP_HZ = ARGS.lp
if ARGS.seed is not None:
    SEED_BASE = ARGS.seed
if ARGS.trials is not None:
    N_TRIALS = ARGS.trials
if ARGS.pilot:
    N_TRIALS = 1
if ARGS.note:
    SETUP_NOTE = ARGS.note
if N_TRIALS < 1 or NOISE_PEAK_DEG <= 0 or REF_LP_HZ <= 0:
    raise SystemExit("--trials must be >= 1 and --amp, --lp must be positive.")
MODE = "dryrun" if DRY_RUN else ("pilot" if ARGS.pilot else "run")
ABORT_ERROR_DEG = NOISE_PEAK_DEG + GUARD_MARGIN_DEG    # tracking-error guard
ABORT_SWING_DEG = NOISE_PEAK_DEG + GUARD_MARGIN_DEG    # runaway guard: |actual - centre|

import numpy as np
import pandas as pd

import can
from smbus2 import SMBus

import matplotlib
if SHOW_PLOTS and os.environ.get("DISPLAY"):
    try:
        matplotlib.use("TkAgg")
    except Exception:
        matplotlib.use("Agg")
else:
    matplotlib.use("Agg")          # safe for headless / SSH sessions

from epicallypowerful.actuation import ActuatorGroup
from epicallypowerful.toolbox import TimedLoop

import bode_noise_tools as bnt

# epicallypowerful.actuation.actuator_group unconditionally attaches a
# FileHandler('motorlog.log') to the 'motorlog' logger at import time
# above; detach and remove it so it stops appearing.
import logging as _logging
_motorlog = _logging.getLogger('motorlog')
for _h in _motorlog.handlers[:]:
    _motorlog.removeHandler(_h)
    _h.close()
if os.path.exists('motorlog.log'):
    os.remove('motorlog.log')

MOTOR_TYPE = 'AK80-9-V3-servo'   # Servo operating mode
JOINT_NAME = "ankle"

print("=" * 78)
print("ankle_bode_noise_PID_control_me.py" + ("   [DRY RUN -- motor never woken]" if DRY_RUN else ""))
print("=" * 78)

# --- Pre-flight 1: hard limits (before anything can move) --------------------------
_reach = NOISE_PEAK_DEG + GUARD_MARGIN_DEG
if not (ANKLE_LIMIT_MIN_DEG <= CENTER_DEG - _reach and CENTER_DEG + _reach <= ANKLE_LIMIT_MAX_DEG):
    raise SystemExit(
        f"Refusing to start: centre {CENTER_DEG:+.1f} deg +/- (peak {NOISE_PEAK_DEG:g} + GUARD_MARGIN_DEG "
        f"{GUARD_MARGIN_DEG:g}) = +/-{_reach:g} deg crosses the ankle's hard limits "
        f"[{ANKLE_LIMIT_MIN_DEG:+.1f}, {ANKLE_LIMIT_MAX_DEG:+.1f}] deg. Lower --amp or GUARD_MARGIN_DEG.")
print(f"[startup] Pre-flight 1 passed: centre {CENTER_DEG:+.1f} +/- {_reach:g} deg is inside "
      f"[{ANKLE_LIMIT_MIN_DEG:+.1f}, {ANKLE_LIMIT_MAX_DEG:+.1f}] deg.")

# --- Reference for the whole run, generated ONCE before the motor wakes -------------------------
REF_OFF, TRIAL_ARR, WIN_ARR = bnt.build_run_arrays(N_TRIALS, REF_LP_HZ, NOISE_PEAK_DEG, SEED_BASE,
                                                   REC_S, FADE_S, HOLD_S, PREROLL_S, REF_LP_ORDER, bnt.FS)
REF_LIST = (CENTER_DEG + REF_OFF).tolist()        # python floats: the loop only indexes these
TRIAL_LIST = TRIAL_ARR.tolist()
WIN_LIST = WIN_ARR.tolist()
N_REF = len(REF_LIST)
PLANNED_S = N_REF / bnt.FS

# --- Pre-flight 2: predicted current per trial ----------------------------------------------------
print(f"[startup] Reference: {N_TRIALS} trial(s), seeds {SEED_BASE + 1}..{SEED_BASE + N_TRIALS}, "
      f"3rd-order LP {REF_LP_HZ:g} Hz, peak +/-{NOISE_PEAK_DEG:g} deg")
_too_high = []
for _k in range(1, N_TRIALS + 1):
    _sel = TRIAL_ARR == _k
    _r = REF_OFF[_sel]
    _w = WIN_ARR[_sel] == 1
    _rms = float(np.sqrt(np.mean(_r[_w] ** 2)))
    _rate = float(np.abs(np.diff(_r)).max() * bnt.FS)
    _pred = bnt.predicted_peak_current(_r, KP, KI, KD, INTEGRAL_LIMIT_DEG_S)
    print(f"             trial {_k}: ref rms {_rms:.2f} deg, peak rate {_rate:5.0f} deg/s, "
          f"predicted peak PID current {_pred:5.1f} A")
    if _pred > PREFLIGHT_MAX_PRED_A:
        _too_high.append((_k, _pred))
if _too_high:
    raise SystemExit(
        "Refusing to start: predicted peak current " + ", ".join(f"trial {k}: {p:.1f} A" for k, p in _too_high)
        + f" exceeds PREFLIGHT_MAX_PRED_A = {PREFLIGHT_MAX_PRED_A:g} A. Lower --lp or --amp.")
print(f"[startup] Pre-flight 2 passed (all predictions <= {PREFLIGHT_MAX_PRED_A:g} A; an upper-bound estimate, "
      f"real current is higher because of encoder noise in the D term).")
print(f"[startup] Planned duration          = {PLANNED_S:.1f} s of PID time ({PREROLL_S:g} s pre-roll + "
      f"{N_TRIALS} x {(2 * FADE_S + REC_S + HOLD_S):g} s), plus homing")

# --- Results directory: results/bode_noise_ankle/<mode>_<N>/ -----------------------------------------
_script_dir = os.path.dirname(os.path.abspath(__file__))
_results_root = os.path.join(_script_dir, "results", f"bode_noise_{JOINT_NAME}")
os.makedirs(_results_root, exist_ok=True)
_existing = [int(n[len(MODE) + 1:]) for n in os.listdir(_results_root)
             if n.startswith(MODE + "_") and n[len(MODE) + 1:].isdigit()]
RUN_NUMBER = max(_existing, default=0) + 1
RUN_DIR = os.path.join(_results_root, f"{MODE}_{RUN_NUMBER}")
os.makedirs(RUN_DIR)
print(f"[startup] This is {MODE} #{RUN_NUMBER}. Results will be saved to: {RUN_DIR}")


# ---------------------------------------------------------------------------
# PID CONTROLLER -- same P/I as ankle_step_test_PID_control_me.py; the D term
# differs (see below), as in knee_bode_noise_PID_control_me.py
# ---------------------------------------------------------------------------
class PID:
    """Discrete PID, computed here in Python:

        e[k]  = r[k] - y[k]
        P     = Kp * e[k]
        I    += Ki * e[k] * Ts            (forward/rectangular sum, clamped)
        D     = -Kd * LP( (y[k] - y[k-1]) / Ts )   if D_ON_MEASUREMENT
                 Kd * (e[k] - e[k-1]) / Ts         otherwise (backward Euler on the ERROR)

    LP is a first-order low-pass at D_FILTER_HZ (exponential, uses the actual tick dt).
    The reference is faded in and out, so there is no step kick here.
    """

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
# MAGNETIC ENCODER (AS5048B) SUPPORT -- identical to the step test
# ------------------------------------------------------------------

def _as5048b_read_angle_raw(bus):
    """Raw single-turn angle in degrees, [0, 360), both bytes in one I2C
    transaction."""
    high_byte, low_byte = bus.read_i2c_block_data(AS5048B_ADDR,
                                                    ANGLE_HIGH_REG, 2)
    raw14 = ((high_byte << 6) | (low_byte & 0x3F)) & 0x3FFF
    return (raw14 / 16384.0) * 360.0


def wrap_deg(angle_deg):
    """Wrap an angle in degrees to (-180, 180]."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


def read_actual_angle_deg(bus):
    """actual_angle = raw reading - MAGNETIC_ENCODER_OFFSET, wrapped."""
    raw_deg = _as5048b_read_angle_raw(bus)
    return wrap_deg(raw_deg - MAGNETIC_ENCODER_OFFSET)


def read_actual_angle_deg_safe(bus, state):
    """Same, but a failed I2C read reuses the last known-good angle for
    that tick. Raises if too many reads fail in a row, or if no good
    reading has ever been obtained."""
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
        print(f"  [warning] Ankle: AS5048B I2C read failed ({e}) -- "
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
                f"Ankle: {state.i2c_fail_streak} consecutive AS5048B I2C "
                f"read failure(s) (last error: {e}) -- {reason}"
            ) from e
        return state.i2c_last_good_deg


# ---------------------------------------------------------------------------
# RUNTIME STATE
# ---------------------------------------------------------------------------
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
    p_prev_deg: float = 0.0    # last known-good actual angle (glitch-reject reuse)

STATE = JointState()
MOTORS = {ANKLE_CAN_ID: MOTOR_TYPE}

print(f"[startup] ANKLE_CAN_ID               = {ANKLE_CAN_ID}")
print(f"[startup] Magnetic encoder          = AS5048B on I2C bus {I2C_BUS}, "
      f"addr 0x{AS5048B_ADDR:02X}, offset {MAGNETIC_ENCODER_OFFSET:+.2f} deg, "
      f"ENCODER_TO_MOTOR_SIGN {ENCODER_TO_MOTOR_SIGN:+.1f}")
print(f"[startup] PID gains                 = KP={KP} A/deg  KI={KI} A/(deg*s)  "
      f"KD={KD} A/(deg/s) ({'D on measurement, ' + format(D_FILTER_HZ, 'g') + ' Hz LP' if D_ON_MEASUREMENT else 'D on error'})   integral clamp +/-{INTEGRAL_LIMIT_DEG_S:.1f} deg*s")
print(f"[startup] MAX_CURRENT_A             = {MAX_CURRENT_A:.1f} A")
print(f"[startup] Ankle hard limits          = [{ANKLE_LIMIT_MIN_DEG:+.1f}, "
      f"{ANKLE_LIMIT_MAX_DEG:+.1f}] deg (unconditional, no margin)")
print(f"[startup] Guards                    = tracking error > {ABORT_ERROR_DEG:.1f} deg, "
      f"swing > {ABORT_SWING_DEG:.1f} deg from centre, driver temp > {MAX_DRIVER_TEMP_C:.0f} C")
print(f"[startup] LOOP_HZ                   = {LOOP_HZ} Hz")
print(f"[startup] Attached                  = {SETUP_NOTE}")
print("[startup] REMINDER: knee blocked mechanically? E-stop in hand?" if not DRY_RUN else
      "[startup] DRY RUN: nothing will be sent to the motor.")


# ---------------------------------------------------------------------------
# Open the CAN bus (not in a dry run) and the magnetic encoder's I2C bus
# ---------------------------------------------------------------------------
if DRY_RUN:
    actuators = None
    print("[startup] Dry run: CAN bus NOT opened, motor never woken.")
else:
    actuators = ActuatorGroup.from_dict(MOTORS, exit_manually=True)
    print("[startup] CAN bus opened and ankle motor registered with EPICally Powerful.")
mag_bus = SMBus(I2C_BUS)
print(f"[startup] I2C bus {I2C_BUS} opened for the AS5048B magnetic encoder.")


def lifecycle(code, can_id):
    """Send the motor's power-on (0xFC) or power-off (0xFD) message."""
    actuators.bus.send(
        can.Message(arbitration_id=can_id, data=[0xFF] * 7 + [code],
                    is_extended_id=True)
    )


def enforce_hard_limits(actual_deg, context):
    """Immediately halts the motor and raises if the magnetic encoder's
    corrected angle is outside [ANKLE_LIMIT_MIN_DEG, ANKLE_LIMIT_MAX_DEG]
    -- no margin, no tolerance, no exceptions. Called every single tick,
    both during homing and during the PID loop, as the FIRST thing done
    with a freshly read actual_deg, before any new command is computed or
    sent. `context` is a short string identifying the calling phase."""
    if  ANKLE_LIMIT_MIN_DEG <= actual_deg <= ANKLE_LIMIT_MAX_DEG:
        return
    print(f"\n  [HARD LIMIT VIOLATION] Ankle: magnetic encoder reads "
          f"{actual_deg:+.2f} deg during {context}, outside "
          f"[{ANKLE_LIMIT_MIN_DEG:+.1f}, {ANKLE_LIMIT_MAX_DEG:+.1f}] deg. "
          f"Commanding zero current/velocity immediately and exiting.")
    try:
        if actuators is not None:
            actuators.set_torque(ANKLE_CAN_ID, 0.0)
            actuators.set_velocity(ANKLE_CAN_ID, 0.0, 0)
    except Exception:
        pass
    raise RuntimeError(
        f"Ankle hard limit violated: {actual_deg:+.2f} deg is outside "
        f"[{ANKLE_LIMIT_MIN_DEG:+.1f}, {ANKLE_LIMIT_MAX_DEG:+.1f}] deg "
        f"during {context} -- this must never happen; aborting immediately."
    )


def home_joint(target_deg, bus, state):
    """Drive the joint to target_deg using Servo Velocity Loop Mode, the
    direction chosen automatically every tick from the live error (the
    joint can start on either side of the centre). The magnetic encoder
    alone decides arrival; then NUM_HOME_STOP_COMMANDS zero-velocity frames
    are sent back-to-back for a clean stop. Same as the step test."""
    print(f"  [home] Ankle: homing to {target_deg:+.2f} deg "
          f"(|HOMING_VELOCITY_RAD_S|={abs(HOMING_VELOCITY_RAD_S):.2f} rad/s, "
          f"direction chosen automatically from the live error every tick, "
          f"Velocity Loop Mode)...")
    clock_h = TimedLoop(rate=HOME_LOOP_HZ)
    t_home0 = time.perf_counter()
    home_tick = 0
    HOME_PRINT_EVERY = 5
    while clock_h():
        now_deg = read_actual_angle_deg_safe(bus, state)
        enforce_hard_limits(now_deg, "homing")

        err = wrap_deg(target_deg - now_deg)

        if abs(err) <= HOME_TOLERANCE_DEG:
            print(f"  [home] Ankle: reached {now_deg:+.2f} deg (target "
                  f"{target_deg:+.2f} deg, error {err:+.2f} deg).")
            break

        if time.perf_counter() - t_home0 > HOME_TIMEOUT_S:
            raise RuntimeError(
                f"Ankle failed to home within {HOME_TIMEOUT_S:.1f} s "
                f"(still {abs(err):.2f} deg from {target_deg:+.2f} deg per "
                f"the magnetic encoder) -- aborting before the control "
                f"loop starts. Verify MAGNETIC_ENCODER_OFFSET is correct, "
                f"verify |HOMING_VELOCITY_RAD_S| isn't too small to make "
                f"progress, and verify by hand that the shaft's ACTUAL "
                f"position isn't genuinely far from target_deg or "
                f"physically obstructed."
            )

        vel_cmd = ENCODER_TO_MOTOR_SIGN * math.copysign(abs(HOMING_VELOCITY_RAD_S), err)
        actuators.set_velocity(ANKLE_CAN_ID, vel_cmd, 0)

        if home_tick % HOME_PRINT_EVERY == 0:
            print(f"  [home] Ankle: t={time.perf_counter() - t_home0:5.2f}s  "
                  f"mag={now_deg:+7.2f} deg  err={err:+7.2f} deg  "
                  f"vel_cmd={vel_cmd:+.2f} rad/s  "
                  f"vel_actual={actuators.get_velocity(ANKLE_CAN_ID):+5.2f} rad/s")
        home_tick += 1

    print(f"  [home] Ankle: sending {NUM_HOME_STOP_COMMANDS} zero-velocity "
          f"stop commands ({HOME_STOP_COMMAND_INTERVAL_S * 1000:.0f} ms apart)...")
    for stop_i in range(NUM_HOME_STOP_COMMANDS):
        actuators.set_velocity(ANKLE_CAN_ID, 0.0, 0)
        time.sleep(HOME_STOP_COMMAND_INTERVAL_S)
    final_deg = read_actual_angle_deg_safe(bus, state)
    enforce_hard_limits(final_deg, "post-homing stop")
    print(f"  [home] Ankle: stopped. Final magnetic-encoder reading = "
          f"{final_deg:+.2f} deg (target {target_deg:+.2f} deg).")


def hold_baseline_velocity_mode(target_deg, hold_duration_s, t0, bus, state):
    """Keep the joint at target_deg for hold_duration_s in Velocity Loop
    Mode (small bidirectional bang-bang hold), the PID not yet running.
    NOT logged. Updates state.p_prev_deg so the PID phase's glitch guard
    has a valid last-known-good reading. Same as the step test."""
    print(f"  [hold] Ankle: holding {target_deg:+.2f} deg for "
          f"{hold_duration_s:.2f}s (velocity-mode bang-bang, PID not yet "
          f"engaged)...")
    clock_hold = TimedLoop(rate=HOME_LOOP_HZ)
    tick = 0
    HOLD_PRINT_EVERY = max(1, HOME_LOOP_HZ // 2)
    while clock_hold():
        t_wall = time.perf_counter() - t0
        if t_wall > hold_duration_s:
            break

        now_deg = read_actual_angle_deg_safe(bus, state)
        enforce_hard_limits(now_deg, "pre-sweep baseline hold")

        err = wrap_deg(target_deg - now_deg)
        if abs(err) > HOME_TOLERANCE_DEG:
            vel_cmd = ENCODER_TO_MOTOR_SIGN * math.copysign(abs(HOMING_VELOCITY_RAD_S), err)
        else:
            vel_cmd = 0.0
        actuators.set_velocity(ANKLE_CAN_ID, vel_cmd, 0)
        state.p_prev_deg = now_deg

        if tick % HOLD_PRINT_EVERY == 0:
            print(f"  [hold] Ankle: t={t_wall:5.2f}s  mag={now_deg:+7.2f} deg  "
                  f"err={err:+6.2f} deg  vel_cmd={vel_cmd:+.2f} rad/s")
        tick += 1

    print(f"  [hold] Ankle: sending {NUM_HOME_STOP_COMMANDS} zero-velocity "
          f"stop commands ({HOME_STOP_COMMAND_INTERVAL_S * 1000:.0f} ms apart) "
          f"before handing off to the PID controller...")
    for stop_i in range(NUM_HOME_STOP_COMMANDS):
        actuators.set_velocity(ANKLE_CAN_ID, 0.0, 0)
        time.sleep(HOME_STOP_COMMAND_INTERVAL_S)
    final_deg = read_actual_angle_deg_safe(bus, state)
    enforce_hard_limits(final_deg, "end of pre-sweep hold")
    state.p_prev_deg = final_deg
    print(f"  [hold] Ankle: hold complete. Final magnetic-encoder reading = "
          f"{final_deg:+.2f} deg (target {target_deg:+.2f} deg).")


def read_temperature():
    """Driver temperature (C) from the last CAN telemetry frame -- no extra
    bus traffic. NaN in a dry run or if the field is missing."""
    if actuators is None:
        return math.nan
    return getattr(actuators.get_data(ANKLE_CAN_ID), "temperature", math.nan)

# ---------------------------------------------------------------------------
# Preallocated log, one array per column (no list growth / GC pauses and no
# per-tick allocation inside the 1 kHz loop)
# ---------------------------------------------------------------------------
MAX_TICKS = int(PLANNED_S * LOOP_HZ * 1.15) + 5000
LOG_T = np.zeros(MAX_TICKS)
LOG_TRIAL = np.zeros(MAX_TICKS, dtype=np.int8)
LOG_REF = np.zeros(MAX_TICKS)
LOG_ACT = np.zeros(MAX_TICKS)
LOG_WIN = np.zeros(MAX_TICKS, dtype=np.int8)
LOG_CUR = np.zeros(MAX_TICKS) if LOG_CURRENT_A else None
N_LOGGED = 0
abort_reason = None
completed = False
exit_code = 0

# --- Signals: during Phase 3 a signal only sets a flag; during shutdown it
# only prints a message (see "STOPPING EARLY" in the docstring).
STOP_REQUESTED = False
_SIGNALS = [s for s in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)) if s is not None]
_ORIGINAL_HANDLERS = {}


def _on_stop_signal(signum, frame):
    global STOP_REQUESTED
    if not STOP_REQUESTED:
        print("\n[run] stop requested (signal) -- finishing this tick, releasing the motor, "
              "then saving and plotting what is complete...")
    STOP_REQUESTED = True


def _on_shutdown_signal(signum, frame):
    print("\n[shutdown] shutdown in progress, please wait...")


def _install(handler):
    for s in _SIGNALS:
        try:
            _ORIGINAL_HANDLERS.setdefault(s, signal.getsignal(s))
            signal.signal(s, handler)
        except (ValueError, OSError):
            pass


def _restore_default_handlers():
    for s, h in _ORIGINAL_HANDLERS.items():
        try:
            signal.signal(s, h if h is not None else signal.SIG_DFL)
        except (ValueError, OSError):
            pass

def build_header(status, achieved):
    return [
        ("joint", JOINT_NAME), ("script", os.path.basename(__file__)),
        ("date", datetime.datetime.now().isoformat(timespec="seconds")),
        ("mode", MODE), ("trial_label", ARGS.trial if ARGS.trial is not None else RUN_NUMBER),
        ("note", SETUP_NOTE),
        ("kp", KP), ("ki", KI), ("kd", KD), ("integral_limit_deg_s", INTEGRAL_LIMIT_DEG_S),
        ("d_on_measurement", D_ON_MEASUREMENT), ("d_filter_hz", D_FILTER_HZ),
        ("center_deg", CENTER_DEG), ("noise_peak_deg", NOISE_PEAK_DEG),
        ("ref_lp_hz", REF_LP_HZ), ("ref_lp_order", REF_LP_ORDER),
        ("rec_s", REC_S), ("fade_s", FADE_S), ("hold_s", HOLD_S), ("preroll_s", PREROLL_S),
        ("n_trials_planned", N_TRIALS),
        ("seed_base", SEED_BASE), ("seeds", ",".join(str(SEED_BASE + k) for k in range(1, N_TRIALS + 1))),
        ("bt_max_lag_s", BT_MAX_LAG_S), ("coh_min", COH_MIN), ("f_lo_hz", F_LO_HZ),
        ("planned_duration_s", f"{PLANNED_S:.1f}"),
        ("loop_hz_nominal", LOOP_HZ), ("loop_hz_achieved", "nan" if math.isnan(achieved) else f"{achieved:.1f}"),
        ("max_current_a", MAX_CURRENT_A),
        ("hard_limits_deg", f"{ANKLE_LIMIT_MIN_DEG:g}, {ANKLE_LIMIT_MAX_DEG:g}"),
        ("abort_error_deg", ABORT_ERROR_DEG), ("abort_swing_deg", ABORT_SWING_DEG),
        ("glitch_jump_deg", GLITCH_JUMP_DEG), ("max_driver_temp_c", MAX_DRIVER_TEMP_C),
        ("encoder_offset_deg", MAGNETIC_ENCODER_OFFSET), ("encoder_to_motor_sign", ENCODER_TO_MOTOR_SIGN),
        ("can_id", ANKLE_CAN_ID), ("as5048b_addr", f"0x{AS5048B_ADDR:02X}"),
        ("log_current_a", LOG_CURRENT_A),
        ("clamp_events", STATE.clamp_count), ("glitch_events", STATE.wrap_reject_count),
        ("i2c_fail_events", STATE.i2c_fail_count),
        ("status", status),
    ]


try:
    # Temperature / first-contact checks happen with the motor awake; the
    # confirmation comes BEFORE waking it.
    if not DRY_RUN and REQUIRE_CONFIRMATION and sys.stdin.isatty():
        input("\n>>> Knee blocked? Leg clamped as in the gait runs? Hand on the E-stop? "
              "Press Enter to wake the ankle motor and start (Ctrl+C aborts)... ")

    if not DRY_RUN:
        for motor_id in MOTORS:
            lifecycle(0xFC, motor_id)
        print("[run] Sent motor wake-up (0xFC) frame. Waiting for fresh "
              "telemetry...")
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
        temp0 = read_temperature()
        print(f"[run] Motor awake and telemetry flowing. Driver temperature: {temp0:.1f} C")
        if temp0 > MAX_DRIVER_TEMP_C:
            raise RuntimeError(f"Driver temperature {temp0:.1f} C is above {MAX_DRIVER_TEMP_C:.0f} C "
                               f"-- let the motor cool before starting.")

        print("\n[run] --- Phase 1: homing to the test centre ---")
        home_joint(CENTER_DEG, mag_bus, STATE)
        STATE.p_prev_deg = CENTER_DEG

        print("\n[run] --- Phase 2: pre-test hold at the centre (velocity-mode, "
              "PID not yet engaged) ---")
        hold_baseline_velocity_mode(CENTER_DEG, PRE_HOLD_S, time.perf_counter(), mag_bus, STATE)
    else:
        STATE.p_prev_deg = read_actual_angle_deg_safe(mag_bus, STATE)
        print(f"[run] Dry run: encoder reads {STATE.p_prev_deg:+.2f} deg "
              f"(centre would be {CENTER_DEG:+.2f} deg). No homing.")

    pid = PID(KP, KI, KD, INTEGRAL_LIMIT_DEG_S)
    print(f"\n[run] --- Phase 3: {N_TRIALS} noise trial(s) (KP={KP} KI={KI} KD={KD}, {LOOP_HZ} Hz) "
          f"--- Ctrl+C stops (motor is released first, finished trials are analysed).\n")

    _install(_on_stop_signal)         # from here on a signal only sets a flag

    clock = TimedLoop(rate=LOOP_HZ)
    t0_pid = time.perf_counter()   # t = 0 is the start of the pre-roll hold
    t_prev = t0_pid
    i = 0
    trial = 0                      # 0 = pre-roll
    trial_prev = -1
    trial_i0 = 0                   # first log row of the current trial
    trial_peak_i = 0.0
    FS_REF = bnt.FS

    def trial_line(k, t_end):
        """One printed line per finished trial (called outside the per-tick path)."""
        sl = slice(trial_i0, i)
        m = LOG_WIN[sl] == 1
        e = LOG_REF[sl][m] - LOG_ACT[sl][m]
        rms = float(np.sqrt(np.mean(e ** 2))) if len(e) else float("nan")
        print(f"[trial {k}/{N_TRIALS}] done  t={t_end:6.1f} s  record-window rms error {rms:.2f} deg  "
              f"peak |I| {trial_peak_i:5.1f} A")

    while clock():
        if STOP_REQUESTED:
            abort_reason = "stopped by user (Ctrl+C)"
            break

        actual_deg_raw = read_actual_angle_deg_safe(mag_bus, STATE)
        now_t = time.perf_counter()
        t_wall = now_t - t0_pid               # time_s: right after the encoder read

        # Hard, unconditional joint-limit check on the freshly read value.
        enforce_hard_limits(actual_deg_raw, "PID control loop")

        i_ref = int(t_wall * FS_REF)
        if i_ref >= N_REF:
            if trial_prev >= 1:
                trial_line(trial_prev, t_wall)
            completed = True
            break
        reference_deg = REF_LIST[i_ref]
        trial = TRIAL_LIST[i_ref]
        if trial != trial_prev:
            if trial_prev >= 1:
                trial_line(trial_prev, t_wall)
            trial_i0 = i
            trial_peak_i = 0.0
            trial_prev = trial

        # Reject the sample (reuse the last known-good position) if it
        # jumps implausibly far in a single tick.
        delta_deg = actual_deg_raw - STATE.p_prev_deg
        if abs(delta_deg) > GLITCH_JUMP_DEG:
            STATE.wrap_reject_count += 1
            STATE.wrap_reject_streak += 1
            STATE.wrap_reject_max_streak = max(
                STATE.wrap_reject_max_streak, STATE.wrap_reject_streak)
            if STATE.wrap_reject_streak > MAX_CONSECUTIVE_WRAP_REJECTS:
                abort_reason = (
                    f"Ankle: {STATE.wrap_reject_streak} consecutive "
                    f"implausible position jump(s) "
                    f"(> {GLITCH_JUMP_DEG:.0f} deg/tick) -- the magnetic "
                    f"encoder reading is not trustworthy right now."
                )
            actual_deg_raw = STATE.p_prev_deg
        else:
            STATE.wrap_reject_streak = 0
        STATE.p_prev_deg = actual_deg_raw

        # NO position filtering: the glitch-rejected raw reading goes
        # straight into the PID, exactly as in the gait scripts.
        actual_deg = actual_deg_raw

        dt = now_t - t_prev
        if dt <= 0.0:
            dt = 1.0 / LOOP_HZ
        t_prev = now_t

        current_cmd, p_term, i_term, d_term, error_deg = pid.update(
            reference_deg, actual_deg, dt)

        # ENCODER_TO_MOTOR_SIGN: flip the current command.
        current_cmd *= ENCODER_TO_MOTOR_SIGN

        current_clamped = current_cmd
        if abs(current_clamped) > MAX_CURRENT_A:
            current_clamped = math.copysign(MAX_CURRENT_A, current_clamped)
            STATE.clamp_count += 1
        if abs(current_clamped) > trial_peak_i:
            trial_peak_i = abs(current_clamped)

        if actuators is not None:
            actuators.set_torque(ANKLE_CAN_ID, current_clamped)   # amperes
        m_temp = read_temperature()

        # --- Guards (the tracking/swing guards need a controlled joint) ----
        if abort_reason is None and not DRY_RUN:
            if ABORT_ERROR_DEG > 0.0 and abs(error_deg) > ABORT_ERROR_DEG:
                abort_reason = (f"Ankle tracking error reached {abs(error_deg):.1f} deg "
                                f"(limit {ABORT_ERROR_DEG:.1f} deg) in trial {trial}.")
            elif abs(actual_deg - CENTER_DEG) > ABORT_SWING_DEG:
                abort_reason = (f"Ankle swung {actual_deg - CENTER_DEG:+.1f} deg from the centre "
                                f"(limit +/-{ABORT_SWING_DEG:.1f} deg) in trial {trial} -- "
                                f"resonance or a sign error?")
        if abort_reason is None and m_temp > MAX_DRIVER_TEMP_C:
            abort_reason = f"Driver temperature {m_temp:.1f} C exceeds {MAX_DRIVER_TEMP_C:.0f} C."

        if i >= MAX_TICKS:
            abort_reason = "log buffer full (loop ran far longer than planned)."
        else:
            LOG_T[i] = t_wall
            LOG_TRIAL[i] = trial
            LOG_REF[i] = reference_deg
            LOG_ACT[i] = actual_deg
            LOG_WIN[i] = WIN_LIST[i_ref]
            if LOG_CUR is not None:
                LOG_CUR[i] = current_clamped
            N_LOGGED = i + 1

        if abort_reason is not None:
            print(f"\n  [ABORT] {abort_reason}")
            print("  Stopping now. The CSV/plots below still show every trial that finished.")
            break

        i += 1

        if actuators is not None and getattr(actuators.notifier, "exception", None):
            print("RX thread died:", actuators.notifier.exception)
            abort_reason = "RX thread died"
            break

    if completed:
        print("\n[run] All trials finished.")

except KeyboardInterrupt:
    abort_reason = abort_reason or "stopped by user (Ctrl+C)"
    print("\n[run] Ctrl+C -- stopping.")

except Exception as e:                      # hard limit, I2C, homing timeout, ...
    abort_reason = abort_reason or f"{type(e).__name__}: {e}"
    exit_code = 1
    print(f"\n[run] ERROR -- {abort_reason}")

finally:
    _install(_on_shutdown_signal)     # a signal now only prints a message
    print("\n[shutdown] Stopping and releasing the motor...")
    # 1) RELEASE THE MOTOR FIRST (zero current AND velocity, then power off).
    if actuators is not None:
        try:
            for _ in range(10):
                for motor_id in MOTORS:
                    actuators.set_torque(motor_id, 0.0)
                    actuators.set_velocity(motor_id, 0.0, 0)
                time.sleep(0.005)
            for motor_id in MOTORS:
                lifecycle(0xFD, motor_id)
        except Exception as e:
            print(f"[shutdown] WARNING: error while zeroing/powering off the motor: {e}")
        try:
            actuators.disable_actuators()
            actuators.notifier.stop()
            actuators.bus.shutdown()
        except Exception:
            pass
        print("[shutdown] Motor limp; verify by hand before approaching.")
    try:
        mag_bus.close()
    except Exception:
        pass

    # 2) Write the data CSV, then analyse and plot whatever finished.
    n_ticks = N_LOGGED
    print(f"[shutdown] Logged {n_ticks} control tick(s).")
    achieved = float("nan")
    if n_ticks > 1:
        elapsed = LOG_T[n_ticks - 1] - LOG_T[0]
        achieved = (n_ticks - 1) / elapsed if elapsed > 0 else float("nan")
        print(f"[shutdown] Achieved loop rate: {achieved:.1f} Hz "
              f"(nominal {LOOP_HZ} Hz, {100.0 * achieved / LOOP_HZ:.1f}%)")
    if STATE.clamp_count > 0:
        print(f"  [warning] Ankle: current clamp engaged on {STATE.clamp_count}/{n_ticks} ticks.")
    if STATE.i2c_fail_count > 0:
        print(f"  [warning] Ankle: AS5048B I2C read failed on {STATE.i2c_fail_count}/{n_ticks} "
              f"tick(s) (worst consecutive streak: {STATE.i2c_fail_max_streak}).")
    if STATE.wrap_reject_count > 0:
        print(f"  [warning] Ankle: rejected {STATE.wrap_reject_count}/{n_ticks} implausible "
              f"position jump(s) (worst consecutive streak: {STATE.wrap_reject_max_streak}).")

    csv_ok = False
    try:
        data = {"time_s": LOG_T[:n_ticks], "trial": LOG_TRIAL[:n_ticks], "reference_deg": LOG_REF[:n_ticks],
                "actual_deg": LOG_ACT[:n_ticks], "in_window": LOG_WIN[:n_ticks]}
        if LOG_CUR is not None:
            data[bnt.CURRENT_COLUMN] = LOG_CUR[:n_ticks]
        out = pd.DataFrame(data)
        header = build_header("", achieved)
        n_done = bnt.analyse_data(out, dict(header))["n_complete"]
        if completed:
            status = f"COMPLETED ({n_done} of {N_TRIALS} trials)"
        else:
            status = f"ABORTED: {abort_reason or 'unknown'}, {n_done} of {N_TRIALS} trials"
        header = build_header(status, achieved) + [("n_trials_completed", n_done)]
        csv_path = os.path.join(RUN_DIR, bnt.DATA_CSV_NAME)
        bnt.write_csv(csv_path, header, out)
        print(f"[shutdown] CSV data saved to: {csv_path}")
        csv_ok = True
    except Exception as e:
        print(f"[shutdown] ERROR writing the CSV: {type(e).__name__}: {e}")
        exit_code = 1

    _restore_default_handlers()       # a stuck plotting step can be interrupted again

    if csv_ok:
        try:
            bnt.analyse_run(RUN_DIR, show=SHOW_PLOTS, make_plots=GENERATE_PLOTS)
        except KeyboardInterrupt:
            print("[shutdown] Analysis interrupted. The data CSV is saved; re-run: "
                  f"python bode_noise_tools.py analyse {RUN_DIR}")
        except Exception as e:
            print(f"[shutdown] Analysis failed ({type(e).__name__}: {e}). The CSV is saved; "
                  f"re-run: python bode_noise_tools.py analyse {RUN_DIR}")
            exit_code = 1

print("\nDone.")
sys.exit(exit_code)
