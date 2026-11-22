#!/usr/bin/env python3
"""
knee_step_test_PID_control_me.py -- Step-response test bench for tuning a
code-level PID position controller for the KNEE joint's actuator, driven
in the CubeMars driver's Servo CURRENT LOOP mode (Control Mode ID 1,
actuators.set_torque(), amperes -- Ch 14 S3.2 of the tutorial series).
"_me" = Magnetic Encoder (the AS5048B, not the motor's own internal one).

This is the knee counterpart of ankle_step_test_PID_control_me.py --
identical control law, structure, homing technique, guards, and plotting;
only the joint-specific settings below (CAN ID, magnetic-encoder address/
offset, hardware limits, step-signal convention) differ, per
leg_joint_specifications.txt's knee section.

PURPOSE
This script exists ONLY to tune KP, KI, KD for a code-level PID position
controller on the bench, using a classic step-response test: hold the
knee at a baseline angle, suddenly command a new target angle (the
"step"), and watch how the joint gets there. The KP/KI/KD values found
here are meant to be carried over, unchanged, into the full knee
gait-trajectory controllers later.

CONTROL LAW -- the PID equation lives here, in Python, not inside the
driver's own internal position loop (which is never used by this script).
Standard discrete-time textbook form, backward Euler on the error for D
and a forward/rectangular sum for I:

    error_deg[k]  = reference_deg[k] - actual_deg[k]
    integral      += error_deg[k] * dt                          (clamped, anti-windup)
    derivative     = (error_deg[k] - error_deg[k-1]) / dt

    current [A] = KP * error_deg[k] + KI * integral + KD * derivative

The PID's output IS the current sent straight to the motor driver's
Current Loop -- there is no separate torque-to-current conversion step.
KT is logged only, as an informational N*m estimate, and never enters the
control law.

NOTE ON DERIVATIVE KICK
Because the reference here is a literal step, error_deg[k]-error_deg[k-1]
contains one large, instantaneous jump at the exact tick the step
happens (the reference itself jumps there) -- so the D-term will show a
single large spike at that tick. That spike is an artifact of the step
TEST SIGNAL, not the plant -- expected, and part of why LOOP_HZ and
MAX_CURRENT_A exist as guards.

INPUT / FEEDBACK
actual_deg[k] comes ONLY from the knee's own external AS5048B magnetic
encoder (I2C) -- this script relies on the magnetic encoder alone; the
motor's internal encoder is never read, at all:

    actual_deg[k] = wrap_deg(magnetic_encoder_reading - MAGNETIC_ENCODER_OFFSET)

NO position filtering is applied -- actual_deg[k] is the raw (glitch-
guarded) reading, fed straight into the PID every tick, exactly as in
knee_ankle_trajectory_PID_control_me.py, so the gains tuned here carry
over to the trajectory controller unchanged. The glitch-jump guard
(GLITCH_JUMP_DEG) rejects an implausible single-tick jump and reuses the
last known-good reading.

SIGN CONVENTIONS -- two separate facts, kept identical to
knee_ankle_trajectory_PID_control_me.py
  - The encoder-to-angle convention is simply raw - OFFSET, wrapped (no
    MAGNETIC_ENCODER_SIGN). Knee: 0 deg = full extension, flexion NEGATIVE.
  - ENCODER_TO_MOTOR_SIGN: whether a positive commanded current/velocity
    moves actual_deg up or down. The PID's current and its P/I/D terms
    are multiplied by it right after the PID computes them, and the
    homing/hold velocity is multiplied by it too, so everything logged,
    printed and sent to the motor agrees. error_deg is a pure position
    quantity and is NOT flipped. Getting this wrong turns the P-term
    into positive feedback (a fast runaway) -- see its definition below.
  - DISPLAY_SIGN is cosmetic only: it flips the sign of the logged/
    plotted reference and actual angles (knee shown flexion-positive, as in the trajectory script) and never
    touches the control loop.

HOMING (velocity-mode-to-target)
Before the PID/current controller ever takes over, the joint is driven
to BASELINE_ANGLE_DEG -- the magnetic encoder's own offset-corrected
zero (actual_deg = 0, i.e. raw == MAGNETIC_ENCODER_OFFSET), which is
this leg's real mechanical full-extension position -- using the same
monitoring/stop technique as velocity_mode_to_target_magnetic_encoder.py
(Servo Velocity Loop Mode, actuators.set_velocity(), magnetic encoder
alone decides arrival, a burst of zero-velocity commands stops it), but
with one difference: since this script always homes to the SAME fixed
BASELINE_ANGLE_DEG every run, and the joint can start on either side of
it (e.g. the previous run ended at STEP_ANGLE_DEG, some degrees into
flexion), the DIRECTION of the commanded velocity is chosen
automatically every tick from the live error -- not a fixed, hand-
verified sign like the reference script uses. See home_joint() below.

HARD JOINT LIMITS -- NO TOLERANCE
KNEE_LIMIT_MIN_DEG / KNEE_LIMIT_MAX_DEG are enforced UNCONDITIONALLY,
every tick, both during homing and during the PID loop: the instant the
magnetic encoder's corrected reading is outside that range, the motor is
immediately zeroed and the script exits (see enforce_hard_limits()).
There is no margin and no grace period -- this is a hard safety rail,
not a soft guard.

TEST SEQUENCE -- the PID controller is NOT involved until step 3
  1. Home the joint (Velocity Loop Mode) to BASELINE_ANGLE_DEG --
     home_joint(), traveling at |HOMING_VELOCITY_RAD_S| with the
     direction chosen automatically from the live error every tick.
  2. Hold BASELINE_ANGLE_DEG for PRE_STEP_S seconds -- STILL Velocity
     Loop Mode, NOT the PID controller: a small bidirectional
     bang-bang hold (hold_baseline_velocity_mode()) keeps the joint at
     the baseline so the pre-step segment is a clean, controller-
     independent reference, not a demonstration of the (not yet tuned)
     PID holding still.
  3. Control HANDS OFF to the PID controller and the reference becomes
     STEP_ANGLE_DEG -- THE step -- held there for POST_STEP_S seconds
     under PID/current control. THIS INSTANT IS t=0 for all logging
     (see t0_pid below) -- the homing and pre-step hold phases above are
     NOT logged at all, so the CSV/plots start exactly at the step, not
     before it. The PID's integral and derivative state start
     completely fresh at this instant (no pre-loaded integral, no
     stale error).
  4. Motor released. Plots + a CSV of the logged data saved to
     results/step_test_knee/run_<N>/ next to this script (N increments
     automatically every run).

WHAT YOU SHOULD SEE WHEN YOU RUN IT
The motor wakes up, homes to BASELINE_ANGLE_DEG (full extension), holds
there (velocity-mode, not PID) for PRE_STEP_S seconds, then the PID
controller engages and the reference steps to STEP_ANGLE_DEG (further
into flexion, i.e. more negative) -- the joint moves to follow it --
printing a status line a few times a second throughout. Logging (and so
the CSV/plots) only covers this last part, starting at the step instant.
On exit the motor is released and:
  - ONE CSV for the run (time_s, dt_s, reference_deg, actual_deg,
    error_deg, p_term_A, i_term_A, d_term_A, current_cmd_A,
    current_clamped, tau_equivalent_Nm, torque_Nm -- one row per control
    tick, every quantity its own column, the step reference alongside
    the measured angle) is saved, so the graphs can be regenerated
    independently later (e.g. in MATLAB);
  - four PNG plots are saved: the step-response plot (reference vs.
    actual, with the hardware limits), the position-error plot, the
    commanded-current plot and the applied-torque plot (k_t * current).

Press Ctrl+C at any time -- the cleanup code still runs, the motor is
released safely, and the CSV/plots (from whatever was logged) are still
produced.
"""

# ===========================================================================
#  USER SETTINGS -- the fields you are most likely to want to change
# ===========================================================================

# --- CAN ID ------------------------------------------------------------
KNEE_CAN_ID = 100     # CAN ID of the knee actuator -- verify against the
                     # actual ID configured on this motor (CubeMarsTool).
                     # If the knee and ankle motors are EVER on the CAN
                     # bus at the same time, their CAN IDs must differ
                     # (knee_ankle_trajectory_servoPositionControl_me_ie.py
                     # uses 50/51 as a starting point for that case).

# --- Step-test signal --------------------------------------------------
# THE parameter this whole script exists to let you set by hand: the step
# target. Per leg_joint_specifications.txt, the knee's whole usable range
# sits at zero or below (0 deg = full extension, more negative = more
# flexion) -- so this is normally a NEGATIVE number, unlike the ankle
# script's positive default. Offset-corrected, absolute joint-degree
# frame (0 deg = full extension).
STEP_ANGLE_DEG = -15.0

BASELINE_ANGLE_DEG = 0.0    # pre-step hold angle (also the homing target --
                             # the magnetic encoder's own offset-corrected
                             # zero, i.e. this joint's full-extension
                             # position).

PRE_STEP_S = 0.0      # seconds spent holding BASELINE_ANGLE_DEG BEFORE the
                        # PID controller engages -- this hold uses Velocity
                        # Loop Mode (see hold_baseline_velocity_mode()), NOT
                        # the PID controller, so it gives a clean,
                        # controller-independent baseline segment on the
                        # plots rather than testing the (not yet tuned) PID
                        # at holding still.
POST_STEP_S = 1.0      # seconds spent holding STEP_ANGLE_DEG under PID
                        # control, after the step

# --- PID gains -- THE knobs this script exists to tune ---------------------
# Current output is in amperes; error/integral/derivative are in degrees,
# degree*seconds, and degrees/second, so the gains carry units of:
#   KP [A/deg], KI [A/(deg*s)], KD [A/(deg/s)]
# Tuning order: start with KI = KD = 0 and raise KP until the step
# response is fast but not oscillatory; add KD to damp any overshoot/
# ringing; add KI last, in small steps, only if a steady-state error
# remains once KP/KD are otherwise settled.
# NOTE: the knee's own inertia/leverage differ from the ankle's -- do NOT
# assume the ankle script's already-tuned gains carry over; start fresh,
# conservative, from these untuned defaults.
# KP = 1.5
# KI = 0.15
# KD = 0.06
KP = 3.0
KI = 0.0
KD = 0.1

# Anti-windup clamp on the ACCUMULATED INTEGRAL (units: degree*seconds --
# i.e. Sum(error_deg * dt) so far, NOT the sampling/loop period). This is
# a different quantity from Ts (the time between two consecutive control
# ticks): Ts is `dt` in the code below, measured fresh every tick from
# the actual elapsed wall-clock time (nominally 1/LOOP_HZ, ~0.001 s at
# LOOP_HZ=1000). INTEGRAL_LIMIT_DEG_S instead bounds how large the I-term's
# running sum is allowed to grow, so a persistent error can't wind the
# integral up without limit while KP/KD are otherwise saturated. Pick it
# so KI * INTEGRAL_LIMIT_DEG_S stays comfortably under MAX_CURRENT_A.
INTEGRAL_LIMIT_DEG_S = 50

# --- Current clamp -----------------------------------------------------
# 60 A is this motor's datasheet maximum current for Current Loop mode --
# NOT a "safe to start with" value. This is a hard ceiling only; keep KP/
# KI conservative so the PID's OWN output stays well under this during
# normal tuning -- the clamp engaging routinely is a sign the gains (or
# STEP_ANGLE_DEG) are too aggressive, not something to rely on.
MAX_CURRENT_A = 60.0    # amperes

# --- Control-loop rate ---------------------------------------------------
LOOP_HZ = 200

# --- Joint hardware limits (OSL v2 mechanical range), degrees --------------
# STRICTLY, UNCONDITIONALLY enforced every tick -- see enforce_hard_limits()
# and the HARD JOINT LIMITS note in the module docstring. No margin.
# Matches knee_ankle_trajectory_PID_control_me.py. The knee's whole range
# reads zero or negative on this sensor's mounting (flexion NEGATIVE).
KNEE_LIMIT_MIN_DEG = -110.0
KNEE_LIMIT_MAX_DEG = 3.0

# --- Runaway guard -----------------------------------------------------
# Soft guard, distinct from the hard position limits above: aborts if the
# TRACKING ERROR (not the absolute position) grows past this many
# degrees -- catches a controller that is diverging even while still
# technically inside the joint limits. Set to 0 to disable (not
# recommended on hardware).
ABORT_ERROR_DEG = 20.0

# --- Homing (velocity-mode-to-target) ---------------------------------------
# Drives the joint to BASELINE_ANGLE_DEG using Servo Velocity Loop Mode
# (actuators.set_velocity()) -- the same monitoring/stop technique as
# velocity_mode_to_target_magnetic_encoder.py (magnetic encoder alone
# decides arrival, a burst of zero-velocity commands stops it), but the
# DIRECTION is chosen automatically every tick from the live error (see
# home_joint() / hold_baseline_velocity_mode()) rather than a fixed,
# hand-verified sign -- this script always homes to the same
# BASELINE_ANGLE_DEG, and the joint can start on either side of it
# between runs, so a fixed sign would only work for one of the two
# possible starting sides.
HOMING_VELOCITY_RAD_S = 0.4   # rad/s at the output shaft. Only the
                                # MAGNITUDE matters here -- the sign you
                                # write is irrelevant, since home_joint()
                                # and hold_baseline_velocity_mode() both
                                # take abs() of this and pick direction
                                # from the live error themselves.
HOME_LOOP_HZ = 1000            # how often the homing loop reads the encoder
                                # and re-sends the velocity command
HOME_TOLERANCE_DEG = 0.2       # how close to BASELINE_ANGLE_DEG counts as "arrived"
HOME_TIMEOUT_S = 30.0          # abort homing if not arrived within this long --
                                # longer than the ankle script's default since
                                # the knee's range of motion is much wider
                                # (0 to 120 deg vs. the ankle's ~60 deg), so a
                                # full-range homing traverse takes longer at
                                # the same HOMING_VELOCITY_RAD_S.

NUM_HOME_STOP_COMMANDS = 15         # zero-velocity frames sent once arrived,
                                      # back-to-back -- a single one isn't
                                      # fully trusted (a dropped CAN frame
                                      # would leave the motor still spinning)
HOME_STOP_COMMAND_INTERVAL_S = 0.001  # gap between them, seconds

# --- Implausible-jump guard ----------------------------------------------
# Any single-tick position jump larger than this is rejected -- the last
# known-good position is reused instead. (Distinct from the hard position
# limits above -- this is about REJECTING one bad sample, not the joint's
# physical range.)
GLITCH_JUMP_DEG = 15.0
MAX_CONSECUTIVE_WRAP_REJECTS = 10

# --- Transmission gear ratio ------------------------------------------------
# Ratio between the actuator's own output shaft and the true joint.
# Logging/display only below (the tau_equivalent estimate) -- the magnetic
# encoder already reads the true joint angle directly, so nothing in the
# control law itself needs this. The knee's own transmission ratio may
# differ from the ankle's (GEAR_RATIO_FACTOR=4.61 there) -- 1.0 here is a
# placeholder; set the knee's actual ratio if known.
GEAR_RATIO_FACTOR = 4.61

# --- Torque constant --------------------------------------------------------
# Logging/display only -- the PID's output current is sent to the driver
# directly, this is never used inside the control law itself.
KT = 0.5701   # N*m/A, output-shaft torque constant

# --- Magnetic encoder (AS5048B) settings -----------------------------------
I2C_BUS = 1
AS5048B_ADDR = 0x40    # knee's own address -- see the docstring note above
ANGLE_HIGH_REG = 0xFE
ANGLE_LOW_REG = 0xFF
AGC_REG = 0xFA
DIAG_REG = 0xFB

# Offset between the AS5048B's raw reading and the joint's true zero (same
# value as in knee_ankle_trajectory_PID_control_me.py). actual_deg =
# wrap_deg(raw - MAGNETIC_ENCODER_OFFSET), no further sign.
MAGNETIC_ENCODER_OFFSET = 120.0   # deg

# ENCODER_TO_MOTOR_SIGN: converts "the direction the PID wants to move, in
# the magnetic encoder's own convention" into "the literal signed
# current/velocity to send to the motor driver" -- i.e. whether a positive
# commanded current/velocity moves actual_deg up or down. -1.0 here, the
# same verified value as KNEE_ENCODER_TO_MOTOR_SIGN in
# knee_ankle_trajectory_PID_control_me.py. This is the one that WOULD be a
# runaway risk if wrong (positive feedback). Applied every tick to the
# homing/hold velocity and to the PID current + P/I/D terms (error_deg
# itself is a pure position quantity and is NOT flipped).
ENCODER_TO_MOTOR_SIGN = -1.0

# How many consecutive failed I2C reads are tolerated before the script
# aborts (rather than running on stale data indefinitely).
MAX_CONSECUTIVE_I2C_FAILS = 25

# --- Plotting -------------------------------------------------------------
GENERATE_PLOTS = True
SHOW_PLOTS = True

# Cosmetic only -- multiplies the reference/actual angles (and the limit
# lines) written to the CSV and drawn on the position plot. The knee is
# displayed flexion-POSITIVE (-1.0); the ankle is unchanged (+1.0). Never
# used anywhere in the control loop.
DISPLAY_SIGN = -1.0

# ===========================================================================
#  END OF USER SETTINGS
# ===========================================================================

import os
import csv
import math
import time
import datetime
from dataclasses import dataclass, field

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
import matplotlib.pyplot as plt

from epicallypowerful.actuation import ActuatorGroup
from epicallypowerful.toolbox import TimedLoop

# epicallypowerful.actuation.actuator_group unconditionally attaches a
# FileHandler('motorlog.log') to the 'motorlog' logger at import time
# above, which creates that file next to this script on every run. Detach
# and remove it here so it stops appearing.
import logging as _logging
_motorlog = _logging.getLogger('motorlog')
for _h in _motorlog.handlers[:]:
    _motorlog.removeHandler(_h)
    _h.close()
if os.path.exists('motorlog.log'):
    os.remove('motorlog.log')

MOTOR_TYPE = 'AK80-9-V3-servo'   # Servo operating mode

print("=" * 78)
print("knee_step_test_PID_control_me.py")
print("=" * 78)

if STEP_ANGLE_DEG < KNEE_LIMIT_MIN_DEG or STEP_ANGLE_DEG > KNEE_LIMIT_MAX_DEG:
    raise ValueError(
        f"STEP_ANGLE_DEG={STEP_ANGLE_DEG:+.1f} deg falls outside the "
        f"knee's hardware limits [{KNEE_LIMIT_MIN_DEG:+.1f}, "
        f"{KNEE_LIMIT_MAX_DEG:+.1f}] deg -- aborting before the motor "
        f"moves."
    )
if BASELINE_ANGLE_DEG < KNEE_LIMIT_MIN_DEG or BASELINE_ANGLE_DEG > KNEE_LIMIT_MAX_DEG:
    raise ValueError(
        f"BASELINE_ANGLE_DEG={BASELINE_ANGLE_DEG:+.1f} deg falls outside "
        f"the knee's hardware limits [{KNEE_LIMIT_MIN_DEG:+.1f}, "
        f"{KNEE_LIMIT_MAX_DEG:+.1f}] deg -- aborting before the motor "
        f"moves."
    )
print("[startup] Pre-flight limit check passed: STEP_ANGLE_DEG and "
      "BASELINE_ANGLE_DEG both fall inside "
      f"[{KNEE_LIMIT_MIN_DEG:+.1f}, {KNEE_LIMIT_MAX_DEG:+.1f}] deg.")


# ---------------------------------------------------------------------------
# Results directory: results/step_test_knee/run_<N>/, N auto-incremented
# ---------------------------------------------------------------------------
_script_dir = os.path.dirname(os.path.abspath(__file__))
_results_root = os.path.join(_script_dir, "results", "step_test_knee")
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
# PID CONTROLLER -- the equation this whole script exists to tune
# ---------------------------------------------------------------------------
class PID:
    """Standard textbook discrete PID, computed here in Python (never
    inside the motor driver's own firmware loop). Output units are
    amperes; error/integral/derivative are in degrees / degree*seconds /
    degrees-per-second:

        e[k]  = r[k] - y[k]
        P     = Kp * e[k]
        I    += Ki * e[k] * Ts            (forward/rectangular sum, clamped)
        D     = Kd * (e[k] - e[k-1]) / Ts  (backward Euler on the ERROR)

    NOTE: because the reference is a literal step, e[k]-e[k-1] contains
    one huge instantaneous jump at the exact tick the step happens (since
    r[k]-r[k-1] is a step there too) -- the D-term will show a single
    large spike at that tick ("derivative kick"). That spike is an
    artifact of the step TEST SIGNAL, not the plant -- expected, and part
    of why LOOP_HZ and MAX_CURRENT_A exist as guards.
    """

    def __init__(self, kp, ki, kd, integral_limit_deg_s):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit_deg_s = integral_limit_deg_s
        self.integral_deg_s = 0.0
        self.prev_error_deg = None

    def reset(self):
        self.integral_deg_s = 0.0
        self.prev_error_deg = None

    def update(self, reference_deg, measurement_deg, dt):
        error_deg = reference_deg - measurement_deg

        self.integral_deg_s += error_deg * dt
        if self.integral_deg_s > self.integral_limit_deg_s:
            self.integral_deg_s = self.integral_limit_deg_s
        elif self.integral_deg_s < -self.integral_limit_deg_s:
            self.integral_deg_s = -self.integral_limit_deg_s

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
    """Read the two angle registers and return the raw single-turn angle
    in degrees, [0, 360). Reads both bytes in one I2C transaction (not
    two separate reads), so they can never be torn from two different
    instants while the shaft is moving."""
    high_byte, low_byte = bus.read_i2c_block_data(AS5048B_ADDR,
                                                    ANGLE_HIGH_REG, 2)
    raw14 = ((high_byte << 6) | (low_byte & 0x3F)) & 0x3FFF
    return (raw14 / 16384.0) * 360.0


def _as5048b_read_health(bus):
    """Read AGC and the diagnostics byte. Informational only (printed in
    the status line) -- not used for the control law or any guard."""
    agc = bus.read_byte_data(AS5048B_ADDR, AGC_REG)
    diag = bus.read_byte_data(AS5048B_ADDR, DIAG_REG)
    return agc, diag


def wrap_deg(angle_deg):
    """Wrap an angle in degrees to (-180, 180].

    The AS5048B only reports a single-turn [0, 360) reading, so
    subtracting MAGNETIC_ENCODER_OFFSET can land anywhere in roughly
    (-360, 360). This brings that back to a small, signed angle centered
    on the calibration zero point.
    """
    return ((angle_deg + 180.0) % 360.0) - 180.0


def read_actual_angle_deg(bus):
    """actual_angle = raw reading - MAGNETIC_ENCODER_OFFSET,
    wrapped. Callers should use
    read_actual_angle_deg_safe() below instead, which also tolerates a
    transient I2C failure."""
    raw_deg = _as5048b_read_angle_raw(bus)
    return wrap_deg(raw_deg - MAGNETIC_ENCODER_OFFSET)


def read_actual_angle_deg_safe(bus, state):
    """Same as read_actual_angle_deg(), but a failed I2C read reuses the
    last known-good angle for that tick instead of crashing. Raises if
    too many reads fail in a row, or if no good reading has ever been
    obtained."""
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
# RUNTIME STATE -- counters and log buffers
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
    last_agc: int = -1
    p_prev_deg: float = 0.0    # last known-good actual angle (glitch-reject reuse)
    log: dict = field(default_factory=lambda: {k: [] for k in (
        "t", "dt", "reference_deg", "actual_deg", "error_deg", "p_term",
        "i_term", "d_term", "current_cmd", "current_clamped",
        "tau_equivalent", "torque_nm")})


STATE = JointState()
MOTORS = {KNEE_CAN_ID: MOTOR_TYPE}

print(f"[startup] KNEE_CAN_ID               = {KNEE_CAN_ID}")
print(f"[startup] Magnetic encoder          = AS5048B on I2C bus {I2C_BUS}, "
      f"addr 0x{AS5048B_ADDR:02X}")
print(f"[startup] MAGNETIC_ENCODER_OFFSET   = {MAGNETIC_ENCODER_OFFSET:+.2f} deg")
print(f"[startup] ENCODER_TO_MOTOR_SIGN     = {ENCODER_TO_MOTOR_SIGN:+.1f}")
print(f"[startup] PID gains                 = KP={KP} A/deg  KI={KI} A/(deg*s)  "
      f"KD={KD} A/(deg/s)")
print(f"[startup] Integral clamp            = +/-{INTEGRAL_LIMIT_DEG_S:.1f} deg*s")
print(f"[startup] MAX_CURRENT_A             = {MAX_CURRENT_A:.1f} A")
print(f"[startup] Knee hard limits          = [{KNEE_LIMIT_MIN_DEG:+.1f}, "
      f"{KNEE_LIMIT_MAX_DEG:+.1f}] deg (unconditional, no margin)")
print(f"[startup] ABORT_ERROR_DEG           = {ABORT_ERROR_DEG:.1f} deg "
      f"(tracking-error guard)")
print(f"[startup] Homing                    = velocity-mode-to-target, "
      f"|HOMING_VELOCITY_RAD_S|={abs(HOMING_VELOCITY_RAD_S):.2f} rad/s "
      f"(direction auto-selected) -> {BASELINE_ANGLE_DEG:+.1f} deg "
      f"(tolerance {HOME_TOLERANCE_DEG:.1f} deg, timeout {HOME_TIMEOUT_S:.1f} s)")
print(f"[startup] Step test                 = baseline {BASELINE_ANGLE_DEG:+.1f} "
      f"deg for {PRE_STEP_S:.1f}s, then step to {STEP_ANGLE_DEG:+.1f} deg for "
      f"{POST_STEP_S:.1f}s")
print(f"[startup] LOOP_HZ                   = {LOOP_HZ} Hz")


# ---------------------------------------------------------------------------
# Open the CAN bus and the magnetic encoder's I2C bus
# ---------------------------------------------------------------------------
actuators = ActuatorGroup.from_dict(MOTORS, exit_manually=True)
print("[startup] CAN bus opened and knee motor registered with EPICally "
      "Powerful.")
mag_bus = SMBus(I2C_BUS)
print(f"[startup] I2C bus {I2C_BUS} opened for the AS5048B magnetic encoder.")


def lifecycle(code, can_id):
    """Send the motor's power-on (0xFC) or power-off (0xFD) message to
    one specific CAN ID."""
    actuators.bus.send(
        can.Message(arbitration_id=can_id, data=[0xFF] * 7 + [code],
                    is_extended_id=True)
    )


def enforce_hard_limits(actual_deg, context):
    """Immediately halts the motor and raises if the magnetic encoder's
    corrected angle is outside [KNEE_LIMIT_MIN_DEG, KNEE_LIMIT_MAX_DEG]
    -- no margin, no tolerance, no exceptions. Called every single tick,
    both during homing and during the PID loop, as the FIRST thing done
    with a freshly read actual_deg, before any new command is computed or
    sent. `context` is a short string identifying the calling phase, used
    only in the printed message."""
    if  KNEE_LIMIT_MIN_DEG <= actual_deg <= KNEE_LIMIT_MAX_DEG:
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
    """Drive the joint to target_deg using Servo Velocity Loop Mode
    (actuators.set_velocity()) -- the same monitoring/stop approach as
    velocity_mode_to_target_magnetic_encoder.py, but with one deliberate
    change: THIS script always homes to the same fixed BASELINE_ANGLE_DEG
    on every run, and the joint can be sitting on either side of it when
    a given run starts (e.g. the previous run ended at STEP_ANGLE_DEG),
    so a single fixed-sign velocity (as the reference script uses,
    requiring a hand-verified sign each time) would only work for one of
    the two possible starting sides. Instead, the sign of the commanded
    velocity is picked automatically every tick from the live error --
    magnitude |HOMING_VELOCITY_RAD_S|, direction toward target_deg -- the
    same technique hold_baseline_velocity_mode() uses. The magnetic
    encoder alone decides arrival; once the corrected angle is within
    HOME_TOLERANCE_DEG of target_deg, NUM_HOME_STOP_COMMANDS zero-velocity
    frames are sent back-to-back for a clean stop.
    """
    print(f"  [home] Knee: homing to {target_deg:+.2f} deg "
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
            print(f"  [home] Knee: reached {now_deg:+.2f} deg (target "
                  f"{target_deg:+.2f} deg, error {err:+.2f} deg).")
            break

        if time.perf_counter() - t_home0 > HOME_TIMEOUT_S:
            raise RuntimeError(
                f"Knee failed to home within {HOME_TIMEOUT_S:.1f} s "
                f"(still {abs(err):.2f} deg from {target_deg:+.2f} deg per "
                f"the magnetic encoder) -- aborting before the control "
                f"loop starts. Verify MAGNETIC_ENCODER_OFFSET is correct, "
                f"verify |HOMING_VELOCITY_RAD_S| isn't too small to make "
                f"progress, and verify by hand that the shaft's ACTUAL "
                f"position isn't genuinely far from target_deg or "
                f"physically obstructed."
            )

        vel_cmd = ENCODER_TO_MOTOR_SIGN * math.copysign(abs(HOMING_VELOCITY_RAD_S), err)
        actuators.set_velocity(KNEE_CAN_ID, vel_cmd, 0)

        if home_tick % HOME_PRINT_EVERY == 0:
            print(f"  [home] Knee: t={time.perf_counter() - t_home0:5.2f}s  "
                  f"mag={now_deg:+7.2f} deg  err={err:+7.2f} deg  "
                  f"vel_cmd={vel_cmd:+.2f} rad/s  "
                  f"vel_actual={actuators.get_velocity(KNEE_CAN_ID):+5.2f} rad/s")
        home_tick += 1

    print(f"  [home] Knee: sending {NUM_HOME_STOP_COMMANDS} zero-velocity "
          f"stop commands ({HOME_STOP_COMMAND_INTERVAL_S * 1000:.0f} ms apart)...")
    for stop_i in range(NUM_HOME_STOP_COMMANDS):
        actuators.set_velocity(KNEE_CAN_ID, 0.0, 0)
        time.sleep(HOME_STOP_COMMAND_INTERVAL_S)
    final_deg = read_actual_angle_deg_safe(bus, state)
    enforce_hard_limits(final_deg, "post-homing stop")
    print(f"  [home] Knee: stopped. Final magnetic-encoder reading = "
          f"{final_deg:+.2f} deg (target {target_deg:+.2f} deg).")


def hold_baseline_velocity_mode(target_deg, hold_duration_s, t0, bus, state):
    """Keep the joint at target_deg for hold_duration_s, still using
    Velocity Loop Mode -- the PID controller is NOT running yet during
    this phase (it only switches on at the step instant, right after
    this function returns). Unlike home_joint()'s one-way travel (a
    fixed-sign velocity command, since it may start far from the
    target), this is a small bidirectional bang-bang hold: whenever the
    magnetic encoder strays more than HOME_TOLERANCE_DEG from
    target_deg, a correction is sent in WHICHEVER direction reduces the
    error (sign taken from the live error, not a fixed constant);
    otherwise zero velocity is commanded. Ends with the same
    zero-velocity stop burst as home_joint().

    NOT logged into state.log -- the CSV/plots are meant to start at the
    exact instant the PID controller engages, not at homing/hold. This
    phase only updates state.p_prev_deg, so the PID phase's own
    glitch-jump guard has a valid "last known-good" reading to compare
    its very first tick against.
    """
    print(f"  [hold] Knee: holding {target_deg:+.2f} deg for "
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
        enforce_hard_limits(now_deg, "pre-step baseline hold")

        err = wrap_deg(target_deg - now_deg)
        if abs(err) > HOME_TOLERANCE_DEG:
            vel_cmd = ENCODER_TO_MOTOR_SIGN * math.copysign(abs(HOMING_VELOCITY_RAD_S), err)
        else:
            vel_cmd = 0.0
        actuators.set_velocity(KNEE_CAN_ID, vel_cmd, 0)
        state.p_prev_deg = now_deg

        if tick % HOLD_PRINT_EVERY == 0:
            print(f"  [hold] Knee: t={t_wall:5.2f}s  mag={now_deg:+7.2f} deg  "
                  f"err={err:+6.2f} deg  vel_cmd={vel_cmd:+.2f} rad/s")
        tick += 1

    print(f"  [hold] Knee: sending {NUM_HOME_STOP_COMMANDS} zero-velocity "
          f"stop commands ({HOME_STOP_COMMAND_INTERVAL_S * 1000:.0f} ms apart) "
          f"before handing off to the PID controller...")
    for stop_i in range(NUM_HOME_STOP_COMMANDS):
        actuators.set_velocity(KNEE_CAN_ID, 0.0, 0)
        time.sleep(HOME_STOP_COMMAND_INTERVAL_S)
    final_deg = read_actual_angle_deg_safe(bus, state)
    enforce_hard_limits(final_deg, "end of pre-step hold")
    state.p_prev_deg = final_deg
    print(f"  [hold] Knee: hold complete. Final magnetic-encoder reading = "
          f"{final_deg:+.2f} deg (target {target_deg:+.2f} deg).")


try:
    for motor_id in MOTORS:
        lifecycle(0xFC, motor_id)
    print("[run] Sent motor wake-up (0xFC) frame. Waiting for fresh "
          "telemetry...")

    # Wait for a few fresh telemetry frames after the wake-up message, so
    # the firmware has processed it before the first command lands.
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
    STATE.p_prev_deg = BASELINE_ANGLE_DEG

    print("\n[run] --- Phase 2: pre-step baseline hold (velocity-mode, "
          "PID not yet engaged) ---")
    t0 = time.perf_counter()
    hold_baseline_velocity_mode(BASELINE_ANGLE_DEG, PRE_STEP_S, t0, mag_bus,
                                 STATE)

    print(f"\n  [STEP] reference stepped {BASELINE_ANGLE_DEG:+.2f} -> "
          f"{STEP_ANGLE_DEG:+.2f} deg -- PID controller engaging now. CSV/"
          f"plot logging starts from this instant (t=0).\n")

    pid = PID(KP, KI, KD, INTEGRAL_LIMIT_DEG_S)
    reference_deg = STEP_ANGLE_DEG

    print(f"[run] --- Phase 3: PID step test (KP={KP} KI={KI} KD={KD}, "
          f"{LOOP_HZ} Hz) --- Ctrl+C stops.\n")

    clock = TimedLoop(rate=LOOP_HZ)
    t0_pid = time.perf_counter()   # fresh time origin -- t=0 is the step instant
    total_duration = POST_STEP_S
    t_prev = t0_pid
    i = 0
    abort_reason = None

    PRINT_EVERY = max(1, LOOP_HZ // 5)

    while clock():
        t_wall = time.perf_counter() - t0_pid
        if t_wall > total_duration:
            break

        actual_deg_raw = read_actual_angle_deg_safe(mag_bus, STATE)
        now_t = time.perf_counter()

        # Hard, unconditional joint-limit check -- on the freshly read
        # sensor value, BEFORE any glitch-rejection substitution below.
        enforce_hard_limits(actual_deg_raw, "PID control loop")

        # Reject the sample (reuse the last known-good position) if it
        # jumps implausibly far in a single tick -- a data-quality guard,
        # separate from the hard position-limit check above. Runs on the
        # RAW reading -- a single-tick I2C glitch should be
        # rejected outright.
        delta_deg = actual_deg_raw - STATE.p_prev_deg
        jump_bad = abs(delta_deg) > GLITCH_JUMP_DEG

        if jump_bad:
            STATE.wrap_reject_count += 1
            STATE.wrap_reject_streak += 1
            STATE.wrap_reject_max_streak = max(
                STATE.wrap_reject_max_streak, STATE.wrap_reject_streak)
            if STATE.wrap_reject_streak > MAX_CONSECUTIVE_WRAP_REJECTS:
                abort_reason = (
                    f"Knee: {STATE.wrap_reject_streak} consecutive "
                    f"implausible position jump(s) "
                    f"(> {GLITCH_JUMP_DEG:.0f} deg/tick) -- the magnetic "
                    f"encoder reading is not trustworthy right now -- "
                    f"stopping rather than running the controller on "
                    f"repeatedly bad data."
                )
            actual_deg_raw = STATE.p_prev_deg
        else:
            STATE.wrap_reject_streak = 0

        STATE.p_prev_deg = actual_deg_raw

        # NO position filtering -- the glitch-rejected raw magnetic-encoder
        # reading is fed straight into the PID, exactly as in
        # knee_ankle_trajectory_PID_control_me.py.
        actual_deg = actual_deg_raw

        dt = now_t - t_prev
        if dt <= 0.0:
            dt = 1.0 / LOOP_HZ
        t_prev = now_t

        current_cmd, p_term, i_term, d_term, error_deg = pid.update(
            reference_deg, actual_deg, dt)

        # ENCODER_TO_MOTOR_SIGN: flip current + all logged terms together,
        # so everything logged/printed/sent agrees -- error_deg itself is a
        # pure position quantity and is NOT flipped. Same convention as
        # knee_ankle_trajectory_PID_control_me.py.
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

        # --- Soft runaway guard: tracking error, not absolute position --
        if ABORT_ERROR_DEG > 0.0 and abs(error_deg) > ABORT_ERROR_DEG:
            abort_reason = (
                f"Knee tracking error reached {abs(error_deg):.1f} deg "
                f"(limit {ABORT_ERROR_DEG:.1f} deg) -- the joint is not "
                f"converging on the step target."
            )

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
        STATE.log["tau_equivalent"].append(current_clamped * KT * GEAR_RATIO_FACTOR)
        STATE.log["torque_nm"].append(KT * current_clamped)

        if abort_reason is not None:
            print(f"\n  [ABORT] {abort_reason}")
            print("  Stopping the test now. The CSV/plots below still show "
                  "everything logged up to this point.")
            break

        if i % PRINT_EVERY == 0:
            try:
                agc, diag = _as5048b_read_health(mag_bus)
                STATE.last_agc = agc
            except OSError:
                pass

            agc_str = f"{STATE.last_agc}" if STATE.last_agc >= 0 else "?"
            clamp_str = " [CLAMPED]" if clamped_flag else ""
            print(f"t={t_wall:5.2f}s  ref={reference_deg:+7.2f} deg  "
                  f"act={actual_deg:+7.2f} deg  "
                  f"err={error_deg:+6.2f} deg  "
                  f"P={p_term:+6.3f}  I={i_term:+6.3f}  D={d_term:+6.3f}  "
                  f"I_cmd={current_clamped:+5.2f} A{clamp_str}  AGC={agc_str}")
        i += 1

        if getattr(actuators.notifier, "exception", None):
            print("RX thread died:", actuators.notifier.exception)
            break

    print("\n[run] Step test finished.")

except KeyboardInterrupt:
    print("\n[run] Ctrl+C -- stopping.")

finally:
    print("\n[shutdown] Stopping and releasing the motor...")
    # Stop path: zero current AND velocity (whichever mode was last
    # active -- homing uses velocity, the PID loop uses current), then
    # release the firmware.
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
    print("[shutdown] Motor limp; verify by hand before approaching.")

    n_ticks = len(STATE.log["t"])
    print(f"[shutdown] Logged {n_ticks} control tick(s).")
    if n_ticks > 1:
        elapsed = STATE.log["t"][-1] - STATE.log["t"][0]
        achieved = (n_ticks - 1) / elapsed if elapsed > 0 else float("nan")
        print(f"[shutdown] Achieved loop rate: {achieved:.1f} Hz "
              f"(nominal {LOOP_HZ} Hz, {100.0 * achieved / LOOP_HZ:.1f}%)")

    if STATE.clamp_count > 0:
        print(f"  [warning] Knee: current clamp engaged on "
              f"{STATE.clamp_count}/{n_ticks} ticks "
              f"({100.0 * STATE.clamp_count / n_ticks:.1f}%) -- consider "
              f"lowering KP/KI if this seems large.")

    if STATE.i2c_fail_count > 0:
        print(f"  [warning] Knee: AS5048B I2C read failed on "
              f"{STATE.i2c_fail_count}/{n_ticks} tick(s) (worst "
              f"consecutive streak: {STATE.i2c_fail_max_streak}).")

    if STATE.wrap_reject_count > 0:
        print(f"  [warning] Knee: rejected {STATE.wrap_reject_count}/"
              f"{n_ticks} implausible position jump(s) (worst consecutive "
              f"streak: {STATE.wrap_reject_max_streak}).")

    # --- CSV export: ONE file for the whole run, every logged quantity
    # as its own column against time_s -- one row per control tick.
    # Meant for re-plotting elsewhere (e.g. MATLAB), so it is always
    # written, independent of GENERATE_PLOTS. ---------------------------
    if n_ticks == 0:
        print("[shutdown] No telemetry was logged at all -- no CSV or "
              "plots were produced.")
    else:
        csv_columns = ["t", "dt", "reference_deg", "actual_deg", "error_deg",
                        "p_term", "i_term", "d_term", "current_cmd",
                        "current_clamped", "tau_equivalent", "torque_nm"]
        csv_header = ["time_s", "dt_s", "reference_deg", "actual_deg",
                       "error_deg", "p_term_A", "i_term_A", "d_term_A",
                       "current_cmd_A", "current_clamped",
                       "tau_equivalent_Nm", "torque_Nm"]
        csv_path = os.path.join(RUN_DIR, "step_response_data.csv")
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(csv_header)
            # DISPLAY_SIGN is cosmetic only: applied to the two angle
            # columns, never to anything the controller used.
            for row in zip(*(
                    [DISPLAY_SIGN * v for v in STATE.log[col]]
                    if col in ("reference_deg", "actual_deg")
                    else STATE.log[col] for col in csv_columns)):
                writer.writerow(row)
        print(f"[shutdown] CSV data saved to: {csv_path}")

        if not GENERATE_PLOTS:
            print("[shutdown] Plot generation disabled (GENERATE_PLOTS = "
                  "False) -- no figures were produced.")
        else:
            d = STATE.log
            s = DISPLAY_SIGN

            # --- Main step-response plot: reference vs. actual (raw
            # magnetic encoder, offset-corrected), with the hardware
            # limits. t=0 is the step instant (see t0_pid). ----
            fig, ax = plt.subplots(1, 1, figsize=(10, 6))
            ax.plot(d["t"], [s * v for v in d["reference_deg"]],
                    color="blue", linewidth=2.5,
                    label="Reference (step signal)")
            ax.plot(d["t"], [s * v for v in d["actual_deg"]],
                    color="red", linewidth=2.5,
                    label="Actual angle (magnetic encoder, offset-corrected)")
            ax.axhline(s * KNEE_LIMIT_MIN_DEG, color="black", linewidth=1.0,
                       linestyle="--", alpha=0.6, label="Hardware limit")
            ax.axhline(s * KNEE_LIMIT_MAX_DEG, color="black", linewidth=1.0,
                       linestyle="--", alpha=0.6)
            ax.set_xlabel("Time [s]", fontsize=12)
            ax.set_ylabel("Knee joint angle [deg]", fontsize=12)
            ax.set_title(f"Knee step response (KP={KP}, KI={KI}, KD={KD})",
                         fontsize=13)
            ax.legend(loc="best", fontsize=11)
            ax.grid(True, alpha=0.3)
            fig.tight_layout()
            position_plot_path = os.path.join(RUN_DIR, "position_response.png")
            fig.savefig(position_plot_path, dpi=300, bbox_inches="tight")
            print(f"[shutdown] Plot saved to: {position_plot_path}")

            def plot_single_panel(traces, ylabel, title_suffix, filename,
                                   hline0=False, extra_hlines=()):
                fig, ax = plt.subplots(1, 1, figsize=(10, 5))
                for log_key, color, lw, alpha, legend_label in traces:
                    ax.plot(d["t"], d[log_key], color=color, linewidth=lw,
                            alpha=alpha, label=legend_label)
                if hline0:
                    ax.axhline(0.0, color="black", linewidth=0.8,
                               linestyle="--", alpha=0.5)
                for y, color, ls, alpha, lbl in extra_hlines:
                    ax.axhline(y, color=color, linewidth=0.8, linestyle=ls,
                               alpha=alpha, label=lbl)
                ax.set_ylabel(f"Knee {ylabel}")
                ax.set_xlabel("Time [s]")
                ax.set_title(f"Knee joint: {title_suffix}")
                ax.legend(loc="best")
                ax.grid(True, alpha=0.3)
                fig.tight_layout()
                out_path = os.path.join(RUN_DIR, filename)
                fig.savefig(out_path, dpi=300, bbox_inches="tight")
                print(f"[shutdown] Plot saved to: {out_path}")
                return fig

            # error_deg is in the controller's own convention (not flipped
            # by DISPLAY_SIGN), same as the trajectory script's error plot.
            err_hlines = ()
            if ABORT_ERROR_DEG > 0.0:
                err_hlines = ((ABORT_ERROR_DEG, "red", ":", 0.5, "ABORT_ERROR_DEG"),
                              (-ABORT_ERROR_DEG, "red", ":", 0.5, None))
            plot_single_panel(
                [("error_deg", "purple", 1.2, 1.0,
                  "Position error (ref - actual)")],
                ylabel="position error [deg]",
                title_suffix="position tracking error vs. time",
                filename="error.png",
                hline0=True,
                extra_hlines=err_hlines,
            )

            plot_single_panel(
                [("current_cmd", "darkorange", 1.2, 1.0, "Commanded current")],
                ylabel="current [A]",
                title_suffix="PID output (current sent to Current Loop)",
                filename="current_command.png",
                hline0=True,
            )

            plot_single_panel(
                [("torque_nm", "green", 1.2, 1.0, "Torque (k_t * current)")],
                ylabel="torque [N*m]",
                title_suffix="applied torque vs. time (k_t * control current)",
                filename="torque.png",
                hline0=True,
            )

            if SHOW_PLOTS:
                try:
                    plt.show()
                except Exception:
                    pass

print("\nDone.")