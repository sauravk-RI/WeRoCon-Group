#!/usr/bin/env python3
"""
knee_ankle_trajectory_PID_control_me_stairs_up.py -- STAIR ASCENT variant of the combined knee + ankle gait-
trajectory controller that drives BOTH joints with the code-level PID
current-loop position controller tuned in knee_step_test_PID_control_me.py
and ankle_step_test_PID_control_me.py (CubeMars Servo CURRENT LOOP mode,
Control Mode ID 1, actuators.set_torque(), amperes), instead of a single
step. The reference each joint tracks comes from one stride of STAIR ASCENT
recorded in stairs_up_normal/AB01_stairs_1_1_up_angle.csv (positions),
derived from the level-walk script
knee_ankle_trajectory_PID_control_me_normal_walk.py -- only the dataset,
stride selection and results folder differ; the control loop, safety
guards and logging are unchanged. The matching _grf.csv is NOT used: this
short clip has no complete heel-strike to heel-strike cycle (see
STRIDE_START_TIME_S below). Same OpenSim model/column conventions as the
level-walk trial, so both *_DATASET_TO_ENCODER_SIGN values are carried over
unchanged.
TIME_SCALE is 4 here, not 2: this stride contains a very fast knee flexion
(~770 deg/s at real speed, vs. ~400 deg/s anywhere in the level-walk
trial), so TIME_SCALE=4 is what brings its peak knee speed (~190 deg/s)
down to the ~190 deg/s that the level-walk script's PID gains were
proven on at TIME_SCALE=2.
"_me" = Magnetic Encoder -- exactly like the step-test scripts, this
script relies EXCLUSIVELY on each joint's own external AS5048B magnetic
encoder for feedback; the motor's internal encoder is never read, at all,
for control OR comparison.

CONTROL LAW -- identical to the two step-test scripts, run independently
per joint, every tick (standard discrete PID, backward Euler on the error
for D, forward/rectangular sum for I):

    error_deg[k]  = reference_deg[k] - actual_deg[k]
    integral      += error_deg[k] * dt                    (clamped, anti-windup)
    derivative     = (error_deg[k] - error_deg[k-1]) / dt

    current [A] = KP * error_deg[k] + KI * integral + KD * derivative

reference_deg[k] is no longer a fixed step target -- it comes from a cubic
spline fit through the recorded gait-cycle angle data (Pd(t) below),
exactly the way the reference is generated in
knee_ankle_trajectory_servoPositionControl_me_ie.py. Everything else about
the control law -- units, the PID class, the glitch-jump guard,
ENCODER_TO_MOTOR_SIGN, and the hard/soft safety guards -- is carried over
UNCHANGED from the step-test scripts, per-joint. NO position filtering is
applied to the magnetic-encoder feedback -- actual_deg[k] is the raw
(glitch-guarded) reading, fed straight into the PID every tick.

WHY TWO SEPARATE PID INSTANCES, NOT SHARED GAINS
The knee and ankle were each tuned independently on the bench (see their
own step-test scripts) -- their inertia, leverage, and transmissions
differ, so each joint keeps its own KP/KI/KD, integral clamp, current
clamp, hardware limits, magnetic-encoder calibration (I2C address, offset),
and ENCODER_TO_MOTOR_SIGN below, all copied verbatim from that joint's own
step-test script. Nothing here re-derives or re-tunes those numbers.

TWO MOTORS, ONE CAN BUS, ONE SHARED I2C BUS -- READ THIS FIRST
KNEE_CAN_ID and ANKLE_CAN_ID below are 100 and 1 respectively, matching
the CAN IDs already verified in the two step-test scripts -- verify these
are still correct on the actual bench setup before running. The two
AS5048B magnetic encoders share I2C_BUS but sit at different addresses
(KNEE_AS5048B_ADDR=0x40, ANKLE_AS5048B_ADDR=0x41), also copied verbatim
from the step-test scripts.

SAFETY NOTE -- if EITHER joint's hard limit, tracking-error guard, or I2C
guard fires, BOTH joints stop immediately: an exoskeleton with only one of
its two joints still being actively driven is not a safe state. This
mirrors knee_ankle_trajectory_servoPositionControl_me_ie.py's own
philosophy, adapted to Current Loop Mode.

DATASET SIGN CONVENTION -- READ BEFORE TRUSTING A RUN
KNEE_DATASET_TO_ENCODER_SIGN / ANKLE_DATASET_TO_ENCODER_SIGN below map the
recorded gait angles onto each joint's OWN magnetic-encoder sign
convention as established in that joint's step-test script:
  - Knee: the step-test script's convention is flexion-NEGATIVE (0 deg =
    full extension, more negative = more flexion, hardware limits
    [-110, +1] deg). The raw "knee_angle_l" column in this dataset already
    reads negative during flexion (-86.94 deg to -12.54 deg across the
    whole clip) -- i.e. it already matches this joint's convention
    directly, hence KNEE_DATASET_TO_ENCODER_SIGN = +1.0 (no negation).
    This is the OPPOSITE of what knee_ankle_trajectory_servoPositionControl_me_ie.py
    does with this same column (it negates it) -- that is because that
    script's knee convention is flexion-POSITIVE, a different (older)
    convention than the one verified in knee_step_test_PID_control_me.py,
    which this script's control law is carried over from unchanged.
  - Ankle: the step-test script's convention is "positive = plantar-
    flexion, negative = dorsiflexion", hardware limits [-25, +25] deg.
    ANKLE_DATASET_TO_ENCODER_SIGN = -1.0 here was independently re-derived
    (see conversation history / commit notes) by tracing the actual gait
    pattern against GRF-detected toe-off timing: the dataset's
    "ankle_angle_l" column peaks positive at midstance (dorsiflexion) and
    peaks negative at push-off (plantarflexion) -- i.e. it is
    dorsiflexion-POSITIVE, the opposite of this joint's plantarflexion-
    positive convention, hence the -1.0 flip. Getting this sign wrong
    would make the ankle trajectory play the recorded gait pattern
    mirrored in time (dorsiflexion where the recording shows
    plantarflexion and vice versa) -- NOT a runaway, since the PID still
    tracks whatever the (possibly mirrored) reference is, inside the
    hardware limits and every guard below.
Neither of the above is the same fact as ENCODER_TO_MOTOR_SIGN (whether a
positive commanded current moves actual_deg up or down) -- that one WOULD
be a runaway risk if wrong, and is carried over unchanged and already
verified in each joint's own step-test script.

HOMING (velocity-mode-to-target, identical technique to the step tests) --
SINGLE PHASE, DIRECTLY TO THE TRAJECTORY'S OWN START ANGLE
Both joints are driven, in ONE step, straight from wherever they happen to
be sitting when the script starts to their own dataset-derived trajectory
starting angle -- there is no separate "home to zero/mean position
first" step. Servo Velocity Loop Mode (actuators.set_velocity()) is used,
direction chosen automatically every tick from the live error (see
home_joints() below) -- not a fixed, hand-verified sign -- since the
starting angle depends on the dataset and STRIDE_COUNT, and the joint may
already be sitting on either side of it. Both joints home together,
interleaved in one loop; whichever joint arrives first simply holds
(zero velocity) while it waits for the other to arrive too -- home_joints()
only returns once BOTH have arrived.

TEST SEQUENCE -- the PID controllers are NOT involved until step 3
  1. Home both joints (Velocity Loop Mode), directly and only, to their
     own dataset-derived trajectory starting angle -- home_joints(). A
     joint that arrives before the other waits there for it.
  2. Once both have arrived, settle_before_trajectory() holds both joints
     at that same position (zero velocity, no further driving needed --
     they are already there) for PRE_TRAJ_HOLD_S seconds, so the PID
     controllers don't engage the instant homing stops.
  3. Control HANDS OFF to both PID controllers together, at the same
     instant, and the reference becomes each joint's own
     spline-interpolated gait trajectory, Pd(t), for traj_duration
     seconds. THIS INSTANT IS t=0 for all logging -- the homing and
     settle phases are NOT logged, so the CSV/plots start exactly at the
     handoff, not before it. Each PID's integral/derivative state starts
     completely fresh at this instant.
  4. Motors released. Plots + one CSV per joint saved to
     results/trajectory_PID_knee_ankle_stairs_up/run_<N>/ next to this script (N
     increments automatically every run).

Press Ctrl+C at any time -- the cleanup code still runs, both motors are
released safely, and the CSV/plots (from whatever was logged) are still
produced.

PER-JOINT ENABLE SWITCHES -- KNEE_CONTROL / ANKLE_CONTROL
Each is 1 (drive this joint) or 0 (leave it completely alone this run --
no homing, no settling, no PID tracking, its motor is never even woken,
and its magnetic encoder is never read). This lets you bench-tune one
joint's PID gains in isolation without the other joint's motor, sensor,
or CAN registration being touched at all:
  - KNEE_CONTROL=1, ANKLE_CONTROL=0 -- knee only.
  - KNEE_CONTROL=0, ANKLE_CONTROL=1 -- ankle only.
  - Both 1 -- the normal combined knee+ankle run.
  - Both 0 -- nothing moves; the script exits immediately, before opening
    the CAN or I2C buses.
When only one joint is enabled, the CSV/plots for the disabled joint are
simply not produced (the plotting code sizes itself to however many
joints are actually in JOINTS).
"""

# ===========================================================================
#  USER SETTINGS -- the fields you are most likely to want to change
# ===========================================================================

import os   # needed up front, since the dataset paths below use it

# Dataset folders live next to this script, so the paths below keep working
# wherever the whole project folder is copied to.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# --- Paths to the dataset files (shared -- both joints come from the
# same trial, same time base) ------------------------------------------------
ANGLE_CSV_PATH = os.path.join(_SCRIPT_DIR, "stairs_up_normal", "AB01_stairs_1_1_up_angle.csv")

# The stride to send to the motors, as a window of this clip's own "time"
# column, in seconds. This clip is a short recording (~2.4 s) that contains
# no complete heel-strike to heel-strike cycle -- the force plates only
# caught one foot contact -- so, unlike the level-walk and ramp scripts,
# the stride is NOT found from GRF. It is instead bounded by two
# consecutive deep knee-flexion peaks of the left knee (-84.5 deg at t=4.275 s and -86.9 deg at t=5.655 s), i.e. one
# full left-leg cycle that starts and ends in nearly the same knee pose.
# Both numbers are snapped to the nearest recorded sample.
STRIDE_START_TIME_S = 4.275
STRIDE_END_TIME_S = 5.655

# Number of times the stride above is repeated back-to-back. The clip is too
# short to hold more than one real stride, so the single stride is tiled.
# Each stride's start-to-end offset is removed with a linear ramp (see
# load_gait_cycle_both()) so the repeats join without a position jump.
STRIDE_COUNT = 5
# True: start the trajectory at the knee's maximum value within the stride
# (the ankle starts at its corresponding value) instead of at the window
# start, which is the knee's deepest-flexion minimum.
START_AT_KNEE_MAX = True

# --- CAN IDs -- MUST be distinct (two motors, one bus). Copied verbatim
# from each joint's own step-test script -- verify against the actual
# configured IDs before running. ---------------------------------------------
KNEE_CAN_ID = 100
ANKLE_CAN_ID = 1

# --- Per-joint enable switches -- see "PER-JOINT ENABLE SWITCHES" in the
# module docstring above. 1 = drive this joint this run, 0 = leave it
# completely untouched (no homing, no PID, motor never woken, sensor never
# read). Use this to bench-tune one joint's PID gains in isolation. -------
KNEE_CONTROL = 1
ANKLE_CONTROL = 1

# --- Control-loop rate (shared) --------------------------------------------
LOOP_HZ = 200

# --- Trajectory shaping (shared) --------------------------------------------
TIME_SCALE = 1   # stretches the whole trajectory by this factor

# --- Reference-trajectory sign corrections (per joint) ----------------------
# See the "DATASET SIGN CONVENTION" note in the module docstring above
# before changing these.
# *_DATASET_TO_ENCODER_SIGN: converts the raw dataset column's sign
# convention into this joint's own magnetic-encoder sign convention. Used
# once, offline, in load_gait_cycle_both() below, to build the reference
# trajectory and homing target -- never touches the live feedback loop.
KNEE_DATASET_TO_ENCODER_SIGN = 1.0
ANKLE_DATASET_TO_ENCODER_SIGN = -1.0
# *_ENCODER_TO_MOTOR_SIGN: converts "the direction the PID wants to move,
# in the magnetic encoder's own convention" into "the literal signed
# current/velocity to send to the motor driver" -- i.e. whether a positive
# commanded current/velocity moves actual_deg up or down. This is the one
# that WOULD be a runaway risk if wrong (see the module docstring). Applied
# every tick, both during homing (home_joints()) and PID trajectory
# tracking, identically to the step-test scripts.
KNEE_ENCODER_TO_MOTOR_SIGN = -1.0
ANKLE_ENCODER_TO_MOTOR_SIGN = -1.0

# --- Explicit trajectory start point (per joint) ----------------------------
# By default (None), the trajectory starts wherever the dataset's own first
# sample places it (auto-detected, sign-corrected). Set either of these to
# a number (encoder-convention degrees) to instead pin the trajectory's
# start to exactly that value -- the whole shape is shifted up/down as a
# block to make Pd(0) equal your chosen number; nothing is trimmed or
# stretched in time. This is independent of AUTO_FIT_TO_LIMITS below, which
# still runs afterward and will further nudge (or abort, if it can't fit)
# whatever start point you end up with -- dataset-derived or overridden --
# so it can't be used to force a start point that doesn't actually fit
# inside the hardware limits.
KNEE_TRAJECTORY_START_DEG = None
ANKLE_TRAJECTORY_START_DEG = None

# --- What to do if a trajectory doesn't fit inside its joint limits --------
AUTO_FIT_TO_LIMITS = True

# --- Pre-trajectory hold (shared timing, both joints together) -------------
# After homing, both joints are actively held at their own dataset start
# angle (velocity-mode bang-bang, NOT the PID controllers) for this long
# before the PID trajectory tracking engages.
PRE_TRAJ_HOLD_S = 1.0

# ===========================================================================
# --- KNEE settings -- copied verbatim from knee_step_test_PID_control_me.py
# ===========================================================================
KNEE_KP = 3.0
KNEE_KI = 0.0
KNEE_KD = 0.1
KNEE_INTEGRAL_LIMIT_DEG_S = 50.0
KNEE_MAX_CURRENT_A = 60.0
KNEE_LIMIT_MIN_DEG = -110.0
KNEE_LIMIT_MAX_DEG = 4.0
KNEE_ABORT_ERROR_DEG = 20.0
KNEE_HOMING_VELOCITY_RAD_S = 0.4
KNEE_HOME_TOLERANCE_DEG = 0.2
KNEE_HOME_TIMEOUT_S = 30.0
KNEE_AS5048B_ADDR = 0x40
KNEE_MAGNETIC_ENCODER_OFFSET = 120.0   # deg
KNEE_GEAR_RATIO_FACTOR = 4.61   # logging/display only, see step-test note

# ===========================================================================
# --- ANKLE settings -- copied verbatim from ankle_step_test_PID_control_me.py
# ===========================================================================
ANKLE_KP = 2.0
ANKLE_KI = 0.0
ANKLE_KD = 0.075
ANKLE_INTEGRAL_LIMIT_DEG_S = 50.0
ANKLE_MAX_CURRENT_A = 60.0
ANKLE_LIMIT_MIN_DEG = -30.0
ANKLE_LIMIT_MAX_DEG = 30.0
ANKLE_ABORT_ERROR_DEG = 15.0
ANKLE_HOMING_VELOCITY_RAD_S = 0.4
ANKLE_HOME_TOLERANCE_DEG = 0.1
ANKLE_HOME_TIMEOUT_S = 30.0
ANKLE_AS5048B_ADDR = 0x41
ANKLE_MAGNETIC_ENCODER_OFFSET = 200.0   # deg
ANKLE_GEAR_RATIO_FACTOR = 4.61   # logging/display only

# --- Shared guard settings -- identical in both step-test scripts ----------
GLITCH_JUMP_DEG = 15.0
MAX_CONSECUTIVE_WRAP_REJECTS = 10
KT = 0.7044   # N*m/A, output-shaft torque constant, logging only

# --- Homing (shared timing) -------------------------------------------------
HOME_LOOP_HZ = 1000
NUM_HOME_STOP_COMMANDS = 5
HOME_STOP_COMMAND_INTERVAL_S = 0.001

# --- Magnetic encoder (AS5048B) register map (shared) -----------------------
I2C_BUS = 1
ANGLE_HIGH_REG = 0xFE
ANGLE_LOW_REG = 0xFF
AGC_REG = 0xFA
DIAG_REG = 0xFB
MAX_CONSECUTIVE_I2C_FAILS = 25

# --- Plotting ---------------------------------------------------------------
GENERATE_PLOTS = True
SHOW_PLOTS = True

# ===========================================================================
#  END OF USER SETTINGS
# ===========================================================================

import csv
import math
import time
from dataclasses import dataclass, field

import can
import numpy as np
import pandas as pd
from scipy.interpolate import CubicSpline
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
GLITCH_JUMP_RAD = math.radians(GLITCH_JUMP_DEG)

print("=" * 78)
print("knee_ankle_trajectory_PID_control_me_stairs_up.py")
print("=" * 78)

if KNEE_CAN_ID == ANKLE_CAN_ID:
    raise ValueError(
        f"KNEE_CAN_ID and ANKLE_CAN_ID are both {KNEE_CAN_ID} -- two "
        f"motors on the same CAN bus cannot share an ID."
    )
if KNEE_AS5048B_ADDR == ANKLE_AS5048B_ADDR:
    raise ValueError(
        f"KNEE_AS5048B_ADDR and ANKLE_AS5048B_ADDR are both "
        f"0x{KNEE_AS5048B_ADDR:02X} -- two sensors on the same I2C bus "
        f"cannot share an address."
    )
if KNEE_CONTROL not in (0, 1) or ANKLE_CONTROL not in (0, 1):
    raise ValueError(
        f"KNEE_CONTROL and ANKLE_CONTROL must each be 0 or 1, got "
        f"KNEE_CONTROL={KNEE_CONTROL!r}, ANKLE_CONTROL={ANKLE_CONTROL!r}."
    )


# ---------------------------------------------------------------------------
# Results directory: results/trajectory_PID_knee_ankle_stairs_up/run_<N>/, N
# auto-incremented, same convention as the step-test scripts.
# ---------------------------------------------------------------------------
_script_dir = os.path.dirname(os.path.abspath(__file__))
_results_root = os.path.join(_script_dir, "results", "trajectory_PID_knee_ankle_stairs_up")
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
# PID CONTROLLER -- identical to the one in both step-test scripts
# ---------------------------------------------------------------------------
class PID:
    """Standard textbook discrete PID, computed here in Python. Output
    units are amperes; error/integral/derivative are in degrees /
    degree*seconds / degrees-per-second:

        e[k]  = r[k] - y[k]
        P     = Kp * e[k]
        I    += Ki * e[k] * Ts            (forward/rectangular sum, clamped)
        D     = Kd * (e[k] - e[k-1]) / Ts  (backward Euler on the ERROR)
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
# MAGNETIC ENCODER (AS5048B) SUPPORT -- parametrized by joint, since each
# joint has its own I2C address and offset.
# ------------------------------------------------------------------

def _as5048b_read_angle_raw(bus, addr):
    """Read the two angle registers in one I2C transaction and return the
    raw single-turn angle in degrees, [0, 360)."""
    high_byte, low_byte = bus.read_i2c_block_data(addr, ANGLE_HIGH_REG, 2)
    raw14 = ((high_byte << 6) | (low_byte & 0x3F)) & 0x3FFF
    return (raw14 / 16384.0) * 360.0


def _as5048b_read_health(bus, addr):
    """AGC + diagnostics byte. Informational only."""
    agc = bus.read_byte_data(addr, AGC_REG)
    diag = bus.read_byte_data(addr, DIAG_REG)
    return agc, diag


def wrap_deg(angle_deg):
    """Wrap an angle in degrees to (-180, 180]."""
    return ((angle_deg + 180.0) % 360.0) - 180.0


def read_actual_angle_deg(bus, joint):
    """actual_angle = raw reading - joint.mag_offset, wrapped. Callers
    should use read_actual_angle_deg_safe() instead."""
    raw_deg = _as5048b_read_angle_raw(bus, joint.as5048b_addr)
    return wrap_deg(raw_deg - joint.mag_offset)


def read_actual_angle_deg_safe(bus, joint):
    """Same as read_actual_angle_deg(), but a failed I2C read reuses the
    last known-good angle for that tick instead of crashing. Raises if
    too many reads fail in a row, or if no good reading has ever been
    obtained."""
    try:
        angle = read_actual_angle_deg(bus, joint)
        joint.i2c_fail_streak = 0
        joint.i2c_last_good_deg = angle
        joint.i2c_has_good_reading = True
        return angle
    except OSError as e:
        joint.i2c_fail_count += 1
        joint.i2c_fail_streak += 1
        joint.i2c_fail_max_streak = max(joint.i2c_fail_max_streak,
                                         joint.i2c_fail_streak)
        print(f"  [warning] {joint.label}: AS5048B I2C read failed ({e}) -- "
              f"fail streak = {joint.i2c_fail_streak}")
        if (not joint.i2c_has_good_reading
                or joint.i2c_fail_streak > MAX_CONSECUTIVE_I2C_FAILS):
            reason = ("no good reading has ever been obtained yet -- "
                      "check wiring / I2C address / power before "
                      "trusting this sensor for control.")
            if joint.i2c_has_good_reading:
                reason = ("the magnetic encoder appears to be "
                          "disconnected or the I2C bus is stuck -- "
                          "aborting rather than running the control "
                          "loop blind on stale data.")
            raise RuntimeError(
                f"{joint.label}: {joint.i2c_fail_streak} consecutive "
                f"AS5048B I2C read failure(s) (last error: {e}) -- "
                f"{reason}"
            ) from e
        return joint.i2c_last_good_deg


# ---------------------------------------------------------------------------
# STEP 1 -- Load the gait cycle from the CSVs, for BOTH joints at once
# ---------------------------------------------------------------------------
def load_gait_cycle_both(angle_csv_path, stride_start_time_s,
                         stride_end_time_s, stride_count=1):
    """Read ONE stride from the angle CSV (repeated stride_count times) -- the samples whose "time"
    column lies between stride_start_time_s and stride_end_time_s (each
    snapped to the nearest recorded sample) -- as time/angle arrays in
    seconds and degrees for BOTH the knee and ankle columns, each shifted
    so its own trajectory starts at angle 0. Also returns each joint's
    absolute starting angle in degrees (its homing target).

    Returns: t, knee_deg_rel, knee_start_deg, ankle_deg_rel, ankle_start_deg
    """
    df = pd.read_csv(angle_csv_path)
    t = df["time"].to_numpy(dtype=float)

    # Raw dataset columns -- see the "DATASET SIGN CONVENTION" note in the
    # module docstring for why these need correcting
    # (KNEE_DATASET_TO_ENCODER_SIGN / ANKLE_DATASET_TO_ENCODER_SIGN applied
    # below, once, on top of the raw read).
    knee_deg_raw = df["knee_angle_l"].to_numpy(dtype=float)
    ankle_deg_raw = df["ankle_angle_l"].to_numpy(dtype=float)

    half_sample_s = 0.5 * float(np.median(np.diff(t)))
    if (stride_start_time_s < t[0] - half_sample_s
            or stride_end_time_s > t[-1] + half_sample_s
            or stride_end_time_s <= stride_start_time_s):
        raise ValueError(
            f"STRIDE_START_TIME_S={stride_start_time_s}, "
            f"STRIDE_END_TIME_S={stride_end_time_s} is not a valid window "
            f"inside this clip's time range [{t[0]:.3f}, {t[-1]:.3f}] s."
        )
    i0 = int(np.argmin(np.abs(t - stride_start_time_s)))
    i1 = int(np.argmin(np.abs(t - stride_end_time_s)))
    print(f"  Clip covers t={t[0]:.3f} s to t={t[-1]:.3f} s ({len(t)} "
          f"samples); using the stride from t={t[i0]:.3f} s to "
          f"t={t[i1]:.3f} s ({i1 - i0 + 1} samples).")

    # Make the stride periodic (last sample == first) by subtracting a
    # linear ramp, then rotate it so it begins at the knee's maximum
    # (extension-most) value instead of the window's start. Being periodic,
    # the rotation needs no data from outside the window.
    t_out = t[i0:i1 + 1] - t[i0]
    ramp = t_out / t_out[-1]
    knee_abs = knee_deg_raw[i0:i1 + 1] - (knee_deg_raw[i1] - knee_deg_raw[i0]) * ramp
    ankle_abs = ankle_deg_raw[i0:i1 + 1] - (ankle_deg_raw[i1] - ankle_deg_raw[i0]) * ramp
    if START_AT_KNEE_MAX:
        m = int(np.argmax(knee_abs[:-1]))
        order = np.concatenate([np.arange(m, len(t_out) - 1), np.arange(0, m)])
        order = np.append(order, order[0])
        knee_abs, ankle_abs = knee_abs[order], ankle_abs[order]
        print(f"  Trajectory rotated to start at the knee maximum "
              f"(t={t[i0] + t_out[m]:.3f} s of the clip, "
              f"knee={knee_abs[0]:+.2f} deg).")

    knee_start_deg = float(knee_abs[0])
    ankle_start_deg = float(ankle_abs[0])
    knee_rel_deg = knee_abs - knee_abs[0]
    ankle_rel_deg = ankle_abs - ankle_abs[0]

    # Tile the stride stride_count times, dropping each repeat's duplicate
    # boundary sample.
    if stride_count > 1:
        T = t_out[-1]
        t_out = np.concatenate(
            [t_out[:-1] + k * T for k in range(stride_count)] + [[stride_count * T]])
        knee_rel_deg = np.concatenate(
            [knee_rel_deg[:-1]] * stride_count + [knee_rel_deg[-1:]])
        ankle_rel_deg = np.concatenate(
            [ankle_rel_deg[:-1]] * stride_count + [ankle_rel_deg[-1:]])

    return t_out, knee_rel_deg, knee_start_deg, ankle_rel_deg, ankle_start_deg


print(f"Loading gait-cycle reference from: {ANGLE_CSV_PATH}")
(t_raw, knee_rel_deg_raw, knee_start_deg_raw,
 ankle_rel_deg_raw, ankle_start_deg_raw) = load_gait_cycle_both(
    ANGLE_CSV_PATH, STRIDE_START_TIME_S, STRIDE_END_TIME_S, STRIDE_COUNT)

# Apply each joint's dataset-sign correction once, here, so the spline,
# the joint-limit check, and the homing target all inherit it consistently.
knee_rel_deg = KNEE_DATASET_TO_ENCODER_SIGN * knee_rel_deg_raw
knee_start_deg = KNEE_DATASET_TO_ENCODER_SIGN * knee_start_deg_raw
ankle_rel_deg = ANKLE_DATASET_TO_ENCODER_SIGN * ankle_rel_deg_raw
ankle_start_deg = ANKLE_DATASET_TO_ENCODER_SIGN * ankle_start_deg_raw
print(f"  Knee start (auto-detected from dataset @ t=0):  "
      f"{knee_start_deg:+.2f} deg")
print(f"  Ankle start (auto-detected from dataset @ t=0):  "
      f"{ankle_start_deg:+.2f} deg")

# KNEE_TRAJECTORY_START_DEG / ANKLE_TRAJECTORY_START_DEG: explicit override
# of where the trajectory starts, in place of the dataset's own auto-
# detected start angle. Since knee_rel_deg[0] / ankle_rel_deg[0] are always
# exactly 0 by construction (see load_gait_cycle_both() above), the
# trajectory's absolute starting value IS start_deg -- so overriding
# start_deg here shifts the WHOLE shape up/down as a block, unchanged in
# time, to begin at exactly the requested value.
if KNEE_TRAJECTORY_START_DEG is not None:
    print(f"  Knee start OVERRIDDEN by KNEE_TRAJECTORY_START_DEG: "
          f"{knee_start_deg:+.2f} deg (dataset) -> "
          f"{KNEE_TRAJECTORY_START_DEG:+.2f} deg")
    knee_start_deg = float(KNEE_TRAJECTORY_START_DEG)
if ANKLE_TRAJECTORY_START_DEG is not None:
    print(f"  Ankle start OVERRIDDEN by ANKLE_TRAJECTORY_START_DEG: "
          f"{ankle_start_deg:+.2f} deg (dataset) -> "
          f"{ANKLE_TRAJECTORY_START_DEG:+.2f} deg")
    ankle_start_deg = float(ANKLE_TRAJECTORY_START_DEG)

traj_duration = t_raw[-1] * TIME_SCALE
print(f"Loaded {STRIDE_COUNT} stride(s): {len(t_raw)} samples, "
      f"{t_raw[-1]:.3f} s of recorded data at real speed -> "
      f"{traj_duration:.3f} s at TIME_SCALE={TIME_SCALE:.2f}.")

# ---------------------------------------------------------------------------
# STEP 2 -- Fit cubic splines: Pd(t) [deg] and Vd(t) [deg/s], one pair per
# joint. Vd(t) is logged only -- the PID control law never uses it (unlike
# the impedance-control scripts).
# ---------------------------------------------------------------------------
t_scaled = t_raw * TIME_SCALE

knee_pos_spline = CubicSpline(t_scaled, knee_rel_deg, bc_type="not-a-knot")
knee_vel_spline = knee_pos_spline.derivative()
ankle_pos_spline = CubicSpline(t_scaled, ankle_rel_deg, bc_type="not-a-knot")
ankle_vel_spline = ankle_pos_spline.derivative()


# ---------------------------------------------------------------------------
# STEP 3 -- Joint object holding each joint's settings and runtime state
# ---------------------------------------------------------------------------
@dataclass
class Joint:
    id: int
    label: str
    pid: PID
    max_current_a: float
    limit_min_deg: float
    limit_max_deg: float
    abort_error_deg: float
    start_deg: float
    pos_spline: CubicSpline
    vel_spline: CubicSpline
    as5048b_addr: int
    mag_offset: float
    encoder_to_motor_sign: float
    gear_ratio_factor: float
    homing_velocity_rad_s: float
    home_tolerance_deg: float
    home_timeout_s: float
    p_prev_deg: float = 0.0    # last known-good actual angle (glitch-reject reuse)
    clamp_count: int = 0
    i2c_fail_count: int = 0
    i2c_fail_streak: int = 0
    i2c_fail_max_streak: int = 0
    i2c_last_good_deg: float = 0.0
    i2c_has_good_reading: bool = False
    wrap_reject_count: int = 0
    wrap_reject_streak: int = 0
    wrap_reject_max_streak: int = 0
    last_agc: int = field(init=False, default=-1)
    log: dict = field(default_factory=lambda: {k: [] for k in (
        "t", "dt", "reference_deg", "vd_degs", "actual_deg",
        "error_deg", "p_term", "i_term", "d_term",
        "current_cmd", "current_clamped", "tau_equivalent", "torque_nm")})


KNEE_JOINT = Joint(
    KNEE_CAN_ID, "Knee",
    PID(KNEE_KP, KNEE_KI, KNEE_KD, KNEE_INTEGRAL_LIMIT_DEG_S),
    KNEE_MAX_CURRENT_A, KNEE_LIMIT_MIN_DEG, KNEE_LIMIT_MAX_DEG,
    KNEE_ABORT_ERROR_DEG, knee_start_deg, knee_pos_spline, knee_vel_spline,
    KNEE_AS5048B_ADDR, KNEE_MAGNETIC_ENCODER_OFFSET,
    KNEE_ENCODER_TO_MOTOR_SIGN, KNEE_GEAR_RATIO_FACTOR,
    KNEE_HOMING_VELOCITY_RAD_S, KNEE_HOME_TOLERANCE_DEG, KNEE_HOME_TIMEOUT_S,
)
ANKLE_JOINT = Joint(
    ANKLE_CAN_ID, "Ankle",
    PID(ANKLE_KP, ANKLE_KI, ANKLE_KD, ANKLE_INTEGRAL_LIMIT_DEG_S),
    ANKLE_MAX_CURRENT_A, ANKLE_LIMIT_MIN_DEG, ANKLE_LIMIT_MAX_DEG,
    ANKLE_ABORT_ERROR_DEG, ankle_start_deg, ankle_pos_spline, ankle_vel_spline,
    ANKLE_AS5048B_ADDR, ANKLE_MAGNETIC_ENCODER_OFFSET,
    ANKLE_ENCODER_TO_MOTOR_SIGN, ANKLE_GEAR_RATIO_FACTOR,
    ANKLE_HOMING_VELOCITY_RAD_S, ANKLE_HOME_TOLERANCE_DEG, ANKLE_HOME_TIMEOUT_S,
)

# Only the joints whose *_CONTROL switch is 1 are driven this run -- see
# "PER-JOINT ENABLE SWITCHES" in the module docstring. Excluding a joint
# here means it's never homed, never tracked, and MOTORS below never
# registers its CAN ID, so its motor is never even woken.
JOINTS = [joint for enabled, joint in
          ((KNEE_CONTROL, KNEE_JOINT), (ANKLE_CONTROL, ANKLE_JOINT)) if enabled]
MOTORS = {joint.id: MOTOR_TYPE for joint in JOINTS}

if not JOINTS:
    print("[startup] KNEE_CONTROL=0 and ANKLE_CONTROL=0 -- nothing to "
          "drive. Exiting without opening the CAN or I2C buses.")
    raise SystemExit(0)

for joint in JOINTS:
    print(f"[startup] {joint.label}: CAN_ID={joint.id}  "
          f"PID(KP={joint.pid.kp}, KI={joint.pid.ki}, KD={joint.pid.kd})  "
          f"MAX_CURRENT_A={joint.max_current_a:.1f}  "
          f"limits=[{joint.limit_min_deg:+.1f}, {joint.limit_max_deg:+.1f}] deg  "
          f"ABORT_ERROR_DEG={joint.abort_error_deg:.1f}")
    print(f"           AS5048B addr=0x{joint.as5048b_addr:02X}  "
          f"offset={joint.mag_offset:+.2f} deg  "
          f"ENCODER_TO_MOTOR_SIGN={joint.encoder_to_motor_sign:+.1f}  "
          f"homing_vel={joint.homing_velocity_rad_s:.2f} rad/s")


# ---------------------------------------------------------------------------
# STEP 4 -- Pre-flight joint-limit check, before the motors are powered
# ---------------------------------------------------------------------------
def check_joint_limits(joint):
    """Compare one joint's trajectory's full range of motion against its
    hardware limits. If it doesn't fit and AUTO_FIT_TO_LIMITS is True,
    shift the whole trajectory to fit and say so; otherwise abort. Runs
    regardless of whether joint.start_deg came from the dataset or from a
    KNEE_TRAJECTORY_START_DEG / ANKLE_TRAJECTORY_START_DEG override above --
    this is the safety net that catches a start point (either source) that
    doesn't actually fit the hardware.

    A cubic spline's true min/max within an interval can only occur at a
    knot or where its derivative is zero, so the dense grid below is
    unioned with every root of vel_spline inside [0, traj_duration].
    """
    t_grid = np.linspace(0.0, traj_duration, 2000)
    crit_t = joint.vel_spline.roots(extrapolate=False)
    crit_t = crit_t[(crit_t >= 0.0) & (crit_t <= traj_duration)]
    t_check = np.union1d(t_grid, crit_t)
    rel_deg = joint.pos_spline(t_check)
    abs_min = joint.start_deg + rel_deg.min()
    abs_max = joint.start_deg + rel_deg.max()
    span = abs_max - abs_min
    limit_span = joint.limit_max_deg - joint.limit_min_deg

    fit_min_deg = joint.limit_min_deg
    fit_max_deg = joint.limit_max_deg

    if abs_min < fit_min_deg or abs_max > fit_max_deg:
        if not AUTO_FIT_TO_LIMITS:
            raise ValueError(
                f"{joint.label} trajectory would command "
                f"[{abs_min:+.1f}, {abs_max:+.1f}] deg, which falls "
                f"outside the joint's hardware limits "
                f"[{joint.limit_min_deg:+.1f}, "
                f"{joint.limit_max_deg:+.1f}] deg, and "
                f"AUTO_FIT_TO_LIMITS is False. Aborting before the "
                f"motor moves."
            )
        if span > limit_span:
            raise ValueError(
                f"{joint.label} trajectory spans {span:.1f} deg, wider "
                f"than the joint's own {limit_span:.1f} deg range "
                f"[{joint.limit_min_deg:+.1f}, "
                f"{joint.limit_max_deg:+.1f}] -- no shift can make "
                f"this fit. Aborting before the motor moves."
            )
        shift = (fit_min_deg - abs_min if abs_min < fit_min_deg
                 else fit_max_deg - abs_max)
        print(f"  [auto-fit] {joint.label}: reference shifted "
              f"{shift:+.2f} deg to fit inside "
              f"[{fit_min_deg:+.1f}, {fit_max_deg:+.1f}] deg "
              f"(was [{abs_min:+.1f}, {abs_max:+.1f}] deg).")
        joint.start_deg += shift
        abs_min += shift
        abs_max += shift

    print(f"  [limit-check] {joint.label}: trajectory range "
          f"[{abs_min:+.1f}, {abs_max:+.1f}] deg is within hardware "
          f"limits [{joint.limit_min_deg:+.1f}, "
          f"{joint.limit_max_deg:+.1f}] deg "
          f"(clearance from each hard limit: "
          f"{abs_min - joint.limit_min_deg:.1f} deg / "
          f"{joint.limit_max_deg - abs_max:.1f} deg).")


for joint in JOINTS:
    check_joint_limits(joint)


# ---------------------------------------------------------------------------
# STEP 5 -- Open the CAN bus and the shared magnetic-encoder I2C bus
# ---------------------------------------------------------------------------
actuators = ActuatorGroup.from_dict(MOTORS, exit_manually=True)
print("[startup] CAN bus opened and both motors registered with EPICally "
      "Powerful.")
mag_bus = SMBus(I2C_BUS)
print(f"[startup] I2C bus {I2C_BUS} opened for both AS5048B magnetic "
      f"encoders.")


def lifecycle(code, can_id):
    """Send the motor's power-on (0xFC) or power-off (0xFD) message to
    one specific CAN ID."""
    actuators.bus.send(
        can.Message(arbitration_id=can_id, data=[0xFF] * 7 + [code],
                    is_extended_id=True)
    )


def enforce_hard_limits(joint, actual_deg, context):
    """Immediately halts BOTH motors and raises if this joint's magnetic
    encoder reads outside its own [limit_min_deg, limit_max_deg] -- no
    margin, no tolerance, no exceptions. Called every tick, both during
    homing and during the PID loop, as the FIRST thing done with a
    freshly read actual_deg. An exoskeleton with only one of its two
    joints still being actively driven is not a safe state, so both
    motors are zeroed here regardless of which joint faulted."""
    if joint.limit_min_deg <= actual_deg <= joint.limit_max_deg:
        return
    print(f"\n  [HARD LIMIT VIOLATION] {joint.label}: magnetic encoder "
          f"reads {actual_deg:+.2f} deg during {context}, outside "
          f"[{joint.limit_min_deg:+.1f}, {joint.limit_max_deg:+.1f}] deg. "
          f"Commanding zero current/velocity on BOTH joints immediately "
          f"and exiting.")
    try:
        for motor_id in MOTORS:
            actuators.set_torque(motor_id, 0.0)
            actuators.set_velocity(motor_id, 0.0, 0)
    except Exception:
        pass
    raise RuntimeError(
        f"{joint.label} hard limit violated: {actual_deg:+.2f} deg is "
        f"outside [{joint.limit_min_deg:+.1f}, {joint.limit_max_deg:+.1f}] "
        f"deg during {context} -- this must never happen; aborting "
        f"immediately."
    )


def home_joints(joints, bus):
    """Drive BOTH joints, directly from wherever they are sitting when
    the script starts, to their own dataset-derived trajectory starting
    angle using Servo Velocity Loop Mode -- interleaved in one loop (both
    joints stepped every tick), no separate "home to zero first" step.
    Direction is chosen automatically every tick from each joint's own
    live error -- magnitude |joint.homing_velocity_rad_s|, direction
    toward joint.start_deg -- exactly the technique used in the step-test
    scripts' home_joint(). Each joint's arrival is judged independently
    against its own home_tolerance_deg/home_timeout_s; a joint that
    arrives simply holds (zero velocity) and waits there while the other
    keeps being driven. This function returns once BOTH have arrived.
    """
    print(f"  [home] Homing knee + ankle to their dataset start angles "
          f"(Velocity Loop Mode, direction chosen automatically from "
          f"each joint's own live error)...")
    clock_h = TimedLoop(rate=HOME_LOOP_HZ)
    t_home0 = time.perf_counter()
    home_tick = 0
    arrived = {joint.id: False for joint in joints}
    HOME_PRINT_EVERY = 5
    while clock_h():
        all_arrived = True
        for joint in joints:
            now_deg = read_actual_angle_deg_safe(bus, joint)
            enforce_hard_limits(joint, now_deg, "homing")

            err = wrap_deg(joint.start_deg - now_deg)

            if abs(err) <= joint.home_tolerance_deg:
                if not arrived[joint.id]:
                    arrived[joint.id] = True
                    print(f"  [home] {joint.label}: reached {now_deg:+.2f} "
                          f"deg (target {joint.start_deg:+.2f} deg, error "
                          f"{err:+.2f} deg) -- holding, waiting for the "
                          f"other joint if it hasn't arrived yet.")
                # Already arrived -- hold still (zero velocity) rather
                # than continuing to bang-bang at full homing velocity
                # while waiting for the other joint.
                vel_cmd = 0.0
            else:
                arrived[joint.id] = False
                all_arrived = False
                if time.perf_counter() - t_home0 > joint.home_timeout_s:
                    raise RuntimeError(
                        f"{joint.label} failed to home within "
                        f"{joint.home_timeout_s:.1f} s (still {abs(err):.2f} "
                        f"deg from {joint.start_deg:+.2f} deg per the "
                        f"magnetic encoder) -- aborting before the control "
                        f"loop starts. Verify that joint's "
                        f"MAGNETIC_ENCODER_OFFSET is correct, verify its "
                        f"|homing_velocity_rad_s| isn't too small to make "
                        f"progress, and verify by hand that the shaft's "
                        f"ACTUAL position isn't genuinely far from "
                        f"joint.start_deg or physically obstructed."
                    )
                vel_cmd = joint.encoder_to_motor_sign * math.copysign(
                    abs(joint.homing_velocity_rad_s), err)

            actuators.set_velocity(joint.id, vel_cmd, 0)

            if home_tick % HOME_PRINT_EVERY == 0:
                print(f"  [home] {joint.label}: "
                      f"t={time.perf_counter() - t_home0:5.2f}s  "
                      f"mag={now_deg:+7.2f} deg  err={err:+7.2f} deg  "
                      f"vel_cmd={vel_cmd:+.2f} rad/s")

        if all_arrived:
            break
        home_tick += 1

    print(f"  [home] Sending {NUM_HOME_STOP_COMMANDS} zero-velocity stop "
          f"commands to both joints "
          f"({HOME_STOP_COMMAND_INTERVAL_S * 1000:.0f} ms apart)...")
    for _ in range(NUM_HOME_STOP_COMMANDS):
        for joint in joints:
            actuators.set_velocity(joint.id, 0.0, 0)
        time.sleep(HOME_STOP_COMMAND_INTERVAL_S)
    for joint in joints:
        final_deg = read_actual_angle_deg_safe(bus, joint)
        enforce_hard_limits(joint, final_deg, "post-homing stop")
        joint.p_prev_deg = final_deg
        print(f"  [home] {joint.label}: stopped. Final magnetic-encoder "
              f"reading = {final_deg:+.2f} deg (target "
              f"{joint.start_deg:+.2f} deg).")


def settle_before_trajectory(joints, hold_duration_s, bus):
    """Both joints have already arrived at their own trajectory start
    angle (home_joints() stopped them there with a zero-velocity burst) --
    this just holds that stop (zero velocity, no further driving needed)
    for hold_duration_s seconds before handing off to the PID
    controllers, so the PID loops don't engage the instant homing stops.
    NOT a second homing/bang-bang pass, and NOT logged -- only updates
    joint.p_prev_deg so the PID phase's own glitch-jump guard has a
    valid "last known-good" reading to compare its very first tick
    against. Both joints' PID loops then start at exactly the same
    instant, right after this function returns.
    """
    print(f"  [settle] Holding both joints at their trajectory start "
          f"angle for {hold_duration_s:.2f}s before handing off to the "
          f"PID controllers...")
    clock_settle = TimedLoop(rate=HOME_LOOP_HZ)
    t0 = time.perf_counter()
    tick = 0
    SETTLE_PRINT_EVERY = max(1, HOME_LOOP_HZ // 2)
    while clock_settle():
        t_wall = time.perf_counter() - t0
        if t_wall > hold_duration_s:
            break
        for joint in joints:
            now_deg = read_actual_angle_deg_safe(bus, joint)
            enforce_hard_limits(joint, now_deg, "pre-trajectory settle")
            actuators.set_velocity(joint.id, 0.0, 0)
            joint.p_prev_deg = now_deg

            if tick % SETTLE_PRINT_EVERY == 0:
                print(f"  [settle] {joint.label}: t={t_wall:5.2f}s  "
                      f"mag={now_deg:+7.2f} deg")
        tick += 1

    for joint in joints:
        final_deg = read_actual_angle_deg_safe(bus, joint)
        enforce_hard_limits(joint, final_deg, "end of pre-trajectory settle")
        joint.p_prev_deg = final_deg
    print("  [settle] Settle complete -- handing off to the PID "
          "controllers now.")


try:
    for motor_id in MOTORS:
        lifecycle(0xFC, motor_id)
    print("[run] Sent motor wake-up (0xFC) frame(s). Waiting for fresh "
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
    print("[run] Motors awake and telemetry flowing.")

    print("\n[run] --- Phase 1: homing (both joints, directly to the "
          "trajectory's own start angle) ---")
    home_joints(JOINTS, mag_bus)

    print("\n[run] --- Phase 2: pre-trajectory settle (PID not yet "
          "engaged) ---")
    settle_before_trajectory(JOINTS, PRE_TRAJ_HOLD_S, mag_bus)

    print(f"\n  [HANDOFF] control handed to both PID controllers now -- "
          f"CSV/plot logging starts from this instant (t=0).\n")

    for joint in JOINTS:
        joint.pid.reset()

    print(f"[run] --- Phase 3: PID trajectory tracking "
          f"(traj_duration={traj_duration:.2f}s, {LOOP_HZ} Hz) --- Ctrl+C "
          f"stops.\n")

    clock = TimedLoop(rate=LOOP_HZ)
    t0_pid = time.perf_counter()   # fresh time origin -- t=0 is the handoff instant
    total_duration = traj_duration
    t_prev = {joint.id: t0_pid for joint in JOINTS}
    i = 0
    abort_reason = None

    PRINT_EVERY = max(1, LOOP_HZ // 5)

    while clock():
        t_wall = time.perf_counter() - t0_pid
        if t_wall > total_duration:
            break

        abort_reasons_this_tick = []

        for joint in JOINTS:
            reference_deg = joint.start_deg + float(joint.pos_spline(t_wall))
            vd_degs = float(joint.vel_spline(t_wall))

            actual_deg = read_actual_angle_deg_safe(mag_bus, joint)
            now_t = time.perf_counter()

            enforce_hard_limits(joint, actual_deg, "PID control loop")

            delta_deg = actual_deg - joint.p_prev_deg
            jump_bad = abs(delta_deg) > GLITCH_JUMP_DEG

            if jump_bad:
                joint.wrap_reject_count += 1
                joint.wrap_reject_streak += 1
                joint.wrap_reject_max_streak = max(
                    joint.wrap_reject_max_streak, joint.wrap_reject_streak)
                if joint.wrap_reject_streak > MAX_CONSECUTIVE_WRAP_REJECTS:
                    abort_reasons_this_tick.append(
                        f"{joint.label}: {joint.wrap_reject_streak} "
                        f"consecutive implausible position jump(s) "
                        f"(> {GLITCH_JUMP_DEG:.0f} deg/tick) -- the "
                        f"magnetic encoder reading is not trustworthy "
                        f"right now."
                    )
                actual_deg = joint.p_prev_deg
            else:
                joint.wrap_reject_streak = 0

            joint.p_prev_deg = actual_deg

            dt = now_t - t_prev[joint.id]
            if dt <= 0.0:
                dt = 1.0 / LOOP_HZ
            t_prev[joint.id] = now_t

            current_cmd, p_term, i_term, d_term, error_deg = joint.pid.update(
                reference_deg, actual_deg, dt)

            # ENCODER_TO_MOTOR_SIGN: flip current + all logged terms
            # together, so everything logged/printed/sent agrees --
            # error_deg itself is a pure position quantity and is NOT
            # flipped. Same convention as both step-test scripts.
            current_cmd *= joint.encoder_to_motor_sign
            p_term *= joint.encoder_to_motor_sign
            i_term *= joint.encoder_to_motor_sign
            d_term *= joint.encoder_to_motor_sign

            current_clamped = current_cmd
            clamped_flag = False
            if abs(current_clamped) > joint.max_current_a:
                current_clamped = math.copysign(joint.max_current_a,
                                                  current_clamped)
                clamped_flag = True
                joint.clamp_count += 1

            actuators.set_torque(joint.id, current_clamped)   # amperes

            if joint.abort_error_deg > 0.0 and abs(error_deg) > joint.abort_error_deg:
                abort_reasons_this_tick.append(
                    f"{joint.label} tracking error reached "
                    f"{abs(error_deg):.1f} deg (limit "
                    f"{joint.abort_error_deg:.1f} deg) -- the joint is no "
                    f"longer following the trajectory."
                )

            joint.log["t"].append(t_wall)
            joint.log["dt"].append(dt)
            joint.log["reference_deg"].append(reference_deg)
            joint.log["vd_degs"].append(vd_degs)
            joint.log["actual_deg"].append(actual_deg)
            joint.log["error_deg"].append(error_deg)
            joint.log["p_term"].append(p_term)
            joint.log["i_term"].append(i_term)
            joint.log["d_term"].append(d_term)
            joint.log["current_cmd"].append(current_clamped)
            joint.log["current_clamped"].append(clamped_flag)
            joint.log["tau_equivalent"].append(
                current_clamped * KT * joint.gear_ratio_factor)
            joint.log["torque_nm"].append(KT * current_clamped)

        if abort_reasons_this_tick:
            abort_reason = " | ".join(abort_reasons_this_tick)
            print(f"\n  [ABORT] {abort_reason}")
            print("  Stopping the trajectory now (both joints). The "
                  "CSV/plots below still show everything logged up to "
                  "this point.")
            break

        if i % PRINT_EVERY == 0:
            for joint in JOINTS:
                try:
                    agc, diag = _as5048b_read_health(mag_bus, joint.as5048b_addr)
                    joint.last_agc = agc
                except OSError:
                    pass

                d = joint.log
                agc_str = f"{joint.last_agc}" if joint.last_agc >= 0 else "?"
                clamp_str = " [CLAMPED]" if d["current_clamped"][-1] else ""
                print(f"t={t_wall:5.2f}s  {joint.label:5s} (id={joint.id:3d})  "
                      f"ref={d['reference_deg'][-1]:+7.2f} deg  "
                      f"act={d['actual_deg'][-1]:+7.2f} deg  "
                      f"err={d['error_deg'][-1]:+6.2f} deg  "
                      f"P={d['p_term'][-1]:+6.3f}  I={d['i_term'][-1]:+6.3f}  "
                      f"D={d['d_term'][-1]:+6.3f}  "
                      f"I_cmd={d['current_cmd'][-1]:+5.2f} A{clamp_str}  "
                      f"AGC={agc_str}")
        i += 1

        if getattr(actuators.notifier, "exception", None):
            print("RX thread died:", actuators.notifier.exception)
            break

    print("\n[run] Trajectory finished.")

except KeyboardInterrupt:
    print("\n[run] Ctrl+C -- stopping.")

finally:
    print("\n[shutdown] Stopping and releasing both motors...")
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
    print("[shutdown] Motors limp; verify by hand before approaching.")

    n_ticks = len(JOINTS[0].log["t"])
    print(f"[shutdown] Logged {n_ticks} control tick(s).")
    if n_ticks > 1:
        elapsed = JOINTS[0].log["t"][-1] - JOINTS[0].log["t"][0]
        achieved = (n_ticks - 1) / elapsed if elapsed > 0 else float("nan")
        print(f"[shutdown] Achieved loop rate: {achieved:.1f} Hz "
              f"(nominal {LOOP_HZ} Hz, {100.0 * achieved / LOOP_HZ:.1f}%)")

    for joint in JOINTS:
        n_joint_ticks = len(joint.log["t"])
        if n_joint_ticks == 0:
            continue
        if joint.clamp_count > 0:
            print(f"  [warning] {joint.label}: current clamp engaged on "
                  f"{joint.clamp_count}/{n_joint_ticks} ticks "
                  f"({100.0 * joint.clamp_count / n_joint_ticks:.1f}%) -- "
                  f"consider lowering KP/KI if this seems large.")
        if joint.i2c_fail_count > 0:
            print(f"  [warning] {joint.label}: AS5048B I2C read failed on "
                  f"{joint.i2c_fail_count}/{n_joint_ticks} tick(s) (worst "
                  f"consecutive streak: {joint.i2c_fail_max_streak}).")
        if joint.wrap_reject_count > 0:
            print(f"  [warning] {joint.label}: rejected "
                  f"{joint.wrap_reject_count}/{n_joint_ticks} implausible "
                  f"position jump(s) (worst consecutive streak: "
                  f"{joint.wrap_reject_max_streak}).")

    # --- CSV export: one file per joint, every logged quantity as its
    # own column against time_s -- meant for re-plotting elsewhere. ------
    if n_ticks == 0:
        print("[shutdown] No telemetry was logged at all -- no CSV or "
              "plots were produced.")
    else:
        csv_columns = ["t", "dt", "reference_deg", "vd_degs", "actual_deg",
                       "error_deg", "p_term", "i_term",
                       "d_term", "current_cmd", "current_clamped",
                       "tau_equivalent", "torque_nm"]
        csv_header = ["time_s", "dt_s", "reference_deg", "reference_vel_degs",
                      "actual_deg", "error_deg",
                      "p_term_A", "i_term_A", "d_term_A", "current_cmd_A",
                      "current_clamped", "tau_equivalent_Nm", "torque_Nm"]
        for joint in JOINTS:
            if not joint.log["t"]:
                continue
            csv_path = os.path.join(RUN_DIR,
                                     f"{joint.label.lower()}_trajectory_data.csv")
            with open(csv_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(csv_header)
                # Cosmetic only: knee angles are written flexion-positive.
                _s = -1.0 if joint is KNEE_JOINT else 1.0
                for row in zip(*(
                        [_s * v for v in joint.log[col]]
                        if col in ("reference_deg", "actual_deg")
                        else joint.log[col] for col in csv_columns)):
                    writer.writerow(row)
            print(f"[shutdown] {joint.label} CSV data saved to: {csv_path}")

        if not GENERATE_PLOTS:
            print("[shutdown] Plot generation disabled (GENERATE_PLOTS = "
                  "False) -- no figures were produced.")
        elif not any(joint.log["t"] for joint in JOINTS):
            print("[shutdown] No trajectory-phase telemetry was logged -- "
                  "skipping plots.")
        else:
            def _new_combined_figure(joints):
                """One figure, one subplot per joint (knee on top, ankle
                on the bottom), sharing a time axis."""
                fig, axes = plt.subplots(len(joints), 1,
                                          figsize=(10, 4.5 * len(joints)),
                                          sharex=True)
                if len(joints) == 1:
                    axes = [axes]
                axes[-1].set_xlabel("Time [s]")
                return fig, axes

            def _save_combined_figure(fig, title_suffix, filename):
                fig.suptitle(title_suffix)
                fig.tight_layout()
                out_path = os.path.join(RUN_DIR, filename)
                fig.savefig(out_path, dpi=300, bbox_inches="tight")
                print(f"[shutdown] Plot saved to: {out_path}")

            def plot_position_combined(joints):
                fig, axes = _new_combined_figure(joints)
                for ax, joint in zip(axes, joints):
                    d = joint.log
                    # Cosmetic only: knee is displayed flexion-positive.
                    s = -1.0 if joint is KNEE_JOINT else 1.0
                    ax.plot(d["t"], [s * v for v in d["reference_deg"]],
                            color="blue",
                            linewidth=1.8, label="Reference (dataset)")
                    ax.plot(d["t"], [s * v for v in d["actual_deg"]],
                            color="red",
                            linewidth=1.3,
                            label="Actual (magnetic encoder)")
                    ax.axhline(s * joint.limit_min_deg, color="black",
                               linewidth=1.0, linestyle="--", alpha=0.6,
                               label="Hardware limit")
                    ax.axhline(s * joint.limit_max_deg, color="black",
                               linewidth=1.0, linestyle="--", alpha=0.6)
                    ax.set_ylabel(f"{joint.label} angle [deg]")
                    ax.set_title(f"{joint.label} joint")
                    ax.legend(loc="best", fontsize=8)
                    ax.grid(True, alpha=0.3)
                _save_combined_figure(
                    fig, "Reference vs. actual trajectory (PID / Current "
                         "Loop Mode)", "position_response.png")

            def plot_error_combined(joints):
                fig, axes = _new_combined_figure(joints)
                for ax, joint in zip(axes, joints):
                    d = joint.log
                    ax.plot(d["t"], d["error_deg"], color="purple",
                            linewidth=1.2, label="Position error (ref - actual)")
                    ax.axhline(0.0, color="black", linewidth=0.8,
                               linestyle="--", alpha=0.5)
                    err_bound = max(
                        5.0, joint.abort_error_deg * 1.15,
                        1.15 * max((abs(v) for v in d["error_deg"]), default=0.0))
                    ax.set_ylim(-err_bound, err_bound)
                    if joint.abort_error_deg > 0.0:
                        ax.axhline(joint.abort_error_deg, color="red",
                                   linewidth=0.8, linestyle=":", alpha=0.5,
                                   label="ABORT_ERROR_DEG")
                        ax.axhline(-joint.abort_error_deg, color="red",
                                   linewidth=0.8, linestyle=":", alpha=0.5)
                    ax.set_ylabel(f"{joint.label} position error [deg]")
                    ax.set_title(f"{joint.label} joint")
                    ax.legend(loc="best", fontsize=8)
                    ax.grid(True, alpha=0.3)
                _save_combined_figure(
                    fig, "Position tracking error vs. time", "error.png")

            def plot_current_combined(joints):
                fig, axes = _new_combined_figure(joints)
                for ax, joint in zip(axes, joints):
                    d = joint.log
                    ax.plot(d["t"], d["current_cmd"], color="darkorange",
                            linewidth=1.2, label="Commanded current")
                    ax.axhline(0.0, color="black", linewidth=0.8,
                               linestyle="--", alpha=0.5)
                    ax.set_ylabel(f"{joint.label} current [A]")
                    ax.set_title(f"{joint.label} joint")
                    ax.legend(loc="best", fontsize=8)
                    ax.grid(True, alpha=0.3)
                _save_combined_figure(
                    fig, "PID output (current sent to Current Loop)",
                    "current_command.png")

            def plot_torque_combined(joints):
                fig, axes = _new_combined_figure(joints)
                for ax, joint in zip(axes, joints):
                    d = joint.log
                    ax.plot(d["t"], d["torque_nm"], color="green",
                            linewidth=1.2, label="Torque (k_t * current)")
                    ax.axhline(0.0, color="black", linewidth=0.8,
                               linestyle="--", alpha=0.5)
                    ax.set_ylabel(f"{joint.label} torque [N*m]")
                    ax.set_title(f"{joint.label} joint")
                    ax.legend(loc="best", fontsize=8)
                    ax.grid(True, alpha=0.3)
                _save_combined_figure(
                    fig, "Applied torque vs. time (k_t * control current)",
                    "torque.png")

            joints_with_data = [joint for joint in JOINTS if joint.log["t"]]
            if len(joints_with_data) < len(JOINTS):
                missing = [joint.label for joint in JOINTS
                           if joint not in joints_with_data]
                print(f"[shutdown] No trajectory-phase telemetry for: "
                      f"{', '.join(missing)} -- skipping the combined "
                      f"plots (they need both joints' data).")
            else:
                plot_position_combined(JOINTS)
                plot_error_combined(JOINTS)
                plot_current_combined(JOINTS)
                plot_torque_combined(JOINTS)

        if SHOW_PLOTS:
            try:
                plt.show()
            except Exception:
                pass

print("\nDone.")