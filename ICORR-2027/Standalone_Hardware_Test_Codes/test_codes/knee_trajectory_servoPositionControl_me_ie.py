#!/usr/bin/env python3
"""
knee_trajectory_servoPositionControl_ema_me.py -- Knee-only trajectory
controller that drives the CubeMars driver's own Servo Position Loop Mode
(Control Mode ID 4, actuators.set_position()) instead of computing torque,
while still using an external AS5048B magnetic encoder (read over I2C) to
actively steer the commanded position. "_me" = Magnetic Encoder.

This is the Position-Loop-Mode counterpart of
knee_trajectory_impedanceControl_ema_me.py -- identical dataset loading,
homing, magnetic-encoder pipeline, guards, and reporting; only the main
trajectory-tracking control law differs (see CONTROL LAW below).

CONTROL LAW
The CubeMars driver's Position Loop Mode closes its own position loop
INSIDE THE FIRMWARE, against the motor's own internal encoder only -- the
CAN frame has no field for substituting an external sensor, so the
magnetic encoder can never replace that inner loop directly. What this
script does instead is what home_joint() already does for homing: an
OUTER correction loop, running here on the Pi, that steers the absolute
target handed to set_position() every tick using the magnetic encoder.

Every tick (at LOOP_HZ):
    err_deg = Pd_deg - P_deg          (desired minus magnetic-encoder actual,
                                        in the joint's own degree frame)
    correction_deg = clamp(err_deg, -ABORT_ERROR_DEG, +ABORT_ERROR_DEG)
    target_internal_deg = internal_now_deg + correction_deg * GEAR_RATIO_FACTOR
    actuators.set_position(id, target_internal_deg, 0, 0, degrees=True)

This is a pure, gain-free proportional correction: unlike impedance
control (which needs a spring constant, Kp in N*m/rad, to convert an angle
error into a torque), commanding a position needs no unit conversion -- a
1 degree error becomes exactly a 1 degree nudge to the target. Anchoring
the correction to internal_now_deg (the internal encoder's CURRENT
reading, not a fixed value recorded once at homing) means it continuously
cancels whatever disagreement currently exists between the internal and
magnetic encoders (backlash, drift, etc.), every tick. CubeMars servo
mode is VESC-derived, and VESC's Position Loop Mode is a pure position
PID: the current it applies is computed from this same error and scaled
to max current -- so the clamp above does not merely guard against a
glitch, it directly caps the torque the driver will ever apply. It is
set equal to ABORT_ERROR_DEG (not a separate constant) so it never binds
during legitimate tracking -- anything past that is already a fault the
runaway guard below stops on anyway.

SAFETY NOTE: the magnetic encoder is the ACTIVE feedback signal here,
exactly as it already is in the impedance-control script -- there it
drives tau = Kp*(Pd-P) + Kd*(Vd-Vfilt) directly; here it drives the
position correction directly. It was never passive logging in either
script. What actually changes is how a given error gets turned into
motion: impedance control turns a position error into a TORQUE, which is
inherently damped and bounded by the motor's own current limit and
physical inertia; Position Loop Mode instead turns the (clamped) error
into an absolute TARGET that the driver's own inner loop chases at
MAXIMUM SPEED AND ACCELERATION, with no damping of its own. So the same
magnetic-encoder glitch produces a more abrupt physical response here than
it would under impedance control -- which is why the existing glitch-jump
reject, disagreement reject, and I2C-fail-abort guards below deserve
extra attention on this script's first bench runs.

CALIBRATING MAGNETIC_ENCODER_OFFSET
MAGNETIC_ENCODER_OFFSET is a fixed, physical property of how the AS5048B
magnet is mounted relative to the joint's own hardstops. It has nothing
to do with the internal (motor) encoder's own zero point.
  1. With the motor powered, physically position the knee shaft at its
     true mechanical reference (the real full-extension hardstop,
     0 deg) by hand.
  2. Run this script with RUN_MODE = "calibrate_offset" (see
     run_offset_calibration() below) and read off "Raw AS5048B" -- that
     value IS MAGNETIC_ENCODER_OFFSET.
  3. Copy that number into MAGNETIC_ENCODER_OFFSET below, set RUN_MODE
     back to "trajectory", and re-run.
  4. Note the AGC value printed at the same time (AGC_AT_CALIBRATION) as
     a reference -- if the AGC drifts far from that later, the magnetic
     coupling may have changed and the offset should be re-measured.

MAGNETIC_ENCODER_SIGN is the constant that determines whether the
position correction is negative feedback (stable) or positive feedback (a
runaway). It must be verified by hand (motor passive, not
torque-commanded) before MAGNETIC_ENCODER_SIGN_VERIFIED is set to True.

HOMING
home_joint() drives the joint to the trajectory's start angle using
Position Loop Mode, then the joint is held there briefly while a start
position is measured -- see home_joint() and _home_and_capture_p0() for
details.

WHAT YOU SHOULD SEE WHEN YOU RUN IT
The motor wakes up, homes to the dataset's start angle, holds briefly
while the start position is measured, then tracks the gait trajectory
for one stride, printing a status line a few times a second. On exit
the motor is released and three plots are saved to a "results" folder
next to this script: position tracking, velocity, and position error.

Press Ctrl+C at any time -- the cleanup code still runs, the motor is
released safely, and the plots (from whatever was logged) are still
produced.
"""

# ===========================================================================
#  USER SETTINGS -- the fields you are most likely to want to change
# ===========================================================================

import os   # needed up front, since the dataset paths below use it

# --- Paths to the dataset files --------------------------------------------
ANGLE_CSV_PATH = "/home/legpi/test_codes/normal_walk_1-2/AB01_normal_walk_1_1-2_angle.csv"
GRF_CSV_PATH = "/home/legpi/test_codes/normal_walk_1-2/AB01_normal_walk_1_1-2_grf.csv"

# How many consecutive strides, starting from the first heel-strike, to
# send to the motor as one trajectory. This trial has 17 available
# strides (valid range 1 to 17).
STRIDE_COUNT = 5

# --- CAN ID ------------------------------------------------------------
KNEE_CAN_ID = 100     # CAN ID of the knee actuator

# --- Run mode --------------------------------------------------------------
# "trajectory": home the joint and run the gait trajectory.
# "calibrate_offset": read-only mode for measuring MAGNETIC_ENCODER_OFFSET
#     (see run_offset_calibration() below). Never commands motion, so it
#     is safe to run regardless of the shaft's position or whether
#     MAGNETIC_ENCODER_SIGN has been verified yet.
RUN_MODE = "trajectory"   # "trajectory" or "calibrate_offset"

# --- Control-loop rate ------------------------------------------------
# Also the sample rate the velocity filter below is designed around, and
# the rate the magnetic-encoder I2C reads happen at.
LOOP_HZ = 500

# --- Reference-trajectory sign correction ---------------------------------
# Flips the dataset's trajectory if its "more flexion" direction doesn't
# match this joint's direction once mounted. This does not affect the
# measured position -- see MAGNETIC_ENCODER_SIGN below for that.
KNEE_SIGN = +1.0

# --- Joint hardware limits, degrees --------------------------------------
# 0 deg = full extension, 120 deg = maximum flexion.
KNEE_LIMIT_MIN_DEG = 0.0
KNEE_LIMIT_MAX_DEG = 120.0

# --- What to do if the trajectory doesn't fit inside the joint limits -----
# True shifts the whole trajectory (not its shape) to fit inside the
# limits. False aborts instead if it doesn't already fit.
AUTO_FIT_TO_LIMITS = True

# How far inside the true hardware limits the homing target should sit,
# so ordinary sensor noise during homing doesn't trip the limit guard.
HOMING_STANDOFF_DEG = 8.0

# --- Homing --------------------------------------------------------------
# home_joint() drives the joint using Position Loop Mode
# (actuators.set_position()), which always moves at maximum speed and
# acceleration to whatever target it's given. To keep that safe,
# home_joint() never sends a large target -- every tick it re-reads the
# internal encoder's position and steps the target by at most
# HOME_MAX_STEP_ACTUATOR_DEG, in the direction that reduces the magnetic
# encoder's error. Arrival is judged from the magnetic encoder alone.
HOME_LOOP_HZ       = 500
HOME_TOLERANCE_DEG = 1.0     # how close counts as "arrived"
HOME_TIMEOUT_S     = 20.0     # abort homing if not arrived within this many seconds

# Maximum step size per tick while homing, in actuator-frame degrees.
HOME_MAX_STEP_ACTUATOR_DEG = 0.5

# --- Post-homing hold ------------------------------------------------------
# After homing, the last position is actively re-sent every tick for
# POST_HOME_SETTLE_S seconds (not a blind pause) while the start
# position is measured.
POST_HOME_SETTLE_S = 0.15
HOME_LIMIT_MARGIN_DEG = 2.0   # allowance for sensor noise around the hardware limits

# --- Trajectory shaping --------------------------------------------------
TIME_SCALE = 3.0   # stretches the whole trajectory by this factor

# --- Runaway guards / outer-loop correction clamp --------------------------
# Abort if the tracking error grows past this many degrees. Set to 0 to
# disable (not recommended on hardware).
#
# This is the ONLY error-magnitude threshold in the script -- it also
# doubles as the outer-loop correction's clamp: every tick the raw
# tracking error (Pd_deg - P_deg) is clamped to +/-ABORT_ERROR_DEG before
# being added onto the internal encoder's current reading as the Position
# Loop Mode target. Because that target is anchored to the internal
# encoder's CURRENT reading, the driver's own internal position error IS
# exactly this clamped correction. CubeMars servo mode is VESC-derived,
# and VESC's Position Loop Mode is a pure position PID: the current it
# applies is computed from that position error and scaled to max current
# (there is no separate motion-profile stage). So this threshold does not
# merely guard against a glitch or trigger an abort -- it also directly
# caps the TORQUE the driver will ever apply while tracking. A value much
# smaller than the trajectory's real tracking error (5 deg was tried and
# was far too small) silently throttles the whole controller: the joint
# falls behind while the driver still only ever sees a tiny error, so it
# never pulls harder, and the run eventually aborts anyway once the real
# error (which the clamp was hiding from the driver) exceeds this same
# threshold. Sharing one constant for both roles keeps that failure mode
# structurally impossible -- the clamp can never bind before the abort
# guard would have fired anyway. If tracking is still sluggish at a
# reasonable value here, the remaining knob is the DRIVER's own position
# PID gains in CubeMarsTool (the AK80-9 supports auto PID identification)
# -- they cannot be set over CAN, since set_position()'s kp/kd arguments
# are ignored in Servo mode.
ABORT_ERROR_DEG = 30.0

# --- Implausible-jump guard --------------------------------------------
# Any single-tick position jump larger than this is rejected -- the last
# known-good position is reused instead.
GLITCH_JUMP_DEG = 60.0

# How many consecutive rejected ticks are tolerated before aborting.
MAX_CONSECUTIVE_WRAP_REJECTS = 10

# --- Cross-sensor disagreement guard ---------------------------------------
# If the magnetic-encoder-derived position disagrees with the internal
# encoder's own position by more than this many degrees, the magnetic
# reading is rejected for that tick.
MAX_ENCODER_DISAGREEMENT_DEG = 15.0

# --- Transmission gear ratio ------------------------------------------------
# Ratio between the actuator's own output shaft and the true joint (e.g.
# an external belt drive). 1.0 means the magnetic encoder is mounted
# directly on the actuator's own output shaft. Shared by both
# INPUT_ENCODER modes below.
GEAR_RATIO_FACTOR = 4.61

# --- Position feedback source for the outer-loop correction ----------------
# "magnetic": del_theta is computed from the AS5048B magnetic encoder
#     (today's behavior, unchanged) -- err_deg = pd_deg - p_deg, where
#     p_deg is already in joint-frame degrees.
# "internal": del_theta is computed from the actuator's own internal
#     encoder instead -- err_deg = pd_deg - p_internal_deg, where
#     p_internal_deg is the internal encoder's reading converted into
#     joint-frame degrees by dividing by GEAR_RATIO_FACTOR (already
#     computed every tick regardless of this setting, for logging and
#     the disagreement guard). Either way, the resulting correction is
#     clamped to +/-ABORT_ERROR_DEG and then multiplied by
#     GEAR_RATIO_FACTOR again before being added to internal_now_deg, so
#     both modes share the exact same control-law structure -- only
#     which sensor drives err_deg changes.
# "magnetic": the magnetic encoder is read every tick, actively steers
#     homing and the control law above, and is protected by every guard
#     below (implausible-jump reject, disagreement reject, I2C-fail
#     abort). A disconnected/dismounted sensor is a fault in this mode.
# "internal": the internal encoder alone drives homing and the control
#     law -- it is self-sufficient on its own, so it never needs the
#     magnetic encoder. The magnetic encoder is still read best-effort
#     every tick and logged/plotted purely for comparison, but none of
#     the guards above apply to it and a disconnected/dismounted sensor
#     is NOT a fault in this mode -- it's fine to run with it unplugged.
INPUT_ENCODER = "internal"   # "magnetic" or "internal"

# --- Velocity low-pass filter settings ------------------------------------
# Exponential Moving Average (EMA) filter cutoff, tuned to this joint's
# own resonance. NOTE: unlike the impedance-control script, this filtered
# velocity is NOT fed into the control law below (Position Loop Mode takes
# no velocity/gain arguments) -- it is computed and logged for diagnostic
# comparison only.
CUTOFF_KNEE_HZ = 50.0

# --- Magnetic encoder (AS5048B) settings -----------------------------------
I2C_BUS = 1
AS5048B_ADDR = 0x40
ANGLE_HIGH_REG = 0xFE
ANGLE_LOW_REG = 0xFF
AGC_REG = 0xFA
DIAG_REG = 0xFB

# Offset between the AS5048B's raw reading and the joint's true zero --
# see the CALIBRATION section at the top of this file for how to
# measure it. A fixed mechanical constant; it does not update itself.
MAGNETIC_ENCODER_OFFSET = 167.5  # deg
AGC_AT_CALIBRATION = 13       # AGC value recorded at calibration time, for reference

# Sign of the magnetic encoder relative to the internal encoder. +1.0
# means both increase in the same physical direction; -1.0 means
# mirrored. Getting this wrong turns the position correction into
# positive feedback (a fast runaway), so it must be verified by hand
# (motor passive) before trusting MAGNETIC_ENCODER_SIGN_VERIFIED = True.
MAGNETIC_ENCODER_SIGN = +1.0

# Set to True only after MAGNETIC_ENCODER_SIGN has been confirmed by
# hand for this joint's own sensor mounting. The script refuses to run
# the trajectory controller while this is False.
MAGNETIC_ENCODER_SIGN_VERIFIED = True

# AGC range considered healthy enough to trust the angle reading.
AGC_HEALTHY_MIN = 10
AGC_HEALTHY_MAX = 100

# How many consecutive failed I2C reads are tolerated before the script
# aborts (rather than running on stale data indefinitely).
MAX_CONSECUTIVE_I2C_FAILS = 25

# --- Plotting -------------------------------------------------------------
GENERATE_PLOTS = True
SHOW_PLOTS = True

# ===========================================================================
#  END OF USER SETTINGS
# ===========================================================================

import os
import math
import time
import datetime
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
from epicallypowerful.actuation.cubemars.cubemars_servo import make_duty_cycle_message
from epicallypowerful.toolbox import TimedLoop

# epicallypowerful.actuation.actuator_group unconditionally attaches a
# FileHandler('motorlog.log') to the 'motorlog' logger at import time above,
# which creates that file next to this script on every run. Detach and
# remove it here so it stops appearing.
import logging as _logging
_motorlog = _logging.getLogger('motorlog')
for _h in _motorlog.handlers[:]:
    _motorlog.removeHandler(_h)
    _h.close()
if os.path.exists('motorlog.log'):
    os.remove('motorlog.log')

GLITCH_JUMP_RAD = math.radians(GLITCH_JUMP_DEG)

MOTOR_TYPE = 'AK80-9-V3-servo'   # Servo operating mode

if RUN_MODE not in ("trajectory", "calibrate_offset"):
    raise ValueError(
        f"RUN_MODE must be 'trajectory' or 'calibrate_offset', got "
        f"{RUN_MODE!r}."
    )

if INPUT_ENCODER not in ("magnetic", "internal"):
    raise ValueError(
        f"INPUT_ENCODER must be 'magnetic' or 'internal', got "
        f"{INPUT_ENCODER!r}."
    )

# Refuse to run the closed-loop position controller until
# MAGNETIC_ENCODER_SIGN has been verified. Skipped in
# RUN_MODE == "calibrate_offset", since that mode never commands motion
# and measuring the offset is itself a step toward verifying the sign by
# hand. Also skipped when INPUT_ENCODER == "internal": the magnetic
# encoder drives nothing in that mode (not even homing), so a wrong or
# unverified sign on it can no longer turn into a runaway -- the sensor
# may not even be connected.
if (RUN_MODE == "trajectory" and INPUT_ENCODER == "magnetic"
        and not MAGNETIC_ENCODER_SIGN_VERIFIED):
    raise RuntimeError(
        "MAGNETIC_ENCODER_SIGN_VERIFIED is False -- refusing to run. "
        "Verify MAGNETIC_ENCODER_SIGN BY HAND, with the motor NOT "
        "torque-commanded (see verify_magnetic_sign.py, or as5048b_read.py "
        "next to the CubeMars software's live position display), THEN set "
        "MAGNETIC_ENCODER_SIGN_VERIFIED = True above. Do not skip this -- "
        "getting MAGNETIC_ENCODER_SIGN wrong turns the outer-loop "
        "correction into positive feedback, which is a fast runaway, not "
        "a slow drift."
    )


# ------------------------------------------------------------------
# EXPONENTIAL MOVING AVERAGE (EMA) FILTER
# ------------------------------------------------------------------

def design_ema_alpha(fc_hz, fs_hz):
    """
    Computes the smoothing coefficient alpha for a 1-pole Exponential
    Moving Average low-pass filter, from a cutoff frequency and sample
    rate:
        alpha = (2*pi*fc*dt) / (1 + 2*pi*fc*dt),   dt = 1/fs
    """
    if fs_hz <= 0.0:
        raise ValueError(f"sample rate must be positive, got {fs_hz}")
    if fc_hz <= 0.0:
        raise ValueError(f"cutoff frequency must be positive, got {fc_hz}")
    dt = 1.0 / fs_hz
    x = 2.0 * math.pi * fc_hz * dt
    alpha = x / (1.0 + x)
    if not 0.0 < alpha < 1.0:
        raise ValueError(
            f"alpha={alpha} fell outside (0, 1) for fc={fc_hz} Hz, "
            f"fs={fs_hz} Hz -- check these values."
        )
    return alpha


class RealtimeEMALPF:
    """
    Streaming Exponential Moving Average low-pass filter:

        V_filt[n] = alpha*V_raw[n] + (1 - alpha)*V_filt[n-1]

    Warm-starts on the first sample so there is no cold-start transient.
    A NaN sample (the magnetic-derived velocity is momentarily
    unavailable -- e.g. a diagnostic-only read gap when INPUT_ENCODER ==
    "internal") holds the last filtered value instead of being mixed
    in: NaN propagates through the recursive average forever otherwise,
    so a single missed sample would permanently blank the filtered
    trace for the rest of the run.
    """
    def __init__(self, fc_hz, fs_hz):
        self.alpha = design_ema_alpha(fc_hz, fs_hz)
        self._warmed = False
        self.y1 = 0.0

    def update(self, v_raw):
        if math.isnan(v_raw):
            return self.y1 if self._warmed else v_raw
        if not self._warmed:
            self.y1 = v_raw
            self._warmed = True
        y = self.alpha * v_raw + (1.0 - self.alpha) * self.y1
        self.y1 = y
        return y


# ------------------------------------------------------------------
# MAGNETIC ENCODER (AS5048B) SUPPORT
# ------------------------------------------------------------------

def _as5048b_read_angle_raw(bus, addr):
    """Read the two angle registers and return the raw single-turn
    angle in degrees, [0, 360). Reads both bytes in one I2C transaction
    (not two separate reads), so they can never be torn from two
    different instants while the shaft is moving."""
    high_byte, low_byte = bus.read_i2c_block_data(addr, ANGLE_HIGH_REG, 2)
    raw14 = ((high_byte << 6) | (low_byte & 0x3F)) & 0x3FFF
    return (raw14 / 16384.0) * 360.0


def _as5048b_read_health(bus, addr):
    """Read AGC and the diagnostics byte. Not needed for the control
    law, so this is called only a few times a second, not every tick."""
    agc = bus.read_byte_data(addr, AGC_REG)
    diag = bus.read_byte_data(addr, DIAG_REG)
    return agc, diag


def wrap_deg(angle_deg):
    """Wrap an angle in degrees to (-180, 180].

    The AS5048B only reports a single-turn [0, 360) reading, so
    subtracting MAGNETIC_ENCODER_OFFSET can land anywhere in roughly
    (-360, 360). This brings that back to a small, signed angle centered
    on the calibration zero point, so a reading that straddles the
    AS5048B's internal 0/360 rollover still comes out continuous.
    """
    return ((angle_deg + 180.0) % 360.0) - 180.0


def read_actual_angle_deg(bus, joint):
    """actual_angle = joint.mag_sign * (raw reading - joint.mag_offset),
    wrapped. Callers should use read_actual_angle_deg_safe() below
    instead, which also tolerates a transient I2C failure."""
    raw_deg = _as5048b_read_angle_raw(bus, joint.as5048b_addr)
    return wrap_deg(joint.mag_sign * (raw_deg - joint.mag_offset))


def read_actual_angle_deg_safe(bus, joint):
    """Same as read_actual_angle_deg(), but a failed I2C read reuses the
    last known-good angle for that tick instead of crashing the
    trajectory. Raises if too many reads fail in a row, or if no good
    reading has ever been obtained."""
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


def read_actual_angle_deg_diagnostic(bus, joint):
    """Best-effort magnetic-encoder read for INPUT_ENCODER == "internal":
    the reading is logged/plotted for comparison only and never feeds
    homing or the control law, so a failed read (sensor dismounted,
    disconnected, never wired up at all, or the I2C bus itself not even
    open -- bus is None) is not an error here -- unlike
    read_actual_angle_deg_safe() above, this never raises and never
    blocks. Returns None for this tick if the read fails; callers must
    already treat that as "no diagnostic sample this tick", not a
    fault."""
    if bus is None:
        return None
    try:
        angle = read_actual_angle_deg(bus, joint)
        joint.i2c_fail_streak = 0
        joint.i2c_last_good_deg = angle
        joint.i2c_has_good_reading = True
        return angle
    except OSError:
        joint.i2c_fail_count += 1
        joint.i2c_fail_streak += 1
        joint.i2c_fail_max_streak = max(joint.i2c_fail_max_streak,
                                         joint.i2c_fail_streak)
        return None


# ---------------------------------------------------------------------------
# STEP 1 -- Load the gait cycle from the CSVs
# ---------------------------------------------------------------------------
def _find_heel_strikes_from_grf(grf_csv_path, angle_t):
    """Find heel-strike indices from the left foot's vertical
    ground-reaction force crossing a threshold. Uses the left foot
    because this dataset's right-side force channels are entirely NaN.
    """
    grf = pd.read_csv(grf_csv_path)
    if len(grf) != len(angle_t):
        raise ValueError(
            f"{grf_csv_path} has {len(grf)} rows but the angle file has "
            f"{len(angle_t)} rows -- they must share the same time base "
            f"for this stride-boundary detection to be valid. Are these "
            f"really two files from the SAME trial folder?"
        )
    fz = grf["LForceY_Vertical"].to_numpy(dtype=float)
    if np.all(np.isnan(fz)):
        raise ValueError(
            f"{grf_csv_path}'s LForceY_Vertical column is entirely NaN -- "
            f"this trial has no usable force data on the left foot either. "
            f"Pick a different trial."
        )
    THRESHOLD_N = 20.0   # "foot is on the ground" once vertical force exceeds this
    on_ground = np.nan_to_num(fz, nan=-1.0) > THRESHOLD_N
    return np.flatnonzero(np.diff(on_ground.astype(int)) == 1) + 1


def load_gait_cycle(angle_csv_path, grf_csv_path, stride_count):
    """Read STRIDE_COUNT consecutive strides from the angle CSV, starting
    at the first heel-strike, as time/angle arrays in seconds and
    radians, shifted so the trajectory starts at angle 0. Also returns
    the knee's absolute starting angle in degrees (the homing target).
    """
    df = pd.read_csv(angle_csv_path)
    t = df["time"].to_numpy(dtype=float)

    # The dataset's own column is extension-positive; this project is
    # flexion-positive, so it is negated here, once, on read.
    knee_deg = -df["knee_angle_l"].to_numpy(dtype=float)

    strikes = _find_heel_strikes_from_grf(grf_csv_path, t)
    max_strides = len(strikes) - 1
    print(f"  Found {len(strikes)} heel-strike(s) in {grf_csv_path} -> "
          f"up to {max_strides} continuous stride(s) available "
          f"(STRIDE_COUNT may range from 1 to {max_strides} for this "
          f"trial).")
    if max_strides < 1 or not (1 <= stride_count <= max_strides):
        raise ValueError(
            f"STRIDE_COUNT={stride_count} is out of range for this trial "
            f"-- valid range is 1 to {max_strides}."
        )

    i0, i1 = int(strikes[0]), int(strikes[stride_count])

    knee_start_deg = float(knee_deg[i0])

    t = t[i0:i1 + 1] - t[i0]
    knee = np.deg2rad(knee_deg[i0:i1 + 1] - knee_deg[i0])

    return t, knee, knee_start_deg


print(f"Loading gait-cycle reference from: {ANGLE_CSV_PATH}")
t_raw, knee_theta_raw, knee_start_deg_raw = load_gait_cycle(
    ANGLE_CSV_PATH, GRF_CSV_PATH, STRIDE_COUNT)

# Apply the mounting sign correction once, here, so the spline, the
# joint-limit check, and the homing target all inherit it consistently.
knee_theta_raw = KNEE_SIGN * knee_theta_raw
knee_start_deg = KNEE_SIGN * knee_start_deg_raw
print(f"  Knee start (auto-detected from dataset @ t=0):  "
      f"{knee_start_deg:+.2f} deg")

traj_duration = t_raw[-1] * TIME_SCALE
print(f"Loaded {STRIDE_COUNT} continuous stride(s): {len(t_raw)} samples, "
      f"{t_raw[-1]:.3f} s of recorded data at real speed -> "
      f"{traj_duration:.3f} s at TIME_SCALE={TIME_SCALE:.2f}.")

# ---------------------------------------------------------------------------
# STEP 2 -- Fit a cubic spline: Pd(t) and Vd(t) from the recorded data
# ---------------------------------------------------------------------------
t_scaled = t_raw * TIME_SCALE

knee_pos_spline = CubicSpline(t_scaled, knee_theta_raw, bc_type="not-a-knot")
knee_vel_spline = knee_pos_spline.derivative()


# ---------------------------------------------------------------------------
# STEP 3 -- Joint object holding this joint's settings and runtime state
# ---------------------------------------------------------------------------
@dataclass
class Joint:
    """Everything the joint needs in one place: static settings (id,
    label, start angle, filter cutoff, limits, splines) plus runtime
    state (filter instance, positions, logging buffers) that fills in as
    the script runs."""
    id: int
    label: str
    cutoff_hz: float
    start_deg: float
    limit_min_deg: float
    limit_max_deg: float
    pos_spline: CubicSpline
    vel_spline: CubicSpline
    input_encoder: str
    as5048b_addr: int
    mag_offset: float
    mag_sign: float
    mag_sign_verified: bool
    agc_healthy_min: int
    agc_healthy_max: int
    gear_ratio_factor: float
    filt: "RealtimeEMALPF" = field(init=False, default=None)
    P0: float = 0.0            # magnetic-encoder starting angle, rad
    P0_internal: float = 0.0   # internal-encoder starting angle, rad (comparison only)
    P_prev: float = 0.0        # previous-tick position, rad (magnetic encoder)
    t_prev: float = 0.0        # previous-tick wall time, s
    i2c_fail_count: int = 0
    i2c_fail_streak: int = 0
    i2c_fail_max_streak: int = 0
    i2c_last_good_deg: float = 0.0
    i2c_has_good_reading: bool = False
    wrap_reject_count: int = 0
    wrap_reject_streak: int = 0
    wrap_reject_max_streak: int = 0
    disagree_count: int = 0
    disagree_streak: int = 0
    disagree_max_streak: int = 0
    last_agc: int = field(init=False, default=-1)
    log: dict = field(default_factory=lambda: {k: [] for k in (
        "t", "dt", "pd_deg", "p_deg", "p_internal_deg", "pos_error_deg",
        "vd_rads", "v_raw", "v_filt", "correction_deg", "cmd_deg")})
    # Position readings recorded during home_joint(), used for the
    # printed homing status line.
    home_t: list = field(default_factory=list)
    home_mag_deg: list = field(default_factory=list)
    home_internal_deg: list = field(default_factory=list)
    home_send_target_deg: list = field(default_factory=list)

    def __post_init__(self):
        self.filt = RealtimeEMALPF(self.cutoff_hz, LOOP_HZ)


JOINT = Joint(
    KNEE_CAN_ID, "Knee", CUTOFF_KNEE_HZ,
    knee_start_deg, KNEE_LIMIT_MIN_DEG, KNEE_LIMIT_MAX_DEG,
    knee_pos_spline, knee_vel_spline,
    INPUT_ENCODER, AS5048B_ADDR, MAGNETIC_ENCODER_OFFSET,
    MAGNETIC_ENCODER_SIGN, MAGNETIC_ENCODER_SIGN_VERIFIED,
    AGC_HEALTHY_MIN, AGC_HEALTHY_MAX, GEAR_RATIO_FACTOR,
)
MOTORS = {JOINT.id: MOTOR_TYPE}

print(f"Velocity filter: Exponential Moving Average (EMA), "
      f"knee fc={CUTOFF_KNEE_HZ} Hz, fs={LOOP_HZ} Hz "
      f"(Nyquist {LOOP_HZ / 2} Hz) -- diagnostic only, not used by the "
      f"Position Loop Mode control law below.")
_feedback_sensor_desc = ("AS5048B magnetic encoder" if JOINT.input_encoder == "magnetic"
                          else "actuator's own internal encoder")
if JOINT.input_encoder == "magnetic":
    _mag_role_desc = ("actively steers homing and the control law, and is "
                       "protected by every guard below -- a disconnected/"
                       "dismounted sensor is a fault in this mode")
else:
    _mag_role_desc = ("read best-effort and logged/plotted for comparison "
                       "only -- it drives nothing and is guarded by "
                       "nothing, so it's fine to run with it disconnected "
                       "or dismounted")
print(f"INPUT_ENCODER = {JOINT.input_encoder!r} -- the outer-loop correction is "
      f"driven by the {_feedback_sensor_desc}. "
      f"The magnetic encoder is {_mag_role_desc} "
      f"(I2C bus {I2C_BUS}, addr 0x{JOINT.as5048b_addr:02X}), "
      f"offset {JOINT.mag_offset:+.2f} deg, sign "
      f"{JOINT.mag_sign:+.1f} (verified={JOINT.mag_sign_verified}).")


# ---------------------------------------------------------------------------
# STEP 4 -- Pre-flight joint-limit check, before the motor is powered
# ---------------------------------------------------------------------------
def check_joint_limits():
    """Compare the trajectory's full range of motion against the
    joint's hardware limits. If it doesn't fit and AUTO_FIT_TO_LIMITS is
    True, shift the whole trajectory to fit and say so; otherwise abort.

    Prefers fitting inside the hardware limits shrunk inward by
    HOMING_STANDOFF_DEG on each side, so the homing target is never left
    flush against a hardstop. Falls back to fitting the true limits if
    the trajectory is too wide for that -- it never violates the actual
    hardware limits either way.

    Runs before the CAN bus is opened, so a bad number here aborts
    before any current is ever sent.

    A cubic spline's true min/max within an interval can only occur at
    one of its knots or at a point where its derivative is zero -- NOT
    necessarily at a point that happens to land on an evenly-spaced
    sampling grid. A coarse fixed grid (the previous implementation
    used only 500 points across the whole trajectory, coarser than
    both the ~200 Hz dataset the spline was fit from and the 1000 Hz
    control loop that evaluates it at runtime) can miss a sharp trough
    or peak between grid points entirely -- exactly what let a knee
    trajectory shifted to start at +8.0 deg actually reach slightly
    below 0 deg at runtime. To make this exact rather than
    resolution-dependent, the dense grid below is unioned with every
    root of vel_spline (pos_spline's own derivative) inside
    [0, traj_duration], which are the only additional candidate
    locations for a true extremum.
    """
    t_grid = np.linspace(0.0, traj_duration, 2000)
    joint = JOINT
    crit_t = joint.vel_spline.roots(extrapolate=False)
    crit_t = crit_t[(crit_t >= 0.0) & (crit_t <= traj_duration)]
    t_check = np.union1d(t_grid, crit_t)
    rel_deg = np.degrees(joint.pos_spline(t_check))
    abs_min = joint.start_deg + rel_deg.min()
    abs_max = joint.start_deg + rel_deg.max()
    span = abs_max - abs_min
    limit_span = joint.limit_max_deg - joint.limit_min_deg

    fit_min_deg = joint.limit_min_deg + HOMING_STANDOFF_DEG
    fit_max_deg = joint.limit_max_deg - HOMING_STANDOFF_DEG
    if span > (fit_max_deg - fit_min_deg):
        print(f"  [warning] {joint.label}: trajectory spans {span:.1f} deg, "
              f"too wide to also honor the {HOMING_STANDOFF_DEG:.1f} deg "
              f"homing standoff on both ends -- falling back to a flush "
              f"fit against the TRUE hardware limits "
              f"[{joint.limit_min_deg:+.1f}, {joint.limit_max_deg:+.1f}] "
              f"deg. The homing target may end up close to a hardstop; "
              f"watch HOME_LIMIT_MARGIN_DEG aborts closely on this run.")
        fit_min_deg, fit_max_deg = joint.limit_min_deg, joint.limit_max_deg

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
          f"(homing target clearance from each hard limit: "
          f"{abs_min - joint.limit_min_deg:.1f} deg / "
          f"{joint.limit_max_deg - abs_max:.1f} deg).")


check_joint_limits()


# ---------------------------------------------------------------------------
# STEP 5 -- Open the CAN bus and the magnetic encoder's I2C bus
# ---------------------------------------------------------------------------
actuators = ActuatorGroup.from_dict(MOTORS, exit_manually=True)

# If INPUT_ENCODER == "internal", the magnetic encoder is optional --
# homing and the control law never need it, so a joint running purely on
# its internal encoder should work with the AS5048B unwired, unpowered,
# or the I2C bus itself not even enabled on this Pi (e.g. /dev/i2c-1
# missing). RUN_MODE == "calibrate_offset" always needs real I2C access
# regardless of INPUT_ENCODER, since that mode's entire purpose is
# reading the physical sensor.
try:
    mag_bus = SMBus(I2C_BUS)
except OSError as e:
    if RUN_MODE == "calibrate_offset" or JOINT.input_encoder == "magnetic":
        reason = ("RUN_MODE='calibrate_offset' always needs the magnetic "
                   "encoder's I2C bus to read its raw angle"
                   if RUN_MODE == "calibrate_offset" else
                   "INPUT_ENCODER=='magnetic' needs it for homing and "
                   "the control law")
        raise RuntimeError(
            f"Could not open I2C bus {I2C_BUS} ({e}) -- {reason}. Enable "
            f"I2C (raspi-config -> Interface Options -> I2C, then "
            f"reboot) and verify the AS5048B wiring/address, or set "
            f"INPUT_ENCODER = 'internal' if you don't need the magnetic "
            f"encoder for control (calibrate_offset mode still needs "
            f"I2C regardless)."
        ) from e
    print(f"  [warning] Could not open I2C bus {I2C_BUS} ({e}) -- "
          f"continuing without the magnetic encoder since INPUT_ENCODER "
          f"== 'internal' doesn't need it for homing or control. "
          f"Magnetic-encoder comparison readings will be unavailable "
          f"(logged/plotted as blank) for this run.")
    mag_bus = None


def lifecycle(code, can_id):
    """Send the motor's power-on (0xFC) or power-off (0xFD) message to
    one specific CAN ID."""
    actuators.bus.send(
        can.Message(arbitration_id=can_id, data=[0xFF] * 7 + [code],
                    is_extended_id=True)
    )


def home_joint(joint, bus):
    """Walk the joint to its dataset starting angle using Position Loop
    Mode, before the trajectory controller takes over. Raises if it
    doesn't arrive within HOME_TIMEOUT_S.

    Position Loop Mode always travels at maximum speed and acceleration
    to whatever target it is given -- there is no speed/accel argument
    to tame it. To stay safe under that, this function never sends a
    large absolute target: every tick it re-reads the internal encoder's
    live position and sends a target that is at most
    HOME_MAX_STEP_ACTUATOR_DEG further than that, in the direction that
    reduces the error of whichever encoder INPUT_ENCODER selects.

    INPUT_ENCODER == "magnetic": the magnetic encoder (absolute,
    single-turn) decides the step direction AND when the joint has
    arrived, exactly as before. The internal encoder is read every tick
    only to know where the actuator output shaft currently is, for the
    step target above -- it never decides arrival. If the magnetic
    encoder reads outside the hardware limits when this function
    starts (e.g. the shaft coasted past a limit after a previous run
    stopped mid-motion), homing does not abort: the target is always
    verified (by check_joint_limits()) to sit inside the limits, so
    every step here moves the shaft back toward the valid range
    regardless of where it starts. A warning is printed once if this
    happens; HOME_TIMEOUT_S remains as the backstop if recovery does
    not converge.

    INPUT_ENCODER == "internal": the internal encoder alone decides
    both the step direction and arrival -- it is self-sufficient for
    homing, exactly as it is for the trajectory control law below. The
    magnetic encoder is still read every tick on a best-effort basis
    purely for the printed status line and later comparison plots; it
    is never used to steer, gate arrival, or trigger a limit warning
    here, and a missing/disconnected/dismounted magnetic sensor is not
    an error in this mode.
    """
    home_target_deg = joint.start_deg
    clock_h = TimedLoop(rate=HOME_LOOP_HZ)
    t_home0 = time.perf_counter()
    home_tick = 0
    limit_warned = False
    HOME_PRINT_EVERY = 5   # print a status line every 5 ticks
    while clock_h():
        internal_now_deg = actuators.get_position(joint.id, degrees=True)

        if joint.input_encoder == "magnetic":
            now_deg = read_actual_angle_deg_safe(bus, joint)
            err = home_target_deg - now_deg
            # Step toward the target by at most HOME_MAX_STEP_ACTUATOR_DEG,
            # scaled by joint.gear_ratio_factor to convert the joint-frame
            # error into the actuator's own frame.
            step_deg = math.copysign(
                min(HOME_MAX_STEP_ACTUATOR_DEG, abs(err) * joint.gear_ratio_factor),
                err)
        else:
            # Diagnostic-only magnetic sample -- may be None if the
            # sensor is dismounted/disconnected; never used below.
            now_deg = read_actual_angle_deg_diagnostic(bus, joint)
            # home_target_deg and internal_now_deg are both already in
            # the actuator's own frame here (joint.gear_ratio_factor == 1.0
            # means that frame is also the joint frame), so err needs
            # no further conversion before being used as the step size.
            err = home_target_deg * joint.gear_ratio_factor - internal_now_deg
            step_deg = math.copysign(
                min(HOME_MAX_STEP_ACTUATOR_DEG, abs(err)), err)

        send_target_deg = internal_now_deg + step_deg

        joint.home_t.append(time.perf_counter() - t_home0)
        joint.home_mag_deg.append(now_deg)
        joint.home_internal_deg.append(internal_now_deg)
        joint.home_send_target_deg.append(send_target_deg)

        if (joint.input_encoder == "magnetic" and not limit_warned and (
                now_deg < joint.limit_min_deg - HOME_LIMIT_MARGIN_DEG
                or now_deg > joint.limit_max_deg + HOME_LIMIT_MARGIN_DEG)):
            limit_warned = True
            print(f"  [home] [warning] {joint.label}: magnetic encoder "
                  f"reads {now_deg:+.2f} deg, outside its hardware limits "
                  f"[{joint.limit_min_deg:+.1f}, {joint.limit_max_deg:+.1f}] "
                  f"deg (margin {HOME_LIMIT_MARGIN_DEG:.1f} deg) -- "
                  f"continuing to home, since home_target_deg "
                  f"({home_target_deg:+.2f} deg) is verified inside the "
                  f"limits and every step moves toward it, back into the "
                  f"valid range. Watch closely; verify by hand if this is "
                  f"unexpected.")

        if abs(err) <= HOME_TOLERANCE_DEG:
            arrived_desc = (f"{now_deg:+.2f} deg per the magnetic encoder"
                             if joint.input_encoder == "magnetic"
                             else f"{internal_now_deg:+.2f} deg per the "
                                  f"internal encoder")
            print(f"  [home] {joint.label}: reached {arrived_desc} "
                  f"(target {home_target_deg:+.2f} deg), internal encoder "
                  f"now reads {internal_now_deg:+.2f} deg.")
            return
        if time.perf_counter() - t_home0 > HOME_TIMEOUT_S:
            sensor_desc = ("per the magnetic encoder" if joint.input_encoder == "magnetic"
                            else "per the internal encoder")
            raise RuntimeError(
                f"{joint.label} failed to home within {HOME_TIMEOUT_S:.1f} "
                f"s (still {abs(err):.2f} deg from "
                f"{home_target_deg:+.2f} deg {sensor_desc}) -- "
                f"aborting before the control loop starts. If this keeps "
                f"happening: verify MAGNETIC_ENCODER_OFFSET is still "
                f"correct (re-measure via RUN_MODE='calibrate_offset' if "
                f"the sensor or magnet was ever repositioned -- and "
                f"remember the knee needs its OWN offset, not the "
                f"ankle's), verify MAGNETIC_ENCODER_SIGN was confirmed by "
                f"hand (see the safety note near the top of this file -- "
                f"a wrong sign here can also show up as homing never "
                f"converging, not just as a runaway), verify by hand that "
                f"the shaft's ACTUAL physical position isn't genuinely "
                f"far from the dataset's expected start angle, and "
                f"consider whether HOME_MAX_STEP_ACTUATOR_DEG is now too "
                f"small for the actual distance being homed (it trades "
                f"homing speed for safety -- see its own comment)."
            )

        if home_tick % HOME_PRINT_EVERY == 0:
            mag_str = f"{now_deg:+7.2f}" if now_deg is not None else "   n/a "
            print(f"  [home] {joint.label}: t={joint.home_t[-1]:5.2f}s  "
                  f"mag={mag_str} deg (err {err:+6.2f})  "
                  f"internal={internal_now_deg:+8.2f} deg  "
                  f"send_target={send_target_deg:+8.2f} deg")

        actuators.set_position(joint.id, send_target_deg, 0, 0,
                                degrees=True)
        home_tick += 1


def _home_and_capture_p0(joint, bus):
    """Run home_joint(), then actively hold the homed position -- re-
    sending it every tick for POST_HOME_SETTLE_S seconds while sampling
    the magnetic encoder -- and return the median reading as P0.

    INPUT_ENCODER == "magnetic": unchanged -- the magnetic reading is
    the live feedback signal, so a reading outside the hardware limits
    here aborts before the trajectory loop starts.

    INPUT_ENCODER == "internal": the magnetic reading is sampled purely
    for diagnostic logging/plotting, so it is read best-effort (never
    raises) and imposes no limit check -- a missing/dismounted magnetic
    sensor never blocks homing in this mode. If no magnetic samples
    were obtained at all, P0 comes back as NaN (diagnostic traces built
    from it will simply be empty/NaN too; nothing downstream depends on
    it for control).
    """
    home_joint(joint, bus)

    hold_target_deg = actuators.get_position(joint.id, degrees=True)
    hold_clock = TimedLoop(rate=HOME_LOOP_HZ)
    t_hold0 = time.perf_counter()
    p0_samples = []
    while time.perf_counter() - t_hold0 < POST_HOME_SETTLE_S:
        hold_clock()
        actuators.set_position(joint.id, hold_target_deg, 0, 0,
                                degrees=True)
        internal_now_deg = actuators.get_position(joint.id, degrees=True)

        if joint.input_encoder == "magnetic":
            now_deg = read_actual_angle_deg_safe(bus, joint)
        else:
            now_deg = read_actual_angle_deg_diagnostic(bus, joint)

        mag_str = f"{now_deg:+7.2f}" if now_deg is not None else "   n/a "
        print(f"  [hold] {joint.label}: t={time.perf_counter() - t_hold0:5.2f}s  "
              f"mag={mag_str} deg  internal={internal_now_deg:+8.2f} deg  "
              f"hold_target={hold_target_deg:+8.2f} deg")

        if joint.input_encoder == "magnetic" and (
                now_deg < joint.limit_min_deg - HOME_LIMIT_MARGIN_DEG
                or now_deg > joint.limit_max_deg + HOME_LIMIT_MARGIN_DEG):
            raise RuntimeError(
                f"{joint.label}: magnetic encoder read {now_deg:+.2f} deg "
                f"(internal encoder reads {internal_now_deg:+.2f} deg) "
                f"while holding position after homing, outside its "
                f"hardware limits [{joint.limit_min_deg:+.1f}, "
                f"{joint.limit_max_deg:+.1f}] deg -- stopping before the "
                f"joint reaches its hardstop."
            )
        if now_deg is not None:
            p0_samples.append(now_deg)

    # Median of every reading taken during the hold, not a single
    # sample, so one bad reading can't become the zero reference for
    # the whole run.
    if not p0_samples:
        p0_deg = math.nan
        print(f"  [P0] {joint.label}: no magnetic-encoder readings were "
              f"obtained during the hold (sensor disconnected/dismounted) "
              f"-- P0 set to NaN. Harmless since INPUT_ENCODER={joint.input_encoder!r} "
              f"doesn't use it for control; the magnetic comparison trace "
              f"will just be empty on the plots.")
    else:
        p0_samples.sort()
        p0_deg = p0_samples[len(p0_samples) // 2]
        print(f"  [P0] {joint.label}: magnetic-encoder start angle = "
              f"{p0_deg:+.2f} deg (median of {len(p0_samples)} readings taken "
              f"while actively holding position; homing target was "
              f"{joint.start_deg:+.2f} deg).")
    return p0_deg


def run_offset_calibration(joint, bus):
    """RUN_MODE == "calibrate_offset" -- read-only helper for measuring
    MAGNETIC_ENCODER_OFFSET. Never calls actuators.set_position() with a
    new target or otherwise commands motion -- the motor is woken only
    so the internal encoder can be shown for context.

    Procedure (see also the CALIBRATION section at the top of this
    file):
      1. With the motor powered, physically position the knee shaft at
         its true mechanical reference (the real full-extension
         hardstop) by hand.
      2. Run this script with RUN_MODE = "calibrate_offset" and read off
         "Raw AS5048B" while the shaft is held there -- that value IS
         MAGNETIC_ENCODER_OFFSET.
      3. Copy that number into MAGNETIC_ENCODER_OFFSET above, set
         RUN_MODE back to "trajectory", and re-run.
      4. Note the AGC value printed too -- compare it against
         AGC_AT_CALIBRATION later; if it drifts far, re-measure.

    The "Internal encoder" column in the printout is shown only for
    context and does not need to read any particular value.

    Ctrl+C to stop.
    """
    print(f"\n=== MAGNETIC_ENCODER_OFFSET calibration mode ({joint.label}) ===")
    print("READ-ONLY: no position commands are ever sent in this mode -- "
          "safe regardless of the shaft's current position.\n")
    print("Physically position the knee shaft at its TRUE mechanical "
          "reference (e.g. the real full-extension hardstop, 0 deg) now, "
          "by hand -- this does NOT depend on CubeMars' \"Set Origin\" or "
          "the internal encoder's zero in any way. Read off 'Raw AS5048B' "
          "below while held there; that value IS MAGNETIC_ENCODER_OFFSET. "
          "Ctrl+C once you have a stable reading.\n")
    clock = TimedLoop(rate=10)   # slow, human-readable cadence
    try:
        while clock():
            # Raw, uncalibrated reading -- the whole point of this mode
            # is to measure the offset, so no offset/sign is applied.
            raw_mag_deg = _as5048b_read_angle_raw(bus, joint.as5048b_addr)
            try:
                agc, _diag = _as5048b_read_health(bus, joint.as5048b_addr)
            except OSError:
                agc = -1
            internal_deg = actuators.get_position(joint.id, degrees=True)
            print(f"  Raw AS5048B (= MAGNETIC_ENCODER_OFFSET if the shaft "
                  f"is at its true mechanical zero right now) = "
                  f"{raw_mag_deg:7.2f} deg  |  "
                  f"Internal encoder (informational only) = "
                  f"{internal_deg:+9.2f} deg  |  "
                  f"AGC={agc:3d} (healthy "
                  f"[{joint.agc_healthy_min},{joint.agc_healthy_max}])")
    except KeyboardInterrupt:
        print("\nCalibration mode stopped.")


try:
    for motor_id in MOTORS:
        lifecycle(0xFC, motor_id)

    # Wait for a few fresh telemetry frames after the wake-up message,
    # so the firmware has processed it before the first command lands.
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

    if RUN_MODE == "calibrate_offset":
        run_offset_calibration(JOINT, mag_bus)
        # Nothing to home or track in this mode. `finally` below still
        # runs (motor shutdown, bus close) either way.
        raise SystemExit(0)

    print("\nHoming the knee joint to the dataset's start angle...")
    p0_deg = _home_and_capture_p0(JOINT, mag_bus)
    JOINT.P0 = math.radians(p0_deg)

    # Internal-encoder starting angle -- comparison logging only, never
    # used by the control law below.
    JOINT.P0_internal = actuators.get_position(JOINT.id)

    print(f"\nRunning knee trajectory (Servo Position Loop Mode, "
          f"magnetic-encoder outer-loop correction, "
          f"ABORT_ERROR_DEG={ABORT_ERROR_DEG:.1f} deg)  |  "
          f"{LOOP_HZ} Hz. Ctrl+C stops.\n")

    clock = TimedLoop(rate=LOOP_HZ)
    t0 = time.perf_counter()
    loop_t_start = t0
    total_duration = traj_duration
    i = 0
    abort_reason = None

    JOINT.P_prev = 0.0
    JOINT.t_prev = t0

    PRINT_EVERY = max(1, LOOP_HZ // 5)

    while clock():
        t_wall = time.perf_counter() - t0
        if t_wall > total_duration:
            break
        t_traj = t_wall   # position along the whole (non-repeating) trajectory

        joint = JOINT
        pd = float(joint.pos_spline(t_traj))
        vd = float(joint.vel_spline(t_traj))

        # INPUT_ENCODER == "magnetic": this reading is the live feedback
        # signal, so a persistent failure raises (see
        # read_actual_angle_deg_safe). INPUT_ENCODER == "internal": it's
        # sampled best-effort, purely for the comparison log/plots -- a
        # missing/dismounted sensor just means no diagnostic sample this
        # tick (actual_deg is None), never an error.
        if joint.input_encoder == "magnetic":
            actual_deg = read_actual_angle_deg_safe(mag_bus, joint)
        else:
            actual_deg = read_actual_angle_deg_diagnostic(mag_bus, joint)
        now_t = time.perf_counter()
        p = (math.radians(actual_deg) - joint.P0
             if actual_deg is not None else math.nan)

        # Internal-encoder position -- one CAN read reused for two
        # purposes below: p_internal_deg (converted into the joint's own
        # frame via GEAR_RATIO_FACTOR, for the disagreement guard and
        # comparison logging) and internal_now_deg (the raw absolute
        # reading, for the outer-loop correction's anchor point).
        internal_pos_rad = actuators.get_position(joint.id)
        internal_now_deg = math.degrees(internal_pos_rad)
        p_internal_deg = math.degrees(
            internal_pos_rad - joint.P0_internal
        ) / joint.gear_ratio_factor

        # Reject the sample (reuse the last known-good position) if
        # either guard fires: a single-tick jump too large since the
        # last reading, or the magnetic and internal encoders disagree
        # by more than MAX_ENCODER_DISAGREEMENT_DEG. This guard was
        # already load-bearing in the impedance-control script (it feeds
        # the same p that drives tau there); what's different here is
        # that a rejected sample's replacement value feeds a position
        # target the driver chases at max speed/acceleration, rather
        # than a torque smoothed by motor inertia and the current clamp.
        delta = p - joint.P_prev
        disagreement_deg = abs(math.degrees(p) - p_internal_deg)
        # Both guards below are disabled when the internal encoder is the
        # control input: they exist to protect the magnetic-encoder-driven
        # control law from a bad/glitchy/disagreeing magnetic reading, but
        # when INPUT_ENCODER == "internal" the magnetic reading no longer
        # feeds the control law (or homing) at all -- it's logged/plotted
        # for comparison only, on a best-effort basis (it may not even be
        # connected). So neither a single-tick jump nor a disagreement
        # between the two sensors is a reason to reject a sample or abort
        # in that mode.
        jump_bad = (joint.input_encoder == "magnetic"
                    and abs(delta) > GLITCH_JUMP_RAD)
        disagree_bad = (joint.input_encoder == "magnetic"
                         and disagreement_deg > MAX_ENCODER_DISAGREEMENT_DEG)

        # Each guard's own streak tracks only that guard's own history,
        # independent of whether the other guard fired this tick.
        if jump_bad:
            joint.wrap_reject_count += 1
            joint.wrap_reject_streak += 1
            joint.wrap_reject_max_streak = max(
                joint.wrap_reject_max_streak, joint.wrap_reject_streak)
        else:
            joint.wrap_reject_streak = 0

        if disagree_bad:
            joint.disagree_count += 1
            joint.disagree_streak += 1
            joint.disagree_max_streak = max(
                joint.disagree_max_streak, joint.disagree_streak)
        else:
            joint.disagree_streak = 0

        if jump_bad or disagree_bad:
            if (joint.wrap_reject_streak > MAX_CONSECUTIVE_WRAP_REJECTS
                    or joint.disagree_streak > MAX_CONSECUTIVE_WRAP_REJECTS):
                reasons = []
                if joint.wrap_reject_streak > MAX_CONSECUTIVE_WRAP_REJECTS:
                    reasons.append(
                        f"{joint.wrap_reject_streak} consecutive "
                        f"implausible position jump(s) "
                        f"(> {GLITCH_JUMP_DEG:.0f} deg/tick)")
                if joint.disagree_streak > MAX_CONSECUTIVE_WRAP_REJECTS:
                    reasons.append(
                        f"{joint.disagree_streak} consecutive "
                        f"magnetic/internal-encoder disagreement(s) "
                        f"(latest {disagreement_deg:.1f} deg, "
                        f"limit {MAX_ENCODER_DISAGREEMENT_DEG:.0f} deg)")
                abort_reason = (
                    f"{joint.label}: " + " and ".join(reasons) + " -- the "
                    f"magnetic encoder reading is not trustworthy right "
                    f"now (near the AS5048B's raw 0/360 rollover if "
                    f"MAGNETIC_ENCODER_OFFSET is still a placeholder, a "
                    f"loose connection, I2C noise, or genuine sensor "
                    f"failure) -- stopping rather than running the "
                    f"controller on repeatedly stale/disagreeing data."
                )
            p = joint.P_prev
            delta = 0.0

        # Velocity: finite difference of the magnetic position, using
        # the measured tick-to-tick time so an occasional slow tick
        # doesn't look like a real velocity spike. Diagnostic only --
        # not used by the control law below.
        dt = now_t - joint.t_prev
        if dt <= 0.0:
            dt = 1.0 / LOOP_HZ
        v_raw = delta / dt

        joint.P_prev = p
        joint.t_prev = now_t

        v_filt = joint.filt.update(v_raw)

        # --- Outer-loop correction: INPUT_ENCODER actively steers the
        # Servo Position Loop Mode target. Pure, gain-free proportional
        # correction: a 1 deg tracking error becomes exactly a 1 deg
        # nudge to the target, anchored to the internal encoder's
        # CURRENT reading (not a fixed homing-time baseline), so it
        # continuously cancels whatever disagreement currently exists
        # between the reference and the chosen feedback sensor. Clamped
        # to +/-ABORT_ERROR_DEG -- see that constant's own comment for
        # why the clamp and the runaway-abort threshold are deliberately
        # the same number.
        pd_deg = math.degrees(pd)
        p_deg = math.degrees(p)
        feedback_deg = p_deg if joint.input_encoder == "magnetic" else p_internal_deg
        err_deg = pd_deg - feedback_deg
        correction_deg = max(-ABORT_ERROR_DEG,
                              min(ABORT_ERROR_DEG, err_deg))

        target_internal_deg = (internal_now_deg
                                + correction_deg * joint.gear_ratio_factor)
        actuators.set_position(joint.id, target_internal_deg, 0, 0,
                                degrees=True)

        # Joint-frame equivalent of what was actually commanded this
        # tick: target_internal_deg converted back to joint-frame degrees
        # the same way p_internal_deg already is. NOT pd_deg + correction
        # -- target_internal_deg is anchored to internal_now_deg (the
        # internal encoder's CURRENT reading), not to the reference.
        cmd_deg = p_internal_deg + correction_deg

        # --- Runaway guards ----------------------------------------
        err_deg_abs = abs(err_deg)
        if ABORT_ERROR_DEG > 0.0 and err_deg_abs > ABORT_ERROR_DEG:
            abort_reason = (
                f"{joint.label} tracking error reached {err_deg_abs:.1f} "
                f"deg (limit {ABORT_ERROR_DEG:.1f} deg) -- the joint "
                f"is no longer following the trajectory."
            )

        # Hardware-limit runaway guard: uses whichever encoder is
        # actually driving control (same choice as feedback_deg above),
        # so this never depends on a magnetic reading that may be
        # disconnected/dismounted when INPUT_ENCODER == "internal".
        abs_deg = (math.degrees(joint.P0 + p) if joint.input_encoder == "magnetic"
                   else joint.start_deg + p_internal_deg)
        if abs_deg < joint.limit_min_deg or abs_deg > joint.limit_max_deg:
            abort_reason = (
                f"{joint.label} reached {abs_deg:+.1f} deg, "
                f"outside its hardware limits "
                f"[{joint.limit_min_deg:+.1f}, "
                f"{joint.limit_max_deg:+.1f}] deg -- stopping "
                f"before the joint reaches its hardstop."
            )

        joint.log["t"].append(t_wall)
        joint.log["dt"].append(dt)
        joint.log["pd_deg"].append(pd_deg)
        joint.log["p_deg"].append(p_deg)
        joint.log["p_internal_deg"].append(p_internal_deg)
        # Signed tracking error: reference minus actual. Positive means
        # the actual position is lagging behind the reference; negative
        # means it has overshot ahead of it.
        joint.log["pos_error_deg"].append(err_deg)
        joint.log["vd_rads"].append(vd)
        joint.log["v_raw"].append(v_raw)
        joint.log["v_filt"].append(v_filt)
        joint.log["correction_deg"].append(correction_deg)
        joint.log["cmd_deg"].append(cmd_deg)

        if abort_reason is not None:
            print(f"\n  [ABORT] {abort_reason}")
            print("  Stopping the trajectory now. The plots below still "
                  "show everything logged up to this point.")
            break

        if i % PRINT_EVERY == 0:
            # Check the AS5048B's AGC a few times a second, not every
            # tick, so it costs no real-time budget. Skipped entirely if
            # mag_bus is None (I2C unavailable, tolerated only when
            # INPUT_ENCODER == "internal" -- see where mag_bus is opened).
            if mag_bus is not None:
                try:
                    agc, diag = _as5048b_read_health(mag_bus, joint.as5048b_addr)
                    joint.last_agc = agc
                    if agc < joint.agc_healthy_min or agc > joint.agc_healthy_max:
                        print(f"  [warning] {joint.label}: AS5048B AGC={agc} "
                              f"is outside the healthy [{joint.agc_healthy_min}, "
                              f"{joint.agc_healthy_max}] band (0=strong field, "
                              f"255=weak field) -- check the air gap / magnet "
                              f"alignment; readings may be degraded.")
                except OSError:
                    pass   # health check is best-effort -- never abort over it

            d = joint.log
            agc_str = f"{joint.last_agc}" if joint.last_agc >= 0 else "?"
            print(f"t={t_wall:5.2f}s  {joint.label:5s} (id={joint.id:3d})  "
                  f"Pd={d['pd_deg'][-1]:+7.2f} deg  "
                  f"P={d['p_deg'][-1]:+7.2f} deg  "
                  f"P_int={d['p_internal_deg'][-1]:+7.2f} deg  "
                  f"Vd={d['vd_rads'][-1]:+6.3f}  "
                  f"Vraw={d['v_raw'][-1]:+6.3f}  "
                  f"Vfilt={d['v_filt'][-1]:+6.3f} rad/s  "
                  f"corr={d['correction_deg'][-1]:+6.2f} deg  "
                  f"cmd={d['cmd_deg'][-1]:+7.2f} deg  "
                  f"AGC={agc_str}")
        i += 1

        if getattr(actuators.notifier, "exception", None):
            print("RX thread died:", actuators.notifier.exception)
            break

    print("\nTrajectory finished.")

except KeyboardInterrupt:
    print("\nCtrl+C -- stopping.")

finally:
    loop_t_end = time.perf_counter()

    # Stop path: the trajectory loop above drove the motor via the
    # driver's own firmware Position Loop Mode (Servo mode 4), which
    # actively servos to the last commanded target -- unlike Current
    # Loop Mode (used throughout the impedance-control scripts), it does
    # not reliably relinquish that target just because a Current-Loop
    # frame (set_torque) arrives. So first force the driver out of
    # Position Loop Mode with an explicit duty-cycle-0 frame (CubeMars/
    # VESC's universal "no output" command, which every control mode
    # yields to), THEN zero current, THEN release the firmware.
    for _ in range(10):
        for motor_id in MOTORS:
            actuators.bus.send(make_duty_cycle_message(motor_id, 0.0))
        time.sleep(0.005)
    for _ in range(10):
        for motor_id in MOTORS:
            actuators.set_torque(motor_id, 0.0)
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
    print("Motor limp; verify by hand before approaching.")

    # ACHIEVED loop rate -- the filter was designed for LOOP_HZ, so if
    # the loop ran slower, its effective cutoff scaled down with it.
    n_ticks = len(JOINT.log["t"])
    if n_ticks > 1:
        elapsed = JOINT.log["t"][-1] - JOINT.log["t"][0]
        achieved = (n_ticks - 1) / elapsed if elapsed > 0 else float("nan")
        print(f"Achieved loop rate: {achieved:.1f} Hz "
              f"(nominal {LOOP_HZ} Hz, {100.0 * achieved / LOOP_HZ:.1f}%)")
        if achieved < 0.95 * LOOP_HZ:
            print(f"  [warning] the loop ran below its nominal rate, so the "
                  f"velocity filter's EFFECTIVE cutoff was roughly "
                  f"{CUTOFF_KNEE_HZ * achieved / LOOP_HZ:.1f} Hz, "
                  f"not the designed value. The I2C reads for the "
                  f"magnetic encoder are a likely contributor if this "
                  f"achieved rate is notably below the internal-encoder "
                  f"version's -- consider reducing AS5048B_ADDR polling "
                  f"overhead or checking I2C bus speed if so.")

    if JOINT.i2c_fail_count > 0:
        print(f"  [warning] {JOINT.label}: AS5048B I2C read failed on "
              f"{JOINT.i2c_fail_count}/{n_ticks} tick(s) (worst "
              f"consecutive streak: {JOINT.i2c_fail_max_streak}) -- the "
              f"last known-good angle was reused each time. Consider "
              f"checking the I2C wiring/pull-ups/bus speed if this "
              f"number is large.")

    if JOINT.wrap_reject_count > 0:
        print(f"  [warning] {JOINT.label}: rejected {JOINT.wrap_reject_count}"
              f"/{n_ticks} implausible position jump(s) (worst consecutive "
              f"streak: {JOINT.wrap_reject_max_streak}) -- the last "
              f"known-good angle was reused each time instead of trusting "
              f"the jump. If this number is more than a handful, "
              f"MAGNETIC_ENCODER_OFFSET is likely still placing the "
              f"operating range too close to the AS5048B's raw 0/360 "
              f"rollover -- re-measure it for this joint's own sensor "
              f"(see the CALIBRATION note near the top of this file).")

    if JOINT.disagree_count > 0:
        print(f"  [warning] {JOINT.label}: the magnetic encoder disagreed "
              f"with the internal encoder by more than "
              f"{MAX_ENCODER_DISAGREEMENT_DEG:.0f} deg on "
              f"{JOINT.disagree_count}/{n_ticks} tick(s) (worst "
              f"consecutive streak: {JOINT.disagree_max_streak}) -- the "
              f"last known-good angle was reused each time instead of "
              f"trusting the disagreeing sample. If it ever fires, look "
              f"at the position plot's magnetic-vs-internal traces right "
              f"around the flagged tick(s) to see which sensor moved and "
              f"which one didn't.")

    if n_ticks > 0:
        clamp_hits = sum(
            1 for c in JOINT.log["correction_deg"]
            if abs(c) >= ABORT_ERROR_DEG - 1e-9)
        if clamp_hits > 0:
            print(f"  [warning] {JOINT.label}: the outer-loop correction "
                  f"clamp (ABORT_ERROR_DEG={ABORT_ERROR_DEG:.1f} deg) "
                  f"engaged on {clamp_hits}/{n_ticks} tick(s) -- this "
                  f"should be rare, since a tracking error this large is "
                  f"already close to triggering the runaway abort.")

    if not GENERATE_PLOTS:
        print("Plot generation disabled (GENERATE_PLOTS = False) -- no "
              "figures were produced.")
    elif not JOINT.log["t"] and not JOINT.home_t:
        print("No telemetry was logged at all, so no plots were produced.")
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        results_dir = os.path.join(script_dir, "results")
        os.makedirs(results_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        def _plot_marked_traces(ax, t, series):
            """Shared low-level plotting helper -- `series` is the only
            part that differs between the panels below.

            series: list of (y_array, color, linewidth, alpha,
            legend_label, marker_or_None). A non-None marker puts a
            sparse, hollow marker on that line (in addition to the solid
            line itself), so a trace that perfectly overlaps another one
            (e.g. actual vs. reference once tracking is good) still
            stays visually distinguishable rather than disappearing
            underneath it. Same marker spacing for every marked trace in
            a panel, so each gets the same number of markers regardless
            of the run's duration/rate; each marked trace also gets its
            own phase offset into that spacing (spread evenly across the
            marked traces), so two overlapping lines don't stack their
            markers on top of each other too.
            """
            markevery = max(1, len(t) // 25)
            n_marked = sum(1 for s in series if s[5] is not None)
            marked_seen = 0
            for y, color, lw, alpha, label, marker in series:
                if marker is not None:
                    offset = (marked_seen * markevery) // max(1, n_marked)
                    marked_seen += 1
                else:
                    offset = 0
                ax.plot(t, y, color=color, linewidth=lw, alpha=alpha,
                        label=label, marker=marker,
                        markevery=(offset, markevery), markersize=7,
                        markerfacecolor="none", markeredgewidth=1.3)

        def _save_figure(fig, filename_suffix):
            fig.tight_layout()
            out_path = os.path.join(results_dir,
                                     f"code-run-{stamp}{filename_suffix}.png")
            fig.savefig(out_path, dpi=300, bbox_inches="tight")
            print(f"Plot saved to: {out_path}")

        def plot_position(joint):
            """Reference vs. actual vs. commanded position. Whichever
            sensor joint.input_encoder selects is the live outer-loop
            feedback; the other is logged for comparison only. Plotted
            in ABSOLUTE joint-frame degrees (joint.start_deg added back
            on), not the relative-to-home degrees the control law and
            joint.log itself use -- otherwise a legitimate dip below the
            trajectory's own t=0 value reads as if it violates the
            hardware limits, when the standoff built into
            check_joint_limits() means it may not. The joint's own
            hardware limits are drawn on the plot."""
            data = joint.log
            if joint.input_encoder == "magnetic":
                p_deg_label = "Actual -- magnetic encoder (outer-loop feedback)"
                p_internal_deg_label = "Actual -- internal encoder (comparison only)"
            else:
                p_deg_label = "Actual -- magnetic encoder (comparison only)"
                p_internal_deg_label = "Actual -- internal encoder (outer-loop feedback)"
            offset = joint.start_deg
            fig, ax = plt.subplots(1, 1, figsize=(10, 5))
            series = [
                (np.asarray(data["pd_deg"]) + offset, "blue", 1.5, 1.0,
                 "Reference (dataset)", None),
                (np.asarray(data["p_deg"]) + offset, "red", 1.2, 1.0,
                 p_deg_label, "o"),
                (np.asarray(data["p_internal_deg"]) + offset, "grey", 1.0, 0.6,
                 p_internal_deg_label, "^"),
                (np.asarray(data["cmd_deg"]) + offset, "green", 1.0, 0.8,
                 "Commanded (internal-encoder actual + outer-loop correction)",
                 None),
            ]
            _plot_marked_traces(ax, data["t"], series)
            ax.axhline(joint.limit_min_deg, color="black", linewidth=1.2,
                       linestyle="--", alpha=0.6, label="Hardware limit min")
            ax.axhline(joint.limit_max_deg, color="black", linewidth=1.2,
                       linestyle="--", alpha=0.6, label="Hardware limit max")
            ax.set_ylabel(f"{joint.label} absolute angle [deg]")
            ax.set_xlabel("Time [s]")
            ax.set_title(f"{joint.label} joint: reference vs. actual vs. "
                         f"commanded trajectory (absolute joint-frame degrees)")
            ax.legend(loc="best")
            ax.grid(True, alpha=0.3)
            _save_figure(fig, "")

        def plot_velocity(joint):
            """Desired vs. RAW vs. FILTERED velocity -- diagnostic only,
            not used by the control law."""
            data = joint.log
            fig, ax = plt.subplots(1, 1, figsize=(10, 5))
            series = [
                (data["v_raw"], "grey", 0.7, 0.7,
                 "Raw velocity (finite-difference of magnetic-encoder position)",
                 None),
                (data["vd_rads"], "blue", 1.5, 1.0,
                 "Desired velocity (spline derivative)", None),
                (data["v_filt"], "red", 1.2, 1.0,
                 "Filtered velocity (EMA)", None),
            ]
            _plot_marked_traces(ax, data["t"], series)
            ax.set_ylabel(f"{joint.label} velocity [rad/s]")
            ax.set_xlabel("Time [s]")
            ax.set_title(f"{joint.label} joint: desired vs. raw vs. filtered "
                         f"velocity (diagnostic only -- not used by the "
                         f"control law)")
            ax.legend(loc="best")
            ax.grid(True, alpha=0.3)
            _save_figure(fig, "-velocity")

        def plot_error(joint):
            """Position tracking error: Pd(t) - P(t), the same quantity
            ABORT_ERROR_DEG checks in magnitude, plotted over the whole
            run."""
            data = joint.log
            fig, ax = plt.subplots(1, 1, figsize=(10, 5))
            ax.plot(data["t"], data["pos_error_deg"], color="purple",
                    linewidth=1.2, alpha=1.0,
                    label="Position error (Pd - P)")
            ax.axhline(0.0, color="black", linewidth=0.8, linestyle="--",
                       alpha=0.5)
            # Auto-scale, but never zoom in tighter than +/-5 deg or
            # narrower than ABORT_ERROR_DEG, so the abort threshold is
            # always visible on the plot.
            err_bound = max(5.0, ABORT_ERROR_DEG * 1.15,
                            1.15 * max((abs(v) for v in data["pos_error_deg"]),
                                       default=0.0))
            ax.set_ylim(-err_bound, err_bound)
            if ABORT_ERROR_DEG > 0.0:
                ax.axhline(ABORT_ERROR_DEG, color="red", linewidth=0.8,
                           linestyle=":", alpha=0.5, label="ABORT_ERROR_DEG")
                ax.axhline(-ABORT_ERROR_DEG, color="red", linewidth=0.8,
                           linestyle=":", alpha=0.5)
            ax.set_ylabel(f"{joint.label} position error [deg]")
            ax.set_xlabel("Time [s]")
            ax.set_title(f"{joint.label} joint: position tracking error vs. time")
            ax.legend(loc="best")
            ax.grid(True, alpha=0.3)
            _save_figure(fig, "-error")

        # Trajectory-phase plots -- skipped if the trajectory loop never
        # ran (e.g. homing failed), even if homing data was logged.
        if not JOINT.log["t"]:
            print("No trajectory-phase telemetry was logged (homing/"
                  "settling never reached the trajectory loop) -- "
                  "skipping the trajectory plots. The homing-phase data "
                  "above still reflects whatever WAS recorded.")
        else:
            plot_position(JOINT)
            plot_velocity(JOINT)
            plot_error(JOINT)

        if SHOW_PLOTS:
            try:
                plt.show()
            except Exception:
                pass

print("Done.")