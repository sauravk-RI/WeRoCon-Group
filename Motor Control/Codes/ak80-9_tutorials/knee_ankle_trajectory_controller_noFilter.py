#!/usr/bin/env python3
"""
knee_ankle_trajectory_controller_nofilter.py — Chapter 15, Step 3: a
synchronized, time-based knee-ankle trajectory controller for a powered
prosthetic leg bench, using two AK80-9 V3.0 actuators driven from a
Raspberry Pi 5 through the EPICally Powerful (EP) API.

WHAT THIS PROGRAM DOES
-----------------------
Both motors follow ONE recorded human gait cycle at the same time, on the
same clock:

    CAN ID 100  ->  the KNEE actuator
    CAN ID   1  ->  the ANKLE actuator

The reference motion comes from the "Walk" cyclic task, 1.2 m/s condition,
of the following public dataset:

    Scherpereel, K. L., Molinaro, D. D., Inan, O. T., Shepherd, M. &
    Young, A. J. "A human lower-limb biomechanics and wearable sensors
    dataset during cyclic and non-cyclic activities." Scientific Data 10,
    924 (2023). SMARTech, https://doi.org/10.35090/gatech/70296.

Licensed CC BY 4.0. Full credit for the underlying motion data belongs to
the original authors and Georgia Tech's EPIC Lab. This script only teaches
you how to stream that data to a pair of AK80-9 actuators.

Download the dataset per Ch 15 "How to get this dataset," copy the
"normal_walk_1_1-2" trial folder onto your Pi, and point ANGLE_CSV_PATH /
GRF_CSV_PATH (below) at its two files. Nothing needs to be pre-processed
by hand.

WHAT THE CONTROLLER DOES, IN ONE SENTENCE
------------------------------------------
Both joints run the SAME impedance (MIT-style) law that Chapter 14 Section
4.2 introduced, computed on the Pi and sent as Servo Current Loop commands:

        tau = Kp * (Pd(t) - P) + Kd * (Vd(t) - V)
        current [A] = tau / 0.5701

where Pd(t) and Vd(t) come from a smooth cubic spline fitted through one
full gait cycle's worth of the dataset's recorded joint-angle data, P is
each motor's own measured position, and V is each motor's measured
velocity, taken STRAIGHT from get_velocity() -- no filtering. (Section 3.9
diagnoses a real problem with that "straight from get_velocity()" choice,
and Ch 15's second script, knee_ankle_trajectory_controller_blpf_filter.py,
fixes it. This is deliberately the simpler, unfixed version first.)

WHAT YOU SHOULD SEE WHEN YOU RUN IT
-------------------------------------
  1. Both motors wake up, then each one homes -- moves gently, on its own,
     to the dataset's real starting angle for this joint -- before anything
     else happens. See "Homing: Starting From the Right Place" for why.
  2. ~0.5 s of stillness once both joints report "homed."
  3. Both shafts begin moving together, tracing out a smooth, human-like
     swinging motion: the knee motor sweeps through a large flexion-
     extension excursion, while the ankle motor makes a smaller
     dorsiflexion/plantarflexion excursion alongside it -- STRIDE_COUNT
     continuous, real recorded strides played back-to-back, stretched by
     TIME_SCALE (a TIME_SCALE of 2.0 means the motion takes twice as long
     as real human gait -- start here before ever trying 1.0).
  4. The terminal prints a status line for both joints, five times a
     second, showing the commanded and measured angle/velocity for each.
  5. After the trajectory ends, both motors are stopped and released, the
     ACHIEVED loop rate is reported, and three two-subplot figures are
     drawn and saved into a "results" folder next to this script: position
     tracking, velocity (desired vs. raw), and the torque decomposition.

Press Ctrl+C at any moment -- the cleanup code still runs, both motors are
stopped and released safely, and the plots (from whatever data was already
logged) are still produced.
"""

# ===========================================================================
#  USER SETTINGS -- the fields you are most likely to want to change
# ===========================================================================

# --- Paths to the Scherpereel et al. (2023) trial files --------------------
# Both files come from the SAME trial folder, "normal_walk_1_1-2" (the
# Walk / 1.2 m/s condition -- Ch 15 §3.1). Verified against the real
# download: this trial's left leg completes one gait cycle (heel-strike to
# heel-strike) in 1.085 s at a 200 Hz sample rate, cadence ~110 steps/min.
ANGLE_CSV_PATH = "AB01_normal_walk_1_1-2_angle.csv"
GRF_CSV_PATH   = "AB01_normal_walk_1_1-2_grf.csv"

# How many CONSECUTIVE strides, starting from the trial's very first
# heel-strike, to send to the motors as one continuous trajectory. There
# is no need to hunt for a stride that "closes" cleanly -- real
# consecutive strides in this dataset are already continuous, recorded
# motion, so STRIDE_COUNT strides are simply played back-to-back exactly
# as recorded, with no artificial loop-closing math needed.
#
# Verified against the real download (Ch 15 §3.2): the normal_walk_1_1-2
# trial contains 18 heel-strikes, i.e. 17 available strides -- valid
# range for THIS dataset is 1 to 17. The script also checks this at load
# time and raises a clear error if STRIDE_COUNT falls outside what the
# downloaded trial actually contains.
STRIDE_COUNT = 1

# --- CAN IDs -----------------------------------------------------------
KNEE_ID  = 100     # knee actuator CAN ID  (Ch 15's convention)
ANKLE_ID = 1       # ankle actuator CAN ID (Ch 15's convention)

# --- Control-loop rate ------------------------------------------------
# Both motors in this chapter are configured for a 500 Hz feedback rate -- 
# keep this matched to whatever CubeMarsTool setting your own motors use.
LOOP_HZ = 500

# --- Impedance gains, ONE pair per joint --------------------------------
# Same meaning as Ch 14 Section 4.2's Kp/Kd: Kp is the virtual spring
# (N*m/rad), Kd is the virtual damper (N*m/(rad/s)). These are the values
# verified stable on the bench for THIS script --
KNEE_KP  = 21.0
KNEE_KD  = 0.8

ANKLE_KP = 40.0
ANKLE_KD = 0.8

# --- Reference-trajectory sign correction ---------------------------------
# The dataset's own positive direction for "more flexion" (knee) or "more
# dorsiflexion" (ankle) may not match this motor's positive direction once
# it's mounted on the leg. Rather than guessing, this is a single, explicit
# switch per joint: +1.0 keeps the dataset's sign as-is, -1.0 flips it.
#
# Verify against the real hardware before trusting either switch to +1.0:
#   KNEE_SIGN:  turn the KNEE shaft by hand toward more FLEXION (bending
#               the knee) and confirm get_position() increases.
#   ANKLE_SIGN: turn the ANKLE shaft by hand toward more DORSIFLEXION
#               (toes drawing up toward the shin) and confirm
#               get_position() increases.
#
# This is NOT the same correction as the knee-extension-positive negation
# already applied inside load_gait_cycle() below -- that one fixes the
# DATASET's own published sign convention to a common flexion-positive
# convention, before this trajectory is ever loaded onto a real leg. This
# pair of switches fixes the MOUNTING direction of THIS motor on THIS leg,
# which can only be confirmed by hand on real hardware, never from a paper.
# Keep both corrections distinct -- if a joint ever moves backwards on the
# bench, this is the one line to check first.
KNEE_SIGN  = +1.0
ANKLE_SIGN = +1.0

# --- Joint hardware limits (OSL v2 mechanical range), degrees --------------
# Matches the OSL's own convention : 0 deg = full extension, 120 deg = max
# flexion for the knee and +30 deg = full dorsiflexion to -30 deg = full plantarflexion. If your own leg's real
# hardstops differ, update these to match -- these bound BOTH the
# pre-flight trajectory check below AND the runtime limit guard inside the
# control loop.
KNEE_LIMIT_MIN_DEG  = 0.0
KNEE_LIMIT_MAX_DEG  = 115.0
ANKLE_LIMIT_MIN_DEG = -25.0
ANKLE_LIMIT_MAX_DEG = 25.0

# --- Dataset start angle -------------------------------------------------
# No manual constant here: this used to be a pair of hardcoded literals
# (KNEE_START_DEG / ANKLE_START_DEG) that had to be kept in sync with
# whichever stride was selected. Since STRIDE_COUNT (above) always starts
# from the trial's first heel-strike, the script now reads each joint's
# true starting angle directly out of the CSV inside load_gait_cycle()
# below -- one less number to maintain by hand.

# --- What to do if the trajectory doesn't fit inside the joint limits -----
# If lets say, the recorded knee angle briefly goes slightly hyperextended (about -6
# deg), just past this joint's 0 deg hardware limit -- a real mismatch
# between the recorded human's knee and this leg's mechanical range, not a
# bug. AUTO_FIT_TO_LIMITS = True shifts the WHOLE trajectory (not its
# shape) up or down by just enough to bring it inside the hardware limits,
# and prints exactly how much it moved. Set to False to instead hard-abort
# whenever the raw trajectory doesn't fit -- useful once you are tuning
# against your own leg's real limits and want to be told immediately
# rather than have the script quietly compensate.
AUTO_FIT_TO_LIMITS = True

# --- Homing --------------------------------------------------------------
# Before the trajectory controller takes over, each joint is walked gently
# to the dataset's real starting angle (auto-detected from the CSV -- see
# "Dataset start angle" above), using Position-Velocity Mode (Ch 10) -- NOT
# Position Loop Mode's actuators.set_position(), which travels at maximum
# speed/acceleration
# (Ch 9's warning) and is the wrong tool for approaching a target gently.
# HOME_LOOP_HZ is deliberately the same 50 Hz Ch 10 §5 and Ch 14 §3.6 both
# use for Position-Velocity Mode demos, not the trajectory loop's 500 Hz --
# homing has no reason to hammer the bus ten times faster than every other
# Position-Velocity example in this series.
HOME_LOOP_HZ       = 50
HOME_SPEED_ERPM    = 1500    # cruise speed while homing, wire units (Ch 10 §5)
HOME_ACCEL_ERPM_S  = 3000    # ramp rate while homing
HOME_TOLERANCE_DEG = 1.0     # "close enough" -- stop polling once within this
HOME_TIMEOUT_S     = 8.0     # SECONDS (not milliseconds). If a joint has not
                              # reached its start angle within this many
                              # seconds, home_joint() raises RuntimeError and
                              # the script aborts before the control loop
                              # ever starts -- it never enters the trajectory
                              # loop from an unhomed position.

# --- Trajectory shaping --------------------------------------------------
TIME_SCALE = 2.0   # stretches the whole trajectory by this factor (2.0 =
                   # half of real walking speed). Use a bigger number for
                   # your very first run; approach 1.0 (real gait speed)
                   # only once you trust your gains and your bench setup.

# --- Startup torque ramp --------------------------------------------------
# The gait cycle begins mid-motion, so Vd(0) is NOT zero. The shaft,
# however, IS stationary at t=0, so on the very first tick the damper term
# alone asks for Kd*Vd(0) of torque -- a real, audible bang on a bench with
# two motors if left uneased.
#
# STARTUP_RAMP_S is a DURATION in seconds, NOT a slope. It is how long the
# ease-in period lasts: the commanded torque is scaled by a factor that
# rises from 0 -> 1 over exactly this many seconds, then stays at 1 for
# the rest of the run. The shape of that rise is a smoothstep S-curve
# (3u^2 - 2u^3, u = t/STARTUP_RAMP_S going from 0 to 1) -- NOT a straight
# ramp. A straight line has a slope corner at the moment it hits 1.0, and
# that corner is itself a small impulse that rings the joint's resonance;
# the smoothstep arrives at 1.0 with zero slope, so there is no corner to
# ring. Set to 0.0 to disable the ease-in entirely (and hear what it was
# doing).
STARTUP_RAMP_S = 0.25

# --- Runaway guards --------------------------------------------------------
# These guards abort the run
# instead of riding out a runaway joint.
#
#   ABORT_ERROR_DEG      -- tracking error past which the joint is clearly
#                            not following the trajectory any more.
#
# Set ABORT_ERROR_DEG to 0 to disable that guard (not recommended on
# hardware). There IS a software current clamp -- see MAX_CURRENT_A below.
ABORT_ERROR_DEG = 20.0

# --- Python-side current clamp -------------------------------------------
# The impedance law above is NOT internally limited -- a bad Kp/Kd, a unit
# mistake, or a stuck shaft could otherwise ask for far more current than
# is sensible, and the joint-limit/tracking-error guards above only react
# AFTER a bad command has already been sent for at least one tick.
# MAX_CURRENT_A clamps the MAGNITUDE of every current value computed from
# the impedance law, every single tick, before it is ever sent to the
# motor -- a hard backstop against exactly that class of mistake.
#
# This project's AK80-9 V3.0 has a 60 A firmware max-current parameter
# (CubeMarsTool -> Basic Settings, set during Ch 4 calibration); 50 A here
# leaves deliberate margin below that firmware ceiling. If you are running
# a DIFFERENT motor, or changed your own firmware limit, check YOUR OWN
# motor's datasheet / CubeMarsTool setting and set MAX_CURRENT_A to a value
# safely AT OR BELOW it -- never above it.
#
# This clamp is a backstop, not a substitute for validating Kp/Kd on the
# bench: if it engages during normal operation (a summary is printed at
# the end of the run), treat that as a sign your gains or units need a
# second look, not as the clamp "doing its job" as intended.
MAX_CURRENT_A = 50.0   # amperes -- keep at/below your own motor's firmware limit

# --- Wrap-glitch guard threshold -------------------------------------------
# See Section 3.9. Any single-tick position jump larger than this is
# treated as a telemetry glitch, not real motion, and corrected. 60 deg
# (converted to radians below, once `math` is imported) is far above any
# physically plausible per-tick motion at this loop rate, and far below
# the ~360 deg jumps the glitch actually produces.
GLITCH_JUMP_DEG = 60.0

# --- Torque constant, shared by both joints -------------------------------
KT = 0.5701   # N*m/A, output-shaft torque constant (Ch 2 Section 6)

# --- Plotting -------------------------------------------------------------
# GENERATE_PLOTS controls whether ANY plot is produced or saved to disk at
# all. Set to False for real/repeated runs on a Raspberry Pi where you do
# NOT want "results/" filling up the SD card with PNGs every run -- all
# terminal telemetry (status lines, achieved loop rate, clamp warnings)
# still prints either way. Default True for the desktop-analysis workflow
# this chapter otherwise assumes.
GENERATE_PLOTS = True

# SHOW_PLOTS only matters when GENERATE_PLOTS is True: whether to also pop
# the figures up on screen once they're saved, in addition to writing them
# to disk. The backend has to be chosen ONCE, BEFORE any figure is
# created: calling matplotlib.use() after figures exist triggers a backend
# switch that CLOSES every existing figure, so a later plt.show() silently
# displays nothing.
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

import matplotlib
# Backend decided up front (see SHOW_PLOTS above), never after figures exist.
if SHOW_PLOTS and os.environ.get("DISPLAY"):
    try:
        matplotlib.use("TkAgg")
    except Exception:
        matplotlib.use("Agg")
else:
    matplotlib.use("Agg")          # safe for headless / SSH sessions
import matplotlib.pyplot as plt

from epicallypowerful.actuation import ActuatorGroup
from epicallypowerful.actuation.cubemars.cubemars_servo import (
    make_position_velocity_mode_message,
)
from epicallypowerful.toolbox import TimedLoop

GLITCH_JUMP_RAD = math.radians(GLITCH_JUMP_DEG)

MOTOR_TYPE = 'AK80-9-V3-servo'   # both motors run in Servo operating mode
                                 # (Ch 13 Section 4.1) -- same dialect for both


# ---------------------------------------------------------------------------
# STEP 1 -- Load the Scherpereel et al. (2023) gait cycle from the CSVs
# ---------------------------------------------------------------------------
def _find_heel_strikes_from_grf(grf_csv_path, angle_t):
    """Detect ALL heel-strike indices from the LEFT foot's vertical
    ground-reaction force crossing a threshold. Returns the full strikes
    array -- the caller decides how many consecutive strides to slice out
    of it and validates STRIDE_COUNT against how many the trial actually
    contains.

    Why the LEFT foot: this dataset's RIGHT-side force channels
    (RForceY_Vertical in the *_grf.csv, and RVerticalF in *_insole_sim.csv)
    are 100% NaN in every trial shipped with this release -- verified
    directly against the download, not assumed. The LEFT foot's
    LForceY_Vertical is real, complete data. That is why this whole
    controller tracks the LEFT knee/ankle angle columns instead of the
    right ones (see knee_angle_l / ankle_angle_l in load_gait_cycle()
    below) -- the angle data and the force data used to segment it into
    strides have to come from the same leg.
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
    THRESHOLD_N = 20.0   # "foot is on the ground" once vertical force
                          # exceeds this -- generous margin above sensor
                          # noise at swing phase, well below body weight
    # nan_to_num maps any stray NaN sample to -1.0 N (i.e. "not on the
    # ground") rather than letting `NaN > THRESHOLD_N` silently evaluate
    # to False and pass a dead sensor off as "foot in the air."
    on_ground = np.nan_to_num(fz, nan=-1.0) > THRESHOLD_N
    return np.flatnonzero(np.diff(on_ground.astype(int)) == 1) + 1


def load_gait_cycle(angle_csv_path, grf_csv_path, stride_count):
    """Read the dataset's per-trial angle CSV and return STRIDE_COUNT
    CONSECUTIVE strides, starting from the trial's very first left
    heel-strike, as plain time / angle arrays, in SECONDS and RADIANS,
    each shifted so the trajectory starts at angle 0.

    Real consecutive strides are already continuous, recorded motion --
    there is no seam between them, so no loop-closing math is needed
    (unlike an earlier draft of this script, which had to hand-pick a
    single stride that "closed" cleanly for repeating).

    Returns: t (s), knee_theta (rad), ankle_theta (rad), knee_start_deg,
    ankle_start_deg -- the first three are NumPy arrays of equal length,
    ready to be splined; the last two are each joint's absolute starting
    angle (deg, this project's sign convention, NOT yet KNEE_SIGN/
    ANKLE_SIGN mounting-corrected) at the very first sample -- the homing
    target.
    """
    df = pd.read_csv(angle_csv_path)
    t = df["time"].to_numpy(dtype=float)

    # Verified against the real download (Ch 15 §3.1): the dataset's own
    # summary figures call the knee column extension-positive; this
    # project (and the OSL's own 0deg=extension convention) is
    # flexion-positive, so the raw column is negated here, once, right
    # where it is read.
    knee_deg = -df["knee_angle_l"].to_numpy(dtype=float)

    # CONFIRMED against the real download (Ch 15 §3.1), not an assumption:
    # plotted against known gait-phase landmarks, this column peaks
    # positive (~+11 deg) at midstance -- dorsiflexion -- and goes most
    # negative (~-22 deg) right around toe-off -- plantarflexion. That is
    # already this project's dorsiflexion-positive convention (and matches
    # the OSL's own example FSM code: ANKLE_THETA_ESWING=+25 for swing
    # dorsiflexion clearance, ANKLE_THETA_LSTANCE=-20 for push-off
    # plantarflexion). No negation needed.
    ankle_deg = df["ankle_angle_l"].to_numpy(dtype=float)

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
    ankle_start_deg = float(ankle_deg[i0])

    t = t[i0:i1 + 1] - t[i0]
    knee = np.deg2rad(knee_deg[i0:i1 + 1] - knee_deg[i0])
    ankle = np.deg2rad(ankle_deg[i0:i1 + 1] - ankle_deg[i0])

    return t, knee, ankle, knee_start_deg, ankle_start_deg


print(f"Loading gait-cycle reference from: {ANGLE_CSV_PATH}")
t_raw, knee_theta_raw, ankle_theta_raw, knee_start_deg_raw, ankle_start_deg_raw = (
    load_gait_cycle(ANGLE_CSV_PATH, GRF_CSV_PATH, STRIDE_COUNT))

# Physical-mounting sign correction (§ USER SETTINGS above), applied at the
# single point where every downstream consumer -- the spline, the joint-
# limit check, and the homing target -- inherits it consistently. Note
# this applies to the START angles too, not just the trajectory shape --
# an earlier draft of this script applied it only to the trajectory arrays
# and left the homing target uncorrected, which would silently home to the
# WRONG absolute angle whenever either sign switch was -1.0.
knee_theta_raw  = KNEE_SIGN * knee_theta_raw
ankle_theta_raw = ANKLE_SIGN * ankle_theta_raw
knee_start_deg  = KNEE_SIGN * knee_start_deg_raw
ankle_start_deg = ANKLE_SIGN * ankle_start_deg_raw
print(f"  Knee start (auto-detected from dataset @ t=0):  "
      f"{knee_start_deg:+.2f} deg")
print(f"  Ankle start (auto-detected from dataset @ t=0): "
      f"{ankle_start_deg:+.2f} deg")

traj_duration = t_raw[-1] * TIME_SCALE
print(f"Loaded {STRIDE_COUNT} continuous stride(s): {len(t_raw)} samples, "
      f"{t_raw[-1]:.3f} s of recorded data at real speed -> "
      f"{traj_duration:.3f} s at TIME_SCALE={TIME_SCALE:.2f}.")

# ---------------------------------------------------------------------------
# STEP 2 -- Fit a cubic spline per joint: Pd(t) and Vd(t) from ONE curve
# ---------------------------------------------------------------------------
# CubicSpline is built on the TIME-SCALED time axis directly, so evaluating
# it at real wall-clock time t gives Pd(t) in rad, and its analytic
# derivative gives Vd(t) in rad/s -- guaranteed consistent with each other
# at every instant (Section 3.3). bc_type="not-a-knot" is unconditional now
# -- STRIDE_COUNT strides are played once, not repeated, so there is no
# cycle to close and "periodic" would only impose an artificial constraint
# tying the start and end derivatives together.
t_scaled = t_raw * TIME_SCALE

knee_pos_spline = CubicSpline(t_scaled, knee_theta_raw, bc_type="not-a-knot")
knee_vel_spline = knee_pos_spline.derivative()

ankle_pos_spline = CubicSpline(t_scaled, ankle_theta_raw, bc_type="not-a-knot")
ankle_vel_spline = ankle_pos_spline.derivative()


# ---------------------------------------------------------------------------
# STEP 3 -- One Joint object per actuator, keeping every per-joint value
# (gains, sign-corrected start angle, hardware limits, splines, logging)
# together instead of several parallel dictionaries.
# ---------------------------------------------------------------------------
@dataclass
class Joint:
    """Everything one joint needs, in one place. Static configuration (id,
    label, gains, start angle, hardware limits, splines) is set once at
    construction; the rest (starting position, previous-tick position,
    logging buffers) is runtime state that fills in as the script
    proceeds."""
    id: int
    label: str
    kp: float
    kd: float
    start_deg: float
    limit_min_deg: float
    limit_max_deg: float
    pos_spline: CubicSpline
    vel_spline: CubicSpline
    P0: float = 0.0        # motor's absolute starting angle, rad -- filled
                            # in once the motor is awake (Step 5 below)
    P_prev: float = 0.0    # previous-tick relative position, rad -- used
                            # only by the wrap-glitch guard (Section 3.9)
    clamp_count: int = 0   # ticks where MAX_CURRENT_A had to clip the
                            # commanded current -- reported at the end
    log: dict = field(default_factory=lambda: {k: [] for k in (
        "t", "pd_deg", "p_deg", "vd_rads", "v_raw",
        "tau_position", "tau_velocity", "tau_total", "current")})


JOINTS = [
    Joint(KNEE_ID, "Knee", KNEE_KP, KNEE_KD,
          knee_start_deg, KNEE_LIMIT_MIN_DEG, KNEE_LIMIT_MAX_DEG,
          knee_pos_spline, knee_vel_spline),
    Joint(ANKLE_ID, "Ankle", ANKLE_KP, ANKLE_KD,
          ankle_start_deg, ANKLE_LIMIT_MIN_DEG, ANKLE_LIMIT_MAX_DEG,
          ankle_pos_spline, ankle_vel_spline),
]
JOINTS_BY_ID = {j.id: j for j in JOINTS}
MOTORS = {j.id: MOTOR_TYPE for j in JOINTS}

# Vd(0) is NOT zero -- the gait cycle begins mid-motion. The shaft, however,
# IS stationary at t=0, so the damper term contributes Kd*Vd(0) of torque
# on the very first tick. It is small at these gains, but it is real, and
# it is worth knowing about before it surprises you (see STARTUP_RAMP_S).
for joint in JOINTS:
    vd0 = float(joint.vel_spline(0.0))
    print(f"  {joint.label:5s}: Vd(0) = {vd0:+.3f} rad/s -> "
          f"startup damper torque {joint.kd * vd0:+.3f} N*m "
          f"({joint.kd * vd0 / KT:+.2f} A) before the "
          f"{STARTUP_RAMP_S:.2f} s ease-in ramp")


# ---------------------------------------------------------------------------
# STEP 4 -- Pre-flight joint-limit check, BEFORE any motor is powered
# ---------------------------------------------------------------------------
def check_joint_limits():
    """Compare the trajectory's full absolute range of motion (dataset
    start angle + the spline's own excursion) against each joint's real
    mechanical limits. If it doesn't fit and AUTO_FIT_TO_LIMITS is True,
    shift the WHOLE trajectory (joint.start_deg only -- the spline's own
    shape never changes) by just enough to bring it inside the limits, and
    say so loudly. If it still doesn't fit -- the excursion itself is
    wider than the joint's range -- no shift can fix that, and the script
    aborts either way.

    This is pure math against the spline -- it runs before the CAN bus is
    even opened, so a bad number here aborts before any current is ever
    sent, not after.
    """
    t_check = np.linspace(0.0, traj_duration, 500)
    for joint in JOINTS:
        rel_deg = np.degrees(joint.pos_spline(t_check))
        abs_min = joint.start_deg + rel_deg.min()
        abs_max = joint.start_deg + rel_deg.max()
        span = abs_max - abs_min
        limit_span = joint.limit_max_deg - joint.limit_min_deg

        if abs_min < joint.limit_min_deg or abs_max > joint.limit_max_deg:
            if not AUTO_FIT_TO_LIMITS:
                raise ValueError(
                    f"{joint.label} trajectory would command "
                    f"[{abs_min:+.1f}, {abs_max:+.1f}] deg, which falls "
                    f"outside the joint's hardware limits "
                    f"[{joint.limit_min_deg:+.1f}, "
                    f"{joint.limit_max_deg:+.1f}] deg, and "
                    f"AUTO_FIT_TO_LIMITS is False. Aborting before any "
                    f"motor moves."
                )
            if span > limit_span:
                raise ValueError(
                    f"{joint.label} trajectory spans {span:.1f} deg, wider "
                    f"than the joint's own {limit_span:.1f} deg range "
                    f"[{joint.limit_min_deg:+.1f}, "
                    f"{joint.limit_max_deg:+.1f}] -- no shift can make "
                    f"this fit. Aborting before any motor moves."
                )
            shift = (joint.limit_min_deg - abs_min if abs_min < joint.limit_min_deg
                     else joint.limit_max_deg - abs_max)
            print(f"  [auto-fit] {joint.label}: reference shifted "
                  f"{shift:+.2f} deg to fit inside "
                  f"[{joint.limit_min_deg:+.1f}, {joint.limit_max_deg:+.1f}] "
                  f"deg (was [{abs_min:+.1f}, {abs_max:+.1f}] deg).")
            joint.start_deg += shift
            abs_min += shift
            abs_max += shift

        print(f"  [limit-check] {joint.label}: trajectory range "
              f"[{abs_min:+.1f}, {abs_max:+.1f}] deg is within hardware "
              f"limits [{joint.limit_min_deg:+.1f}, "
              f"{joint.limit_max_deg:+.1f}] deg.")


check_joint_limits()


# ---------------------------------------------------------------------------
# STEP 5 -- Open the CAN bus and register BOTH motors (Ch 15 Step 2 pattern)
# ---------------------------------------------------------------------------
actuators = ActuatorGroup.from_dict(MOTORS, exit_manually=True)


def lifecycle(code, can_id):
    """Send the motor's power-on (0xFC) or power-off (0xFD) message to ONE
    specific CAN ID -- the same firmware quirk documented in Ch 13 Section 5
    and generalized to multiple motors in Ch 15 Step 2."""
    actuators.bus.send(
        can.Message(arbitration_id=can_id, data=[0xFF] * 7 + [code],
                    is_extended_id=True)
    )


def send_pos_vel(can_id, pos_deg, speed_erpm, accel_erpm_s2):
    """Engineering units in, wire counts (1 count = 10 ERPM) out -- same
    conversion Ch 10 §5 and Ch 14 §3.6 already use. Used only for homing
    below; the trajectory loop itself commands current, not position."""
    actuators.bus.send(make_position_velocity_mode_message(
        can_id, pos_deg, int(speed_erpm / 10), int(accel_erpm_s2 / 10)))


def home_joint(joint):
    """Walk ONE joint gently to its dataset starting angle using
    Position-Velocity Mode, polling position every tick, before the
    trajectory controller ever takes over. Raises if it doesn't arrive
    within HOME_TIMEOUT_S -- the control loop below must never start with
    a joint sitting somewhere other than where its trajectory assumes.

    Joints are homed ONE AT A TIME, not together: if homing fails, this
    keeps the failure isolated to a single joint and message instead of
    two shafts moving toward two different targets at once while a
    problem is still being diagnosed.
    """
    clock_h = TimedLoop(rate=HOME_LOOP_HZ)
    t_home0 = time.perf_counter()
    while clock_h():
        now_deg = actuators.get_position(joint.id, degrees=True)
        err = joint.start_deg - now_deg
        if abs(err) <= HOME_TOLERANCE_DEG:
            print(f"  [home] {joint.label}: reached {now_deg:+.2f} deg "
                  f"(target {joint.start_deg:+.2f} deg).")
            return
        if time.perf_counter() - t_home0 > HOME_TIMEOUT_S:
            raise RuntimeError(
                f"{joint.label} failed to home within {HOME_TIMEOUT_S:.1f} "
                f"s (still {abs(err):.2f} deg from "
                f"{joint.start_deg:+.2f} deg) -- aborting before the "
                f"control loop starts."
            )
        send_pos_vel(joint.id, joint.start_deg, HOME_SPEED_ERPM,
                     HOME_ACCEL_ERPM_S)


try:
    for motor_id in MOTORS:
        lifecycle(0xFC, motor_id)

    # Event-driven start: wait for a few FRESH telemetry frames after the
    # wake-up message, so the firmware has actually processed it before
    # the first control command lands (Ch 14 Section 4.2's bench-learned
    # startup pattern). EACH motor gets its OWN full timeout budget here
    # -- with two motors sharing one wait window, the second motor's wait
    # could otherwise be silently cut short by however long the first
    # motor's wait took.
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

    # Homing only means what it claims if the one-time Set Origin /
    # zero_encoder() commissioning step (Ch 9 §2 / Ch 13 §4.5) was already
    # done when this motor was mounted to the leg -- confirm that once,
    # not per-run, before trusting these targets.
    print("\nHoming both joints to the dataset's start angle...")
    for joint in JOINTS:
        home_joint(joint)
    time.sleep(0.5)   # settle before the trajectory controller takes over

    # Capture each motor's starting angle using the SAME high-level
    # getters proven in Ch 15 Step 2's dual-motor demo -- get_position()
    # already returns a continuous, unwrapped output-shaft angle in rad,
    # so no manual multi-turn bookkeeping is needed here. Read AFTER
    # homing, so P0 reflects where each joint actually ended up, not just
    # the target it was aiming for.
    #
    # KNOWN-GOOD-REFERENCE CORRECTION: home_joint() just confirmed the
    # joint is within HOME_TOLERANCE_DEG of joint.start_deg, so that
    # target IS a trustworthy ground truth for what this read SHOULD say.
    # The same multi-turn unwrap glitch that the in-loop wrap-glitch guard
    # (Section 3.9) protects against can also strike this ONE-OFF read --
    # a single bad telemetry frame right at the homing/current-mode
    # handoff can silently bake a spurious +-360 deg (or more) offset into
    # P0 itself, with no "previous tick" for the in-loop guard to compare
    # against and catch it. Left uncorrected, EVERY p = get_position() -
    # P0 computed for the rest of the run inherits that offset from tick
    # zero -- the controller sees a huge phantom error immediately and
    # commands a huge torque in response, exactly the "knee ran to +319
    # deg and aborted within the first tick or two" failure this fixes.
    # Rounding the raw reading to the nearest whole turn away from the
    # known target removes exactly that offset before P0 is ever used.
    for joint in JOINTS:
        raw_p0 = actuators.get_position(joint.id)
        target_rad = math.radians(joint.start_deg)
        turns_off = round((raw_p0 - target_rad) / (2.0 * math.pi))
        joint.P0 = raw_p0 - turns_off * 2.0 * math.pi
        if turns_off != 0:
            print(f"  [P0 fix] {joint.label}: raw P0 reading was "
                  f"{turns_off:+d} full turn(s) off from the just-homed "
                  f"target ({math.degrees(raw_p0):+.1f} deg vs "
                  f"{joint.start_deg:+.1f} deg) -- corrected before the "
                  f"control loop starts.")

    print(f"\nRunning knee-ankle trajectory: "
          f"knee Kp={KNEE_KP}, Kd={KNEE_KD}  |  "
          f"ankle Kp={ANKLE_KP}, Kd={ANKLE_KD}  |  "
          f"{LOOP_HZ} Hz. Ctrl+C stops.\n")

    clock = TimedLoop(rate=LOOP_HZ)
    t0 = time.perf_counter()
    loop_t_start = t0
    total_duration = traj_duration
    i = 0
    abort_reason = None

    # Status print divisor: 500 Hz / 100 = 5 status blocks per second.
    # This used to be every 20 ticks (25 blocks/s). At that rate the
    # stdout flushes -- especially over SSH -- were themselves stalling
    # the control loop, producing exactly the timing stutter that the
    # wrap-glitch guard below exists to clean up after. Printing is not
    # free inside a 2 ms budget.
    PRINT_EVERY = max(1, LOOP_HZ // 5)

    while clock():
        t_wall = time.perf_counter() - t0
        if t_wall > total_duration:
            break
        t_traj = t_wall   # position along the whole (non-repeating) trajectory

        # Ease-in factor: 0 -> 1 over STARTUP_RAMP_S, then constant at 1.
        # Smoothstep (3u^2 - 2u^3) rather than a straight line, so the ramp
        # arrives at 1.0 with zero slope and does not kick the resonance on
        # its way out. Applied identically to both joints so they stay
        # synchronized.
        if STARTUP_RAMP_S > 0.0 and t_wall < STARTUP_RAMP_S:
            u = t_wall / STARTUP_RAMP_S
            ramp = u * u * (3.0 - 2.0 * u)
        else:
            ramp = 1.0

        for joint in JOINTS:
            pd = float(joint.pos_spline(t_traj))
            vd = float(joint.vel_spline(t_traj))

            # Same getters as Ch 15 Step 2 -- P is the CURRENT reading
            # minus the STARTING reading captured above, so it starts at
            # 0 rad exactly like Pd does (Section 3.3).
            p = actuators.get_position(joint.id) - joint.P0
            v_raw = actuators.get_velocity(joint.id)

            # --- Wrap-glitch guard (Section 3.9) ------------------------
            # If the loop stutters (heavy CPU load, thermal throttling),
            # position telemetry can arrive in a delayed, bunched burst.
            # The multi-turn unwrap logic can then misjudge which way a
            # large jump wrapped and insert a spurious +-360 deg offset
            # that persists for the rest of the run. No real tick-to-tick
            # motion can plausibly exceed a few degrees (even the AK80-9's
            # rated 570 rpm ceiling is under 7 deg per tick at this loop
            # rate), so any jump past GLITCH_JUMP_RAD is almost certainly
            # this glitch, not real motion -- snap it back to the nearest
            # physically sane value instead of trusting it.
            delta = p - joint.P_prev
            if abs(delta) > GLITCH_JUMP_RAD:
                p -= round(delta / (2.0 * math.pi)) * 2.0 * math.pi
            joint.P_prev = p

            # v_raw goes STRAIGHT into the Kd term here -- no filtering.
            # Section 3.9 diagnoses exactly what this costs.
            tau_position = joint.kp * (pd - p)
            tau_velocity = joint.kd * (vd - v_raw)
            tau = (tau_position + tau_velocity) * ramp
            current = tau / KT   # see MAX_CURRENT_A in USER SETTINGS
            if abs(current) > MAX_CURRENT_A:
                current = math.copysign(MAX_CURRENT_A, current)
                joint.clamp_count += 1

            actuators.set_torque(joint.id, current)   # amperes (Ch 13 4.5)

            # --- Runaway guards ----------------------------------------
            # Checked AFTER the command is sent, so the joint is never
            # left holding a bad torque while we decide whether to abort.
            err_deg = abs(math.degrees(pd - p))
            if ABORT_ERROR_DEG > 0.0 and err_deg > ABORT_ERROR_DEG:
                abort_reason = (
                    f"{joint.label} tracking error reached {err_deg:.1f} "
                    f"deg (limit {ABORT_ERROR_DEG:.1f} deg) -- the joint "
                    f"is no longer following the trajectory."
                )

            abs_deg = math.degrees(joint.P0 + p)
            if abs_deg < joint.limit_min_deg or abs_deg > joint.limit_max_deg:
                abort_reason = (
                    f"{joint.label} reached {abs_deg:+.1f} deg, "
                    f"outside its hardware limits "
                    f"[{joint.limit_min_deg:+.1f}, "
                    f"{joint.limit_max_deg:+.1f}] deg -- stopping "
                    f"before the joint reaches its hardstop."
                )

            joint.log["t"].append(t_wall)
            joint.log["pd_deg"].append(math.degrees(pd))
            joint.log["p_deg"].append(math.degrees(p))
            joint.log["vd_rads"].append(vd)
            joint.log["v_raw"].append(v_raw)
            joint.log["tau_position"].append(tau_position)
            joint.log["tau_velocity"].append(tau_velocity)
            joint.log["tau_total"].append(tau)
            joint.log["current"].append(current)

        if abort_reason is not None:
            print(f"\n  [ABORT] {abort_reason}")
            print("  Stopping the trajectory now. The plots below still "
                  "show everything logged up to this point -- the torque "
                  "decomposition will tell you which term ran away.")
            break

        if i % PRINT_EVERY == 0:
            for joint in JOINTS:
                d = joint.log
                print(f"t={t_wall:5.2f}s  {joint.label:5s} (id={joint.id:3d})  "
                      f"Pd={d['pd_deg'][-1]:+7.2f} deg  "
                      f"P={d['p_deg'][-1]:+7.2f} deg  "
                      f"Vd={d['vd_rads'][-1]:+6.3f}  "
                      f"Vraw={d['v_raw'][-1]:+6.3f} rad/s  "
                      f"tau_p={d['tau_position'][-1]:+6.3f}  "
                      f"tau_v={d['tau_velocity'][-1]:+6.3f}  "
                      f"tau={d['tau_total'][-1]:+6.3f} N*m  "
                      f"I={d['current'][-1]:+5.2f} A")
            print()
        i += 1

        if getattr(actuators.notifier, "exception", None):
            print("RX thread died:", actuators.notifier.exception)
            break

    print("\nTrajectory finished.")

except KeyboardInterrupt:
    print("\nCtrl+C -- stopping.")

finally:
    loop_t_end = time.perf_counter()

    # Stop path: zero current, release the firmware -- same pattern as
    # Ch 14 Section 4.2's impedance script, repeated per motor.
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
    print("Motors limp; verify by hand before approaching.")

    # -----------------------------------------------------------------
    # ACHIEVED loop rate.
    # -----------------------------------------------------------------
    n_ticks = len(JOINTS[0].log["t"])
    if n_ticks > 1:
        elapsed = JOINTS[0].log["t"][-1] - JOINTS[0].log["t"][0]
        achieved = (n_ticks - 1) / elapsed if elapsed > 0 else float("nan")
        print(f"Achieved loop rate: {achieved:.1f} Hz "
              f"(nominal {LOOP_HZ} Hz, {100.0 * achieved / LOOP_HZ:.1f}%)")

    for joint in JOINTS:
        if joint.clamp_count > 0:
            print(f"  [warning] {joint.label}: current clamp engaged on "
                  f"{joint.clamp_count}/{n_ticks} ticks "
                  f"({100.0 * joint.clamp_count / n_ticks:.1f}%) -- "
                  f"consider revisiting Kp/Kd if this seems large.")

    if not GENERATE_PLOTS:
        print("Plot generation disabled (GENERATE_PLOTS = False) -- no "
              "figures were produced.")
    elif not any(joint.log["t"] for joint in JOINTS):
        # Nothing was logged (e.g. the bus never came up) -- do not save
        # three empty figures and pretend they are results.
        print("No telemetry was logged, so no plots were produced.")
    else:
        # Save into a "results" folder next to this script.
        script_dir = os.path.dirname(os.path.abspath(__file__))
        results_dir = os.path.join(script_dir, "results")
        os.makedirs(results_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        def plot_two_panel(traces, ylabel, title_suffix, filename_suffix):
            """One consolidated plotting helper for all three figures below
            -- `traces` is the only part that actually differs between
            them.

            traces: list of (log_key, color, linewidth, alpha, legend_label)
            """
            fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
            for ax, joint in zip(axes, JOINTS):
                data = joint.log
                if not data["t"]:
                    continue
                for log_key, color, lw, alpha, legend_label in traces:
                    ax.plot(data["t"], data[log_key], color=color,
                            linewidth=lw, alpha=alpha, label=legend_label)
                ax.set_ylabel(f"{joint.label} {ylabel}")
                ax.set_title(f"{joint.label} joint: {title_suffix}")
                ax.legend(loc="best")
                ax.grid(True, alpha=0.3)
            axes[-1].set_xlabel("Time [s]")
            fig.tight_layout()
            out_path = os.path.join(results_dir,
                                    f"code-run-{stamp}{filename_suffix}.png")
            fig.savefig(out_path, dpi=300, bbox_inches="tight")
            print(f"Plot saved to: {out_path}")
            return fig

        # STEP 6a -- reference vs. actual POSITION, one panel per joint
        plot_two_panel(
            [("pd_deg", "blue", 1.5, 1.0, "Reference (dataset)"),
             ("p_deg", "red", 1.2, 1.0, "Actual (motor telemetry)")],
            ylabel="angle [deg]",
            title_suffix="reference vs. actual trajectory",
            filename_suffix="",
        )

        # STEP 6b -- desired vs. RAW velocity (Section 3.9's central
        # figure: the raw get_velocity() staircase against the smooth
        # reference).
        plot_two_panel(
            [("vd_rads", "blue", 1.5, 1.0, "Desired velocity (spline derivative)"),
             ("v_raw", "red", 1.0, 0.9, "Actual velocity (get_velocity())")],
            ylabel="velocity [rad/s]",
            title_suffix="desired vs. actual velocity",
            filename_suffix="-velocity",
        )

        # STEP 6c -- spring torque term vs. damper torque term, to show
        # WHICH term carries the quantization noise (Section 3.9.1).
        plot_two_panel(
            [("tau_position", "green", 1.0, 0.8, "tau_position = Kp*(Pd-P)"),
             ("tau_velocity", "orange", 1.0, 0.8, "tau_velocity = Kd*(Vd-V)")],
            ylabel="torque [N*m]",
            title_suffix="torque decomposition",
            filename_suffix="-torque",
        )

        # The backend was already chosen at import time, so this either
        # opens windows (TkAgg) or does nothing at all (Agg) -- either
        # way the PNGs above are already on disk.
        if SHOW_PLOTS:
            try:
                plt.show()
            except Exception:
                pass

print("Done.")