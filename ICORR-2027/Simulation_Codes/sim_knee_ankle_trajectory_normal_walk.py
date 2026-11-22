#!/usr/bin/env python3
"""
sim_knee_ankle_trajectory_normal_walk.py -- MuJoCo simulation of the OSL V2 bench leg
following the level-walking (1.2 m/s) gait trajectory, optionally driving the REAL leg at the same time.

Hardware twin: Hardware_Codes/knee_ankle_trajectory_PID_control_me_normal_walk.py
(run THAT script on the leg's Raspberry Pi for a sim-to-real test).

    SIM_2_REAL = 0   simulation only. Builds the reference from datasets/ with the
                     settings below, simulates it in real time, logs it. No network.
    SIM_2_REAL = 1   sim-to-real. Connects to the leg Pi (HARDWARE_HOST), lets you ARM
                     and START it from this terminal, runs the simulation and the leg
                     from the same start instant, and logs BOTH into one run folder.

Results: results/normal_walk/run_<N>_<date-time>/   (see README.md)
Video:   python make_live_plot_video.py results/normal_walk/run_<N>_<date-time>

The plant, drivetrain and control law are the final paper model (see osl_sim/engine.py
and validate_engine.py). The reference settings below are copied from the
hardware script; in sim-to-real mode the leg's own resolved reference and gains are used
(USE_HARDWARE_SETTINGS) and any difference from these settings is printed and logged.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

# ===========================================================================
#  USER SETTINGS
# ===========================================================================
GAIT_NAME = "normal_walk"          # must match SIM_LINK_GAIT in the hardware script

# --- Sim-to-real pipeline ---------------------------------------------------
SIM_2_REAL = 1                # 0 = simulation only, 1 = drive the real leg too

# HARDWARE_HOST: the leg Pi's address. Leave exactly ONE line below uncommented.
# See "Connecting the leg and the simulation" in README.md (sections 6 and 9)
# for the full procedure, including what to install, for each case.
# ---- SAME network (lab Wi-Fi/Ethernet): this leg Pi's current LAN IP ------
HARDWARE_HOST = "172.31.15.243"
# HARDWARE_HOST = "rpi1.local"        # mDNS hostname alternative, same network only
# ---- DIFFERENT network / over the internet: this leg Pi's Tailscale IP ----
# HARDWARE_HOST = "100.x.x.x"               # `tailscale ip -4` on the leg Pi, once installed

SIM_LINK_PORT = 8795          # must match the hardware script
SIM_LINK_TOKEN = "osl-sim2real-change-me"   # must match; env OSL_SIM2REAL_TOKEN overrides
GO_LEAD_S = 1.0               # start is scheduled this far ahead (use 1.5-2 s over internet)
LINK_TIMEOUT_S = 3.0          # sim gives up if the leg is silent this long (use 5.0 over internet)
USE_HARDWARE_SETTINGS = True  # True = use the leg's reference + gains (recommended)

# --- Simulation display / pacing --------------------------------------------
SHOW_VIEWER = True            # open the MuJoCo 3-D viewer
REALTIME = True               # pace the simulation to the wall clock (forced on in sim-to-real)
WAIT_FOR_ENTER_TO_START = True   # standalone: wait for Enter before starting
KEEP_VIEWER_OPEN = True       # keep the viewer open after the run until you close it

# --- Reference trajectory (copied from the hardware script) -----------------
ANGLE_CSV_PATH = os.path.join(_HERE, "datasets", "normal_walk_1-2", "AB01_normal_walk_1_1-2_angle.csv")
GRF_CSV_PATH = os.path.join(_HERE, "datasets", "normal_walk_1-2", "AB01_normal_walk_1_1-2_grf.csv")
STRIDE_SELECTION = "heel_strike"   # "heel_strike" (GRF) or "time_window" (stairs clips)
LEG = "l"
STRIDE_START_INDEX = 0       # 0-based heel strike the trajectory starts at
STRIDE_COUNT = 5            # consecutive strides
TIME_SCALE = 1
LOOP_HZ = 200
KNEE_DATASET_TO_ENCODER_SIGN = 1.0
ANKLE_DATASET_TO_ENCODER_SIGN = -1.0
KNEE_TRAJECTORY_START_DEG = -10
ANKLE_TRAJECTORY_START_DEG = None
AUTO_FIT_TO_LIMITS = True
# hardware limits, ENCODER frame (knee flexion-negative) -- used for the auto-fit only
KNEE_LIMIT_MIN_DEG = -120.0
KNEE_LIMIT_MAX_DEG = 2.0
ANKLE_LIMIT_MIN_DEG = -30.0
ANKLE_LIMIT_MAX_DEG = 30.0

# --- PID gains (Kp A/deg, Ki A/(deg.s), Kd A/(deg/s)) -- same as the hardware ---
KNEE_KP = 3.0
KNEE_KI = 0.0
KNEE_KD = 0.10
ANKLE_KP = 2.0
ANKLE_KI = 0.0
ANKLE_KD = 0.075
# ===========================================================================
#  END OF USER SETTINGS
# ===========================================================================

if __name__ == "__main__":
    from osl_sim.runner import run
    raise SystemExit(run(dict(globals())))
