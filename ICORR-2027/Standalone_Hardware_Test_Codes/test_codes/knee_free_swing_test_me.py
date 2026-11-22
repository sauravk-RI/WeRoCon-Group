#!/usr/bin/env python3
"""
knee_free_swing_test_me.py -- KNEE free-swing test (Elery et al., IEEE T-RO 2020,
Sec. IV-A.3, Figs. 9 and 10).

The knee motor is NOT used: this script never opens the CAN bus, so the knee is
unpowered and back-driven by the weight of the shank and foot alone. Only the
knee's magnetic encoder (AS5048B) is read ("_me"), using the same register
read / offset / wrap code as the other knee scripts in this repo.

PROCEDURE (NUM_TRIALS trials in ONE run)
  1. Terminal: press Enter  -> trial starts, the encoder is monitored (not logged).
  2. Bring the knee to the top extremum (~80 deg flexion). The terminal tells
     you the moment the angle reaches TARGET_PEAK_DEG.
  3. Release it without pushing. The moment the knee starts to fall, logging
     starts (t = 0 is the peak/release instant; the few samples between the
     peak and the detection are back-filled, so nothing is lost).
  4. Logging stops once the knee is back at ~0 deg (hard stop). The terminal
     prompts for the next trial; press Enter when ready.
  5. After the last trial: one CSV (all trials) + one overlay plot (all trials
     vs time, like Fig. 10) are saved to
     results/free_swing_test_knee/run_<N>/ (N auto-increments).

Angles are the encoder's RELATIVE angle (raw - MAGNETIC_ENCODER_OFFSET, wrapped).
Knee flexion reads NEGATIVE on this encoder; it is logged and plotted as a
POSITIVE flexion angle (DISPLAY_SIGN = -1). The raw relative value is also kept
in the CSV (column encoder_relative_deg).

Ctrl+C at any time: finished trials are still saved (an unfinished trial is
discarded).
"""

# ===========================================================================
#  USER SETTINGS
# ===========================================================================
NUM_TRIALS = 4

# --- Release / stop detection (all in positive flexion degrees) -------------
TARGET_PEAK_DEG = 80.0        # terminal notice when the knee reaches this angle
ARM_MIN_ANGLE_DEG = 50.0      # release detection is only active once the knee has been above this
RELEASE_DROP_DEG = 0.5        # knee has fallen this far below its running peak -> decrease has started (smaller = earlier trigger, more noise-sensitive)
ZERO_TOL_DEG = 1.0            # "back at zero" = flexion <= this ...
ZERO_SETTLE_S = 0.10          # ... continuously for this long -> trial over
TRIAL_TIMEOUT_S = 5.0         # stop a trial that never returns to zero (kept, flagged incomplete)

# --- Sampling ---------------------------------------------------------------
LOOP_HZ = 500                 # target rate (actual rate is limited by the I2C read time)

# --- Magnetic encoder (AS5048B), same as the other knee scripts -------------
I2C_BUS = 1
AS5048B_ADDR = 0x40
ANGLE_HIGH_REG = 0xFE
ANGLE_LOW_REG = 0xFF

# KNEE ENCODER OFFSET (deg): the raw encoder reading at the knee's zero (full extension).
# relative angle = raw - MAGNETIC_ENCODER_OFFSET. Set it so the knee reads ~0 at the hard stop.
MAGNETIC_ENCODER_OFFSET = 121.8   # deg
DISPLAY_SIGN = -1.0               # encoder flexion is negative -> logged/plotted positive
MAX_CONSECUTIVE_I2C_FAILS = 25
GLITCH_JUMP_DEG = 15.0            # implausible jump between two consecutive samples
MAX_CONSECUTIVE_WRAP_REJECTS = 10

# --- Output -----------------------------------------------------------------
GENERATE_PLOTS = True
SHOW_PLOTS = True
# ===========================================================================
#  END OF USER SETTINGS
# ===========================================================================

import csv
import os
import time

from smbus2 import SMBus

import matplotlib
if SHOW_PLOTS and os.environ.get("DISPLAY"):
    try:
        matplotlib.use("TkAgg")
    except Exception:
        matplotlib.use("Agg")
else:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Encoder
# ---------------------------------------------------------------------------
def wrap_deg(angle_deg):
    return ((angle_deg + 180.0) % 360.0) - 180.0


def read_relative_angle_deg(bus):
    """Relative (offset-removed) encoder angle in degrees, wrapped to (-180, 180]."""
    hi, lo = bus.read_i2c_block_data(AS5048B_ADDR, ANGLE_HIGH_REG, 2)
    raw14 = ((hi << 6) | (lo & 0x3F)) & 0x3FFF
    return wrap_deg((raw14 / 16384.0) * 360.0 - MAGNETIC_ENCODER_OFFSET)


class Encoder:
    """Safe reader: a failed I2C read reuses the last good angle (raises after too
    many in a row); an implausible jump is rejected the same way."""

    def __init__(self, bus):
        self.bus = bus
        self.last = None
        self.fail_streak = 0
        self.fail_total = 0
        self.wrap_streak = 0
        self.reject_total = 0

    def read(self):
        try:
            a = read_relative_angle_deg(self.bus)
            self.fail_streak = 0
        except OSError as e:
            self.fail_total += 1
            self.fail_streak += 1
            if self.last is None or self.fail_streak > MAX_CONSECUTIVE_I2C_FAILS:
                raise RuntimeError(f"AS5048B I2C read failed {self.fail_streak}x in a row "
                                   f"({e}) -- check wiring / address / power.") from e
            return self.last
        if self.last is not None and abs(a - self.last) > GLITCH_JUMP_DEG:
            self.wrap_streak += 1
            if self.wrap_streak <= MAX_CONSECUTIVE_WRAP_REJECTS:
                self.reject_total += 1
                return self.last
        self.wrap_streak = 0
        self.last = a
        return a


# ---------------------------------------------------------------------------
# One trial
# ---------------------------------------------------------------------------
def run_trial(enc, trial_no):
    """Returns (t_list, enc_rel_list, complete_flag). Blocks until the trial ends."""
    period = 1.0 / LOOP_HZ
    peak = -1e9                    # running max of flexion (deg)
    peak_idx = 0
    hist = []                      # (t_perf, enc_rel) since start of monitoring; trimmed to the peak
    notified = False
    armed = False
    logging_on = False
    t0 = None
    t_log, a_log = [], []
    zero_since = None
    next_tick = time.perf_counter()

    while True:
        now = time.perf_counter()
        enc_rel = enc.read()
        flex = DISPLAY_SIGN * enc_rel

        if not logging_on:
            # ---- monitoring: keep only samples from the running peak onward ----
            if flex >= peak:
                peak = flex
                hist = []
            hist.append((now, enc_rel))
            if not notified and flex >= TARGET_PEAK_DEG:
                notified = True
                print(f"  >>> Knee reached {flex:.1f} deg (target {TARGET_PEAK_DEG:.0f}). "
                      f"Release it when ready.")
            if peak >= ARM_MIN_ANGLE_DEG and not armed:
                armed = True
            if armed and flex <= peak - RELEASE_DROP_DEG:
                # Free fall began at the peak sample (hist[0]); back-fill from there.
                logging_on = True
                t0 = hist[0][0]
                for tp, a in hist:
                    t_log.append(tp - t0)
                    a_log.append(a)
                print(f"  Free fall detected (peak {peak:.1f} deg) -- logging...")
        else:
            t_log.append(now - t0)
            a_log.append(enc_rel)
            if flex <= ZERO_TOL_DEG:
                if zero_since is None:
                    zero_since = now
                elif now - zero_since >= ZERO_SETTLE_S:
                    return t_log, a_log, True
            else:
                zero_since = None
            if now - t0 > TRIAL_TIMEOUT_S:
                print(f"  [warning] Trial {trial_no}: knee did not return to zero within "
                      f"{TRIAL_TIMEOUT_S:.1f} s -- stopping, trial flagged incomplete.")
                return t_log, a_log, False

        next_tick += period
        sleep = next_tick - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_tick = time.perf_counter()   # running behind; don't accumulate debt


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------
def make_run_dir():
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "results", "free_swing_test_knee")
    os.makedirs(root, exist_ok=True)
    nums = [int(n[4:]) for n in os.listdir(root) if n.startswith("run_") and n[4:].isdigit()]
    run_no = max(nums, default=0) + 1
    path = os.path.join(root, f"run_{run_no}")
    os.makedirs(path)
    return run_no, path


def save_outputs(trials, run_dir):
    if not trials:
        print("No completed trials -- nothing to save.")
        return
    csv_path = os.path.join(run_dir, "free_swing_knee.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["trial", "time_s", "knee_angle_deg", "encoder_relative_deg", "complete"])
        for n, (t, a, ok) in trials.items():
            for ti, ai in zip(t, a):
                w.writerow([n, f"{ti:.5f}", f"{DISPLAY_SIGN * ai:.4f}", f"{ai:.4f}", int(ok)])
    print(f"CSV saved: {csv_path}")

    if not GENERATE_PLOTS:
        return
    styles = ["-", "--", "-.", ":"]
    colors = ["black", "red", "orange", "black"]
    fig, ax = plt.subplots(figsize=(7, 4))
    for i, (n, (t, a, ok)) in enumerate(trials.items()):
        ax.plot(t, [DISPLAY_SIGN * x for x in a], styles[i % 4], color=colors[i % 4],
                lw=1.4, label=f"Trial {n}" + ("" if ok else " (incomplete)"))
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Knee Angle (°)")
    ax.set_title("Knee free swing: return to zero after release")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    png = os.path.join(run_dir, "free_swing_knee.png")
    fig.savefig(png, dpi=200)
    print(f"Plot saved: {png}")
    if SHOW_PLOTS and os.environ.get("DISPLAY"):
        plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    run_no, run_dir = make_run_dir()
    print("=" * 70)
    print(f"Knee free-swing test, {NUM_TRIALS} trials. Run #{run_no} -> {run_dir}")
    print("The knee motor is not used (unpowered). Only the encoder is read.")
    print("=" * 70)
    trials = {}
    with SMBus(I2C_BUS) as bus:
        enc = Encoder(bus)
        try:
            for n in range(1, NUM_TRIALS + 1):
                input(f"\n[Trial {n}/{NUM_TRIALS}] Press Enter to start monitoring, "
                      f"then lift the knee to ~{TARGET_PEAK_DEG:.0f} deg and release... ")
                print("  Monitoring the knee encoder (not logging yet)...")
                t, a, ok = run_trial(enc, n)
                trials[n] = (t, a, ok)
                dur = t[-1] if t else 0.0
                print(f"  Trial {n} done: {len(t)} samples over {dur:.3f} s "
                      f"(~{len(t) / dur if dur else 0:.0f} Hz), "
                      f"start angle {max(DISPLAY_SIGN * x for x in a):.1f} deg.")
        except KeyboardInterrupt:
            print("\nInterrupted -- saving finished trials (current trial discarded).")
        except RuntimeError as e:
            print(f"\n[error] {e}\nSaving finished trials.")
        if enc.fail_total or enc.reject_total:
            print(f"[note] I2C read failures: {enc.fail_total}, rejected glitches: {enc.reject_total}")
    save_outputs(trials, run_dir)


if __name__ == "__main__":
    main()
