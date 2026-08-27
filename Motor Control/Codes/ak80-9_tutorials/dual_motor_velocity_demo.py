#!/usr/bin/env python3
"""
dual_motor_velocity_demo.py — Chapter 15, Step 3: spin TWO AK80-9 actuators
at the same time, each at its own steady speed, using Servo Velocity Loop
Mode (Ch 8) for both.

WHAT THIS PROGRAM DOES
-----------------------
Two motors share one CAN bus:
    CAN ID 100  →  the KNEE actuator  →  target speed  1.0 rad/s
    CAN ID   1  →  the ANKLE actuator →  target speed  2.0 rad/s

Both targets are commanded from the SAME 200 Hz control loop, so both
motors spin continuously and simultaneously for RUN_S seconds, then both
are brought to a controlled stop together.

This is deliberately the simplest possible two-motor program. Step 4 of
this chapter reuses every pattern here — one ActuatorGroup, lifecycle()
called per motor ID, a small per-motor dictionary of targets — and simply
replaces "constant velocity" with "position read from a time-indexed
trajectory table."

WHAT YOU SHOULD SEE WHEN YOU RUN IT
-------------------------------------
  1. ~0.5 s of stillness while both motors wake up.
  2. Both shafts begin turning at the same moment: the knee motor settles
     into a steady, gentle rotation at 1.0 rad/s (~9.5 rpm); the ankle
     motor settles into a steady rotation at 2.0 rad/s (~19 rpm) — twice
     as fast, running the whole time alongside the knee motor.
  3. The terminal prints one line per motor, ~10 times a second, so you
     can watch both v_cmd/vel pairs track their own targets side by side.
  4. After RUN_S seconds both motors are commanded to zero velocity, given
     a moment to stop, then released ("limp") together, and the program
     prints "Done."

Press Ctrl+C at any moment — the cleanup code still runs, and BOTH motors
are stopped and released safely.

WANT BOTH MOTORS AT THE SAME SPEED INSTEAD? Change TARGET_VEL below so
both dictionary values are equal, e.g. {100: 1.5, 1: 1.5}. Nothing else
in the script needs to change.
"""

# ---------------------------------------------------------------------------
# IMPORTS
# ---------------------------------------------------------------------------
import time                # clocks and sleep()
import can                 # raw CAN messages, needed for the lifecycle() quirk

from epicallypowerful.actuation import ActuatorGroup   # EP's main motor class
from epicallypowerful.toolbox import TimedLoop         # EP's metronome

# ---------------------------------------------------------------------------
# SETTINGS
# ---------------------------------------------------------------------------
MOTOR_TYPE = 'AK80-9-V3-servo'   # both motors are in Servo operating mode
                                 # (Ch 13 §4.1) — same dialect for both

# Every motor on the bus, and the EP model+dialect string it uses.
# This one dictionary is what makes the ActuatorGroup a TWO-motor group.
MOTORS = {
    100: MOTOR_TYPE,   # knee actuator
      1: MOTOR_TYPE,   # ankle actuator
}

# Each motor's target speed, in rad/s at the OUTPUT shaft. Change these two
# numbers to change what each joint does — set them equal to make both
# motors spin at the same speed.
TARGET_VEL = {
    100: 1.0,   # knee:  1.0 rad/s ( ≈  9.5 rpm )
      1: 2.0,   # ankle: 2.0 rad/s ( ≈ 19.1 rpm )
}

LOOP_HZ = 200     # BOTH motors are configured for a 200 Hz feedback rate
                 # (this chapter's opening section) — so the control loop
                 # runs at 200 Hz too, matching command rate to sampling
                 # rate to telemetry rate for both actuators at once.
RUN_S   = 15.0    # total run time, in seconds

# ---------------------------------------------------------------------------
# STEP 1 — Open the CAN bus and register BOTH motors
# ---------------------------------------------------------------------------
# One ActuatorGroup, built from the MOTORS dictionary above, now manages
# bus traffic for CAN ID 100 AND CAN ID 1 together (Ch 13 §4.5, Ch 14 §1.2).
actuators = ActuatorGroup.from_dict(MOTORS)

# ---------------------------------------------------------------------------
# STEP 2 — The firmware power-on / power-off workaround, made multi-motor
# ---------------------------------------------------------------------------
def lifecycle(code, can_id):
    """Send the motor's power-on (0xFC) or power-off (0xFD) message to ONE
    specific CAN ID. Same 8-byte frame as Ch 13 §5 / Ch 14 §1.2 — the only
    change is that the target CAN ID is now a parameter, not a constant,
    so this one function serves every motor on the bus."""
    actuators.bus.send(
        can.Message(
            arbitration_id=can_id,
            data=[0xFF] * 7 + [code],
            is_extended_id=True,
        )
    )

# Wake EVERY motor in the group, one at a time, by its own CAN ID.
for motor_id in MOTORS:
    lifecycle(0xFC, motor_id)
time.sleep(0.5)   # let both motors activate and their first telemetry arrive

print(f"AK80-9 dual-motor demo on can0. Knee (ID 100) -> "
      f"{TARGET_VEL[100]:+.1f} rad/s, Ankle (ID 1) -> "
      f"{TARGET_VEL[1]:+.1f} rad/s, for {RUN_S:.0f}s. Ctrl+C stops.\n")

# ---------------------------------------------------------------------------
# STEP 3 — The control loop: command BOTH motors every tick
# ---------------------------------------------------------------------------
clock = TimedLoop(rate=LOOP_HZ)
t0 = time.perf_counter()
i = 0

try:
    while clock():
        t = time.perf_counter() - t0
        if t > RUN_S:
            break

        # Send every motor its own target velocity, inside the SAME tick.
        # This is what "simultaneous" means here: both commands leave the
        # Pi within the same 5 ms (200 Hz) control-loop iteration.
        for motor_id, v in TARGET_VEL.items():
            actuators.set_velocity(motor_id, v, 0)   # 3rd arg unused in Servo

        # Print one line per motor, ~10 times a second, so both columns
        # of telemetry are readable side by side.
        if i % 20 == 0:
            for motor_id, v in TARGET_VEL.items():
                print(f"t={t:5.2f}s  id={motor_id:3d}  v_cmd={v:+5.2f}  "
                      f"pos={actuators.get_position(motor_id):+7.2f} rad  "
                      f"vel={actuators.get_velocity(motor_id):+5.2f} rad/s  "
                      f"|i|={abs(actuators.get_torque(motor_id)):4.2f} A")
            print()   # blank line between ticks, for readability

        i += 1

        # Same background-thread safety net as Ch 13 §5.
        if getattr(actuators.notifier, "exception", None):
            print("RX thread died:", actuators.notifier.exception)
            break

# ---------------------------------------------------------------------------
# STEP 4 — Cleanup: stop and release BOTH motors, always
# ---------------------------------------------------------------------------
finally:
    for motor_id in MOTORS:
        actuators.set_velocity(motor_id, 0.0, 0)     # controlled stop
    time.sleep(0.3)
    for motor_id in MOTORS:
        actuators.set_torque(motor_id, 0.0)          # 0 A -> limp
    for motor_id in MOTORS:
        lifecycle(0xFD, motor_id)                    # polite power-off, each

print("\nDone.")
