#!/usr/bin/env python3
"""
first_motion.py — First-motion demo: driving the AK80-9 from a Raspberry Pi 5
                using Python and the EPICally Powerful (EP) library.

WHAT THIS PROGRAM DOES
----------------------
This program makes the motor's output shaft swing smoothly back and forth,
like a slow pendulum, for 15 seconds — by commanding the motor's SPEED
(not its position) many times per second.

The speed we command follows a sine wave over time:

    speed = 1.0 * sin(2π * 0.2 * t)      [rad/s]

which means:
  - the shaft speed rises smoothly from 0 up to +1.0 rad/s (about 9.5 rpm),
  - slows back down to 0, reverses, speeds up to -1.0 rad/s the other way,
  - and repeats. One full back-and-forth takes 5 seconds (0.2 cycles/second),
    so in 15 seconds you will see 3 complete swings.

WHAT YOU SHOULD SEE WHEN YOU RUN IT
-----------------------------------
  1. About half a second of stillness (the motor is being "woken up").
  2. The shaft starts turning gently, speeds up, slows, reverses — a smooth,
     continuous sinusoidal rocking motion. The motor's green LED stays on.
  3. The terminal prints ~10 lines per second showing the speed we COMMANDED
     next to the speed the motor MEASURED — the two should track each other
     closely. It also shows shaft position (radians) and motor current (amps).
  4. After 15 s the motor is brought to a stop, then released ("limp" —
     you could turn the shaft by hand), and the program prints "Done."

Press Ctrl+C at any moment to stop early — the cleanup code at the bottom
still runs and the motor is still stopped and released safely.

ONE IMPORTANT QUIRK THIS CODE WORKS AROUND
------------------------------------------
Our motor's firmware requires a special "power on" CAN message before it will
keep obeying commands (and a matching "power off" message when we are done).
The EP library does NOT send these itself — without them, the motor obeys for
about half a second and then shuts itself off mid-motion. So this script sends
those two lifecycle messages manually. See the `lifecycle()` function below.
"""

# ---------------------------------------------------------------------------
# IMPORTS — the toolboxes this program needs
# ---------------------------------------------------------------------------
import math                # gives us math.sin() and math.pi for the sine wave
import time                # gives us clocks (perf_counter) and sleep()
import can                 # "python-can": lets us hand-build raw CAN messages
                           # (needed only for the power on/off quirk above)

from epicallypowerful.actuation import ActuatorGroup   # EP's main motor class:
                           # opens the CAN bus and talks to one or more motors
from epicallypowerful.toolbox import TimedLoop         # EP's metronome: keeps
                           # our control loop ticking at a steady rate

# ---------------------------------------------------------------------------
# SETTINGS — the numbers that define this demo (safe to tune, gently!)
# ---------------------------------------------------------------------------
CAN_ID   = 100               # the motor's address on the CAN bus — must match
                             # the CAN ID you configured in the CubeMars tool
MOTOR    = 'AK80-9-V3-servo' # tells EP which motor AND which protocol to use:
                             # 'AK80-9-V3' = our motor model; the '-servo'
                             # suffix selects Servo mode (the mode our motor's
                             # firmware is configured in). Without the suffix,
                             # EP would speak MIT mode and the motor would
                             # silently ignore every command.
LOOP_HZ  = 100               # how many times PER SECOND we send a new speed
                             # command (100 Hz = one command every 10 ms)
VEL_AMP  = 1.0               # the PEAK speed of the swing, in rad/s at the
                             # output shaft (1.0 rad/s ≈ 9.5 rpm — gentle)
SWING_HZ = 0.2               # how many full back-and-forth cycles per second
                             # (0.2 Hz = one complete swing every 5 seconds)
RUN_S    = 15.0              # total running time of the demo, in seconds

# ---------------------------------------------------------------------------
# STEP 1 — Open the CAN bus and set up the motor object
# ---------------------------------------------------------------------------
# This one line does a lot: EP configures the can0 interface, opens it,
# starts a background thread that listens for the motor's telemetry
# (position / speed / current reports, arriving ~200x per second),
# and sends a couple of harmless "zero command" frames to the motor.
# From here on, `actuators` is our handle for talking to the motor.
actuators = ActuatorGroup.from_dict({CAN_ID: MOTOR})

# ---------------------------------------------------------------------------
# STEP 2 — The firmware "power on / power off" workaround
# ---------------------------------------------------------------------------
def lifecycle(code):
    """Send the motor's special power-on (0xFC) or power-off (0xFD) message.

    The message is 8 bytes: FF FF FF FF FF FF FF followed by the code byte,
    sent to the motor's plain CAN ID. This is exactly the frame the proven
    TMotorCANControl library sends in its start()/stop() — EP omits it,
    which is why the motor kept shutting off mid-demo before we added this.
    """
    actuators.bus.send(                       # use EP's already-open CAN bus
        can.Message(
            arbitration_id=CAN_ID,            # address it to our motor (100)
            data=[0xFF] * 7 + [code],         # FF FF FF FF FF FF FF + FC/FD
            is_extended_id=True,              # the protocol uses 29-bit IDs
        )
    )

lifecycle(0xFC)          # "wake up and stay awake" 
time.sleep(0.5)          # give the motor a moment to activate, and let the
                         # first telemetry reports arrive before we start

print(f"AK80-9 servo on can0, ID {CAN_ID}. {RUN_S:.0f}s sine. Ctrl+C stops.\n")

# ---------------------------------------------------------------------------
# STEP 3 — The control loop: command a new speed every 10 ms for 15 s
# ---------------------------------------------------------------------------
clock = TimedLoop(rate=LOOP_HZ)   # EP's metronome: each call to clock()
                                  # waits just long enough so the loop body
                                  # runs exactly LOOP_HZ times per second
t0 = time.perf_counter()          # remember the start time (a stopwatch zero)
i = 0                             # loop counter — used only to thin out prints

try:                              # `try` so the cleanup below ALWAYS runs,
                                  # even if you press Ctrl+C mid-motion
    while clock():                              # tick... tick... at 100 Hz
        t = time.perf_counter() - t0            # seconds elapsed since start
        if t > RUN_S:                           # 15 seconds are up?
            break                               # leave the loop, go to cleanup

        # The heart of the demo: compute this instant's target speed from
        # the sine formula.  sin() sweeps smoothly between -1 and +1, so
        # v sweeps smoothly between -VEL_AMP and +VEL_AMP.
        v = VEL_AMP * math.sin(2 * math.pi * SWING_HZ * t)

        # Send that speed to the motor. Units are rad/s at the OUTPUT shaft —
        # EP converts to the motor's internal units (ERPM) for us. The third
        # argument is a gain that Servo mode doesn't use, so we pass 0.
        actuators.set_velocity(CAN_ID, v, 0)

        # Read back what the motor is reporting right now (its latest
        # telemetry). These getters are instant — they just return the most
        # recent values the background listener thread has stored.
        if i % 10 == 0:                          # only print every 10th loop
                                                 # (10 lines/s — readable)
            print(f"t={t:5.2f}s  v_cmd={v:+5.2f}  "
                  f"pos={actuators.get_position(CAN_ID):+7.2f} rad  "     # where the shaft is
                  f"vel={actuators.get_velocity(CAN_ID):+5.2f} rad/s  "   # how fast it's really turning
                  f"|i|={abs(actuators.get_torque(CAN_ID)):4.2f} A")      # motor current (in AMPS in
                                                                          # servo mode, not N·m!)
        i += 1                                   # count this loop iteration

        # Safety net: EP's telemetry listener runs in a background thread.
        # If that thread ever crashes, our readings would silently freeze.
        # This check notices the crash immediately and stops the demo.
        if getattr(actuators.notifier, "exception", None):
            print("RX thread died:", actuators.notifier.exception)
            break

# ---------------------------------------------------------------------------
# STEP 4 — Cleanup: always stop the motor gracefully
# ---------------------------------------------------------------------------
finally:                          # runs no matter HOW the loop ended
    actuators.set_velocity(CAN_ID, 0.0, 0)   # command speed 0 → motor brakes
                                             # itself to a controlled stop
    time.sleep(0.3)                          # give it a moment to stop fully
    actuators.set_torque(CAN_ID, 0.0)        # command 0 amps → no holding
                                             # force at all; shaft is "limp"
                                             # and can be turned by hand
    lifecycle(0xFD)                          # polite "power off" message —
                                             # mirrors the wake-up at the top

print("\nDone.")
