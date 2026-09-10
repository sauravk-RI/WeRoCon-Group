#!/usr/bin/env python3
"""
mit_impedance_ep.py — Impedance (MIT-law) control of the AK80-9 V3.0 from
a Raspberry Pi, through EP's Servo dialect. The law runs on the Pi:

        tau = DES_TORQUE + KP*(DES_POS - pos) + KD*(DES_VEL - vel)

and is delivered as motor current = tau / 0.5701 [A]. This is the same
equation the "MIT mode" that runs inside the driver on 
giving commands from the CubeMars Upper Computer Software
— here it runs on Python instead, 200x per second.

No value is clamped or checked by this code. YOU choose every number;
the comments below tell you the physics and the motor ratings to respect.
"""

# ============================================================================================
#  USER SETTINGS — Set Your Own Values for Position, Velocity and Torque Control Respectively.
# ============================================================================================
DES_POS    = 0.0    # [rad] target, relative to position at startup.
                    # Any value works (multi-turn OK, error is unwrapped).

DES_VEL    = 0.0    # [rad/s] velocity setpoint / feedforward.
                    # Physics cap: no-load top speed ~ 1.16 rad/s per volt
                    # of bus voltage (KV100 motor / 9:1 gear), e.g. ~28
                    # rad/s at 24 V, ~41 rad/s at the 48 V rating.

DES_TORQUE = 0.0    # [N*m] feedforward torque.
                    # AK80-9 ratings: ~9 N*m continuous (~16 A), 18 N*m
                    # peak (~32 A) for seconds only — sustained peak
                    # overheats the windings in minutes. Watch the temp
                    # column; back off above ~60 C.

KP         = 100.0    # [N*m/rad] SPRING STRENGTH. Bigger = pulls harder to
                    # the target and feels stiffer to your finger.
                    # Careful: the Pi learns the shaft's position a split
                    # second LATE (the reading travels over the wire).
                    # A very strong spring reacting to old information
                    # over-corrects, and the shaking GROWS instead of
                    # settling — like catching a ball while watching it
                    # on delayed video. Good zone here: KP = 1 to 3.
                    # Rule of thumb for this rig:
                    #     KP_max ~ LOOP_HZ / 100    (with KD ~ KP / 10)
                    # e.g. 200 Hz -> ~2-3, 500 Hz -> ~5, 1000 Hz -> ~10.
                    # It flattens at the top (the CAN HAT has a fixed few
                    # ms of delay no loop rate removes) — approach in
                    # small steps and back off at the first shake.
                    # (The manual's "up to 500" is for the math running
                    # INSIDE the motor, which reacts ~100x faster.)
                    # FORCE CEILING: the spring can only pull as hard as
                    # the power supply allows. With a PSU current limit
                    # I_lim, pushing the shaft beyond about
                    #     (I_lim * 0.5701) / KP   radians
                    # hits that limit -> supply voltage folds -> driver
                    # undervoltage-cuts and the torque VANISHES until you
                    # ease off (e.g. 3 A limit, KP=2 -> gives up at ~49
                    # deg of push). Raise the PSU limit or lower KP to
                    # move this ceiling. Not a bug — it's the weakest
                    # link in the power chain announcing itself.

KD         = 0.0    # [N*m/(rad/s)] SHOCK ABSORBER. Brakes the motion so
                    # the shaft doesn't bounce around the target.
                    # Too little -> it rings/bounces like a spring with
                    # no damper. Too much (> ~1 here) -> the speed
                    # reading's small noise gets multiplied into a
                    # buzzing, chattering motor.
                    # Easy rule: start with  KD = KP / 10.
                    # KD = 0 -> pure bouncy spring (fun to feel once;
                    # use small KP and keep the shaft free).

RUN_S      = 60.0   # [s] run time; Ctrl+C always stops earlier.

CAN_ID     = 1    # Set this value according to your motor's CAN ID.

LOOP_HZ    = 500    # how many times per second the Pi checks the shaft
                    # and corrects. More checks = fresher information =
                    # a stronger spring stays stable. 200 suits this Pi.
                    # Hardware limits of this rig (Pi 5 + MCP2515 HAT):
                    # 500 = safe maximum (also set the motor's telemetry
                    # rate to 500 Hz in CubeMarsTool to match). ~1000 =
                    # absolute ceiling, where the CAN HAT starts dropping
                    # frames (check with: ip -s link show can0).

SIGN       = +1.0   # measured polarity of THIS unit (+A -> +rad).
                    # If the motor, wiring, or mounting changes, re-run
                    # the polarity probe (velocity sign under a small
                    # current ramp) — a wrong sign turns the spring into
                    # a repeller and the shaft oscillates at the target's
                    # antipode.

KT         = 0.5701 # [N*m/A] torque per amp at the output (9:1 gear).
# ===========================================================================

import math
import os
import time
import can
from epicallypowerful.actuation import ActuatorGroup

acts = ActuatorGroup.from_dict({CAN_ID: 'AK80-9-V3-servo'},
                               exit_manually=True)

def lifecycle(code):
    """This firmware needs FF..FC to wake and stay awake in
    the Servo dialect; FF..FD releases it."""
    acts.bus.send(can.Message(arbitration_id=CAN_ID,
                              data=[0xFF] * 7 + [code], is_extended_id=True))

def rel(a, b):                       # shortest signed angle a-b
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi

DT = 1.0 / LOOP_HZ

try:
    lifecycle(0xFC)
    # Event-driven start: wait for FRESH telemetry AFTER the wake frame
    # (a few new frames, ~30-50 ms total), so the firmware has processed
    # FF..FC before the first command lands. (A naive timestamp check
    # passes instantly on stale pre-wake frames — bench-learned bug.)
    ts0 = acts.get_data(CAN_ID).timestamp
    t0, fresh = time.perf_counter(), 0
    while fresh < 5 and time.perf_counter() - t0 < 1.0:
        ts = acts.get_data(CAN_ID).timestamp
        if ts > ts0:
            fresh, ts0 = fresh + 1, ts
        time.sleep(0.002)
    lifecycle(0xFC)                    # re-assert wake, belt and braces
    time.sleep(0.02)

    d = acts.get_data(CAN_ID)
    raw_prev, pos_c = d.current_position, 0.0     # unwrapped position
    print(f"Impedance running ({RUN_S:.0f}s, Ctrl+C stops): "
          f"pos={DES_POS} rad, vel={DES_VEL}, tau_ff={DES_TORQUE}, "
          f"kp={KP}, kd={KD}\n")

    t0, i = time.perf_counter(), 0
    while time.perf_counter() - t0 < RUN_S:
        d = acts.get_data(CAN_ID)
        pos_c += rel(d.current_position, raw_prev)   # incremental unwrap
        raw_prev = d.current_position

        tau = (DES_TORQUE + KP * (DES_POS - pos_c)
               + KD * (DES_VEL - d.current_velocity))
        acts.set_torque(CAN_ID, SIGN * tau / KT)     # amps, unclamped

        i += 1
        if i % 100 == 0:
            print(f"t={time.perf_counter() - t0:5.1f}s  "
                  f"target={math.degrees(DES_POS):+7.1f} deg  "
                  f"actual={math.degrees(pos_c):+7.1f} deg  "
                  f"tau={tau:+5.2f} N*m  cmd={SIGN * tau / KT:+5.2f} A  "
                  f"vel={d.current_velocity:+6.2f}  "
                  f"temp={getattr(d, 'temperature', float('nan')):4.1f} C")
        time.sleep(DT)

    print("\nTime limit reached — exiting.")

except KeyboardInterrupt:
    print("\nCtrl+C — stopping.")
finally:
    # Stop path: zero current, release the firmware. If the bus died
    # mid-run, recover can0 once and retry — then it's the E-stop's job.
    for attempt in (1, 2):
        try:
            for _ in range(10):
                acts.set_torque(CAN_ID, 0.0)
                time.sleep(0.005)
            lifecycle(0xFD)
            break
        except can.CanOperationError:
            os.system('sudo /sbin/ip link set can0 down')
            os.system('sudo /sbin/ip link set can0 txqueuelen 1000 up '
                      'type can bitrate 1000000')
            time.sleep(0.3)
    try:
        acts.disable_actuators()
        acts.notifier.stop()
        acts.bus.shutdown()
    except Exception:
        pass
    print("Done. Motor limp; verify by hand before approaching.")
