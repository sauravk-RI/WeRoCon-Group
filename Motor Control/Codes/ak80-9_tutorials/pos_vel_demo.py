#!/usr/bin/env python3
"""pos_vel_demo.py — Ch 10's Position–Velocity Mode from Python (Servo mode, Control Mode ID 6)."""
import time
import can
from epicallypowerful.actuation import ActuatorGroup
from epicallypowerful.toolbox import TimedLoop
from epicallypowerful.actuation.cubemars.cubemars_servo import make_position_velocity_mode_message

CAN_ID    = 100
TRAVEL_DEG = 270.0           
SPEED_ERPM = 4000             # ≈ 21 rpm ≈ 2.2 rad/s at the shaft (÷189, Ch 3 §2)
ACC_ERPM_S = 8000             # gentle ramps (≈ 42 shaft rpm gained per second)
LEG_S      = 3.0              # time budget per leg of the trip

actuators = ActuatorGroup.from_dict({CAN_ID: 'AK80-9-V3-servo'})

def lifecycle(code):
    actuators.bus.send(can.Message(arbitration_id=CAN_ID,
                                   data=[0xFF] * 7 + [code],
                                   is_extended_id=True))

def send_pos_vel(pos_deg, speed_erpm, acc_erpm_s2):
    """Engineering units in; wire counts (1 count = 10 ERPM) out. Ch 10 §5."""
    actuators.bus.send(make_position_velocity_mode_message(
        CAN_ID, pos_deg, int(speed_erpm / 10), int(acc_erpm_s2 / 10)))

lifecycle(0xFC)
time.sleep(0.5)

home = actuators.get_position(CAN_ID, degrees=True)    # absolute targets: ask first
target = home + TRAVEL_DEG
print(f"Pos–Vel demo: {home:+.1f}° -> {target:+.1f}° at {SPEED_ERPM} ERPM, then back.\n")

clock = TimedLoop(rate=50)
t0 = time.perf_counter()
i = 0
try:
    while clock():
        t = time.perf_counter() - t0
        if t > 2 * LEG_S:
            break
        goal = target if t < LEG_S else home           # out, then back
        send_pos_vel(goal, SPEED_ERPM, ACC_ERPM_S)     # speed = positive cruise magnitude
        if i % 25 == 0:
            print(f"t={t:4.1f}s  goal={goal:+7.1f}°  "
                  f"pos={actuators.get_position(CAN_ID, degrees=True):+7.1f}°  "
                  f"vel={actuators.get_velocity(CAN_ID):+5.2f} rad/s")
        i += 1
finally:
    actuators.set_torque(CAN_ID, 0.0)
    lifecycle(0xFD)

print("Done.")
