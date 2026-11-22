#!/usr/bin/env python3
"""position_loop_demo.py — Ch 9's Position Loop from Python (Servo mode, Control Mode ID 4)."""
import time
import can
from epicallypowerful.actuation import ActuatorGroup
from epicallypowerful.toolbox import TimedLoop

CAN_ID  = 1
STEP_DEG = 0.0               # a NEARBY target 
HOLD_S   = 60.0                # hold the position for 60s

actuators = ActuatorGroup.from_dict({CAN_ID: 'AK80-9-V3-servo'})

def lifecycle(code):
    actuators.bus.send(can.Message(arbitration_id=CAN_ID,
                                   data=[0xFF] * 7 + [code],
                                   is_extended_id=True))

lifecycle(0xFC)
time.sleep(0.5)

home = actuators.get_position(CAN_ID, degrees=True)    # WHERE IS THE SHAFT? Ask first!
target = home + STEP_DEG
print(f"Position demo: {home:+.1f}° -> {target:+.1f}° -> back. Small on purpose.\n")

clock = TimedLoop(rate=50)
t0 = time.perf_counter()
i = 0
try:
    while clock():
        t = time.perf_counter() - t0
        if t > 2 * HOLD_S:
            break
        goal = target if t < HOLD_S else home          # out, then back
        actuators.set_position(CAN_ID, goal, 0, 0, degrees=True)   # gains unused in Servo — pass 0, 0
        if i % 25 == 0:                                            
            print(f"t={t:4.1f}s  goal={goal:+7.1f}°  "
                  f"pos={actuators.get_position(CAN_ID, degrees=True):+7.1f}°")
        i += 1
finally:
    actuators.set_torque(CAN_ID, 0.0)                  # release the hold → limp
    lifecycle(0xFD)

print("Done.")
