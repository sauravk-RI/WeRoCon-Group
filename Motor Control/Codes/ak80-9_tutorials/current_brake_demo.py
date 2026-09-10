#!/usr/bin/env python3
"""current_brake_demo.py — Ch 7's Current Brake from Python (Servo mode, Control Mode ID 2)."""
import time
import can
from epicallypowerful.actuation import ActuatorGroup
from epicallypowerful.toolbox import TimedLoop
from epicallypowerful.actuation.cubemars.cubemars_servo import make_current_brake_message

CAN_ID  = 100
BRAKE_A = 20                 # holding current, amperes (0–60 A range; stay tiny)
RUN_S   = 6.0

actuators = ActuatorGroup.from_dict({CAN_ID: 'AK80-9-V3-servo'})

def lifecycle(code):
    actuators.bus.send(can.Message(arbitration_id=CAN_ID,
                                   data=[0xFF] * 7 + [code],
                                   is_extended_id=True))

lifecycle(0xFC)
time.sleep(0.5)
print(f"Brake demo: {BRAKE_A:.1f} A hold for {RUN_S:.0f}s — "
      "gently try turning the flag by hand, then let go.\n")

clock = TimedLoop(rate=50)
t0 = time.perf_counter()
i = 0
try:
    while clock():
        t = time.perf_counter() - t0
        if t > RUN_S:
            break
        actuators.bus.send(make_current_brake_message(CAN_ID, BRAKE_A))
        if i % 25 == 0:
            print(f"t={t:4.1f}s  pos={actuators.get_position(CAN_ID):+6.2f} rad  "
                  f"temp={actuators.get_temperature(CAN_ID):3.0f} °C")
        i += 1
finally:
    actuators.set_torque(CAN_ID, 0.0)                  # brake released → limp
    lifecycle(0xFD)

print("Done.")
