#!/usr/bin/env python3
"""current_loop_demo.py — Ch 6's Current Loop from Python (Servo mode, Control Mode ID 1)."""
import time
import can
from epicallypowerful.actuation import ActuatorGroup
from epicallypowerful.toolbox import TimedLoop

CAN_ID = 100
AMPS   = 0.6                  # ≈ 0.34 N·m at the output (× 0.5701) — small on purpose
RUN_S  = 6                  # a short run of 6s: the shaft is ACCELERATING the whole time

actuators = ActuatorGroup.from_dict({CAN_ID: 'AK80-9-V3-servo'})

def lifecycle(code):
    actuators.bus.send(can.Message(arbitration_id=CAN_ID,
                                   data=[0xFF] * 7 + [code],
                                   is_extended_id=True))

lifecycle(0xFC)
time.sleep(0.5)
print(f"Current-loop demo: {AMPS:.1f} A for {RUN_S:.1f}s — expect steady acceleration.\n")

clock = TimedLoop(rate=100)
t0 = time.perf_counter()
i = 0
try:
    while clock():
        t = time.perf_counter() - t0
        if t > RUN_S:
            break
        actuators.set_torque(CAN_ID, AMPS)             # amperes, not N·m!
        if i % 10 == 0:
            print(f"t={t:4.2f}s  i_cmd={AMPS:.2f} A  "
                  f"vel={actuators.get_velocity(CAN_ID):+6.2f} rad/s   <- climbing!")
        i += 1
finally:
    actuators.set_torque(CAN_ID, 0.0)                  # push off → shaft coasts, then limp
    lifecycle(0xFD)

print("Done.")
