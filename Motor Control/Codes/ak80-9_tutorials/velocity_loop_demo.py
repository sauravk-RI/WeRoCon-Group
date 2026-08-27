#!/usr/bin/env python3
"""velocity_loop_demo.py — Ch 8's Velocity Loop from Python (Servo mode, Control Mode ID 3)."""
import time
import can
from epicallypowerful.actuation import ActuatorGroup
from epicallypowerful.toolbox import TimedLoop

CAN_ID = 100
SPEED  = 2.0                  # rad/s at the output shaft (≈ 19 rpm)
RUN_S  = 10.0                  # 5s forward + 5s reverse

actuators = ActuatorGroup.from_dict({CAN_ID: 'AK80-9-V3-servo'})

def lifecycle(code):
    actuators.bus.send(can.Message(arbitration_id=CAN_ID,
                                   data=[0xFF] * 7 + [code],
                                   is_extended_id=True))

lifecycle(0xFC)
time.sleep(0.5)
print(f"Velocity demo: ±{SPEED:.0f} rad/s cruise. Ctrl+C stops.\n")

clock = TimedLoop(rate=100)
t0 = time.perf_counter()
i = 0
try:
    while clock():
        t = time.perf_counter() - t0
        if t > RUN_S:
            break
        v = SPEED if t < RUN_S / 2 else -SPEED
        actuators.set_velocity(CAN_ID, v, 0)           # third arg unused in Servo — pass 0
        if i % 10 == 0:
            print(f"t={t:4.1f}s  v_cmd={v:+4.1f}  "
                  f"vel={actuators.get_velocity(CAN_ID):+5.2f} rad/s  "
                  f"|i|={abs(actuators.get_torque(CAN_ID)):4.2f} A")
        i += 1
finally:
    actuators.set_velocity(CAN_ID, 0.0, 0)             # controlled stop first...
    time.sleep(0.3)
    actuators.set_torque(CAN_ID, 0.0)                  # ...then limp
    lifecycle(0xFD)

print("Done.")
