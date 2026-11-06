#!/usr/bin/env python3
"""duty_cycle_demo.py — Ch 5's Duty Cycle Mode from Python (Servo mode, Control Mode ID 0)."""
import time
import can
from epicallypowerful.actuation import ActuatorGroup
from epicallypowerful.toolbox import TimedLoop
from epicallypowerful.actuation.cubemars.cubemars_servo import make_duty_cycle_message

CAN_ID = 100
DUTY   = 0.04     # 4% of bus voltage — gentle, visible spin
RUN_S  = 4.0      # 2 s forward + 2 s reverse # Total time for which we are gonna operate our motor

actuators = ActuatorGroup.from_dict({CAN_ID: 'AK80-9-V3-servo'})

def lifecycle(code):
    actuators.bus.send(can.Message(arbitration_id=CAN_ID,
                                   data=[0xFF] * 7 + [code],
                                   is_extended_id=True))

lifecycle(0xFC)
time.sleep(0.5)
print(f"Duty-cycle demo: ±{DUTY:.0%} for {RUN_S:.0f}s. Ctrl+C stops.\n")

clock = TimedLoop(rate=50)     # refresh the command 50×/s  # control frequency
t0 = time.perf_counter()  # recording the exact starting time
i = 0
try:

    # Note: This complete while block is gonna run 50 times every second.  
    while clock():                          # while the clock is running
        t = time.perf_counter() - t0
        if t > RUN_S:
            break
        duty = DUTY if t < RUN_S / 2 else -DUTY        # flip halfway
        actuators.bus.send(make_duty_cycle_message(CAN_ID, duty))
        if i % 25 == 0:               # print twice a second
            print(f"t={t:4.1f}s  duty={duty:+.2f}  "
                  f"vel={actuators.get_velocity(CAN_ID):+5.2f} rad/s  "
                  f"|i|={abs(actuators.get_torque(CAN_ID)):4.2f} A")
        i += 1
        
finally:
    actuators.bus.send(make_duty_cycle_message(CAN_ID, 0.0))  # drive off
    time.sleep(0.3)
    actuators.set_torque(CAN_ID, 0.0)                          # 0 A → limp
    lifecycle(0xFD)

print("Done.")
