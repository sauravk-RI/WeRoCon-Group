#  CAN Motor Control: MIT Mode & Servo Mode

This document outlines the software implementations for controlling the CubeMars AK80-9 V3.0 actuator using CAN communication.

To meet the dynamic demands of a prosthetic joint, the project implements three distinct software solutions:
1. **MIT Mode Control (Custom Low-Level Loop):** Implements a direct, low-latency joint impedance loop (`aktest.py`).
2. **Servo Mode Control (OpenSourceLeg API):** Uses the bionics-focused **`opensourceleg`** library for high-level Velocity, Position, and Current control.
3. **Dynamic Zero Calibration:** A raw CAN script to dynamically establish a relative joint "zero point" on boot.

---

## 1. MIT Mode Control (Low-Level Custom Loop)

MIT Mode is the standard framework for active joint bionics. It bypasses the actuator's internal PID loops, allowing the Raspberry Pi to send high-frequency ($100\text{ Hz}$) control frames to dynamically modulate joint stiffness ($K_p$) and damping ($K_d$).

### Custom MIT Script (`aktest.py`)
This script executes your custom control loop, converts floating-point physical units (rad, rad/s, N·m) into raw unsigned integers, and parses the motor's real-time feedback.

```python
import can
import time

# AK80-9 parameter ranges
P_MIN, P_MAX   = -12.56, 12.56
V_MIN, V_MAX   = -65.0,  65.0
T_MIN, T_MAX   = -18.0,  18.0
KP_MIN, KP_MAX =  0.0,   500.0
KD_MIN, KD_MAX =  0.0,   5.0

MOTOR_ID   = 100
MIT_CMD_ID = 8

def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    return int((x - x_min) * ((1 << bits) / (x_max - x_min)))

def pack_mit(p, v, kp, kd, t):
    p_int  = float_to_uint(p,  P_MIN,  P_MAX,  16)
    v_int  = float_to_uint(v,  V_MIN,  V_MAX,  12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int  = float_to_uint(t,  T_MIN,  T_MAX,  12)

    buf = [0]*8
    buf[0] = kp_int >> 4
    buf[1] = ((kp_int & 0xF) << 4) | (kd_int >> 8)
    buf[2] = kd_int & 0xFF
    buf[3] = p_int >> 8
    buf[4] = p_int & 0xFF
    buf[5] = v_int >> 4
    buf[6] = ((v_int & 0xF) << 4) | (t_int >> 8)
    buf[7] = t_int & 0xFF
    return buf

def parse_feedback(msg):
    d = msg.data
    pos_raw = (d[0] << 8) | d[1]
    spd_raw = (d[2] << 8) | d[3]
    cur_raw = (d[4] << 8) | d[5]
    if pos_raw > 32767: pos_raw -= 65536
    if spd_raw > 32767: spd_raw -= 65536
    if cur_raw > 32767: cur_raw -= 65536
    pos  = pos_raw * 0.1
    spd  = spd_raw * 10.0
    cur  = cur_raw * 0.01
    temp = d[6]
    err  = d[7]
    return pos, spd, cur, temp, err

def send_mit(bus, p, v, kp, kd, t):
    data   = pack_mit(p, v, kp, kd, t)
    can_id = (MIT_CMD_ID << 8) | MOTOR_ID
    msg    = can.Message(arbitration_id=can_id,
                         data=data,
                         is_extended_id=True)
    bus.send(msg)

# --- MAIN ---
bus = can.interface.Bus(channel='can0', bustype='socketcan')

print("MIT continuous mode running...")
print("Press Ctrl+C to stop")

# Control parameters - change these to experiment
P_DES = 0.0    # target position (rad)
V_DES = 0.0    # target velocity (rad/s)
KP    = 10.0   # stiffness
KD    = 1.0    # damping
T_FF  = 0.0    # feedforward torque (N·m)

try:
    while True:
        # Send command
        send_mit(bus, p=P_DES, v=V_DES, kp=KP, kd=KD, t=T_FF)

        # Read feedback
        msg = bus.recv(timeout=0.01)
        if msg and (msg.arbitration_id & 0xFF) == MOTOR_ID:
            pos, spd, cur, temp, err = parse_feedback(msg)
            print(f"pos={pos:.1f}° spd={spd:.0f}ERPM cur={cur:.2f}A temp={temp}°C err={err}")

            # Safety cutoff
            if err != 0:
                print(f"ERROR DETECTED: {err} — stopping!")
                break
            if temp > 70:
                print(f"TEMPERATURE TOO HIGH: {temp}°C — stopping!")
                break

        time.sleep(0.01)  # 100Hz loop

except KeyboardInterrupt:
    print("\nStopped by user")

finally:
    # Send zero torque before closing
    send_mit(bus, p=0.0, v=0.0, kp=0.0, kd=0.0, t=0.0)
    bus.shutdown()
    print("Bus closed safely")
```

---

## 2. Servo Mode Control (Using OpenSourceLeg API)

For high-level joint commands, we transition to Servo Mode using the **`opensourceleg`** (OSL) library. This library provides a professional, unified interface designed specifically for active bionic limbs.

### A. Velocity Control Implementation
The script below demonstrates how to configure and execute velocity control (e.g., maintaining a continuous rotational speed of $1.0\text{ rad/s}$):

```python
from opensourceleg.actuators.tmotor import TMotorServoActuator
import time

# Initialize the OSL actuator object
motor = TMotorServoActuator(
    motor_type="AK80-9",
    motor_id=100
)

# Start communication and enable the motor
motor.start()

# Transition the actuator to Velocity Control Mode
motor.set_control_mode(type(motor.mode).VELOCITY)

try:
    while True:
        motor.update()
        motor.set_motor_velocity(1.0)   # Command velocity: 1.0 rad/s

        print(
            f"pos={motor.motor_position:.3f}  "
            f"vel={motor.motor_velocity:.3f}"
        )

        time.sleep(0.05)

except KeyboardInterrupt:
    # Ensure the motor stops receiving active current commands on exit
    motor.stop()
```

### B. Position and Current (Torque) Control
By using the same `TMotorServoActuator` framework, we can easily change modes to command other parameters:

* **Position Control:**
  ```python
  motor.set_control_mode(type(motor.mode).POSITION)
  # Inside loop:
  motor.set_motor_position(1.57) # Command joint to 90 degrees (1.57 radians)
  ```
* **Current (Torque) Control:**
  ```python
  motor.set_control_mode(type(motor.mode).CURRENT)
  # Inside loop:
  motor.set_motor_current(2.5) # Command output drive current to 2.5 Amps
  ```

---

## 3. Relative Position Calibration (Zero Offset Tracking)

Because absolute encoders read the raw magnetic angle of the motor's rotor, the raw position value on boot does not correspond to the human user's leg angle. 

To establish an anatomical straight-leg reference point, the script below reads the actuator's physical orientation on startup, sets it as the software **ZERO offset**, and calculates all subsequent joint movements relative to that initial position.

```python
import can
import time

MOTOR_ID = 100

def parse_feedback(msg):
    """
    Helper function to parse only the position degrees from the feedback frame.
    """
    d = msg.data
    pos_raw = (d[0] << 8) | d[1]

    if pos_raw > 32767:
        pos_raw -= 65536

    pos_deg = pos_raw * 0.1
    return pos_deg

bus = can.interface.Bus(
    channel='can0',
    interface='socketcan'
)

print("Reading current position...")

zero_offset = None

# Step 1: Wait to receive the first raw feedback packet on boot to set the zero-offset
while zero_offset is None:
    msg = bus.recv(timeout=1)

    if msg and (msg.arbitration_id & 0xFF) == MOTOR_ID:
        zero_offset = parse_feedback(msg)

print(f"\nCurrent position set as ZERO")
print(f"Zero offset = {zero_offset:.2f}°\n")

# Step 2: Read continuous raw values and calculate the relative joint angle
try:
    while True:
        msg = bus.recv(timeout=1)

        if msg and (msg.arbitration_id & 0xFF) == MOTOR_ID:
            actual_pos = parse_feedback(msg)
            
            # Relative position is the difference between actual and startup offset
            relative_pos = actual_pos - zero_offset

            print(
                f"Actual = {actual_pos:8.2f}°   "
                f"Relative = {relative_pos:8.2f}°"
            )

except KeyboardInterrupt:
    print("\nStopped")

finally:
    bus.shutdown()
```
