# Setup and Testing of CubeMars AK Actuators

## Model Used
**CubeMars AK80-9 V3.0**

## Objective
To configure, calibrate, and test the CubeMars AK80-9 V3.0 motor using the CubeMars AK Config Tool software (via the R-Link module) to verify its operation in both Servo Mode and MIT Mode.

---

# Introduction

The CubeMars AK80-9 V3.0 is a high-performance, highly integrated robotic actuator designed for dynamic robotic applications such as quadrupeds, exoskeletons, and industrial manipulators.

## Key Features

- High-torque BLDC motor
- 9:1 planetary gearbox
- Integrated motor controller
- Dual encoder feedback
- Improved thermal dissipation
- Enhanced mechanical robustness
- Support for Servo Mode and MIT Mode

---

# Technical Specifications

| Parameter | Specification |
|------------|-------------|
| Driver Model | AK80-9 V3.0 |
| Input Voltage | 48 V |
| Working Voltage Range | 18–52 V |
| Rated Output Current (RMS) | 20 A |
| Peak Output Current | 60 A |
| Standby Power Consumption | ≤ 1 W |
| CAN Bus Bitrate | 1 Mbps |
| Driver Dimensions | 63 mm × 57 mm |
| Ambient Temperature Range | -20 °C to 65 °C |
| Maximum Driver Temperature | 100 °C |
| Inner Encoder Resolution | 21-bit Single-turn Absolute Encoder |
| Outer Encoder Resolution | Optional 15-bit Single-turn Absolute Encoder |
| Communication Interface | CAN + UART |

# R-Link Overview

| Parameter | Specification |
|------------|-------------|
| Rated Voltage | 5 V |
| VCC Selection | 3.3 V (Default) |
| Standby Current | ≤ 30 mA |
| Dimensions | 73.8 × 23.6 × 14.5 mm |
| Operating Temperature | -20 °C to 65 °C |
| Maximum Board Temperature | 85 °C |

---

# Initial Hardware Setup & Communication

Before attempting to calibrate the driver board or run any operating modes, complete the physical connections and establish communication with the PC software.

### 1. Hardware Setup
1. Connect the AK80-9 actuator to the driver board.
2. Connect the R-Link module to the communication port on the driver.
3. Connect the R-Link USB interface to the host computer.
4. Connect the power supply (18–52 V).
5. Launch the **CubeMars AK Config Tool**.

<img width="1600" height="900" alt="WhatsApp Image 2026-06-04 at 12 52 15" src="https://github.com/user-attachments/assets/c280e381-3587-457d-95d7-dc8bd6ea08c4" />



### 2. Establish Communication
1. Navigate to the **Connection** panel in the software.
2. Select the appropriate COM port.
3. Set the baud rate to **921600**.
4. Click **Connect**.
5. Verify that real-time motor parameters are actively updating in the dashboard.

---

# Driver Board Calibration

When you have reinstalled the driver board on the motor, changed the wiring sequence of the motor's three-phase lines, or updated the firmware, recalibration is necessary (it was calibrated when it left the factory for the first time, so there is no need to calibrate it again out-of-the-box). After calibration, the motor can be used.

### Calibration Steps

* **STEP 0:** Ensure that the motor's power supply is stable, the connectors are properly connected, and successfully connected to the upper computer, then enter the system settings page.
* **STEP 1:** Click **Read** and wait for the connection interface to display.
* **STEP 2:** Click **Motor Identification**. After a brief beep, the motor will start rotating. Wait for about 10 seconds before the motor stops rotating. When the connection status message appears, it indicates that the motor parameter identification process is complete.
* **STEP 3:** Click **Encoder Identification**. The motor will rotate slowly. Wait for about 45 seconds. When the connection interface display shows completion, the encoder parameter identification is finished.
* **STEP 4:** Click **Write**. When the connection interface display is complete, the calibration is saved and finished.

> ⚠️ **Warning:** The entire process of motor parameter and encoder parameter identification must be kept under no-load conditions; otherwise, it may lead to inaccurate identification parameters or motor damage. The encoder parameter identification process generates heat, and performing this process consecutively multiple times can cause the motor temperature to rise sharply.

---

# Servo Mode

Servo Mode offloads position and velocity loops to the actuator's internal driver board. This is useful for testing specific joint configurations or calibrating absolute joint angle limits.

| Control Mode | Description | Governing Equation | Controlled Parameter |
|-------------|-------------|-------------------|---------------------|
| Duty Cycle Mode | Controls average motor voltage through PWM | `V_avg = D × V_supply` | Voltage |
| Current Loop Mode | Regulates motor current | `T = Kt × I` | Current / Torque |
| Current Brake Mode | Generates braking torque | `T_brake = Kt × I_brake` | Braking Torque |
| Velocity Loop Mode | Controls motor speed | `e_v = ω_ref - ω_actual` | Velocity |
| Position Loop Mode | Controls shaft position | `e_p = θ_ref - θ_actual` | Position |
| Position-Velocity Loop Mode | Position control with velocity and acceleration constraints | `|ω| ≤ ω_max`, `|α| ≤ α_max` | Position, Velocity, Acceleration |

## Servo Mode Configuration Procedure

### 1. Configure Servo Control
1. Open the **Servo Control** tab.
2. Select the desired control mode.
3. Configure the required command parameters.
4. Click the **Run** button to execute commands.
5. Monitor motor response through the real-time status window.

### 2. Position-Velocity Control
1. Enter the desired position value.
2. Set the velocity limit.
3. Set the acceleration limit.
4. Execute the command.
5. Verify that the motor reaches the target position smoothly.

### 3. Multi-Turn Position Mode
1. Select **Multi Mode**.
2. Enter the desired position command within the range **-36000° to +36000°**.
3. Execute the command.
4. Observe motor position tracking over multiple revolutions.

### 4. Single-Turn Position Mode
1. Select **Single Mode**.
2. Enter the desired position command within the range **0° to 359°**.
3. Execute the command.
4. Verify accurate position tracking within a single revolution.

### 5. Origin Configuration
#### Set Origin
1. Move the actuator to the desired reference position.
2. Click **Set Origin**.
3. The current rotor position is stored as the new zero position.

#### Back Origin
1. Click **Back Origin**.
2. The actuator automatically returns to the stored zero position.

> **Warning:** The motor may move at high speed while returning to the origin position. Ensure the workspace is clear before executing this command.

<img width="1920" height="1080" alt="Screenshot (104)" src="https://github.com/user-attachments/assets/98f2d4a5-1214-446d-a355-d6a3774e2790" />


---

# MIT Mode

MIT Mode combines position, velocity, and torque control within a single control law.

## MIT Control Law

```text
T = Kp(θ_ref − θ_actual)+ Kd(ω_ref − ω_actual)+ T_ff
```

## MIT Mode Configuration Procedure

### 1. Open MIT Control Interface
1. Navigate to the **MIT Control** tab.
2. Confirm that communication with the actuator is active.

### 2. Configure MIT Parameters
Set the required control parameters:
* Position (P)
* Velocity (S)
* Feedforward Torque (T)
* Position Gain (KP)
* Velocity Gain (KD)
* Motor ID

### 3. Position Control Test
1. Enter the desired position command.
2. Configure suitable KP and KD values.
3. Execute the command.
4. Observe actuator position tracking performance.

### 4. Stiffness Tuning (KP)
1. Begin with a low KP value.
2. Gradually increase KP.
3. Observe changes in position holding capability and response stiffness.
4. Record system behavior at different gain values.

### 5. Damping Tuning (KD)
1. Keep KP constant.
2. Incrementally increase KD.
3. Observe the reduction in oscillations and overshoot.
4. Evaluate system stability.

### 6. Velocity Control Test
1. Set the desired velocity.
2. Set KP to zero.
3. Configure an appropriate KD value.
4. Execute the command and verify velocity tracking.

### 7. Torque Control Test
1. Set KP and KD to zero.
2. Enter the desired feedforward torque value.
3. Execute the command.
4. Observe the generated output torque.

### 8. Real-Time Monitoring
Monitor the following parameters during testing:
* Position
* Velocity
* Torque
* Phase Current
* Bus Voltage
* Motor Temperature
* MOSFET Temperature

### 9. Emergency Stop
1. Press the **STOP** button if abnormal behavior is observed.
2. Disconnect power if required.
3. Verify parameter values before restarting the actuator.

<img width="1920" height="1080" alt="Screenshot (103)" src="https://github.com/user-attachments/assets/e434aeed-4a62-43c4-bbd5-1245a6fc766f" />


---

# Firmware Update

1. Select the corresponding firmware from the drop-down list.
2. Click **Jump to IAP**.
3. Click **Upload** and wait for the upgrade progress bar to reach 100%.
4. Click **Jump to App** and wait 5 seconds for the motor to enter its operational mode.

---

## Results

- Successfully established communication between the AK80-9 actuator and the host system using the R-Link interface.
- Verified the functionality of Servo Mode and MIT Mode control frameworks.
- Successfully tested position, velocity, current, and torque control operations.
- Real-time monitoring of actuator parameters was achieved through the CubeMars AK Config Tool.
- Stable actuator performance was observed during all configuration and testing procedures.

---
