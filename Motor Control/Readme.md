# Motor Control

CAN-bus control of CubeMars AK80-9 actuators on a Raspberry Pi, used to drive the knee and ankle
joints of the prosthetic leg along human gait trajectories from the Winter gait dataset.

## Guides (suggested reading order)

| # | Guide | What it covers |
|---|-------|----------------|
| 1 | [Raspberry Pi setup](Documentation/Raspberry_Pi_Setup_Guide.md) | Headless setup of the Pi that controls the leg |
| 2 | [CAN setup](Documentation/CAN_Setup.md) | Wiring, drivers, libraries and a loopback check |
| 3 | [AK80-9 control modes](Documentation/CubeMars_AK80_9control%20modes.md) | Setting up and testing the actuator |
| 4 | [CAN motor control](Documentation/CAN_Motor_Control.md) | MIT-mode and servo-mode implementations |
| 5 | [AS5048B joint encoder](Documentation/AS5048A_Absolute_Magnetic_Encoder.md) | Measuring the true joint angle over I²C |
| 6 | [Knee trajectory tracking](Documentation/Knee_Trajectory_Tracking.md) | Single-joint knee gait tracking |
| 7 | [Ankle trajectory tracking](Documentation/ankle.md) | Single-joint ankle gait tracking |
| 8 | [Simultaneous knee + ankle](Documentation/Knee_Ankle_Simultaenous_trajectory.md) | Two actuators on one CAN bus |

The scripts are embedded in each guide.

