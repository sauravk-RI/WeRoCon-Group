# WeRoCon Lab — Wearable Robotics & Control

Open documentation, code and tutorials from the **Wearable Robotics & Control (WeRoCon) Lab** at
**IIT Jodhpur**. We build powered lower-limb wearable robots: an open-source prosthetic leg driven
by CubeMars AK80-9 actuators, and exoskeletons for gait rehabilitation.

This repository is the lab's shared knowledge base. Each folder documents one subsystem well enough
that a new student can rebuild it from scratch.

**Lab website:** [home.iitj.ac.in/~sauravk](https://home.iitj.ac.in/~sauravk) · **PI:** Saurav Kumar

---

## Repository map

| Folder | What you'll find |
|--------|------------------|
| [`IMU/`](IMU) | Thigh-angle and gait-phase estimation with MPU9250 and MicroStrain 3DM-GV7-AR IMUs: setup guides, Kalman filtering, calibration, live dashboard, sample data |
| [`Motor Control/`](Motor%20Control) | CAN-bus control of CubeMars AK80-9 actuators (MIT and servo modes), knee and ankle trajectory tracking, AS5048B joint encoders |
| [`Mechanical/`](Mechanical) | Mechanical design of the prosthetic leg (coming soon) |
| [`Raspberry_Pi_5_Basics/`](Raspberry_Pi_5_Basics) | Beginner tutorial series: OS install, SSH, VNC, Wi-Fi and everyday terminal commands |
| [`Research_Paper.md`](Research_Paper.md) | Work-in-progress write-up of the AK80-9 trajectory-tracking results |

## Where to start

1. **New to Raspberry Pi?** Work through [Raspberry Pi 5 Basics](Raspberry_Pi_5_Basics/00_Contents.md) in order.
2. **Working on sensing?** Follow [MPU9250 setup](IMU/Documentation/MPU9250_setup.md) or [MicroStrain setup](IMU/Documentation/3DM-GV7-AR-Microstrain_setup.md).
3. **Working on actuation?** Start with [CAN setup](Motor%20Control/Documentation/CAN_Setup.md), then [AK80-9 control modes](Motor%20Control/Documentation/CubeMars_AK80_9control%20modes.md).

## Hardware used

Raspberry Pi 5 · CubeMars AK80-9 V3.0 actuators (CAN) · AS5048B 14-bit magnetic encoders (I²C) ·
MPU9250 and MicroStrain 3DM-GV7-AR IMUs

## Contributing

Lab members and outside contributors are welcome. Read [CONTRIBUTING.md](CONTRIBUTING.md) first;
if you've never opened a pull request, the
[step-by-step guide for new GitHub users](Readme_How_To_Do_Open_Source_Contribution_To_This_Repository_For_New_Github_Users.md)
walks you through it.

## License

Released under the [MIT License](LICENSE).
