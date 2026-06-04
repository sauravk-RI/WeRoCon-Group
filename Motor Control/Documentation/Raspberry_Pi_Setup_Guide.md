# Raspberry Pi Setup Guide

This document outlines the procedure for setting up the Raspberry Pi to control the prosthetic leg. Since the prosthetic leg is a wearable and headless device (i.e., it operates without a dedicated monitor, keyboard, or mouse), the Raspberry Pi must be configured for wireless remote access.

## Objectives

- Install Raspberry Pi OS on the MicroSD card
- Configure Wi‑Fi and SSH before first boot
- Connect remotely using SSH (PuTTY)
- Enable and use VNC for graphical access when required
- Prepare the Raspberry Pi for CAN communication and motor control development

---

## Hardware Requirements

| Component | Description |
|------------|------------|
| Raspberry Pi | Raspberry Pi 4 or Raspberry Pi 5 |
| MicroSD Card | 32 GB or larger |
| SD Card Adapter | USB or Full-size SD adapter |
| Host Computer | Windows/Linux/macOS system connected to the same Wi‑Fi network |
| Power Supply | Official Raspberry Pi power adapter or regulated battery source |

---

# 1. Flashing Raspberry Pi OS

The Raspberry Pi OS image is written to the MicroSD card using Raspberry Pi Imager v2.0.7.

## Procedure

1. Insert the MicroSD card into the host computer using an SD card adapter.
2. Open Raspberry Pi Imager v2.0.7.
3. Allow administrator permissions if prompted.
4. Select:
   - Device: Your Raspberry Pi model
   - OS: Raspberry Pi OS (64-bit)
   - Storage: Your MicroSD card
5. Configure hostname, username, password, Wi‑Fi, and enable SSH.
6. Click Write and wait for verification.
7. Safely eject the MicroSD card.

---

# 2. First Boot

1. Insert the MicroSD card into the Raspberry Pi.
2. Connect power.
3. Wait 2–3 minutes for initialization.

---

# 3. Remote Access Using SSH

SSH is the primary interface for development and debugging.

## Windows (PuTTY)

- Hostname: `prosthetic-leg.local`
- Port: `22`
- Connection Type: `SSH`

## Linux/macOS

```bash
ssh <username>@prosthetic-leg.local
```

---

# 4. Graphical Access Using VNC

Enable VNC:

```bash
sudo raspi-config nonint do_vnc 0
```

If using Bookworm and VNC shows a black screen:

```bash
sudo raspi-config nonint do_wayland W1
sudo reboot
```

Connect using RealVNC Viewer and enter:

```text
prosthetic-leg.local
```

---

# Remote Access Summary

| Tool | Purpose |
|--------|---------|
| SSH (PuTTY) | Command-line access |
| VNC Viewer | Remote desktop access |
| Raspberry Pi Imager | OS installation |

---


