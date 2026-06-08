# Raspberry Pi CAN Setup & Communication Guide

This document covers the physical connections, driver installation, software library setup, and the loopback validation check required to establish a CAN communication network on your Raspberry Pi.

---

## 1. Hardware Requirements & Setup

To establish a CAN interface on your Raspberry Pi, configure the hardware connections as follows:

* **Raspberry Pi** with a **CAN HAT** (e.g., Waveshare RS485/CAN HAT) mounted securely on the GPIO pins.
* **Actuator Connection:** Connections from the motor's **XT30 2+2 connector**:
  * **CAN_H** on the HAT $\rightarrow$ **CAN_H** (White wire)
  * **CAN_L** on the HAT $\rightarrow$ **CAN_L** (Blue wire)
  * **Power Connection:** Battery power connected to the motor via your XT60-to-XT30 power lines.

---

## 2. Driver & Library Configuration

### Step A: Enable the MCP2515 Driver
We must enable the Serial Peripheral Interface (SPI) bus and define the overlay parameters for your CAN controller chip.

1. Open your boot configuration file:
   * *If using Raspberry Pi OS Bookworm:*
     ```bash
     sudo nano /boot/firmware/config.txt
     ```
   * *If using an older OS version (Bullseye or earlier):*
     ```bash
     sudo nano /boot/config.txt
     ```

2. Scroll to the bottom of the file and add the following lines:
   ```text
   dtparam=spi=on
   dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
   ```
   > ⚠️ **Note on Oscillator frequency:** Check the physical metal crystal oscillator on your CAN HAT. If the crystal displays **12.000**, use `12000000`. If it displays **8.000**, change the parameter to `oscillator=8000000`.

3. Save the file (`Ctrl + O`, then `Enter`) and exit (`Ctrl + X`).
4. Reboot the Raspberry Pi:
   ```bash
   sudo reboot
   ```

### Step B: Automate SocketCAN on Boot
To ensure the `can0` network interface starts automatically at **1 Mbps** on every boot:

1. Create a network interface configuration file:
   ```bash
   sudo nano /etc/network/interfaces.d/can0
   ```
2. Paste the following configuration:
   ```text
   auto can0
   iface can0 can static
       bitrate 1000000
       up /sbin/ip link set $IFACE txqueuelen 1000
   ```
3. Save the file and exit.

---

## 3. Installation of Utilities & Libraries

To interact with the CAN interface via command-line utilities and write custom code, install the required packages and libraries:

1. Install **can-utils** (for system-level sending and dumping):
   ```bash
   sudo apt update && sudo apt install can-utils -y
   ```
2. Install **python-can** (the Python library for SocketCAN):
   ```bash
   sudo apt install python3-can -y
   ```

---

## 4. Setting the CAN ID on the Motor

For the Raspberry Pi to address your motor specifically, the motor's CAN ID must be configured.

1. Connect your motor to your PC using the R-Link module.
2. Open the **CubeMars AK Config Tool**.
3. Under the **Application Configuration** section, set the **CAN ID** (e.g., `104` or `0x68` in hex).
4. Click **Write** to save this setting to the motor's flash memory.

<img width="1600" height="720" alt="CAN_id_Setup" src="https://github.com/user-attachments/assets/f564cdce-cee9-4b54-bd9c-5b51421269d5" />

---

## 5. Loopback Communication Check (Without Motor)

Before powering on the motor, verify that the Raspberry Pi can transmit and receive packets locally using **Loopback Mode**. This isolates the Raspberry Pi's transceiver from the external bus to verify the internal software/hardware path is functioning.

1. Shut down the interface:
   ```bash
   sudo ip link set can0 down
   ```
2. Configure the interface to **loopback on**:
   ```bash
   sudo ip link set can0 type can bitrate 1000000 loopback on
   ```
3. Bring the interface up:
   ```bash
   sudo ip link set can0 up
   ```
4. Open a second PuTTY terminal window.
   * In **Terminal 1**, run `candump` to listen:
     ```bash
     candump can0
     ```
     *(This terminal will wait silently for incoming packets).*
   * In **Terminal 2**, send a test packet:
     ```bash
     cansend can0 123#DEADBEEF
     ```
5. **Expected Output:** Your sent message should instantly appear in Terminal 1:
   ```text
   can0  123   [4]  DE AD BE EF
   ```

If this test succeeds, your Raspberry Pi’s CAN controller, transceiver, drivers, and libraries are functional.

<img width="1600" height="682" alt="CAN Setup" src="https://github.com/user-attachments/assets/0ed2374d-35f1-4a03-a845-80b8a72bb1bd" />

---

## 6. Switching to Normal Mode (Connecting the Motor)
To switch back to standard transceiver mode so the Pi can send physical signals to your actuator:

```bash
# 1. Take the interface down
sudo ip link set can0 down

# 2. Turn loopback off
sudo ip link set can0 type can bitrate 1000000 loopback off

# 3. Bring the interface back up
sudo ip link set can0 up
```
The Raspberry Pi is now ready to transmit signals through the physical CAN High and CAN Low wires to the motor.
