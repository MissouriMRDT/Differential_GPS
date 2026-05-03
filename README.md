# Differential GPS Navigation Board

This repository contains the software and configuration for the Mars Rover Design Team's Differential GPS Navigation Board. The core of the system is a Python daemon (`nav_board.py`) that interfaces with an ArduPilot-based flight controller (e.g., ARK FPV, Pixhawk) over a USB MAVLink 2 connection. It extracts high-precision RTK GPS data and EKF-fused compass headings, and broadcasts them to the rover's autonomy system via RoveComm.

## Table of Contents

1. [Hardware Requirements](#1-hardware-requirements)

2. [Raspberry Pi Setup (From Scratch)](#2-raspberry-pi-setup-from-scratch)

3. [Flight Controller (ArduPilot) Setup](#3-flight-controller-ardupilot-setup)

4. [GPS & Compass Setup](#4-gps--compass-setup)

5. [Software Installation](#5-software-installation)

6. [Deployment (Systemd Service)](#6-deployment-systemd-service)

7. [Troubleshooting](#7-troubleshooting)

## 1. Hardware Requirements

* **Companion Computer:** Raspberry Pi 4 or 5 (or similar Linux SBC)

* **Flight Controller:** ArduPilot compatible board (ARK FPV, Pixhawk, Cube)

* **GPS:** U-blox F9P (Dual Antenna RTK) or M8P module with integrated compass

* **Connection:** High-quality USB-A to USB-C/Micro cable connecting the Pi to the Flight Controller

## 2. Raspberry Pi Setup (From Scratch)

If you are starting with a brand-new Raspberry Pi, follow these steps to configure the OS for hardware serial communication.

### Step 2.1: Flash the OS

1. Download and install the **Raspberry Pi Imager**.

2. Select **Ubuntu Server 22.04 LTS (64-bit)** or **Raspberry Pi OS Lite (64-bit)**.

3. Click the **Gear Icon (Advanced Options)** before writing:

   * Set the hostname (e.g., `nav-board`).

   * Enable SSH (Use password authentication or provide your public key).

   * Set your username and password.

   * Configure Wi-Fi if necessary.

4. Write the image to the SD card, insert it into the Pi, and boot it up.

### Step 2.2: System Dependencies & Permissions

SSH into your Raspberry Pi and run the following system updates:

```
sudo apt update && sudo apt upgrade -y
sudo apt install -y git python3-pip pipenv build-essential
```

**CRITICAL: Add user to the `dialout` group.** Without this, Python cannot read data from the USB serial ports.

```
sudo usermod -a -G dialout $USER
```

*(You must log out and log back in, or reboot the Pi, for this to take effect).*

### Step 2.3: Disable ModemManager

Linux systems include a service called `ModemManager` that assumes all newly connected `/dev/ttyACM*` devices are cellular modems. It will aggressively ping the flight controller with `AT` commands, deadlocking the MAVLink connection. Disable it permanently:

```
sudo systemctl stop ModemManager
sudo systemctl disable ModemManager
```

## 3. Flight Controller (ArduPilot) Setup

The flight controller acts as an advanced sensor-fusion board. You must configure it via **Mission Planner** or **QGroundControl** before running this script.

### Step 3.1: MAVLink 2 Configuration

Connect the flight controller to your laptop, open QGroundControl, and navigate to the **Parameters** tab. Search for the `SERIAL0` parameters (Serial 0 represents the main USB Virtual COM Port).

* **`SERIAL0_PROTOCOL` = `2` (MAVLink 2)** \* *Required to transmit extended precision fields like `hdg_acc`.*

* **`SERIAL0_BAUD` = `115200`**

### Step 3.2: Rover EKF Tweaks (Crucial for Ground Vehicles)

Because rovers drive backwards and slip sideways, the default drone-based EKF (Extended Kalman Filter) will panic and violently snap the heading by 90° or 180° if the GPS track disagrees with the compass. **Apply these settings to force the EKF to strictly trust the compass:**

* **`EK3_GSF_USE_MASK` = `0`** (Disabled)

  * *Prevents the flight controller from resetting the yaw based on the GPS ground-course.*

* **`COMPASS_LEARN` = `0`** (Disabled)

  * *Prevents the EKF from permanently altering compass calibrations when you drive in reverse.*

* **`EK3_SRC1_YAW` = `1`** (Compass)

  * *Forces the EKF to use the Magnetometer as the sole source of truth for heading.*

## 4. GPS & Compass Setup

### Step 4.1: Physical Mounting

* **Interference:** Rovers generate massive Electro-Magnetic Interference (EMI) from drive motors and high-current power wires. The GPS/Compass puck **must** be mounted on a non-magnetic mast, at least several inches away from power traces.

* **Orientation:** The physical arrow on the GPS puck must point exactly towards the front of the rover. If physical constraints force you to mount it rotated, you must update the **`COMPASS_ORIENT`** parameter in ArduPilot (e.g., `Yaw90`, `Yaw180`).

### Step 4.2: Calibration

Once the GPS/Compass is permanently mounted on the rover, take the rover outside away from metal buildings. Connect to QGroundControl, go to **Sensors -> Compass**, and perform a full compass calibration dance. **Do not skip this step, or your heading will drift based on which direction you face.**

## 5. Software Installation

### Step 5.1: Clone the Repository

Clone this repository to the Raspberry Pi. **You must use `--recursive` to pull down the RoveComm submodule.**

```
git clone --recursive [https://github.com/MarsRoverDesignTeam/Differential_GPS.git](https://github.com/MarsRoverDesignTeam/Differential_GPS.git)
cd Differential_GPS
```

*(If you forgot `--recursive`, run `git submodule update --init --recursive` inside the folder).*

### Step 5.2: Install Python Dependencies

This project uses `pipenv` to manage dependencies (PyMavlink, PySerial, etc.) in a clean virtual environment.

```
pipenv install
```

### Step 5.3: Test the Script

Plug the flight controller into the Raspberry Pi via USB. Run the script manually to verify data is flowing:

```
pipenv run python nav_board.py
```

If successful, you should see logs stating `EKF Absolute Horizontal Position Lock Acquired!` followed by a stream of `[TX GPS]` and `[TX Compass]` coordinates.

## 6. Deployment (Systemd Service)

To make the script run automatically in the background every time the Raspberry Pi turns on, install it as a `systemd` service utilizing the Pipenv virtual environment.

### Step 6.1: Locate your Pipenv executable

Systemd needs the absolute path to `pipenv`. Run this command to find it:

```
which pipenv
```

*(Note the output, e.g., `/usr/bin/pipenv` or `/home/ubuntu/.local/bin/pipenv`)*.

### Step 6.2: Configure the Service File

Open the included `nav_board.service` file and ensure it is structured like this. Replace `ubuntu` with your actual username, and update the `ExecStart` path with the output from Step 6.1:

```
[Unit]
Description=Differential GPS MAVLink Service
After=network.target

[Service]
Type=simple
# Change 'ubuntu' to your actual Raspberry Pi username
User=ubuntu

# systemd MUST start in the folder where the Pipfile lives
WorkingDirectory=/home/ubuntu/Differential_GPS

# Use the absolute path to pipenv (from Step 6.1), followed by "run python"
ExecStart=/usr/bin/pipenv run python nav_board.py

Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

### Step 6.3: Enable and Start the Service

1. Copy the service file to the systemd directory:

   ```
   sudo cp nav_board.service /etc/systemd/system/
   ```

2. Reload the systemd daemon to recognize the new file:

   ```
   sudo systemctl daemon-reload
   ```

3. Enable the service to run on boot:

   ```
   sudo systemctl enable nav_board
   ```

4. Start the service immediately:

   ```
   sudo systemctl start nav_board
   ```

To view the live logs of the background service, run:

```
journalctl -u nav_board -f
```

## 7. Troubleshooting

* **Hanging on "Broadcasting GCS heartbeat..."**

  * Ensure `ModemManager` is fully disabled.

  * Ensure the flight controller's `SERIAL0_PROTOCOL` is exactly `2` (MAVLink 2). PyMavlink will silently discard MAVLink 2 packets if it expects MAVLink 1, and vice versa.

* **"Device /dev/ttyACM1 is dead" or instant crashes on connect**

  * Modern flight controllers expose two virtual USB ports. `ttyACM0` is usually MAVLink, and `ttyACM1` is usually SLCAN (DroneCAN). Sending MAVLink packets to the CAN interface will instantly crash the port. The script is designed to auto-sort and pick `ttyACM0`, but if you have multiple USB devices plugged in, you can force the port:

    ```
    pipenv run python nav_board.py --serial-path /dev/ttyACM0
    ```

* **Heading accuracy stays locked at `360.0`**

  * This is normal if the rover is stationary and utilizing a single-antenna GPS. Single-antenna GPS units can only estimate heading accuracy while moving. Once you drive a few meters, this value will drop. (Dual-antenna RTK setups will provide stationary heading accuracy).

* **Rover heading suddenly snaps 90° or 180° while driving**

  * Your EKF is using GPS Ground Course as a fallback and fighting the compass. Ensure you have followed the parameters outlined in **Step 3.2**.