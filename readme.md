# CrashTap: Hit and Run Detection Device

## Overview

CrashTap is a device designed to help identify hit-and-run cases by detecting collisions between vehicles. It utilizes a piezoelectric sensor to detect impact and transmits relevant data via BLE (Bluetooth Low Energy) using an ESP32 microcontroller. The device is equipped with an RFID/NFC module to ensure vehicle identification and tracking.

## Features

- **Collision Detection**: Uses a piezoelectric sensor to detect sudden impacts
- **Wireless Communication**: Built-in BLE (Bluetooth Low Energy) for data transmission
- **Vehicle Identification**: RFID/NFC module to store vehicle details
- **Compact and Efficient**: Designed to be integrated seamlessly into vehicles

## Device Specifications

| Component            | Description                                 |
| -------------------- | ------------------------------------------- |
| Microcontroller      | ESP32 (Built-in BLE)                        |
| Sensor               | Piezoelectric sensor (for impact detection) |
| Communication Module | RFID/NFC (for vehicle identification)       |

## How It Works

1. The piezoelectric sensor detects a collision and sends a signal to the ESP32
2. The ESP32 processes the signal and logs the impact event
3. Once a collision is detected, the BLE modules of both vehicle devices broadcast their Vehicle Registration Numbers to each other
4. The RFID/NFC module captures the identity of the involved vehicles
5. The data can be used to identify vehicles involved in the collision and assist in investigations

## Future Enhancements

- [ ] **Enhanced Impact Analysis**: To differentiate between minor and major collisions
