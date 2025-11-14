<div align="center">
  <img src="img/flight32_logo_400.jpg" alt="Flight32 Logo" width="400">
  <h1>Flight32 Flight Controller Firmware</h1>
  <p>
    <strong>A high-performance, open-source flight controller firmware designed to unleash the full potential of the ESP32.</strong>
  </p>
  <p>
    <img src="https://img.shields.io/badge/Version-0.3.0-blue.svg" alt="Version">
    <img src="https://img.shields.io/badge/License-MIT-green.svg" alt="License">
    <img src="https://img.shields.io/badge/Platform-ESP32-orange.svg" alt="Platform">
    <img src="https://img.shields.io/badge/Framework-Arduino-cyan.svg" alt="Framework">
    <img src="https://img.shields.io/badge/Build-Passing-brightgreen.svg" alt="Build Status">
  </p>
</div>

---

Flight32 is a robust, extensible, and user-friendly firmware that turns any ESP32 into a powerful flight controller. Built for stability, performance, and ease of use, it's the perfect brain for your next drone project.

## ✨ Core Features

*   🚀 **Peak Performance & Stability**: Experience ultra-stable flight with a real-time FreeRTOS-based scheduler that guarantees reliable, predictable performance.
*   🧠 **Dual-Core Powerhouse**: Harnesses the full power of the ESP32's dual-core architecture, dedicating one core to flight-critical tasks and the other to communications.
*   🧩 **Clean & Modular Architecture**: A clean, object-oriented design makes the firmware easy to understand, modify, and extend.
*   📊 **Real-time System Insights**: Tune and debug on the fly with a powerful, built-in terminal and MultiWii Serial Protocol (MSP) support. Monitor CPU load, loop times, and memory usage to squeeze every drop of performance out of your hardware.
*   🎛️ **Persistent On-the-Fly Tuning**: A full PID controller and complete channel mapping are easily adjustable via the terminal, with all settings saved persistently to non-volatile storage.
*   📡 **Extensible Receiver & IMU Support**: Built with a generic task structure to easily support new receiver protocols (currently IBUS, PPM) and IMU sensors (currently MPU6050).
*   🔧 **Configurable MPU6050**: Fine-tune your MPU6050 with configurable gyroscope range, accelerometer range, and low-pass filter settings, along with improved temperature accuracy.
*   ⚙️ **DShot Motor Control**: Precise and efficient digital motor control using the ESP32's RMT peripheral.

## 🎬 Terminal in Action

Get immediate insight into the system's performance with the `tasks` command.

```plaintext
[Flight32 ~]$ tasks

Task Name        State      Prio   CPU %    Loop (us)  Avg (us)   Max (us)   Stack HWM (bytes)
-------------------------------------------------------------------------------------------------------------------
RX / Receiver    Waiting    4      0.06     1          0          7          3556
IMU / Sensor     Waiting    5      1.29     351        354        488        2716
MOTORS / DShot   Waiting    3      0.70     100        100        285        3112
PID Controller   Waiting    4      10.76    1766       1744       1998       2516
Serial Manager   Running    1      0.03     7          7          34         7216
-------------------------------------------------------------------------------------------------------------------
```

## 📂 Codebase Structure

The Flight32 firmware is organized into a clean, modular structure to promote readability, maintainability, and ease of extension.

```
/
├── flight32.ino                # Main Arduino sketch file
├── install_libs.sh             # Script to install required libraries
├── README.md                   # You are here!
└── src/
    ├── com_manager.cpp/h       # Core communication (logging) manager
    ├── flight_controller.cpp/h # Main flight controller class
    ├── settings_manager.cpp/h  # Manages persistent settings in NVS
    ├── config/                 # Header-only configuration files for modules
    ├── imu/                    # IMU sensor interfaces and implementations
    ├── pid/                    # PID controller implementation
    ├── protocols/              # MSP, IBUS, and other communication protocols
    ├── scheduler/              # FreeRTOS task scheduler
    ├── tasks/                  # All major system tasks (IMU, RX, Motor, etc.)
    ├── terminal/               # Interactive serial terminal
    └── utils/                  # Utility functions and version info
```

## 🏁 Quick Start

Get your drone in the air in just a few minutes.

1.  **Install Board Support**: Make sure you have the **ESP32 board package** installed in your Arduino environment.

2.  **Install External Libraries**: Run the provided script to automatically install the required external libraries.

    ```bash
    # Make the script executable
    chmod +x install_libs.sh

    # Run the installer
    ./install_libs.sh
    ```

3.  **Compile & Upload (Arduino IDE)**: Open `flight32.ino` in the Arduino IDE, select your board and port, and hit upload. The IMU will automatically calibrate on every boot.

#### For Power Users (Arduino CLI)

```bash
# 1. Install libraries first by running ./install_libs.sh

# 2. Compile the firmware
arduino-cli compile --fqbn esp32:esp32:esp32 flight32.ino

# 3. Upload to your flight controller
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 flight32.ino
```

## 🎛️ Full Control via Serial Manager

Our interactive serial manager provides complete control over your flight controller through both a powerful command-line interface (CLI) and MultiWii Serial Protocol (MSP) support. Here are some of the available CLI commands (type `help` in the terminal for a full list).

<details>
  <summary><strong>System Commands</strong></summary>
  
  *   `help`           - Shows the main help message or help for a specific category.
  *   `status`         - Displays firmware version and system status.
  *   `tasks`          - Provides a detailed list of all running tasks.
  *   `mem`            - Shows current memory usage.
  *   `reboot`         - Reboots the flight controller.
  *   `factory_reset confirm` - Resets all settings to their default values.
</details>

<details>
  <summary><strong>Sensor & IMU Commands</strong></summary>
  
  *   `get imu.data`   - Displays the latest accelerometer, gyroscope, and temperature readings.
  *   `get imu.config` - Shows the current sensor configuration.
  *   `set imu.calibrate`  - Starts the gyro calibration routine.
</details>

<details>
  <summary><strong>Receiver (RX) Commands</strong></summary>
  
  *   `get rx.data`        - Shows the latest RX channel data.
  *   `set rx.protocol <protocol>` - Sets the RX protocol (e.g., 'set rx.protocol IBUS').
  *   `get rx.value.all`   - Shows all mapped RX channel values.
  *   `set rx.channel.<name> <index>` - Sets the RX channel mapping (e.g., 'set rx.channel.roll 1').
</details>

<details>
  <summary><strong>PID Tuning Commands</strong></summary>
  
  *   `get pid`        - Gets the current PID gains for all axes.
  *   `set pid <axis> <p|i|d> <value>` - Sets a specific PID gain (e.g., `set pid roll p 20`).
  *   `reset pid confirm` - Resets all PID gains to their default values.
</details>

<details>
  <summary><strong>Settings & Configuration Commands</strong></summary>
  
  *   `get <key>`      - Gets the value of a specific setting.
  *   `set <key> <value>`- Sets a new value for a setting.
  *   `dump`           - Dumps all settings in a parsable format for backup.
  *   `save`           - Saves all current settings to persistent storage.
  *   `settings`       - Lists all available settings.
</details>

---

## ESP32 Devkit Pinout (Common)

This section provides a common pinout reference for ESP32 Devkit boards, focusing on pins typically used in flight controller applications. Please note that pin configurations can vary between different ESP32 versions and manufacturers. Always refer to your specific board's documentation.

**Power & Ground:**
*   **5V:** 5V power supply input/output.
*   **3.3V:** 3.3V power supply output.
*   **GND:** Ground.

**I2C (for IMU, e.g., MPU6050):**
*   **GPIO21 (SDA):** I2C Data Line.
*   **GPIO22 (SCL):** I2C Clock Line.

**UART (for RX, e.g., IBUS):**
*   **GPIO16 (RX2):** UART2 Receive.
*   **GPIO17 (TX2):** UART2 Transmit.

**Motor Control (DShot - example GPIOs, can be configured):**
*   **GPIO18:** Motor 1 Signal (Front Right).
*   **GPIO19:** Motor 2 Signal (Front Left).
*   **GPIO23:** Motor 3 Signal (Rear Right).
*   **GPIO25:** Motor 4 Signal (Rear Left).

**Note:** This is a general guide. Always consult the schematic and pinout diagram specific to your ESP32 Devkit board.

---

## 🗺️ Roadmap

Flight32 is an actively developed project. Here's a glimpse of what's on the horizon:

- [ ] Integration of more advanced sensors (e.g., barometer, magnetometer).
- [ ] Implementation of different flight modes (e.g., Acro, Angle, Horizon).
- [ ] Support for more receiver protocols (e.g., CRSF, SBUS).
- [ ] Support for more ESC protocols.

## 🤝 Contribute

Have an idea or a bug fix? We welcome all contributions! Feel free to open an issue or submit a pull request. Your input helps make Flight32 the best open-source flight controller firmware available.

---

<p align="center">
  Licensed under the <a href="./LICENSE">MIT License</a>.
</p>