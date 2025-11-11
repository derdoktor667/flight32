# Flight32 Gemini Context

This document provides context for the Flight32 project, an open-source ESP32 flight controller firmware.

## Project Overview

Flight32 is a robust, extensible, and user-friendly firmware for ESP32-based flight controllers. It leverages the dual-core architecture of the ESP32, dedicating one core to flight-critical tasks and the other to communications and user interaction. The firmware is built on FreeRTOS and features a clean, modular, object-oriented design that makes it easy to understand, modify, and extend.

### Key Technologies

*   **MCU:** ESP32
*   **Framework:** Arduino
*   **RTOS:** FreeRTOS
*   **IMU:** MPU6050
*   **Motor Protocol:** DShot
*   **Receiver Protocols:** IBUS, PPM

### Architecture

The firmware is architected around a set of independent tasks, each responsible for a specific function (e.g., reading sensor data, processing RC input, controlling motors). These tasks are managed by a custom scheduler that runs on FreeRTOS.

The core components are:

*   **`FlightController`:** The main class that initializes and manages all other components.
*   **`Scheduler`:** A simple scheduler that creates and starts all the FreeRTOS tasks.
*   **`TaskBase`:** An abstract base class for all tasks, providing a common interface and basic loop-timing metrics.
*   **Tasks:**
    *   `Mpu6050Task`: Manages the MPU6050 IMU.
    *   `RxTask`: Handles RC receiver input (IBUS or PPM).
    *   `MotorTask`: Controls the motors using the DShot protocol.
    *   `PidTask`: Implements the PID controller for flight stabilization.
    *   `TerminalTask`: Provides a command-line interface for configuration and debugging.
*   **`SettingsManager`:** Manages persistent settings using the ESP32's non-volatile storage (NVS).
*   **`ComManager`:** A communication manager for sending and receiving data between tasks.

## Building and Running

The project can be built and uploaded using the Arduino IDE or the Arduino CLI.

### Arduino IDE

1.  Open `flight32.ino` in the Arduino IDE.
2.  Select your ESP32 board and port.
3.  Click the "Upload" button.

### Arduino CLI

```bash
# Compile the firmware
arduino-cli compile --fqbn esp32:esp32:esp32 flight32.ino

# Upload to your flight controller
arduino-cli upload -p <Your_Port> --fqbn esp32:esp32:esp32 flight32.ino
```

## Development Conventions

*   **Object-Oriented:** The code is written in an object-oriented style.
*   **Modularity:** The firmware is divided into modules (tasks, managers, etc.), each with a specific responsibility.
*   **Configuration:** All major hardware and task parameters are centralized in `src/config.h` for easy customization.
*   **No Magic Numbers:** Magic numbers are avoided in favor of named constants.
*   **Comments:** The code is well-commented, with a focus on explaining the "why" behind the code.
*   **Task-Based:** The firmware is built around a set of independent tasks that communicate with each other through a communication manager.
*   **Real-time:** The firmware is designed to be real-time, with a focus on predictable performance.
*   **Extensibility:** The modular design makes it easy to add new features and sensors.
