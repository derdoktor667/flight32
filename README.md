# Flight32: The Open-Source Firmware for Your ESP32 Flight Controller

![Flight32 Logo Placeholder](https://via.placeholder.com/400x150?text=Flight32+Logo)

Welcome to Flight32 – the robust and extensible firmware solution specifically designed for ESP32-based flight controllers. Dive into the world of precise flight control and experience the power of the ESP32 in your next drone project.

## ✨ Highlights that Inspire:

*   **Stable Flight Performance**: A FreeRTOS-based task scheduler ensures reliable and predictable loop times, forming the core of any stable flight control system.
*   **ESP32 Dual-Core Power**: Full utilization of the ESP32's dual-core architecture for efficient task distribution and maximum processing power.
*   **Modular Design**: A clean, object-oriented architecture with clearly defined components (FlightController, Terminal, ComManager) makes the firmware easy to understand and simple to extend.
*   **Real-time Insights**: Monitor your tasks live via the integrated terminal. View CPU load, current, average, and maximum loop times, as well as stack usage – all at a glance for optimal performance analysis.
*   **Professional Terminal Experience**: Enjoy a bash-like prompt, character echo, and backspace support for a more intuitive and professional interaction with the flight controller.

## 🚀 Current Features:

*   **MPU6050 IMU Integration**: Seamless integration of the MPU6050 Inertial Measurement Unit for accelerometer, gyroscope, and temperature readings. Includes calibration and configuration commands.
*   **Flysky IBUS Receiver Support**: Connect your Flysky receiver via Serial2 using the IBUS protocol. Monitor channel data and connection status directly from the terminal.
*   **DShotRMT Motor Control**: Integrated DShot300 motor protocol using the ESP32's RMT peripheral for precise and efficient motor control. Supports up to 4 motors.
*   **Refactored Task Management**: Improved task base class and scheduler for better maintainability and clearer task state reporting ("Waiting" instead of "Blocked").

## 💻 Terminal Commands:

The command-line interface provides real-time interaction and debugging capabilities. Type `help` to see a list of available commands.

### System Commands:
*   `help`           - Shows this help message.
*   `status`         - Shows firmware information.
*   `tasks`          - Shows information about running tasks (state, priority, CPU usage, loop times, stack HWM).
*   `mem`            - Shows current memory usage (heap total, free, min free).
*   `reboot`         - Reboots the ESP32.

### MPU6050 Commands:
*   `get mpu.data`   - Displays the latest accelerometer, gyroscope, and temperature readings from the MPU6050.
*   `get mpu.config` - Shows the current configuration settings of the MPU6050 (gyro range, accel range, LPF bandwidth).
*   `set mpu.calibrate`  - Initiates a calibration routine for the MPU6050 sensor.

### IBUS Commands:
*   `get ibus.data`  - Shows the latest channel data received from the IBUS receiver.
*   `get ibus.status`- Shows the current connection status of the IBUS receiver.

### Motor Commands:
*   `set motor.throttle <id> <throttle>` - Sets the throttle value (0-2047) for a specific motor (0-3). Example: `set motor.throttle 0 1000`.

### Settings Commands:
*   `get <display_key>`      - Gets the value of any setting using its user-friendly display key. Displays both the description and human-readable value where available (e.g., `get gyro.resolution` might show `gyro.resolution (MPU6050 Gyroscope Range): 250_DPS`).
*   `set <display_key> <value>`- Sets the value of any setting using its user-friendly display key. Accepts human-readable values where available (e.g., `set gyro.resolution 250_DPS`). Provides informative feedback on invalid inputs.
*   `dump`           - Dumps all settings in a parsable format for backup, using user-friendly display keys and human-readable values (e.g., `set gyro.resolution = 250_DPS`).
*   `factory_reset confirm` - Resets all settings to their default values and reboots the ESP32. Requires `confirm` argument.

## 🏁 Quick Start:

To get Flight32 running on your ESP32, all you need is the Arduino IDE or Arduino CLI:

1.  **Install Board Support**: Ensure the ESP32 board package is installed in your Arduino environment.
2.  **Compile & Upload**: Open `flight32.ino`, select your board and port, then upload the firmware. Alternatively, use the Arduino CLI:
    ```bash
    arduino-cli compile --fqbn esp32:esp32:esp32 flight32.ino
    arduino-cli upload -p <Your_Port> --fqbn esp32:esp32:esp32 flight32.ino
    ```

## 💡 Outlook:

Flight32 is constantly evolving. Future developments include the integration of more sensors, advanced flight algorithms, and even more intuitive configuration. Be a part of the journey!

## 🤝 Contribute:

We welcome all contributions! Whether it's bug reports, feature suggestions, or code contributions – your ideas help Flight32 become even better.

---

**Flight32 – Take Control.**