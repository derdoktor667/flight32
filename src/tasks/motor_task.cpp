/**
 * @file motor_task.cpp
 * @brief Implements motor control using DShotRMT for ESP32.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

// Motot Mixing matrix:
// RR =  T - P + R - Y  (rear right)
// FR =  T - P - R + Y  (front right)
// RL =  T + P + R + Y  (rear left)
// FL =  T + P - R - Y  (front left)

#include "../config/motor_config.h"
#include "../protocols/serial_4way_protocol.h"
#include "motor_task.h"
#include "../com_manager.h"
#include "../settings_manager.h"
#include <DShotRMT.h>
#include <driver/gpio.h> // Required for gpio_num_t

extern bool _is_passthrough_active; // Global flag from com_manager.cpp

dshot_mode_t get_dshot_mode_from_index(uint8_t index)
{
    switch ((DshotProtocolIndex)index)
    {
    case DshotProtocolIndex::DSHOT150:
        return DSHOT150;
    case DshotProtocolIndex::DSHOT300:
        return DSHOT300;
    case DshotProtocolIndex::DSHOT600:
        return DSHOT600;
    case DshotProtocolIndex::DSHOT1200:
        return DSHOT1200;
    default:
        return DSHOT300;
    }
}

MotorTask::MotorTask(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms,
                     const int *motor_pins, SettingsManager *settings_manager)
    : TaskBase(name, stack_size, priority, core_id, task_delay_ms),
      _motor_pins(motor_pins),
      _settings_manager(settings_manager),
      _throttle_command(0.0f),
      _pitch_command(0.0f),
      _roll_command(0.0f),
      _yaw_command(0.0f),
      _motorTestState(IDLE),
      _testMotorNum(0),
      _testThrottle(0.0f),
      _testDuration(0),
      _testStartTime(0)
{
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        _motor_throttles[i] = 0;
        _dshot_drivers[i] = nullptr;
    }
}



void MotorTask::setup()
{
    uint8_t protocol_index = _settings_manager->getSettingValue(NVS_KEY_MOTOR_PROTOCOL).toInt();
    dshot_mode_t dshot_protocol = get_dshot_mode_from_index(protocol_index);

    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        _dshot_drivers[i] = std::make_unique<DShotRMT>(_motor_pins[i], dshot_protocol);
        _dshot_drivers[i]->begin();
    }

    com_send_log(ComMessageType::LOG_INFO, "MotorTask: DShotRMT drivers initialized for %d motors on protocol %s.", NUM_MOTORS, _settings_manager->getSettingValueHumanReadable(NVS_KEY_MOTOR_PROTOCOL).c_str());
}

void MotorTask::run()
{
    if (!_dshot_drivers[FIRST_MOTOR_INDEX].get())
        return;

    if (_inPassthroughMode)
    {
        // In passthrough mode, the run loop should not control motors.
        // Motor commands will come directly from the passthrough handler.
        return;
    }

    if (_motorTestState != IDLE)
    {
        // Handle motor test mode
        if (_motorTestState == SPINNING_TIMED && (millis() - _testStartTime >= _testDuration))
        {
            stopMotorTest(); // Time's up, stop the motor
            return;          // Exit run() after stopping
        }

        // Apply test throttle to the designated motor, others are off
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            if (_dshot_drivers[i])
            {
                if (i == _testMotorNum)
                {
                    // Convert normalized throttle (0-1) to raw DShot throttle (MOTOR_MIN_THROTTLE_RAW to MOTOR_MAX_THROTTLE_RAW)
                    uint16_t raw_throttle = (uint16_t)constrain(_testThrottle * (MOTOR_MAX_THROTTLE_RAW - MOTOR_MIN_THROTTLE_RAW) + MOTOR_MIN_THROTTLE_RAW, MOTOR_MIN_THROTTLE_RAW, MOTOR_MAX_THROTTLE_RAW);
                    _dshot_drivers[i].get()->sendThrottle(raw_throttle);
                }
                else
                {
                    _dshot_drivers[i]->sendThrottle(0); // Other motors off
                }
            }
        }
        return; // Exit run() as we are in test mode
    }

    // Normal PID control logic
    if (_throttle_command < THROTTLE_DEADZONE_THRESHOLD)
    {
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            _motor_throttles[i] = 0;
        }
    }
    else
    {
        float m_rr = _throttle_command - _pitch_command + _roll_command - _yaw_command;
        float m_fr = _throttle_command - _pitch_command - _roll_command + _yaw_command;
        float m_rl = _throttle_command + _pitch_command + _roll_command + _yaw_command;
        float m_fl = _throttle_command + _pitch_command - _roll_command - _yaw_command;

        const float min_throttle = MOTOR_MIN_THROTTLE_RAW;
        const float max_throttle = MOTOR_MAX_THROTTLE_RAW;

        _motor_throttles[MOTOR_INDEX_RR] = constrain(m_rr * (max_throttle - min_throttle) + min_throttle, min_throttle, max_throttle);
        _motor_throttles[MOTOR_INDEX_FR] = constrain(m_fr * (max_throttle - min_throttle) + min_throttle, min_throttle, max_throttle);
        _motor_throttles[MOTOR_INDEX_RL] = constrain(m_rl * (max_throttle - min_throttle) + min_throttle, min_throttle, max_throttle);
        _motor_throttles[MOTOR_INDEX_FL] = constrain(m_fl * (max_throttle - min_throttle) + min_throttle, min_throttle, max_throttle);
    }

    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        if (_dshot_drivers[i])
        {
            _dshot_drivers[i].get()->sendThrottle(_motor_throttles[i]);
        }
    }
}

void MotorTask::update(float throttle, float pitch, float roll, float yaw)
{
    _throttle_command = throttle;
    _pitch_command = pitch;
    _roll_command = roll;
    _yaw_command = yaw;
}

void MotorTask::setThrottle(uint8_t motor_id, uint16_t throttle)
{
    if (motor_id < NUM_MOTORS && _dshot_drivers[motor_id])
    {
        _dshot_drivers[motor_id].get()->sendThrottle(throttle);
        _motor_throttles[motor_id] = throttle;
    }
}

void MotorTask::startMotorTest(uint8_t motorNum, float throttle, uint32_t duration_ms)
{
    if (motorNum >= NUM_MOTORS)
    {
        com_send_log(ComMessageType::LOG_ERROR, "MotorTest: Invalid motor number %d.", motorNum);
        return;
    }
    if (!_is_valid_throttle_percentage(throttle))
    {
        return;
    }

    stopMotorTest(); // Stop any ongoing test first

    _testMotorNum = motorNum;
    _testThrottle = throttle;
    _testDuration = duration_ms;
    _testStartTime = millis();
    _motorTestState = SPINNING_TIMED;
    com_send_log(ComMessageType::LOG_INFO, "MotorTest: Motor %d spinning at %.1f%% throttle for %lu ms.", motorNum + 1, throttle * CPU_PERCENTAGE_FACTOR, duration_ms);
}

void MotorTask::startContinuousMotorTest(uint8_t motorNum, float throttle)
{
    if (motorNum >= NUM_MOTORS)
    {
        com_send_log(ComMessageType::LOG_ERROR, "MotorTest: Invalid motor number %d.", motorNum);
        return;
    }
    if (!_is_valid_throttle_percentage(throttle))
    {
        return;
    }

    stopMotorTest(); // Stop any ongoing test first

    _testMotorNum = motorNum;
    _testThrottle = throttle;
    _motorTestState = SPINNING_CONTINUOUS;
    com_send_log(ComMessageType::LOG_INFO, "MotorTest: Motor %d spinning continuously at %.1f%% throttle. Send 'motor stop' to stop.", motorNum + 1, throttle * CPU_PERCENTAGE_FACTOR);
}

void MotorTask::stopMotorTest()
{
    if (_motorTestState != IDLE)
    {
        _motorTestState = IDLE;
        _testMotorNum = 0;
        _testThrottle = 0.0f;
        _testDuration = 0;
        _testStartTime = 0;
        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            if (_dshot_drivers[i])
            {
                _dshot_drivers[i].get()->sendThrottle(0); // Stop all motors
            }
        }
        com_send_log(ComMessageType::LOG_INFO, "MotorTest: All motors stopped.");
    }
}

bool MotorTask::_is_valid_throttle_percentage(float throttle)
{
    if (throttle < MIN_NORMALIZED_THROTTLE || throttle > MAX_NORMALIZED_THROTTLE)
    {
        com_send_log(ComMessageType::LOG_ERROR, "MotorTest: Invalid throttle percentage %f. Must be between %.0f and %.0f.", throttle, MIN_NORMALIZED_THROTTLE, MAX_NORMALIZED_THROTTLE);
        return false;
    }
    return true;
}

uint16_t MotorTask::getMotorOutput(uint8_t motor_id) const
{
    if (motor_id < NUM_MOTORS)
    {
        return _motor_throttles[motor_id];
    }
    return 0; // Return 0 for invalid motor_id
}

void MotorTask::enterPassthroughMode()
{
    _inPassthroughMode = true;
    stopMotorTest(); // Ensure motors are stopped if a test was running

    // Initialize 1-Wire for selected motor
    // For now, setting motor 0 as selected for passthrough. This will be dynamic later.
    _passthrough_motor_idx = 0; 
    _1wire_init(_passthrough_motor_idx);

    com_send_log(ComMessageType::LOG_INFO, "MotorTask: Entered passthrough mode.");
}

void MotorTask::exitPassthroughMode()
{
    _inPassthroughMode = false;

    // De-initialize 1-Wire for selected motor
    _1wire_deinit(_passthrough_motor_idx);

    com_send_log(ComMessageType::LOG_INFO, "MotorTask: Exited passthrough mode.");
}

// 1-Wire Passthrough Helper Functions
void MotorTask::_1wire_init(uint8_t motor_idx)
{
    if (motor_idx >= NUM_MOTORS) return; // Invalid motor

    _passthrough_motor_idx = motor_idx;
    _passthrough_gpio = (gpio_num_t)_motor_pins[motor_idx];

    // Store original DShot mode
    if (_dshot_drivers[motor_idx]) {
        _original_dshot_mode = _dshot_drivers[motor_idx]->getMode();
    } else {
        _original_dshot_mode = DSHOT_OFF; // Default to DSHOT_OFF if driver not active
    }

    // Disable the DShot driver
    _dshot_drivers[motor_idx].reset(); // Release unique_ptr

    // Configure GPIO for 1-Wire communication (initial state: input high)
    gpio_reset_pin(_passthrough_gpio);
    gpio_set_direction(_passthrough_gpio, GPIO_MODE_INPUT);
    gpio_set_pull_mode(_passthrough_gpio, GPIO_PULLUP_ONLY);

    com_send_log(ComMessageType::LOG_INFO, "MotorTask: 1-Wire initialized for motor %d on GPIO %d.", motor_idx, _passthrough_gpio);
}

void MotorTask::_1wire_deinit(uint8_t motor_idx)
{
    if (motor_idx >= NUM_MOTORS) return; // Invalid motor

    // Re-initialize DShot driver
    uint8_t protocol_index = _settings_manager->getSettingValue(NVS_KEY_MOTOR_PROTOCOL).toInt();
    dshot_mode_t dshot_protocol = get_dshot_mode_from_index(protocol_index);

    _dshot_drivers[motor_idx] = std::make_unique<DShotRMT>((gpio_num_t)_motor_pins[motor_idx], dshot_protocol);
    _dshot_drivers[motor_idx]->begin();

    com_send_log(ComMessageType::LOG_INFO, "MotorTask: 1-Wire de-initialized for motor %d. DShot re-initialized.", motor_idx);
}

void MotorTask::_1wire_output_mode()
{
    gpio_set_direction(_passthrough_gpio, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(_passthrough_gpio, GPIO_FLOATING);
}

void MotorTask::_1wire_input_mode()
{
    gpio_set_direction(_passthrough_gpio, GPIO_MODE_INPUT);
    gpio_set_pull_mode(_passthrough_gpio, GPIO_PULLUP_ONLY);
}

void MotorTask::_1wire_set_high()
{
    gpio_set_level(_passthrough_gpio, 1);
}

void MotorTask::_1wire_set_low()
{
    gpio_set_level(_passthrough_gpio, 0);
}

int MotorTask::_1wire_read_pin()
{
    return gpio_get_level(_passthrough_gpio);
}

void MotorTask::_1wire_half_bit_delay()
{
    delayMicroseconds(sil_half_bit_delay_us);
}

void MotorTask::_1wire_byte_delay()
{
    delayMicroseconds(sil_t_byte_us);
}

uint8_t MotorTask::_1wire_read_byte()
{
    uint8_t data = 0;
    for (int i = 0; i < 8; i++)
    {
        _1wire_input_mode();
        _1wire_half_bit_delay(); // Wait for bit to settle
        data >>= 1; // Shift right to build byte from LSB
        if (_1wire_read_pin() == 1) // Explicitly check for HIGH
        {
            data |= 0x80; // If pin is high, set MSB (since we shifted right)
        }
        _1wire_half_bit_delay(); // Complete bit time
    }
    return data;
}

void MotorTask::_1wire_send_byte(uint8_t byte)
{
    _1wire_output_mode();
    for (int i = 0; i < 8; i++)
    {
        if (byte & 0x01) // Send LSB first
        {
            _1wire_set_high();
        }
        else
        {
            _1wire_set_low();
        }
        _1wire_half_bit_delay(); // Half bit duration
        _1wire_set_low();        // Ensure low for next half
        _1wire_half_bit_delay(); // Remaining half bit duration
        byte >>= 1;
    }
}

uint8_t MotorTask::_1wire_send_byte_and_read_byte(uint8_t byte)
{
    _1wire_send_byte(byte);
    _1wire_input_mode(); // Switch to input for reading
    _1wire_half_bit_delay(); // Wait for response
    return _1wire_read_byte();
}

uint8_t MotorTask::_1wire_transaction(uint8_t cmd, uint16_t address, uint8_t* data, uint8_t length, bool is_write)
{
    uint8_t checksum = 0;
    uint8_t response_ack = NACK;
    uint8_t response_length = 0;

    // Send START_BYTE
    _1wire_send_byte(START_BYTE);
    checksum += START_BYTE;

    // Send LENGTH
    uint8_t payload_length = 0;
    if (is_write) {
        // Length of address (2) + data length
        payload_length = 2 + length;
    } else {
        // Length of address (2)
        payload_length = 2;
    }
    _1wire_send_byte(payload_length);
    checksum += payload_length;

    // Send COMMAND
    _1wire_send_byte(cmd);
    checksum += cmd;

    // Send ADDRESS
    _1wire_send_byte((uint8_t)(address >> 8)); // Address high
    checksum += (uint8_t)(address >> 8);
    _1wire_send_byte((uint8_t)address);      // Address low
    checksum += (uint8_t)address;

    // Send DATA (if writing)
    if (is_write) {
        for (int i = 0; i < length; i++) {
            _1wire_send_byte(data[i]);
            checksum += data[i];
        }
    }

    // Send CHECKSUM
    _1wire_send_byte(checksum);

    // Read response
    uint8_t rx_start_byte = _1wire_read_byte(); // Should be START_BYTE
    uint8_t rx_length = _1wire_read_byte();
    
    if (rx_length > 0) {
        // Read response data
        for (int i = 0; i < rx_length; i++) {
            if (!is_write && data) { // Only fill data if reading
                data[i] = _1wire_read_byte();
            } else {
                _1wire_read_byte(); // Discard byte if not reading
            }
        }
    }
    
    response_ack = _1wire_read_byte(); // ACK/NACK
    uint8_t response_checksum = _1wire_read_byte(); // Checksum

    // TODO: Validate CRC of response

    return response_ack;
}

uint8_t MotorTask::passthroughRead(uint8_t motorId, uint16_t address, uint8_t* data, uint8_t length)
{
    _is_passthrough_active = true; // Suppress logs during passthrough operation

    if (motorId >= NUM_MOTORS) {
        _is_passthrough_active = false;
        return ACK_I_INVALID_CHANNEL;
    }

    _passthrough_motor_idx = motorId; // Set selected motor for 1-Wire functions

    // Handshake (simplified version of BL_ConnectEx for basic read)
    _1wire_output_mode();
    _1wire_set_high();
    delayMicroseconds(sil_t_reset_us);
    _1wire_input_mode();
    _1wire_half_bit_delay(); // Wait for bit to settle

    uint8_t response = _1wire_read_byte(); // Read dummy byte

    if (response != 0x00) // Expect 0x00 from ESC during reset
    {
        com_send_log(ComMessageType::LOG_ERROR, "Passthrough Read: Handshake failed (response: %d).", response);
        _is_passthrough_active = false;
        return ACK_D_GENERAL_ERROR;
    }

    _1wire_send_byte(0x30); // Send 'Connect' command (simplified, actual is more complex)
    response = _1wire_read_byte();

    if (response != 0xCC) // Expected response for successful connection
    {
        com_send_log(ComMessageType::LOG_ERROR, "Passthrough Read: Connect failed (response: %d).", response);
        _is_passthrough_active = false;
        return ACK_D_GENERAL_ERROR;
    }

    // Use _1wire_transaction for actual read
    uint8_t transaction_result = _1wire_transaction(READ_DATA_BLOCK, address, data, length, false);

    _is_passthrough_active = false;
    return transaction_result;
}

uint8_t MotorTask::passthroughWrite(uint8_t motorId, uint16_t address, const uint8_t* data, uint8_t length)
{
    _is_passthrough_active = true; // Suppress logs during passthrough operation

    if (motorId >= NUM_MOTORS) {
        _is_passthrough_active = false;
        return ACK_I_INVALID_CHANNEL;
    }

    _passthrough_motor_idx = motorId; // Set selected motor for 1-Wire functions

    // Handshake (simplified version of BL_ConnectEx for basic write)
    _1wire_output_mode();
    _1wire_set_high();
    delayMicroseconds(sil_t_reset_us);
    _1wire_input_mode();
    _1wire_half_bit_delay(); // Wait for bit to settle

    uint8_t response = _1wire_read_byte(); // Read dummy byte

    if (response != 0x00) // Expect 0x00 from ESC during reset
    {
        com_send_log(ComMessageType::LOG_ERROR, "Passthrough Write: Handshake failed (response: %d).", response);
        _is_passthrough_active = false;
        return ACK_D_GENERAL_ERROR;
    }

    _1wire_send_byte(0x30); // Send 'Connect' command (simplified, actual is more complex)
    response = _1wire_read_byte();

    if (response != 0xCC) // Expected response for successful connection
    {
        com_send_log(ComMessageType::LOG_ERROR, "Passthrough Write: Connect failed (response: %d).", response);
        _is_passthrough_active = false;
        return ACK_D_GENERAL_ERROR;
    }

    // Use _1wire_transaction for actual write
    uint8_t transaction_result = _1wire_transaction(WRITE_DATA_BLOCK, address, const_cast<uint8_t*>(data), length, true);

    _is_passthrough_active = false;
    return transaction_result;
}

uint8_t MotorTask::passthroughErase(uint8_t motorId, uint16_t address)
{
    _is_passthrough_active = true; // Suppress logs during passthrough operation

    if (motorId >= NUM_MOTORS) {
        _is_passthrough_active = false;
        return ACK_I_INVALID_CHANNEL;
    }

    _passthrough_motor_idx = motorId; // Set selected motor for 1-Wire functions

    // Handshake (simplified version of BL_ConnectEx for basic erase)
    _1wire_output_mode();
    _1wire_set_high();
    delayMicroseconds(sil_t_reset_us);
    _1wire_input_mode();
    _1wire_half_bit_delay(); // Wait for bit to settle

    uint8_t response = _1wire_read_byte(); // Read dummy byte

    if (response != 0x00) // Expect 0x00 from ESC during reset
    {
        com_send_log(ComMessageType::LOG_ERROR, "Passthrough Erase: Handshake failed (response: %d).", response);
        _is_passthrough_active = false;
        return ACK_D_GENERAL_ERROR;
    }

    _1wire_send_byte(0x30); // Send 'Connect' command (simplified, actual is more complex)
    response = _1wire_read_byte();

    if (response != 0xCC) // Expected response for successful connection
    {
        com_send_log(ComMessageType::LOG_ERROR, "Passthrough Erase: Connect failed (response: %d).", response);
        _is_passthrough_active = false;
        return ACK_D_GENERAL_ERROR;
    }

    // Use _1wire_transaction for actual erase
    uint8_t transaction_result = _1wire_transaction(ERASE_CODE_PAGE, address, nullptr, 0, true);

    _is_passthrough_active = false;
    return transaction_result;
}