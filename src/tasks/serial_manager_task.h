/**
 * @file serial_manager_task.h
 * @brief Defines the SerialManagerTask class for managing serial communication.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include "../scheduler/task_base.h"
#include "../scheduler/scheduler.h"
#include "../settings_manager.h"

#include "../config/serial_config.h" // Explicitly include for PASSTHROUGH_BUFFER_SIZE
#include "../com_manager.h"
class ImuTask;
class RxTask;
class MotorTask;
class PidTask;
#include "../terminal/terminal.h"
#include <memory>

#include "../protocols/msp_protocol.h"

#include "../protocols/msp_processor.h"

// Forward declaration
class FlightController;

class SerialManagerTask : public TaskBase
{
public:
    SerialManagerTask(const char *name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID, uint32_t task_delay_ms, Scheduler *scheduler, FlightController* flightController, ImuTask *imu_task, RxTask *rx_task, MotorTask *motor_task, PidTask *pid_task, SettingsManager *settings_manager);

    void setup() override;
    void run() override;
    void showPrompt();

    void enterPassthroughMode();
    void exitPassthroughMode();

private:
    Scheduler *_scheduler;
    FlightController* _flightController;
    ImuTask *_imu_task;
    RxTask *_rx_task;
    MotorTask *_motor_task;
    PidTask *_pid_task;
    SettingsManager *_settings_manager;
    std::unique_ptr<Terminal> _terminal; // Use std::unique_ptr for automatic memory management
    std::unique_ptr<MspProcessor> _msp_processor;

    bool _inPassthroughMode = false;

    // Passthrough Parser State
    enum class PassthroughParserState
    {
        IDLE,
        COMMAND,
        ADDR_H,
        ADDR_L,
        PARAM_LEN,
        PARAMS,
        CRC_H,
        CRC_L
    };
    
    PassthroughParserState _passthrough_state = PassthroughParserState::IDLE;
    uint8_t _passthrough_cmd = 0;
    uint16_t _passthrough_addr = 0;
    uint8_t _passthrough_param_len = 0;
    uint8_t _passthrough_params[PASSTHROUGH_BUFFER_SIZE];
    uint8_t _passthrough_param_index = 0;
    uint16_t _passthrough_crc_expected = 0;
    uint16_t _passthrough_crc_calculated = 0;

    // Serial Protocol State
    ComSerialMode _current_mode = ComSerialMode::TERMINAL; // Current serial communication mode
    unsigned long _last_msp_activity_ms = 0;

    bool _should_show_prompt = true; // New flag to control prompt display

    // MSP Mode Variables
    enum class MspState
    {
        IDLE,
        PROTO_IDENTIFIER,
        DIRECTION_V1,
        DIRECTION_V2,
        FLAG_V2,
        PAYLOAD_LENGTH_V1,
        PAYLOAD_LENGTH_JUMBO_LOW,
        PAYLOAD_LENGTH_JUMBO_HIGH,
        PAYLOAD_LENGTH_V2_LOW,
        PAYLOAD_LENGTH_V2_HIGH,
        CODE_V1,
        CODE_JUMBO_V1,
        CODE_V2_LOW,
        CODE_V2_HIGH,
        PAYLOAD_V1,
        PAYLOAD_V2,
        CHECKSUM_V1,
        CHECKSUM_V2
    };
    
    MspState _msp_state = MspState::IDLE;
    uint8_t _msp_protocol_version = 0; // 1 for V1, 2 for V2
    uint8_t _msp_flag_v2 = 0;          // Flag byte for MSP v2
    uint16_t _msp_command_id = 0;      // 2 bytes for MSP v2
    uint16_t _msp_message_length_expected = 0; // 2 bytes for MSP v2 and jumbo frames
    uint16_t _msp_message_length_received = 0;
    uint8_t _msp_crc = 0;
    uint8_t _msp_payload_buffer[MSP_MAX_PAYLOAD_SIZE]; // Consider increasing this for MSP V2 if needed
    uint8_t _msp_payload_index = 0;

    // Terminal Mode Functions
    void _handle_terminal_input(char incoming_char);

    // MSP Mode Functions
    bool _parse_msp_char(uint8_t c); // Returns true if char was consumed by MSP, false otherwise
    void _process_msp_message();
    static uint8_t _crc8_dvb_s2(uint8_t crc, uint8_t ch);

    // Passthrough Mode Functions
    void _parse_passthrough_char(uint8_t c);
    void _process_passthrough_message();
    void _send_passthrough_response(uint8_t cmd, uint16_t addr, const uint8_t *param, uint8_t param_len, uint8_t ack);
    uint16_t _crc_xmodem_update(uint16_t crc, uint8_t data);
};
