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
#include "../config/terminal_config.h"
#include "../com_manager.h"
#include "imu_task.h"
#include "rx_task.h"
#include "motor_task.h"
#include "pid_task.h"
#include "../terminal/terminal.h"
#include <memory>

#include "../protocols/msp_protocol.h"

class SerialManagerTask : public TaskBase
{
public:
    SerialManagerTask(const char *name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID, uint32_t task_delay_ms, Scheduler *scheduler, ImuTask *imu_task, RxTask *rx_task, MotorTask *motor_task, PidTask *pid_task, SettingsManager *settings_manager);

    void setup() override;
    void run() override;
    void showPrompt();

private:
    Scheduler *_scheduler;
    ImuTask *_imu_task;
    RxTask *_rx_task;
    MotorTask *_motor_task;
    PidTask *_pid_task;
    SettingsManager *_settings_manager;
    std::unique_ptr<Terminal> _terminal; // Use std::unique_ptr for automatic memory management

    // Serial Protocol State
    ComSerialMode _current_mode = ComSerialMode::TERMINAL; // Current serial communication mode
    unsigned long _last_msp_activity_ms = 0;
    static constexpr unsigned long MSP_TIMEOUT_MS = 2000; // Switch back to terminal after 2 seconds of no MSP activity
    uint8_t _msp_handshake_state = 0; // State for MSP handshake sequence

    // MSP Mode Variables
    enum class MspState
    {
        IDLE,
        HEADER_START,
        HEADER_M,
        HEADER_SIZE,
        HEADER_CMD,
        PAYLOAD,
        CRC
    };
    MspState _msp_state = MspState::IDLE;
    uint8_t _msp_payload_size = 0;
    uint8_t _msp_command_id = 0;
    uint8_t _msp_crc = 0;
    uint8_t _msp_payload_buffer[MSP_MAX_PAYLOAD_SIZE];
    uint8_t _msp_payload_index = 0;

    // Terminal Mode Functions
    void _handle_terminal_input(char incoming_char);

    // MSP Mode Functions
    void _parse_msp_char(uint8_t c);
    void _process_msp_message();
    void _send_msp_response(uint8_t cmd, uint8_t *payload, uint8_t size);
    
    // Added new helper function for reading int16_t values
    void _write_int16_to_payload(uint8_t *payload, int &index, int16_t value);
    int16_t _read_int16_from_payload(const uint8_t *payload, int &index);

    // MSP Command Handlers
    void _handle_msp_api_version();
    void _handle_msp_fc_variant();
    void _handle_msp_fc_version();
    void _handle_msp_board_info();
    void _handle_msp_build_info();
    void _handle_msp_reboot();
    void _handle_msp_eeprom_write();
    void _handle_msp_reset_settings();
    void _handle_msp_status();
    void _handle_msp_mem_stats();
    void _handle_msp_get_setting();
    void _handle_msp_set_setting();
    void _handle_msp_pid_get();
    void _handle_msp_pid_set();
    void _handle_msp_raw_imu();
    void _handle_msp_attitude();
    void _handle_msp_rc();
    void _handle_msp_motor();
};
