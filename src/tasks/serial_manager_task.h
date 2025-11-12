#pragma once

#include <Arduino.h>
#include "../scheduler/task_base.h"
#include "../scheduler/scheduler.h"
#include "../settings_manager.h"
#include "../config.h"
#include "../com_manager.h"
#include "imu_task.h"
#include "rx_task.h"
#include "motor_task.h"
#include "pid_task.h"
#include "../terminal/terminal.h" // Include the new Terminal class

// MSP Commands
#define MSP_API_VERSION 1
#define MSP_FC_VARIANT 2
#define MSP_FC_VERSION 3
#define MSP_BOARD_INFO 4
#define MSP_BUILD_INFO 5
#define MSP_REBOOT 6
#define MSP_STATUS 7
#define MSP_MEM_STATS 8
#define MSP_GET_SETTING 9
#define MSP_SET_SETTING 10
#define MSP_PID 11
#define MSP_SET_PID 202
#define MSP_EEPROM_WRITE 200
#define MSP_RESET_SETTINGS 201

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
    Terminal *_terminal; // Instance of the new Terminal class

    // Serial Protocol State
    enum class SerialMode {
        TERMINAL,
        MSP
    };
    SerialMode _current_mode = SerialMode::TERMINAL;
    unsigned long _last_msp_activity_ms = 0;
    static constexpr unsigned long MSP_TIMEOUT_MS = 2000; // Switch back to terminal after 2 seconds of no MSP activity

    // MSP Mode Variables
    enum class MspState {
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

    // Common Helpers
    // const char *_format_bytes(uint32_t bytes); // Moved to com_manager.h
    // char _byte_buffer[BYTE_BUFFER_SIZE]; // Moved to com_manager.cpp

    // Terminal Mode Functions
    void _handle_terminal_input(char incoming_char);

    // MSP Mode Functions
    void _parse_msp_char(uint8_t c);
    void _process_msp_message();
    void _send_msp_response(uint8_t cmd, uint8_t *payload, uint8_t size);

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
};
