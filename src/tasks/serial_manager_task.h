/**
 * @file serial_manager_task.h
 * @brief Manages serial communication, handling both CLI terminal and MSP protocol with automatic switching.
 * @author Your Name
 * @date 2025-11-12
 * @license MIT
 */

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

// Terminal Command Categories (re-defined here as they are no longer in terminal_task.h)
enum class CommandCategory
{
    SYSTEM,
    IMU,
    RX,
    MOTOR,
    PID,
    SETTINGS,
    UNKNOWN
};

// Forward declaration of SerialManagerTask for Command struct
class SerialManagerTask;

// Command struct definition (needs to be outside the class for static member array)
struct Command
{
    const char *name;
    void (SerialManagerTask::*handler)(String &);
    const char *help;
    CommandCategory category;
};

struct CategoryInfo {
    CommandCategory category;
    const char* prefix;
    const char* description;
};

struct ChannelMapping {
    const char* name;
    const char* key;
};

// MSP Commands (re-defined here as they are no longer in msp_task.h)
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

    // --- Serial Protocol State ---
    enum class SerialMode {
        TERMINAL,
        MSP
    };
    SerialMode _current_mode = SerialMode::TERMINAL;
    unsigned long _last_msp_activity_ms = 0;
    static constexpr unsigned long MSP_TIMEOUT_MS = 2000; // Switch back to terminal after 2 seconds of no MSP activity

    // --- Terminal Mode Variables ---
    String _terminal_input_buffer;
    bool _terminal_should_quit = false;

    // --- MSP Mode Variables ---
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
    uint8_t _msp_payload_buffer[128];
    uint8_t _msp_payload_index = 0;

    // --- Common Helpers ---
    bool _check_imu_task_available();
    bool _check_rx_task_available();
    bool _check_motor_task_available();
    bool _check_pid_task_available();
    const char *_format_bytes(uint32_t bytes);
    char _byte_buffer[BYTE_BUFFER_SIZE]; // For _format_bytes

    // --- Terminal Mode Functions ---
    void _handle_terminal_input(char incoming_char);
    void _parse_terminal_command(String &command);
    const char *_get_category_string(CommandCategory category);
    CommandCategory _get_category_from_string(String &category_str);
    CommandCategory _get_setting_category(const char *display_key);
    const char *_get_motor_name(uint8_t motor_id);
    int8_t _get_motor_id(String &motor_name);
    static const char *_get_task_state_string(eTaskState state);

    // Terminal Command Handlers (from old TerminalTask)
    void _handle_help(String &args);
    void _handle_status(String &args);
    void _handle_tasks(String &args);
    void _handle_mem(String &args);
    void _handle_reboot(String &args);
    void _handle_quit(String &args);
    void _handle_imu_data(String &args);
    void _handle_imu_calibrate(String &args);
    void _handle_imu_lpf_bandwidth(String &args);
    void _handle_rx_data(String &args);
    void _handle_rx_status(String &args);
    void _handle_rx_protocol(String &args);
    void _handle_rx_value_single(String &args);
    void _handle_rx_value_all(String &args);
    void _handle_rx_channel_mapping(String &args);
    void _handle_motor_throttle(String &args);
    void _handle_motor_test(String &args);
    void _handle_motor_stop(String &args);
    void _handle_pid_get(String &args);
    void _handle_pid_set(String &args);
    void _handle_pid_reset_defaults(String &args);
    void _handle_get_setting(String &args);
    void _handle_set_setting(String &args);
    void _handle_save_settings(String &args);
    void _handle_factory_reset(String &args);
    void _handle_list_settings(String &args);
    void _handle_dump_settings(String &args);

    static const Command _terminal_commands[];
    static const int _num_terminal_commands;
    static const CategoryInfo _terminal_category_info[];
    static const int _num_terminal_categories;
    static const ChannelMapping _terminal_channel_map[];
    static const int _num_terminal_channel_mappings;

    // --- MSP Mode Functions ---
    void _parse_msp_char(uint8_t c);
    void _process_msp_message();
    void _send_msp_response(uint8_t cmd, uint8_t *payload, uint8_t size);

    // MSP Command Handlers (from old MspTask)
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
