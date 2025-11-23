/**
 * @file msp_processor.h
 * @brief Processes MSP commands.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */
#pragma once

#include "msp_protocol.h"
#include <cstdint>
#include <memory> // For std::unique_ptr if used for other members

// Forward declarations to avoid circular dependencies
class FlightController;
class PidTask;
class ImuTask;
class RxTask;
class MotorTask;
class SettingsManager;

class MspProcessor {
public:
    MspProcessor(FlightController* flightController, PidTask* pidTask, ImuTask* imuTask, RxTask* rxTask, MotorTask* motorTask, SettingsManager* settingsManager);

    void process_msp_command(uint8_t command, const uint8_t* payload, uint8_t payload_size, uint8_t protocol_version);

private:
    FlightController* _flightController;
    PidTask* _pidTask;
    ImuTask* _imuTask;
    RxTask* _rxTask;
    MotorTask* _motorTask;
    SettingsManager* _settingsManager;

private:
    void send_msp_response(uint8_t command, const uint8_t* payload, uint8_t payload_size, uint8_t protocol_version);
    void send_msp_error(uint8_t command, uint8_t protocol_version);

    void process_api_version_command(uint8_t protocol_version);
    void process_fc_variant_command(uint8_t protocol_version);
    void process_fc_version_command(uint8_t protocol_version);
    void process_board_info_command(uint8_t protocol_version);
    void process_build_info_command(uint8_t protocol_version);
    void process_reboot_command(uint8_t protocol_version);
    void process_mem_stats_command(uint8_t protocol_version);
    void process_ident_command(uint8_t protocol_version);
    void process_get_setting_command(const uint8_t* payload, uint8_t payload_size, uint8_t protocol_version);
    void process_set_setting_command(const uint8_t* payload, uint8_t payload_size, uint8_t protocol_version);
    void process_pid_command(uint8_t protocol_version);
    void process_status_command(uint8_t protocol_version);
    void process_raw_imu_command(uint8_t protocol_version);
    void process_motor_command(uint8_t protocol_version);
    void process_rc_command(uint8_t protocol_version);
    void process_attitude_command(uint8_t protocol_version);
    void process_box_command(uint8_t protocol_version);
    void process_boxnames_command(uint8_t protocol_version);
    void process_mode_ranges_command(uint8_t protocol_version);
    void process_motor_config_command(uint8_t protocol_version);
    void process_uid_command(uint8_t protocol_version);
    void process_sensor_status_command(uint8_t protocol_version);
    void process_eeprom_write_command(uint8_t protocol_version);
    void process_reset_settings_command(uint8_t protocol_version);
    void process_set_pid_command(const uint8_t* payload, uint8_t payload_size, uint8_t protocol_version);
    void process_get_filter_config_command(uint8_t protocol_version);
    void process_set_filter_config_command(const uint8_t* payload, uint8_t payload_size, uint8_t protocol_version);
    void process_set_box_command(const uint8_t* payload, uint8_t payload_size, uint8_t protocol_version);

    // Added to handle currently unimplemented commands
    void process_name_command(uint8_t protocol_version);
    void process_feature_config_command(uint8_t protocol_version);
    void process_arming_config_command(uint8_t protocol_version);
    void process_rx_config_command(uint8_t protocol_version);
    void process_rx_map_command(uint8_t protocol_version);
    void process_failsafe_config_command(uint8_t protocol_version);
    void process_rc_tuning_command(uint8_t protocol_version);
    void process_advanced_config_command(uint8_t protocol_version);
    void process_filter_config_command(uint8_t protocol_version);
    void process_status_ex_command(uint8_t protocol_version);
    void process_altitude_command(uint8_t protocol_version);
    void process_analog_command(uint8_t protocol_version);
    void process_battery_state_command(uint8_t protocol_version);
    void process_osd_config_command(uint8_t protocol_version);
    void process_vtx_config_command(uint8_t protocol_version);
    void process_pidnames_command(uint8_t protocol_version);
    void process_boxids_command(uint8_t protocol_version);
};
