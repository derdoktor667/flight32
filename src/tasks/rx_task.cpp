/**
 * @file rx_task.cpp
 * @brief Implements the RxTask for handling RC receiver input.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#include "rx_task.h"
#include "../config/rx_config.h"
#include "../com_manager.h"
#include "../protocols/rx_ibus_protocol.h"
#include "../protocols/rx_ppm_protocol.h"

RxTask::RxTask(const char *name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID, uint32_t task_delay_ms, SettingsManager *settings_manager)
    : TaskBase(name, stackSize, priority, coreID, task_delay_ms), _rx_protocol(nullptr), _settings_manager(settings_manager) {}

void RxTask::setup()
{
    if (!_settings_manager)
    {
        com_send_log(ComMessageType::LOG_ERROR, "RxTask: SettingsManager not provided!");
        return;
    }

    RcProtocolType protocol_type = (RcProtocolType)_settings_manager->getSettingValue(NVS_KEY_RC_PROTOCOL_TYPE).toInt();

    switch (protocol_type)
    {
    case RcProtocolType::IBUS:
        _rx_protocol = std::make_unique<RxIbusProtocol>();
        com_send_log(ComMessageType::LOG_INFO, "RxTask: Selected IBUS protocol.");
        break;
    case RcProtocolType::PPM:
        _rx_protocol = std::make_unique<RxPpmProtocol>();
        com_send_log(ComMessageType::LOG_INFO, "RxTask: Selected PPM protocol.");
        break;
    case RcProtocolType::NONE:
    default:
        com_send_log(ComMessageType::LOG_ERROR, "RxTask: No valid RX protocol selected in settings!");
        break;
    }

    if (_rx_protocol)
    {
        uint8_t rx_pin_to_use = 0;

        switch (protocol_type)
        {
        case RcProtocolType::IBUS:
            rx_pin_to_use = IBUS_RX_PIN;
            com_send_log(ComMessageType::LOG_INFO, "RxTask: Initializing IBUS on RX:%d", rx_pin_to_use);
            break;
        case RcProtocolType::PPM:
            rx_pin_to_use = _settings_manager->getSettingValue(NVS_KEY_RX_PIN).toInt();
            com_send_log(ComMessageType::LOG_INFO, "RxTask: Initializing PPM on pin %d", rx_pin_to_use);
            break;
        default:
            com_send_log(ComMessageType::LOG_ERROR, "RxTask: Unknown protocol type for pin configuration!");
            return;
        }

        _rx_protocol->begin(rx_pin_to_use);
    }
    else
    {
        com_send_log(ComMessageType::LOG_ERROR, "RxTask: Failed to initialize RX protocol!");
    }
}

void RxTask::run()
{
    if (_rx_protocol && _rx_protocol->updateChannels())
    {
    }
}

int16_t RxTask::getChannel(const uint8_t channel_nr) const
{
    if (_rx_protocol)
    {
        return _rx_protocol->getChannelValue(channel_nr);
    }
    return INVALID_CHANNEL_VALUE;
}
