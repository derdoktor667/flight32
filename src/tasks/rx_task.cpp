/**
 * @file rx_task.cpp
 * @brief Implements the generic RX receiver task for various protocols.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#include "rx_task.h"
#include "../config.h"
#include "../com_manager.h"

RxTask::RxTask(const char *name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID, uint32_t task_delay_ms, RxProtocol *rx_protocol)
    : TaskBase(name, stackSize, priority, coreID, task_delay_ms), _rx_protocol(rx_protocol) {}

void RxTask::setup()
{
    if (_rx_protocol)
    {
        _rx_protocol->begin(IBUS_UART_NUM, IBUS_RX_PIN, IBUS_TX_PIN, IBUS_BAUD_RATE);
        com_send_log(LOG_INFO, "RxTask: Initialized RX protocol.");
    }
    else
    {
        com_send_log(LOG_ERROR, "RxTask: No RX protocol provided!");
    }
}

void RxTask::run()
{
    if (_rx_protocol)
    {
        _rx_protocol->readChannels();
    }
}

int16_t RxTask::getChannel(const uint8_t channel_nr) const
{
    if (_rx_protocol)
    {
        return _rx_protocol->getChannelValue(channel_nr);
    }
    return 0; // Return 0 or a default value if no protocol is set
}
