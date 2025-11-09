/**
 * @file rx_ibus_protocol.cpp
 * @brief IBUS implementation of the RxProtocol interface.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#include "rx_ibus_protocol.h"
#include "config.h"
#include "com_manager.h"

RxIbusProtocol::RxIbusProtocol() : _ibus(Serial2, IBUS_RX_PIN) {}

void RxIbusProtocol::begin(uint8_t uart_num, uint8_t rx_pin, uint8_t tx_pin, uint32_t baud_rate)
{
    _ibus.begin();
    // Parameters (uart_num, rx_pin, tx_pin, baud_rate) are ignored as FlyskyIBUS is configured in the constructor.

}

bool RxIbusProtocol::readChannels()
{
    // FlyskyIBUS updates channels asynchronously via ISR.
    // This method just needs to be called to allow the task to run.
    return true;
}

int16_t RxIbusProtocol::getChannelValue(uint8_t channel)
{
    return _ibus.getChannel(channel);
}

bool RxIbusProtocol::isConnected()
{
    return !_ibus.hasFailsafe();
}
