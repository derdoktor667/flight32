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
    // Log that the parameters are ignored as IBUS is configured in the constructor
    com_send_log(LOG_INFO, "RxIbusProtocol: begin() parameters (UART:%d, RX:%d, TX:%d, Baud:%d) ignored. IBUS configured in constructor.", uart_num, rx_pin, tx_pin, baud_rate);
}

bool RxIbusProtocol::readChannels()
{
    // A more robust check might involve checking _ibus.getReadTime() against current time.
    return true;
}

int16_t RxIbusProtocol::getChannelValue(uint8_t channel)
{
    return _ibus.getChannel(channel);
}

bool RxIbusProtocol::isConnected()
{
    // More robust connection checking might involve a timeout mechanism or checking specific channel values (e.g., failsafe).
    return true; // Placeholder, needs more robust implementation if required.
}
