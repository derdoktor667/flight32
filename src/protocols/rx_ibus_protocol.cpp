/**
 * @file rx_ibus_protocol.cpp
 * @brief IBUS implementation of the RxProtocol interface.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#include "rx_ibus_protocol.h"
#include "../config/rx_config.h"
#include "../com_manager.h"
#include "FlyskyIBUS.h"

RxIbusProtocol::RxIbusProtocol() : _ibus(IBUS_SERIAL_PORT, IBUS_RX_PIN) {}

void RxIbusProtocol::begin(uint8_t rx_pin)
{
    _ibus.begin();
}

bool RxIbusProtocol::updateChannels()
{
    _ibus.read();
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
