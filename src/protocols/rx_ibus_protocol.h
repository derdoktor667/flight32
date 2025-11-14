/**
 * @file rx_ibus_protocol.h
 * @brief IBUS implementation of the RxProtocol interface.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#pragma once

#include "rx_protocol.h"
#include "../config/rx_config.h"
#include "../../libraries/FlyskyIBUS/FlyskyIBUS.h"

class RxIbusProtocol : public RxProtocol
{
public:
    RxIbusProtocol();
    void begin(uint8_t rx_pin) override;
    bool updateChannels() override;
    int16_t getChannelValue(uint8_t channel) override;
    bool isConnected() override;

private:
    FlyskyIBUS _ibus;
};
