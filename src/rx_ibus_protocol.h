/**
 * @file rx_ibus_protocol.h
 * @brief IBUS implementation of the RxProtocol interface.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#pragma once

#include "rx_protocol.h"
#include <FlyskyIBUS.h>

class RxIbusProtocol : public RxProtocol
{
public:
    RxIbusProtocol();
    void begin(uint8_t uart_num, uint8_t rx_pin, uint8_t tx_pin, uint32_t baud_rate) override;
    bool readChannels() override;
    int16_t getChannelValue(uint8_t channel) override;
    bool isConnected() override;

private:
    FlyskyIBUS _ibus;
};
