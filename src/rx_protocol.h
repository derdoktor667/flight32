/**
 * @file rx_protocol.h
 * @brief Abstract base class for various RC receiver protocols.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#pragma once

#include <cstdint>

class RxProtocol
{
public:
    virtual ~RxProtocol() = default;

    virtual void begin(uint8_t rx_pin) = 0;

    virtual bool updateChannels() = 0;

    virtual int16_t getChannelValue(uint8_t channel) = 0;

    virtual bool isConnected() = 0;
};
