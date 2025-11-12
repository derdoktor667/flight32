/**
 * @file rx_ppm_protocol.h
 * @brief PPM implementation of the RxProtocol interface.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#pragma once

#include "rx_protocol.h"
#include <Arduino.h>
#include "config.h" // Include config.h for PPM constants

// PPM configuration constants are now defined in config.h

class RxPpmProtocol : public RxProtocol
{
public:
    RxPpmProtocol();
    void begin(uint8_t uart_num, uint8_t rx_pin, uint8_t tx_pin, uint32_t baud_rate) override;
    bool readChannels() override;
    int16_t getChannelValue(uint8_t channel) override;
    bool isConnected() override;

    // Static ISR handler
    static void IRAM_ATTR ppmISR();

private:
    uint8_t _ppm_pin;
    volatile uint16_t _channel_values[PPM_MAX_CHANNELS];
    volatile uint32_t _last_pulse_time;
    volatile uint8_t _current_channel;
    volatile bool _new_data_available;
    volatile bool _is_connected;

    static RxPpmProtocol *_instance;
};
