/**
 * @file rx_ppm_protocol.h
 * @brief PPM implementation of the RxProtocol interface.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#pragma once

#include "rx_protocol.h"
#include <Arduino.h> // For pin functions

// Configuration for PPM
static constexpr uint8_t PPM_MAX_CHANNELS = 8; // Max channels to read
static constexpr uint16_t PPM_MIN_PULSE_WIDTH = 700; // us
static constexpr uint16_t PPM_MAX_PULSE_WIDTH = 2300; // us
static constexpr uint16_t PPM_FRAME_LENGTH_MIN = 3000; // us (min time between frames)

class RxPpmProtocol : public RxProtocol
{
public:
    RxPpmProtocol();
    void begin(uint8_t uart_num, uint8_t rx_pin, uint8_t tx_pin, uint32_t baud_rate) override; // uart_num, tx_pin, baud_rate are ignored for PPM
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

    // Private static instance for ISR
    static RxPpmProtocol* _instance;
};
