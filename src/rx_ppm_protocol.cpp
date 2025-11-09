/**
 * @file rx_ppm_protocol.cpp
 * @brief PPM implementation of the RxProtocol interface.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#include "rx_ppm_protocol.h"
#include "com_manager.h" // For logging

// Initialize static member
RxPpmProtocol* RxPpmProtocol::_instance = nullptr;

RxPpmProtocol::RxPpmProtocol()
    : _ppm_pin(0), _last_pulse_time(0), _current_channel(0), _new_data_available(false), _is_connected(false)
{
    for (uint8_t i = 0; i < PPM_MAX_CHANNELS; ++i)
    {
        _channel_values[i] = 0; // Initialize with a default value
    }
    _instance = this; // Set the static instance
}

void RxPpmProtocol::begin(uint8_t uart_num, uint8_t rx_pin, uint8_t tx_pin, uint32_t baud_rate)
{
    _ppm_pin = rx_pin; // For PPM, rx_pin is the signal pin
    pinMode(_ppm_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(_ppm_pin), RxPpmProtocol::ppmISR, CHANGE);
    com_send_log(LOG_INFO, "PPM Protocol: Initializing on pin %d", _ppm_pin);
}

bool RxPpmProtocol::readChannels()
{
    if (_new_data_available)
    {
        _new_data_available = false;
        _is_connected = true; // Assume connected if new data is available
        return true;
    }
    _is_connected = (micros() - _last_pulse_time < (PPM_FRAME_LENGTH_MIN * PPM_CONNECTION_TIMEOUT_FACTOR)); // Check for recent pulses
    return false;
}

int16_t RxPpmProtocol::getChannelValue(uint8_t channel)
{
    if (channel < PPM_MAX_CHANNELS)
    {
        return _channel_values[channel];
    }
    return 0; // Invalid channel
}

bool RxPpmProtocol::isConnected()
{
    return _is_connected;
}

// Static ISR handler
void IRAM_ATTR RxPpmProtocol::ppmISR()
{
    if (_instance == nullptr) return;

    uint32_t current_time = micros();
    uint32_t pulse_width = current_time - _instance->_last_pulse_time;
    _instance->_last_pulse_time = current_time;

    if (pulse_width > PPM_FRAME_LENGTH_MIN) // Start of a new PPM frame (sync pulse)
    {
        _instance->_current_channel = 0;
    }
    else if (_instance->_current_channel < PPM_MAX_CHANNELS) // Channel pulse
    {
        if (pulse_width >= PPM_MIN_PULSE_WIDTH && pulse_width <= PPM_MAX_PULSE_WIDTH)
        {
            _instance->_channel_values[_instance->_current_channel] = pulse_width;
            _instance->_current_channel++;
            if (_instance->_current_channel >= PPM_MAX_CHANNELS) {
                _instance->_new_data_available = true;
            }
        }
    }
}
