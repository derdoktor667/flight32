/**
 * @file rx_ppm_protocol.cpp
 * @brief Implements the PPM (Pulse Position Modulation) protocol for RC receiver input.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#include "rx_ppm_protocol.h"
#include "com_manager.h"
#include "config.h"

RxPpmProtocol *RxPpmProtocol::_instance = nullptr;

RxPpmProtocol::RxPpmProtocol()
    : _ppm_pin(DEFAULT_RX_PIN), _last_pulse_time(0), _current_channel(0), _new_data_available(false), _is_connected(false)
{
    for (uint8_t i = 0; i < PPM_MAX_CHANNELS; ++i)
    {
        _channel_values[i] = RC_CHANNEL_CENTER;
    }
    _instance = this;
}

void RxPpmProtocol::begin(uint8_t rx_pin)
{
    _ppm_pin = rx_pin;
    pinMode(_ppm_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(_ppm_pin), RxPpmProtocol::ppmISR, CHANGE);
    com_send_log(LOG_INFO, "PPM Protocol: Initializing on pin %d", _ppm_pin);
}

bool RxPpmProtocol::updateChannels()
{
    if (_new_data_available)
    {
        _new_data_available = false;
        _is_connected = true;
        return true;
    }
    _is_connected = (micros() - _last_pulse_time < (PPM_FRAME_LENGTH_MIN * PPM_CONNECTION_TIMEOUT_FACTOR));
    return false;
}

int16_t RxPpmProtocol::getChannelValue(uint8_t channel)
{
    if (channel < PPM_MAX_CHANNELS)
    {
        return _channel_values[channel];
    }
    return INVALID_CHANNEL_VALUE;
}

bool RxPpmProtocol::isConnected()
{
    return _is_connected;
}

void IRAM_ATTR RxPpmProtocol::ppmISR()
{
    if (_instance == nullptr)
        return;

    uint32_t current_time = micros();
    uint32_t pulse_width = current_time - _instance->_last_pulse_time;
    _instance->_last_pulse_time = current_time;

    if (pulse_width > PPM_FRAME_LENGTH_MIN)
    {
        _instance->_current_channel = 0;
    }
    else if (_instance->_current_channel < PPM_MAX_CHANNELS)
    {
        if (pulse_width >= PPM_MIN_PULSE_WIDTH && pulse_width <= PPM_MAX_PULSE_WIDTH)
        {
            _instance->_channel_values[_instance->_current_channel] = pulse_width;
            _instance->_current_channel++;
            if (_instance->_current_channel >= PPM_MAX_CHANNELS)
            {
                _instance->_new_data_available = true;
            }
        }
    }
}
