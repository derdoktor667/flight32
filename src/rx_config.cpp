/**
 * @file rx_config.cpp
 * @brief Defines the hardware serial port for the IBUS receiver.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#include "config/rx_config.h"
#include <HardwareSerial.h>

HardwareSerial &IBUS_SERIAL_PORT = Serial2;