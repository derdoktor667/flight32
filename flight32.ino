/**
 * @file flight32.ino
 * @brief The Flight32 firmware.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#include <Arduino.h>
#include "src/flight_controller.h"
#include "src/config/serial_config.h"

//
FlightController *fc;

//
void setup()
{
    fc = new FlightController();
    fc->setup();
}

//
void loop()
{
    // ...what kind of sorcery is this?
}
