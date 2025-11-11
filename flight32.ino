/**
 * @file flight32.ino
 * @brief Main entry point for the Flight32 firmware.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#include <Arduino.h>
#include "src/flight_controller.h"

FlightController *fc;

void setup()
{
    fc = new FlightController();
    fc->setup();
}

void loop()
{
    // ...what kind of sorcery is this? 
}
