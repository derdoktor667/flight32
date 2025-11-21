/**
 * @file flight32.ino
 * @brief The Flight32 firmware.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#include <Arduino.h>
#include "src/flight_controller.h"

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


// #include <Arduino.h>
// #include "src/flight_controller.h" // Explicitly pointing to src/

// FlightController flight_controller;

// void setup() {
//   flight_controller.setup();
// }

// void loop() {
//   // Main loop is handled by the FreeRTOS scheduler
//   vTaskDelete(NULL); // Delete the loop task, as it's not needed
// }