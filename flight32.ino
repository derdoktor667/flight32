
//
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
