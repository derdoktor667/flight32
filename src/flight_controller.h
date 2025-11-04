#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class Scheduler;

class FlightController {
public:
    FlightController();
    void setup();

private:
    Scheduler* _scheduler;
    QueueHandle_t _com_queue;
};