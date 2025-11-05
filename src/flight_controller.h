#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <ESP32_MPU6050.h>
#include "tasks/mpu6050_task.h"

class Scheduler;

class FlightController
{
public:
    FlightController();
    void setup();
    Scheduler *getScheduler() const { return _scheduler; }

private:
    Scheduler *_scheduler;
    QueueHandle_t _com_queue;
    ESP32_MPU6050 _mpu6050;
};