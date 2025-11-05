#pragma once

#include "../scheduler/task_base.h"
#include <ESP32_MPU6050.h>

class Mpu6050Task : public TaskBase
{
public:
    Mpu6050Task(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms, ESP32_MPU6050 &mpu6050_sensor);
    void setup() override;
    void run() override;

private:
    ESP32_MPU6050 &_mpu6050_sensor;
};
