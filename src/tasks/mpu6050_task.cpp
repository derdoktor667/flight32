#include "mpu6050_task.h"
#include "../com_manager.h"

Mpu6050Task::Mpu6050Task(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms, ESP32_MPU6050 &mpu)
    : TaskBase(name, stack_size, priority, core_id, task_delay_ms), _mpu(mpu) {}

void Mpu6050Task::setup()
{
    // Nothing to do here, MPU6050 is initialized in FlightController
}

void Mpu6050Task::run()
{
    _mpu.update();
}
