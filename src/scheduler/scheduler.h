/**
 * @file scheduler.h
 * @brief Defines the Scheduler class for managing FreeRTOS tasks.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include "task_base.h"
#include "scheduler_config.h"

class Scheduler
{
public:
    Scheduler();
    bool addTask(TaskBase *task);
    void start();

    uint8_t getTaskCount() const { return _task_count; }
    TaskBase *getTask(uint8_t index) const { return _tasks[index]; }

private:
    static void _task_trampoline(void *pvParameters);
    TaskBase *_tasks[MAX_TASKS];
    uint8_t _task_count;
};
