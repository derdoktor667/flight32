#pragma once

#include <Arduino.h>
#include "task_base.h"
#include "../config.h"

class Scheduler
{
public:
    Scheduler();
    bool addTask(TaskBase *task);
    void start();

    // --- Getters for task introspection ---
    uint8_t getTaskCount() const { return _task_count; }
    TaskBase *getTask(uint8_t index) const { return _tasks[index]; }

private:
    static void _task_trampoline(void *pvParameters);
    TaskBase *_tasks[MAX_TASKS];
    uint8_t _task_count;
};
