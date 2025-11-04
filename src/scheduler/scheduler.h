#pragma once

#include <Arduino.h>
#include "task.h"
#include "../config.h"

class Scheduler {
public:
    Scheduler();
    bool addTask(TaskBase* task);
    void start();

private:
    static void taskRunner(void* pvParameters);
    TaskBase* tasks[MAX_TASKS];
    uint8_t taskCount;
};
