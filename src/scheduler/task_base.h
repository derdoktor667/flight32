/**
 * @file task_base.h
 * @brief Defines the abstract base class for FreeRTOS tasks in Flight32.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"

class TaskBase
{
public:
    virtual void setup() = 0;
    virtual void run() = 0;

    const char *getName() const { return _task_name; }
    uint32_t getStackSize() const { return _stack_size; }
    UBaseType_t getPriority() const { return _priority; }
    BaseType_t getCoreID() const { return _core_id; }
    TaskHandle_t getHandle() const { return _handle; }
    uint32_t getLoopTime() const { return _loop_time_us; }
    uint32_t getAvgLoopTime() const { return _avg_loop_time_us; }
    uint32_t getMaxLoopTime() const { return _max_loop_time_us; }
    uint32_t getTaskDelayMs() const { return _task_delay_ms; }

    void taskRunner();

    void suspend();
    void resume();

    void setHandle(TaskHandle_t handle) { _handle = handle; }

private:
    TaskHandle_t _handle = nullptr;

protected:
    TaskBase(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms)
        : _task_name(name),
          _stack_size(stack_size),
          _priority(priority),
          _core_id(core_id),
          _task_delay_ms(task_delay_ms) {}

private:
    const char *_task_name;
    uint32_t _stack_size;
    UBaseType_t _priority;
    BaseType_t _core_id;
    uint32_t _task_delay_ms;

    uint32_t _loop_time_us = 0;
    uint32_t _avg_loop_time_us = 0;
    uint32_t _max_loop_time_us = 0;
    uint64_t _total_loop_time_us = 0;
    uint32_t _loop_count = 0;
};