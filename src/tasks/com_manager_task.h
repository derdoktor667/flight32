/**
 * @file com_manager_task.h
 * @brief Definition of the ComManagerTask class, a FreeRTOS task for handling communication.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include "../scheduler/task_base.h"
#include "../com_manager.h" // For com_task (global function)
#include "../config/com_manager_config.h"

class ComManagerTask : public TaskBase
{
public:
    ComManagerTask(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms);

    void setup() override;
    void run() override;

private:
    // Any members needed by the communication task can be added here
};
