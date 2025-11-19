/**
 * @file com_manager_task.cpp
 * @brief Implementation of the ComManagerTask class.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#include "com_manager_task.h"
#include "../com_manager.h" // For com_task (global function)
#include "../config/com_manager_config.h"

ComManagerTask::ComManagerTask(const char *name, uint32_t stack_size, UBaseType_t priority, BaseType_t core_id, uint32_t task_delay_ms)
    : TaskBase(name, stack_size, priority, core_id, task_delay_ms)
{
}

void ComManagerTask::setup()
{
    // No specific setup needed for the com_task itself within this class, as it's a global function
    // and its setup is handled by its internal logic or com_manager.cpp.
    // However, if ComManager were to be an object and need initialization, it would go here.
}

void ComManagerTask::run()
{
    // The original com_task is a global FreeRTOS task function.
    // We will call it directly here.
    com_task(nullptr); 
    // Note: com_task contains an infinite loop, so this run() method
    // will effectively never return. If com_task was refactored to be
    // part of an object, its loop would be internal to that object's
    // run method.
}
