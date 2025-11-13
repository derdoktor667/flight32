/**
 * @file scheduler.cpp
 * @brief Implements the Scheduler class for managing FreeRTOS tasks.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#include "scheduler.h"

Scheduler::Scheduler() : _task_count(0)
{
    for (uint8_t i = 0; i < MAX_TASKS; i++)
    {
        _tasks[i] = nullptr;
    }
}

bool Scheduler::addTask(TaskBase *task)
{
    if (_task_count < MAX_TASKS)
    {
        _tasks[_task_count++] = task;
        return true;
    }
    return false;
}

void Scheduler::start()
{
    for (uint8_t i = 0; i < _task_count; i++)
    {
        TaskHandle_t handle;
        xTaskCreatePinnedToCore(
            _task_trampoline,
            _tasks[i]->getName(),
            _tasks[i]->getStackSize(),
            _tasks[i],
            _tasks[i]->getPriority(),
            &handle,
            _tasks[i]->getCoreID());
                    _tasks[i]->setHandle(handle);    }
}

void Scheduler::_task_trampoline(void *pvParameters)
{
    TaskBase *task = static_cast<TaskBase *>(pvParameters);
    task->taskRunner();
}
