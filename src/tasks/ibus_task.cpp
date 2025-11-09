/**
 * @file ibus_task.cpp
 * @brief Implements the IBUS receiver task for Flysky IBUS protocol.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#include "ibus_task.h"
#include "../config.h"
#include "../com_manager.h"

IbusTask::IbusTask(const char *name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID, uint32_t task_delay_ms)
    : TaskBase(name, stackSize, priority, coreID, task_delay_ms), _ibus_receiver() {}

void IbusTask::setup()
{
    _ibus_receiver.begin();
    com_send_log(LOG_INFO, "IBusTask: Initialized on Serial2 at %d baud.", IBUS_BAUD_RATE);
}

void IbusTask::run()
{
    // The FlyskyIBUS library handles its loop internally via interrupts.
    // No explicit loop() call is needed here.
}
