#include "flight_controller.h"
#include "scheduler/scheduler.h"
#include "terminal/terminal.h"
#include "com_manager.h"
#include "config.h"
#include "firmware.h"
#include <Arduino.h>

// The global queue handle, declared as extern in com_manager.h
QueueHandle_t com_queue;

FlightController::FlightController() : _scheduler(nullptr) {}

void FlightController::setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    
    // Create the IO message queue and assign it to the global handle
    com_queue = xQueueCreate(COM_QUEUE_LENGTH, sizeof(com_message_t));

    // Create the IO manager task
    xTaskCreate(
        com_task,          // Task function
        "com_task",        // Task name
        2048,             // Stack size
        NULL,             // Parameter
        2,                // Priority
        NULL              // Task handle
    );

    com_send_log(LOG_INFO, "Flight32 Firmware v%s starting...", get_firmware_version());

    _scheduler = new Scheduler();

    TerminalTask* terminalTask = new TerminalTask(
        "terminal_task", 
        TERMINAL_TASK_STACK_SIZE, 
        (UBaseType_t)TERMINAL_TASK_PRIORITY, 
        (BaseType_t)TERMINAL_TASK_CORE_ID
    );
    _scheduler->addTask(terminalTask);
    _scheduler->start();
}