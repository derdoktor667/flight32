#include "scheduler.h"

Scheduler::Scheduler() : taskCount(0) {
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        tasks[i] = nullptr;
    }
}

bool Scheduler::addTask(TaskBase* task) {
    if (taskCount < MAX_TASKS) {
        tasks[taskCount++] = task;
        return true;
    }
    return false;
}

void Scheduler::start() {
    for (uint8_t i = 0; i < taskCount; i++) {
        xTaskCreatePinnedToCore(
            taskRunner,
            tasks[i]->getName(),
            tasks[i]->getStackSize(),
            tasks[i],
            tasks[i]->getPriority(),
            NULL,
            tasks[i]->getCoreID()
        );
    }
}

void Scheduler::taskRunner(void* pvParameters) {
    TaskBase* task = static_cast<TaskBase*>(pvParameters);
    task->setup();
    while (true) {
        task->run();
    }
}
