#pragma once

#include <Arduino.h>

class TaskBase {
public:
    virtual void setup() = 0;
    virtual void run() = 0;
    const char* getName() { return taskName; }
    uint32_t getStackSize() { return stackSize; }
    UBaseType_t getPriority() { return priority; }
    BaseType_t getCoreID() { return coreID; }

protected:
    TaskBase(const char* name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID)
        : taskName(name), stackSize(stackSize), priority(priority), coreID(coreID) {}

private:
    const char* taskName;
    uint32_t stackSize;
    UBaseType_t priority;
    BaseType_t coreID;
};
