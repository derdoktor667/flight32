#pragma once

#include <Arduino.h>
#include "../scheduler/task_base.h"
#include <FlyskyIBUS.h>

class IbusTask : public TaskBase
{
public:
    IbusTask(const char *name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID, uint32_t task_delay_ms);
    void setup() override;
    void run() override;

    // Getter for individual IBUS channel data
    uint16_t getChannel(const uint8_t channel_nr) const { return _ibus_receiver.getChannel(channel_nr); }
    uint8_t getChannelCount() const { return _ibus_receiver.getChannelCount(); }

private:
    FlyskyIBUS _ibus_receiver;
};