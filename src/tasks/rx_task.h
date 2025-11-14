/**
 * @file rx_task.h
 * @brief Defines the RxTask class for handling RC receiver input.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include "../scheduler/task_base.h"
#include "../protocols/rx_protocol.h"
#include "../settings_manager.h"
#include "../config/rx_config.h"

class RxTask : public TaskBase
{
public:
    RxTask(const char *name, uint32_t stackSize, UBaseType_t priority, BaseType_t coreID, uint32_t task_delay_ms, SettingsManager *settings_manager);
    ~RxTask();
    void setup() override;
    void run() override;

    int16_t getChannel(const uint8_t channel_nr) const;

private:
    RxProtocol *_rx_protocol;
    SettingsManager *_settings_manager;
};