/**
 * @file task_base.cpp
 * @brief Implements the taskRunner method for the TaskBase class.
 * @author Wastl Kraus
 * @date 2025-11-09
 * @license MIT
 */

#include "task_base.h"

void TaskBase::taskRunner()
{
    while (true)
    {
        uint32_t start_us = micros();

        run(); // Execute the derived class's work

        uint32_t end_us = micros();

        // --- Update loop metrics ---
        _loop_time_us = end_us - start_us;
        if (_loop_time_us > _max_loop_time_us)
        {
            _max_loop_time_us = _loop_time_us;
        }
        _total_loop_time_us += _loop_time_us;
        _loop_count++;
        _avg_loop_time_us = _total_loop_time_us / _loop_count;

        // Yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(_task_delay_ms));
    }
}
