/**
 * @file task_base.cpp
 * @brief Implements the base class for FreeRTOS tasks, providing common functionality.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#include "task_base.h"

void TaskBase::taskRunner()
{
    while (true)
    {
        uint32_t start_us = micros();

        run();

        uint32_t end_us = micros();

        _loop_time_us = end_us - start_us;
        if (_loop_time_us > _max_loop_time_us)
        {
            _max_loop_time_us = _loop_time_us;
        }
        _total_loop_time_us += _loop_time_us;
        _loop_count++;
        _avg_loop_time_us = _total_loop_time_us / _loop_count;

        vTaskDelay(pdMS_TO_TICKS(_task_delay_ms));
    }
}
