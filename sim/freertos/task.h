// Host simulation shim -- NOT compiled into the firmware.
// Delays are virtual: they advance a fake tick counter instead of sleeping, so
// a long run finishes immediately. Define SIM_REAL_TIME_DELAYS to sleep for
// real and watch the loop at its true rate.
#ifndef SIM_FREERTOS_TASK_H
#define SIM_FREERTOS_TASK_H

#include "freertos/FreeRTOS.h"

#ifdef SIM_REAL_TIME_DELAYS
#include <chrono>
#include <thread>
#endif

inline TickType_t & sim_tick_counter() {
    static TickType_t ticks = 0;
    return ticks;
}

inline TickType_t xTaskGetTickCount() { return sim_tick_counter(); }

inline void vTaskDelay(TickType_t ticks) {
    sim_tick_counter() += ticks;
#ifdef SIM_REAL_TIME_DELAYS
    std::this_thread::sleep_for(std::chrono::milliseconds(ticks));
#endif
}

inline void vTaskDelayUntil(TickType_t * prev_wake, TickType_t period) {
    const TickType_t deadline = *prev_wake + period;
    if (sim_tick_counter() < deadline) {
        vTaskDelay(deadline - sim_tick_counter());
    }
    *prev_wake = deadline;
}

#endif
