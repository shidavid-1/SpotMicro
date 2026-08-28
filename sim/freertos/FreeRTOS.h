// Host simulation shim -- NOT compiled into the firmware.
#ifndef SIM_FREERTOS_H
#define SIM_FREERTOS_H

#include <cstdint>

typedef uint32_t TickType_t;
typedef int      BaseType_t;

#define configTICK_RATE_HZ   1000
#define portTICK_PERIOD_MS   1
#define pdMS_TO_TICKS(ms)    ((TickType_t)(ms))
#define pdTRUE               1
#define pdFALSE              0

#endif
