// Host simulation shim -- NOT compiled into the firmware.
// Routes ESP_LOGx to stdout so gait/transform output can be inspected on a PC.
#ifndef SIM_ESP_LOG_H
#define SIM_ESP_LOG_H

#include <cstdio>

#define ESP_LOGE(tag, fmt, ...) std::printf("E (%s) " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) std::printf("W (%s) " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) std::printf("I (%s) " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) std::printf("D (%s) " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGV(tag, fmt, ...) std::printf("V (%s) " fmt "\n", tag, ##__VA_ARGS__)

#endif
