// Host simulation shim -- NOT compiled into the firmware.
// Enough of driver/i2c.h and the GPIO/esp_err types for the control-path
// sources to compile on a PC. Every bus operation is a no-op that reports
// success, so no hardware is touched.
#ifndef SIM_DRIVER_I2C_H
#define SIM_DRIVER_I2C_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdint>
#include <cstddef>

typedef int esp_err_t;
#define ESP_OK   0
#define ESP_FAIL -1

typedef enum { GPIO_NUM_NC = -1, GPIO_NUM_0 = 0, GPIO_NUM_11 = 11, GPIO_NUM_12 = 12 } gpio_num_t;
typedef enum { GPIO_PULLUP_DISABLE = 0, GPIO_PULLUP_ENABLE = 1 } gpio_pullup_t;
typedef enum { I2C_NUM_0 = 0, I2C_NUM_1 = 1, I2C_NUM_MAX } i2c_port_t;
typedef enum { I2C_MODE_SLAVE = 0, I2C_MODE_MASTER = 1 } i2c_mode_t;
typedef enum { I2C_MASTER_ACK = 0, I2C_MASTER_NACK = 1, I2C_MASTER_LAST_NACK = 2 } i2c_ack_type_t;

// Field order must match the real i2c_config_t: Common.cpp initializes it with
// designated initializers, which C++ requires to appear in declaration order.
typedef struct {
    i2c_mode_t mode;
    int        sda_io_num;
    int        scl_io_num;
    bool       sda_pullup_en;
    bool       scl_pullup_en;
    union {
        struct { uint32_t clk_speed; } master;
        struct { uint8_t addr_10bit_en; uint16_t slave_addr; uint32_t maximum_speed; } slave;
    };
    uint32_t   clk_flags;
} i2c_config_t;

typedef void * i2c_cmd_handle_t;

inline esp_err_t i2c_param_config(i2c_port_t, const i2c_config_t *) { return ESP_OK; }
inline esp_err_t i2c_driver_install(i2c_port_t, i2c_mode_t, size_t, size_t, int) { return ESP_OK; }
inline esp_err_t i2c_driver_delete(i2c_port_t) { return ESP_OK; }

inline i2c_cmd_handle_t i2c_cmd_link_create() { return (i2c_cmd_handle_t)1; }
inline void      i2c_cmd_link_delete(i2c_cmd_handle_t) {}
inline esp_err_t i2c_master_start(i2c_cmd_handle_t) { return ESP_OK; }
inline esp_err_t i2c_master_stop(i2c_cmd_handle_t) { return ESP_OK; }
inline esp_err_t i2c_master_write_byte(i2c_cmd_handle_t, uint8_t, bool) { return ESP_OK; }
inline esp_err_t i2c_master_write(i2c_cmd_handle_t, const uint8_t *, size_t, bool) { return ESP_OK; }
inline esp_err_t i2c_master_read_byte(i2c_cmd_handle_t, uint8_t * data, i2c_ack_type_t) { if (data) *data = 0; return ESP_OK; }
inline esp_err_t i2c_master_read(i2c_cmd_handle_t, uint8_t * data, size_t len, i2c_ack_type_t) {
    for (size_t i = 0; data && i < len; ++i) data[i] = 0;
    return ESP_OK;
}
inline esp_err_t i2c_master_cmd_begin(i2c_port_t, i2c_cmd_handle_t, TickType_t) { return ESP_OK; }

#endif
