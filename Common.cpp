///// All and Constant Struct Definitions /////
/* 
   This file contains all of the general custom
   structures, constants, and functions
*/

#include "Common.hpp"

#include "driver/i2c.h"
#include <cmath>

// Functions //

// REQUIRES: i2c bus is properly wired
// EFFECTS: Configures and prepares
// i2c bus using global "i2c_controller"
void init_i2c() {
    i2c_config_t conf = {
        .mode          = I2C_MODE_MASTER,
        .sda_io_num    = sda_pin,
        .scl_io_num    = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master        = { .clk_speed = 400000 },
        // No constraint on the source clock: any clock that can produce
        // 400 kHz is fine. Would need I2C_SCLK_SRC_FLAG_AWARE_DFS if dynamic
        // frequency scaling were enabled, since an APB-derived clock would
        // otherwise drift with the CPU frequency.
        .clk_flags     = 0,
    };
    i2c_param_config(i2c_port, &conf);
    i2c_driver_install(i2c_port, I2C_MODE_MASTER, 0, 0, 0);
}

// REQUIRES: port is 1 or 0, addr, reg, and data are valid
// EFFECTS: Constructs, executes, and deletes a list at ptr that
// writes register reg of device with address addr to be data.
// This executes at I2C controller port.
esp_err_t write_to_reg(uint8_t addr, uint8_t reg, uint8_t data) {
    esp_err_t valid_connection;
    i2c_cmd_handle_t ptr = i2c_cmd_link_create();
    i2c_master_start(ptr);
    i2c_master_write_byte(ptr, (addr << 1), true);
    i2c_master_write_byte(ptr, reg, true);
    i2c_master_write_byte(ptr, data, true);
    i2c_master_stop(ptr);
    valid_connection = i2c_master_cmd_begin(i2c_port, ptr, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(ptr);
    return valid_connection;
}

float distance_between(const vec<float> lhs, const vec<float> rhs) {
    return (float)sqrt(pow(lhs.x - rhs.x, 2) + pow(lhs.y - rhs.y, 2) + pow(lhs.z - rhs.z, 2));
}

// Operator Overloads //

vec<float> operator-(const vec<float> &lhs, const vec<float> &rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}