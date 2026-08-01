///// All and Constant Struct Definitions /////
/* 
   This file contains all of the general custom
   structures, constants, and functions
*/

#include "driver/i2c.h"

#ifndef UNIVERSAL_CONSTANTS_STRUCTS_HPP
#define UNIVERSAL_CONSTANTS_STRUCTS_HPP

// Constants //
constexpr float control_frequency = 80; // Hz
constexpr gpio_num_t sda_pin = GPIO_NUM_11;
constexpr gpio_num_t scl_pin = GPIO_NUM_12;
constexpr i2c_port_t i2c_port = I2C_NUM_0;
constexpr uint8_t IMU_ADDR = 0x68;
constexpr uint8_t PCA_ADDR = 0x40;

// Structures //
template <typename T>
struct vec {
    T x;
    T y;
    T z;
};

template <typename T>
struct triplet {
    T first;
    T second;
    T third;
};

template <typename T>
struct legQuad {
    T frontLeft;
    T frontRight;
    T backLeft;
    T backRight;
};

// Functions //

// REQUIRES: i2c bus is properly wired
// EFFECTS: Configures and prepares
// i2c bus using global "i2c_controller"
void init_i2c();

// REQUIRES: port is 1 or 0, addr, reg, and data are valid
// EFFECTS: Constructs, executes, and deletes a list at ptr that
// writes register reg of device with address addr to be data.
// This executes at I2C controller port.
esp_err_t write_to_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t data);

#endif