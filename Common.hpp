///// All and Constant Struct Definitions /////
/* 
   This file contains all of the general custom
   structures, constants, and functions
*/

#ifndef UNIVERSAL_CONSTANTS_STRUCTS_HPP
#define UNIVERSAL_CONSTANTS_STRUCTS_HPP

#include "driver/i2c.h"
#include "esp_log.h"
#include <cstdlib>

// Constants //
constexpr float control_frequency = 80; // Hz
constexpr float dt = 1.0f/control_frequency;
constexpr gpio_num_t sda_pin = GPIO_NUM_11;
constexpr gpio_num_t scl_pin = GPIO_NUM_12;
constexpr i2c_port_t i2c_port = I2C_NUM_0;
constexpr uint8_t IMU_ADDR = 0x68;
constexpr uint8_t PCA_ADDR = 0x40;

// Leg geometry, shared by the model and the IK solver
constexpr float HIP_OFFSET      = 10.0f;          // mm
constexpr float SHOULDER_LENGTH = 61.5f;          // mm
constexpr float UPPERLEG_LENGTH = 110.88535746f;  // mm
constexpr float FORELEG_LENGTH  = 118.25f;        // mm
constexpr float FOOTTIP_RADIUS  = 40.0f;          // mm

constexpr float RAD_PER_DEG = 0.01745329251994329577f;
constexpr float gravity     = 9.81f; // m/s^2

// Enums //

enum class LegType {frontLeft, frontRight, backLeft, backRight};

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

// Macros //

// EFFECTS: If cond is false, logs the failed condition with its file and line
// under "tag" and aborts. Unlike assert(), this is not removed by NDEBUG or by
// the ESP-IDF assertion-level setting, so it is the right check for a condition
// that must hold before the servos are commanded. Use plain assert() for
// checks that are only there to catch programmer error during bring-up.
#define TPUP_CHECK(cond, tag)                                                      do {                                                                               if (!(cond)) {                                                                     ESP_LOGE((tag), "Check failed: %s (%s:%d)",                                             #cond, __FILE__, __LINE__);                                           abort();                                                                   }                                                                          } while (0)

// Functions //

// REQUIRES: i2c bus is properly wired
// EFFECTS: Configures and prepares
// i2c bus using global "i2c_port"
void init_i2c();

// REQUIRES: port is 1 or 0, addr, reg, and data are valid
// EFFECTS: Constructs, executes, and deletes a list at ptr that
// writes register reg of device with address addr to be data.
// This executes at I2C controller port.
esp_err_t write_to_reg(uint8_t addr, uint8_t reg, uint8_t data);

float distance_between(const vec<float> lhs, const vec<float> rhs);

// Operator Overloads //

vec<float> operator-(const vec<float> &lhs, const vec<float> &rhs);

#endif