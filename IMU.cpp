#include "IMU.hpp"
#include "Common.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <vector>
#include <cstdint>

//////////////////// IMU ////////////////////

// NOTE: it is critical that accel_zero and angvel_zero are initialized
// to be 0 in order for calibrate() to function properly, due to the
// implementation of binary_to_float.
IMU::IMU(AngularRange angRange, AccelRange acclRange)
    : angRange(angRange), acclRange(acclRange),
      raw_angvel({}), raw_accel({}),
      angvel({}), angvel_zero({}),
      accel({}), accel_zero({}) {

    // Check that the IMU is plugged in and at its proper address
    // configure the register pointer to point to WHO_AM_I
    uint8_t IM_IMU = 0;
    i2c_cmd_handle_t ptr = i2c_cmd_link_create();
    i2c_master_start(ptr);
    i2c_master_write_byte(ptr, IMU_ADDR << 1, true);
    i2c_master_write_byte(ptr, WHO_AM_I, true);
    i2c_master_stop(ptr);
    if (i2c_master_cmd_begin(i2c_port, ptr, pdMS_TO_TICKS(50)) != ESP_OK) {
        ESP_LOGE("IMU ERROR", "Failed to read from IMU, check that IMU is plugged in.");
        abort();
    }
    i2c_cmd_link_delete(ptr);

    // read from the IMU
    ptr = i2c_cmd_link_create();
    i2c_master_start(ptr);
    i2c_master_write_byte(ptr, (IMU_ADDR << 1) + 1, true);
    i2c_master_read_byte(ptr, &IM_IMU, I2C_MASTER_NACK);
    i2c_master_stop(ptr);
    if (i2c_master_cmd_begin(i2c_port, ptr, pdMS_TO_TICKS(50)) != ESP_OK) {
        ESP_LOGE("IMU ERROR", "Failed to read from IMU, check that IMU is plugged in.");
        abort();
    }
    i2c_cmd_link_delete(ptr);
    
    if (IM_IMU == WHO_AM_I_IMU_VALUE) {
        config_IMU();
    }
    else {
        ESP_LOGE("ERROR", "Address in WHO_AM_I register incorrect, check that correct device is plugged in.");
        abort();
    }
}

// When called, updates the IMU readings, then stores the readings
// immediately after the update as the zero values for accel and angvel.
void IMU::calibrate() {
    vec<float> angvel_sum;
    vec<float> accel_sum;
    vec<float> angvel_avg;
    vec<float> accel_avg;
    uint32_t loop_iterations = 10000;
    for (uint32_t i = 0; i < loop_iterations; ++i) {
        reset_calibration();
        update();
        angvel_sum = angvel_sum + angvel;
        accel_sum = accel_sum + accel;
    }

    // Compute averages:
    angvel_avg.x = angvel_sum.x / loop_iterations;
    angvel_avg.y = angvel_sum.y / loop_iterations;
    angvel_avg.z = angvel_sum.z / loop_iterations;
    accel_avg.x = accel_sum.x / loop_iterations;
    accel_avg.y = accel_sum.y / loop_iterations;
    accel_avg.z = accel_sum.z / loop_iterations;

    angvel_zero = angvel_avg;
    accel_zero.x = accel_avg.x;
    accel_zero.y = accel_avg.y;
    accel_zero.z = accel_avg.z - gravity;
        
}

void IMU::reset_calibration() {
    angvel_zero = {0.0f, 0.0f, 0.0f};
    accel_zero = {0.0f, 0.0f, 0.0f};
}

void IMU::update() {
    i2c_cmd_handle_t ptr;
    uint8_t data[14];

    // configure the register pointer
    ptr = i2c_cmd_link_create();
    i2c_master_start(ptr);
    i2c_master_write_byte(ptr, IMU_ADDR << 1, true);
    i2c_master_write_byte(ptr, 0x3B, true);
    i2c_master_stop(ptr);
    i2c_master_cmd_begin(i2c_port, ptr, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(ptr);

    // read from the IMU
    ptr = i2c_cmd_link_create();
    i2c_master_start(ptr);
    i2c_master_write_byte(ptr, (IMU_ADDR << 1) + 1, true);
    for (uint8_t i = 0; i < 13; ++i) {
        if(i2c_master_read_byte(ptr, data + i, I2C_MASTER_ACK) != ESP_OK) {
            ESP_LOGI("ERROR", "Unable to read from IMU");
        }
    }
    i2c_master_read_byte(ptr, data + 13, I2C_MASTER_NACK);
    i2c_master_stop(ptr);
    i2c_master_cmd_begin(i2c_port, ptr, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(ptr);

    // Parse the data from the IMU
    raw_accel.x   = ((uint16_t)data[0] << 8U) | (uint16_t)data[1];
    raw_accel.y   = ((uint16_t)data[2] << 8U) | (uint16_t)data[3];
    raw_accel.z   = ((uint16_t)data[4] << 8U) | (uint16_t)data[5];
    raw_angvel.x  = ((uint16_t)data[8] << 8U) | (uint16_t)data[9];
    raw_angvel.y  = ((uint16_t)data[10] << 8U) | (uint16_t)data[11];
    raw_angvel.z  = ((uint16_t)data[12] << 8U) | (uint16_t)data[13];

    // Decipher the data
    binary_to_float();
}

vec<float> IMU::get_angvel() const {
    return angvel;
}

vec<float> IMU::get_accel() const {
    return accel;
}

//////////////////// PRIVATE HELPERS ////////////////////

void IMU::binary_to_float() {
    angvel.x = raw_angvel.x     * ang_conversion()  - angvel_zero.x;
    angvel.y = raw_angvel.y     * ang_conversion()  - angvel_zero.y;
    angvel.z = raw_angvel.z     * ang_conversion()  - angvel_zero.z;
    accel.x  = raw_accel.x      * accl_conversion() - accel_zero.x;
    accel.y  = raw_accel.y      * accl_conversion() - accel_zero.y;
    accel.z  = raw_accel.z      * accl_conversion() - accel_zero.z;
}

uint8_t IMU::angRange_to_bin() const {
    switch (angRange) {
        case AngularRange::plus_minus_250dps:  return (uint8_t) 0b00;
        case AngularRange::plus_minus_500dps:  return (uint8_t) (0b01 << 3);
        case AngularRange::plus_minus_1000dps: return (uint8_t) (0b10 << 3);
        case AngularRange::plus_minus_2000dps: return (uint8_t) (0b11 << 3);
    }
    __builtin_unreachable();
}

uint8_t IMU::accelRange_to_bin() const {
    switch (acclRange) {
        case AccelRange::plus_minus_2g:  return (uint8_t) 0b00;
        case AccelRange::plus_minus_4g:  return (uint8_t) (0b01 << 3);
        case AccelRange::plus_minus_8g:  return (uint8_t) (0b10 << 3);
        case AccelRange::plus_minus_16g: return (uint8_t) (0b11 << 3);
    }
    __builtin_unreachable();
}

float IMU::ang_conversion() const {
    switch(angRange) {
        case AngularRange::plus_minus_250dps:  return 500.0f/65536.0f;
        case AngularRange::plus_minus_500dps:  return 1000.0f/65536.0f;
        case AngularRange::plus_minus_1000dps: return 2000.0f/65536.0f;
        case AngularRange::plus_minus_2000dps: return 4000.0f/65536.0f;
    }
    __builtin_unreachable();
}

float IMU::accl_conversion() const {
    switch(acclRange) {
        case AccelRange::plus_minus_2g:  return gravity * 4.0f/65536.0f;
        case AccelRange::plus_minus_4g:  return gravity * 8.0f/65536.0f;
        case AccelRange::plus_minus_8g:  return gravity * 16.0f/65536.0f;
        case AccelRange::plus_minus_16g: return gravity * 32.0f/65536.0f;
    }
    __builtin_unreachable();
}

void IMU::config_IMU() const {
    // clear the sleep bit to turn on the device
    write_to_reg(IMU_ADDR, PWR_MGMT_1, 0x00);
    // configure the gyroscope to +/- 500 dps
    write_to_reg(IMU_ADDR, GYRO_CONFIG, angRange_to_bin());
    // configure the accelerometer +/- 4g
    write_to_reg(IMU_ADDR, ACCEL_CONFIG, accelRange_to_bin());
}
