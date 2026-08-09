#include "Servo.hpp"
#include "Common.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include <cstdlib>

//////////////////// FREE FUNCTIONS ////////////////////

AngleLimit limit_for(ServoType t) {
    switch (t) {
        case ServoType::hip:      return { 40.0f, 175.0f };
        case ServoType::shoulder: return {  0.0f, 180.0f };
        case ServoType::knee:     return { 20.0f, 180.0f };
        case ServoType::free:     return {  0.0f, 180.0f };
    }
    __builtin_unreachable();
}

const char* type_name(ServoType t) {
    switch (t) {
        case ServoType::hip:      return "hip";
        case ServoType::shoulder: return "shoulder";
        case ServoType::knee:     return "knee";
        case ServoType::free:     return "no restriction";
    }
    return "unknown";
}

Calibration calibration_for(int num) {
    switch (num) {
        case 0:  return { 1.00f,  0.0f, ServoType::free};   // uncalibrated identity
        case 1:  return { 1.02f, -0.4f, ServoType::hip};
        case 2:  return { 1.05f, -0.5f, ServoType::shoulder};
        case 3:  return { 1.05f,  0.0f, ServoType::knee};
        case 4:  return { 1.03f, -0.7f, ServoType::hip};
        case 5:  return { 1.05f, -0.4f, ServoType::shoulder};
        case 6:  return { 1.05f,  0.3f, ServoType::knee};
        case 7:  return { 1.05f, -0.1f, ServoType::hip};
        case 8:  return { 1.04f, -0.5f, ServoType::shoulder};
        case 9:  return { 1.02f,  0.5f, ServoType::knee};
        case 10: return { 1.03f,  0.9f, ServoType::hip};
        case 11: return { 1.04f,  0.4f, ServoType::shoulder};
        case 12: return { 1.04f,  0.9f, ServoType::knee};
    }
    ESP_LOGI("ERROR MESSAGE", "Failed to construct servo: num %d is invalid.", num);
    exit(1);
}

static void init_pca9685() {
    write_to_reg(PCA_ADDR, MODE1, 16);
    write_to_reg(PCA_ADDR, PRE_SCALE, pre_scale);
    write_to_reg(PCA_ADDR, MODE1, 0);
}

//////////////////// SERVO ////////////////////

Servo::Servo(int num)
    : servo_num(num), type(calibration_for(num).type),
      led_pos(num - 1),
      slope(calibration_for(num).slope),
      offset(calibration_for(num).offset),
      curr_pulse_width(0),
      des_angle(0.0f), true_angle(0.0f) {

    // calibration_for() above already aborted if num was invalid, so
    // by this point num is known good. ServoType being an enum likewise
    // guarantees type is valid, so neither needs a check here.

    if (i2c_port < 0 || i2c_port >= I2C_NUM_MAX) {
        ESP_LOGI("ERROR MESSAGE", "Failed to construct servo num %d because port is invalid.", servo_num);
        exit(1);
    }

    if (led_pos > 15) {
        ESP_LOGI("ERROR MESSAGE", "Failed to construct servo num %d because LED_pos \"%d\" is invalid.", servo_num, led_pos);
        exit(1);
    }

    if (num == 0){
        ESP_LOGI("WARNING", "Created \"free\" servo which will move with no restrictions");
    }
}

Servo::Servo(int num, ServoType type, int led_pos)
    : servo_num(num), type(type),
      led_pos(led_pos),
      slope(calibration_for(num).slope),
      offset(calibration_for(num).offset),
      curr_pulse_width(0),
      des_angle(0.0f), true_angle(0.0f) {

    // calibration_for() above already aborted if num was invalid, so
    // by this point num is known good. ServoType being an enum likewise
    // guarantees type is valid, so neither needs a check here.

    if (i2c_port < 0 || i2c_port >= I2C_NUM_MAX) {
        ESP_LOGI("ERROR MESSAGE", "Failed to construct servo num %d because port is invalid.", servo_num);
        exit(1);
    }

    if (led_pos > 15) {
        ESP_LOGI("ERROR MESSAGE", "Failed to construct servo num %d because LED_pos \"%d\" is invalid.", servo_num, led_pos);
        exit(1);
    }

    if (num == 0){
        ESP_LOGI("WARNING", "Created \"free\" servo which will move with no restrictions");
    }
}

ServoType Servo::get_type() const {
    return type;
}

void Servo::move_to_angle(float angle) {
    // Check that the target angle is within this servo's allowable
    // physical range. The range is looked up by type (Free spans the
    // full 0-180 travel, so it never trips this).
    AngleLimit lim = limit_for(type);
    if (angle < lim.min || angle > lim.max) {
        ESP_LOGI("ERROR MESSAGE",
                 "%s servo at %d attempted to go to %.1f, which is outside of [%.1f, %.1f].",
                 type_name(type), led_pos, angle, lim.min, lim.max);
        exit(1);
    }

    float angle_sent = (angle - offset) / slope;

    // Translate angle into width
    float width_in_usec = angle_sent * (1000.0 / 90) + 500;

    // Update member variable
    des_angle = angle;

    move_to_pwm(width_in_usec);
}

// NOTE: This applies NO calibration information
void Servo::move_to_pwm(float width) {
    uint8_t LED_ON_L  = 6 + 4 * led_pos;
    uint8_t LED_ON_H  = LED_ON_L + 1;
    uint8_t LED_OFF_L = LED_ON_L + 2;
    uint8_t LED_OFF_H = LED_ON_L + 3;
    uint8_t countMSBs = 0;
    uint8_t countLSBs = 0;
    int numCounts = std::round(width / period * 4096 - 1);

    if (numCounts > 4095 || numCounts < 0) {
        ESP_LOGI("ERROR MESSAGE", "Number of counts: %d, not in allowable range", numCounts);
    }
    else {
        countMSBs = numCounts >> 8;
        countLSBs = numCounts;
    }

    // Write to each of the four registers
    if (write_to_reg(PCA_ADDR, LED_ON_H, 0) != ESP_OK)
        ESP_LOGI("ERROR MESSAGE", "Servo %d is unable to write to register %s.", led_pos, "LED_ON_H");
    if (write_to_reg(PCA_ADDR, LED_ON_L, 0) != ESP_OK)
        ESP_LOGI("ERROR MESSAGE", "Servo %d is unable to write to register %s.", led_pos, "LED_ON_L");
    if (write_to_reg(PCA_ADDR, LED_OFF_H, countMSBs) != ESP_OK)
        ESP_LOGI("ERROR MESSAGE", "Servo %d is unable to write to register %s.", led_pos, "LED_OFF_H");
    if (write_to_reg(PCA_ADDR, LED_OFF_L, countLSBs) != ESP_OK)
        ESP_LOGI("ERROR MESSAGE", "Servo %d is unable to write to register %s.", led_pos, "LED_OFF_L");

    // update all member variables:
    curr_pulse_width = numCounts * 4096 * true_pwm_freq * 1000000;
    true_angle       = (curr_pulse_width - 500) * (90.0 / 1000) * slope + offset;
}