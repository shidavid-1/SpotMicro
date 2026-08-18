#include "Servo.hpp"
#include "Common.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include <cstdlib>

//////////////////// FREE FUNCTIONS ////////////////////

// in the frame of reference of the SPOT.
// the hip limits are the physical limits on theta_1
// the shoulder limits are the physical limits on theta_3
// the knee limits are the physical limits on theta_2
AngleLimit limit_for(ServoType t) {
    switch (t) {
        case ServoType::hip:      return { -40.0f, 60.0f };
        case ServoType::shoulder: return {   -5.0f, 135.0f };
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
        case 0:  return { 1.00f,  0.0f };   // uncalibrated identity
        case 1:  return { 1.02f, -0.4f };
        case 2:  return { 1.05f, -0.5f };
        case 3:  return { 1.05f,  0.0f };
        case 4:  return { 1.03f, -0.7f };
        case 5:  return { 1.05f, -0.4f };
        case 6:  return { 1.05f,  0.3f };
        case 7:  return { 1.05f, -0.1f };
        case 8:  return { 1.04f, -0.5f };
        case 9:  return { 1.02f,  0.5f };
        case 10: return { 1.03f,  0.9f };
        case 11: return { 1.04f,  0.4f };
        case 12: return { 1.04f,  0.9f };
    }
    ESP_LOGI("ERROR MESSAGE", "Failed to construct servo: num %d is invalid.", num);
    exit(1);
}

SpotCalibration spot_calibration_for(int num) {
    switch (num) {
        case 0:  return {   0, ServoType::free };   // no meaningful center
        case 1:  return { 319, ServoType::hip };
        case 2:  return { 297, ServoType::shoulder };
        case 3:  return { 292, ServoType::knee };
        case 4:  return { 325, ServoType::hip };
        case 5:  return { 309, ServoType::shoulder };
        case 6:  return { 300, ServoType::knee };
        case 7:  return { 304, ServoType::hip };
        case 8:  return { 290, ServoType::shoulder };
        case 9:  return { 316, ServoType::knee };
        case 10: return { 316, ServoType::hip };
        case 11: return { 283, ServoType::shoulder };
        case 12: return { 317, ServoType::knee };
    }
    ESP_LOGI("ERROR MESSAGE", "Failed to construct servo: num %d is invalid.", num);
    exit(1);
}

void init_pca9685() {
    write_to_reg(PCA_ADDR, MODE1, 16);
    write_to_reg(PCA_ADDR, PRE_SCALE, pre_scale);
    write_to_reg(PCA_ADDR, MODE1, 0);
}

//////////////////// SERVO ////////////////////

// Use this to create the actual servos on the spot!
Servo::Servo(int num)
    : servo_num(num),
      led_pos(num - 1),
      slope(calibration_for(num).slope),
      offset(calibration_for(num).offset) {

    // calibration_for() above already aborted if num was invalid, so
    // by this point num is known good and needs no check here.

    if (i2c_port < 0 || i2c_port >= I2C_NUM_MAX) {
        ESP_LOGI("ERROR MESSAGE", "Failed to construct servo num %d because port is invalid.", servo_num);
        exit(1);
    }

    if (led_pos > 15 || led_pos < 0) {
        ESP_LOGI("ERROR MESSAGE", "Failed to construct servo num %d because LED_pos \"%d\" is invalid.", servo_num, led_pos);
        exit(1);
    }

    if (num == 0){
        ESP_LOGI("WARNING", "Created \"free\" servo which will move with no restrictions");
    }
}

Servo::Servo(int num, int led_pos)
    : servo_num(num),
      led_pos(led_pos),
      slope(calibration_for(num).slope),
      offset(calibration_for(num).offset) {

    // calibration_for() above already aborted if num was invalid, so
    // by this point num is known good and needs no check here.

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

int Servo::get_servo_num() const {
    return servo_num;
}

float Servo::get_slope() const {
    return slope;
}

void Servo::move_to_angle(float angle) {
    // Need to rework this formula to be centered around
    // the center_pwm value rather than the arbitrary
    // offset constant.
    float angle_sent = (angle - offset) / slope;

    // Translate angle into width
    float width_in_usec = angle_sent * (1000.0 / 90) + 500;

    int numCounts = std::round(width_in_usec / period * 4096 - 1);

    move_to_pwm(numCounts);
}

// NOTE: This applies NO calibration information
void Servo::move_to_pwm(int numCounts) {
    uint8_t LED_ON_L  = 6 + 4 * led_pos;
    uint8_t LED_ON_H  = LED_ON_L + 1;
    uint8_t LED_OFF_L = LED_ON_L + 2;
    uint8_t LED_OFF_H = LED_ON_L + 3;
    uint8_t countMSBs = 0;
    uint8_t countLSBs = 0;

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
}

//////////////////// SPOT SERVO ////////////////////

// Use this to create the actual servos on the spot!
SpotServo::SpotServo(int num)
    : Servo(num),
      center_count(spot_calibration_for(num).center_count),
      type(spot_calibration_for(num).type) {

    // The Servo base constructor runs first and already aborted if num or the
    // I2C port was invalid, so by this point num is known good.
}

ServoType SpotServo::get_type() const {
    return type;
}

int SpotServo::get_center_count() const {
    return center_count;
}

void SpotServo::move_to_angle(float angle) {
    // Check that the target angle is within this servo's allowable
    // physical range. The range is looked up by type (Free spans the
    // full 0-180 travel, so it never trips this).
    AngleLimit lim = limit_for(type);
    if (angle < lim.min || angle > lim.max) {
        if (angle < lim.min) {
            angle = lim.min;
        }
        else {
            angle = lim.max;
        }
    }
    
    // Intermediate variable to keep track of 
    // computed pulse width in microseconds
    float diff_angle = 0;
    float diff_width = 0;
    int diff_count = 0;

    // Compute adjusted diff_angle based on servo number
    switch(get_servo_num()) {
        case 1:
        case 10:
            // Left hip
            diff_angle = - 1/get_slope() * angle;
            break;
        case 4:
        case 7:
            // Right hip
            diff_angle = 1/get_slope() * angle;
            break;
        case 2:
        case 8:
            // Left upper
            diff_angle = 1/get_slope() * (45.0f - angle);
            break;
        case 5:
        case 11:
            // Right upper
            diff_angle = - 1/get_slope() * (45.0 - angle);
            break;
        case 3:
        case 9:
            // Left lower
            diff_angle = - 1/get_slope() * (90.0 - angle);
            break;
        case 6:
        case 12:
            // right lower
            diff_angle = 1/get_slope() * (90.0 - angle);
            break;
        default:
            diff_angle = 0;
    }

    // Calculate and command to proper counts
    diff_width = diff_angle * (1000.0f/90);
    diff_count = std::round(diff_width / period * 4096);
    move_to_pwm(center_count + diff_count);
}