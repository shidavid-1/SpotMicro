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
// These limits are per joint TYPE while center_count is per SERVO, so a limit
// only holds if the WORST-centered servo of that type can reach it. At the old
// shoulder max 135 servo 8 (center 290) landed at 467 us and at the old knee
// max 180 servo 9 (center 316) landed at 2534 us, both outside the 500-2500 us
// window that test_limit_reachability() checks. Pulling those maxima to 131 and
// 176 puts all twelve inside it and keeps gaits off the extremes, where a servo
// would be pushing its internal end stop. Hips clear by 250+ us, so they are
// unchanged. Margins are thin -- servo 8 clears by 7 us and servo 9 by 10 -- so
// re-check this if any center_count is re-clocked.
AngleLimit limit_for(ServoType t) {
    switch (t) {
        case ServoType::hip:      return { -30.0f, 50.0f };
        case ServoType::shoulder: return { -5.0f, 131.0f };
        case ServoType::knee:     return { 20.0f, 176.0f };
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
        case 5:  return { 288, ServoType::shoulder };
        case 6:  return { 300, ServoType::knee };
        case 7:  return { 304, ServoType::hip };
        case 8:  return { 290, ServoType::shoulder };
        case 9:  return { 312, ServoType::knee };
        case 10: return { 326, ServoType::hip };
        case 11: return { 283, ServoType::shoulder };
        case 12: return { 303, ServoType::knee };
    }
    ESP_LOGI("ERROR MESSAGE", "Failed to construct servo: num %d is invalid.", num);
    exit(1);
}

void init_pca9685() {
    write_to_reg(PCA_ADDR, MODE1, 1U << 4);
    write_to_reg(PCA_ADDR, PRE_SCALE, pre_scale);
    write_to_reg(PCA_ADDR, MODE1, 1U << 5);
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

    if (led_pos > 15 || led_pos < 0) {
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
    uint8_t countMSBs = 0;
    uint8_t countLSBs = 0;

    if (numCounts > 4095 || numCounts < 0) {
        ESP_LOGI("ERROR MESSAGE", "Number of counts: %d, not in allowable range", numCounts);
        return; // don't change position
    }
    else {
        countMSBs = numCounts >> 8;
        countLSBs = numCounts;
    }

    write_to_servo_regs(countMSBs, countLSBs);
}

void Servo::write_to_servo_regs(uint8_t MSB, uint8_t LSB) {    
    uint8_t LED_ON_L  = 6 + 4 * led_pos;
    
    i2c_cmd_handle_t ptr = i2c_cmd_link_create();
    i2c_master_start(ptr);
    i2c_master_write_byte(ptr, (PCA_ADDR << 1), true);
    i2c_master_write_byte(ptr, LED_ON_L, true);
    i2c_master_write_byte(ptr, 0U, true);
    i2c_master_write_byte(ptr, 0U, true);
    i2c_master_write_byte(ptr, LSB, true);
    i2c_master_write_byte(ptr, MSB, true);
    i2c_master_stop(ptr);
    if (i2c_master_cmd_begin(i2c_port, ptr, pdMS_TO_TICKS(50)) != ESP_OK) {
        ESP_LOGE("SERVO", "Servo at channel %d is unable to write to registers.", led_pos);
    }
    i2c_cmd_link_delete(ptr);
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
        // Clamping means the foot will not land where the planner put it.
        // Folded silently, that reads like a controls bug later.
        ESP_LOGW("SERVO", "Servo %d clamped: %.1f -> %.1f deg",
                 get_servo_num(), (double)angle,
                 (double)(angle < lim.min ? lim.min : lim.max));
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