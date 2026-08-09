#ifndef SERVO_CONTROL_HPP
#define SERVO_CONTROL_HPP

// Drives PCA9685-connected hobby servos over I2C, with per-servo calibration
// and per-joint angle limits.

#include "Common.hpp"     // triplet<T>, i2c_port, PCA_ADDR
#include "driver/i2c.h"   // i2c_port_t
#include <cmath>          // std::ceil (for pre_scale)
#include <cstdint>

// Set to a value in valid range [50,330]
constexpr float desired_pwm_freq = 50;

constexpr int   MODE1     = 0x00;
constexpr int   PRE_SCALE = 0xFE;
constexpr float oscillator_freq = 25000000.0f;
// 12 bit counter tick = prescale + 1, thus the -1 in the formula below
constexpr uint8_t pre_scale = std::ceil(oscillator_freq / (4096 * desired_pwm_freq) - 1);
constexpr float true_pwm_freq = oscillator_freq / (4096.0f * (float)(pre_scale + 1));
// period is in microseconds
constexpr double period = 1.0 / true_pwm_freq * 1000000;

// The kind of joint a servo drives. Determines its allowable angle range.
// "Free" means no boundary protection (full 0-180 travel).
enum class ServoType { hip, shoulder, knee, free };

// Enum representing the 4 different leg positions
enum class LegType { frontLeft, frontRight, backLeft, backRight };

// Inclusive [min, max] angle range (degrees) a servo of a given type may move to.
struct AngleLimit { float min; float max; };

// Calibration coefficients for one servo's best-fit line (from the calibration
// sheet). move_to_angle applies: angle_sent = (angle - offset) / slope.
// Also contains the type of servo.
struct Calibration { float slope; float offset; ServoType type; };


// EFFECTS: Returns the allowable angle range for the given servo type.
AngleLimit limit_for(ServoType t);

// EFFECTS: Returns a human-readable name for the given servo type (for logging).
const char* type_name(ServoType t);

static void init_pca9685();

// REQUIRES: num is 0-12.
// EFFECTS: Returns the calibration for servo number `num`. num 1-12 are the
// physical servos (per the sticker); num 0 is the reserved uncalibrated servo
// (identity: slope 1, offset 0). Aborts the program if num is out of range.
Calibration calibration_for(int num);

class Servo {
    private:
        const int           servo_num;  // servo name/number
        const ServoType     type;       // Hip, Knee, Shoulder, or Free
        const int           led_pos;    // int from 0 - 15
        const float         slope;      // slope of best fit line from calibration sheet
        const float         offset;     // y intercept of best fit line from calibration sheet;
                                        // the sign must match the sign of the y intercept
                                        // in the equation for the best fit line from the sheet
        uint32_t            curr_pulse_width; // stores the value describing the most recent pulse
                                              // width sent to the servo in microseconds
        float               des_angle;        // stores the last angle that we wanted to move to
        float               true_angle;       // stores the actual angle that the servo moved to
        // NOTE: Both angular members store the physical angles that we wish to be at and
        // believe to actually be at. That is, they are NOT necessarily the angles the servos
        // were commanded to, rather, the angle that we believe an imaginary needle on a servo points to.

    public:
        // REQUIRES: num is 0-12, port is a valid I2C port.
        // EFFECTS: Constructs an on-robot servo. num is the servo number based
        // on the sticker on each servo ('S9' has num = 9); num = 0 is reserved
        // for testing an uncalibrated servo. Type and led_pos (num - 1) are
        // derived from num. Aborts the program on invalid arguments.
        Servo(int num);

        // REQUIRES: num is 0-12, port is a valid I2C port, 0 <= led_pos <= 15.
        // EFFECTS: Constructs a servo with an explicitly chosen type and LED
        // position, for wiring that does not follow the num - 1 convention.
        // Calibration coefficients are still looked up from num.
        // Aborts the program on invalid arguments.
        Servo(int num, ServoType type, int led_pos);

        // EFFECTS: Returns the joint type this servo drives.
        ServoType get_type() const;

        // REQUIRES: port is 1 or 0, addr, and reg are valid
        // 0 <= angle <= 180, 0 <= led_pos <= 15
        // EFFECTS: Moves the servo at led_pos of device with
        // address addr to the angle angle (in degrees)
        // NOTE: The angles passed are interpretted in the frame of
        // reference of the SERVOS. This is DIFFERENT than the
        // thetas defined for IK ONLY for the shoulder servo.
        // So if you send 85 degrees to the shoulder servo
        // the corresponding theta value is actually 95 degrees
        // if you want theta to be 95, you send 85 to the shoulder.
        void move_to_angle(float angle);
        
        void move_to_pwm(float width);
};

#endif // SERVO_CONTROL_HPP