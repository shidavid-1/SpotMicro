///// On-Boot Self Tests /////
/* Cheap assertions over the pure (hardware-independent) math in this
   codebase. Call once from app_main before anything else runs, then read
   the results over the serial monitor.

   Scope is deliberately small: only logic that is FINISHED and stable
   belongs here. Code still under construction changes shape too often for
   a test to be worth maintaining. As pieces settle (servo angle math,
   limit_for, the shoulder complement), add them.
*/

#ifndef SELF_TEST_HPP
#define SELF_TEST_HPP

#include "IMU.hpp"
#include "Servo.hpp"

// Usable pulse window for TPUP's servos. Counts outside this are still legal
// 12-bit values that move_to_pwm writes without complaint, but the servo
// ignores them -- which presents as "the joint refuses to move".
constexpr float SERVO_MIN_PULSE_US = 500.0f;
constexpr float SERVO_MAX_PULSE_US = 2500.0f;

// Every intermediate value in SpotServo::move_to_angle.
struct SpotAngleMath {
    float       effective;      // angle after clamping to limit_for(type)
    bool        clamped;
    float       diff_angle;     // deg
    float       diff_width;     // us
    int         diff_count;
    int         final_count;    // what move_to_pwm receives
    float       final_width;    // us
    const char *branch;         // e.g. "left hip"
    const char *formula;
};

// TEST SUPPORT ONLY.
// EFFECTS: Reproduces SpotServo::move_to_angle's arithmetic for `requested`
// on servo `servo_num`, without commanding anything.
//
// WARNING: this RECOMPUTES rather than observes, because move_to_angle
// exposes no intermediates. The per-servo branch is transcribed from
// Servo.cpp. Change the formula there and you must change it here, or every
// test that depends on this will confidently report the wrong numbers. The
// permanent fix is to extract the math into Servo.cpp as a free function that
// both the driver and the tests call.
SpotAngleMath spot_angle_math(int servo_num, float requested);

// EFFECTS: Runs every self test and logs a PASS/FAIL line per case plus a
// summary. Returns true if all cases passed.
// NOTE: Borrows `imu` to reach its private conversion helpers. The IMU's
// range settings are saved and restored, so the object is unchanged on
// return. No I2C traffic is generated.
bool run_self_tests(IMU &imu);

// TEST SUPPORT ONLY. Not for use in flight code.
// MODIFIES: imu's stored ranges, and the IMU's config registers over I2C.
// EFFECTS: Re-ranges a constructed IMU so the saturation test can exercise
// every range without building a new object. Exists because the ranges are
// otherwise write-once at construction, exactly as the test plan anticipated.
void test_only_set_ranges(IMU &imu, AngularRange ang, AccelRange acc);

// TEST SUPPORT ONLY.
// EFFECTS: Returns true if every one of the IMU's data vectors is still all
// zero, for checking the constructor's "initialize with zero values" clause.
bool test_only_vectors_are_zero(const IMU &imu);

#endif // SELF_TEST_HPP
