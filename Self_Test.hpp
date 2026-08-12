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
