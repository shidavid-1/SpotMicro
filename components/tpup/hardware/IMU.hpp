///// IMU Abstraction Layer /////
/* This file is responsible for managing all
   incoming data from the IMU. Specifically:
   INPUT: Robot motion
          Control Loop Frequency
   OUTPUT: Raw Angular Velocity
           Raw Linear Acceleration
*/

#ifndef IMU_HPP
#define IMU_HPP

#include "Common.hpp"
#include <cstdint>

constexpr uint8_t WHO_AM_I = 0x75;
constexpr uint8_t WHO_AM_I_IMU_VALUE = 0x68;
constexpr uint8_t PWR_MGMT_1 = 0x6B;
constexpr uint8_t GYRO_CONFIG = 0x1B;
constexpr uint8_t ACCEL_CONFIG = 0x1C;

enum class AccelRange {plus_minus_2g, plus_minus_4g, plus_minus_8g, plus_minus_16g};
enum class AngularRange {plus_minus_250dps, plus_minus_500dps, plus_minus_1000dps, plus_minus_2000dps};

// Grants the on-boot self tests access to the private conversion helpers.
struct IMUSelfTest;

class IMU {
       friend struct IMUSelfTest;
       // Test support only, defined in Self_Test.cpp. Declared here so the
       // hardware test session can re-range a constructed IMU and inspect
       // the freshly constructed state; neither is part of the flight API.
       friend void test_only_set_ranges(IMU &, AngularRange, AccelRange);
       friend bool test_only_vectors_are_zero(const IMU &);

       private:
              // Overhead //
              AngularRange angRange;      // Stores which range of operation its in
              AccelRange acclRange;       // Stores which range of operation its in
              // Data //
              vec<int16_t> raw_angvel;
              vec<int16_t> raw_accel;           
              vec<float> angvel;       // Most recent angvel values
              vec<float> angvel_zero;
              vec<float> accel;        // Most recent accel values
              vec<float> accel_zero;

              // EFFECTS: Updates angvel and accel by converting
              // raw_angvel and raw_accel
              void binary_to_float();

              uint8_t angRange_to_bin() const;

              uint8_t accelRange_to_bin() const;

              float ang_conversion() const;

              float accl_conversion() const;

              void config_IMU() const;

       public:
              IMU(AngularRange angRange, AccelRange acclrange);

              // MODIFIES: angvel_zero and accel_zero
              void calibrate();

              void reset_calibration();

              // MODIFIES: raw and converted values
              void update();

              vec<float> get_angvel() const;

              vec<float> get_accel() const;

              // NOTE: no setters because only IMU sets values //
};

#endif