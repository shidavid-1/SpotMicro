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

constexpr uint8_t PWR_MGMT_1 = 0x6B;
constexpr uint8_t GYRO_CONFIG = 0x1B;
constexpr uint8_t ACCEL_CONFIG = 0x1C;
constexpr float binary_to_500dps = 1000.0f/65536.0f;
constexpr float binary_to_4g = 8.0f/65536.0f;

enum class AngularRange {plus_minus_2g, plus_minus_4g, plus_minus_8g, plus_minus_16g};
enum class AccelRange {plus_minus_250dps, plus_minus_500dps, plus_minus_1000dps, plus_minus_2000dps};

class IMU {
       private:
              // Overhead //
              AngularRange angRange;      // Stores which range of operation its in
              AccelRange acclRange;       // Stores which range of operation its in
              // Data //
              vec<uint16_t> raw_angvel;
              vec<uint16_t> raw_accel;           
              vec<float> angvel;       // Most recent angvel values
              vec<float> angvel_zero;
              vec<float> accel;        // Most recent accel values
              vec<float> accel_zero;

              vec<float> binary_to_float(vec<uint16_t> raw_values, bool isAngvel) const;

       public:
              IMU(AngularRange angRange, AccelRange acclrange);

              // MODIFIES: angvel_zero and accel_zero
              void calibrate();

              // MODIFIES: raw and converted values
              void update();

              vec<float> get_angvel() const;

              vec<float> get_accel() const;

              // NOTE: no setters because only IMU sets values //
};

#endif