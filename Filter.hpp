///// Implement Complementary Filter /////

#include "Common.hpp"

#ifndef SERVO_HPP
#define SERVO_HPP

vec<float> calculate_orientation(const vec<float> angvel, const vec<float> accel);

vec<float> calculate_acceleration(const vec<float> angvel, const vec<float> accel);

#endif