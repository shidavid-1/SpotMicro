///// Implement Complementary Filter /////

#ifndef FILTER_HPP
#define FILTER_HPP

#include "Common.hpp"

vec<float> calculate_orientation(const vec<float> angvel, const vec<float> accel);

vec<float> calculate_acceleration(const vec<float> angvel, const vec<float> accel);

#endif