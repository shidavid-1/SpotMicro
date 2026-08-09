///// Implement Complementary Filter /////

#ifndef FILTER_HPP
#define FILTER_HPP

#include "Common.hpp"
#include "Spot_Model.hpp"

const float alpha = 0.98;
const float dt = 1.0f/control_frequency;

vec<float> calculate_orientation(SpotModel * TPUP, const vec<float> angvel, const vec<float> accel);

vec<float> calculate_acceleration(const vec<float> ori, const vec<float> accel);

#endif