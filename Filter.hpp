///// Implement Complementary Filter /////

#ifndef FILTER_HPP
#define FILTER_HPP

#include "Common.hpp"
#include "Spot_Model.hpp"

const float alpha = 0.98;

// Updates orientation in TPUP and returns acceleration as a vec<Float>
vec<float> filter_IMU_data(SpotModel * TPUP, const vec<float> angvel, const vec<float> accel);

#endif