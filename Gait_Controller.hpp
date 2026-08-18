///// Gait Controller /////
/*
   Computes all next foot tip coordinates in
   the body reference frame based on SpotModel.
*/

#ifndef GAIT_CONTROLLER_HPP
#define GAIT_CONTROLLER_HPP

#include "Common.hpp"
#include "Spot_Model.hpp"

legQuad<vec<float>> compute_next_points(const float phase);

#endif