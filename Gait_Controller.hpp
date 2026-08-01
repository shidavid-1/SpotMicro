///// Gait Controller /////
/*
   Computes all next foot tip coordinates in
   the body reference frame based on SpotModel.
*/

#include "Spot_Model.hpp"
#include "Common.hpp"

#ifndef GAIT_CONTROLLER_HPP
#define GAIT_CONTROLLER_HPP

legQuad<vec<float>> compute_next_points(const SpotModel * TPUP);

#endif