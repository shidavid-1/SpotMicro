///// State Solver /////
/*
   Takes in the orientation and servo angles
   to produce an estimate for the actual position
   of the dog with respect to the ground.
*/
#include "Common.hpp"
#include "Spot_Model.hpp"

#ifndef STATE_SOLVER_HPP
#define STATE_SOLVER_HPP

vec<float> calculate_position(const SpotModel * TPUP);

#endif