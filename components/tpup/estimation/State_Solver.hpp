///// State Solver /////
/*
   Takes in the orientation and servo angles
   to produce an estimate for the actual position
   of the dog with respect to the ground.
*/
#ifndef STATE_SOLVER_HPP
#define STATE_SOLVER_HPP

#include "Common.hpp"
#include "Spot_Model.hpp"

vec<float> calculate_position(const SpotModel * TPUP);

#endif