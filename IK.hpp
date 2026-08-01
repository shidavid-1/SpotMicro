///// Inverse Kinematics for Single Leg /////
/*
   Computes angles to reach a particular position
   from within the reference frame of the leg.
*/

#include "Common.hpp"
#include "Spot_Model.hpp"
#include <utility>


// REQUIRES: pos is a triplet where 
// the elements represent
// the <x, y, z> coordinates, respectively
// EFFECTS: Computes the respective angles (degrees) to
// reach the position passed as a parameter.
// If the point isn't reachable, either an error
// will be thrown or the closest point will be reached
// by computing the theta_1 and setting the other angles
// such that the upper leg and foreleg are completely 
// extended.

// NEED TO STILL IMPLEMENT THE UNREACHABILE HANDLING STUFF & foot tip offset
std::pair<triplet<float>, bool> position_to_angles(const vec<float> &pos, const LegType lType);