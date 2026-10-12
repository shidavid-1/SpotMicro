///// Inverse Kinematics for Single Leg /////
/*
   Computes angles to reach a particular position
   from within the reference frame of the leg.
*/

#ifndef IK_HPP
#define IK_HPP

#include "Common.hpp"
#include <utility>


// REQUIRES: pos is a triplet where 
// the elements represent
// the <x, y, z> coordinates, respectively
// EFFECTS: Computes the respective angles (degrees) to
// reach the position passed as a parameter.
// If the point isn't reachable, theta_1 is still computed
// and the leg is sent to the closest pose it can hold:
// completely extended if the point is too far away, and
// completely folded and tucked under the body if the point
// is nearer the shoulder than the leg can fold.
// Returns <{theta_1, theta_2, theta_3}, reachable>, where
// reachable is false when the angles describe one of those
// approximations rather than the requested point.
std::pair<triplet<float>, bool> position_to_angles(const vec<float> &pos);

#endif // IK_HPP