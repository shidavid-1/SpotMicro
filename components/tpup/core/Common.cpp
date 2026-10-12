///// All and Constant Struct Definitions /////
/* 
   This file contains all of the general custom
   structures, constants, and functions
*/

#include "Common.hpp"

#include <cmath>

// Functions //

float distance_between(const vec<float> &lhs, const vec<float> &rhs) {
    return (float)sqrt(pow(lhs.x - rhs.x, 2) + pow(lhs.y - rhs.y, 2) + pow(lhs.z - rhs.z, 2));
}

float magnitude(const vec<float> &my_vec) {
    return (float) sqrt((float) (my_vec.x * my_vec.x + my_vec.y * my_vec.y + my_vec.z * my_vec.z));
}

// Operator Overloads //
//
// The vec/legQuad operators are templates, so they live in Common.hpp -- see
// the note there.