///// Transformer /////
/*
   Transforms all of the body reference coordinates
   into each leg's reference frame
*/

#include "Common.hpp"

#ifndef TRANSFORMER_HPP
#define TRANSFORMER_HPP

legQuad<vec<float>> transform(const legQuad<vec<float>> gait_output);

#endif