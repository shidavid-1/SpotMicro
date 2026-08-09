///// Transformer /////
/*
   Transforms all of the body reference coordinates
   into each leg's reference frame
*/

#ifndef TRANSFORMER_HPP
#define TRANSFORMER_HPP

#include "Common.hpp"

legQuad<vec<float>> transform(legQuad<vec<float>> gait_output);

static vec<float> change_origin(const vec<float> position, LegType type);

static vec<float> reflect(const vec<float> position);

#endif