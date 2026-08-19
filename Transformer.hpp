///// Transformer /////
/*
   Transforms all of the body reference coordinates
   into each leg's reference frame
*/

#ifndef TRANSFORMER_HPP
#define TRANSFORMER_HPP

#include "Common.hpp"
#include "Spot_Model.hpp"

legQuad<vec<float>> transform(legQuad<vec<float>> gait_output, SpotModel * TPUP);

#endif
