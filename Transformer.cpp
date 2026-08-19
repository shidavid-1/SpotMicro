///// Transformer /////
/*
   Transforms all of the body reference coordinates
   into each leg's reference frame
*/

#include "Transformer.hpp"

#include "Common.hpp"
#include "Spot_Model.hpp"

// Internal helpers: declared here rather than in the header so each including
// translation unit does not get its own unused copy.
static void change_origin(vec<float> * position, LegType type, const SpotModel * TPUP);

static void reflect_y(vec<float> * position);

legQuad<vec<float>> transform(legQuad<vec<float>> gait_output, SpotModel * TPUP) {
   change_origin(&(gait_output.frontLeft), LegType::frontLeft, TPUP);
   change_origin(&(gait_output.frontRight), LegType::frontRight, TPUP);
   change_origin(&(gait_output.backLeft), LegType::backLeft, TPUP);
   change_origin(&(gait_output.backRight), LegType::backRight, TPUP);

   reflect_y(&(gait_output.frontRight));
   reflect_y(&(gait_output.backRight));

   return gait_output;
}

static void change_origin(vec<float> * position, LegType type, const SpotModel * TPUP) {
   switch(type) {
      case LegType::frontLeft:   *position = *position - TPUP->get_origins().frontLeft;  break;
      case LegType::frontRight:  *position = *position - TPUP->get_origins().frontRight; break;
      case LegType::backLeft:    *position = *position - TPUP->get_origins().backLeft;   break;
      case LegType::backRight:   *position = *position - TPUP->get_origins().backRight;  break;
   }
}

static void reflect_y(vec<float> * position) {
   position->y *= -1;
}
