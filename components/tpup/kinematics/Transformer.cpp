///// Transformer /////
/*
   Transforms all of the body reference coordinates
   into each leg's reference frame
*/

#include "Transformer.hpp"

#include "Common.hpp"
#include "Spot_Model.hpp"

void Transformer::change_origin(vec<float> * position, LegType type) {
   switch(type) {
      case LegType::frontLeft:   *position = *position - TPUP->get_origins().frontLeft;  break;
      case LegType::frontRight:  *position = *position - TPUP->get_origins().frontRight; break;
      case LegType::backLeft:    *position = *position - TPUP->get_origins().backLeft;   break;
      case LegType::backRight:   *position = *position - TPUP->get_origins().backRight;  break;
   }
}

void Transformer::reflect_y(vec<float> * position) {
   position->y *= -1;
}

Transformer::Transformer(const SpotModel * Tpup)
   : TPUP(Tpup), next_points_per_leg({}) {}

void Transformer::transform(legQuad<vec<float>> gait_output) {
   change_origin(&(gait_output.frontLeft), LegType::frontLeft);
   change_origin(&(gait_output.frontRight), LegType::frontRight);
   change_origin(&(gait_output.backLeft), LegType::backLeft);
   change_origin(&(gait_output.backRight), LegType::backRight);

   reflect_y(&(gait_output.frontRight));
   reflect_y(&(gait_output.backRight));

   next_points_per_leg = gait_output;
}

vec<float> Transformer::get_next_points_frontLeft() const {
   return next_points_per_leg.frontLeft;
}

vec<float> Transformer::get_next_points_frontRight() const {
   return next_points_per_leg.frontRight;
}

vec<float> Transformer::get_next_points_backLeft() const {
   return next_points_per_leg.backLeft;
}

vec<float> Transformer::get_next_points_backRight() const {
   return next_points_per_leg.backRight;
}

legQuad<vec<float>> Transformer::get_next_points() const {
   return next_points_per_leg;
}
