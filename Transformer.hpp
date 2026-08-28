///// Transformer /////
/*
   Transforms all of the body reference coordinates
   into each leg's reference frame
*/

#ifndef TRANSFORMER_HPP
#define TRANSFORMER_HPP

#include "Common.hpp"
#include "Spot_Model.hpp"

class Transformer {
   private:
      const SpotModel * TPUP;
      legQuad<vec<float>> next_points_per_leg;

      void change_origin(vec<float> * position, LegType type);

      void reflect_y(vec<float> * position);

   public:
      Transformer(const SpotModel * Tpup);
      
      void transform(legQuad<vec<float>> gait_output);

      vec<float> get_next_points_frontLeft() const;

      vec<float> get_next_points_frontRight() const;

      vec<float> get_next_points_backLeft() const;

      vec<float> get_next_points_backRight() const;

      legQuad<vec<float>> get_next_points() const;
};

#endif
