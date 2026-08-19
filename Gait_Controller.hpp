///// Gait Controller /////
/*
   Computes all next foot tip coordinates in
   the body reference frame based on SpotModel.
*/

#ifndef GAIT_CONTROLLER_HPP
#define GAIT_CONTROLLER_HPP

#include "Common.hpp"
#include "Spot_Model.hpp"

struct GaitParams {
   float body_height;
   float duty_factor;
   float step_height;
   float stride_length;
   float velocity;
};

class GaitController {
   private:
      GaitParams params;
      legQuad<TrapezoidPath> paths;             // path each leg takes
      const legQuad<vec<float>> tip_origins;    // origin from which each trapezoid is define
                                                // note that this is different from origins in leg
      const legQuad<float> offset = {0.0f, 0.5f, 0.5f, 0.0f};
      legQuad<vec<float>> output_points;
      float phi;
   
   public:
      
      GaitController(GaitParams params);
      void compute_next_points();
      float phi_to_percent(float phi) const;
      float increment_phi(float delta);
      float set_phi(float new_phi);
      float get_phi() const;
};

#endif