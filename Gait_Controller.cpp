///// Gait Controller /////
/*
   Computes all next foot tip coordinates in
   the body reference frame based on SpotModel.
*/

#include "Gait_Controller.hpp"

#include "Common.hpp"
#include "Spot_Model.hpp"
#include <vector>
#include "TrapezoidPath.hpp"

// Class responsible for managing phi
// taking input parameters for the gait

struct GaitParams {
   float body_height;
   float step_height;
   float stride_length;
   float velocity;
   float duty_factor;
   float prop;          // See trapezoidpath.hpp for definition
};

class GaitController {
   private:
      GaitParams params;
      legQuad<vec<float>> tip_origins;    // origin from which each trapezoid is define
                                                // note that this is different from origins in leg
      legQuad<TrapezoidPath> paths;             // path each leg takes
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

GaitController::GaitController(GaitParams params)
   : params(params),
     tip_origins({ /* NEEDS TO BE UPDATED WITH ACTUAL VALUES */
      {1.0f, 1.0f, -params.body_height},
      {1.0f, 1.0f, -params.body_height},
      {1.0f, 1.0f, -params.body_height},
      {1.0f, 1.0f, -params.body_height},
     }),
     paths({
      TrapezoidPath(tip_origins.frontLeft, params.step_height, params.stride_length, params.prop),
      TrapezoidPath(tip_origins.frontRight, params.step_height, params.stride_length, params.prop),
      TrapezoidPath(tip_origins.backLeft, params.step_height, params.stride_length, params.prop),
      TrapezoidPath(tip_origins.backRight, params.step_height, params.stride_length, params.prop),
     }),
     phi(0.0f)
{
   /* ERROR CHECKING */
   if (body_height < -MAX_BODY_HEIGHT) {
      // CONTINUE HERE, INTRODUCE CONSTANTS FOR MAX BODY HEIGHT AND SUCH TO PREMATURELY RULE OUT OBVIOUSLY BAD GAITS
   }
}
