///// Gait Controller /////
/*
   Computes all next foot tip coordinates in
   the body reference frame based on SpotModel.
*/

#ifndef GAIT_CONTROLLER_HPP
#define GAIT_CONTROLLER_HPP

#include "Common.hpp"
#include "Spot_Model.hpp"
#include "TrapezoidPath.hpp"

// Class responsible for managing phi
// taking input parameters for the gait

/* NEED TO UPDATE WITH ACTUAL VALUES */
// all values in mm
constexpr float MAX_BODY_HEIGHT = 500.0f;    // max height of body origin above ground
constexpr float MAX_STEP_HEIGHT = 500.0f;    // max step height
constexpr float MAX_STRIDE_LENGTH = 300.0f;  // max length of stride before paths collide
constexpr float MAX_VELOCITY = 100.0f;       // mm/s

constexpr float MIN_BODY_HEIGHT = 10.0f;    // max height of body origin above ground
constexpr float MIN_STEP_HEIGHT = 10.0f;    // max step height
constexpr float MIN_STRIDE_LENGTH = 10.0f;  // max length of stride before paths collide
constexpr float MIN_VELOCITY = 0.0f;        // mm/s, exclusive: a zero velocity
                                            // would make the gait period infinite

// unitless, [inclusive, exclusive) bounds
constexpr float MIN_DUTY_FACTOR = 0.5f;      // below this the trot has a flight phase
constexpr float MAX_DUTY_FACTOR = 1.0f;      // at this the foot never leaves the ground
constexpr float MIN_PROP = 0.0f;             // at this the trapezoid degenerates to a triangle


struct GaitParams {
   float body_height;
   float step_height;
   float stride_length;
   float velocity;
   float duty_factor;
   float prop;          // See trapezoidpath.hpp for definition
};

// invariant: 0 <= phi < 1

class GaitController {
   private:
      GaitParams params;
      legQuad<vec<float>> tip_origins;    // origin from which each trapezoid is define
                                                // note that this is different from origins in leg
      legQuad<TrapezoidPath> paths;             // path each leg takes
      const legQuad<float> offset = {0.0f, 0.5f, 0.5f, 0.0f};
      legQuad<vec<float>> output_points;
      float phi;           // gait phase variable, drives gait controller
      float gait_period;   // amount of time in seconds for a leg to fully trace its path
      float phi_increment; // amount to increment phi by for the given control loop frequency
                           // in order to make sure foot stays in stance phase for appropriate amt of time

      // MODIFIES: gait_period, phi_increment
      // EFFECTS: Recomputes the cycle timing from the current params. Must be
      // called after any change to params.
      void update_timing();

      // EFFECTS: Returns the path belonging to leg. A switch rather than a
      // chain of ifs so that -Wswitch flags this function if a leg is ever
      // added to LegType.
      const TrapezoidPath & path_for(LegType leg) const;

      // REQUIRES: 0 <= ph < 1
      // EFFECTS: Maps a gait phase onto the fraction of leg's path perimeter
      // the foot should have reached at that phase. Splits the cycle by time
      // first -- duty_factor of it on the base segment, the remainder spread
      // across the other three by arc length -- so that stance lasts exactly
      // as long as duty_factor asks for. Without this the stance share would
      // be fixed by the trapezoid's proportions and duty_factor would have no
      // effect. Takes the leg rather than a stance fraction so that each leg
      // is timed off its own path, which matters once legs can differ (e.g. a
      // longer stride on the outside of a turn).
      float phi_to_percent(float ph, LegType leg) const;

   public:
      
      GaitController(GaitParams p);
      void compute_next_points();
      float add_phi_offset(float off) const;
      void increment_phi();
      void reset_phi();
      float get_phi() const;

      // EFFECTS: Returns the foot tip points computed by the most recent call
      // to compute_next_points.
      legQuad<vec<float>> get_output_points() const;

      // REQUIRES: p passes validate_params (aborts otherwise)
      // MODIFIES: params, tip_origins, paths, gait_period, phi_increment
      // EFFECTS: Adopts a new set of gait parameters and rebuilds every value
      // derived from them. phi is deliberately left alone so the gait stays
      // continuous across a parameter change.
      void set_params(GaitParams p);
};

#endif