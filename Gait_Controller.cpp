///// Gait Controller /////
/*
   Computes all next foot tip coordinates in
   the body reference frame based on SpotModel.
*/

#include "Gait_Controller.hpp"

#include "Common.hpp"
#include "Spot_Model.hpp"
#include "TrapezoidPath.hpp"
#include "esp_log.h"
#include <cmath>
#include <cstdlib>

static const char *TAG = "GAIT ERROR";

const TrapezoidPath & GaitController::path_for(LegType leg) const {
    switch (leg) {
        case LegType::frontLeft:  return paths.frontLeft;
        case LegType::frontRight: return paths.frontRight;
        case LegType::backLeft:   return paths.backLeft;
        case LegType::backRight:  return paths.backRight;
    }

    // Only reachable if an out-of-range value was cast to LegType.
    ESP_LOGE(TAG, "Invalid leg type.");
    abort();
}

float GaitController::phi_to_percent(float ph, LegType leg) const {

    const float stance = path_for(leg).stance_fraction();

    if (0.0f <= ph && ph < params.duty_factor) {
        return ph / params.duty_factor * stance;
    }

    else if (params.duty_factor <= ph && ph < 1.0f) {
        return (ph - params.duty_factor) / (1 - params.duty_factor) * (1 - stance) + stance;
    }

    else {
        ESP_LOGE("PHI ERROR", "Invalid phi value of %.3f entered.", ph);
        abort();
    }
}

// Aborts if any gait parameter is outside its safe range. Returns the parameters
// unchanged so that it can be called from the member initializer list: "params" is
// declared first, so every check runs before any TrapezoidPath is constructed.
static GaitParams validate_params(const GaitParams &p) {
   if (!std::isfinite(p.body_height) || !std::isfinite(p.step_height) ||
       !std::isfinite(p.stride_length) || !std::isfinite(p.velocity) ||
       !std::isfinite(p.duty_factor) || !std::isfinite(p.prop)) {
      ESP_LOGE(TAG, "Gait parameter is NaN or infinite.");
      abort();
   }

   if (p.body_height < MIN_BODY_HEIGHT) {
      ESP_LOGE(TAG, "Body height %.1f mm is below the %.1f mm minimum.", p.body_height, MIN_BODY_HEIGHT);
      abort();
   }
   if (p.body_height > MAX_BODY_HEIGHT) {
      ESP_LOGE(TAG, "Body height %.1f mm exceeds the %.1f mm maximum.", p.body_height, MAX_BODY_HEIGHT);
      abort();
   }

   if (p.step_height < MIN_STEP_HEIGHT) {
      ESP_LOGE(TAG, "Step height %.1f mm is below the %.1f mm minimum.", p.step_height, MIN_STEP_HEIGHT);
      abort();
   }
   if (p.step_height > MAX_STEP_HEIGHT) {
      ESP_LOGE(TAG, "Step height %.1f mm exceeds the %.1f mm maximum.", p.step_height, MAX_STEP_HEIGHT);
      abort();
   }
   if (p.step_height > p.body_height) {
      ESP_LOGE(TAG, "Step height %.1f mm is not less than body height %.1f mm; foot tip would rise above the body.",
               p.step_height, p.body_height);
      abort();
   }

   if (p.stride_length < MIN_STRIDE_LENGTH) {
      ESP_LOGE(TAG, "Stride length %.1f mm is below the %.1f mm minimum.", p.stride_length, MIN_STRIDE_LENGTH);
      abort();
   }

   if (p.velocity <= MIN_VELOCITY) {
      ESP_LOGE(TAG, "Velocity %.1f mm/s is at or below the %.1f mm/s minimum.", p.velocity, MIN_VELOCITY);
      abort();
   }
   if (p.velocity > MAX_VELOCITY) {
      ESP_LOGE(TAG, "Velocity %.1f mm/s exceeds the %.1f mm/s maximum.", p.velocity, MAX_VELOCITY);
      abort();
   }

   if (p.duty_factor < MIN_DUTY_FACTOR) {
      ESP_LOGE(TAG, "Duty factor %.3f is below %.3f; gait would have a flight phase.",
               p.duty_factor, MIN_DUTY_FACTOR);
      abort();
   }
   if (p.duty_factor >= MAX_DUTY_FACTOR) {
      ESP_LOGE(TAG, "Duty factor %.3f is not less than %.3f; foot would never leave the ground.",
               p.duty_factor, MAX_DUTY_FACTOR);
      abort();
   }

   if (p.prop <= MIN_PROP) {
      ESP_LOGE(TAG, "Prop %.3f is not greater than %.3f; trapezoid has no top segment.", p.prop, MIN_PROP);
      abort();
   }

   return p;
}

// Builds the point each leg's trapezoid is defined about. 
// This needs to be done in the global FOR, thus the addition
// of the leg origin values
static legQuad<vec<float>> make_tip_origins(const GaitParams &p) {
   return { /* NEEDS TO BE UPDATED WITH ACTUAL VALUES */
      {global_leg_origins.frontLeft.x + leg_tip_origins_xy.first, global_leg_origins.frontLeft.y + leg_tip_origins_xy.second, -p.body_height},
      {global_leg_origins.frontRight.x + leg_tip_origins_xy.first, global_leg_origins.frontRight.y - leg_tip_origins_xy.second, -p.body_height},
      {global_leg_origins.backLeft.x + leg_tip_origins_xy.first, global_leg_origins.backLeft.y + leg_tip_origins_xy.second, -p.body_height},
      {global_leg_origins.backRight.x + leg_tip_origins_xy.first, global_leg_origins.backRight.y - leg_tip_origins_xy.second, -p.body_height}
   };
}

// Builds one path per leg. Factored out for the same reason as above.
static legQuad<TrapezoidPath> make_paths(const legQuad<vec<float>> &origins, const GaitParams &p) {
   return {
      TrapezoidPath(origins.frontLeft, p.step_height, p.stride_length, p.prop),
      TrapezoidPath(origins.frontRight, p.step_height, p.stride_length, p.prop),
      TrapezoidPath(origins.backLeft, p.step_height, p.stride_length, p.prop),
      TrapezoidPath(origins.backRight, p.step_height, p.stride_length, p.prop),
   };
}

GaitController::GaitController(GaitParams p)
   : params(validate_params(p)),
     tip_origins(make_tip_origins(params)),
     paths(make_paths(tip_origins, params)),
     phi(0.0f) {

   TPUP_CHECK(!do_leg_paths_intersect(), "Leg Path Intersection Error");
   update_timing();
}

void GaitController::update_timing() {
   gait_period = params.stride_length / (params.velocity * params.duty_factor);
   phi_increment = ideal_dt / gait_period;
}

void GaitController::set_params(GaitParams p) {
   params = validate_params(p);
   tip_origins = make_tip_origins(params);
   paths = make_paths(tip_origins, params);
   TPUP_CHECK(!do_leg_paths_intersect(), "Leg Path Intersection Error");
   update_timing();
}


void GaitController::compute_next_points() {
   output_points.frontLeft = paths.frontLeft.percent_of_path(
      phi_to_percent(add_phi_offset(offset.frontLeft), LegType::frontLeft));
   output_points.frontRight = paths.frontRight.percent_of_path(
      phi_to_percent(add_phi_offset(offset.frontRight), LegType::frontRight));
   output_points.backLeft = paths.backLeft.percent_of_path(
      phi_to_percent(add_phi_offset(offset.backLeft), LegType::backLeft));
   output_points.backRight = paths.backRight.percent_of_path(
      phi_to_percent(add_phi_offset(offset.backRight), LegType::backRight));
}

void GaitController::increment_phi() {
   TPUP_CHECK(0.0f <= phi_increment && phi_increment < 1.0f, TAG);
   TPUP_CHECK(0.0f <= phi && phi < 1.0f, TAG);

   if (phi + phi_increment >= 1.0f) {
      phi = phi + phi_increment - 1.0f;
   }

   else {
      phi = phi + phi_increment;
   }
}

// REQUIRES: 0 <= offset < 1.0, phi invariant holds
// before and after
// MODIFIES: NOTHING
// EFFECTS: returns phi + offset with wraparound.
float GaitController::add_phi_offset(float off) const {
   TPUP_CHECK(0.0f <= off && off < 1.0f, TAG);
   TPUP_CHECK(0.0f <= phi && phi < 1.0f, TAG);

   if (phi + off >= 1.0f) {
      return phi + off - 1.0f;
   }

   else {
      return phi + off;
   }
}

// REQUIRES: tolerance >= 0 (if tolerance is set to negative number, the
// tolerance used will just be the magnitude of that number)
// EFFECTS: Computes distance between rearmost points of both front paths
// and frontmost points of both rear paths to ensure that the distance between
// them is > tolerance + 2 * foot_tip_radius;
bool GaitController::do_leg_paths_intersect() const {

   // Check if any of the rear points are farther in front of the front points //
   if (paths.backLeft.get_bottomLeft_point().x > paths.frontLeft.get_bottomRight_point().x ||
       paths.backLeft.get_topLeft_point().x > paths.frontLeft.get_topRight_point().x) {
         return true;
   }

   if (paths.backRight.get_bottomLeft_point().x > paths.frontRight.get_bottomRight_point().x ||
       paths.backRight.get_topLeft_point().x > paths.frontRight.get_topRight_point().x) {
         return true;
   }

   // Check if left side paths intersect //
   if (distance_between(paths.frontLeft.get_bottomRight_point(), paths.backLeft.get_bottomLeft_point()) <= tolerance + 2 * FOOTTIP_RADIUS ||
       distance_between(paths.frontLeft.get_bottomRight_point(), paths.backLeft.get_topLeft_point()) <= tolerance + 2 * FOOTTIP_RADIUS) {
         return true;
   }

   if (distance_between(paths.frontLeft.get_topRight_point(), paths.backLeft.get_bottomLeft_point()) <= tolerance + 2 * FOOTTIP_RADIUS ||
       distance_between(paths.frontLeft.get_topRight_point(), paths.backLeft.get_topLeft_point()) <= tolerance + 2 * FOOTTIP_RADIUS) {
         return true;
   }

   // Check if right side paths intersec //
   if (distance_between(paths.frontRight.get_bottomRight_point(), paths.backRight.get_bottomLeft_point()) <= tolerance + 2 * FOOTTIP_RADIUS ||
       distance_between(paths.frontRight.get_bottomRight_point(), paths.backRight.get_topLeft_point()) <= tolerance + 2 * FOOTTIP_RADIUS) {
         return true;
   }

   if (distance_between(paths.frontRight.get_topRight_point(), paths.backRight.get_bottomLeft_point()) <= tolerance + 2 * FOOTTIP_RADIUS ||
       distance_between(paths.frontRight.get_topRight_point(), paths.backRight.get_topLeft_point()) <= tolerance + 2 * FOOTTIP_RADIUS) {
         return true;
   }

   return false;
}

void GaitController::reset_phi() {
   phi = 0.0f;
}

float GaitController::get_phi() const {
   return phi;
}

legQuad<vec<float>> GaitController::get_output_points() const {
   return output_points;
}
