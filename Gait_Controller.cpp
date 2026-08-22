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
   if (p.stride_length > MAX_STRIDE_LENGTH) {
      ESP_LOGE(TAG, "Stride length %.1f mm exceeds the %.1f mm maximum; paths would collide.",
               p.stride_length, MAX_STRIDE_LENGTH);
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

// Builds the point each leg's trapezoid is defined about. Factored out of the
// constructor so that set_params rebuilds them exactly the same way.
static legQuad<vec<float>> make_tip_origins(const GaitParams &p) {
   return { /* NEEDS TO BE UPDATED WITH ACTUAL VALUES */
      {1.0f, 1.0f, -p.body_height},
      {1.0f, 1.0f, -p.body_height},
      {1.0f, 1.0f, -p.body_height},
      {1.0f, 1.0f, -p.body_height},
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

   update_timing();
}

void GaitController::update_timing() {
   gait_period = params.stride_length / (params.velocity * params.duty_factor);
   phi_increment = dt / gait_period;
}

void GaitController::set_params(GaitParams p) {
   params = validate_params(p);
   tip_origins = make_tip_origins(params);
   paths = make_paths(tip_origins, params);
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

void GaitController::reset_phi() {
   phi = 0.0f;
}

float GaitController::get_phi() const {
   return phi;
}

legQuad<vec<float>> GaitController::get_output_points() const {
   return output_points;
}
