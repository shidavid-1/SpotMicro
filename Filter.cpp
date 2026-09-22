///// Implement Complementary Filter /////

#include "Filter.hpp"

#include "Common.hpp"
#include "Spot_Model.hpp"
#include <cmath>
#include "esp_timer.h"

using namespace std;

Filter::Filter(const float &T, const float &del_t, const float &threshold) 
    : Tau(T), alpha(0.0f), dt(del_t), grav_threshold(threshold),
      prev_time_stamp(0.0f), curr_time_stamp(0.0f) {
    if (Tau > 0.0f) {
        alpha = exp(-dt / Tau);
    }
    else {
        ESP_LOGE("FILTER ERROR", "Invalid Tau value of %f", Tau);
        abort();
    }
}

void Filter::update_alpha_dt() {
    prev_time_stamp = curr_time_stamp;
    curr_time_stamp = esp_timer_get_time();
    if (prev_time_stamp == 0) {
        prev_time_stamp = curr_time_stamp;
    }
    dt = (curr_time_stamp - prev_time_stamp) / (1e6f);
    alpha = exp(-dt / Tau);
}

// Pitch backward (i.e. nose up) gives negative theta_y
// Roll left (i.e. left shoulder down) gives negative theta_x
vec<float> Filter::calculate_orientation(const SpotModel * TPUP, const vec<float> angvel, const vec<float> accel) {
    vec<float> new_orientation;
    update_alpha_dt();
    
    // x is roll
    new_orientation.x = angvel.x * dt + TPUP->get_orientation().x;

    // Note that pitch is this formula because we're using the Euler angle convention ZYX, so pitch is always defined as the angle between
    // gcos(theta) and g. Additionally, the negative sign in front of accel.x is just to meet the sign convention of the RH system
    // y is pitch
    new_orientation.y = angvel.y * dt + TPUP->get_orientation().y;

    if (fabs(magnitude(accel) - gravity) < grav_threshold) {
        new_orientation.x *= alpha;
        new_orientation.x += (1 - alpha) * ((float)atan2(accel.y, accel.z) / RAD_PER_DEG);

        new_orientation.y *= alpha;
        new_orientation.y += (1 - alpha) * ((float)atan2(-accel.x, sqrt(accel.z * accel.z + accel.y * accel.y)) / RAD_PER_DEG);
    }
    
    // This measurement will drift over dog operation.
    new_orientation.z = angvel.z * dt + TPUP->get_orientation().z;
    return new_orientation;
}

vec<float> Filter::calculate_acceleration(const vec<float> ori, const vec<float> accel) {
    vec<float> accel_no_grav;
    accel_no_grav.x = accel.x + gravity * sin(ori.y * RAD_PER_DEG);
    accel_no_grav.y = accel.y - gravity * cos(ori.y * RAD_PER_DEG) * sin(ori.x * RAD_PER_DEG);
    accel_no_grav.z = accel.z - gravity * cos(ori.y * RAD_PER_DEG) * cos(ori.x * RAD_PER_DEG);
    return accel_no_grav;
}

vec<float> Filter::filter_IMU_data(SpotModel * TPUP, const vec<float> angvel, const vec<float> accel) {
    TPUP->set_orientation(calculate_orientation(TPUP, angvel, accel));
    return calculate_acceleration(TPUP->get_orientation(), accel);
}