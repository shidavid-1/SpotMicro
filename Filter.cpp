///// Implement Complementary Filter /////

#include "Filter.hpp"

#include "Common.hpp"
#include "Spot_Model.hpp"
#include "IK.hpp"
#include <cmath>

// Pitch forward (i.e. nose down) gives negative theta_y
// Roll left (i.e. left shoulder down) gives negative theta_x
vec<float> calculate_orientation(const SpotModel * TPUP, const vec<float> angvel, const vec<float> accel) {
    vec<float> new_orientation;
    new_orientation.x = alpha * (angvel.x * dt + TPUP->get_orientation().x) + (1 - alpha) * ((float)atan2(accel.y, accel.z) / RAD_PER_DEG);
    new_orientation.y = alpha * (angvel.y * dt + TPUP->get_orientation().y) + (1 - alpha) * ((float)atan2(accel.x, accel.z) / RAD_PER_DEG);
    new_orientation.z = angvel.z * dt + TPUP->get_orientation().z;
    return new_orientation;
}

vec<float> calculate_acceleration(const vec<float> ori, const vec<float> accel) {
    vec<float> accel_no_grav;
    accel_no_grav.x = accel.x - gravity * cos(ori.x * RAD_PER_DEG);
    accel_no_grav.y = accel.y - gravity * sin(ori.x * RAD_PER_DEG);
    accel_no_grav.z = accel.z - gravity * sin(ori.y * RAD_PER_DEG);
    return accel_no_grav;
}