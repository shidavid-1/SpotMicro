///// Implement Complementary Filter /////

#ifndef FILTER_HPP
#define FILTER_HPP

#include "Common.hpp"
#include "Spot_Model.hpp"

class Filter {
    private:
        const float Tau = 4.26;
        float alpha;
        float dt;              // in seconds
        float grav_threshold;
        int64_t prev_time_stamp; // in microseconds
        int64_t curr_time_stamp; // in microseconds

        vec<float> calculate_orientation(const SpotModel * TPUP, const vec<float> angvel, const vec<float> accel);
        vec<float> calculate_acceleration(const vec<float> ori, const vec<float> accel);
        void update_alpha_dt();
        
    public:
        Filter(const float &del_t, const float &grav_threshold);

        // Updates orientation in TPUP and returns acceleration as a vec<Float>
        vec<float> filter_IMU_data(SpotModel * TPUP, const vec<float> angvel, const vec<float> accel);
};

#endif