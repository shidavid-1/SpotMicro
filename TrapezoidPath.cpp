#include "Common.hpp"
#include <vector>
#include "esp_log.h"

#include "TrapezoidPath.hpp"

TrapezoidPath::TrapezoidPath(vec<float> origin, float height, float base_width, float prop) 
    : origin(origin), perimeter(0.0f), height(height), base_width(base_width), prop(prop) {
    TPUP_CHECK(prop > 0.0f, "NEGATIVE PROPORTION IN TRAPEZOID PATH");
    TPUP_CHECK(height > 0.0f, "NEGATIVE HEIGHT IN TRAPEZOID PATH");
    vec<float> point = {origin.x + base_width / 2.0f, origin.y, origin.z};
    points.push_back(point);
    point = {origin.x - base_width / 2.0f, origin.y, origin.z};
    points.push_back(point);
    point = {origin.x - prop * base_width / 2.0f, origin.y, origin.z + height};
    points.push_back(point);
    point = {origin.x + prop * base_width / 2.0f, origin.y, origin.z + height};
    points.push_back(point);

    lengths.push_back(distance_between(points.at(0), points.at(1)));
    lengths.push_back(distance_between(points.at(1), points.at(2)));
    lengths.push_back(distance_between(points.at(2), points.at(3)));
    lengths.push_back(distance_between(points.at(3), points.at(0)));

    for (int i = 0; i < 4; ++i) {
        perimeter += lengths.at(i);
    }
}

vec<float> TrapezoidPath::percent_of_path(float percent) const {

    float a = lengths.at(0);
    float b = lengths.at(1);
    float c = lengths.at(2);
    float d = lengths.at(3);
    float p = perimeter;

    // Convert percent into a point along the trapezoid
    
    // if in segment a
    if (0 <= percent && percent < a / p) {
        float p_a = (p * percent) / a;
        return {
            points.at(0).x - p_a * a,
            points.at(0).y,
            points.at(0).z
        };
    }

    // if in segment b
    else if (a / p <= percent && percent < (a + b) / p) {
        float p_b = (p * percent - a) / b;
        return {
            points.at(1).x + p_b * (points.at(2).x - points.at(1).x),
            points.at(1).y,
            points.at(1).z + p_b * height
        };
    }

    // if in segment c
    else if ((a + b) / p <= percent && percent < (a + b + c) / p) {
        float p_c = (p * percent - a - b) / c;
        return {
            points.at(2).x + p_c * c,
            points.at(2).y,
            points.at(2).z
        };
    }

    // if in segment d
    else if ((a + b + c) / p <= percent && percent < 1) {
        float p_d = (p * percent - a - b - c) / d;
        return {
            points.at(3).x + p_d * (points.at(0).x - points.at(3).x),
            points.at(3).y,
            points.at(3).z - p_d * height
        };
    }

    else {
        ESP_LOGE("TRAPEZOID PATH ERROR", "Invalid percent entered.");
        abort();
    }
}

float TrapezoidPath::stance_fraction() const {
    return lengths.at(0) / perimeter;
}

vec<float> TrapezoidPath::get_bottomLeft_point() const {
    return points.at(0);
}
vec<float> TrapezoidPath::get_bottomRight_point() const{
    return points.at(1);
}
vec<float> TrapezoidPath::get_topRight_point() const {
    return points.at(2);
}
vec<float> TrapezoidPath::get_topLeft_point() const {
    return points.at(3);
}

