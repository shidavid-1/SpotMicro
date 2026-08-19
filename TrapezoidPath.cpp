#include "Common.hpp"
#include <vector>
#include <cassert>
#include "esp_log.h"

#include "TrapezoidPath.hpp"

TrapezoidPath::TrapezoidPath(vec<float> origin, float height, float base_width, float prop) 
    : origin(origin), perimeter(0.0f), height(height), base_width(base_width), prop(prop) {
    assert(prop < 1.0f);
    vec<float> point = {origin.x, origin.y + base_width / 2.0f, origin.z};
    points.push_back(point);
    point = {origin.x, origin.y - base_width / 2.0f, origin.z};
    points.push_back(point);
    point = {origin.x, origin.y - prop * base_width / 2.0f, origin.z + height};
    points.push_back(point);
    point = {origin.x, origin.y + prop * base_width / 2.0f, origin.z + height};
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
    assert(0.0f <= percent && percent < 1.0f);

    float a = lengths.at(0);
    float b = lengths.at(1);
    float c = lengths.at(2);
    float d = lengths.at(3);
    float p = perimeter;
    
    // if in segment a
    if (0 <= percent && percent < a / p) {
        float p_a = (p * percent) / a;
        return {
            points.at(0).x,
            points.at(0).y - p_a * a,
            points.at(0).z
        };
    }

    // if in segment b
    else if (a / p <= percent && percent < (a + b) / p) {
        float p_b = (p * percent - a) / b;
        return {
            points.at(1).x,
            points.at(1).y + p_b * (1 - prop) * base_width / 2.0f,
            points.at(1).z + p_b * height
        };
    }

    // if in segment c
    else if ((a + b) / p <= percent && percent < (a + b + c) / p) {
        float p_c = (p * percent - a - b) / c;
        return {
            points.at(2).x,
            points.at(2).y + p_c * c,
            points.at(2).z
        };
    }

    // if in segment d
    else if ((a + b + c) / p <= percent && percent < 1) {
        float p_d = (p * percent - a - b - c) / d;
        return {
            points.at(3).x,
            points.at(3).y + p_d * (1 - prop) * base_width / 2.0f,
            points.at(0).z + (1 - p_d) * height
        };
    }

    else {
        ESP_LOGE("TRAPEZOID PATH ERROR", "Invalid percent entered.");
        abort();
    }
}

