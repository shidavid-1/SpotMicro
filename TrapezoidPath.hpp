#ifndef TRAPEZOID_PATH_HPP
#define TRAPEZOID_PATH_HPP

#include "Common.hpp"
#include <vector>

// Class for an in-plane trapezoid
class TrapezoidPath {
    private:
        vec<float> origin;
        float perimeter;
        float height;
        float base_width;
        float prop;
        std::vector<vec<float>> points;
        std::vector<float> lengths;
        static const int numSides = 4;

    public:

        /* Trapezoid side length names
           intersection of d and a is the
           starting point of the cycle
                   c
            _______________
           /               \
        d /                 \ b
          -------------------
                   a

        */
        // Height is the distance between the bottom segment and the top segment
        // Base_width is the length of the bottom segment
        // Prop defines length of the top segment in terms of the bottom segment
        // top_width = base_width * prop
        // Origin is the midpoint of the bottom segment, from which everything
        // else is defined relative to
        // REQUIRES: prop < 1
        TrapezoidPath(vec<float> origin, float height, float base_width, float prop);

        // returns the point corresponding to "percent" of
        // the way along the perimeter of the trapezoid starting
        // at the point in the trapezoid with the lowest z coordinate 
        // and greatest y coordinate
        // REQUIRES: 0 <= percent < 1
        vec<float> percent_of_path(float percent) const;
};

#endif
