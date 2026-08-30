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
                ^ z
                |
                |
            <---O y (out of screen)
            x

            Note that the "bottom left" point is the intersection of d and a
            "Bottom right" is the intersection of b and a
            "top right" is the intersection of c and b
            "top left" is the intersection of c and d

            Note that the datum is the robot's body datum rather than any particular
            leg datum.

        */
        // Height is the distance between the bottom segment and the top segment
        // Base_width is the length of the bottom segment
        // Prop defines length of the top segment in terms of the bottom segment
        // top_width = base_width * prop
        // Origin is the midpoint of the bottom segment, from which everything
        // else is defined relative to
        // Note that in gaitcontroller, the origin is defined in the global reference frame
        // REQUIRES: prop > 0
        TrapezoidPath(vec<float> origin, float height, float base_width, float prop);

        // REQUIRES: 0 <= percent < 1
        // EFFECTS: Returns the point "percent" of the way along the perimeter
        // of the trapezoid, measured by arc length, starting at the corner with
        // the lowest z coordinate and greatest x coordinate (where d meets a)
        // and travelling along a in the -x direction.
        // Note that this is parameterized by distance, not by time: sweeping
        // percent linearly moves the point at a constant speed. A caller that
        // needs a non-uniform speed along the path is responsible for its own
        // mapping onto percent.
        vec<float> percent_of_path(float percent) const;

        // EFFECTS: Returns side a's share of the perimeter, i.e. the fraction
        // of a full traversal spent on the base segment.
        float stance_fraction() const;

        vec<float> get_bottomLeft_point() const;
        vec<float> get_bottomRight_point() const;
        vec<float> get_topRight_point() const;
        vec<float> get_topLeft_point() const;
        std::vector<vec<float>> get_all_points() const;
};

#endif
