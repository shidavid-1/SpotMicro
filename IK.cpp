///// Inverse Kinematics for Single Leg /////
/*
   Computes angles to reach a particular position
   from within the reference frame of the leg.
*/

#include "IK.hpp"

#include "Common.hpp"       // leg geometry constants

#include <cmath>
#include <utility>

// Helpers //
namespace {

// EFFECTS: Clamps v into [-1, 1] so that inverse trig calls
// stay in their domain when floating point error (or an
// unreachable target) pushes an argument slightly past it.
inline float clamp_unit(float v) {
    if (v >  1.0f) return  1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
}

// EFFECTS: Degree-valued inverse trig / trig, matching the
// derivation, which is written entirely in degrees.
inline float acos_deg(float v)   { return std::acos(clamp_unit(v)) / RAD_PER_DEG; }
inline float asin_deg(float v)   { return std::asin(clamp_unit(v)) / RAD_PER_DEG; }
inline float cos_deg(float deg)  { return std::cos(deg * RAD_PER_DEG); }
inline float sin_deg(float deg)  { return std::sin(deg * RAD_PER_DEG); }

} // namespace

// REQUIRES: pos is a triplet where
// the elements represent
// the <x, y, z> coordinates, respectively
// EFFECTS: Computes the respective angles (degrees) to
// reach the position passed as a parameter.
// If the point isn't reachable, theta_1 is still computed
// and the leg is sent to the closest pose it can hold:
// completely extended if the point is too far away, and
// completely folded and tucked under the body if the point
// is nearer the shoulder than the leg can fold.
// Nothing is thrown; the returned bool is true when the
// target was reached exactly and false when it is one of
// those approximations.
std::pair<triplet<float>, bool> position_to_angles(const vec<float> &pos, const LegType lType) {
    // Left/right mirroring is applied upstream by the Transformer
    // (reflect_y), so every leg reaches IK in the same frame.
    (void)lType;

    // Leg geometry, named to match the derivation.
    constexpr float a = UPPERLEG_LENGTH;  // upper leg
    constexpr float b = FORELEG_LENGTH;   // fore leg
    constexpr float f = HIP_OFFSET;       // hip offset
    constexpr float g = FOOTTIP_RADIUS;   // foot tip radius
    constexpr float d = SHOULDER_LENGTH;  // shoulder length

    bool reachable = true;

    // Desired foot tip. The commanded z is the contact point, so lift it
    // by the tip radius to get the center of the foot tip sphere.
    const float e_xi = pos.x;
    const float e_yi = pos.y;
    const float e_zi = pos.z + g;
    const float e_x  = e_xi;   // x is never clamped

    // The shoulder joint always sits a distance d from the hip axis, so no
    // point closer than d (measured in the y-z plane) is reachable. Push
    // such a target radially outward onto that circle.
    const float yz_radius = std::sqrt(e_yi * e_yi + e_zi * e_zi);

    // Target sits exactly on the hip rotation axis. Every hip angle projects
    // to that same point, so there is no radial direction to preserve and the
    // equations above are genuinely undefined (they evaluate 0/0). Command a
    // fixed tucked pose instead.
    if (yz_radius == 0.0f) {
        return {{0.0f, 0.0f, 0.0f}, false};
    }

    const bool yz_clamped = (yz_radius < d);

    float e_y = e_yi;
    float e_z = e_zi;
    float r   = yz_radius;   // radius actually used, after any clamping

    if (yz_clamped) {
        reachable = false;
        r   = d;
        e_y = d * e_yi / yz_radius;
        e_z = d * e_zi / yz_radius;
    }

    // Hip rotation. r >= d > 0 always, so neither quotient below blows up.
    const float from_below = acos_deg(-e_z / r);   // angle off straight-down

    const float shoulder_lead = asin_deg(d / r);

    float theta_1 = (e_y >= 0.0f)
                  ?  from_below - shoulder_lead
                  : -from_below - shoulder_lead;

    // from_below is in [0,180] and shoulder_lead in [0,90], so theta_1 lands
    // in [-270, 90]. Wrap it into (-180, 180] to keep the hip angle in one
    // canonical range. Without this, the two sides of e_y == 0 name the same
    // pose 360 apart whenever the target is above the hip axis, and the servo
    // mapping (-theta_1 + 90) would clamp them to opposite ends of travel.
    // NOTE: this relocates the seam to theta_1 = +-180 rather than removing
    // it -- an angle cannot be single-valued and continuous everywhere. Every
    // theta_1 seam, before and after, is a pure 360 wrap (at e_y == 0 the
    // acos argument is exactly +-1, so the two branches differ by 0 or 360),
    // so the pose is continuous throughout; only the labelling jumps.
    if (theta_1 <= -180.0f) {
        theta_1 += 360.0f;
    }

    const float sin_t1 = sin_deg(theta_1);
    const float cos_t1 = cos_deg(theta_1);

    // Hip origin -> shoulder joint, once the hip has rotated by theta_1.
    const float d_x = 0.0f;
    const float d_y =  f * sin_t1 + d * cos_t1;
    const float d_z = -f * cos_t1 + d * sin_t1;

    // Shoulder joint -> foot tip. c is the chord the upper leg and
    // foreleg have to span together.
    const float c_x = e_x - d_x;
    const float c_y = e_y - d_y;
    const float c_z = e_z - d_z;
    const float c   = std::sqrt(c_x * c_x + c_y * c_y + c_z * c_z);

    // "Down" inside the leg plane is v = (sin t1, -cos t1) in the y-z plane;
    // it tilts with the hip as theta_1 changes. c_w is the chord's component
    // along it, so c_w < 0 means the foot tip sits above the shoulder joint
    // *within that plane* -- which is the question the branch below is asking.
    // Testing e_z > d_z instead measures the same thing against world z, and
    // the two only agree when theta_1 is 0.
    const float c_w = c_y * sin_t1 - c_z * cos_t1;

    const float cx_over_c = c_x / c;

    // Law of cosines across the knee.
    const float knee_cos = (a * a + b * b - c * c) / (2.0f * a * b);

    float theta_2 = 0.0f;
    float theta_3 = 0.0f;

    if (knee_cos > 1.0f) {
        // Target is nearer the shoulder than the leg's minimum span |a - b|,
        // so the leg cannot fold tightly enough to touch it. Curl up as far
        // as the knee goes and tuck the leg under the body.
        reachable = false;
        theta_2   = 0.0f;
        theta_3   = 0.0f;
    }
    else if (knee_cos < -1.0f) {
        // Target is farther than the leg's maximum span a + b. Extend fully
        // and aim the straightened leg at it.
        reachable = false;
        theta_2   = 180.0f;
        theta_3   = acos_deg(-cx_over_c);
    }
    else {
        theta_2 = acos_deg(knee_cos);

        // Interior angle between the upper leg and the chord c.
        const float upper_to_chord = acos_deg((a * a + c * c - b * b) / (2.0f * a * c));

        theta_3 = (c_w < 0.0f)
                ? 360.0f - acos_deg(-cx_over_c) - upper_to_chord
                : 180.0f - acos_deg( cx_over_c) - upper_to_chord;
    }

    return {{theta_1, theta_2, theta_3}, reachable};
}
