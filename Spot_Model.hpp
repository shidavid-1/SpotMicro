///// Spot Micro Model /////
/*
   The purpose of this file is to construct an object
   which holds the theoretical state of the robot
   per each control loop cycle.
*/

#include "Common.hpp"
#include "Servo.hpp"

#ifndef SPOT_MODEL_HPP
#define SPOT_MODEL_HPP

// ── Circle parameters — find and replace each of these ──────────────────
constexpr uint8_t HIP_LED_POS             = 13;
constexpr uint8_t SHOULDER_LED_POS        = 14;
constexpr uint8_t KNEE_LED_POS            = 15;
constexpr float   CIRCLE_CENTER_X         = 0;   // inches
constexpr float   CIRCLE_CENTER_Y         = 0;   // inches
constexpr int     NUM_STEPS               = 144;
constexpr int     STEP_DELAY_MS           = 25;
constexpr int     SERVO_SEQUENCE_DELAY_MS = 2.5;
// ────────────────────────────────────────────────────────────────────────

enum class LegType {frontLeft, frontRight, backLeft, backRight};

class SpotModel {
    private:
        Leg frontLeft;
        Leg frontRight;
        Leg backLeft;
        Leg backRight;
        vec<float> orientation;
        vec<float> position;    // I actually don't think the x y coordinates matter.
    public:
        void set_orientation();

        void set_positition();

        vec<float> get_orientation() const;
        
        vec<float> get_position() const;
};

class Leg {
    private:
        LegType lType;
        Servo hip;
        Servo shoulder;
        Servo knee;
        triplet<float> angles;

    public:
        constexpr static float HIP_OFFSET       = 0.3838583f; // 9.75 mm  
        constexpr static float SHOULDER_LENGTH  = 2.354331f; // 59.8 mm
        constexpr static float UPPERLEG_LENGTH  = 4.3701456693f; // 111.0017 mm
        constexpr static float FORELEG_LENGTH   = 4.7983818898f; // 121.8789 mm note that this is bolt to bolt rather than bolt to foot tip surface
        constexpr static float FOOTTIP_RADIUS   = 0.0f/* NEED TO GET ACTUAL VALUE*/;
        
        Leg(LegType lType, Servo hip, Servo shoulder, Servo knee);

        triplet<float> get_angles() const;

        triplet<float> set_angles();

        // REQUIRES: SAFETY CRITICAL: hip shoulder knee servo types are proper
        // EFFECTS: Moves the servos in one leg such that the tip of the foot
        // is located at (x_pos, y_pos, z_pos).
        // NOTE: If an unreachable position is inputted the foot tip will move
        // to the closest reachable point.
        void move_to_pos(const vec<float> pos) const;

        // EFFECTS: Traces a circle with the foot tip, in NUM_STEPS steps,
        // clockwise if `clockwise` is true and counter-clockwise otherwise.
        void trace_circle(bool clockwise) const;
};

#endif