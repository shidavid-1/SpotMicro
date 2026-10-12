///// Spot Micro Model /////
/*
   The purpose of this file is to construct an object
   which holds the theoretical state of the robot
   per each control loop cycle.
*/

#ifndef SPOT_MODEL_HPP
#define SPOT_MODEL_HPP

#include "Common.hpp"
#include "Servo.hpp"

// ── Circle parameters — find and replace each of these ──────────────────
constexpr uint8_t HIP_LED_POS             = 13;
constexpr uint8_t SHOULDER_LED_POS        = 14;
constexpr uint8_t KNEE_LED_POS            = 15;
constexpr float   CIRCLE_CENTER_X         = 0;   // mm
constexpr float   CIRCLE_CENTER_Y         = 0;   // mm
constexpr int     NUM_STEPS               = 144;
constexpr int     STEP_DELAY_MS           = 25;
constexpr int     SERVO_SEQUENCE_DELAY_MS = 2.5;
// ────────────────────────────────────────────────────────────────────────

// NOTE: Leg is defined before SpotModel because SpotModel holds four Leg
// members by value, which requires Leg to be a complete type at that point.
class Leg {
    private:
        LegType type;
        SpotServo hip;
        SpotServo shoulder;
        SpotServo knee;
        // DEFERRED: cached joint angles. Nothing tracks leg state yet, and a
        // member that is never written is worse than no member -- move_to_pos
        // would leave it stale while reading code assumed it was current.
        // Re-enable together with get_angles/set_angles below, and set it in
        // both the constructor and move_to_pos when you do.
        // triplet<float> angles;

    public:
        // NOTE: leg geometry (HIP_OFFSET, SHOULDER_LENGTH, UPPERLEG_LENGTH,
        // FORELEG_LENGTH, FOOTTIP_RADIUS) now lives in Common.hpp so the IK
        // solver can use it without depending on the model/servo stack.

        Leg(LegType lType, SpotServo hip, SpotServo shoulder, SpotServo knee);

        // DEFERRED: angle tracking is not supported yet. These are commented
        // out rather than left as stubs so no caller can build against an
        // accessor that returns an uninitialized `angles`. See the member above.
        // triplet<float> get_angles() const;

        // void set_angles(triplet<float> angs);

        // REQUIRES: SAFETY CRITICAL: hip shoulder knee servo types are proper
        // EFFECTS: Moves the servos in one leg such that the tip of the foot
        // is located at (x_pos, y_pos, z_pos). The delay between each servo
        // movement is delay_ms.
        // NOTE: If an unreachable position is inputted the foot tip will move
        // to the closest reachable point.
        void move_to_pos(const vec<float> &pos, const int delay_ms);

        // EFFECTS: Traces a circle with the foot tip, in NUM_STEPS steps,
        // clockwise if `clockwise` is true and counter-clockwise otherwise.
        void trace_circle(bool clockwise);
};

class SpotModel {
    private:
        Leg frontLeft;
        Leg frontRight;
        Leg backLeft;
        Leg backRight;
        vec<float> orientation; // Tells me the pitch, roll, and yaw with respect to the initial orientation
                                // the code works best when this initial orientation has the body transverse
                                // plane perpendicular to gravity.
        vec<float> position;    // I actually don't think the x y coordinates matter.
        const legQuad<vec<float>> leg_origins = global_leg_origins;
    
    public:
        
        SpotModel();

        void set_orientation(vec<float> ori);

        void set_positition(vec<float> pos);

        vec<float> get_orientation() const;
        
        vec<float> get_position() const;

        legQuad<vec<float>> get_origins() const;

        // REQUIRES: all_positions contains the positions that each
        // foot tip needs to be at relative to EACH LEG'S FRAME OF REFERENCE
        // MODIFIES: eventually will modify the state of all spotServos
        // EFFECTS: moves all legs of the spot such that each leg's foot
        // tip is coincident with the corresponding point in all_positions
        void move_all_servos(const legQuad<vec<float>> &all_positions, const int delay_ms);
};

#endif
