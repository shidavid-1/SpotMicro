#include "Spot_Model.hpp"

#include "Common.hpp"
#include "IK.hpp"
#include "Servo.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"   // pdMS_TO_TICKS
#include "freertos/task.h"       // vTaskDelay
#include <cmath>                 // std::cos, std::sin, M_PI
#include <cstdlib>               // exit

SpotModel::SpotModel()
    : frontLeft(LegType::frontLeft, SpotServo(1), SpotServo(2), SpotServo(3)), 
      frontRight(LegType::frontRight, SpotServo(4), SpotServo(5), SpotServo(6)), 
      backLeft(LegType::backLeft, SpotServo(7), SpotServo(8), SpotServo(9)), 
      backRight(LegType::backRight, SpotServo(10), SpotServo(11), SpotServo(12)),
      orientation({}), position({}) {}

void SpotModel::set_orientation(vec<float> ori) {
    orientation = ori;
}

void SpotModel::set_positition(vec<float> pos) {
    position = pos;
}

vec<float> SpotModel::get_orientation() const {
    return orientation;
}

vec<float> SpotModel::get_position() const {
    return position;
}

legQuad<vec<float>> SpotModel::get_origins() const {
    return leg_origins;
}

void SpotModel::move_all_servos(const legQuad<vec<float>> &all_positions, const int delay_ms) {
    frontLeft.move_to_pos(all_positions.frontLeft, delay_ms);
    frontRight.move_to_pos(all_positions.frontRight, delay_ms);
    backLeft.move_to_pos(all_positions.backLeft, delay_ms);
    backRight.move_to_pos(all_positions.backRight, delay_ms);
}

Leg::Leg(LegType type, SpotServo hip, SpotServo shoulder, SpotServo knee)
    : type(type), hip(hip), shoulder(shoulder), knee(knee) {

    if (static_cast<int>(type) < 0 || static_cast<int>(type) > 3) {
        ESP_LOGI("ERROR MESSAGE", "Invalid leg type specified.");
        exit(1);
    }

    if (hip.get_type() != ServoType::hip ||
        shoulder.get_type() != ServoType::shoulder ||
        knee.get_type() != ServoType::knee) {
        ESP_LOGI("ERROR MESSAGE", "Servo type mismatch: Hip type: %s, Shoulder type: %s, Knee type: %s", type_name(hip.get_type()), type_name(shoulder.get_type()), type_name(knee.get_type()));
        exit(1);
    }
}

// DEFERRED: see the note on Leg's angle members in Spot_Model.hpp. Angle
// tracking is not supported yet; these definitions come back when the `angles`
// member does.
// triplet<float> Leg::get_angles() const {
//     return angles;
// }
//
// void Leg::set_angles(triplet<float> angs) {
//     angles = angs;
// }

void Leg::move_to_pos(const vec<float> &pos, const int delay_ms) {
    triplet thetas = position_to_angles(pos).first;
    hip.move_to_angle(thetas.first);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    shoulder.move_to_angle(thetas.third);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    knee.move_to_angle(thetas.second);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

void Leg::trace_circle(bool clockwise) {
    for (int i = 0; i <= NUM_STEPS; i++) {
        float t = clockwise
            ? 2.0f * M_PI * (1.0f - (float)i / NUM_STEPS)
            : 2.0f * M_PI * ((float)i / NUM_STEPS);

        // Millimeters, to match the leg geometry in Common.hpp and the frame
        // position_to_angles works in. Formerly inches: a 3 in radius about
        // (0, 5.35) in the z = -5 in plane.
        vec<float> pos = {
            0.0f + 76.2f * std::cos(t),
            135.89f + 76.2f * std::sin(t),
            -127.0f
        };

        move_to_pos(pos, 2);
        vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
    }
}