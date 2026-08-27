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

void Leg::move_to_pos(vec<float> pos) {
    triplet thetas = position_to_angles(pos, type).first;
    hip.move_to_angle(-thetas.first + 90);
    vTaskDelay(pdMS_TO_TICKS(SERVO_SEQUENCE_DELAY_MS));
    shoulder.move_to_angle(180 - thetas.third);
    vTaskDelay(pdMS_TO_TICKS(SERVO_SEQUENCE_DELAY_MS));
    knee.move_to_angle(thetas.second);
    vTaskDelay(pdMS_TO_TICKS(SERVO_SEQUENCE_DELAY_MS));
}

void Leg::trace_circle(bool clockwise) {
    for (int i = 0; i <= NUM_STEPS; i++) {
        float t = clockwise
            ? 2.0f * M_PI * (1.0f - (float)i / NUM_STEPS)
            : 2.0f * M_PI * ((float)i / NUM_STEPS);

        vec pos = {
            0.0f + 3.0f * std::cos(t),
            5.35f + 3.0f * std::sin(t),
            -5.0f
        };

        move_to_pos(pos);
        vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS));
    }
}