#include "TrapezoidPath.hpp"
#include "Transformer.hpp"
#include "Spot_Model.hpp"
#include "Servo.hpp"
#include "IMU.hpp"
#include "IK.hpp"
#include "Gait_Controller.hpp"
#include "Filter.hpp"
#include "Common.hpp"

int main(int argc, char* argv[]) {

    // Create all objects:

    // Spot Model //
    SpotModel TPUP;

    // Gait Parameters //
    float height_of_body = 230.0f;
    float height_of_step = 30.0f;
    float length_of_stride = 50.0f;
    float velocity_of_dog = 10.0f;
    float duty_fact = 0.7;
    float proportion = 1.2;

    GaitParams params = {height_of_body, height_of_step, length_of_stride, velocity_of_dog,
                         duty_fact, proportion};

    // Gait Controller //
    GaitController controller = GaitController(params);

    // Transformer //
    Transformer trans = Transformer(&TPUP);

    // Miscellanious Variables //
    legQuad<triplet<float>> all_leg_angles = {};
    int SERVO_DELAY_MS = 2.0f;

    TickType_t initialTime = xTaskGetTickCount();

    int count = 0;

    while (count < 5000) {
        controller.compute_next_points();
        controller.increment_phi();

        trans.transform(controller.get_output_points());

        TPUP.move_all_servos(trans.get_next_points(), SERVO_DELAY_MS);

        ESP_LOGI("TESTING", "Front Left: (%f, %f, %f)", trans.get_next_points_frontLeft().x, trans.get_next_points_frontLeft().y, trans.get_next_points_frontLeft().z);
        ++count;

        // Creates rough absolute deadlines for each loop iteration.
        vTaskDelayUntil(&initialTime, pdMS_TO_TICKS(12));
    }
}