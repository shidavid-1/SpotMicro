#include "TrapezoidPath.hpp"
#include "Transformer.hpp"
#include "Spot_Model.hpp"
#include "Servo.hpp"
#include "IMU.hpp"
#include "IK.hpp"
#include "Gait_Controller.hpp"
#include "Filter.hpp"
#include "Common.hpp"

// Selected by TPUP_CONTROL_LOOP (see platformio.ini). The other app_main
// lives in main/main.cpp and runs the hardware test session; exactly one of
// the two is compiled in, since two definitions would not link.
#ifdef TPUP_CONTROL_LOOP

extern "C" void app_main(void) {

    // Must come first: every device on the bus depends on it.
    init_i2c();

    // Sets the PCA9685 prescaler for desired_pwm_freq. Without it the chip
    // stays at its ~200 Hz power-on default and every pulse width is wrong.
    init_pca9685();
    vTaskDelay(pdMS_TO_TICKS(1));   // oscillator needs up to 500 us to settle

    // Create all objects:

    // Spot Model //
    SpotModel TPUP;

    // Gait Parameters //
    float height_of_body = 230.0f;
    float height_of_step = 30.0f;
    float length_of_stride = 50.0f;
    float velocity_of_dog = 100.0f;
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
    int SERVO_DELAY_MS = 0.0f;

    TickType_t initialTime = xTaskGetTickCount();

    int count = 0;

    while (count < 5000) {
        controller.compute_next_points();
        controller.increment_phi();

        trans.transform(controller.get_output_points());

        TPUP.move_all_servos(trans.get_next_points(), SERVO_DELAY_MS);

        //ESP_LOGI("TESTING", "Front Left: (%f, %f, %f)", trans.get_next_points_frontLeft().x, trans.get_next_points_frontLeft().y, trans.get_next_points_frontLeft().z);
        ++count;

        // Creates rough absolute deadlines for each loop iteration.
        vTaskDelayUntil(&initialTime, pdMS_TO_TICKS(12));
    }
}

#endif // TPUP_CONTROL_LOOP
