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
    float height_of_body = 500.0f;
    float height_of_step = 100.0f;
    float length_of_stride = 200.0f;
    float velocity_of_dog = 100.0f;
    float duty_fact = 0.7;
    float proportion = 1.2;

    GaitParams params = {height_of_body, height_of_step, length_of_stride, velocity_of_dog,
                         duty_fact, proportion};

    // Gait Controller //
    GaitController controller = GaitController(params);

    // Transformer //
    Transformer trans = Transformer(&TPUP);

    while (1) {
        controller.compute_next_points();
        controller.increment_phi();

        trans.transform(controller.get_output_points());

        position_to_angles(trans.get_next_points_frontLeft());
        position_to_angles(trans.get_next_points_frontRight());
        position_to_angles(trans.get_next_points_backLeft());
        position_to_angles(trans.get_next_points_backRight());

        // Left off here, next problem is figuring out where to store the
        // next angles and how to make it command all of the servos
        // current idea: Just make it quick, construct a legQuad<triplet<float>>
        // put all of the angular positions into that and then
        // write a function that allows you to adjust how much time you want
        // the delay to be between servo commands and then commands all of the servos
        // sequentially within the TPUP model.
    }
}