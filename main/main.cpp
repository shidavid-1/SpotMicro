///// Test Session Entry Point /////
/* This main exists only to run the hardware test session. It is not the
   flight firmware. When the control loop is written, either replace the body
   of app_main or guard the test session behind a build flag.
*/

#include "Common.hpp"
#include "Test_Session.hpp"

#ifndef TPUP_CONTROL_LOOP

extern "C" void app_main(void) {
    // Must come first: every device on the bus depends on it, and the IMU
    // constructor aborts if the bus is not up.
    init_i2c();

    run_test_session();
}

#endif // !TPUP_CONTROL_LOOP
