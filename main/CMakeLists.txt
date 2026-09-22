# The source files live in the repo root rather than inside this component,
# so they are pulled in by relative path and the root is added as an include
# directory.
#
# Deliberately NOT built for this testing session: Filter.cpp. Its two static
# helpers are called above their definitions, so it does not compile yet, and
# nothing in the session needs it.
#
# IK.cpp, Spot_Model.cpp and Transformer.cpp used to sit on that list too. They
# are in now because Self_Test.cpp's test_transformer exercises Transformer,
# which pulls SpotModel::get_origins() out of Spot_Model.cpp, which pulls
# position_to_angles() out of IK.cpp -- the whole chain has to link or the
# session does not build. The tradeoff the old note was guarding against still
# stands: a compile error in any of these three now blocks the hardware tests.
#
# TrapezoidPath.cpp is here for the same reason: test_TrapezoidPath constructs
# paths and calls percent_of_path, and the header alone does not define them.
# Note that the sim build (sim/build_sim.ps1) keeps its own source list, so a
# file added to one build is not automatically in the other.
#
# Both entry points are listed below. Each app_main is guarded on
# TPUP_CONTROL_LOOP, so exactly one survives preprocessing -- two definitions
# would not link. Which one is decided by the PlatformIO environment, not by
# editing anything here: -e walk defines the flag (control loop in the root
# main.cpp), -e test leaves it undefined (test session in main/main.cpp). See
# the header comment in platformio.ini.

idf_component_register(
    SRCS
        "main.cpp"          # test-session app_main
        "../main.cpp"       # control-loop app_main
        "../Gait_Controller.cpp"
        "../Test_Session.cpp"
        "../Self_Test.cpp"
        "../Common.cpp"
        "../IMU.cpp"
        "../Servo.cpp"
        "../IK.cpp"
        "../Spot_Model.cpp"
        "../Transformer.cpp"
        "../TrapezoidPath.cpp"
        "../Filter.cpp"
    INCLUDE_DIRS
        "."
        ".."
    REQUIRES
        driver
        nvs_flash      # Test_Session persists results across reboots
        esp_app_format # esp_app_get_description(), for the build stamp
        esp_timer      # Filter timestamps each sample
)
