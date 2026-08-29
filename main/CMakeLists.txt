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

idf_component_register(
    SRCS
        "main.cpp"
        "../Test_Session.cpp"
        "../Self_Test.cpp"
        "../Common.cpp"
        "../IMU.cpp"
        "../Servo.cpp"
        "../IK.cpp"
        "../Spot_Model.cpp"
        "../Transformer.cpp"
    INCLUDE_DIRS
        "."
        ".."
    REQUIRES
        driver
        nvs_flash      # Test_Session persists results across reboots
        esp_app_format # esp_app_get_description(), for the build stamp
)
