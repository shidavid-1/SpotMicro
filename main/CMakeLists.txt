# The source files live in the repo root rather than inside this component,
# so they are pulled in by relative path and the root is added as an include
# directory.
#
# Deliberately NOT built for this testing session: IK.cpp, Filter.cpp,
# Spot_Model.cpp, Transformer.cpp. They are still under construction and a
# compile error in one of them would block the hardware tests, which do not
# depend on any of them. Add them here as they stabilize.

idf_component_register(
    SRCS
        "main.cpp"
        "../Test_Session.cpp"
        "../Self_Test.cpp"
        "../Common.cpp"
        "../IMU.cpp"
        "../Servo.cpp"
    INCLUDE_DIRS
        "."
        ".."
    REQUIRES
        driver
)
