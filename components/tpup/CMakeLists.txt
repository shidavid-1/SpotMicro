# Robot code: everything the walking firmware runs.
#
# The subfolders are a layering, not separate components: each one only
# depends on the folders above it in this list. All of them are public include
# directories, so a source file can #include any header here by bare name.
#
# Note that the sim build (sim/build_sim.ps1) keeps its own source list, so a
# file added here is not automatically in the sim.

idf_component_register(
    SRCS
        "core/Common.cpp"
        "hardware/Board.cpp"
        "hardware/Servo.cpp"
        "hardware/IMU.cpp"
        "kinematics/IK.cpp"
        "kinematics/Spot_Model.cpp"
        "kinematics/Transformer.cpp"
        "gait/TrapezoidPath.cpp"
        "gait/Gait_Controller.cpp"
        "estimation/Filter.cpp"
    INCLUDE_DIRS
        "core"
        "hardware"
        "kinematics"
        "gait"
        "estimation"
    REQUIRES
        driver         # I2C, exposed through Board.hpp and Servo.hpp
        esp_timer      # Filter timestamps each sample
)
