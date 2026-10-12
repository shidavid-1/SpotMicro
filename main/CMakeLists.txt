# Entry points only. The robot code is the tpup component and the test harness
# is tpup_test, both under components/.
#
# Both entry points are listed below. Each app_main is guarded on
# TPUP_CONTROL_LOOP, so exactly one survives preprocessing -- two definitions
# would not link. Which one is decided by the PlatformIO environment, not by
# editing anything here: -e walk defines the flag (control loop in
# walk_main.cpp), -e test leaves it undefined (test session in test_main.cpp).
# See the header comment in platformio.ini.

idf_component_register(
    SRCS
        "test_main.cpp"     # test-session app_main
        "walk_main.cpp"     # control-loop app_main
    REQUIRES
        tpup
        tpup_test
)
