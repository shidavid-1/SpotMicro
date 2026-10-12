# Hardware test session and on-boot self tests. Only test_main.cpp calls into
# this component; the walking firmware does not use it.

idf_component_register(
    SRCS
        "Test_Session.cpp"
        "Self_Test.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        tpup
        nvs_flash      # Test_Session persists results across reboots
        esp_app_format # esp_app_get_description(), for the build stamp
        esp_timer      # esp_timer_get_time() in Test_Session
)
