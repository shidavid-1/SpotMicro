// Host simulation entry point -- NOT compiled into the firmware.
//
// The control loop lives in app_main() in main/walk_main.cpp, which is the
// signature ESP-IDF calls on the target. A PC executable needs main(), so this
// shim provides one and hands straight off. Keeping it here means walk_main.cpp is
// byte-identical between the sim build and the firmware build.

extern "C" void app_main(void);

int main() {
    app_main();
    return 0;
}
