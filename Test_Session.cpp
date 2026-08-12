#include "Test_Session.hpp"
#include "Self_Test.hpp"
#include "Common.hpp"
#include "IMU.hpp"
#include "Servo.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

//////////////////// SESSION CONFIGURATION ////////////////////

// Ranges the session's IMU is constructed with, and restored to after the
// saturation test walks through all four.
constexpr AngularRange DEFAULT_ANG_RANGE = AngularRange::plus_minus_500dps;
constexpr AccelRange   DEFAULT_ACC_RANGE = AccelRange::plus_minus_4g;

// Acceptance criteria, taken from the test plan.
constexpr float SERVO_ANGLE_TOL = 1.0f;   // deg, Servo::move_to_angle
constexpr float SPOT_LIMIT_TOL  = 5.0f;   // deg, SpotServo saturation
constexpr float FACE_FRACTION   = 0.90f;  // update(): 90% of |a| on one axis

// Not from the plan: post-calibration rest thresholds, so calibrate() has a
// number to pass or fail against rather than a judgement call.
constexpr float REST_GYRO_TOL  = 2.0f;    // dps
constexpr float REST_ACCEL_TOL = 0.5f;    // m/s^2

// The plan lists 0, 45, 90, 145, 180. 145 breaks the even spacing of the
// other four and is very likely meant to be 135; left as written so the test
// matches the document. Change this line if it was a typo.
constexpr float SWEEP_ANGLES[] = { 0.0f, 45.0f, 90.0f, 145.0f, 180.0f };
constexpr int   NUM_SWEEP_ANGLES = sizeof(SWEEP_ANGLES) / sizeof(SWEEP_ANGLES[0]);


//////////////////// CONSOLE HELPERS ////////////////////

// EFFECTS: Makes stdin/stdout unbuffered so typed characters arrive one at a
// time and prompts appear without waiting for a newline.
static void console_init() {
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
}

// EFFECTS: Blocks until the user types a line, echoing as they go. Yields to
// the scheduler between polls so the idle task still runs.
static void read_line(char *buf, size_t n) {
    size_t i = 0;
    for (;;) {
        int c = fgetc(stdin);
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (c == '\r' || c == '\n') {
            printf("\n");
            break;
        }
        if (c == '\b' || c == 127) {          // backspace
            if (i > 0) { --i; printf("\b \b"); }
            continue;
        }
        if (i + 1 < n) { buf[i++] = (char)c; printf("%c", c); }
    }
    buf[i] = '\0';
}

// EFFECTS: Prints prompt and waits for the user to press Enter.
static void wait_for_enter(const char *prompt) {
    char scratch[8];
    printf("\n    >> %s, then press Enter... ", prompt);
    read_line(scratch, sizeof scratch);
}

// EFFECTS: Prompts for an integer in [lo, hi]. Re-asks until valid.
static int ask_int(const char *prompt, int lo, int hi) {
    char line[16];
    for (;;) {
        printf("    %s [%d-%d]: ", prompt, lo, hi);
        read_line(line, sizeof line);
        char *end = NULL;
        long v = strtol(line, &end, 10);
        if (end != line && v >= lo && v <= hi) return (int)v;
        printf("    ! Not a number in range.\n");
    }
}

// EFFECTS: Prompts for a float. Returns false on a blank line, which callers
// treat as "done".
static bool ask_float(const char *prompt, float *out) {
    char line[24];
    printf("    %s (blank to finish): ", prompt);
    read_line(line, sizeof line);
    if (line[0] == '\0') return false;
    *out = strtof(line, NULL);
    return true;
}


//////////////////// RESULT RECORDING ////////////////////

enum class Result { not_run, pass, fail, skip };

struct TestRecord {
    const char *name;
    Result      result;
    char        note[64];
};

// Ordered to follow the test plan document.
static TestRecord records[] = {
    { "Self tests (pure math)",            Result::not_run, "" },  // 0
    { "IMU ctor: zeroed + ranges set",     Result::not_run, "" },  // 1
    { "IMU ctor: range saturation",        Result::not_run, "" },  // 2
    { "IMU calibrate(): at rest",          Result::not_run, "" },  // 3
    { "IMU update(): six-face",            Result::not_run, "" },  // 4
    { "Servo::move_to_angle sweep",        Result::not_run, "" },  // 5
    { "Servo::move_to_pwm (scoped)",       Result::not_run, "" },  // 6
    { "SpotServo #1: limit saturation",    Result::not_run, "" },  // 7
    { "SpotServo #2: protractor angles",   Result::not_run, "" },  // 8
};
constexpr int NUM_TESTS = sizeof(records) / sizeof(records[0]);

// EFFECTS: Asks the operator to judge the test just performed and stores the
// verdict. The operator is the instrument for most of these, so the verdict
// has to come from them.
static void ask_result(int index) {
    char line[16];
    for (;;) {
        printf("\n    Verdict for \"%s\" -- (p)ass / (f)ail / (s)kip: ", records[index].name);
        read_line(line, sizeof line);
        if (line[0] == 'p') { records[index].result = Result::pass; break; }
        if (line[0] == 'f') { records[index].result = Result::fail; break; }
        if (line[0] == 's') { records[index].result = Result::skip; break; }
    }
    printf("    Note (optional, blank to skip): ");
    read_line(records[index].note, sizeof records[index].note);
}

static const char *result_str(Result r) {
    switch (r) {
        case Result::pass:    return "PASS";
        case Result::fail:    return "FAIL";
        case Result::skip:    return "SKIP";
        case Result::not_run: return "----";
    }
    return "????";
}


//////////////////// SHARED HARDWARE ////////////////////

// EFFECTS: Returns the session's IMU, constructing it on first use. Deferred
// so the servo-only tests still run with no IMU on the bus (the IMU
// constructor aborts when WHO_AM_I fails).
static IMU &imu_instance() {
    static IMU imu(DEFAULT_ANG_RANGE, DEFAULT_ACC_RANGE);
    return imu;
}

// EFFECTS: Writes the PCA9685 prescaler once per boot.
static void ensure_pca_ready() {
    static bool done = false;
    if (!done) {
        init_pca9685();
        done = true;
        printf("    (PCA9685 initialized: %.2f Hz, period %.1f us)\n",
               (double)true_pwm_freq, (double)period);
    }
}

// EFFECTS: Reads one register from an I2C device into *out. Two transactions:
// one to set the register pointer, one to read it back.
static esp_err_t read_reg(uint8_t addr, uint8_t reg, uint8_t *out) {
    i2c_cmd_handle_t ptr = i2c_cmd_link_create();
    i2c_master_start(ptr);
    i2c_master_write_byte(ptr, addr << 1, true);
    i2c_master_write_byte(ptr, reg, true);
    i2c_master_stop(ptr);
    esp_err_t err = i2c_master_cmd_begin(i2c_port, ptr, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(ptr);
    if (err != ESP_OK) return err;

    ptr = i2c_cmd_link_create();
    i2c_master_start(ptr);
    i2c_master_write_byte(ptr, (addr << 1) + 1, true);
    i2c_master_read_byte(ptr, out, I2C_MASTER_NACK);
    i2c_master_stop(ptr);
    err = i2c_master_cmd_begin(i2c_port, ptr, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(ptr);
    return err;
}

// Set the moment anything calls update() or calibrate(), because the
// constructor's "vectors start zeroed" clause is only observable before that
// happens.
static bool imu_dirtied = false;

// EFFECTS: Averages `samples` IMU readings taken `interval_ms` apart.
static void average_reading(IMU &imu, int samples, int interval_ms,
                            vec<float> *accel_out, vec<float> *angvel_out) {
    vec<float> a = {0, 0, 0};
    vec<float> g = {0, 0, 0};
    imu_dirtied = true;
    for (int i = 0; i < samples; ++i) {
        imu.update();
        vec<float> ai = imu.get_accel();
        vec<float> gi = imu.get_angvel();
        a.x += ai.x; a.y += ai.y; a.z += ai.z;
        g.x += gi.x; g.y += gi.y; g.z += gi.z;
        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }
    const float n = (float)samples;
    *accel_out  = { a.x / n, a.y / n, a.z / n };
    *angvel_out = { g.x / n, g.y / n, g.z / n };
}


//////////////////// TESTS ////////////////////

// T0: the pure math checks. No hardware judgement needed, so this scores
// itself.
static void test_self_tests() {
    printf("\n--- Self tests (pure math) ---\n");
    printf("Verifies ang_conversion() and accl_conversion() against\n"
           "hand-computed scale factors. A failure here is definitely code,\n"
           "not hardware.\n\n");

    const bool ok = run_self_tests(imu_instance());
    records[0].result = ok ? Result::pass : Result::fail;
    snprintf(records[0].note, sizeof records[0].note, "automatic");
    printf("\n    Recorded automatically: %s\n", result_str(records[0].result));
}


// T1: the deterministic half of the constructor's contract -- vectors start
// zeroed, and the requested ranges actually reach the device. Reading the
// registers back proves the second clause exactly, in two seconds.
static void test_imu_ctor_config() {
    printf("\n--- IMU ctor: zeroed vectors + ranges set ---\n");
    printf("Desired behavior: construct with all-zero vectors, and set the\n"
           "angular and acceleration ranges to those requested.\n\n");

    // Clause 1 -- only observable before anything calls update() or
    // calibrate(), so it is reported as inconclusive rather than failed once
    // the object has been touched.
    if (!imu_dirtied) {
        const bool zeroed = test_only_vectors_are_zero(imu_instance());
        printf("      vectors all zero after construction: %s\n", zeroed ? "YES" : "NO");
    }
    else {
        printf("      vectors all zero after construction: INCONCLUSIVE\n"
               "        (a previous test already called update(); reset the\n"
               "         board and run this test first to check it)\n");
    }

    // Clause 2 -- walk every range and read the config registers back.
    struct RangeCase {
        AngularRange ang;
        AccelRange   acc;
        const char  *name;
        uint8_t      fs_sel;   // expected bits 4:3 in both config registers
    };
    const RangeCase cases[] = {
        { AngularRange::plus_minus_250dps,  AccelRange::plus_minus_2g,  "250 dps /  2g", 0b00 << 3 },
        { AngularRange::plus_minus_500dps,  AccelRange::plus_minus_4g,  "500 dps /  4g", 0b01 << 3 },
        { AngularRange::plus_minus_1000dps, AccelRange::plus_minus_8g,  "1000 dps/  8g", 0b10 << 3 },
        { AngularRange::plus_minus_2000dps, AccelRange::plus_minus_16g, "2000 dps/ 16g", 0b11 << 3 },
    };

    uint8_t who = 0, pwr = 0;
    if (read_reg(IMU_ADDR, WHO_AM_I, &who) != ESP_OK) {
        printf("    ! I2C read failed. Check wiring before continuing.\n");
        ask_result(1);
        return;
    }
    read_reg(IMU_ADDR, PWR_MGMT_1, &pwr);
    printf("      WHO_AM_I   = 0x%02X (expect 0x%02X)\n", who, WHO_AM_I_IMU_VALUE);
    printf("      PWR_MGMT_1 = 0x%02X (expect sleep bit clear)\n\n", pwr);

    int mismatches = 0;
    for (const RangeCase &c : cases) {
        test_only_set_ranges(imu_instance(), c.ang, c.acc);
        vTaskDelay(pdMS_TO_TICKS(10));

        uint8_t gyro = 0, accel = 0;
        read_reg(IMU_ADDR, GYRO_CONFIG,  &gyro);
        read_reg(IMU_ADDR, ACCEL_CONFIG, &accel);

        const bool ok = ((gyro & 0x18) == c.fs_sel) && ((accel & 0x18) == c.fs_sel);
        if (!ok) ++mismatches;
        printf("      %-14s GYRO=0x%02X ACCEL=0x%02X  expect FS_SEL 0x%02X  %s\n",
               c.name, gyro, accel, c.fs_sel, ok ? "ok" : "MISMATCH");
    }

    test_only_set_ranges(imu_instance(), DEFAULT_ANG_RANGE, DEFAULT_ACC_RANGE);
    printf("\n      (ranges restored to the session default)\n");
    printf("\n    Suggested verdict: %s\n", mismatches == 0 ? "PASS" : "FAIL");
    ask_result(1);
}


// T2: the saturation test from the plan -- the only check that the scale
// matches the configured range through real motion rather than a register.
static void test_imu_saturation() {
    printf("\n--- IMU ctor: range saturation ---\n");
    printf("Desired behavior: readings saturate at the bound of the range the\n"
           "IMU was configured with, and never read past it.\n");
    printf("\nCAUTION: shake firmly, not violently. You are trying to reach the\n"
           "bound, not to test the solder joints. Hold the PCB, not the wires.\n"
           "Reaching 2000 dps by hand is genuinely hard -- falling short is\n"
           "inconclusive, not a failure.\n");

    struct RangeCase {
        AngularRange ang;
        AccelRange   acc;
        const char  *name;
        float        ang_bound;   // dps
        float        acc_bound;   // m/s^2
    };
    const RangeCase cases[] = {
        { AngularRange::plus_minus_250dps,  AccelRange::plus_minus_2g,  "250 dps / 2g",   250.0f,  2.0f * gravity },
        { AngularRange::plus_minus_500dps,  AccelRange::plus_minus_4g,  "500 dps / 4g",   500.0f,  4.0f * gravity },
        { AngularRange::plus_minus_1000dps, AccelRange::plus_minus_8g,  "1000 dps / 8g", 1000.0f,  8.0f * gravity },
        { AngularRange::plus_minus_2000dps, AccelRange::plus_minus_16g, "2000 dps / 16g",2000.0f, 16.0f * gravity },
    };

    imu_dirtied = true;
    int overruns = 0;
    for (const RangeCase &c : cases) {
        test_only_set_ranges(imu_instance(), c.ang, c.acc);

        char prompt[96];
        snprintf(prompt, sizeof prompt, "Range set to %s -- ready to shake for 8 seconds", c.name);
        wait_for_enter(prompt);

        float peak_a = 0.0f, peak_g = 0.0f;
        const int ticks = 8 * 1000 / 20;
        for (int i = 0; i < ticks; ++i) {
            imu_instance().update();
            vec<float> a = imu_instance().get_accel();
            vec<float> g = imu_instance().get_angvel();
            peak_a = fmaxf(peak_a, fmaxf(fabsf(a.x), fmaxf(fabsf(a.y), fabsf(a.z))));
            peak_g = fmaxf(peak_g, fmaxf(fabsf(g.x), fmaxf(fabsf(g.y), fabsf(g.z))));
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        const bool accel_over = peak_a > c.acc_bound * 1.02f;
        const bool gyro_over  = peak_g > c.ang_bound * 1.02f;
        if (accel_over || gyro_over) ++overruns;

        printf("      peak |accel| = %7.2f / bound %7.2f m/s^2  %s\n",
               (double)peak_a, (double)c.acc_bound, accel_over ? "EXCEEDED BOUND" : "ok");
        printf("      peak |angvel|= %7.1f / bound %7.1f dps    %s\n",
               (double)peak_g, (double)c.ang_bound, gyro_over ? "EXCEEDED BOUND" : "ok");
        printf("      reached %.0f%% of accel bound, %.0f%% of gyro bound\n",
               (double)(100.0f * peak_a / c.acc_bound), (double)(100.0f * peak_g / c.ang_bound));
    }

    test_only_set_ranges(imu_instance(), DEFAULT_ANG_RANGE, DEFAULT_ACC_RANGE);
    printf("\n      (ranges restored to the session default)\n");
    printf("\n    %d of 4 ranges reported a reading past its bound.\n", overruns);
    printf("    PASS if nothing exceeded its bound AND at least the lower\n"
           "    ranges got close to theirs. A peak that pins at exactly the\n"
           "    bound is the expected shape of a pass.\n");
    ask_result(2);
}


// T3: calibrate(). Gravity must SURVIVE on z -- Filter needs it for tilt.
static void test_imu_calibrate() {
    printf("\n--- IMU calibrate(): at rest ---\n");
    printf("Desired behavior: at rest, all angular velocities read ~0, accel\n"
           "x and y read ~0, and accel z reads +%.2f m/s^2.\n", (double)gravity);
    printf("\nThe plan calls for a spy class injecting a known nonzero bias.\n"
           "This runs the on-target equivalent: measure the real bias before\n"
           "and after, which also tells you how large your actual bias is.\n");

    wait_for_enter("Place the IMU LEVEL and STILL (calibrating on a slope "
                   "bakes that tilt in permanently)");

    vec<float> a_before, g_before;
    average_reading(imu_instance(), 20, 20, &a_before, &g_before);
    printf("\n      before: accel %7.2f %7.2f %7.2f   angvel %8.2f %8.2f %8.2f\n",
           (double)a_before.x, (double)a_before.y, (double)a_before.z,
           (double)g_before.x, (double)g_before.y, (double)g_before.z);

    imu_instance().calibrate();

    vec<float> a_after, g_after;
    average_reading(imu_instance(), 20, 20, &a_after, &g_after);
    printf("      after:  accel %7.2f %7.2f %7.2f   angvel %8.2f %8.2f %8.2f\n",
           (double)a_after.x, (double)a_after.y, (double)a_after.z,
           (double)g_after.x, (double)g_after.y, (double)g_after.z);
    printf("      target: accel    ~0.00   ~0.00  %+7.2f   angvel     ~0.00    ~0.00    ~0.00\n",
           (double)gravity);
    printf("      tolerance: accel +/- %.2f m/s^2, gyro +/- %.2f dps\n",
           (double)REST_ACCEL_TOL, (double)REST_GYRO_TOL);

    const bool ok = fabsf(a_after.x) <= REST_ACCEL_TOL
                 && fabsf(a_after.y) <= REST_ACCEL_TOL
                 && fabsf(a_after.z - gravity) <= REST_ACCEL_TOL
                 && fabsf(g_after.x) <= REST_GYRO_TOL
                 && fabsf(g_after.y) <= REST_GYRO_TOL
                 && fabsf(g_after.z) <= REST_GYRO_TOL;

    printf("\n    Suggested verdict: %s\n", ok ? "PASS" : "FAIL");
    printf("    If accel z came out ~0 instead of ~%.2f, calibrate() is\n"
           "    subtracting gravity along with the bias, and Filter will have\n"
           "    nothing to estimate tilt from.\n", (double)gravity);
    ask_result(3);
}


// T4: update(), validated by the six-face orientation test. Body frame per
// the plan: X forward, Y to the dog's left, Z up.
static void test_imu_update_six_face() {
    printf("\n--- IMU update(): six-face ---\n");
    printf("Desired behavior: update() refreshes the members with the sensor's\n"
           "current values.\n");
    printf("\nValidation: on each of six faces, at least %.0f%% of the linear\n"
           "acceleration magnitude must lie on the expected axis, with the\n"
           "expected sign.\n", (double)(FACE_FRACTION * 100.0f));
    printf("Body frame: X forward, Y to the dog's LEFT, Z up.\n");
    printf("\nNOTE: run this BEFORE calibrate(), or biases captured in one\n"
           "orientation will skew the other five.\n");

    struct Face {
        const char *instruction;
        int         axis;        // 0 = x, 1 = y, 2 = z
        float       sign;        // expected sign on that axis
    };
    const Face faces[] = {
        { "normal upright mounting (Z up)",            2, +1.0f },
        { "inverted (Z down)",                         2, -1.0f },
        { "nose up, X pointing at the ceiling",        0, +1.0f },
        { "nose down, X pointing at the floor",        0, -1.0f },
        { "dog's left side up, Y at the ceiling",      1, +1.0f },
        { "dog's right side up, Y at the floor",       1, -1.0f },
    };
    const char *axis_name[3] = { "x", "y", "z" };

    int failures = 0;
    for (const Face &f : faces) {
        char prompt[112];
        snprintf(prompt, sizeof prompt, "Rest the IMU: %s", f.instruction);
        wait_for_enter(prompt);

        vec<float> a, g;
        average_reading(imu_instance(), 20, 20, &a, &g);

        const float axis[3] = { a.x, a.y, a.z };
        const float norm = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
        const float fraction = norm > 0.0f ? fabsf(axis[f.axis]) / norm : 0.0f;
        const bool  sign_ok  = axis[f.axis] * f.sign > 0.0f;
        const bool  ok       = sign_ok && fraction >= FACE_FRACTION;

        printf("      accel = %7.2f %7.2f %7.2f   |a| = %.2f\n",
               (double)a.x, (double)a.y, (double)a.z, (double)norm);
        printf("      %.0f%% on %s axis, sign %s   -> %s\n",
               (double)(fraction * 100.0f), axis_name[f.axis],
               sign_ok ? "correct" : "INVERTED",
               ok ? "pass" : "FAIL");
        if (!ok) ++failures;
    }

    printf("\n    %d of 6 faces failed.\n", failures);
    printf("    Interpreting failures:\n");
    printf("      - magnitude lands on the wrong axis  -> axes are swapped\n");
    printf("      - right axis, sign inverted          -> sign convention flipped\n");
    printf("      - |a| consistently far from %.2f     -> wrong range or scale\n", (double)gravity);
    printf("      - one polarity reads huge positive   -> signedness bug\n");
    printf("      - a value that never changes         -> update() returning stale data\n");
    ask_result(4);
}


// T5: Servo::move_to_angle. Off robot, single servo, needle attached.
static void test_servo_move_to_angle() {
    printf("\n--- Servo::move_to_angle sweep ---\n");
    printf("Desired behavior: move the servo to the desired angle.\n");
    printf("\nSETUP: OFF the robot, single servo, long needle axially locked to\n"
           "the axle, protractor beneath. Assumes slope and offset are already\n"
           "correct for this servo.\n");
    printf("Acceptance: needle within +/- %.0f degree of each commanded angle.\n",
           (double)SERVO_ANGLE_TOL);

    const int num = ask_int("Servo number (0 = uncalibrated identity)", 0, 12);
    ensure_pca_ready();
    Servo servo(num);

    const Calibration cal = calibration_for(num);
    printf("    Servo %d: slope %.3f, offset %+.2f\n",
           num, (double)cal.slope, (double)cal.offset);

    printf("\n  Ascending pass:\n");
    for (int i = 0; i < NUM_SWEEP_ANGLES; ++i) {
        char prompt[64];
        snprintf(prompt, sizeof prompt, "Ready to command %.0f degrees", (double)SWEEP_ANGLES[i]);
        wait_for_enter(prompt);
        servo.move_to_angle(SWEEP_ANGLES[i]);
        printf("      commanded %.0f deg -> read the needle and note the error\n",
               (double)SWEEP_ANGLES[i]);
        vTaskDelay(pdMS_TO_TICKS(600));
    }

    printf("\n  Descending pass. Not in the plan, but free: any difference at\n"
           "  the same angle between the two passes is gear backlash, which\n"
           "  will show up later as gait inconsistency.\n");
    for (int i = NUM_SWEEP_ANGLES - 1; i >= 0; --i) {
        char prompt[72];
        snprintf(prompt, sizeof prompt, "Ready to command %.0f degrees (descending)",
                 (double)SWEEP_ANGLES[i]);
        wait_for_enter(prompt);
        servo.move_to_angle(SWEEP_ANGLES[i]);
        printf("      commanded %.0f deg -> read the needle\n", (double)SWEEP_ANGLES[i]);
        vTaskDelay(pdMS_TO_TICKS(600));
    }

    printf("\n    PASS if every reading was within +/- %.0f degree on the\n"
           "    ascending pass. Record the descending numbers even if you do\n"
           "    not gate on them.\n", (double)SERVO_ANGLE_TOL);
    ask_result(5);
}


// T6: move_to_pwm. Skipped per the plan, recorded so the gap stays visible.
static void test_move_to_pwm_skipped() {
    printf("\n--- Servo::move_to_pwm (scoped) ---\n");
    printf("SKIPPED per the test plan: if move_to_angle passes, the counts it\n"
           "computes are reaching the servo correctly, so move_to_pwm passes\n"
           "with it.\n");
    printf("\nThat inference holds for the composition, with one caveat worth\n"
           "recording: move_to_angle passing proves angle -> physical position\n"
           "is correct END TO END, not that counts -> microseconds is correct\n"
           "on its own. A systematic error there (most likely the PCA9685\n"
           "oscillator being off its nominal 25 MHz) gets absorbed into each\n"
           "servo's slope and offset and cancels out.\n");
    printf("\nPractical consequence: the calibration is tied to THIS PCA9685.\n"
           "Swap the board and all twelve servos shift together. Worth ten\n"
           "minutes with a scope before the recalibration you have planned.\n");

    records[6].result = Result::skip;
    snprintf(records[6].note, sizeof records[6].note, "implied by move_to_angle");
    printf("\n    Recorded as SKIP.\n");
}


// T7: SpotServo validation #1 -- command past the limit, confirm saturation
// within +/- 5 degrees of the bound.
static void test_spotservo_limits() {
    printf("\n--- SpotServo #1: limit saturation ---\n");
    printf("Desired behavior: commanding past a joint's limit saturates at\n"
           "that limit rather than driving into the mechanical stop.\n");
    printf("Acceptance: the joint stops within +/- %.0f degrees of its bound.\n",
           (double)SPOT_LIMIT_TOL);
    printf("\nSAFETY: body on a stand, legs clear, bench supply current-limited.\n"
           "If the clamp is broken this is exactly what stalls a servo -- keep\n"
           "a hand on the power switch and listen for buzzing.\n");
    printf("Run this BEFORE the protractor test: it is the one that protects\n"
           "the hardware.\n");

    const int choice = ask_int("Servo number (0 = walk all twelve)", 0, 12);
    ensure_pca_ready();

    const int first = (choice == 0) ? 1  : choice;
    const int last  = (choice == 0) ? 12 : choice;

    for (int num = first; num <= last; ++num) {
        SpotServo joint(num);
        const AngleLimit lim = limit_for(joint.get_type());

        printf("\n  == Servo %d (%s), limits [%.0f, %.0f], center count %d ==\n",
               num, type_name(joint.get_type()),
               (double)lim.min, (double)lim.max, joint.get_center_count());

        wait_for_enter("Ready: commanding the MINIMUM limit");
        joint.move_to_angle(lim.min);
        printf("      commanded %.0f (the bound). Mark this position.\n", (double)lim.min);
        vTaskDelay(pdMS_TO_TICKS(800));

        wait_for_enter("Commanding 30 degrees BELOW the limit -- it must not move");
        joint.move_to_angle(lim.min - 30.0f);
        printf("      commanded %.0f, should saturate at %.0f (+/- %.0f)\n",
               (double)(lim.min - 30.0f), (double)lim.min, (double)SPOT_LIMIT_TOL);
        vTaskDelay(pdMS_TO_TICKS(800));

        wait_for_enter("Ready: commanding the MAXIMUM limit");
        joint.move_to_angle(lim.max);
        printf("      commanded %.0f (the bound). Mark this position.\n", (double)lim.max);
        vTaskDelay(pdMS_TO_TICKS(800));

        wait_for_enter("Commanding 30 degrees ABOVE the limit -- it must not move");
        joint.move_to_angle(lim.max + 30.0f);
        printf("      commanded %.0f, should saturate at %.0f (+/- %.0f)\n",
               (double)(lim.max + 30.0f), (double)lim.max, (double)SPOT_LIMIT_TOL);
        vTaskDelay(pdMS_TO_TICKS(800));
    }

    printf("\n    PASS only if every joint held still on BOTH out-of-range\n"
           "    commands, within +/- %.0f degrees of its bound. Any motion\n"
           "    means the clamp is not holding.\n", (double)SPOT_LIMIT_TOL);
    ask_result(7);
}


// T8: SpotServo validation #2 -- paper protractor at three angles per joint.
static void test_spotservo_protractor() {
    printf("\n--- SpotServo #2: protractor angles ---\n");
    printf("Desired behavior: the joint moves to the commanded angle in the\n"
           "Spot's frame of reference.\n");
    printf("\nMethod: paper protractor cut for one angle below center, center,\n"
           "and one above, taped at the joint and eyeballed. Joint slop makes\n"
           "anything finer meaningless -- what you are really checking is\n"
           "DIRECTION and rough magnitude.\n");
    printf("SAFETY: body on a stand, legs clear.\n");

    const int num = ask_int("Servo number", 1, 12);
    ensure_pca_ready();
    SpotServo joint(num);

    const AngleLimit lim = limit_for(joint.get_type());
    printf("\n    Servo %d: type %s, limits [%.0f, %.0f], center count %d\n",
           num, type_name(joint.get_type()),
           (double)lim.min, (double)lim.max, joint.get_center_count());

    // The angle that maps to center_count differs by joint type: the hip
    // formula is centered on 0, the upper on 45, the lower on 90.
    float center_angle = 0.0f;
    switch (joint.get_type()) {
        case ServoType::hip:      center_angle =  0.0f; break;
        case ServoType::shoulder: center_angle = 45.0f; break;
        case ServoType::knee:     center_angle = 90.0f; break;
        case ServoType::free:     center_angle =  0.0f; break;
    }
    printf("    Angle that maps to center_count: %.0f\n", (double)center_angle);
    if (center_angle < lim.min || center_angle > lim.max) {
        printf("    ! That center angle lies OUTSIDE this joint's limits, so\n"
               "      move_to_angle will clamp and the joint can never reach\n"
               "      its own center count. Worth resolving before trusting\n"
               "      any result here.\n");
    }

    float angle;
    while (ask_float("Angle to command", &angle)) {
        const float effective = angle < lim.min ? lim.min
                              : angle > lim.max ? lim.max
                              : angle;
        if (effective != angle) {
            printf("      ! %.1f is outside [%.0f, %.0f], will clamp to %.1f\n",
                   (double)angle, (double)lim.min, (double)lim.max, (double)effective);
        }
        joint.move_to_angle(angle);
        printf("      commanded %.1f deg -> check the protractor\n", (double)angle);
        vTaskDelay(pdMS_TO_TICKS(600));
    }

    printf("\n    PASS if the joint moved in the DIRECTION the convention says\n"
           "    it should and landed roughly where the paper says. A joint\n"
           "    that moves the wrong way is a sign error, not a calibration\n"
           "    error -- note which joint, since the left/right pairs use\n"
           "    mirrored formulas.\n");
    ask_result(8);
}


//////////////////// MENU ////////////////////

static void print_menu() {
    printf("\n=====================================================\n");
    printf("  TPUP HARDWARE TEST SESSION\n");
    printf("=====================================================\n");
    for (int i = 0; i < NUM_TESTS; ++i) {
        printf("  %d) %-32s [%s]\n", i, records[i].name, result_str(records[i].result));
    }
    printf("  s) summary\n");
    printf("  q) quit\n");
    printf("-----------------------------------------------------\n");
    printf("  Order matters twice:\n");
    printf("    - run 4 (six-face) BEFORE 3 (calibrate)\n");
    printf("    - run 7 (limits) BEFORE 8 (protractor)\n");
    printf("Select: ");
}

static void print_summary() {
    printf("\n=====================================================\n");
    printf("  SUMMARY\n");
    printf("=====================================================\n");
    int passed = 0, failed = 0, skipped = 0, not_run = 0;
    for (int i = 0; i < NUM_TESTS; ++i) {
        printf("  %-4s  %-32s  %s\n", result_str(records[i].result),
               records[i].name, records[i].note);
        switch (records[i].result) {
            case Result::pass:    ++passed;  break;
            case Result::fail:    ++failed;  break;
            case Result::skip:    ++skipped; break;
            case Result::not_run: ++not_run; break;
        }
    }
    printf("-----------------------------------------------------\n");
    printf("  %d passed, %d failed, %d skipped, %d not run\n",
           passed, failed, skipped, not_run);
    printf("\n  Copy this into test_results.md with today's date and the git\n");
    printf("  SHA. The recorded numbers are the point -- they are what makes\n");
    printf("  a future regression obvious.\n");
    printf("=====================================================\n");
}

void run_test_session() {
    console_init();

    printf("\n\n");
    printf("#####################################################\n");
    printf("  TPUP hardware test session\n");
    printf("  Servo and IMU verification\n");
    printf("#####################################################\n");
    printf("\nBEFORE YOU START:\n");
    printf("  - Robot body on a stand, legs unloaded and free to move\n");
    printf("  - Bench supply with a current limit set\n");
    printf("  - Nothing in the path of a joint that may snap to a new pose\n");
    printf("  - Power switch within reach\n");

    for (;;) {
        print_menu();

        char line[16];
        read_line(line, sizeof line);

        if (line[0] == 'q') break;
        if (line[0] == 's') { print_summary(); continue; }

        char *end = NULL;
        const long choice = strtol(line, &end, 10);
        if (end == line || choice < 0 || choice >= NUM_TESTS) {
            printf("  ! Pick a number from the menu.\n");
            continue;
        }

        switch (choice) {
            case 0: test_self_tests();             break;
            case 1: test_imu_ctor_config();        break;
            case 2: test_imu_saturation();         break;
            case 3: test_imu_calibrate();          break;
            case 4: test_imu_update_six_face();    break;
            case 5: test_servo_move_to_angle();    break;
            case 6: test_move_to_pwm_skipped();    break;
            case 7: test_spotservo_limits();       break;
            case 8: test_spotservo_protractor();   break;
        }
    }

    print_summary();
    printf("\nSession ended. Reset the board to run again.\n");
}
