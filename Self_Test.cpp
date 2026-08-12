#include "Self_Test.hpp"
#include "Common.hpp"
#include "esp_log.h"
#include <cmath>

static const char *TAG = "SELFTEST";

static int cases_run    = 0;
static int cases_failed = 0;

// EFFECTS: Logs a PASS/FAIL line comparing actual against expected and
// records the result in the running tally.
static void expect_near(const char *what, float actual, float expected, float tol) {
    ++cases_run;
    if (std::fabs(actual - expected) <= tol) {
        ESP_LOGI(TAG, "PASS  %-34s  %.9g", what, actual);
    }
    else {
        ++cases_failed;
        ESP_LOGE(TAG, "FAIL  %-34s  got %.9g, expected %.9g", what, actual, expected);
    }
}


// Grants the tests access to IMU's private conversion helpers. Declared as a
// friend in IMU.hpp.
struct IMUSelfTest {

    // MODIFIES: imu.angRange (restored before returning)
    // EFFECTS: Checks ang_conversion() against hand-computed scale factors
    // for all four gyroscope ranges.
    static void test_ang_conversion(IMU &imu) {
        // The MPU6050 reports a signed 16 bit count spanning the full range,
        // so one LSB is (2 * bound) / 65536 degrees per second.
        struct Case { AngularRange range; const char *name; float per_lsb; float bound; };
        const Case cases[] = {
            { AngularRange::plus_minus_250dps,  "ang_conversion 250dps",  500.0f  / 65536.0f,  250.0f },
            { AngularRange::plus_minus_500dps,  "ang_conversion 500dps",  1000.0f / 65536.0f,  500.0f },
            { AngularRange::plus_minus_1000dps, "ang_conversion 1000dps", 2000.0f / 65536.0f, 1000.0f },
            { AngularRange::plus_minus_2000dps, "ang_conversion 2000dps", 4000.0f / 65536.0f, 2000.0f },
        };

        const AngularRange saved = imu.angRange;
        for (const Case &c : cases) {
            imu.angRange = c.range;
            const float conv = imu.ang_conversion();

            // 1. The scale factor itself matches the hand calculation.
            expect_near(c.name, conv, c.per_lsb, c.per_lsb * 1e-4f);

            // 2. The property that actually matters: a full scale reading
            //    lands on the range bound. Catches a factor pasted into the
            //    wrong case, which check 1 alone would miss if both were
            //    edited together.
            expect_near("  -> full scale hits bound", 32767.0f * conv, c.bound, c.bound * 1e-3f);
        }
        imu.angRange = saved;
    }

    // MODIFIES: imu.acclRange (restored before returning)
    // EFFECTS: Checks accl_conversion() against hand-computed scale factors
    // for all four accelerometer ranges.
    // NOTE: Scale factors are m/s^2 per LSB, so the bounds are the range in
    // g scaled by `gravity`.
    static void test_accl_conversion(IMU &imu) {
        struct Case { AccelRange range; const char *name; float per_lsb; float bound; };
        const Case cases[] = {
            { AccelRange::plus_minus_2g,  "accl_conversion 2g",  gravity *  4.0f / 65536.0f, gravity *  2.0f },
            { AccelRange::plus_minus_4g,  "accl_conversion 4g",  gravity *  8.0f / 65536.0f, gravity *  4.0f },
            { AccelRange::plus_minus_8g,  "accl_conversion 8g",  gravity * 16.0f / 65536.0f, gravity *  8.0f },
            { AccelRange::plus_minus_16g, "accl_conversion 16g", gravity * 32.0f / 65536.0f, gravity * 16.0f },
        };

        const AccelRange saved = imu.acclRange;
        for (const Case &c : cases) {
            imu.acclRange = c.range;
            const float conv = imu.accl_conversion();

            expect_near(c.name, conv, c.per_lsb, c.per_lsb * 1e-4f);
            expect_near("  -> full scale hits bound", 32767.0f * conv, c.bound, c.bound * 1e-3f);
        }
        imu.acclRange = saved;
    }
};


void test_only_set_ranges(IMU &imu, AngularRange ang, AccelRange acc) {
    imu.angRange  = ang;
    imu.acclRange = acc;
    imu.config_IMU();   // push the new ranges out to the device
}

bool test_only_vectors_are_zero(const IMU &imu) {
    return imu.raw_angvel.x == 0 && imu.raw_angvel.y == 0 && imu.raw_angvel.z == 0
        && imu.raw_accel.x  == 0 && imu.raw_accel.y  == 0 && imu.raw_accel.z  == 0
        && imu.angvel.x == 0.0f && imu.angvel.y == 0.0f && imu.angvel.z == 0.0f
        && imu.accel.x  == 0.0f && imu.accel.y  == 0.0f && imu.accel.z  == 0.0f
        && imu.angvel_zero.x == 0.0f && imu.angvel_zero.y == 0.0f && imu.angvel_zero.z == 0.0f
        && imu.accel_zero.x  == 0.0f && imu.accel_zero.y  == 0.0f && imu.accel_zero.z  == 0.0f;
}

bool run_self_tests(IMU &imu) {
    cases_run    = 0;
    cases_failed = 0;

    ESP_LOGI(TAG, "==== self tests ====");

    IMUSelfTest::test_ang_conversion(imu);
    IMUSelfTest::test_accl_conversion(imu);

    if (cases_failed == 0) {
        ESP_LOGI(TAG, "==== %d/%d passed ====", cases_run, cases_run);
    }
    else {
        ESP_LOGE(TAG, "==== %d/%d FAILED ====", cases_failed, cases_run);
    }
    return cases_failed == 0;
}
