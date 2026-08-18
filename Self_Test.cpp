#include "Self_Test.hpp"
#include "Common.hpp"
#include "esp_log.h"
#include <cmath>
#include <cstdio>

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


// EFFECTS: Logs a PASS/FAIL line for a boolean condition and records it.
static void expect_true(const char *what, bool ok, const char *detail) {
    ++cases_run;
    if (ok) {
        ESP_LOGI(TAG, "PASS  %-34s  %s", what, detail);
    }
    else {
        ++cases_failed;
        ESP_LOGE(TAG, "FAIL  %-34s  %s", what, detail);
    }
}


SpotAngleMath spot_angle_math(int servo_num, float requested) {
    const Calibration     cal  = calibration_for(servo_num);
    const SpotCalibration spot = spot_calibration_for(servo_num);
    const AngleLimit      lim  = limit_for(spot.type);
    const float           slope = cal.slope;

    SpotAngleMath m = {};

    // Step 1: clamp against the joint limits.
    float angle = requested;
    m.clamped = (requested < lim.min) || (requested > lim.max);
    if (requested < lim.min) angle = lim.min;
    if (requested > lim.max) angle = lim.max;
    m.effective = angle;

    // Step 2: per-servo branch, transcribed from SpotServo::move_to_angle.
    m.branch  = "default (no mapping!)";
    m.formula = "diff_angle = 0";
    switch (servo_num) {
        case 1: case 10:
            m.diff_angle = -1.0f / slope * angle;
            m.branch = "left hip";    m.formula = "diff_angle = -(1/slope) * angle";
            break;
        case 4: case 7:
            m.diff_angle =  1.0f / slope * angle;
            m.branch = "right hip";   m.formula = "diff_angle = +(1/slope) * angle";
            break;
        case 2: case 8:
            m.diff_angle =  1.0f / slope * (45.0f - angle);
            m.branch = "left upper";  m.formula = "diff_angle = +(1/slope) * (45 - angle)";
            break;
        case 5: case 11:
            m.diff_angle = -1.0f / slope * (45.0f - angle);
            m.branch = "right upper"; m.formula = "diff_angle = -(1/slope) * (45 - angle)";
            break;
        case 3: case 9:
            m.diff_angle = -1.0f / slope * (90.0f - angle);
            m.branch = "left lower";  m.formula = "diff_angle = -(1/slope) * (90 - angle)";
            break;
        case 6: case 12:
            m.diff_angle =  1.0f / slope * (90.0f - angle);
            m.branch = "right lower"; m.formula = "diff_angle = +(1/slope) * (90 - angle)";
            break;
        default:
            break;
    }

    // Steps 3-5: degrees -> microseconds -> counts -> absolute count.
    m.diff_width  = m.diff_angle * (1000.0f / 90.0f);
    m.diff_count  = (int)roundf(m.diff_width / (float)period * 4096.0f);
    m.final_count = spot.center_count + m.diff_count;
    m.final_width = (float)m.final_count / 4096.0f * (float)period;
    return m;
}


// EFFECTS: Checks that both ends of every joint's declared angle limit map to
// a pulse the servo can actually act on.
//
// This is the check that catches the mismatch between limit_for(), which is
// per joint TYPE, and center_count/slope, which are per SERVO. Two shoulders
// with different center counts have different reachable ranges, but they
// share one AngleLimit -- so a limit is only valid if the WORST servo of that
// type can reach it. Nothing else in the codebase ties those two halves
// together, which is how an unreachable limit goes unnoticed until a joint
// silently refuses to move.
static void test_limit_reachability() {
    for (int num = 1; num <= 12; ++num) {
        const SpotCalibration spot = spot_calibration_for(num);
        const AngleLimit      lim  = limit_for(spot.type);

        char label[40];
        snprintf(label, sizeof label, "servo %2d (%s) limits", num, type_name(spot.type));

        // A reversed limit makes the clamp send every angle to one bound or
        // the other, so check the ordering before anything else.
        if (lim.min > lim.max) {
            char detail[96];
            snprintf(detail, sizeof detail,
                     "min %.1f > max %.1f -- reversed, clamp is broken",
                     (double)lim.min, (double)lim.max);
            expect_true(label, false, detail);
            continue;
        }

        const SpotAngleMath lo = spot_angle_math(num, lim.min);
        const SpotAngleMath hi = spot_angle_math(num, lim.max);

        const SpotAngleMath *ends[] = { &lo, &hi };
        bool ok = true;
        for (const SpotAngleMath *m : ends) {
            if (m->final_count < 0 || m->final_count > 4095) ok = false;
            if (m->final_width < SERVO_MIN_PULSE_US)         ok = false;
            if (m->final_width > SERVO_MAX_PULSE_US)         ok = false;
        }

        char detail[112];
        snprintf(detail, sizeof detail,
                 "%+.0f->%d(%.0fus)  %+.0f->%d(%.0fus)",
                 (double)lim.min, lo.final_count, (double)lo.final_width,
                 (double)lim.max, hi.final_count, (double)hi.final_width);
        expect_true(label, ok, detail);
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
    test_limit_reachability();

    if (cases_failed == 0) {
        ESP_LOGI(TAG, "==== %d/%d passed ====", cases_run, cases_run);
    }
    else {
        ESP_LOGE(TAG, "==== %d/%d FAILED ====", cases_failed, cases_run);
    }
    return cases_failed == 0;
}
