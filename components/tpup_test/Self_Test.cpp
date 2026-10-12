#include "Self_Test.hpp"
#include "Common.hpp"
#include "Spot_Model.hpp"
#include "Transformer.hpp"
#include "TrapezoidPath.hpp"
#include "esp_log.h"
#include <cmath>
#include <cstdio>
#include <vector>

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


// Tolerance for the transform checks, in mm.
//
// Absolute rather than relative: these coordinates are bounded (|x| stays under
// ~250 mm) and the origin case expects exact zeros, which a relative tolerance
// has nothing to scale against. 1e-3 mm sits about two orders of magnitude
// above one float ULP at 100 mm and far below anything a servo could resolve,
// so it absorbs rounding without masking a real fault -- the mistakes this
// catches (a dropped origin subtraction, a missing y reflection) are wrong by
// tens of millimetres, not microns.
constexpr float TRANSFORM_TOL_MM = 1e-3f;


// Tolerance for the trapezoid path checks, in mm.
//
// Absolute, for the same reasons as above: the coordinates are bounded (|x|
// stays under 60 mm on the path shape) and several expected components are
// exact zeros. The worst rounding in percent_of_path is at a segment
// boundary, where (perimeter * percent - a) subtracts two nearly equal
// numbers around 120 -- that cancellation is bounded by an ULP at 120, about
// 8e-6 mm, spread over the segment that follows. 1e-3 mm sits well above it
// and far below the failures this suite exists to catch, which are a segment
// walked in the wrong direction or a dropped prop factor: wrong by tens of
// millimetres.
constexpr float TRAPEZOID_TOL_MM = 1e-3f;


// EFFECTS: Returns true if every component of a is within tol of b.
static bool vec_near(const vec<float> &a, const vec<float> &b, float tol) {
    return std::fabs(a.x - b.x) <= tol
        && std::fabs(a.y - b.y) <= tol
        && std::fabs(a.z - b.z) <= tol;
}


// EFFECTS: Logs a PASS/FAIL line comparing a single point against expected,
// component-wise within tol, and records the result in the running tally.
// Prints both points on failure -- for a point test, which component drifted
// and by how much is the whole diagnosis.
static void expect_vec_near(const char *what,
                            const vec<float> &actual,
                            const vec<float> &expected,
                            float tol) {
    ++cases_run;
    if (vec_near(actual, expected, tol)) {
        ESP_LOGI(TAG, "PASS  %-34s  (%.4f, %.4f, %.4f)",
                 what, (double)actual.x, (double)actual.y, (double)actual.z);
    }
    else {
        ++cases_failed;
        ESP_LOGE(TAG, "FAIL  %-34s  got (%.4f, %.4f, %.4f), expected (%.4f, %.4f, %.4f)",
                 what, (double)actual.x, (double)actual.y, (double)actual.z,
                 (double)expected.x, (double)expected.y, (double)expected.z);
    }
}


// EFFECTS: Logs a PASS/FAIL line comparing a trapezoid's four corners against
// expected, in the constructor's order, and records ONE result for the set.
// A failure names the first corner that missed, which is what separates a bad
// prop (only the top two corners move) from a bad origin (all four move).
static void expect_corners_near(const char *what,
                                const std::vector<vec<float>> &actual,
                                const std::vector<vec<float>> &expected,
                                float tol) {
    static const char *const names[] = { "bottomLeft", "bottomRight", "topRight", "topLeft" };
    const size_t count = sizeof names / sizeof names[0];

    ++cases_run;
    if (actual.size() != count || expected.size() != count) {
        ++cases_failed;
        ESP_LOGE(TAG, "FAIL  %-34s  got %u corners, expected %u",
                 what, (unsigned)actual.size(), (unsigned)expected.size());
        return;
    }
    for (size_t i = 0; i < count; ++i) {
        if (!vec_near(actual.at(i), expected.at(i), tol)) {
            ++cases_failed;
            ESP_LOGE(TAG, "FAIL  %-34s  %s got (%.4f, %.4f, %.4f), expected (%.4f, %.4f, %.4f)",
                     what, names[i],
                     (double)actual.at(i).x,   (double)actual.at(i).y,   (double)actual.at(i).z,
                     (double)expected.at(i).x, (double)expected.at(i).y, (double)expected.at(i).z);
            return;
        }
    }
    ESP_LOGI(TAG, "PASS  %-34s  four corners within %g mm", what, (double)tol);
}


// EFFECTS: Logs a PASS/FAIL line comparing all four legs against expected,
// component-wise within tol, and records the result in the running tally.
// A failure names the first leg that missed and prints both points: "the quad
// did not match" on its own does not say which corner moved, which is the one
// thing you need to know to tell a bad origin from a bad reflection.
static void expect_quad_near(const char *what,
                             const legQuad<vec<float>> &actual,
                             const legQuad<vec<float>> &expected,
                             float tol) {
    struct Corner { const char *name; const vec<float> *got; const vec<float> *want; };
    const Corner corners[] = {
        { "frontLeft",  &actual.frontLeft,  &expected.frontLeft  },
        { "frontRight", &actual.frontRight, &expected.frontRight },
        { "backLeft",   &actual.backLeft,   &expected.backLeft   },
        { "backRight",  &actual.backRight,  &expected.backRight  },
    };

    ++cases_run;
    for (const Corner &c : corners) {
        if (!vec_near(*c.got, *c.want, tol)) {
            ++cases_failed;
            ESP_LOGE(TAG, "FAIL  %-34s  %s got (%.4f, %.4f, %.4f), expected (%.4f, %.4f, %.4f)",
                     what, c.name,
                     (double)c.got->x,  (double)c.got->y,  (double)c.got->z,
                     (double)c.want->x, (double)c.want->y, (double)c.want->z);
            return;
        }
    }
    ESP_LOGI(TAG, "PASS  %-34s  four legs within %g mm", what, (double)tol);
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

// EFFECTS: Checks Transformer::transform against hand-computed results, for a
// point sitting on each leg's origin and then for four offsets from it.
//
// transform does two separate things and only one of them is obvious. It
// re-expresses each body-frame point in its own leg's frame (subtract that
// leg's origin), and it then MIRRORS the right side across y, so both sides
// can share a single IK solution. The mirror is the half that is easy to
// forget, so the offsets below are picked to be closed under y reflection:
// each one's expected right-leg result is another of the four. one <-> three
// and two <-> four.
//
// NOTE: compared component-wise within TRANSFORM_TOL_MM, not with ==. The
// arithmetic does happen to come out exact in binary32 for these particular
// coordinates, but that is a coincidence of the numbers chosen and not a
// property of transform -- swap in offsets that do not land on representable
// sums and an exact comparison would start failing on rounding alone.
static void test_transformer() {
    const vec<float> offset_one   = { 50.0f,  25.0f, -100.0f};
    const vec<float> offset_two   = {-50.0f,  25.0f, -100.0f};
    const vec<float> offset_three = { 50.0f, -25.0f, -100.0f};
    const vec<float> offset_four  = {-50.0f, -25.0f, -100.0f};

    // Safe to build here: SpotServo's constructor only reads the calibration
    // tables, so no I2C traffic is generated and no joint is commanded.
    const SpotModel TPUP;
    Transformer trans = Transformer(&TPUP);

    legQuad<vec<float>> expected    = {};
    legQuad<vec<float>> gait_output = global_leg_origins;

    // Each leg origin maps to that leg's own frame origin, so all four fall to
    // zero. The mirror is invisible here -- it only turns y into -0.0f.
    trans.transform(gait_output);
    expect_quad_near("transform at leg origins", trans.get_next_points(), expected,
                     TRANSFORM_TOL_MM);

    gait_output = global_leg_origins;
    gait_output.frontLeft  = gait_output.frontLeft  + offset_one;
    gait_output.frontRight = gait_output.frontRight + offset_one;
    gait_output.backLeft   = gait_output.backLeft   + offset_one;
    gait_output.backRight  = gait_output.backRight  + offset_one;
    expected.frontLeft  = offset_one;
    expected.frontRight = offset_three;   // offset_one reflected across y
    expected.backLeft   = offset_one;
    expected.backRight  = offset_three;
    trans.transform(gait_output);
    expect_quad_near("transform offset (+50,+25,-100)", trans.get_next_points(), expected,
                     TRANSFORM_TOL_MM);

    gait_output = global_leg_origins;
    gait_output.frontLeft  = gait_output.frontLeft  + offset_two;
    gait_output.frontRight = gait_output.frontRight + offset_two;
    gait_output.backLeft   = gait_output.backLeft   + offset_two;
    gait_output.backRight  = gait_output.backRight  + offset_two;
    expected.frontLeft  = offset_two;
    expected.frontRight = offset_four;    // offset_two reflected across y
    expected.backLeft   = offset_two;
    expected.backRight  = offset_four;
    trans.transform(gait_output);
    expect_quad_near("transform offset (-50,+25,-100)", trans.get_next_points(), expected,
                     TRANSFORM_TOL_MM);

    gait_output = global_leg_origins;
    gait_output.frontLeft  = gait_output.frontLeft  + offset_three;
    gait_output.frontRight = gait_output.frontRight + offset_three;
    gait_output.backLeft   = gait_output.backLeft   + offset_three;
    gait_output.backRight  = gait_output.backRight  + offset_three;
    expected.frontLeft  = offset_three;
    expected.frontRight = offset_one;     // offset_three reflected across y
    expected.backLeft   = offset_three;
    expected.backRight  = offset_one;
    trans.transform(gait_output);
    expect_quad_near("transform offset (+50,-25,-100)", trans.get_next_points(), expected,
                     TRANSFORM_TOL_MM);

    gait_output = global_leg_origins;
    gait_output.frontLeft  = gait_output.frontLeft  + offset_four;
    gait_output.frontRight = gait_output.frontRight + offset_four;
    gait_output.backLeft   = gait_output.backLeft   + offset_four;
    gait_output.backRight  = gait_output.backRight  + offset_four;
    expected.frontLeft  = offset_four;
    expected.frontRight = offset_two;     // offset_four reflected across y
    expected.backLeft   = offset_four;
    expected.backRight  = offset_two;
    trans.transform(gait_output);
    expect_quad_near("transform offset (-50,-25,-100)", trans.get_next_points(), expected,
                     TRANSFORM_TOL_MM);
}

// EFFECTS: Checks TrapezoidPath's corner layout and its arc-length
// parameterization against hand-computed values.
//
// Two separate things are under test here. The constructor places four corners
// from (origin, height, base_width, prop): the base is base_width wide and
// centred on the origin, the top edge sits height above it and is
// base_width * prop wide, also centred. prop is therefore the only input that
// moves the top two corners, and the three shapes below bracket it -- prop < 1
// (a proper trapezoid), prop == 1 (a rectangle) and prop > 1 (a top edge that
// overhangs the base). All three share an origin, height and base_width, so
// the bottom two corners must come out identical across them; only the top
// pair may move.
//
// percent_of_path then walks that outline by ARC LENGTH rather than by segment
// index, which is the part that is easy to get subtly wrong. Equal steps in
// percent are equal distances travelled, so the segment boundaries land at
// cumulative-length fractions, NOT at 0.25 / 0.5 / 0.75 -- a check that only
// sampled the quarter points would pass just as happily against a
// per-segment-quarter implementation, so the cases below deliberately mix
// boundary percents with mid-segment ones.
static void test_TrapezoidPath() {
    // Every percent case runs on this one. 120 wide by 40 tall with prop 0.5
    // gives a 60 mm top edge and slanted sides of sqrt(30^2 + 40^2) = 50, so
    // the side lengths are a = 120, b = 50, c = 60, d = 50 and the perimeter
    // is a round 280. That makes the boundary fractions and every expected
    // point below exact rather than approximate, which is why this shape and
    // not the -100 mm one carries the parameterization cases.
    TrapezoidPath percent_trap   = TrapezoidPath({0.0f, 0.0f,    0.0f}, 40.0f, 120.0f, 0.5f);

    // The corner cases. Same origin, height and base for all three -- only
    // prop differs, so the bottom corners are shared and the top edge comes
    // out 80, 100 and 120 mm wide respectively.
    TrapezoidPath short_side_up  = TrapezoidPath({0.0f, 5.0f, -100.0f}, 50.0f, 100.0f, 0.8f);
    TrapezoidPath rectangle      = TrapezoidPath({0.0f, 5.0f, -100.0f}, 50.0f, 100.0f, 1.0f);
    TrapezoidPath long_side_up   = TrapezoidPath({0.0f, 5.0f, -100.0f}, 50.0f, 100.0f, 1.2f);

    // Order matches the constructor: bottom left (greatest x), bottom right,
    // top right, top left. y is carried through from the origin untouched --
    // the path is planar, so any y drift is a bug on its own.
    const std::vector<vec<float>> expected_short = {
        {50.0f, 5.0f, -100.0f}, {-50.0f, 5.0f, -100.0f}, {-40.0f, 5.0f, -50.0f}, {40.0f, 5.0f, -50.0f}};
    const std::vector<vec<float>> expected_rectangle = {
        {50.0f, 5.0f, -100.0f}, {-50.0f, 5.0f, -100.0f}, {-50.0f, 5.0f, -50.0f}, {50.0f, 5.0f, -50.0f}};
    const std::vector<vec<float>> expected_long = {
        {50.0f, 5.0f, -100.0f}, {-50.0f, 5.0f, -100.0f}, {-60.0f, 5.0f, -50.0f}, {60.0f, 5.0f, -50.0f}};

    expect_corners_near("corners prop 0.8 (tapered)", short_side_up.get_all_points(),
                        expected_short, TRAPEZOID_TOL_MM);
    expect_corners_near("corners prop 1.0 (rectangle)", rectangle.get_all_points(),
                        expected_rectangle, TRAPEZOID_TOL_MM);
    expect_corners_near("corners prop 1.2 (overhanging)", long_side_up.get_all_points(),
                        expected_long, TRAPEZOID_TOL_MM);

    // --- the parameterization ------------------------------------------
    //
    // Three of the percents below (3/7, 17/28, 23/28) sit exactly on a segment
    // boundary. Which side of the comparison they land on after rounding is
    // not fixed, but it does not matter: the end of one segment and the start
    // of the next are the same corner, so both branches agree and the case
    // stays meaningful either way.

    // Start of the cycle: the corner where d meets a, lowest z and greatest x.
    expect_vec_near("path 0% (start corner)",
                    percent_trap.percent_of_path(0.0f),
                    {60.0f, 0.0f, 0.0f}, TRAPEZOID_TOL_MM);

    // Mid-base. 0.25 * 280 = 70 mm along the 120 mm base, travelling -x from
    // x = +60, so x = -10. Note this is NOT a quarter of the way along a --
    // it is well past the middle of it, which is the whole point of measuring
    // by arc length.
    expect_vec_near("path 25% (on base a)",
                    percent_trap.percent_of_path(0.25f),
                    {-10.0f, 0.0f, 0.0f}, TRAPEZOID_TOL_MM);

    // a / perimeter = 120/280 = 3/7: the corner where a meets b, base fully
    // traversed and the climb not yet started.
    expect_vec_near("path 3/7 (corner a->b)",
                    percent_trap.percent_of_path(0.42857142857f),
                    {-60.0f, 0.0f, 0.0f}, TRAPEZOID_TOL_MM);

    // Climbing b. 0.5 * 280 = 140 mm, minus the 120 mm base leaves 20 mm into
    // a 50 mm slant, i.e. 40% up it: x = -60 + 0.4 * 30 = -48, z = 0.4 * 40.
    // Halfway through the cycle is only two fifths of the way up the first
    // slant, which an implementation indexing by segment would get wrong.
    expect_vec_near("path 50% (climbing b)",
                    percent_trap.percent_of_path(0.5f),
                    {-48.0f, 0.0f, 16.0f}, TRAPEZOID_TOL_MM);

    // (a + b) / perimeter = 170/280 = 17/28: corner where b meets c, top of
    // the climb.
    expect_vec_near("path 17/28 (corner b->c)",
                    percent_trap.percent_of_path(0.60714285714f),
                    {-30.0f, 0.0f, 40.0f}, TRAPEZOID_TOL_MM);

    // On the top edge. 0.75 * 280 = 210 mm, minus 170 leaves 40 mm into the
    // 60 mm top edge. c runs in +x, opposite to a, so x = -30 + 40 = +10 --
    // the sign here is what catches a top edge walked backwards.
    expect_vec_near("path 75% (on top edge c)",
                    percent_trap.percent_of_path(0.75f),
                    {10.0f, 0.0f, 40.0f}, TRAPEZOID_TOL_MM);

    // (a + b + c) / perimeter = 230/280 = 23/28: corner where c meets d, and
    // the start of the descent.
    expect_vec_near("path 23/28 (corner c->d)",
                    percent_trap.percent_of_path(0.82142857142f),
                    {30.0f, 0.0f, 40.0f}, TRAPEZOID_TOL_MM);

    // Descending d. 25/28 * 280 = 250 mm, minus 230 leaves 20 mm into the
    // 50 mm slant, 40% of the way down: x = 30 + 0.4 * 30 = 42,
    // z = 40 - 0.4 * 40 = 24. The last case before the path closes back onto
    // the start corner.
    expect_vec_near("path 25/28 (descending d)",
                    percent_trap.percent_of_path(0.89285714285f),
                    {42.0f, 0.0f, 24.0f}, TRAPEZOID_TOL_MM);

    // The base's share of the perimeter, which a gait splits into stance and
    // swing. 120/280 = 3/7. It reads the same lengths the cases above walk, so
    // a perimeter summed over the wrong sides shows up here as well as in the
    // points.
    expect_near("stance_fraction (120/280)",
                percent_trap.stance_fraction(), 120.0f / 280.0f, 1e-6f);
}


bool run_self_tests(IMU &imu) {
    cases_run    = 0;
    cases_failed = 0;

    ESP_LOGI(TAG, "==== self tests ====");

    IMUSelfTest::test_ang_conversion(imu);
    IMUSelfTest::test_accl_conversion(imu);
    test_limit_reachability();
    test_transformer();
    test_TrapezoidPath();

    if (cases_failed == 0) {
        ESP_LOGI(TAG, "==== %d/%d passed ====", cases_run, cases_run);
    }
    else {
        ESP_LOGE(TAG, "==== %d/%d FAILED ====", cases_failed, cases_run);
    }
    return cases_failed == 0;
}
