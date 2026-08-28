#include "Test_Session.hpp"
#include "Self_Test.hpp"
#include "Common.hpp"
#include "IMU.hpp"
#include "Servo.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_app_desc.h"
#include "nvs_flash.h"
#include "nvs.h"
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
constexpr float SERVO_ANGLE_TOL = 2.0f;   // deg, Servo::move_to_angle (+/- 2 deg)
// No angular tolerance for the SpotServo limit test: the criterion is now
// physical (no contact at a bound, no motion past one), not "within N degrees
// of the bound".

// How far past a bound the limit test commands. Only needs to be enough that
// the clamp is unambiguous -- the clamp runs before the count math, so a
// bigger overshoot produces the exact same final count, just an alarming
// number on screen.
constexpr float LIMIT_OVERSHOOT = 10.0f;  // deg

// SERVO_MIN_PULSE_US / SERVO_MAX_PULSE_US come from Self_Test.hpp, so the
// self test and the interactive tests judge reachability by the same rule.
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

// Repeat-to-zero endurance diagnostic: 300 commands to 0 degrees, split into
// three blocks so a wiring or mounting change can be made between them and the
// behavior compared block to block. It has no single pass/fail behavior, so it
// records no verdict -- see the diagnostic below.
constexpr int REPEAT_BLOCK_SIZE   = 100;
constexpr int REPEAT_NUM_BLOCKS   = 3;
constexpr int REPEAT_DELAY_MS     = 100;  // between consecutive commands
constexpr int REPEAT_PROGRESS_EVERY = 10; // print a progress line this often


//////////////////// CONSOLE HELPERS ////////////////////

// EFFECTS: Makes stdin/stdout unbuffered so typed characters arrive one at a
// time and prompts appear without waiting for a newline.
static void console_init() {
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
}

// EFFECTS: Discards anything already sitting in the input buffer.
// Terminals send Enter as CRLF: read_line stops on the '\r' and leaves the
// '\n' behind, which would otherwise satisfy the NEXT prompt instantly and
// make the six-face test skip straight through its faces.
static void drain_input() {
    while (fgetc(stdin) != EOF) { }
    clearerr(stdin);
}

// EFFECTS: Blocks until the user types a line, echoing as they go. Yields to
// the scheduler between polls so the idle task still runs.
static void read_line(char *buf, size_t n) {
    size_t i = 0;
    drain_input();
    for (;;) {
        int c = fgetc(stdin);
        if (c == EOF) {
            // Clear the EOF indicator, or stdio latches it and every later
            // read returns EOF without ever touching the UART again.
            clearerr(stdin);
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

// EFFECTS: Prints prompt and waits for the user to press Enter, or to type q
// and press Enter. Returns true if they asked to quit, so a caller running a
// long walk can stop between steps rather than only at the end of it.
static bool wait_for_enter_or_quit(const char *prompt) {
    char scratch[8];
    printf("\n    >> %s, then press Enter (q + Enter to quit)... ", prompt);
    read_line(scratch, sizeof scratch);
    return scratch[0] == 'q' || scratch[0] == 'Q';
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

// Key codes returned by read_key(). A printable key comes back as its own
// character code; the arrow keys arrive as multi-byte escape sequences, so
// they are folded into these values, chosen to sit above the byte range.
enum {
    KEY_UP = 256, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_ESC
};

// EFFECTS: Reads one character, giving up after timeout_ms. Returns -1 if
// nothing arrived in time.
static int read_char_timeout(int timeout_ms) {
    for (int waited = 0; waited <= timeout_ms; waited += 10) {
        const int c = fgetc(stdin);
        if (c != EOF) return c;
        clearerr(stdin);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return -1;
}

// EFFECTS: Blocks until a key is pressed, then returns its character code or
// one of the KEY_* values above. Yields to the scheduler between polls so the
// idle task still runs.
// NOTE: An arrow key is not one byte -- the terminal sends "ESC [ C" (or
// "ESC O C" when it is in application cursor mode). The bytes after the ESC
// are already in flight, so a short wait tells a real arrow apart from a bare
// ESC press: a timeout means the ESC stood alone.
static int read_key() {
    for (;;) {
        const int c = fgetc(stdin);
        if (c == EOF) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (c != 0x1B) return c;

        const int second = read_char_timeout(60);
        if (second != '[' && second != 'O') return KEY_ESC;
        switch (read_char_timeout(60)) {
            case 'A': return KEY_UP;
            case 'B': return KEY_DOWN;
            case 'C': return KEY_RIGHT;
            case 'D': return KEY_LEFT;
            default:  return KEY_ESC;
        }
    }
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

static void save_results();   // defined below, in the persistence section

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
    save_results();
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


//////////////////// PERSISTENCE ////////////////////

// Results survive reboots so a session can be run across several sittings.
// Stored in NVS as one blob; only the verdict and note are saved, since the
// names are compile-time constants.

static const char *NVS_NAMESPACE = "tpup_test";
static const char *NVS_KEY       = "results";

// Bump when the layout of PersistedRecord or the test list changes, so stale
// blobs from an older firmware are discarded rather than misread.
constexpr uint8_t PERSIST_VERSION = 3;

struct PersistedBlob {
    uint8_t version;
    uint8_t count;
    struct {
        uint8_t result;
        char    note[64];
    } entries[NUM_TESTS];
};

// EFFECTS: Brings up NVS, formatting it if the partition is new or was left
// in a bad state. Safe to call repeatedly.
static void nvs_ready() {
    static bool done = false;
    if (done) return;
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        printf("  ! NVS unavailable (%s); results will not persist.\n", esp_err_to_name(err));
    }
    done = true;
}

// EFFECTS: Writes the current verdicts to NVS.
static void save_results() {
    nvs_ready();
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;

    PersistedBlob blob = {};
    blob.version = PERSIST_VERSION;
    blob.count   = NUM_TESTS;
    for (int i = 0; i < NUM_TESTS; ++i) {
        blob.entries[i].result = (uint8_t)records[i].result;
        strncpy(blob.entries[i].note, records[i].note, sizeof blob.entries[i].note - 1);
    }

    nvs_set_blob(h, NVS_KEY, &blob, sizeof blob);
    nvs_commit(h);
    nvs_close(h);
}

// EFFECTS: Restores verdicts saved by a previous session, if the stored blob
// matches this firmware's layout. Returns true if anything was restored.
static bool load_results() {
    nvs_ready();
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;

    PersistedBlob blob = {};
    size_t len = sizeof blob;
    const esp_err_t err = nvs_get_blob(h, NVS_KEY, &blob, &len);
    nvs_close(h);

    if (err != ESP_OK || len != sizeof blob) return false;
    if (blob.version != PERSIST_VERSION || blob.count != NUM_TESTS) {
        printf("  (stored results are from a different firmware layout; ignored)\n");
        return false;
    }

    int restored = 0;
    for (int i = 0; i < NUM_TESTS; ++i) {
        records[i].result = (Result)blob.entries[i].result;
        strncpy(records[i].note, blob.entries[i].note, sizeof records[i].note - 1);
        records[i].note[sizeof records[i].note - 1] = '\0';
        if (records[i].result != Result::not_run) ++restored;
    }
    if (restored > 0) {
        printf("  Restored %d recorded result(s) from a previous session.\n", restored);
    }
    return restored > 0;
}

// EFFECTS: Resets one test to not-run and persists the change.
static void clear_one(int index) {
    records[index].result = Result::not_run;
    records[index].note[0] = '\0';
    save_results();
    printf("  Cleared \"%s\".\n", records[index].name);
}

// EFFECTS: Resets every test to not-run and persists the change.
static void clear_all() {
    for (int i = 0; i < NUM_TESTS; ++i) {
        records[i].result = Result::not_run;
        records[i].note[0] = '\0';
    }
    save_results();
    printf("  All results cleared.\n");
}


//////////////////// CENTER COUNT OVERRIDES ////////////////////

// A SpotServo's center count is compiled in, in spot_calibration_for()
// (Servo.cpp). Finding a center by hand means trying counts until the horn
// sits right, which is no use if every attempt needs a rebuild -- so the
// session keeps its own override table in NVS, and every center this file
// reads comes from effective_center() rather than straight from the servo.
//
// The override is SESSION-LOCAL: nothing outside this file sees it, so
// SpotServo::move_to_angle still does its math around the compiled center. A
// center that is settled on only becomes real once it is pasted back into
// spot_calibration_for(); the jog tool's 't' key prints that block ready to go.

constexpr int     NUM_SERVO_NUMS = 13;   // servo numbers 0-12
constexpr int16_t NO_OVERRIDE    = -1;

static const char *NVS_KEY_CENTERS = "centers";

// Bump when the layout of CenterBlob changes, so stale blobs from an older
// firmware are discarded rather than misread.
constexpr uint8_t CENTERS_VERSION = 1;

struct CenterBlob {
    uint8_t version;
    uint8_t count;
    int16_t center[NUM_SERVO_NUMS];
};

// EFFECTS: Fills `table` from the centers saved by a previous session, if the
// stored blob matches this firmware's layout. Returns how many overrides were
// restored, leaving `table` untouched when there is nothing to restore.
static int load_centers_into(int16_t *table) {
    nvs_ready();
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return 0;

    CenterBlob blob = {};
    size_t len = sizeof blob;
    const esp_err_t err = nvs_get_blob(h, NVS_KEY_CENTERS, &blob, &len);
    nvs_close(h);

    if (err != ESP_OK || len != sizeof blob) return 0;
    if (blob.version != CENTERS_VERSION || blob.count != NUM_SERVO_NUMS) {
        printf("  (stored centers are from a different firmware layout; ignored)\n");
        return 0;
    }

    int restored = 0;
    for (int i = 0; i < NUM_SERVO_NUMS; ++i) {
        table[i] = blob.center[i];
        if (table[i] != NO_OVERRIDE) ++restored;
    }
    return restored;
}

// EFFECTS: Returns the override table, loading it from NVS on first use. An
// entry of NO_OVERRIDE means "use the compiled-in center".
static int16_t *centers() {
    static int16_t table[NUM_SERVO_NUMS];
    static bool    loaded = false;
    if (!loaded) {
        for (int i = 0; i < NUM_SERVO_NUMS; ++i) table[i] = NO_OVERRIDE;
        // Set before the read, not after: a failed read leaves an empty table,
        // and retrying it on every lookup would hammer NVS.
        loaded = true;
        load_centers_into(table);
    }
    return table;
}

// EFFECTS: Writes the override table to NVS.
static void save_centers() {
    nvs_ready();
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;

    CenterBlob blob = {};
    blob.version = CENTERS_VERSION;
    blob.count   = NUM_SERVO_NUMS;
    for (int i = 0; i < NUM_SERVO_NUMS; ++i) blob.center[i] = centers()[i];

    nvs_set_blob(h, NVS_KEY_CENTERS, &blob, sizeof blob);
    nvs_commit(h);
    nvs_close(h);
}

// REQUIRES: num is 0-12.
// EFFECTS: Returns the center count this session uses for servo `num`: its
// override if one is set, otherwise the value compiled into Servo.cpp.
static int effective_center(int num) {
    if (num >= 0 && num < NUM_SERVO_NUMS && centers()[num] != NO_OVERRIDE) {
        return centers()[num];
    }
    return spot_calibration_for(num).center_count;
}

// REQUIRES: num is 0-12.
// EFFECTS: Returns true if servo `num`'s center has been overridden here.
static bool center_is_overridden(int num) {
    return num >= 0 && num < NUM_SERVO_NUMS && centers()[num] != NO_OVERRIDE;
}

// REQUIRES: num is 0-12, count is a legal 12-bit count.
// EFFECTS: Records `count` as servo `num`'s center for this session and
// persists it.
static void set_center_override(int num, int count) {
    centers()[num] = (int16_t)count;
    save_centers();
}

// REQUIRES: num is 0-12.
// EFFECTS: Drops servo `num`'s override, so the compiled-in center applies
// again.
static void clear_center_override(int num) {
    centers()[num] = NO_OVERRIDE;
    save_centers();
}

// EFFECTS: Returns the enum spelling of `t`. Distinct from type_name(), which
// is worded for a human reading the console; this one has to compile.
static const char *type_enum_name(ServoType t) {
    switch (t) {
        case ServoType::hip:      return "ServoType::hip";
        case ServoType::shoulder: return "ServoType::shoulder";
        case ServoType::knee:     return "ServoType::knee";
        case ServoType::free:     return "ServoType::free";
    }
    return "ServoType::free";
}

// EFFECTS: Prints the body of spot_calibration_for() with this session's
// centers filled in, ready to paste into Servo.cpp. Overridden lines carry the
// compiled value they replace, so a change can be read back or undone by eye.
static void print_center_table() {
    printf("\n\n----- paste over the body of spot_calibration_for() in Servo.cpp -----\n\n");
    for (int n = 0; n < NUM_SERVO_NUMS; ++n) {
        const SpotCalibration c      = spot_calibration_for(n);
        const int             center = effective_center(n);

        char note[64] = "";
        if (center != c.center_count) {
            snprintf(note, sizeof note, "   // was %d", c.center_count);
        }
        else if (n == 0) {
            snprintf(note, sizeof note, "   // no meaningful center");
        }
        printf("        case %-2d: return { %4d, %-20s };%s\n",
               n, center, type_enum_name(c.type), note);
    }
    printf("\n----- end -----\n");
    printf("Until this is pasted in, the centers above live only in this\n"
           "session's NVS: SpotServo::move_to_angle still uses the compiled ones.\n");
}

// EFFECTS: Notes at startup that an earlier session left centers overridden,
// since every later printout of a center silently reflects them.
static void report_center_overrides() {
    int n = 0;
    for (int i = 0; i < NUM_SERVO_NUMS; ++i) {
        if (center_is_overridden(i)) ++n;
    }
    if (n > 0) {
        printf("  %d servo center(s) overridden by an earlier jog session "
               "(menu 'j' to review).\n", n);
    }
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

// EFFECTS: Probes every 7-bit address and reports which devices ACK. This is
// the first thing to run when a device "should" be connected but isn't
// behaving: it separates "nothing on the bus" from "wrong address" from "the
// device is there and the bug is in our code".
static void i2c_scan() {
    printf("\n--- I2C bus scan (SDA=%d, SCL=%d) ---\n", (int)sda_pin, (int)scl_pin);
    printf("Expected: 0x%02X (MPU6050 IMU), 0x%02X (PCA9685 servo driver)\n\n",
           IMU_ADDR, PCA_ADDR);

    int found = 0;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    for (int hi = 0; hi < 8; ++hi) {
        printf("%02x: ", hi * 16);
        for (int lo = 0; lo < 16; ++lo) {
            const int addr = hi * 16 + lo;
            if (addr < 0x08 || addr > 0x77) { printf("   "); continue; }

            i2c_cmd_handle_t ptr = i2c_cmd_link_create();
            i2c_master_start(ptr);
            i2c_master_write_byte(ptr, (addr << 1) | 0, true);
            i2c_master_stop(ptr);
            const esp_err_t err = i2c_master_cmd_begin(i2c_port, ptr, pdMS_TO_TICKS(20));
            i2c_cmd_link_delete(ptr);

            if (err == ESP_OK) { printf("%02x ", addr); ++found; }
            else               { printf("-- "); }
        }
        printf("\n");
    }

    printf("\n  %d device(s) responded.\n", found);
    if (found == 0) {
        printf("  Nothing at all: check power, ground, SDA/SCL wiring, and that\n"
               "  both lines have pull-ups. A bus with no pull-ups reads as\n"
               "  silence exactly like an absent device.\n");
    }
    printf("  IMU  @ 0x%02X: %s\n", IMU_ADDR, "expected for MPU6050 with AD0 low");
    printf("  PCA  @ 0x%02X: %s\n", PCA_ADDR, "expected for PCA9685 with all address jumpers open");
}

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
    save_results();
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

    // Clear any zeros left over from an earlier run of this test. calibrate()
    // captures the ALREADY zero-subtracted readings, so without this the old
    // offsets compound and "before" would show a bias that is not the raw one.
    imu_instance().reset_calibration();

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


// EFFECTS: Asks which servo to drive and resolves its PCA9685 channel,
// bringing the PCA up along the way. Shared by the tests that command a
// single bench servo directly.
//
// Servo 0's led_pos would be num - 1 = -1, and move_to_pwm computes
// LED_ON_L = 6 + 4 * led_pos = 2 -- which is SUBADR1, not a PWM channel.
// Commanding it would scribble over the PCA9685's addressing registers
// instead of moving anything, so ask which channel it is actually wired to
// and let the caller use the explicit-led_pos constructor.
static void ask_servo_and_channel(int *num_out, int *led_pos_out) {
    const int num = ask_int("Servo number (0 = uncalibrated identity)", 0, 12);
    ensure_pca_ready();

    int led_pos = num - 1;
    if (num == 0) {
        printf("\n    Servo 0 has no implied channel (led_pos would be -1, which\n"
               "    maps onto the PCA9685 config registers rather than a PWM\n"
               "    output). Specify the channel it is plugged into.\n");
        led_pos = ask_int("PCA9685 channel", 0, 15);
    }
    printf("    Using PCA9685 channel %d\n", led_pos);

    *num_out     = num;
    *led_pos_out = led_pos;
}


// T5: Servo::move_to_angle. Off robot, single servo, needle attached.
static void test_servo_move_to_angle() {
    printf("\n--- Servo::move_to_angle sweep ---\n");
    printf("Desired behavior: move the servo to the desired angle.\n");
    printf("\nSETUP: OFF the robot, single servo, long needle axially locked to\n"
           "the axle, protractor beneath. Assumes slope and offset are already\n"
           "correct for this servo.\n");
    printf("Acceptance: needle within +/- %.0f degrees of each commanded angle.\n",
           (double)SERVO_ANGLE_TOL);

    int num = 0, led_pos = 0;
    ask_servo_and_channel(&num, &led_pos);
    Servo servo(num, led_pos);

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

    printf("\n    PASS if every reading was within +/- %.0f degrees on the\n"
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
    save_results();
    printf("\n    Recorded as SKIP.\n");
}


// SpotAngleMath plus the two reachability flags this session needs. The
// arithmetic itself lives in Self_Test.cpp so there is exactly one test-side
// copy of the formula rather than one per file.
struct SpotMath : SpotAngleMath {
    bool count_out_of_range;   // outside the 12-bit counter
    bool pulse_unusable;       // in range, but not a pulse a servo obeys
};

// EFFECTS: Runs the shared angle-to-count math for `requested` and classifies
// whether the result is something the hardware can act on.
static SpotMath compute_spot_math(const SpotServo &joint, float requested) {
    SpotMath m = {};
    static_cast<SpotAngleMath &>(m) = spot_angle_math(joint.get_servo_num(), requested);

    m.count_out_of_range = (m.final_count < 0) || (m.final_count > 4095);
    m.pulse_unusable     = !m.count_out_of_range
                        && (m.final_width < SERVO_MIN_PULSE_US
                            || m.final_width > SERVO_MAX_PULSE_US);
    return m;
}


// EFFECTS: Prints every intermediate value for `requested`, without
// commanding anything.
static void print_spot_servo_math(const SpotServo &joint, float requested) {
    const int        num    = joint.get_servo_num();
    const float      slope  = joint.get_slope();
    const int        center = joint.get_center_count();
    const AngleLimit lim    = limit_for(joint.get_type());
    const SpotMath   m      = compute_spot_math(joint, requested);

    const float angle       = m.effective;
    const bool  clamped     = m.clamped;
    const char *branch      = m.branch;
    const char *formula     = m.formula;
    const float diff_angle  = m.diff_angle;
    const float diff_width  = m.diff_width;
    const int   diff_count  = m.diff_count;
    const int   final_count = m.final_count;
    const float final_width = m.final_width;

    printf("\n      +-- servo math -------------------------------------------\n");
    printf("      | servo_num          = %d (%s, %s)\n", num, type_name(joint.get_type()), branch);
    printf("      | requested angle    = %+8.2f deg\n", (double)requested);
    printf("      | limits             = [%.1f, %.1f]\n", (double)lim.min, (double)lim.max);
    printf("      | effective angle    = %+8.2f deg   %s\n", (double)angle,
           clamped ? "<-- CLAMPED" : "(within limits)");
    printf("      |\n");
    printf("      | get_slope()        = %8.4f\n", (double)slope);
    printf("      | 1/slope            = %8.4f\n", (double)(1.0f / slope));
    printf("      | %s\n", formula);
    printf("      | diff_angle         = %+8.3f deg\n", (double)diff_angle);
    printf("      |\n");
    printf("      | usec per degree    = %8.4f  (1000/90)\n", (double)(1000.0f / 90.0f));
    printf("      | diff_width         = %+8.2f us\n", (double)diff_width);
    printf("      |\n");
    printf("      | period             = %8.2f us  (%.2f Hz, prescale %u)\n",
           (double)period, (double)true_pwm_freq, (unsigned)pre_scale);
    printf("      | counts per period  = 4096\n");
    printf("      | us per count       = %8.4f us\n", (double)(period / 4096.0));
    printf("      | diff_count         = %+8d counts  (round(diff_width / period * 4096))\n", diff_count);
    printf("      |\n");
    printf("      | center_count       = %8d counts  (%.1f us)\n",
           center, (double)((float)center / 4096.0f * (float)period));
    printf("      | FINAL count        = %8d counts  (center + diff)\n", final_count);
    printf("      | FINAL pulse width  = %8.1f us\n", (double)final_width);

    if (m.count_out_of_range) {
        printf("      | !! OUT OF COUNT RANGE. move_to_pwm logs an error and\n");
        printf("      |    returns WITHOUT writing, so the servo holds its\n");
        printf("      |    last position -- the command is dropped, not obeyed.\n");
    }
    else if (m.pulse_unusable) {
        printf("      | !! PULSE OUTSIDE SERVO RANGE (%.0f-%.0f us).\n",
               (double)SERVO_MIN_PULSE_US, (double)SERVO_MAX_PULSE_US);
        printf("      |    The count is a legal 12-bit value, so move_to_pwm\n");
        printf("      |    writes it happily -- but the servo will ignore it or\n");
        printf("      |    jam against its internal stop. THIS IS THE USUAL\n");
        printf("      |    REASON A JOINT DOES NOT MOVE ON COMMAND.\n");
    }
    printf("      +----------------------------------------------------------\n");
}


//////////////////// PRE-TEST POSING ////////////////////

// Driving one joint to its limit is only safe if the rest of the robot is out
// of the way. Each joint type needs a different arrangement of the others.
//
// Servo numbering runs in per-leg triples -- (1,2,3) (4,5,6) (7,8,9)
// (10,11,12), each being hip, shoulder, knee -- so the knee belonging to the
// leg under test is always (shoulder number + 1).
//
// The first two triples (servos 1-6) are the FRONT pair of legs and the last
// two (servos 7-12) the REAR pair, following the frontLeft, frontRight,
// backLeft, backRight ordering LegType uses in Common.hpp. Two of the pose
// transitions below depend on that split, so fix it here if the harness is
// ever wired the other way round.

// Where the straighten pass parks a leg: knee fully extended, shoulder swung
// out away from the body. Commanded through move_to_angle, so the knee lands
// on the knee maximum -- limit_for clamps 180 to 176 -- which is the same
// place the shoulder pose asks a knee to be anyway.
constexpr float STRAIGHT_SHOULDER_ANGLE = 90.0f;
constexpr float STRAIGHT_KNEE_ANGLE     = 180.0f;

// Between consecutive servo commands, so a dozen servos do not all draw stall
// current at once. A stagger, not a settle: the joints still move together.
constexpr int POSE_STAGGER_MS = 150;

// A hip coming off a bound has up to 100 degrees to travel back to neutral,
// and by then the straighten pass has extended the leg into one long lever --
// commanded in one go, the servo slams the whole arm from a standstill. The
// same move issued as a run of short commands ends in the same place with a
// fraction of the acceleration. These set how short: no single command asks
// for more than POSE_STEP_MAX_DEG, and each gets POSE_STEP_MS to run before
// the next is sent.
constexpr float POSE_STEP_MAX_DEG = 15.0f;
constexpr int   POSE_STEP_MS      = 120;

// After each step of the straighten pass. This one IS a settle: the whole
// point of straightening is that it finishes before anything else moves, so it
// has to cover the joint's travel time, not just spread the current draw.
constexpr int STRAIGHTEN_SETTLE_MS = 900;

// Between the last REAR servo command and the first FRONT one when the legs
// fold back into the hip pose. The rear pair must be fully contracted before a
// front leg so much as starts moving, and the six rear commands are staggered
// over 750 ms before this hold even begins, so the first rear joint has had
// most of two seconds by the time the front pair is touched. Raise it again if
// a rear leg is ever still moving when the front pair starts.
constexpr int REAR_FIRST_SETTLE_MS = 1000;

// Where the leg under test parks its own knee while its shoulder is swept to
// its bounds. This used to be the knee's center count, which works out to 90
// deg by construction; 70 deg is a set angle instead, so it is commanded
// through move_to_angle like any other posed joint.
constexpr float SHOULDER_TEST_KNEE_ANGLE = 70.0f;

// Servo numbers run 0-12; index 0 is the reserved uncalibrated servo.
constexpr int POSE_TABLE_SIZE = 13;

struct PoseTarget {
    bool  command;      // false: leave this joint where it is
    float angle;
};

// EFFECTS: Returns where `servo` should be placed while `under_test` is being
// swept to its limits.
static PoseTarget pose_target_for(int servo, ServoType servo_type,
                                  int under_test, ServoType test_type) {
    if (servo == under_test) return { false, 0.0f };

    const int leg_under_test = (under_test - 1) / 3;
    const int leg_of_servo   = (servo - 1) / 3;

    switch (test_type) {
        case ServoType::shoulder:
            // Hips centered, other shoulders at 90, other knees folded to
            // 180, and this leg's own knee parked at SHOULDER_TEST_KNEE_ANGLE.
            if (servo_type == ServoType::hip)      return { true,   0.0f };
            if (servo_type == ServoType::shoulder) return { true,  90.0f };
            if (leg_of_servo == leg_under_test)
                return { true, SHOULDER_TEST_KNEE_ANGLE };
            return { true, 180.0f };

        case ServoType::knee:
            // Hips centered, all shoulders at 90. Other knees are left alone.
            if (servo_type == ServoType::hip)      return { true,  0.0f };
            if (servo_type == ServoType::shoulder) return { true, 90.0f };
            return { false, 0.0f };

        case ServoType::hip:
            // Other hips centered, shoulders at 0, knees tucked to 20.
            if (servo_type == ServoType::hip)      return { true,  0.0f };
            if (servo_type == ServoType::shoulder) return { true,  0.0f };
            return { true, 20.0f };

        case ServoType::free:
            return { false, 0.0f };
    }
    return { false, 0.0f };
}


//////////////////// POSE TRANSITIONS ////////////////////

// Which pose the robot was last put in, so a call knows where it is starting
// FROM as well as where it is going. ServoType::free stands for "unknown":
// nothing has been posed yet this run, so the arrangement could be anything.
//
// The two rules below are hardware facts found on the bench, not anything the
// code can derive from the pose tables. Both are about the PATH between two
// poses rather than the poses themselves: each pose is safe to sit in, and the
// collision happens on the way there.
static ServoType last_pose = ServoType::free;

// The angle each servo was last commanded to, indexed by servo number, or NAN
// for one this run has not commanded yet. A ramped move needs to know where it
// is starting from, and nothing on this robot reports its own position.
static float last_angle[POSE_TABLE_SIZE] = {};

// EFFECTS: Forgets where the robot is. Called at the start of a walk: whatever
// an earlier test, the jog tool, or a hand left it in, the session cannot know.
static void reset_pose_memory() {
    last_pose = ServoType::free;
    for (int i = 0; i < POSE_TABLE_SIZE; ++i) last_angle[i] = NAN;
}

// EFFECTS: How many commands a pose move of servo `n` to `target` should be
// split into. Only a HIP ramps: it is the joint that has to come off a bound
// with the leg extended into a long lever, which is the move that was stressing
// the joint. Everything else keeps its single command. 1 means no ramp -- which
// is also the answer when the session has no idea where the joint is starting
// from, since there is nothing to interpolate.
static int pose_steps_for(int n, ServoType type, float target) {
    if (type != ServoType::hip || std::isnan(last_angle[n])) return 1;
    const int steps = (int)std::ceil(std::fabs(target - last_angle[n]) / POSE_STEP_MAX_DEG);
    return steps < 1 ? 1 : steps;
}

// MODIFIES: servo `n`.
// EFFECTS: Commands `n` to `target`, spread over `steps` commands from wherever
// the session last left it. Every step is a real move_to_angle, so the joint
// limits apply to each one exactly as they would to the single command this
// replaces, and the joint ends up in the same place either way.
static void move_pose_angle(SpotServo &s, int n, float target, int steps) {
    if (steps <= 1 || std::isnan(last_angle[n])) {
        s.move_to_angle(target);
    }
    else {
        const float from = last_angle[n];
        for (int i = 1; i <= steps; ++i) {
            s.move_to_angle(from + (target - from) * (float)i / (float)steps);
            vTaskDelay(pdMS_TO_TICKS(POSE_STEP_MS));
        }
    }
    last_angle[n] = target;
}

// EFFECTS: True if the move from `from` to `to` has to open the legs out
// straight before it commands the final pose. Going from the hip pose, where
// the legs sit tucked (shoulders at 0, knees folded to 20), into the shoulder
// pose swings each leg out while it is still folded, and the joints of a leg
// collide on the way -- seen on the bench. An UNKNOWN starting pose is treated
// the same, since it may well be the hip pose.
static bool needs_straighten_first(ServoType from, ServoType to) {
    return to == ServoType::shoulder
        && (from == ServoType::hip || from == ServoType::free);
}

// EFFECTS: True if the move from `from` to `to` has to fold the rear legs
// before the front ones. Coming out of the knee pose into the hip pose every
// leg contracts, and the rear pair has to be all the way in before a front leg
// starts moving. An UNKNOWN starting pose is treated the same: a walk that was
// quit part way through leaves the robot in whichever pose it got to, and the
// knee pose is one of them.
static bool needs_rear_first(ServoType from, ServoType to) {
    return to == ServoType::hip
        && (from == ServoType::knee || from == ServoType::free);
}

// MODIFIES: every shoulder and knee on the robot, except those on `skip_leg`.
// EFFECTS: Opens the legs out straight -- shoulders first, then knees -- with
// the FRONT pair finishing completely before the REAR pair moves at all, and
// each step given time to arrive before the next starts. `skip_leg` is a
// zero-based leg index to leave untouched, or -1 to open all four. Records
// what each joint was commanded to in `straightened`, indexed by servo number,
// leaving NAN for the hips, which this pass does not touch, and for anything
// on the skipped leg.
static void straighten_legs(float *straightened, int skip_leg) {
    printf("\n     -- Straighten pass --\n");
    printf("        shoulders -> %.0f deg, then knees -> %.0f deg (clamped to the\n"
           "        knee maximum, %.0f). Front pair first, then the rear pair,\n"
           "        %d ms per step to arrive.\n",
           (double)STRAIGHT_SHOULDER_ANGLE, (double)STRAIGHT_KNEE_ANGLE,
           (double)limit_for(ServoType::knee).max, STRAIGHTEN_SETTLE_MS);
    if (skip_leg >= 0) {
        printf("        Leg %d (servos %d-%d) is left where it is.\n",
               skip_leg + 1, skip_leg * 3 + 1, skip_leg * 3 + 3);
    }

    // Shoulders before knees: the leg swings clear of the body while it is
    // still folded up, and only extends once it is out there.
    const ServoType joint_order[] = { ServoType::shoulder, ServoType::knee };

    for (int half = 0; half < 2; ++half) {
        const bool front = (half == 0);
        const int  lo    = front ?  1 :  7;
        const int  hi    = front ?  6 : 12;
        printf("        %s pair (servos %d-%d):\n",
               front ? "FRONT" : "REAR", lo, hi);

        for (const ServoType which : joint_order) {
            const float target = (which == ServoType::shoulder)
                               ? STRAIGHT_SHOULDER_ANGLE
                               : STRAIGHT_KNEE_ANGLE;
            for (int n = lo; n <= hi; ++n) {
                if (skip_leg >= 0 && (n - 1) / 3 == skip_leg) continue;
                SpotServo s(n);
                if (s.get_type() != which) continue;
                s.move_to_angle(target);
                straightened[n] = target;
                last_angle[n]   = target;
                printf("          servo %2d (%-8s): %+.0f deg\n",
                       n, type_name(s.get_type()), (double)target);
                vTaskDelay(pdMS_TO_TICKS(POSE_STAGGER_MS));
            }
            vTaskDelay(pdMS_TO_TICKS(STRAIGHTEN_SETTLE_MS));
        }
    }
    printf("        Legs straight.\n");
}

// EFFECTS: Commands servo `n` to its place in the pose, or says why it is
// being left alone. Returns true if a command was actually sent. `straightened`
// holds what the straighten pass commanded each servo, or NAN for one it did
// not touch, so a joint that pass already parked on its pose target is not
// commanded a second time. `deferred` is a servo the fold-back pass will place
// afterwards, or 0 for none.
static bool command_pose_servo(int n, int under_test, ServoType test_type,
                               const float *straightened, int deferred) {
    SpotServo s(n);
    const PoseTarget t = pose_target_for(n, s.get_type(), under_test, test_type);

    if (n == deferred) {
        printf("      servo %2d (%-8s): held for the fold-back pass\n",
               n, type_name(s.get_type()));
        return false;
    }

    if (!t.command) {
        printf("      servo %2d (%-8s): %s\n", n, type_name(s.get_type()),
               n == under_test ? "UNDER TEST, not posed" : "left as-is");
        return false;
    }

    // Compared as REQUESTED angles, which is what both sides are: the
    // straighten pass and the pose both ask a knee for 180, and move_to_angle
    // clamps both to the same 176.
    if (!std::isnan(straightened[n])
        && std::fabs(straightened[n] - t.angle) < 0.5f) {
        printf("      servo %2d (%-8s): %+.0f deg already, from the straighten pass\n",
               n, type_name(s.get_type()), (double)t.angle);
        return false;
    }

    const int steps = pose_steps_for(n, s.get_type(), t.angle);
    if (steps > 1) {
        printf("      servo %2d (%-8s): %+.0f deg, ramped from %+.0f in %d steps"
               " of %.0f deg\n",
               n, type_name(s.get_type()), (double)t.angle, (double)last_angle[n],
               steps, (double)(std::fabs(t.angle - last_angle[n]) / (float)steps));
    }
    else {
        printf("      servo %2d (%-8s): %+.0f deg\n",
               n, type_name(s.get_type()), (double)t.angle);
    }
    move_pose_angle(s, n, t.angle, steps);
    // Stagger so a dozen servos do not all draw stall current at once.
    vTaskDelay(pdMS_TO_TICKS(POSE_STAGGER_MS));
    return true;
}

// MODIFIES: the knee on the leg under test.
// EFFECTS: Puts the leg under test into the angles the SHOULDER POSE asks for,
// after the straighten pass has taken it out of them and after every other
// joint is placed. pose_target_for skips the joint under test, so those two
// angles are the ones it hands that leg's peers: 90 deg for a shoulder -- where
// the straighten pass already left it, so nothing is re-commanded -- and
// SHOULDER_TEST_KNEE_ANGLE for the knee.
//
// Straightening is a way THROUGH the transition, not a place to test from. This
// is the last thing to move before the first bound is commanded.
static void fold_back_leg(int shoulder) {
    const int knee = shoulder + 1;   // per-leg triples: hip, shoulder, knee

    printf("\n     -- Fold-back pass (the leg under test) --\n");
    printf("        Servos %d and %d take the angles the shoulder pose asks for,\n"
           "        now that the straighten pass is done with them.\n",
           shoulder, knee);

    printf("        servo %2d (shoulder): %+.0f deg already, from the straighten"
           " pass\n", shoulder, (double)STRAIGHT_SHOULDER_ANGLE);

    SpotServo kn(knee);
    kn.move_to_angle(SHOULDER_TEST_KNEE_ANGLE);
    last_angle[knee] = SHOULDER_TEST_KNEE_ANGLE;
    printf("        servo %2d (knee    ): %+.0f deg\n",
           knee, (double)SHOULDER_TEST_KNEE_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(STRAIGHTEN_SETTLE_MS));
}

// EFFECTS: Describes and then commands the pre-test pose for the joint under
// test, sequencing the transition out of the previous pose where that
// transition needs it. Every other joint moves, so this pauses for
// confirmation first. Returns false if the operator asked to quit at that
// prompt, in which case nothing has been commanded.
static bool prepare_safe_pose(int under_test, ServoType test_type) {
    printf("\n  -- Pre-test pose (servo %d, %s under test) --\n",
           under_test, type_name(test_type));
    printf("     coming from: %s pose\n",
           last_pose == ServoType::free ? "unknown (nothing posed yet)"
                                        : type_name(last_pose));

    switch (test_type) {
        case ServoType::shoulder:
            printf("     hips -> 0 deg, other shoulders -> 90 deg,\n"
                   "     other knees -> 180 deg, this leg's knee -> %.0f deg\n",
                   (double)SHOULDER_TEST_KNEE_ANGLE);
            break;
        case ServoType::knee:
            printf("     hips -> 0 deg, all shoulders -> 90 deg\n"
                   "     (other knees left where they are)\n");
            break;
        case ServoType::hip:
            printf("     other hips -> 0 deg, all shoulders -> 0 deg,\n"
                   "     all knees -> 20 deg\n");
            break;
        case ServoType::free:
            printf("     no pose defined for a free servo; nothing will move\n");
            return true;
    }

    const bool straighten = needs_straighten_first(last_pose, test_type);
    const bool rear_first = needs_rear_first(last_pose, test_type);

    if (straighten) {
        printf("\n     TRANSITION RULE (%s pose -> shoulder pose): the legs open\n"
               "     out straight FIRST -- front pair, then rear pair -- and only\n"
               "     then move into the pose above. Going there directly folds a\n"
               "     leg's joints into each other. The leg under test then takes\n"
               "     its posed angles back, before any bound is commanded.\n",
               last_pose == ServoType::free ? "unknown" : "hip");
    }
    if (rear_first) {
        printf("\n     TRANSITION RULE (%s pose -> hip pose): the REAR legs\n"
               "     contract FIRST and get %d ms to finish before a single front\n"
               "     joint is commanded.\n",
               last_pose == ServoType::free ? "unknown" : "knee",
               REAR_FIRST_SETTLE_MS);
    }

    printf("\n     ELEVEN joints are about to move. Keep hands clear and make\n"
           "     sure the legs have room to swing.\n");
    if (wait_for_enter_or_quit("Clear of the robot")) return false;

    // The knee the fold-back pass will place once everything else is posed.
    // straighten only ever fires for a shoulder test, so under_test is the
    // shoulder and under_test + 1 is its knee.
    const int fold_knee = straighten ? under_test + 1 : 0;

    float straightened[POSE_TABLE_SIZE];
    for (int i = 0; i < POSE_TABLE_SIZE; ++i) straightened[i] = NAN;
    if (straighten) straighten_legs(straightened, -1);

    printf("\n     -- Pose pass --\n");
    if (rear_first) {
        printf("        REAR legs (servos 7-12) contract first:\n");
        for (int n = 7; n <= 12; ++n) {
            command_pose_servo(n, under_test, test_type, straightened, fold_knee);
        }
        printf("        Rear pair commanded. Holding %d ms for it to finish\n"
               "        contracting before the front pair is touched.\n",
               REAR_FIRST_SETTLE_MS);
        vTaskDelay(pdMS_TO_TICKS(REAR_FIRST_SETTLE_MS));
        printf("        FRONT legs (servos 1-6):\n");
        for (int n = 1; n <= 6; ++n) {
            command_pose_servo(n, under_test, test_type, straightened, fold_knee);
        }
    }
    else {
        for (int n = 1; n <= 12; ++n) {
            command_pose_servo(n, under_test, test_type, straightened, fold_knee);
        }
    }

    if (straighten) fold_back_leg(under_test);

    printf("     Pose set.\n");
    last_pose = test_type;
    vTaskDelay(pdMS_TO_TICKS(400));
    return true;
}


//////////////////// RESTING POSE ////////////////////

// Where a leg that is not being looked at parks: hip centered, shoulder and
// knee at the straighten-pass angles, which together put the leg out straight
// and pointing at the ground under its own hip. The knee angle goes through
// move_to_angle like any other, so it clamps to the knee maximum (176 deg) --
// four degrees short of straight is as straight as that joint gets.
constexpr float REST_HIP_ANGLE = 0.0f;

// MODIFIES: the shoulders and knees of every leg the pose straightens, and the
// hips of the three legs the joint under test is not on.
// EFFECTS: Parks those three legs at the resting angles and leaves the leg
// under test exactly where it is, so the only thing that moves while a
// protractor is being read is the joint being read. `first_pose` says this is
// the first pose of the walk, where the leg under test is straightened along
// with the rest: the robot is coming from an unknown position and that leg
// wants a known one as much as its neighbours do. Every later pose leaves it
// out, since nothing but the reads themselves has moved it since.
//
// Sequenced the way every other pose change on this robot is: all four legs
// (or three) straighten FIRST -- shoulders before knees, so a folded leg
// swings clear of the body before it extends, and the FRONT pair before the
// REAR pair -- then the hips come to center, last and ramped, since by then
// the leg they are turning is extended into a long lever. Returns false if the
// operator quit at the confirmation prompt, in which case nothing has been
// commanded.
static bool rest_other_legs(int under_test, bool first_pose) {
    const int leg  = (under_test - 1) / 3;
    const int skip = first_pose ? -1 : leg;

    printf("\n  -- Resting pose (the other three legs) --\n");
    printf("     hip -> %.0f deg, shoulder -> %.0f deg, knee -> %.0f deg: the leg\n"
           "     straightened out and pointing at the ground.\n",
           (double)REST_HIP_ANGLE, (double)STRAIGHT_SHOULDER_ANGLE,
           (double)STRAIGHT_KNEE_ANGLE);
    if (first_pose) {
        printf("     ALL FOUR legs straighten first -- front pair, then rear pair\n"
               "     -- leg %d included, since the robot is coming from a position\n"
               "     the session cannot know. Only its HIP is left alone, and\n"
               "     from here on nothing but your own commands moves that leg.\n",
               leg + 1);
    }
    else {
        printf("     Leg %d (servos %d-%d) is NOT posed. The joint under test has to\n"
               "     stay where the protractor is reading it, and the two joints it\n"
               "     shares a leg with are left alone with it.\n",
               leg + 1, leg * 3 + 1, leg * 3 + 3);
    }
    printf("\n     %s joints are about to move. Keep hands clear and make sure\n"
           "     the legs have room to swing.\n", first_pose ? "ELEVEN" : "NINE");
    if (wait_for_enter_or_quit("Clear of the robot")) return false;

    float straightened[POSE_TABLE_SIZE];
    for (int i = 0; i < POSE_TABLE_SIZE; ++i) straightened[i] = NAN;
    straighten_legs(straightened, skip);

    printf("\n     -- Hip pass --\n");
    for (int n = 1; n <= 12; ++n) {
        if ((n - 1) / 3 == leg) continue;
        SpotServo s(n);
        if (s.get_type() != ServoType::hip) continue;

        const int steps = pose_steps_for(n, s.get_type(), REST_HIP_ANGLE);
        if (steps > 1) {
            printf("        servo %2d (hip     ): %+.0f deg, ramped from %+.0f in"
                   " %d steps\n", n, (double)REST_HIP_ANGLE,
                   (double)last_angle[n], steps);
        }
        else {
            printf("        servo %2d (hip     ): %+.0f deg\n",
                   n, (double)REST_HIP_ANGLE);
        }
        move_pose_angle(s, n, REST_HIP_ANGLE, steps);
        vTaskDelay(pdMS_TO_TICKS(POSE_STAGGER_MS));
    }

    // Three legs at rest and one wherever the protractor left it is none of
    // the poses the transition rules know how to leave, so the next pose
    // change is sequenced as if the robot could be in the worst of them.
    last_pose = ServoType::free;
    printf("     Resting pose set. Leg %d is free to be read.\n", leg + 1);
    vTaskDelay(pdMS_TO_TICKS(400));
    return true;
}


// EFFECTS: Drives every on-robot servo to its stored center count. Commanded
// by COUNT rather than by angle, so the pose does not depend on the
// angle-to-count math that the servo tests exist to check. Servo 0 is skipped:
// it is the reserved uncalibrated servo, and its center count of 0 is a
// placeholder rather than a position.
//
// For ASSEMBLY, not for testing: nothing is judged and no verdict is recorded.
// Horns have to go on with the servo at center, or the joint's usable travel
// ends up offset from its declared limits. Run this, then mount the horns.
static void center_all_servos() {
    printf("\n--- Center all servos ---\n");
    printf("Sets servos 1-12 to their center PWM counts, including any center\n"
           "set with the jog tool. Assembly helper, not a test: no verdict is\n"
           "recorded. Mount the horns after this runs, not before.\n");
    printf("\nSAFETY: body on a stand, legs clear, bench supply current-limited.\n");
    printf("ALL TWELVE joints are about to move, and any joint sitting far from\n"
           "center will snap to it. Keep hands clear and give the legs room to\n"
           "swing.\n");

    char confirm[8];
    printf("  Center all twelve servos now? (y/N): ");
    read_line(confirm, sizeof confirm);
    if (confirm[0] != 'y' && confirm[0] != 'Y') {
        printf("  Cancelled.\n");
        return;
    }

    ensure_pca_ready();

    for (int n = 1; n <= 12; ++n) {
        SpotServo s(n);
        s.move_to_pwm(effective_center(n));
        printf("      servo %2d (%-8s): center count %d%s\n",
               n, type_name(s.get_type()), effective_center(n),
               center_is_overridden(n) ? " (session override)" : "");
        // Stagger so twelve servos do not all draw stall current at once.
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    printf("     All twelve servos centered.\n");
}


//////////////////// PWM JOG / SET CENTER ////////////////////

// EFFECTS: Returns the pulse width, in microseconds, that `counts` produces.
// Inverse of count_for_us(); both follow the count math in
// Servo::move_to_angle, where a pulse of n counts is (n + 1) ticks long.
static float us_for_count(int counts) {
    return (float)(counts + 1) / 4096.0f * (float)period;
}

// EFFECTS: Returns the count whose pulse comes closest to `us`.
static int count_for_us(float us) {
    return (int)std::lround((double)us / period * 4096.0 - 1.0);
}

// Jogging is unclamped by design -- it commands raw counts, so none of the
// joint limits apply -- but it still stays inside the pulse window a hobby
// servo answers to at all. Past either edge the servo either ignores the pulse
// or grinds against its internal stop, and neither is worth walking into one
// keypress at a time.
static int jog_min_count() { return count_for_us(SERVO_MIN_PULSE_US); }
static int jog_max_count() { return count_for_us(SERVO_MAX_PULSE_US); }

// EFFECTS: Returns `count` brought inside the jog window.
static int clamp_jog_count(int count) {
    if (count < jog_min_count()) return jog_min_count();
    if (count > jog_max_count()) return jog_max_count();
    return count;
}

// EFFECTS: Asks which servo to jog, writing it to *out. Returns false on a
// blank line or 'q', meaning "back to the menu": ask_int() re-asks until it
// gets a number, and a tool entered by accident needs a way out.
static bool ask_jog_servo(int *out) {
    char line[16];
    for (;;) {
        printf("    Servo number [0-12], 0 = uncalibrated bench servo "
               "(blank or q to go back): ");
        read_line(line, sizeof line);
        if (line[0] == '\0' || line[0] == 'q' || line[0] == 'Q') return false;

        char *end = NULL;
        const long v = strtol(line, &end, 10);
        if (end != line && v >= 0 && v <= 12) {
            *out = (int)v;
            return true;
        }
        printf("    ! Not a number in range.\n");
    }
}

// EFFECTS: Rewrites the jog readout in place, on the line the cursor is
// already on, so holding an arrow key down does not scroll the screen.
static void print_jog_status(int num, int count) {
    const int center = effective_center(num);
    printf("\r    servo %2d | count %4d | %6.1f us | center %4d | offset %+5d %-12s ",
           num, count, (double)us_for_count(count), center, count - center,
           center_is_overridden(num) ? "[override]" : "");
    fflush(stdout);
}

// EFFECTS: Steps one servo's pulse width a count at a time from the keyboard,
// and records the count it looks right at as that servo's center.
//
// Counts go out through Servo::move_to_pwm, NOT SpotServo: no calibration, no
// angle math, no limit clamp, so the servo lands exactly where the count says
// and what is on screen is what the PCA9685 was handed. A center set here is
// stored in this session's NVS only -- 't' prints it as source to paste into
// spot_calibration_for(), which is what makes it real.
//
// Calibration and assembly helper, not a test: nothing is judged and no
// verdict is recorded.
static void jog_servo_pwm() {
    printf("\n--- Jog PWM counts / set a servo center ---\n");
    printf("Walks one servo's pulse width one count at a time and lets the\n"
           "count you settle on be recorded as that servo's center. No verdict\n"
           "is recorded.\n");
    printf("\nSAFETY: body on a stand, the joint clear, supply current-limited.\n"
           "Counts are commanded RAW -- no joint limits apply here -- so a long\n"
           "walk in one direction can push a joint into a mechanical stop.\n"
           "Move in small steps and listen for buzzing.\n");
    printf("\nKEYS:\n");
    printf("  right arrow  /  .     +1 count       left arrow  /  ,     -1 count\n");
    printf("  up arrow     /  >    +10 counts      down arrow  /  <    -10 counts\n");
    printf("  =  jump to a count you type          r  back to the stored center\n");
    printf("  c  set THIS servo's center to the count on screen\n");
    printf("  x  drop this servo's override        t  print the paste-ready table\n");
    printf("  n  switch to another servo           q / Esc  back to the menu\n");
    printf("\nIf the arrow keys do nothing, the terminal is eating them: use the\n"
           ". and , keys, which do the same thing.\n");
    printf("The jog window is %.0f-%.0f us (counts %d-%d).\n",
           (double)SERVO_MIN_PULSE_US, (double)SERVO_MAX_PULSE_US,
           jog_min_count(), jog_max_count());

    ensure_pca_ready();

    for (;;) {
        printf("\n");
        int num = 0;
        if (!ask_jog_servo(&num)) return;

        // Servo 0's led_pos would be num - 1 = -1, and move_to_pwm computes
        // LED_ON_L = 6 + 4 * led_pos = 2 -- which is SUBADR1, not a PWM
        // channel. Commanding it would scribble over the PCA9685's addressing
        // registers instead of moving anything, so ask which channel it is
        // actually wired to and use the explicit-led_pos constructor.
        int led_pos = num - 1;
        if (num == 0) {
            printf("\n    Servo 0 has no implied channel (led_pos would be -1, which\n"
                   "    maps onto the PCA9685 config registers rather than a PWM\n"
                   "    output). Specify the channel it is plugged into.\n");
            led_pos = ask_int("PCA9685 channel", 0, 15);
        }
        Servo servo(num, led_pos);

        int count = effective_center(num);
        if (count < jog_min_count() || count > jog_max_count()) {
            // Servo 0's center is a placeholder 0, which is not a pulse any
            // servo answers to. Start from mid-travel rather than throwing the
            // horn at an end stop on the first command.
            count = count_for_us(1500.0f);
            printf("    Servo %d has no usable center (%d counts); starting at\n"
                   "    mid-travel, %d counts.\n", num, effective_center(num), count);
        }

        printf("    Channel %d. About to command count %d (%.0f us) -- the joint\n"
               "    will snap there if it is sitting somewhere else.\n",
               led_pos, count, (double)us_for_count(count));
        wait_for_enter("Clear of the joint");

        servo.move_to_pwm(count);

        // wait_for_enter's read_line stops on the '\r' of a CRLF and leaves the
        // '\n' behind. Left in the buffer it would read as a keypress the
        // moment the jog loop starts.
        drain_input();
        print_jog_status(num, count);

        bool another_servo = false;
        while (!another_servo) {
            const int key = read_key();

            int step = 0;
            switch (key) {
                case KEY_RIGHT: case '.': step =  +1; break;
                case KEY_LEFT:  case ',': step =  -1; break;
                case KEY_UP:    case '>': step = +10; break;
                case KEY_DOWN:  case '<': step = -10; break;
                default: break;
            }

            if (step != 0) {
                const int next = clamp_jog_count(count + step);
                if (next == count) {
                    printf("\n    ! At the %s of the jog window (%.0f-%.0f us);"
                           " not going further.\n",
                           step > 0 ? "top" : "bottom",
                           (double)SERVO_MIN_PULSE_US, (double)SERVO_MAX_PULSE_US);
                }
                else {
                    count = next;
                    servo.move_to_pwm(count);
                }
                print_jog_status(num, count);
                continue;
            }

            switch (key) {
                case '=': {
                    printf("\n");
                    count = ask_int("Count", jog_min_count(), jog_max_count());
                    servo.move_to_pwm(count);
                    drain_input();
                    break;
                }
                case 'r': case 'R': {
                    const int center = effective_center(num);
                    if (center < jog_min_count() || center > jog_max_count()) {
                        printf("\n    ! Servo %d has no usable stored center (%d counts);"
                               " staying put.\n", num, center);
                        break;
                    }
                    count = center;
                    servo.move_to_pwm(count);
                    printf("\n    Returned to the stored center, %d counts.\n", count);
                    break;
                }
                case 'c': case 'C':
                    set_center_override(num, count);
                    printf("\n    Center for servo %d is now %d counts (%.0f us).\n",
                           num, count, (double)us_for_count(count));
                    printf("    Saved in this session's NVS, so it survives a reboot and\n"
                           "    is what 'p' (center all servos) will use -- but\n"
                           "    SpotServo::move_to_angle still works off the center\n"
                           "    compiled into Servo.cpp. Press 't' for the block to paste.\n");
                    break;
                case 'x': case 'X':
                    if (!center_is_overridden(num)) {
                        printf("\n    Servo %d has no override to drop.\n", num);
                        break;
                    }
                    clear_center_override(num);
                    printf("\n    Override dropped; servo %d is back to its compiled\n"
                           "    center of %d counts.\n", num, effective_center(num));
                    break;
                case 't': case 'T':
                    print_center_table();
                    break;
                case 'n': case 'N':
                    another_servo = true;
                    continue;
                case 'q': case 'Q': case KEY_ESC:
                    printf("\n    Leaving servo %d at count %d (%.0f us).\n",
                           num, count, (double)us_for_count(count));
                    return;
                case '\r': case '\n':
                    // Stray line ending from a prompt; not a command.
                    continue;
                default:
                    printf("\n    ! Unknown key. Arrows or . , > < to move,"
                           " c to set center, q to quit.\n");
                    break;
            }

            print_jog_status(num, count);
        }
    }
}


// T7: SpotServo validation #1 -- two separate claims per bound. First, that
// the bound itself is a SAFE pose: nothing in one part of the body touching
// anything in another, no pinched wires. Second, that commanding past the
// bound produces no further motion.
static void test_spotservo_limits() {
    printf("\n--- SpotServo #1: limit saturation ---\n");
    printf("Desired behavior: each joint's declared bound is a safe place to\n"
           "be, and commanding past it moves the joint no further.\n");
    printf("\nAcceptance, both parts must hold at EVERY bound:\n");
    printf("  1. AT the bound: no component in one part of the body is in\n"
           "     contact with a component in another part. PINCHED WIRES\n"
           "     COUNT AS A FAILURE.\n");
    printf("  2. BEYOND the bound: the servo does not move at all.\n");
    printf("\nA bound that causes contact is a bound that is set wrong, even if\n"
           "the clamp itself works perfectly. Both halves are being tested.\n");
    printf("\nSAFETY: body on a stand, legs clear, bench supply current-limited.\n"
           "If the clamp is broken this is exactly what stalls a servo -- keep\n"
           "a hand on the power switch and listen for buzzing.\n");
    printf("Run this BEFORE the protractor test: it is the one that protects\n"
           "the hardware.\n");
    printf("\nWalking all twelve does not have to be finished in one go: q +\n"
           "Enter at ANY prompt below stops the walk where it is and goes\n"
           "straight to the verdict.\n");

    const int choice = ask_int("Servo number (0 = walk all twelve)", 0, 12);
    ensure_pca_ready();

    const int first = (choice == 0) ? 1  : choice;
    const int last  = (choice == 0) ? 12 : choice;

    // However the robot was left -- by an earlier test, by the jog tool, or by
    // hand -- the session cannot know, so the first transition is sequenced as
    // if it were coming from the most awkward pose it could be in.
    reset_pose_memory();

    bool quit       = false;
    int  stopped_at = 0;

    for (int num = first; num <= last && !quit; ++num) {
        SpotServo joint(num);
        const AngleLimit lim = limit_for(joint.get_type());

        printf("\n  == Servo %d (%s), limits [%.0f, %.0f], center count %d ==\n",
               num, type_name(joint.get_type()),
               (double)lim.min, (double)lim.max, joint.get_center_count());

        if (!prepare_safe_pose(num, joint.get_type())) {
            quit = true;
            stopped_at = num;
            break;
        }

        // Four commands per joint, alternating: a bound, then past that same
        // bound. The pair either side of a bound should produce IDENTICAL
        // final counts -- that is the clamp working, and it is visible in the
        // math dump before the servo even moves.
        const float commands[] = {
            lim.min,
            lim.min - LIMIT_OVERSHOOT,
            lim.max,
            lim.max + LIMIT_OVERSHOOT,
        };
        char under[64], over[64];
        snprintf(under, sizeof under, "%.0f degrees BEYOND the minimum bound",
                 (double)LIMIT_OVERSHOOT);
        snprintf(over,  sizeof over,  "%.0f degrees BEYOND the maximum bound",
                 (double)LIMIT_OVERSHOOT);
        const char *labels[] = {
            "the MINIMUM bound", under, "the MAXIMUM bound", over,
        };
        // Even steps land on a bound (inspect for contact); odd steps go past
        // it (confirm nothing moves).
        const bool is_bound[] = { true, false, true, false };

        for (int k = 0; k < 4 && !quit; ++k) {
            char prompt[96];
            snprintf(prompt, sizeof prompt, "Ready: commanding %s", labels[k]);
            if (wait_for_enter_or_quit(prompt)) {
                quit = true;
                stopped_at = num;
                break;
            }

            print_spot_servo_math(joint, commands[k]);
            const SpotMath m = compute_spot_math(joint, commands[k]);

            // Refuse to send a command the hardware cannot act on. Sending it
            // anyway would either be dropped by move_to_pwm's out-of-range
            // guard or emit a pulse the servo ignores -- either way the joint
            // does not move, which reads as "the clamp failed" when the real
            // problem is that this joint's limit is not physically reachable.
            if (m.count_out_of_range || m.pulse_unusable) {
                printf("      -> NOT COMMANDED. Servo %d at %.1f deg maps to count %d\n",
                       num, (double)m.effective, m.final_count);
                printf("         (%.1f us), which is %s.\n", (double)m.final_width,
                       m.count_out_of_range ? "outside the 12-bit counter"
                                            : "outside the servo's usable pulse window");
                printf("         This joint's limit is unreachable -- narrow the\n"
                       "         bound in limit_for() rather than trusting this result.\n");
                continue;
            }

            joint.move_to_angle(commands[k]);
            // Where this joint now is, so the next pose can ramp back off the
            // bound instead of snapping off it.
            last_angle[num] = m.effective;
            printf("      -> servo %d commanded %.1f deg (count %d, %.1f us)\n",
                   num, (double)commands[k], m.final_count, (double)m.final_width);
            vTaskDelay(pdMS_TO_TICKS(800));

            if (is_bound[k]) {
                printf("\n      INSPECT AT THE BOUND. Check every one of these:\n");
                printf("        - no part of this leg touching the body\n");
                printf("        - no part of this leg touching another leg\n");
                printf("        - no linkage or horn fouling its own bracket\n");
                printf("        - NO PINCHED, STRETCHED, OR RUBBING WIRES\n");
                printf("      Contact anywhere is a FAIL for this bound, even if\n");
                printf("      the joint stopped exactly where it was told to.\n");
                if (wait_for_enter_or_quit("Inspected")) {
                    quit = true;
                    stopped_at = num;
                }
            }
            else {
                printf("\n      The clamp should have held this at %.1f deg.\n",
                       (double)m.effective);
                printf("      It must not have moved AT ALL from the previous\n");
                printf("      position. Any motion, even slight, is a FAIL.\n");
                if (wait_for_enter_or_quit("Confirmed whether it moved")) {
                    quit = true;
                    stopped_at = num;
                }
            }
        }
    }

    if (quit) {
        printf("\n    STOPPED EARLY during servo %d.\n", stopped_at);
        if (stopped_at < last) {
            printf("    Servos %d-%d were not tested at all.\n",
                   stopped_at + 1, last);
        }
        printf("    The verdict below covers only what was actually walked --\n"
               "    say so in the note.\n");
    }

    printf("\n    PASS requires BOTH, for every bound tested:\n");
    printf("      1. No contact between components in separate parts of the\n"
           "         body at any bound, and no pinched wires.\n");
    printf("      2. No motion at all when commanded past a bound.\n");
    printf("\n    These fail for different reasons and want different fixes:\n");
    printf("      - contact at a bound  -> the BOUND is wrong; narrow it in\n"
           "                               limit_for() and retest\n");
    printf("      - motion past a bound -> the CLAMP is wrong; the bug is in\n"
           "                               SpotServo::move_to_angle\n");
    printf("    Note which one you saw, and on which joint, in the note below.\n");
    ask_result(7);
}


// Where the knee on the leg under test is parked while that leg's SHOULDER is
// being read: the lower leg folds up out of the way instead of hanging in line
// with the segment the protractor is measuring. The limits test parks a knee
// at SHOULDER_TEST_KNEE_ANGLE for its own reasons; this is the protractor
// test's own number and moves independently of it. Inside the knee limits
// [20, 176], so move_to_angle passes it through unclamped.
constexpr float PROTRACTOR_SHOULDER_KNEE_ANGLE = 40.0f;

// MODIFIES: the knee on the leg that `under_test` belongs to.
// EFFECTS: Folds that knee to PROTRACTOR_SHOULDER_KNEE_ANGLE when the joint
// under test is a shoulder, and does nothing at all for a hip or a knee.
// Called before the protractor goes on, so the leg has finished moving by the
// time there are hands near it, and given the same settle as the straighten
// pass -- this is the last thing to move before a reading is taken.
static void park_knee_for_shoulder(int under_test) {
    SpotServo joint(under_test);
    if (joint.get_type() != ServoType::shoulder) return;

    const int knee = under_test + 1;   // per-leg triples: hip, shoulder, knee

    printf("\n  -- Knee park (the leg under test) --\n");
    printf("     servo %2d (knee    ): %+.0f deg, folded clear while servo %d is\n"
           "     read. It is the one joint on this leg that the resting pose\n"
           "     leaves alone and the reading still needs placed.\n",
           knee, (double)PROTRACTOR_SHOULDER_KNEE_ANGLE, under_test);

    SpotServo kn(knee);
    kn.move_to_angle(PROTRACTOR_SHOULDER_KNEE_ANGLE);
    last_angle[knee] = PROTRACTOR_SHOULDER_KNEE_ANGLE;
    vTaskDelay(pdMS_TO_TICKS(STRAIGHTEN_SETTLE_MS));
}


// What the operator typed at the angle prompt. A blank line and a q part ways
// here: one finishes the joint and moves the walk on, the other stops the walk
// where it is.
enum class AngleReply { angle, done, quit };

// EFFECTS: Prompts for the next angle to command on the joint under test.
// A number is an angle to send; a blank line finishes the joint; q quits the
// walk, the same q the limit test takes at any of its prompts.
static AngleReply ask_protractor_angle(float *out) {
    char line[24];
    printf("    Angle to command (blank finishes this joint, q quits): ");
    read_line(line, sizeof line);
    if (line[0] == '\0')                    return AngleReply::done;
    if (line[0] == 'q' || line[0] == 'Q')   return AngleReply::quit;
    *out = strtof(line, NULL);
    return AngleReply::angle;
}

// EFFECTS: Runs the protractor check on one joint: reports its limits and the
// angle that maps to its center count, then commands whatever angles the
// operator types until they finish the joint or quit the walk. Returns true if
// they quit. Records nothing -- the single verdict for the whole test is taken
// by the caller, since the operator is judging one property (does the joint
// move the way the convention says) and it either holds across the joints they
// looked at or it does not.
static bool protractor_one_joint(int num) {
    SpotServo joint(num);

    const AngleLimit lim = limit_for(joint.get_type());
    printf("\n  == Servo %d (%s), limits [%.0f, %.0f], center count %d ==\n",
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
    for (;;) {
        const AngleReply reply = ask_protractor_angle(&angle);
        if (reply == AngleReply::done) return false;
        if (reply == AngleReply::quit) return true;

        const float effective = angle < lim.min ? lim.min
                              : angle > lim.max ? lim.max
                              : angle;
        if (effective != angle) {
            printf("      ! %.1f is outside [%.0f, %.0f], will clamp to %.1f\n",
                   (double)angle, (double)lim.min, (double)lim.max, (double)effective);
        }
        joint.move_to_angle(angle);
        // The resting pose ramps a hip from wherever the session last left it,
        // and this is the only thing that moves the joint under test, so what
        // the operator typed is what the next pose has to start its ramp from.
        // Recorded as REQUESTED, matching what move_pose_angle stores.
        last_angle[num] = angle;
        printf("      commanded %.1f deg -> check the protractor\n", (double)angle);
        vTaskDelay(pdMS_TO_TICKS(600));
    }
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
    printf("Blank line at the angle prompt finishes a joint; walking all\n"
           "twelve then moves on to the next one.\n");
    printf("Walking all twelve does not have to be finished in one go: q +\n"
           "Enter at ANY prompt below -- an angle prompt included -- stops the\n"
           "walk where it is and goes straight to the verdict.\n");
    printf("\nThis starts by straightening ALL FOUR legs -- the front two, then\n"
           "the back two -- and then holds the three legs you are NOT reading\n"
           "at hip %.0f, shoulder %.0f, knee %.0f, each hanging straight down, so\n"
           "no folded leg sits in the way of the one you are reading. Say no if\n"
           "the robot is not assembled, or if a leg cannot safely be moved.\n",
           (double)REST_HIP_ANGLE, (double)STRAIGHT_SHOULDER_ANGLE,
           (double)STRAIGHT_KNEE_ANGLE);
    printf("A SHOULDER is read with its own leg's knee folded to %.0f deg, so\n"
           "that knee is commanded there too before the reading is taken.\n",
           (double)PROTRACTOR_SHOULDER_KNEE_ANGLE);

    const int choice = ask_int("Servo number (0 = walk all twelve)", 0, 12);

    char hold[8];
    printf("    Hold the other three legs at rest? (Y/n): ");
    read_line(hold, sizeof hold);
    const bool rest_others = (hold[0] != 'n' && hold[0] != 'N');
    if (!rest_others) {
        printf("    Nothing but the joint under test will be commanded.\n");
    }

    ensure_pca_ready();

    const int first = (choice == 0) ? 1  : choice;
    const int last  = (choice == 0) ? 12 : choice;

    // However the robot was left -- by an earlier test, by the jog tool, or by
    // hand -- the session cannot know, so the first pose change is sequenced
    // as if it were coming from the most awkward pose it could be in.
    if (rest_others) reset_pose_memory();

    // Which leg the resting pose is currently holding the other three around.
    // Only a joint on a different leg needs it redone: the three joints of one
    // leg are all read with the same nine joints parked.
    int  posed_leg  = -1;
    bool quit       = false;
    bool quit_early = false;   // quit at an angle prompt, not at the pose one
    int  stopped_at = 0;

    for (int num = first; num <= last; ++num) {
        if (rest_others && (num - 1) / 3 != posed_leg) {
            // posed_leg is still -1 on the first pose of the walk, which is
            // the one that straightens all four legs.
            if (!rest_other_legs(num, posed_leg < 0)) {
                quit       = true;
                stopped_at = num;
                break;
            }
            posed_leg = (num - 1) / 3;
        }

        // A shoulder is read with its own leg's knee folded. Gated on the
        // resting pose for the same reason that is: an operator who said no
        // to legs moving meant this leg too.
        if (rest_others) park_knee_for_shoulder(num);

        // Walking all twelve means the protractor has to be moved and
        // re-taped between joints, so pause rather than commanding a joint
        // nobody is looking at yet. After the resting pose, not before: the
        // legs finish moving before anyone puts a hand near them.
        if (choice == 0) {
            char prompt[64];
            snprintf(prompt, sizeof prompt, "Protractor taped to servo %d", num);
            wait_for_enter(prompt);
        }
        if (protractor_one_joint(num)) {
            quit       = true;
            quit_early = true;
            stopped_at = num;
            break;
        }
    }

    if (quit_early) {
        printf("\n    STOPPED EARLY during servo %d.\n", stopped_at);
        if (stopped_at < last) {
            printf("    Servos %d-%d were not read at all.\n",
                   stopped_at + 1, last);
        }
        printf("    The verdict below covers only what was actually read --\n"
               "    say so in the note.\n");
    }
    else if (quit) {
        printf("\n    STOPPED before servo %d: the resting pose was declined, and\n"
               "    it is what clears the other legs out of the way.\n", stopped_at);
        if (stopped_at > first) {
            printf("    Servos %d-%d were read; %d-%d were not.\n",
                   first, stopped_at - 1, stopped_at, last);
        }
        printf("    The verdict below covers only what was actually read --\n"
               "    say so in the note.\n");
    }

    printf("\n    PASS if the joint moved in the DIRECTION the convention says\n"
           "    it should and landed roughly where the paper says. A joint\n"
           "    that moves the wrong way is a sign error, not a calibration\n"
           "    error -- note which joint, since the left/right pairs use\n"
           "    mirrored formulas.\n");
    if (choice == 0) {
        printf("    One verdict covers all twelve: FAIL it if ANY joint was\n"
               "    wrong, and name that joint in the note.\n");
    }
    ask_result(8);
}


//////////////////// REPEAT-TO-ZERO DIAGNOSTIC ////////////////////

// Repeat-to-zero endurance. Commands one servo to 0 degrees REPEAT_BLOCK_SIZE
// times per block, REPEAT_NUM_BLOCKS blocks, 300 commands in total. The point
// is not the angle -- it is what changes BETWEEN blocks: the operator describes
// the change (reversed connection, different supply, remounted horn) before
// each block, and that label is printed with the block so the console log reads
// as a labeled comparison rather than 300 identical lines.
//
// A diagnostic rather than a test: there is no one behavior to pass or fail,
// only a labeled comparison to read, so no verdict is recorded. Anything a run
// turns up belongs in the note of whichever test it explains.
static void diagnostic_servo_repeat_zero() {
    printf("\n--- Servo repeat-to-zero (%d x %d), diagnostic ---\n",
           REPEAT_NUM_BLOCKS, REPEAT_BLOCK_SIZE);
    printf("Desired behavior: %d commands to 0 degrees leave the servo at the\n"
           "same physical position every time -- no creep, no buzzing, no\n"
           "drift, no growing current draw.\n",
           REPEAT_NUM_BLOCKS * REPEAT_BLOCK_SIZE);
    printf("\nEach block of %d is labeled with whatever you are changing for\n"
           "that block, so the blocks can be compared against each other.\n",
           REPEAT_BLOCK_SIZE);
    printf("Commands are spaced %d ms apart, so a block takes about %d s.\n",
           REPEAT_DELAY_MS, (REPEAT_BLOCK_SIZE * REPEAT_DELAY_MS) / 1000);
    printf("\nNo verdict is recorded: this is a diagnostic, and what it\n"
           "produces is the labeled log below.\n");

    int num = 0, led_pos = 0;
    ask_servo_and_channel(&num, &led_pos);
    Servo servo(num, led_pos);

    const Calibration cal = calibration_for(num);
    printf("    Servo %d: slope %.3f, offset %+.2f\n",
           num, (double)cal.slope, (double)cal.offset);

    // Labels are kept for the recap at the end, so the whole session can be
    // read off one screen instead of scrolling back through the blocks.
    static const char *ORDINAL[] = { "FIRST", "SECOND", "THIRD", "FOURTH", "FIFTH" };
    static_assert(REPEAT_NUM_BLOCKS <= 5, "add ordinals for the extra blocks");
    char labels[REPEAT_NUM_BLOCKS][64];
    char observations[REPEAT_NUM_BLOCKS][64];

    for (int block = 0; block < REPEAT_NUM_BLOCKS; ++block) {
        printf("\n=== %s %d commands (block %d of %d) ===\n",
               ORDINAL[block], REPEAT_BLOCK_SIZE, block + 1, REPEAT_NUM_BLOCKS);
        printf("    What is being changed for these %d commands?\n", REPEAT_BLOCK_SIZE);
        printf("    (e.g. \"reverse connection\", \"stock wiring\", \"5V supply\")\n");
        printf("    Label: ");
        read_line(labels[block], sizeof labels[block]);
        if (labels[block][0] == '\0') {
            snprintf(labels[block], sizeof labels[block], "(no label given)");
        }

        // Sized for the longest possible expansion: 48 characters of
        // fixed text plus a full 63-character label. The build treats
        // format truncation as an error, so this cannot be sized for
        // the labels people are likely to type.
        char prompt[128];
        snprintf(prompt, sizeof prompt,
                 "Make that change, then stand clear -- block %d: %s",
                 block + 1, labels[block]);
        wait_for_enter(prompt);

        printf("\n  >>> %s %d commands to 0 deg -- %s\n",
               ORDINAL[block], REPEAT_BLOCK_SIZE, labels[block]);
        for (int i = 1; i <= REPEAT_BLOCK_SIZE; ++i) {
            servo.move_to_angle(0.0f);
            if (i % REPEAT_PROGRESS_EVERY == 0) {
                printf("      [%s: %s] %3d / %d commanded to 0 deg\n",
                       ORDINAL[block], labels[block], i, REPEAT_BLOCK_SIZE);
            }
            vTaskDelay(pdMS_TO_TICKS(REPEAT_DELAY_MS));
        }
        printf("  <<< block %d done (%s): %d commands to 0 deg sent\n",
               block + 1, labels[block], REPEAT_BLOCK_SIZE);

        printf("\n    What did the servo do during this block? (blank to skip)\n");
        printf("    Observation: ");
        read_line(observations[block], sizeof observations[block]);
    }

    printf("\n    Recap -- %d commands to 0 deg in total:\n",
           REPEAT_NUM_BLOCKS * REPEAT_BLOCK_SIZE);
    for (int block = 0; block < REPEAT_NUM_BLOCKS; ++block) {
        printf("      block %d (%s %d): %-28s -> %s\n",
               block + 1, ORDINAL[block], REPEAT_BLOCK_SIZE, labels[block],
               observations[block][0] ? observations[block] : "(no observation)");
    }

    printf("\n    What to look for: the servo holding the same position\n"
           "    through every block, with blocks differing only where you\n"
           "    expected them to. Creep that accumulates over a block points at\n"
           "    the servo losing or mis-latching commands; a difference that\n"
           "    appears only in one block belongs to whatever you changed for\n"
           "    it. If a run explains a test result, record it in that test's\n"
           "    note.\n");
}


//////////////////// MENU ////////////////////

// EFFECTS: Records verdicts for tests that were performed but whose stored
// results are gone. Nothing is run and nothing moves; the verdict comes from
// the operator's memory of the earlier run, which is exactly as trustworthy as
// that memory, so the note matters more here than anywhere else.
static void mark_results_by_hand() {
    printf("\n--- Mark results by hand ---\n");
    printf("Records a verdict WITHOUT running anything. This is for a test that\n"
           "WAS performed and whose record was lost -- NVS wiped, or a firmware\n"
           "layout change that discarded the stored blob.\n");
    printf("It is NOT a way to pass a test that was never run: a marked PASS\n"
           "reads identically to a real one in the summary, so say in the note\n"
           "that it was marked by hand and roughly when it was actually run.\n");

    for (;;) {
        printf("\n");
        for (int i = 0; i < NUM_TESTS; ++i) {
            printf("  %d) %-32s [%s]\n", i, records[i].name,
                   result_str(records[i].result));
        }

        const int which = ask_int("Which test to mark", 0, NUM_TESTS - 1);
        ask_result(which);          // same p/f/s prompt and note, and persists

        char again[8];
        printf("    Mark another? (y/N): ");
        read_line(again, sizeof again);
        if (again[0] != 'y' && again[0] != 'Y') break;
    }
}

static void print_menu() {
    printf("\n=====================================================\n");
    printf("  TPUP HARDWARE TEST SESSION\n");
    printf("=====================================================\n");
    for (int i = 0; i < NUM_TESTS; ++i) {
        printf("  %d) %-32s [%s]\n", i, records[i].name, result_str(records[i].result));
    }
    printf("  i) I2C bus scan (diagnostic)\n");
    printf("  p) center all servos (assembly helper, no verdict)\n");
    printf("  j) jog PWM counts / set a servo center (no verdict)\n");
    printf("  z) servo repeat-to-zero endurance (%d x %d, no verdict)\n",
           REPEAT_NUM_BLOCKS, REPEAT_BLOCK_SIZE);
    printf("  s) summary (Markdown, ready to paste)\n");
    printf("  m) mark results by hand (no test run -- for lost records)\n");
    printf("  r) reset ONE test result\n");
    printf("  c) clear ALL results\n");
    printf("  q) quit\n");
    printf("-----------------------------------------------------\n");
    printf("  Results persist across reboots. Order matters twice:\n");
    printf("    - run 4 (six-face) BEFORE 3 (calibrate)\n");
    printf("    - run 7 (limits) BEFORE 8 (protractor)\n");
    printf("Select: ");
}

// EFFECTS: Prints the results as a ready-to-paste Markdown section. The
// firmware version comes from the app descriptor, which ESP-IDF fills from
// `git describe` at build time -- so the commit is stamped in automatically
// and there is nothing to look up by hand.
static void print_summary() {
    const esp_app_desc_t *app = esp_app_get_description();

    int passed = 0, failed = 0, skipped = 0, not_run = 0;
    for (int i = 0; i < NUM_TESTS; ++i) {
        switch (records[i].result) {
            case Result::pass:    ++passed;  break;
            case Result::fail:    ++failed;  break;
            case Result::skip:    ++skipped; break;
            case Result::not_run: ++not_run; break;
        }
    }

    printf("\n\n----- copy from here -----\n\n");
    printf("## Test run - build %s (%s %s)\n\n", app->version, app->date, app->time);
    printf("Firmware: `%s` | ESP-IDF %s\n\n", app->version, app->idf_ver);
    printf("| # | Test | Result | Note |\n");
    printf("|---|------|--------|------|\n");
    for (int i = 0; i < NUM_TESTS; ++i) {
        printf("| %d | %s | %s | %s |\n", i, records[i].name,
               result_str(records[i].result),
               records[i].note[0] ? records[i].note : "");
    }
    printf("\n**%d passed, %d failed, %d skipped, %d not run**\n",
           passed, failed, skipped, not_run);
    printf("\n----- copy to here -----\n\n");
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
    printf("  - Power switch within reach\n\n");

    load_results();
    report_center_overrides();

    for (;;) {
        print_menu();

        char line[16];
        read_line(line, sizeof line);

        if (line[0] == 'q') break;
        if (line[0] == 's') { print_summary(); continue; }
        if (line[0] == 'i') { i2c_scan(); continue; }
        if (line[0] == 'p') { center_all_servos(); continue; }
        if (line[0] == 'j') { jog_servo_pwm(); continue; }
        if (line[0] == 'z') { diagnostic_servo_repeat_zero(); continue; }
        if (line[0] == 'm') { mark_results_by_hand(); continue; }
        if (line[0] == 'r') {
            const int which = ask_int("Which test to reset", 0, NUM_TESTS - 1);
            clear_one(which);
            continue;
        }
        if (line[0] == 'c') {
            char confirm[8];
            printf("  Clear ALL %d results? This cannot be undone. (y/N): ", NUM_TESTS);
            read_line(confirm, sizeof confirm);
            if (confirm[0] == 'y' || confirm[0] == 'Y') clear_all();
            else printf("  Cancelled.\n");
            continue;
        }

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
