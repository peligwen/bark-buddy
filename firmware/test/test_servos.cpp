// test_servos.cpp — Host-side tests for LEDC-based servo control.
// Validates: duty values, soft-start ramp, write, detach, shutdown ramp.

#include "mock_arduino.h"
#include "mock_preferences.h"
#include "../include/config.h"
#include "../include/servos.h"
#include "../src/servos.cpp"

#include <cstdio>
#include <cstdlib>
#include <cmath>

static int g_pass = 0;
static int g_fail = 0;

static void check(bool cond, const char* label) {
    if (cond) {
        printf("  PASS  %s\n", label);
        g_pass++;
    } else {
        printf("  FAIL  %s\n", label);
        g_fail++;
    }
}

// Expected LEDC duty for a given pulse width (mirrors us_to_duty logic)
static uint32_t expected_duty(uint16_t us) {
    return (uint32_t)((uint64_t)us * LEDC_MAX_DUTY / (1000000UL / SERVO_FREQ_HZ));
}

// ------------------------------------------------------------------ //
// Test 1: servos_init() ends with STANDING_POSE duties on all pins
// ------------------------------------------------------------------ //
static void test_init_reaches_standing() {
    printf("\nTest: init reaches standing pose\n");
    servo_log_reset();
    mock_reset_clock();

    servos_init();

    bool all_correct = true;
    for (int i = 0; i < 8; i++) {
        uint32_t expected = expected_duty(STANDING_POSE[i]);
        uint32_t actual   = _servo_duty[SERVO_PINS[i]];
        if (actual != expected) {
            printf("  servo %d: expected duty %u got %u\n", i, expected, actual);
            all_correct = false;
        }
    }
    check(all_correct, "all servos at standing duty after init");
    check(servos_active(), "servos_active() true after init");
}

// ------------------------------------------------------------------ //
// Test 2: Soft-start ramp is monotonic (for each servo from rest to standing)
// ------------------------------------------------------------------ //
static void test_softstart_monotonic() {
    printf("\nTest: soft-start ramp is monotonic\n");
    servo_log_reset();
    mock_reset_clock();
    servos_detach_all();

    servos_init();

    // Build per-pin duty sequences from the log
    // Focus on servo 0 (FL_hip, pin 25) — representative
    int pin = SERVO_PINS[0];
    uint32_t prev = 0;
    bool monotonic = true;
    bool saw_start = false;

    for (int i = 0; i < servo_log_count(); i++) {
        const ServoCapture& c = servo_log_entries()[i];
        if (c.pin != (uint8_t)pin) continue;
        if (!saw_start) {
            saw_start = true;
            prev = c.duty;
            continue;
        }
        // Allow equal (same step written twice at start/end of ramp)
        // Direction: REST_POSE[0]=1800 → STANDING_POSE[0]=2096, so duty should be non-decreasing
        bool going_up = STANDING_POSE[0] >= REST_POSE[0];
        if (going_up && c.duty < prev) { monotonic = false; break; }
        if (!going_up && c.duty > prev) { monotonic = false; break; }
        prev = c.duty;
    }
    check(saw_start, "servo log has entries for pin 25");
    check(monotonic, "FL_hip soft-start ramp is monotonic");
}

// ------------------------------------------------------------------ //
// Test 3: servo_write_us() produces correct duty on correct pin
// ------------------------------------------------------------------ //
static void test_write_duty() {
    printf("\nTest: servo_write_us produces correct duty\n");
    servo_log_reset();
    mock_reset_clock();
    servos_init();

    uint16_t test_us = 1800;
    servo_write_us(0, test_us);
    uint32_t expected = expected_duty(test_us);
    uint32_t actual   = _servo_duty[SERVO_PINS[0]];
    check(actual == expected, "servo 0 write 1800us → correct duty");

    // Verify read-back
    check(servo_read_us(0) == test_us, "servo_read_us returns written value");

    // Verify clamp: write above max
    servo_write_us(1, 9999);
    check(servo_read_us(1) == SERVO_MAX_US, "write above max is clamped to SERVO_MAX_US");

    // Verify clamp: write below min
    servo_write_us(2, 100);
    check(servo_read_us(2) == SERVO_MIN_US, "write below min is clamped to SERVO_MIN_US");
}

// ------------------------------------------------------------------ //
// Test 4: servos_detach_all() zeros duties and marks inactive
// ------------------------------------------------------------------ //
static void test_detach() {
    printf("\nTest: servos_detach_all zeros duties\n");
    servo_log_reset();
    mock_reset_clock();
    servos_init();

    servos_detach_all();

    check(!servos_active(), "servos_active() false after detach");

    bool all_zero = true;
    for (int i = 0; i < 8; i++) {
        if (_servo_duty[SERVO_PINS[i]] != 0) { all_zero = false; break; }
    }
    check(all_zero, "all duties zero after detach");
}

// ------------------------------------------------------------------ //
// Test 5: servos_shutdown_to_lying_down() ramps and detaches
// ------------------------------------------------------------------ //
static void test_shutdown_ramp() {
    printf("\nTest: shutdown ramp ends at lying-down and detaches\n");
    servo_log_reset();
    mock_reset_clock();
    servos_init();

    // Move servo 0 away from center first
    servo_write_us(0, 2096);

    bool result = servos_shutdown_to_lying_down();
    check(result, "shutdown returns true");
    check(!servos_active(), "servos inactive after shutdown");

    // All duties should be 0 (detached)
    bool all_zero = true;
    for (int i = 0; i < 8; i++) {
        if (_servo_duty[SERVO_PINS[i]] != 0) { all_zero = false; break; }
    }
    check(all_zero, "all duties zero after shutdown");

    // Second call returns false (already detached)
    check(!servos_shutdown_to_lying_down(), "second shutdown call returns false");
}

// ------------------------------------------------------------------ //
// Test 6: us_to_duty round-trip — well-known values
// ------------------------------------------------------------------ //
static void test_duty_values() {
    printf("\nTest: duty calculation for known pulse widths\n");
    // period = 20000us, resolution = 14 bits (16383 max)
    // duty = us * 16383 / 20000
    uint32_t d_center = expected_duty(1500);
    uint32_t d_min    = expected_duty(500);
    uint32_t d_max    = expected_duty(2500);

    check(d_center > d_min && d_center < d_max, "center duty is between min and max");
    // 500us / 20000us * 16383 = 409.575 → 409
    check(d_min == 409, "500us duty == 409");
    // 1500us / 20000us * 16383 = 1228.725 → 1228
    check(d_center == 1228, "1500us duty == 1228");
    // 2500us / 20000us * 16383 = 2047.875 → 2047
    check(d_max == 2047, "2500us duty == 2047");
}

// ------------------------------------------------------------------ //
// Test 8: servos_attach_at() sets all servos to the given pose
// ------------------------------------------------------------------ //
static void test_attach_at() {
    printf("\nTest: servos_attach_at sets pose without ramping\n");
    servo_log_reset();
    mock_reset_clock();

    // Start from detached state
    servos_detach_all();

    bool result = servos_attach_at(REST_POSE);
    check(result, "servos_attach_at returns true when detached");
    check(servos_active(), "servos_active() true after attach_at");

    // Verify all servos are at REST_POSE
    bool all_correct = true;
    for (int i = 0; i < 8; i++) {
        uint32_t expected = expected_duty(REST_POSE[i]);
        uint32_t actual   = _servo_duty[SERVO_PINS[i]];
        if (actual != expected) {
            printf("  servo %d: expected duty %u (REST_POSE %u) got %u\n",
                   i, expected, REST_POSE[i], actual);
            all_correct = false;
        }
    }
    check(all_correct, "all servos at REST_POSE duty after attach_at");
    check(servo_read_us(0) == REST_POSE[0], "servo_read_us returns REST_POSE value");

    // Second call while attached returns false
    bool second = servos_attach_at(REST_POSE);
    check(!second, "servos_attach_at returns false when already attached");
    check(servos_active(), "servos_active() still true after second attach_at");
}

// ------------------------------------------------------------------ //
// Test 9: servos_shutdown_to_rest() ramps to REST_POSE and detaches
// ------------------------------------------------------------------ //
static void test_shutdown_to_rest() {
    printf("\nTest: servos_shutdown_to_rest ramps to REST_POSE and detaches\n");
    servo_log_reset();
    mock_reset_clock();

    // Start from standing
    servos_init();

    // Move servo 0 away from rest
    servo_write_us(0, 2000);

    bool result = servos_shutdown_to_rest();
    check(result, "servos_shutdown_to_rest returns true");
    check(!servos_active(), "servos inactive after shutdown_to_rest");

    // All duties should be 0 (detached)
    bool all_zero = true;
    for (int i = 0; i < 8; i++) {
        if (_servo_duty[SERVO_PINS[i]] != 0) { all_zero = false; break; }
    }
    check(all_zero, "all duties zero after shutdown_to_rest");

    // Second call returns false (already detached)
    check(!servos_shutdown_to_rest(), "second shutdown_to_rest call returns false");
}

// ------------------------------------------------------------------ //
// Test 10: servos_ramp_to with steps=0 must not crash and reach target
// ------------------------------------------------------------------ //
static void test_ramp_to_zero_steps() {
    printf("\nTest: servos_ramp_to with steps=0 does not crash and reaches target\n");
    servo_log_reset();
    mock_reset_clock();

    servos_detach_all();
    servos_attach_at(REST_POSE);
    servos_ramp_to(STANDING_POSE, 100, 0);  // steps=0 must not crash
    for (int i = 0; i < 8; i++) {
        check(servo_read_us(i) == STANDING_POSE[i], "zero-steps ramp reaches target");
    }
}

// ------------------------------------------------------------------ //
// Main
// ------------------------------------------------------------------ //
int main() {
    printf("=== servo tests ===\n");

    test_duty_values();
    test_init_reaches_standing();
    test_softstart_monotonic();
    test_write_duty();
    test_detach();
    test_shutdown_ramp();
    test_attach_at();
    test_shutdown_to_rest();
    test_ramp_to_zero_steps();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
