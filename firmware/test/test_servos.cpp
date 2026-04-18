// test_servos.cpp — Host-side tests for LEDC-based servo control.
// Validates: duty values, engage ramp, write, detach, disengage ramp.

#include "mock_arduino.h"
#include "mock_preferences.h"
#include "../include/config.h"
#include "../include/servos.h"
#include "../src/offsets.cpp"
#include "../src/pin_registry.cpp"
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

// Run the non-blocking engage ramp to completion in a simulated clock.
// Steps the mock clock forward and calls servos_ramp_tick() each step.
// Returns the RampResult from the completing tick.
static RampResult run_ramp_to_completion() {
    RampResult result = RampResult::NONE;
    while (servos_is_ramping()) {
        mock_advance_ms(20);
        result = servos_ramp_tick((uint32_t)millis());
    }
    return result;
}

// ------------------------------------------------------------------ //
// Test: servos_engage_start() attaches at REST_POSE and kicks off ramp
// ------------------------------------------------------------------ //
static void test_engage_attaches_at_rest() {
    printf("\nTest: servos_engage_start attaches at REST_POSE\n");
    servo_log_reset();
    mock_reset_clock();
    servos_detach_all();

    bool ok = servos_engage_start();
    check(ok, "servos_engage_start returns true from detached state");
    check(servos_engaged(), "servos_engaged() true after engage_start");
    check(servos_is_ramping(), "servos_is_ramping() true after engage_start");

    // All servos should be at REST_POSE on the initial write
    bool all_correct = true;
    for (int i = 0; i < 8; i++) {
        uint32_t expected = expected_duty(REST_POSE[i]);
        uint32_t actual   = _servo_duty[SERVO_PINS[i]];
        if (actual != expected) {
            printf("  servo %d: expected %u got %u\n", i, expected, actual);
            all_correct = false;
        }
    }
    check(all_correct, "all servos at REST_POSE duty after engage_start");

    // Second call while ramping returns false
    check(!servos_engage_start(), "second engage_start returns false while ramping");
}

// ------------------------------------------------------------------ //
// Test: engage ramp completes at STANDING_POSE
// ------------------------------------------------------------------ //
static void test_engage_ramp_reaches_standing() {
    printf("\nTest: engage ramp reaches STANDING_POSE\n");
    servo_log_reset();
    mock_reset_clock();
    servos_detach_all();

    servos_engage_start();
    RampResult engage_result = run_ramp_to_completion();

    check(engage_result == RampResult::ENGAGE_COMPLETE, "ramp completion signals ENGAGE_COMPLETE");
    check(!servos_is_ramping(), "ramp no longer in progress");
    check(servos_engaged(), "servos still engaged after ramp");

    bool all_correct = true;
    for (int i = 0; i < 8; i++) {
        if (servo_read_us(i) != STANDING_POSE[i]) {
            printf("  servo %d: expected %u got %u\n",
                   i, STANDING_POSE[i], servo_read_us(i));
            all_correct = false;
        }
    }
    check(all_correct, "all servos at STANDING_POSE after engage ramp");
}

// ------------------------------------------------------------------ //
// Test: Engage ramp is monotonic for each servo (rest → standing)
// ------------------------------------------------------------------ //
static void test_engage_ramp_monotonic() {
    printf("\nTest: engage ramp is monotonic\n");
    servo_log_reset();
    mock_reset_clock();
    servos_detach_all();

    servos_engage_start();
    run_ramp_to_completion();

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
        // REST_POSE[0]=1800 → STANDING_POSE[0]=2096, so duty should be non-decreasing
        bool going_up = STANDING_POSE[0] >= REST_POSE[0];
        if (going_up && c.duty < prev) { monotonic = false; break; }
        if (!going_up && c.duty > prev) { monotonic = false; break; }
        prev = c.duty;
    }
    check(saw_start, "servo log has entries for pin 25");
    check(monotonic, "FL_hip engage ramp is monotonic");
}

// ------------------------------------------------------------------ //
// Test: servo_write_us() produces correct duty on correct pin (engaged)
// ------------------------------------------------------------------ //
static void test_write_duty() {
    printf("\nTest: servo_write_us produces correct duty\n");
    servo_log_reset();
    mock_reset_clock();
    servos_detach_all();
    servos_engage_start();
    run_ramp_to_completion();

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
// Test: servo_write_us is a no-op when not engaged
// ------------------------------------------------------------------ //
static void test_write_when_disengaged_noop() {
    printf("\nTest: servo_write_us no-op when not engaged\n");
    servo_log_reset();
    mock_reset_clock();
    servos_detach_all();

    servo_write_us(0, 1800);
    check(servo_read_us(0) == 0, "servo_read_us still 0 after write-while-disengaged");
    check(_servo_duty[SERVO_PINS[0]] == 0, "duty unchanged after write-while-disengaged");
}

// ------------------------------------------------------------------ //
// Test: servos_detach_all() zeros duties and marks disengaged
// ------------------------------------------------------------------ //
static void test_detach() {
    printf("\nTest: servos_detach_all zeros duties\n");
    servo_log_reset();
    mock_reset_clock();
    servos_detach_all();
    servos_engage_start();
    run_ramp_to_completion();

    servos_detach_all();

    check(!servos_engaged(), "servos_engaged() false after detach");
    check(!servos_is_ramping(), "servos_is_ramping() false after detach");

    bool all_zero = true;
    for (int i = 0; i < 8; i++) {
        if (_servo_duty[SERVO_PINS[i]] != 0) { all_zero = false; break; }
    }
    check(all_zero, "all duties zero after detach");
}

// ------------------------------------------------------------------ //
// Test: disengage ramp reaches REST_POSE and detaches
// ------------------------------------------------------------------ //
static void test_disengage_ramp() {
    printf("\nTest: disengage ramp reaches REST_POSE then detaches\n");
    servo_log_reset();
    mock_reset_clock();
    servos_detach_all();

    servos_engage_start();
    run_ramp_to_completion();

    // Now disengage
    servos_disengage_start();
    check(servos_is_ramping(), "ramping true after disengage_start");

    RampResult result = run_ramp_to_completion();
    check(result == RampResult::DISENGAGE_COMPLETE, "ramp completion signals DISENGAGE_COMPLETE");

    check(!servos_engaged(), "servos disengaged after ramp");
    check(!servos_is_ramping(), "ramping false after completion");

    // All duties should be 0 (detached)
    bool all_zero = true;
    for (int i = 0; i < 8; i++) {
        if (_servo_duty[SERVO_PINS[i]] != 0) { all_zero = false; break; }
    }
    check(all_zero, "all duties zero after disengage");
}

// ------------------------------------------------------------------ //
// Test: battery cutoff latches and prevents future engage
// ------------------------------------------------------------------ //
static void test_battery_cutoff() {
    printf("\nTest: battery cutoff prevents engage\n");
    servo_log_reset();
    mock_reset_clock();
    servos_detach_all();
    servos_engage_start();
    run_ramp_to_completion();

    check(!servos_battery_cutoff(), "battery cutoff initially clear");
    servos_set_battery_cutoff();
    check(servos_battery_cutoff(), "battery cutoff latched");
    check(!servos_engaged(), "servos detached by battery cutoff");

    // Cannot engage again
    bool ok = servos_engage_start();
    check(!ok, "servos_engage_start returns false after battery cutoff");
    check(!servos_engaged(), "servos stay disengaged after cutoff");
}

// ------------------------------------------------------------------ //
// Test: us_to_duty round-trip — well-known values
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
// Test: disengage while mid-engage-ramp reverses toward rest
// ------------------------------------------------------------------ //
static void test_disengage_mid_engage() {
    printf("\nTest: disengage mid-engage-ramp reverses toward rest\n");
    servo_log_reset();
    mock_reset_clock();
    servos_detach_all();
    // Reset battery-cutoff latch from any prior test.
    s_battery_cutoff = false;

    servos_engage_start();
    // Step a few ticks partway through ramp
    for (int i = 0; i < 10; i++) {
        mock_advance_ms(20);
        servos_ramp_tick((uint32_t)millis());
    }
    check(servos_is_ramping(), "still ramping after partial time");

    // Capture current position before disengage
    uint16_t mid_pos_0 = servo_read_us(0);
    check(mid_pos_0 != REST_POSE[0] && mid_pos_0 != STANDING_POSE[0],
          "servo 0 in-between rest and standing mid-ramp");

    // Kick off disengage — should use current position as ramp start
    servos_disengage_start();
    check(servos_is_ramping(), "still ramping after disengage");

    RampResult result = run_ramp_to_completion();
    check(result == RampResult::DISENGAGE_COMPLETE, "ramp direction flipped to disengage");

    check(!servos_engaged(), "servos disengaged after ramp");
}

// ------------------------------------------------------------------ //
// Main
// ------------------------------------------------------------------ //
int main() {
    printf("=== servo tests ===\n");

    test_duty_values();
    test_engage_attaches_at_rest();
    test_engage_ramp_reaches_standing();
    test_engage_ramp_monotonic();
    test_write_duty();
    test_write_when_disengaged_noop();
    test_detach();
    test_disengage_ramp();
    test_battery_cutoff();
    test_disengage_mid_engage();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
