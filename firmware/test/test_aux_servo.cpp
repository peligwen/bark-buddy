// test_aux_servo.cpp — Unit tests for aux servo ports (firmware indices 8-10).
// Built with -DAUX_SERVOS_ENABLED=1 to exercise the guarded code path.

// Override AUX_SERVOS_ENABLED from config.h via compiler flag (-DAUX_SERVOS_ENABLED=1)
#include "mock_arduino.h"
#include "mock_preferences.h"
#include "../include/config.h"
#include "../include/servos.h"
#include "../src/offsets.cpp"
#include "../src/pin_registry.cpp"
#include "../src/servos.cpp"

#include <cstdio>
#include <cstdlib>

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

// Full reset: clear LEDC log, clock, engage state, battery cutoff.
static void reset_state() {
    servo_log_reset();
    mock_reset_clock();
    s_battery_cutoff = false;
    servos_detach_all();
    s_battery_cutoff = false;
}

// Engage and complete the ramp so aux servos are live.
static void bring_up() {
    servos_engage_start();
    // Run ramp to completion.
    uint32_t step_ms = 20;
    uint32_t elapsed = 0;
    while (elapsed <= SOFTSTART_DURATION_MS + step_ms) {
        mock_advance_ms(step_ms);
        elapsed += step_ms;
        RampResult r = servos_ramp_tick((uint32_t)millis());
        if (r != RampResult::NONE) break;
    }
}

// ------------------------------------------------------------------ //
// Test 1: read_write_roundtrip
// ------------------------------------------------------------------ //
static void test_read_write_roundtrip() {
    printf("\nTest: aux_read_write_roundtrip\n");
    reset_state();
    bring_up();

    aux_servo_write_us(0, 1500);
    check(aux_servo_read_us(0) == 1500, "write 1500 -> read_us == 1500");

    aux_servo_write_us(1, 1200);
    check(aux_servo_read_us(1) == 1200, "write 1200 -> read_us == 1200");

    aux_servo_write_us(2, 1800);
    check(aux_servo_read_us(2) == 1800, "write 1800 -> read_us == 1800");
}

// ------------------------------------------------------------------ //
// Test 2: clamping high
// ------------------------------------------------------------------ //
static void test_clamp_high() {
    printf("\nTest: aux_clamp_high\n");
    reset_state();
    bring_up();

    aux_servo_write_us(0, 9999);
    check(aux_servo_read_us(0) == SERVO_MAX_US,
          "write 9999 -> read_us == SERVO_MAX_US");
}

// ------------------------------------------------------------------ //
// Test 3: clamping low
// ------------------------------------------------------------------ //
static void test_clamp_low() {
    printf("\nTest: aux_clamp_low\n");
    reset_state();
    bring_up();

    aux_servo_write_us(0, 100);
    check(aux_servo_read_us(0) == SERVO_MIN_US,
          "write 100 -> read_us == SERVO_MIN_US");
}

// ------------------------------------------------------------------ //
// Test 4: write-while-disengaged is a no-op
// ------------------------------------------------------------------ //
static void test_write_while_disengaged_noop() {
    printf("\nTest: aux_write_while_disengaged_noop\n");
    reset_state();

    // Confirm fully disengaged before writing.
    check(!servos_engaged(), "servos_engaged() false (precondition)");

    aux_servo_write_us(0, 1800);
    check(aux_servo_read_us(0) == 0,
          "aux_servo_read_us(0) still 0 after write-while-disengaged");
    check(_servo_duty[AUX_SERVO_PINS[0]] == 0,
          "LEDC duty unchanged after write-while-disengaged");
}

// ------------------------------------------------------------------ //
// Test 5: init sets neutral (1500us) on all aux channels
// ------------------------------------------------------------------ //
static void test_init_sets_neutral() {
    printf("\nTest: aux_init_sets_neutral\n");
    reset_state();
    bring_up();

    // aux_servo_init() is called inside servos_engage_start().
    // The aux servos that have NOT been written should still sit at 1500us.
    // Verify by checking LEDC duty for all three pins after a clean engage.
    uint32_t expected_duty = (uint32_t)((uint64_t)1500 * LEDC_MAX_DUTY / (1000000UL / SERVO_FREQ_HZ));

    // After bring_up(), no aux writes have been done — pins should be at 1500 duty.
    // (The ramp writes main servos; aux servos are set once in aux_servo_init().)
    bool all_neutral = true;
    for (int i = 0; i < AUX_SERVO_COUNT; i++) {
        if (_servo_duty[AUX_SERVO_PINS[i]] != expected_duty) {
            printf("  pin %d duty=%u expected=%u\n",
                   AUX_SERVO_PINS[i], _servo_duty[AUX_SERVO_PINS[i]], expected_duty);
            all_neutral = false;
        }
    }
    check(all_neutral, "all aux servo pins set to 1500us neutral on init");
}

// ------------------------------------------------------------------ //
// Test 6: out-of-range idx is a no-op
// ------------------------------------------------------------------ //
static void test_out_of_range_idx_noop() {
    printf("\nTest: aux_out_of_range_idx_noop\n");
    reset_state();
    bring_up();

    uint16_t before = aux_servo_read_us(0);
    aux_servo_write_us(AUX_SERVO_COUNT, 1500);  // idx == AUX_SERVO_COUNT is out of range
    check(aux_servo_read_us(0) == before, "out-of-range idx does not affect idx 0");
    check(aux_servo_read_us(AUX_SERVO_COUNT) == 0,
          "read with out-of-range idx returns 0");
}

// ------------------------------------------------------------------ //
// Main
// ------------------------------------------------------------------ //
int main() {
    printf("=== test_aux_servo ===\n");

    test_read_write_roundtrip();
    test_clamp_high();
    test_clamp_low();
    test_write_while_disengaged_noop();
    test_init_sets_neutral();
    test_out_of_range_idx_noop();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
