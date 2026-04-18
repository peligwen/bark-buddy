// test_engage.cpp — Unit tests for the engage/disengage FSM in servos.cpp.
// Focused on state machine correctness: ramp lifecycle, guard conditions,
// battery cutoff latch, heartbeat-style detach, and write gating.

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

// Reset all state before each test.
static void reset_state() {
    servo_log_reset();
    mock_reset_clock();
    // Clear battery-cutoff latch (internal state — accessed directly as in test_servos.cpp).
    s_battery_cutoff = false;
    servos_detach_all();
    // detach_all only runs when engaged/ramping; force-clear cutoff then detach again
    // in case the above path was a no-op.
    s_battery_cutoff = false;
}

// Advance clock step-by-step and call servos_ramp_tick() until it returns
// non-NONE or the duration has elapsed. Returns the completing RampResult.
static RampResult run_ramp(uint32_t duration_ms) {
    RampResult result = RampResult::NONE;
    uint32_t step_ms = 20;
    uint32_t elapsed = 0;
    while (elapsed <= duration_ms + step_ms) {
        mock_advance_ms(step_ms);
        elapsed += step_ms;
        result = servos_ramp_tick((uint32_t)millis());
        if (result != RampResult::NONE) break;
    }
    return result;
}

// ------------------------------------------------------------------ //
// Test 1: engage_basic
// ------------------------------------------------------------------ //
static void test_engage_basic() {
    printf("\nTest: engage_basic\n");
    reset_state();

    bool ok = servos_engage_start();
    check(ok, "engage_start returns true from fresh state");
    check(servos_engaged(), "servos_engaged() true immediately after engage_start");
    check(servos_is_ramping(), "servos_is_ramping() true immediately after engage_start");

    RampResult result = run_ramp(SOFTSTART_DURATION_MS + 1);
    check(result == RampResult::ENGAGE_COMPLETE, "ramp returns ENGAGE_COMPLETE");
    check(!servos_is_ramping(), "servos_is_ramping() false after ramp completes");
    check(servos_engaged(), "servos_engaged() still true after ramp completes");
}

// ------------------------------------------------------------------ //
// Test 2: engage_ramp_monotonic
// ------------------------------------------------------------------ //
static void test_engage_ramp_monotonic() {
    printf("\nTest: engage_ramp_monotonic\n");
    reset_state();

    servos_engage_start();

    // Walk through ramp tick by tick, capturing servo 0 (FL_hip) duty.
    int pin = SERVO_PINS[0];
    uint32_t prev_duty = _servo_duty[pin];
    bool monotonic = true;

    while (servos_is_ramping()) {
        mock_advance_ms(20);
        servos_ramp_tick((uint32_t)millis());
        uint32_t cur = _servo_duty[pin];
        // REST_POSE[0]=1800 < STANDING_POSE[0]=2096 → duty should be non-decreasing
        if (STANDING_POSE[0] >= REST_POSE[0] && cur < prev_duty) {
            monotonic = false;
            printf("  duty dropped: %u -> %u\n", prev_duty, cur);
        }
        prev_duty = cur;
    }

    check(monotonic, "FL_hip (servo 0) duty is monotonically non-decreasing during engage ramp");
}

// ------------------------------------------------------------------ //
// Test 3: disengage_basic
// ------------------------------------------------------------------ //
static void test_disengage_basic() {
    printf("\nTest: disengage_basic\n");
    reset_state();

    servos_engage_start();
    run_ramp(SOFTSTART_DURATION_MS + 1);

    servos_disengage_start();
    check(servos_is_ramping(), "is_ramping() true after disengage_start");

    RampResult result = run_ramp(SHUTDOWN_RAMP_MS + 1);
    check(result == RampResult::DISENGAGE_COMPLETE, "ramp returns DISENGAGE_COMPLETE");
    check(!servos_engaged(), "servos_engaged() false after disengage ramp");
    check(!servos_is_ramping(), "servos_is_ramping() false after disengage ramp");
    check(_servo_duty[SERVO_PINS[0]] == 0, "servo 0 duty == 0 (detached) after disengage");
}

// ------------------------------------------------------------------ //
// Test 4: double_disengage_no_ramp_reset
// ------------------------------------------------------------------ //
static void test_double_disengage_no_ramp_reset() {
    printf("\nTest: double_disengage_no_ramp_reset\n");
    reset_state();

    servos_engage_start();
    run_ramp(SOFTSTART_DURATION_MS + 1);

    // Start disengage ramp, then step halfway through it.
    servos_disengage_start();
    uint32_t half = SHUTDOWN_RAMP_MS / 2;
    for (uint32_t t = 0; t < half; t += 20) {
        mock_advance_ms(20);
        servos_ramp_tick((uint32_t)millis());
    }
    check(servos_is_ramping(), "still ramping at halfway point");

    // Call disengage_start again — should be a no-op due to guard.
    servos_disengage_start();
    check(servos_is_ramping(), "is_ramping() still true after duplicate disengage_start");

    // Ramp should complete in the remaining half-window.
    RampResult result = run_ramp(SHUTDOWN_RAMP_MS + 1);
    check(result == RampResult::DISENGAGE_COMPLETE,
          "ramp completes normally despite duplicate disengage_start call");
}

// ------------------------------------------------------------------ //
// Test 5: engage_when_already_engaged
// ------------------------------------------------------------------ //
static void test_engage_when_already_engaged() {
    printf("\nTest: engage_when_already_engaged\n");
    reset_state();

    servos_engage_start();
    run_ramp(SOFTSTART_DURATION_MS + 1);  // complete ramp — now engaged, not ramping

    bool second = servos_engage_start();
    check(!second, "second engage_start returns false when already engaged");
    check(servos_engaged(), "still engaged after rejected second call");
    check(!servos_is_ramping(), "is_ramping() unchanged (false) after rejected second call");
}

// ------------------------------------------------------------------ //
// Test 6: engage_when_ramping
// ------------------------------------------------------------------ //
static void test_engage_when_ramping() {
    printf("\nTest: engage_when_ramping\n");
    reset_state();

    servos_engage_start();
    // Step partway through ramp — still ramping
    for (int i = 0; i < 5; i++) {
        mock_advance_ms(20);
        servos_ramp_tick((uint32_t)millis());
    }
    check(servos_is_ramping(), "is_ramping() true mid-ramp");

    bool second = servos_engage_start();
    check(!second, "engage_start returns false while already ramping");
    check(servos_is_ramping(), "is_ramping() still true after rejected second call");
    check(servos_engaged(), "engaged flag unchanged after rejected second call");
}

// ------------------------------------------------------------------ //
// Test 7: battery_cutoff_latches
// ------------------------------------------------------------------ //
static void test_battery_cutoff_latches() {
    printf("\nTest: battery_cutoff_latches\n");
    reset_state();

    check(!servos_battery_cutoff(), "battery_cutoff initially false");

    servos_set_battery_cutoff();
    check(servos_battery_cutoff(), "battery_cutoff true after set");
    check(!servos_engaged(), "servos detached by battery cutoff");

    bool ok = servos_engage_start();
    check(!ok, "engage_start returns false after battery cutoff");
    check(!servos_engaged(), "servos stay disengaged after cutoff");
    check(servos_battery_cutoff(), "cutoff latch cannot be cleared — still latched");
}

// ------------------------------------------------------------------ //
// Test 8: heartbeat_detach
// ------------------------------------------------------------------ //
static void test_heartbeat_detach() {
    printf("\nTest: heartbeat_detach\n");
    reset_state();

    servos_engage_start();
    run_ramp(SOFTSTART_DURATION_MS + 1);
    check(servos_engaged(), "engaged after ramp");

    // Simulate watchdog calling detach_all directly (no battery cutoff latch).
    servos_detach_all();
    check(!servos_engaged(), "servos_engaged() false after detach_all");
    check(!servos_battery_cutoff(), "battery_cutoff NOT latched after detach_all");

    // Can re-engage cleanly.
    bool ok = servos_engage_start();
    check(ok, "can re-engage after heartbeat-style detach_all");
}

// ------------------------------------------------------------------ //
// Test 9: disengage_mid_engage_ramp
// ------------------------------------------------------------------ //
static void test_disengage_mid_engage_ramp() {
    printf("\nTest: disengage_mid_engage_ramp\n");
    reset_state();

    servos_engage_start();
    // Step partway (10 ticks × 20ms = 200ms, well within SOFTSTART_DURATION_MS=2000ms)
    for (int i = 0; i < 10; i++) {
        mock_advance_ms(20);
        servos_ramp_tick((uint32_t)millis());
    }
    check(servos_is_ramping(), "still ramping at 200ms");

    // Kick disengage mid-ramp — should flip ramp direction.
    servos_disengage_start();
    check(servos_is_ramping(), "is_ramping() true after mid-ramp disengage_start");

    RampResult result = run_ramp(SHUTDOWN_RAMP_MS + 1);
    check(result == RampResult::DISENGAGE_COMPLETE,
          "ramp completes as DISENGAGE_COMPLETE after direction flip");
    check(!servos_engaged(), "servos disengaged after flipped ramp");
    check(!servos_is_ramping(), "is_ramping() false after completion");
}

// ------------------------------------------------------------------ //
// Test 10: ramp_tick_idempotent_after_completion
// ------------------------------------------------------------------ //
static void test_ramp_tick_idempotent_after_completion() {
    printf("\nTest: ramp_tick_idempotent_after_completion\n");
    reset_state();

    servos_engage_start();
    run_ramp(SOFTSTART_DURATION_MS + 1);
    check(!servos_is_ramping(), "ramp completed");

    // Multiple calls after completion should all return NONE.
    for (int i = 0; i < 5; i++) {
        mock_advance_ms(50);
        RampResult r = servos_ramp_tick((uint32_t)millis());
        if (r != RampResult::NONE) {
            printf("  tick %d returned non-NONE\n", i);
            check(false, "ramp_tick returns NONE after completion (idempotent)");
            return;
        }
    }
    check(true, "ramp_tick returns NONE on all repeated calls after completion");
}

// ------------------------------------------------------------------ //
// Test 11: write_while_disengaged_noop
// ------------------------------------------------------------------ //
static void test_write_while_disengaged_noop() {
    printf("\nTest: write_while_disengaged_noop\n");
    reset_state();

    // Confirm fully disengaged.
    check(!servos_engaged(), "servos_engaged() false (precondition)");

    servo_write_us(0, 1800);
    check(servo_read_us(0) == 0, "servo_read_us(0) still 0 after write-while-disengaged");
    check(_servo_duty[SERVO_PINS[0]] == 0,
          "LEDC duty unchanged after write-while-disengaged");
}

// ------------------------------------------------------------------ //
// Test 12: write_during_ramp_blocked_by_caller (flag state check)
// ------------------------------------------------------------------ //
static void test_write_during_ramp_blocked_by_caller() {
    printf("\nTest: write_during_ramp_blocked_by_caller\n");
    reset_state();

    servos_engage_start();
    // Immediately after engage_start, ramping is true — caller should gate writes.
    check(servos_is_ramping(), "is_ramping() true — caller should gate direct writes");
    check(servos_engaged(), "servos_engaged() true during ramp");

    // The flag contract is: if is_ramping() is true, caller (e.g. command_handlers)
    // must not issue servo_write_us() directly. We verify only the flag state here.
    // (Actual enforcement happens via require_engaged checks in command_handlers.cpp.)
    check(servos_is_ramping() == true,
          "is_ramping() provides the gate signal callers need to block direct writes");
}

// ------------------------------------------------------------------ //
// Main
// ------------------------------------------------------------------ //
int main() {
    printf("=== test_engage ===\n");

    test_engage_basic();
    test_engage_ramp_monotonic();
    test_disengage_basic();
    test_double_disengage_no_ramp_reset();
    test_engage_when_already_engaged();
    test_engage_when_ramping();
    test_battery_cutoff_latches();
    test_heartbeat_detach();
    test_disengage_mid_engage_ramp();
    test_ramp_tick_idempotent_after_completion();
    test_write_while_disengaged_noop();
    test_write_during_ramp_blocked_by_caller();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
