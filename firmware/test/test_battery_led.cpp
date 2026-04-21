#include "mock_arduino.h"
#include "../include/battery_led.h"
#include "../include/config.h"

#include <cstdio>

static int g_pass = 0;
static int g_fail = 0;

static void check(bool cond, const char* label) {
    if (cond) { printf("  PASS  %s\n", label); g_pass++; }
    else       { printf("  FAIL  %s\n", label); g_fail++; }
}

// ------------------------------------------------------------------ //
// Test: boot state
// ------------------------------------------------------------------ //
static void test_boot() {
    printf("\nTest: boot state\n");
    mock_reset_clock();
    battery_led_init();
    check(battery_led_state() == BatteryLedState::Ok, "state is Ok after init");
}

// ------------------------------------------------------------------ //
// Test: voltage transitions without hysteresis interference
// ------------------------------------------------------------------ //
static void test_state_transitions() {
    printf("\nTest: state transitions\n");
    mock_reset_clock();
    battery_led_init();

    // Drop into Low
    battery_led_update_voltage(BATTERY_LOW_MV, false, false);
    check(battery_led_state() == BatteryLedState::Low, "at LOW threshold -> Low");

    // Drop into Critical
    battery_led_update_voltage(BATTERY_CRITICAL_MV, false, false);
    check(battery_led_state() == BatteryLedState::Critical, "at CRITICAL threshold -> Critical");

    // Slight recovery — still in Critical due to hysteresis
    battery_led_update_voltage(BATTERY_CRITICAL_MV + BATTERY_HYSTERESIS_MV - 1, false, false);
    check(battery_led_state() == BatteryLedState::Critical,
          "below CRITICAL+HYSTERESIS -> stays Critical");

    // Fully recover past Critical hysteresis but still within Low range
    battery_led_update_voltage(BATTERY_CRITICAL_MV + BATTERY_HYSTERESIS_MV + 1, false, false);
    check(battery_led_state() == BatteryLedState::Low,
          "above CRITICAL+HYSTERESIS but below LOW -> Low");

    // Recover past Low hysteresis -> Ok
    battery_led_update_voltage(BATTERY_LOW_MV + BATTERY_HYSTERESIS_MV + 1, false, false);
    check(battery_led_state() == BatteryLedState::Ok,
          "above LOW+HYSTERESIS -> Ok");
}

// ------------------------------------------------------------------ //
// Test: hysteresis prevents immediate recovery
// ------------------------------------------------------------------ //
static void test_hysteresis() {
    printf("\nTest: hysteresis\n");
    mock_reset_clock();
    battery_led_init();

    // Enter Low
    battery_led_update_voltage(BATTERY_LOW_MV - 1, false, false);
    check(battery_led_state() == BatteryLedState::Low, "below LOW -> Low");

    // Exactly at entry threshold — not enough to recover (need > LOW + HYSTERESIS)
    battery_led_update_voltage(BATTERY_LOW_MV, false, false);
    check(battery_led_state() == BatteryLedState::Low,
          "at LOW threshold while in Low -> stays Low (hysteresis)");

    // One mV below recovery threshold
    battery_led_update_voltage(BATTERY_LOW_MV + BATTERY_HYSTERESIS_MV, false, false);
    check(battery_led_state() == BatteryLedState::Low,
          "at LOW+HYSTERESIS -> stays Low (needs strictly greater)");

    // One mV above recovery threshold
    battery_led_update_voltage(BATTERY_LOW_MV + BATTERY_HYSTERESIS_MV + 1, false, false);
    check(battery_led_state() == BatteryLedState::Ok,
          "above LOW+HYSTERESIS -> Ok");
}

// ------------------------------------------------------------------ //
// Test: cutoff latches and does not recover without absent
// ------------------------------------------------------------------ //
static void test_cutoff_latch() {
    printf("\nTest: cutoff latch\n");
    mock_reset_clock();
    battery_led_init();

    // cutoff=true immediately forces Cutoff regardless of mV
    battery_led_update_voltage(7400, true, false);
    check(battery_led_state() == BatteryLedState::Cutoff,
          "cutoff=true at high mV -> Cutoff");

    // Good voltage + cutoff=false does NOT release the latch
    battery_led_update_voltage(8000, false, false);
    check(battery_led_state() == BatteryLedState::Cutoff,
          "cutoff=false after latch -> still Cutoff (one-way)");
}

// ------------------------------------------------------------------ //
// Test: tick advances blink phase at correct cadence
// ------------------------------------------------------------------ //
static void test_tick_cadence() {
    printf("\nTest: tick cadence\n");
    mock_reset_clock();
    battery_led_init();

    // Low state: half-period = 500 ms
    battery_led_update_voltage(BATTERY_LOW_MV, false, false);
    check(battery_led_state() == BatteryLedState::Low, "in Low state");

    // Tick before half-period — state unchanged
    battery_led_tick(400);
    check(battery_led_state() == BatteryLedState::Low, "still Low after 400ms tick");

    // Critical state: half-period = 250 ms
    battery_led_update_voltage(BATTERY_CRITICAL_MV, false, false);
    check(battery_led_state() == BatteryLedState::Critical, "in Critical state");

    // Verify tick doesn't crash during Cutoff (panic cadence = 125 ms)
    battery_led_update_voltage(0, true, false);
    battery_led_tick(125);
    check(battery_led_state() == BatteryLedState::Cutoff, "still Cutoff after tick");
    battery_led_tick(250);
    check(battery_led_state() == BatteryLedState::Cutoff, "still Cutoff after second tick");
}

// ------------------------------------------------------------------ //
// Test: absent state overrides all others
// ------------------------------------------------------------------ //
static void test_absent_state() {
    printf("\nTest: absent state\n");
    mock_reset_clock();
    battery_led_init();

    // absent=true from Ok
    battery_led_update_voltage(3500, false, true);
    check(battery_led_state() == BatteryLedState::Absent,
          "absent=true from Ok -> Absent");

    // absent=true stays Absent even with cutoff=true
    battery_led_update_voltage(3500, true, true);
    check(battery_led_state() == BatteryLedState::Absent,
          "absent=true with cutoff=true -> Absent (absent wins)");

    // tick doesn't panic-blink in Absent
    battery_led_tick(0);
    battery_led_tick(125);
    battery_led_tick(500);
    check(battery_led_state() == BatteryLedState::Absent,
          "tick does not change Absent state");

    // absent=false with no cutoff -> Ok
    battery_led_update_voltage(7400, false, false);
    check(battery_led_state() == BatteryLedState::Ok,
          "absent=false, cutoff=false -> Ok");
}

// ------------------------------------------------------------------ //
// Test: absent overrides a previously latched Cutoff
// ------------------------------------------------------------------ //
static void test_absent_clears_cutoff_led() {
    printf("\nTest: absent clears Cutoff LED state\n");
    mock_reset_clock();
    battery_led_init();

    // Latch the Cutoff LED state
    battery_led_update_voltage(6000, true, false);
    check(battery_led_state() == BatteryLedState::Cutoff, "cutoff latched");

    // Absent is detected (main.cpp cleared the servos latch, passes absent=true)
    battery_led_update_voltage(3500, false, true);
    check(battery_led_state() == BatteryLedState::Absent,
          "absent=true after Cutoff -> Absent");

    // Battery reconnects, absent clears, no cutoff
    battery_led_update_voltage(7400, false, false);
    check(battery_led_state() == BatteryLedState::Ok,
          "battery returns, absent=false, no cutoff -> Ok (not stuck in Cutoff)");
}

// ------------------------------------------------------------------ //
// Main
// ------------------------------------------------------------------ //
int main() {
    printf("=== battery_led tests ===\n");

    test_boot();
    test_state_transitions();
    test_hysteresis();
    test_cutoff_latch();
    test_tick_cadence();
    test_absent_state();
    test_absent_clears_cutoff_led();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
