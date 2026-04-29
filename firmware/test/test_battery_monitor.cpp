// test_battery_monitor.cpp — exercise absent-debounce + cutoff latch policy
//
// servos.cpp's cutoff functions are stubbed in-test so we can observe latch
// transitions without dragging in the rest of the firmware.

#include "mock_arduino.h"
#include "../include/battery_monitor.h"
#include "../include/config.h"

#include <cstdio>

// --- servos.cpp stubs (cutoff latch only) ---
static bool g_cutoff_latch = false;
bool servos_battery_cutoff()       { return g_cutoff_latch; }
void servos_set_battery_cutoff()   { g_cutoff_latch = true; }
void servos_clear_battery_cutoff() { g_cutoff_latch = false; }

// --- harness ---
static int g_pass = 0;
static int g_fail = 0;

static void check(bool cond, const char* label) {
    if (cond) { printf("  PASS  %s\n", label); g_pass++; }
    else       { printf("  FAIL  %s\n", label); g_fail++; }
}

static void reset_all() {
    g_cutoff_latch = false;
    battery_monitor_init();
}

// ----- tests -----

static void test_init_state() {
    printf("\nTest: init state\n");
    reset_all();
    check(!battery_monitor_is_absent(), "starts present");
}

static void test_normal_voltage() {
    printf("\nTest: normal voltage emits clean reading\n");
    reset_all();
    BatteryEvent e = battery_monitor_observe(7400);
    check(e.voltage_mv == 7400, "voltage_mv echoed");
    check(e.present, "present=true at 7.4V");
    check(!e.low, "not low at 7.4V");
    check(!e.cutoff_triggered_now, "no cutoff edge");
    check(!e.absent_cleared_latch_now, "no clear-latch edge");
    check(e.percent > 50 && e.percent <= 60, "percent ~58 at 7400 mV");
}

static void test_low_threshold() {
    printf("\nTest: low threshold\n");
    reset_all();
    BatteryEvent e = battery_monitor_observe(BATTERY_LOW_MV - 1);
    check(e.low, "low=true just below threshold");
    check(!e.cutoff_triggered_now, "no cutoff yet");
    check(!servos_battery_cutoff(), "latch not set");
}

static void test_cutoff_latches_once() {
    printf("\nTest: cutoff fires exactly once on entry\n");
    reset_all();
    BatteryEvent e1 = battery_monitor_observe(BATTERY_CUTOFF_MV - 1);
    check(e1.cutoff_triggered_now, "edge on first sub-cutoff sample");
    check(servos_battery_cutoff(), "latch set");
    BatteryEvent e2 = battery_monitor_observe(BATTERY_CUTOFF_MV - 1);
    check(!e2.cutoff_triggered_now, "no edge on subsequent sub-cutoff sample");
    check(servos_battery_cutoff(), "latch still set");
}

static void test_absent_debounce() {
    printf("\nTest: absent requires N consecutive low samples\n");
    reset_all();
    for (int i = 0; i < BATTERY_ABSENT_SAMPLES - 1; i++) {
        battery_monitor_observe(3500);
        check(!battery_monitor_is_absent(),
              "still present before reaching debounce sample count");
    }
    battery_monitor_observe(3500);
    check(battery_monitor_is_absent(), "absent after BATTERY_ABSENT_SAMPLES");
}

static void test_absent_clears_stale_cutoff() {
    printf("\nTest: going absent clears a stale cutoff latch\n");
    reset_all();
    battery_monitor_observe(BATTERY_CUTOFF_MV - 1);
    check(servos_battery_cutoff(), "latch set after low-voltage cutoff");
    BatteryEvent ev = {};
    for (int i = 0; i < BATTERY_ABSENT_SAMPLES; i++) {
        ev = battery_monitor_observe(3500);
    }
    check(battery_monitor_is_absent(), "absent");
    check(ev.absent_cleared_latch_now, "absent transition cleared latch with edge");
    check(!servos_battery_cutoff(), "latch cleared");
}

static void test_present_to_absent_hysteresis() {
    printf("\nTest: hysteresis between absent and present\n");
    reset_all();
    // Drive absent
    for (int i = 0; i < BATTERY_ABSENT_SAMPLES; i++) battery_monitor_observe(3500);
    check(battery_monitor_is_absent(), "absent");
    // A reading inside the dead zone (between ABSENT_MV and ABSENT_MV+HYSTERESIS)
    // should not flip to present.
    int mid = BATTERY_ABSENT_MV + 100;
    for (int i = 0; i < 10; i++) battery_monitor_observe(mid);
    check(battery_monitor_is_absent(), "still absent inside dead zone");
    // Cross above the hysteresis band for N samples → present.
    int hi = BATTERY_ABSENT_MV + BATTERY_ABSENT_HYSTERESIS_MV + 100;
    for (int i = 0; i < BATTERY_ABSENT_SAMPLES; i++) battery_monitor_observe(hi);
    check(!battery_monitor_is_absent(), "present after exceeding hysteresis band");
}

static void test_low_only_when_present() {
    printf("\nTest: low=false when absent\n");
    reset_all();
    for (int i = 0; i < BATTERY_ABSENT_SAMPLES; i++) battery_monitor_observe(3500);
    check(battery_monitor_is_absent(), "absent");
    BatteryEvent e = battery_monitor_observe(3500);
    check(!e.low, "low=false when absent (USB-only must not show low)");
}

int main() {
    test_init_state();
    test_normal_voltage();
    test_low_threshold();
    test_cutoff_latches_once();
    test_absent_debounce();
    test_absent_clears_stale_cutoff();
    test_present_to_absent_hysteresis();
    test_low_only_when_present();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
