// test_buzzer.cpp — Unit tests for buzzer clamp helpers and mock stub smoke test.
// buzzer_mock.cpp is compiled as a separate translation unit (see Makefile).

#include "mock_arduino.h"
#include "../include/buzzer.h"
#include "../include/config.h"

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

// ------------------------------------------------------------------ //
// Test: buzzer_clamp_freq
// ------------------------------------------------------------------ //
static void test_clamp_freq() {
    printf("\nTest: buzzer_clamp_freq\n");
    check(buzzer_clamp_freq(2400) == 2400,  "2400 Hz passes through unchanged");
    check(buzzer_clamp_freq(31)   == 31,    "31 Hz (min valid) passes through");
    check(buzzer_clamp_freq(20000)== 20000, "20000 Hz (max valid) passes through");
    check(buzzer_clamp_freq(30)   == 2400,  "30 Hz (below min) clamped to 2400");
    check(buzzer_clamp_freq(0)    == 2400,  "0 Hz clamped to 2400");
    check(buzzer_clamp_freq(20001)== 2400,  "20001 Hz (above max) clamped to 2400");
    check(buzzer_clamp_freq(1000) == 1000,  "1000 Hz passes through");
}

// ------------------------------------------------------------------ //
// Test: buzzer_clamp_dur
// ------------------------------------------------------------------ //
static void test_clamp_dur() {
    printf("\nTest: buzzer_clamp_dur\n");
    check(buzzer_clamp_dur(200)  == 200,  "200 ms passes through");
    check(buzzer_clamp_dur(0)    == 0,    "0 ms passes through");
    check(buzzer_clamp_dur(5000) == 5000, "5000 ms (max valid) passes through");
    check(buzzer_clamp_dur(5001) == 5000, "5001 ms clamped to 5000");
    check(buzzer_clamp_dur(9999) == 5000, "9999 ms clamped to 5000");
}

// ------------------------------------------------------------------ //
// Test: BUZZER_PIN config sanity
// ------------------------------------------------------------------ //
static void test_config_defines() {
    printf("\nTest: config defines\n");
    check(BUZZER_PIN == 21, "BUZZER_PIN == 21");
}

// ------------------------------------------------------------------ //
// Test: mock stubs are callable without crashing
// ------------------------------------------------------------------ //
static void test_mock_smoke() {
    printf("\nTest: mock stub smoke\n");
    buzzer_init();
    check(true, "buzzer_init does not crash");

    buzzer_tone(2400, 200);
    check(true, "buzzer_tone does not crash");

    buzzer_tone(0, 100);
    check(true, "buzzer_tone(0, 100) does not crash");

    buzzer_stop();
    check(true, "buzzer_stop does not crash");
}

// ------------------------------------------------------------------ //
// Main
// ------------------------------------------------------------------ //
int main() {
    printf("=== buzzer tests ===\n");

    test_clamp_freq();
    test_clamp_dur();
    test_config_defines();
    test_mock_smoke();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
