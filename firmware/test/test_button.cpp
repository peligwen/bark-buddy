// test_button.cpp — Unit tests for the debounced button driver in button.cpp.
//
// Strategy: include button.cpp directly (as test_engage.cpp does with servos.cpp),
// but WITHOUT defining MOCK_FIRMWARE so the real debounce logic runs.
// We stub out the Arduino GPIO calls (pinMode/digitalRead/HIGH/LOW/INPUT_PULLUP)
// before the include, and control digitalRead via mock_button_raw_pressed.

#include "mock_arduino.h"
#include "../include/config.h"

// --- Arduino GPIO stubs (not in mock_arduino.h) ---
#ifndef HIGH
#define HIGH 1
#define LOW  0
#endif
#ifndef INPUT_PULLUP
#define INPUT_PULLUP 2
#endif

// Controllable fake button state: true = pressed (LOW = active)
static bool mock_button_raw_pressed = false;

inline void pinMode(uint8_t, uint8_t) {}
// GPIO 5 active-LOW: return LOW when pressed, HIGH when not pressed
inline int digitalRead(uint8_t) {
    return mock_button_raw_pressed ? LOW : HIGH;
}

// Now include the real button implementation (hardware guards are #ifndef MOCK_FIRMWARE
// which is NOT defined here, so the full debounce logic compiles in).
#include "../src/button.cpp"

#include "../include/button.h"

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

// Reset all driver state between tests.
static void reset_button() {
    mock_reset_clock();
    mock_button_raw_pressed = false;
    button_init();  // zeroes all statics
}

// ------------------------------------------------------------------ //
// Test: config defines sanity
// ------------------------------------------------------------------ //
static void test_config_defines() {
    printf("\nTest: config_defines\n");
    check(BUTTON_PIN          == 5,    "BUTTON_PIN == 5");
    check(BUTTON_DEBOUNCE_MS  == 20,   "BUTTON_DEBOUNCE_MS == 20");
    check(BUTTON_LONG_PRESS_MS== 1000, "BUTTON_LONG_PRESS_MS == 1000");
}

// ------------------------------------------------------------------ //
// Test: no event before debounce window
// ------------------------------------------------------------------ //
static void test_press_no_event_before_debounce() {
    printf("\nTest: press_no_event_before_debounce\n");
    reset_button();

    mock_button_raw_pressed = true;

    // Advance just under the debounce window — should still get NONE
    for (int ms = 1; ms < BUTTON_DEBOUNCE_MS; ms++) {
        mock_advance_ms(1);
        ButtonEvent ev = button_update((uint32_t)millis());
        if (ev != ButtonEvent::NONE) {
            printf("  unexpected event at %d ms\n", ms);
            check(false, "no event before debounce window");
            return;
        }
    }
    check(true, "NONE returned for all ticks before debounce window");
}

// ------------------------------------------------------------------ //
// Test: PRESS event fires after debounce window
// ------------------------------------------------------------------ //
static void test_press_fires_after_debounce() {
    printf("\nTest: press_fires_after_debounce\n");
    reset_button();

    mock_button_raw_pressed = true;

    // Advance up through the debounce window
    ButtonEvent ev = ButtonEvent::NONE;
    for (int ms = 0; ms <= BUTTON_DEBOUNCE_MS; ms++) {
        mock_advance_ms(1);
        ev = button_update((uint32_t)millis());
        if (ev != ButtonEvent::NONE) break;
    }
    check(ev == ButtonEvent::PRESS, "PRESS event fires after debounce window");
}

// ------------------------------------------------------------------ //
// Test: NONE on every tick while button held (no duplicate PRESS)
// ------------------------------------------------------------------ //
static void test_no_duplicate_press_while_held() {
    printf("\nTest: no_duplicate_press_while_held\n");
    reset_button();

    mock_button_raw_pressed = true;

    // Get through the debounce window to fire the first PRESS
    for (int ms = 0; ms <= BUTTON_DEBOUNCE_MS; ms++) {
        mock_advance_ms(1);
        button_update((uint32_t)millis());
    }

    // Hold for 200 more ms — all ticks should return NONE
    bool got_extra = false;
    for (int ms = 0; ms < 200; ms++) {
        mock_advance_ms(1);
        ButtonEvent ev = button_update((uint32_t)millis());
        if (ev != ButtonEvent::NONE) {
            got_extra = true;
            printf("  unexpected event %d at +%d ms\n", (int)ev, ms);
        }
    }
    check(!got_extra, "NONE on all ticks while button is held stable");
}

// ------------------------------------------------------------------ //
// Test: RELEASE event for short press (held < BUTTON_LONG_PRESS_MS)
// ------------------------------------------------------------------ //
static void test_short_press_release() {
    printf("\nTest: short_press_release\n");
    reset_button();

    mock_button_raw_pressed = true;
    // Settle press
    for (int ms = 0; ms <= BUTTON_DEBOUNCE_MS; ms++) {
        mock_advance_ms(1);
        button_update((uint32_t)millis());
    }

    // Hold for 500 ms (well under BUTTON_LONG_PRESS_MS=1000)
    mock_advance_ms(500);
    button_update((uint32_t)millis());

    // Release
    mock_button_raw_pressed = false;
    ButtonEvent ev = ButtonEvent::NONE;
    for (int ms = 0; ms <= BUTTON_DEBOUNCE_MS; ms++) {
        mock_advance_ms(1);
        ev = button_update((uint32_t)millis());
        if (ev != ButtonEvent::NONE) break;
    }
    check(ev == ButtonEvent::RELEASE, "RELEASE event on short press release");
}

// ------------------------------------------------------------------ //
// Test: LONG_PRESS event when held >= BUTTON_LONG_PRESS_MS
// ------------------------------------------------------------------ //
static void test_long_press() {
    printf("\nTest: long_press\n");
    reset_button();

    mock_button_raw_pressed = true;
    // Settle press
    for (int ms = 0; ms <= BUTTON_DEBOUNCE_MS; ms++) {
        mock_advance_ms(1);
        button_update((uint32_t)millis());
    }

    // Hold for >= BUTTON_LONG_PRESS_MS
    mock_advance_ms(BUTTON_LONG_PRESS_MS);
    button_update((uint32_t)millis());

    // Release
    mock_button_raw_pressed = false;
    ButtonEvent ev = ButtonEvent::NONE;
    for (int ms = 0; ms <= BUTTON_DEBOUNCE_MS; ms++) {
        mock_advance_ms(1);
        ev = button_update((uint32_t)millis());
        if (ev != ButtonEvent::NONE) break;
    }
    check(ev == ButtonEvent::LONG_PRESS, "LONG_PRESS event when held >= BUTTON_LONG_PRESS_MS");
}

// ------------------------------------------------------------------ //
// Test: bounce suppression — brief glitch does not fire PRESS
// ------------------------------------------------------------------ //
static void test_bounce_suppression() {
    printf("\nTest: bounce_suppression\n");
    reset_button();

    // Simulate bounce: press for 10ms (under debounce), then release
    mock_button_raw_pressed = true;
    for (int ms = 0; ms < BUTTON_DEBOUNCE_MS - 1; ms++) {
        mock_advance_ms(1);
        ButtonEvent ev = button_update((uint32_t)millis());
        check(ev == ButtonEvent::NONE, "no event during bounce glitch");
    }

    // Release before debounce completes
    mock_button_raw_pressed = false;
    mock_advance_ms(1);
    ButtonEvent ev = button_update((uint32_t)millis());
    check(ev == ButtonEvent::NONE, "no event after bounce release");
}

// ------------------------------------------------------------------ //
// Test: button_held_ms returns 0 when not pressed
// ------------------------------------------------------------------ //
static void test_held_ms_idle() {
    printf("\nTest: held_ms_idle\n");
    reset_button();
    check(button_held_ms() == 0, "button_held_ms() == 0 when not pressed");
}

// ------------------------------------------------------------------ //
// Test: NONE returned after release (no extra events)
// ------------------------------------------------------------------ //
static void test_none_after_release() {
    printf("\nTest: none_after_release\n");
    reset_button();

    mock_button_raw_pressed = true;
    for (int ms = 0; ms <= BUTTON_DEBOUNCE_MS; ms++) {
        mock_advance_ms(1);
        button_update((uint32_t)millis());
    }

    mock_button_raw_pressed = false;
    // Consume the release event
    for (int ms = 0; ms <= BUTTON_DEBOUNCE_MS; ms++) {
        mock_advance_ms(1);
        button_update((uint32_t)millis());
    }

    // All subsequent ticks should be NONE
    bool got_extra = false;
    for (int ms = 0; ms < 100; ms++) {
        mock_advance_ms(1);
        ButtonEvent ev = button_update((uint32_t)millis());
        if (ev != ButtonEvent::NONE) got_extra = true;
    }
    check(!got_extra, "NONE on all ticks after release settles");
}

// ------------------------------------------------------------------ //
// Main
// ------------------------------------------------------------------ //
int main() {
    printf("=== test_button ===\n");

    test_config_defines();
    test_press_no_event_before_debounce();
    test_press_fires_after_debounce();
    test_no_duplicate_press_while_held();
    test_short_press_release();
    test_long_press();
    test_bounce_suppression();
    test_held_ms_idle();
    test_none_after_release();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
