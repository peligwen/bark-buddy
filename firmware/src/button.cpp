// firmware/src/button.cpp
// Debounced driver for K1 user button on GPIO 5 (active-LOW, 10K pullup to 3V3).

#include "button.h"
#include "config.h"

#ifndef MOCK_FIRMWARE
#include <Arduino.h>
#endif

// --- State ---
static bool     s_stable     = false;  // true = pressed (LOW)
static uint32_t s_since_ms   = 0;      // when raw level last changed
static bool     s_raw_prev   = false;  // previous raw read
static uint32_t s_press_time = 0;      // millis() when stable press began

void button_init() {
#ifndef MOCK_FIRMWARE
    pinMode(BUTTON_PIN, INPUT_PULLUP);
#endif
    s_stable     = false;
    s_since_ms   = 0;
    s_raw_prev   = false;
    s_press_time = 0;
}

ButtonResult button_update(uint32_t now_ms) {
#ifdef MOCK_FIRMWARE
    (void)now_ms;
    return ButtonResult{ButtonEvent::NONE, 0};
#else
    // LOW = pressed (active-LOW with pullup)
    bool raw_pressed = (digitalRead(BUTTON_PIN) == LOW);

    if (raw_pressed != s_raw_prev) {
        // Raw level changed — restart debounce timer
        s_since_ms = now_ms;
        s_raw_prev = raw_pressed;
    }

    // Only act once the level has been stable for BUTTON_DEBOUNCE_MS
    if ((now_ms - s_since_ms) >= BUTTON_DEBOUNCE_MS && raw_pressed != s_stable) {
        if (raw_pressed) {
            // Stable transition: not-pressed → pressed
            s_stable     = raw_pressed;
            s_press_time = now_ms;
            return ButtonResult{ButtonEvent::PRESS, 0};
        } else {
            // Stable transition: pressed → not-pressed
            // Snapshot held duration BEFORE flipping s_stable so button_held_ms()
            // still sees the press as active if called on the same tick.
            uint32_t held = now_ms - s_press_time;
            s_stable      = raw_pressed;   // flip AFTER snapshot
            return ButtonResult{
                (held >= BUTTON_LONG_PRESS_MS) ? ButtonEvent::LONG_PRESS
                                               : ButtonEvent::RELEASE,
                held
            };
        }
    }

    return ButtonResult{ButtonEvent::NONE, 0};
#endif
}

uint32_t button_held_ms() {
#ifdef MOCK_FIRMWARE
    return 0;
#else
    if (!s_stable) return 0;
    return (uint32_t)(millis() - s_press_time);
#endif
}
