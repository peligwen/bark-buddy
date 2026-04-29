// firmware/src/update_led.cpp
//
// Eye-LED rainbow effect — runs while a firmware OTA is in flight. The hue
// rotates over ~2 s and brightness pulses on a ~3 s sine; both eye LEDs
// (sensor_led_set 1 and 2) track in unison so the effect reads as a single
// "I'm doing something" signal even from a glance.
//
// The active flag is owned here. Callers (handle_cmd_ota_update) flip it via
// update_led_set_active(); main loop polls update_led_tick().

#include "update_led.h"
#include "config.h"
#include "sensor_task.h"

#include <math.h>

static bool          s_active   = false;
static unsigned long s_last_ms  = 0;

void update_led_set_active(bool active) {
    s_active = active;
}

bool update_led_is_active() {
    return s_active;
}

void update_led_tick(unsigned long now_ms) {
    if (!s_active) return;
    if (now_ms - s_last_ms < 30) return;
    s_last_ms = now_ms;

    float hue = fmodf((float)now_ms / 2000.0f, 1.0f);
    float brightness = 0.6f + 0.4f * sinf((float)now_ms / 3000.0f * 2.0f * (float)M_PI);
    if (brightness < 0.2f) brightness = 0.2f;
    if (brightness > 1.0f) brightness = 1.0f;

    // HSV → RGB (saturation = 1)
    float r = 0, g = 0, b = 0;
    int hi = (int)(hue * 6.0f);
    float f  = hue * 6.0f - hi;
    float bq = brightness * (1.0f - f);
    float bt = brightness * f;
    switch (hi % 6) {
        case 0: r=brightness; g=bt;         b=0;          break;
        case 1: r=bq;         g=brightness; b=0;          break;
        case 2: r=0;          g=brightness; b=bt;         break;
        case 3: r=0;          g=bq;         b=brightness; break;
        case 4: r=bt;         g=0;          b=brightness; break;
        case 5: r=brightness; g=0;          b=bq;         break;
    }
    sensor_led_set(1, (uint8_t)(r * LED_BRIGHTNESS),
                      (uint8_t)(g * LED_BRIGHTNESS),
                      (uint8_t)(b * LED_BRIGHTNESS));
    sensor_led_set(2, (uint8_t)(r * LED_BRIGHTNESS),
                      (uint8_t)(g * LED_BRIGHTNESS),
                      (uint8_t)(b * LED_BRIGHTNESS));
}
