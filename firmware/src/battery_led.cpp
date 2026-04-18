#include "battery_led.h"
#include "config.h"
#include <Arduino.h>

static BatteryLedState s_state     = BatteryLedState::Ok;
static unsigned long   s_last_ms   = 0;
static bool            s_led_on    = false;
static bool            s_ok_pinned = false;  // true once Ok state has driven pin HIGH

static unsigned int half_period_ms(BatteryLedState st) {
    switch (st) {
        case BatteryLedState::Low:      return 500;
        case BatteryLedState::Critical: return 250;
        case BatteryLedState::Cutoff:   return 125;
        default:                        return 0;
    }
}

void battery_led_init(void) {
#ifndef MOCK_FIRMWARE
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    digitalWrite(ONBOARD_LED_PIN, HIGH);  // active-LOW: HIGH = off
#endif
    s_state     = BatteryLedState::Ok;
    s_last_ms   = 0;
    s_led_on    = false;
    s_ok_pinned = false;
}

void battery_led_update_voltage(int mv, bool cutoff) {
    if (cutoff) {
        // One-way latch — once cutoff, state never recovers
        if (s_state != BatteryLedState::Cutoff) {
            s_state   = BatteryLedState::Cutoff;
            s_last_ms = 0;  // reset phase
        }
        return;
    }

    BatteryLedState next = s_state;

    switch (s_state) {
        case BatteryLedState::Ok:
            if      (mv <= BATTERY_CRITICAL_MV) next = BatteryLedState::Critical;
            else if (mv <= BATTERY_LOW_MV)      next = BatteryLedState::Low;
            break;
        case BatteryLedState::Low:
            // Worsen: no hysteresis needed going down
            if (mv <= BATTERY_CRITICAL_MV) {
                next = BatteryLedState::Critical;
            // Recover: must clear LOW threshold by full hysteresis margin
            } else if (mv > BATTERY_LOW_MV + BATTERY_HYSTERESIS_MV) {
                next = BatteryLedState::Ok;
            }
            break;
        case BatteryLedState::Critical:
            // Recover: must clear CRITICAL threshold by full hysteresis margin
            if (mv > BATTERY_CRITICAL_MV + BATTERY_HYSTERESIS_MV) {
                next = (mv <= BATTERY_LOW_MV) ? BatteryLedState::Low : BatteryLedState::Ok;
            }
            break;
        default:
            break;
    }

    if (next != s_state) {
        s_state   = next;
        s_last_ms = 0;  // reset phase on state change
        s_ok_pinned = false;
    }
}

void battery_led_tick(unsigned long now_ms) {
    if (s_state == BatteryLedState::Ok) {
        if (!s_ok_pinned) {
#ifndef MOCK_FIRMWARE
            digitalWrite(ONBOARD_LED_PIN, HIGH);  // off
#endif
            s_ok_pinned = true;
            s_led_on    = false;
        }
        return;
    }

    unsigned int half = half_period_ms(s_state);
    if (now_ms - s_last_ms >= half) {
        s_led_on  = !s_led_on;
        s_last_ms = now_ms;
#ifndef MOCK_FIRMWARE
        digitalWrite(ONBOARD_LED_PIN, s_led_on ? LOW : HIGH);  // active-LOW
#endif
    }
}

BatteryLedState battery_led_state(void) {
    return s_state;
}
