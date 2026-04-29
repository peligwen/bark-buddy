// firmware/src/battery_monitor.cpp
//
// Battery telemetry / absent-detect / cutoff policy. See header for design.
//
// The absent debounce uses two counters (below / above the hysteresis band)
// that reset whenever a sample lands in the dead zone. This avoids the noisy-
// boundary failure mode where alternating samples flip state every tick.

#include "battery_monitor.h"
#include "config.h"
#include "servos.h"

#include <Arduino.h>

static bool s_absent       = false;
static int  s_below_count  = 0;
static int  s_above_count  = 0;

void battery_monitor_init() {
    s_absent      = false;
    s_below_count = 0;
    s_above_count = 0;
}

int battery_monitor_read_mv() {
    int raw = analogRead(BATTERY_ADC_PIN);
    float voltage = (raw / 4095.0f) * 3.3f * BATTERY_DIVIDER;
    return (int)(voltage * 1000);
}

BatteryEvent battery_monitor_observe(int mv) {
    BatteryEvent ev = {};
    ev.voltage_mv = mv;

    if (mv < BATTERY_ABSENT_MV) {
        s_above_count = 0;
        if (++s_below_count >= BATTERY_ABSENT_SAMPLES && !s_absent) {
            s_absent = true;
            // Battery vanished while a stale cutoff was latched — clear it
            // so plugging back in (or USB-only operation) doesn't require
            // a reboot just to engage.
            if (servos_battery_cutoff()) {
                servos_clear_battery_cutoff();
                ev.absent_cleared_latch_now = true;
            }
        }
    } else if (mv > BATTERY_ABSENT_MV + BATTERY_ABSENT_HYSTERESIS_MV) {
        s_below_count = 0;
        if (++s_above_count >= BATTERY_ABSENT_SAMPLES) {
            s_absent = false;
        }
    }
    // Dead zone (BATTERY_ABSENT_MV ≤ mv ≤ +HYSTERESIS) — no counter change.

    if (!s_absent && mv < BATTERY_CUTOFF_MV && !servos_battery_cutoff()) {
        servos_set_battery_cutoff();
        ev.cutoff_triggered_now = true;
    }

    ev.present = !s_absent;
    ev.low     = ev.present && mv < BATTERY_LOW_MV;
    int pct    = (mv - 6000) * 100 / 2400;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    ev.percent = pct;
    return ev;
}

bool battery_monitor_is_absent() {
    return s_absent;
}
