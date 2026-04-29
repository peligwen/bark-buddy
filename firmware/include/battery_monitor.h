// firmware/include/battery_monitor.h
//
// Battery telemetry + absent-detect state machine.
//
// Three jobs:
//   1. Convert a 1 Hz ADC reading on BATTERY_ADC_PIN to millivolts.
//   2. Debounce a battery-absent signal (N consecutive samples below
//      BATTERY_ABSENT_MV → absent; N samples above the hysteresis threshold
//      → present). Absent = USB-only operation; the regulator leakage rail
//      reads ~3.5 V which sits between 0 and the 2S minimum 6 V.
//   3. Latch the low-voltage cutoff via servos.cpp. Cleared if a present-
//      to-absent transition happens with the latch already set.
//
// `observe()` is a pure-ish function over the previous internal state and
// the new sample; it returns the resulting reading plus any edge events
// that happened on this tick. The caller emits JSON / stops gait / updates
// the battery LED — keeping I/O out of this module makes it testable.

#pragma once

#include <stdint.h>

struct BatteryEvent {
    int  voltage_mv;
    int  percent;                    // 0-100, mapped from 6000-8400 mV
    bool present;                    // !absent
    bool low;                        // present && < BATTERY_LOW_MV
    bool cutoff_triggered_now;       // edge: this sample set the cutoff latch
    bool absent_cleared_latch_now;   // edge: this sample cleared a stale latch
};

void battery_monitor_init();

// Read battery voltage from the ADC (BATTERY_ADC_PIN). Wraps analogRead and
// applies the resistor-divider scaling. Returns millivolts.
int battery_monitor_read_mv();

// Advance the absent-debounce state machine and the cutoff latch policy
// using the supplied measurement. Side effects on this module's state and
// on servos.cpp's cutoff latch only — no JSON or gait touch.
BatteryEvent battery_monitor_observe(int voltage_mv);

bool battery_monitor_is_absent();
