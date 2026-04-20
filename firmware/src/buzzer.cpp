// firmware/src/buzzer.cpp
// SEA-12085 magnetic buzzer on GPIO 21 via S8050 NPN transistor.
// HIGH = buzzer on. Confirmed V1.2 schematic.
// Uses LEDC channel 9 (channels 0-7: leg servos, 8: probe, 9: buzzer).
// Non-blocking: duration is enforced via esp_timer one-shot callback.

#include "buzzer.h"
#include "config.h"

#ifndef MOCK_FIRMWARE
#include <Arduino.h>
#include <esp_timer.h>

static esp_timer_handle_t s_stop_timer = nullptr;

static void buzzer_stop_callback(void*) {
    buzzer_stop();
}

void buzzer_init() {
    ledcSetup(BUZZER_LEDC_CH, 2400, 8);
    ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CH);
    ledcWrite(BUZZER_LEDC_CH, 0);
}

void buzzer_tone(uint16_t freq_hz, uint32_t duration_ms) {
    // Cancel any pending stop before starting a new tone.
    // esp_timer_stop is non-blocking; esp_timer_delete blocks until any in-flight
    // callback completes, ensuring no stale callback can run after ledcAttachPin.
    if (s_stop_timer) {
        esp_timer_stop(s_stop_timer);
        esp_timer_delete(s_stop_timer);  // blocks until callback done
        s_stop_timer = nullptr;
    }

    if (freq_hz == 0) return;  // stop-only path

    freq_hz     = buzzer_clamp_freq(freq_hz);
    duration_ms = buzzer_clamp_dur(duration_ms);

    ledcSetup(BUZZER_LEDC_CH, freq_hz, 8);
    ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CH);
    ledcWrite(BUZZER_LEDC_CH, 128);  // 50% duty

    if (duration_ms > 0) {
        esp_timer_create_args_t args = {};
        args.callback = buzzer_stop_callback;
        args.name     = "buzzer_stop";
        esp_timer_create(&args, &s_stop_timer);
        esp_timer_start_once(s_stop_timer, (uint64_t)duration_ms * 1000ULL);
    }
}

void buzzer_stop() {
    // Detach after writing 0 so the GPIO returns to low-Z output driven low,
    // keeping the transistor off cleanly with no residual LEDC duty cycle.
    ledcWrite(BUZZER_LEDC_CH, 0);
    ledcDetachPin(BUZZER_PIN);
}

#else  // MOCK_FIRMWARE — no-op stubs so the TU compiles on the host

void buzzer_init() {}
void buzzer_tone(uint16_t, uint32_t) {}
void buzzer_stop() {}

#endif  // MOCK_FIRMWARE
