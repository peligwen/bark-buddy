// Mock buzzer — no-op stub; replaces firmware/src/buzzer.cpp at link time.
#include "buzzer.h"
#include <cstdio>

void buzzer_init() {}

void buzzer_tone(uint16_t freq_hz, uint32_t duration_ms) {
    printf("[mock] buzzer_tone freq=%u dur=%u\n", freq_hz, duration_ms);
}

void buzzer_stop() { printf("[mock] buzzer_stop\n"); }
