#pragma once
#include <stdint.h>

void buzzer_init();
void buzzer_tone(uint16_t freq_hz, uint32_t duration_ms);
void buzzer_stop();

// Pure validation helpers — used by handler and unit tests
inline uint16_t buzzer_clamp_freq(uint16_t f) { return (f < 31 || f > 20000) ? 2400u : f; }
inline uint32_t buzzer_clamp_dur(uint32_t d)  { return d > 5000 ? 5000u : d; }
