#pragma once
#include <stdint.h>
#include "config.h"
#include <Arduino.h>

// Return value for servos_ramp_tick:
enum class RampResult { NONE, ENGAGE_COMPLETE, DISENGAGE_COMPLETE };

// Start engaging: attach LEDC, write REST_POSE, begin non-blocking ramp to STANDING_POSE.
// Returns false if already engaged/ramping or PINS_VERIFIED=0.
bool servos_engage_start();

// Start disengaging: begin non-blocking ramp to REST_POSE, then detach.
// No-op if already disengaged or a disengage ramp is already in flight.
void servos_disengage_start();

// Drive engage/disengage ramp. Call every main-loop iteration.
// Returns ENGAGE_COMPLETE or DISENGAGE_COMPLETE on the tick a ramp finishes, NONE otherwise.
RampResult servos_ramp_tick(uint32_t now_ms);

// Is a ramp currently in progress?
bool servos_is_ramping();

// Are servos currently attached (true during ramp AND after)?
bool servos_engaged();

// Write servo pulse width in microseconds (clamped). Returns false if not engaged or out of range.
bool servo_write_us(uint8_t index, uint16_t pulse_us);

// Read current cached pulse width.
uint16_t servo_read_us(uint8_t index);

// Immediately detach all servos. Called by battery-cutoff and heartbeat watchdogs.
void servos_detach_all();

// Mark that battery cutoff has been latched (prevents future engage).
void servos_set_battery_cutoff();
bool servos_battery_cutoff();

// --- Auxiliary servo ports (indices 8-10, GPIOs 15/0/12) ---
// Guarded by AUX_SERVOS_ENABLED. Respects engage state — no-op when disengaged.
#if AUX_SERVOS_ENABLED
void     aux_servo_init();
void     aux_servo_write_us(uint8_t idx, uint16_t us);  // idx: 0-2 (firmware indices 8-10)
uint16_t aux_servo_read_us(uint8_t idx);
#endif

// --- Runtime pin remapping ---

// Hot-swap a servo's GPIO pin. Re-attaches LEDC and writes last known position if engaged.
// On failure, writes error message to err and returns false.
bool servos_set_pin(uint8_t idx, uint8_t new_pin, String& err);

// Get current GPIO pin assigned to servo index (idx 0-7). Returns 0 for out-of-range.
inline uint8_t servo_get_pin(uint8_t idx) { return idx < 8 ? SERVO_PINS[idx] : 0; }
