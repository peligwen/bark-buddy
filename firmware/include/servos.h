#pragma once
#include <stdint.h>

// Initialize 8-servo LEDC hardware PWM. Returns false if PINS_VERIFIED is 0.
// Performs soft-start: attaches at REST_POSE, pauses, then ramps to standing over 2s.
bool servos_init();

// Attach all LEDC channels and set servos to the given pose without ramping.
// Returns false if already attached or PINS_VERIFIED=0.
bool servos_attach_at(const uint16_t pose[8]);

// Blocking ramp from current positions to target over duration_ms using the given step count.
// No-op if servos are not attached.
void servos_ramp_to(const uint16_t target[8], uint16_t duration_ms, uint8_t steps);

// Ramp to REST_POSE, settle, then detach. Returns false if not attached.
bool servos_shutdown_to_rest();

// Smoothly transition to lying-down pose, settle, then detach all servos. Blocking (~2s).
// Returns false if servos were not attached.
bool servos_shutdown_to_lying_down();

// Set servo pulse width in microseconds (clamped to min/max).
// In frail mode: also clamped to ±FRAIL_MAX_OFFSET_US from standing,
// and slew-rate limited to FRAIL_SLEW_RATE_US per tick.
void servo_write_us(uint8_t index, uint16_t pulse_us);


// Read current pulse width for a servo.
uint16_t servo_read_us(uint8_t index);

// Detach all servos (safe stop — no PWM output).
void servos_detach_all();

// Are servos currently attached and active?
bool servos_active();

// --- Frail mode ---

// Enable/disable frail mode (servo protection for testing).
// When enabled: range clamp, slew rate limit, duty cycle tracking.
void servos_set_frail(bool enabled);

// Is frail mode currently active?
bool servos_frail();

// Call every loop iteration to track duty cycle and enforce cooldown.
// Returns true if servos are in cooldown (writes are blocked).
bool servos_update_duty(unsigned long now_ms);
