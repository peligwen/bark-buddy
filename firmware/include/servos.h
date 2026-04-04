#pragma once
#include <stdint.h>

// Initialize 8-servo RMT PWM. Returns false if PINS_VERIFIED is 0.
// Performs soft-start: attaches at center, ramps to standing pose over 2s.
bool servos_init();

// Set servo pulse width in microseconds (clamped to min/max).
// In frail mode: also clamped to ±FRAIL_MAX_OFFSET_US from standing,
// and slew-rate limited to FRAIL_SLEW_RATE_US per tick.
void servo_write_us(uint8_t index, uint16_t pulse_us);

// Set servo angle (0-180 degrees, mapped to pulse range).
void servo_write_angle(uint8_t index, float angle_deg);

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
