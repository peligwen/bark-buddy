#pragma once
#include <stdint.h>

// Initialize 8-servo LEDC PWM. Returns false if PINS_VERIFIED is 0.
// Performs soft-start: attaches at center, ramps to standing pose over 2s.
bool servos_init();

// Set servo pulse width in microseconds (clamped to min/max).
void servo_write_us(uint8_t index, uint16_t pulse_us);

// Set servo angle (0-180 degrees, mapped to pulse range).
void servo_write_angle(uint8_t index, float angle_deg);

// Read current pulse width for a servo.
uint16_t servo_read_us(uint8_t index);

// Detach all servos (safe stop — no PWM output).
void servos_detach_all();

// Are servos currently attached and active?
bool servos_active();
