#pragma once
#include <stdint.h>

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

// Write servo pulse width in microseconds (clamped). No-op if not engaged.
void servo_write_us(uint8_t index, uint16_t pulse_us);

// Read current cached pulse width.
uint16_t servo_read_us(uint8_t index);

// Immediately detach all servos. Called by battery-cutoff and heartbeat watchdogs.
void servos_detach_all();

// Mark that battery cutoff has been latched (prevents future engage).
void servos_set_battery_cutoff();
bool servos_battery_cutoff();
