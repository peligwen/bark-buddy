#pragma once
#include <stdint.h>

// Calibration mode: sweep one servo at a time, stream IMU response.
// All non-target servos hold standing pose. Gait engine must be stopped.

struct CalibrateSweep {
    uint8_t servo;      // servo index 0-7
    uint16_t from_us;   // start pulse width
    uint16_t to_us;     // end pulse width
    uint16_t step_us;   // increment per step
    uint16_t dwell_ms;  // hold time per step
    float tilt_limit;   // abort if pitch or roll exceeds this (degrees)
};

// Start a calibration sweep. Runs asynchronously in the main loop.
void calibrate_start(const CalibrateSweep& sweep);

// Stop any running calibration and return to standing.
void calibrate_stop();

// Call from loop() — advances the sweep state machine.
// Returns true while a sweep is running.
bool calibrate_update(unsigned long now_ms);

// Is a calibration sweep currently active?
bool calibrate_active();
