#pragma once
#include "body_transform.h"

struct BalanceConfig {
    float kp_pitch, ki_pitch, kd_pitch;
    float kp_roll,  ki_roll,  kd_roll;
    float max_correction_deg;  // output clamped to ±this value
    float deadband_deg;        // errors smaller than this produce zero output
};

void balance_init(const BalanceConfig& config);
void balance_enable(bool enabled);
bool balance_is_enabled();

// Update balance: given current IMU pitch and roll (degrees), and time step (seconds),
// returns a BodyPose correction to apply. Only pitch and roll fields are set.
// Returns zero BodyPose if disabled.
BodyPose balance_update(float pitch_deg, float roll_deg, float dt);

// Reset integrators and previous error — call on mode transitions (walk↔stand, enable/disable)
void balance_reset();
