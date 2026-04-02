#pragma once
#include "kinematics.h"
#include <cmath>

// Physical properties of the MechDog — inferred from real IMU data.
// Defaults are initial guesses; the analyzer fits these to captured profiles.

struct PhysicalParams {
    // Body mass and center of mass offset from geometric center (meters)
    float body_mass_kg = 0.6f;
    float com_offset_x = 0.01f;    // positive = forward-heavy
    float com_offset_y = 0.0f;     // positive = top-heavy
    float com_offset_z = 0.0f;     // positive = left-heavy

    // Leg segment masses (each leg, kg)
    float upper_leg_mass = 0.02f;
    float lower_leg_mass = 0.015f;

    // Moment of inertia approximations (kg*m^2)
    float inertia_pitch = 0.001f;  // resistance to pitch rotation
    float inertia_roll = 0.0008f;  // resistance to roll rotation

    // Servo dynamics
    float servo_lag_ms = 20.0f;    // delay between command and actual position
    float servo_speed_dps = 300.0f; // max servo speed (degrees/sec)

    // Ground interaction
    float friction_coeff = 0.8f;   // foot-ground friction
    float ground_compliance = 0.0f; // 0 = rigid, >0 = soft ground (mm/N)

    // Damping
    float pitch_damping = 0.05f;   // natural damping of pitch oscillation
    float roll_damping = 0.05f;
};

struct DynamicState {
    float pitch = 0;         // radians
    float roll = 0;          // radians
    float pitch_vel = 0;     // rad/s
    float roll_vel = 0;      // rad/s
    float body_y = 0;        // vertical position
    float body_y_vel = 0;    // vertical velocity
};

// Simulate one timestep of body dynamics given current leg angles
// Returns the predicted IMU readings (pitch, roll in degrees)
inline void simulate_step(
    const PhysicalParams& params,
    DynamicState& state,
    const float angles[8],
    float dt,
    float& out_pitch_deg,
    float& out_roll_deg,
    float& out_gz_dps
) {
    if (dt <= 0 || dt > 0.1f) return;

    // Forward kinematics — get foot positions
    BodyState body = compute_body_state(angles);

    // Compute support polygon torques from foot contact
    // Each foot in contact exerts a restoring force proportional to
    // how far the body has tilted from level
    float pitch_torque = 0;
    float roll_torque = 0;
    float total_support = 0;

    for (int i = 0; i < 4; i++) {
        float foot_y = body.legs[i].foot.y;
        // Foot is in contact if close to ground
        bool in_contact = foot_y < (FOOT_R_M + 0.005f);
        if (!in_contact) continue;

        total_support += 1.0f;

        // Lever arms from center of mass
        float lever_x = body.legs[i].foot.x - params.com_offset_x;
        float lever_z = body.legs[i].foot.z - params.com_offset_z;

        // Gravity torque through center of mass offset
        pitch_torque += lever_x * params.body_mass_kg * 9.81f * 0.001f;
        roll_torque += lever_z * params.body_mass_kg * 9.81f * 0.001f;
    }

    // Add center of mass offset contribution to torques
    pitch_torque -= params.com_offset_x * params.body_mass_kg * 9.81f * sinf(state.pitch);
    roll_torque -= params.com_offset_z * params.body_mass_kg * 9.81f * sinf(state.roll);

    // Angular acceleration
    float pitch_accel = pitch_torque / params.inertia_pitch
                       - params.pitch_damping * state.pitch_vel;
    float roll_accel = roll_torque / params.inertia_roll
                      - params.roll_damping * state.roll_vel;

    // Integrate
    state.pitch_vel += pitch_accel * dt;
    state.roll_vel += roll_accel * dt;
    state.pitch += state.pitch_vel * dt;
    state.roll += state.roll_vel * dt;

    // Clamp to reasonable range
    if (state.pitch > 0.5f) state.pitch = 0.5f;
    if (state.pitch < -0.5f) state.pitch = -0.5f;
    if (state.roll > 0.5f) state.roll = 0.5f;
    if (state.roll < -0.5f) state.roll = -0.5f;

    out_pitch_deg = state.pitch * (180.0f / M_PI);
    out_roll_deg = state.roll * (180.0f / M_PI);
    out_gz_dps = state.pitch_vel * (180.0f / M_PI); // approximate
}

// Compute error between predicted and captured IMU profile
inline float profile_error(
    const PhysicalParams& params,
    const float* captured_pitch,    // degrees, array of N samples
    const float* captured_roll,
    const float* servo_angles,      // 8 floats per sample (hip,knee × 4 legs)
    int num_samples,
    float sample_dt
) {
    DynamicState state = {};
    float total_error = 0;

    for (int i = 0; i < num_samples; i++) {
        const float* angles = &servo_angles[i * 8];
        float pred_pitch, pred_roll, pred_gz;
        simulate_step(params, state, angles, sample_dt, pred_pitch, pred_roll, pred_gz);

        // Servo lag: compare predicted to captured shifted by lag
        int lag_samples = (int)(params.servo_lag_ms / 1000.0f / sample_dt);
        int ci = i + lag_samples;
        if (ci >= num_samples) ci = num_samples - 1;

        float pitch_err = pred_pitch - captured_pitch[ci];
        float roll_err = pred_roll - captured_roll[ci];

        total_error += pitch_err * pitch_err + roll_err * roll_err;
    }

    return total_error / num_samples;
}
