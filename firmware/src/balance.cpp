#include "../include/balance.h"
#include <math.h>

static BalanceConfig s_cfg = {};
static bool s_enabled = false;

// Pitch PID state
static float s_pitch_integral = 0.0f;
static float s_pitch_prev_err = 0.0f;
static bool  s_pitch_first    = true;

// Roll PID state
static float s_roll_integral = 0.0f;
static float s_roll_prev_err = 0.0f;
static bool  s_roll_first    = true;

static float pid_update(float error, float kp, float ki, float kd,
                        float& integral, float& prev_err, bool& first_call,
                        float max_out, float dt) {
    // Seed prev_err on first call to avoid derivative spike from zero
    if (first_call) {
        prev_err = error;
        first_call = false;
    }
    // Deadband applied by caller
    float raw_derivative = (error - prev_err) / dt;
    // Clamp derivative contribution to prevent it from overpowering proportional term
    float d_contrib = kd * raw_derivative;
    if (d_contrib >  max_out) d_contrib =  max_out;
    if (d_contrib < -max_out) d_contrib = -max_out;

    integral += error * dt;

    // Anti-windup: clamp integral so ki*integral <= max_out
    if (ki > 0.0f) {
        float max_integral = max_out / ki;
        if (integral >  max_integral) integral =  max_integral;
        if (integral < -max_integral) integral = -max_integral;
    }

    float output = kp * error + ki * integral + d_contrib;

    // Clamp output
    if (output >  max_out) output =  max_out;
    if (output < -max_out) output = -max_out;

    prev_err = error;
    return output;
}

void balance_init(const BalanceConfig& config) {
    s_cfg = config;
    balance_reset();
}

void balance_enable(bool enabled) {
    s_enabled = enabled;
}

bool balance_is_enabled() {
    return s_enabled;
}

BodyPose balance_update(float pitch_deg, float roll_deg, float dt) {
    BodyPose out = {};
    if (!s_enabled) return out;

    float pitch_err = pitch_deg;
    float roll_err  = roll_deg;

    // Deadband
    if (fabsf(pitch_err) < s_cfg.deadband_deg) {
        pitch_err = 0.0f;
        s_pitch_integral = 0.0f;  // clear integral in deadband
    }
    if (fabsf(roll_err) < s_cfg.deadband_deg) {
        roll_err = 0.0f;
        s_roll_integral = 0.0f;
    }

    if (pitch_err != 0.0f) {
        out.pitch = pid_update(pitch_err,
                               s_cfg.kp_pitch, s_cfg.ki_pitch, s_cfg.kd_pitch,
                               s_pitch_integral, s_pitch_prev_err, s_pitch_first,
                               s_cfg.max_correction_deg, dt);
    } else {
        s_pitch_prev_err = 0.0f;
        s_pitch_first    = true;
    }

    if (roll_err != 0.0f) {
        out.roll = pid_update(roll_err,
                              s_cfg.kp_roll, s_cfg.ki_roll, s_cfg.kd_roll,
                              s_roll_integral, s_roll_prev_err, s_roll_first,
                              s_cfg.max_correction_deg, dt);
    } else {
        s_roll_prev_err = 0.0f;
        s_roll_first    = true;
    }

    return out;
}

void balance_reset() {
    s_pitch_integral = 0.0f;
    s_pitch_prev_err = 0.0f;
    s_pitch_first    = true;
    s_roll_integral  = 0.0f;
    s_roll_prev_err  = 0.0f;
    s_roll_first     = true;
}
