#pragma once
#include <cmath>

// PID balance controller — adjusts hip/knee offsets to level the body
// based on IMU pitch/roll feedback.

struct PIDGains {
    float kp = 0.02f;     // proportional
    float ki = 0.001f;    // integral
    float kd = 0.005f;    // derivative
    float max_output = 0.15f;  // max correction in radians
};

struct BalancePID {
    PIDGains pitch_gains;
    PIDGains roll_gains;

    // Correction outputs (radians, applied to hip angles)
    float pitch_correction = 0;
    float roll_correction = 0;

    void update(float pitch_deg, float roll_deg, float dt) {
        pitch_correction = pid_step(pitch_deg, pitch_gains, _pitch_integral, _pitch_prev, dt);
        roll_correction = pid_step(roll_deg, roll_gains, _roll_integral, _roll_prev, dt);
    }

    void reset() {
        pitch_correction = roll_correction = 0;
        _pitch_integral = _pitch_prev = 0;
        _roll_integral = _roll_prev = 0;
    }

    // Apply corrections to gait angles
    // Front legs get +pitch correction, rear get -pitch
    // Left legs get +roll correction, right get -roll
    void apply(float angles[8]) {
        // FL hip (index 0)
        angles[0] += pitch_correction + roll_correction;
        // FR hip (index 2)
        angles[2] += pitch_correction - roll_correction;
        // RL hip (index 4)
        angles[4] -= pitch_correction + roll_correction;
        // RR hip (index 6)
        angles[6] -= pitch_correction - roll_correction;
    }

private:
    float _pitch_integral = 0, _pitch_prev = 0;
    float _roll_integral = 0, _roll_prev = 0;

    float pid_step(float error, const PIDGains& g, float& integral, float& prev, float dt) {
        if (dt <= 0) return 0;
        integral += error * dt;
        // Anti-windup
        float max_i = g.max_output / (g.ki > 0 ? g.ki : 1.0f);
        if (integral > max_i) integral = max_i;
        if (integral < -max_i) integral = -max_i;

        float derivative = (error - prev) / dt;
        prev = error;

        float output = g.kp * error + g.ki * integral + g.kd * derivative;
        if (output > g.max_output) output = g.max_output;
        if (output < -g.max_output) output = -g.max_output;
        return output;
    }
};
