#pragma once
#include "kinematics.h"
#include <cstdlib>
#include <cmath>

// Simulated IMU: derives pitch/roll/yaw from body kinematics
// Adds configurable noise to simulate real sensor behavior

struct IMUConfig {
    float noise_stddev_deg = 0.0f;    // gaussian noise on pitch/roll
    float drift_rate_dps = 0.0f;      // yaw drift rate (degrees/second)
    float latency_ms = 0.0f;          // measurement lag
    float bias_pitch_deg = 0.0f;      // constant offset
    float bias_roll_deg = 0.0f;
};

struct SimIMU {
    IMUConfig config;
    float yaw_drift_accum = 0.0f;
    float last_pitch = 0.0f;
    float last_roll = 0.0f;

    // Read simulated IMU from body state
    void read(const BodyState& body, float dt, float& out_pitch, float& out_roll, float& out_yaw) {
        float pitch = body.pitch * (180.0f / M_PI);
        float roll = body.roll * (180.0f / M_PI);

        // Add noise
        if (config.noise_stddev_deg > 0) {
            pitch += gaussian_noise() * config.noise_stddev_deg;
            roll += gaussian_noise() * config.noise_stddev_deg;
        }

        // Add bias
        pitch += config.bias_pitch_deg;
        roll += config.bias_roll_deg;

        // Yaw drift
        yaw_drift_accum += config.drift_rate_dps * dt;

        out_pitch = pitch;
        out_roll = roll;
        out_yaw = body.yaw * (180.0f / M_PI) + yaw_drift_accum;
    }

    void reset() {
        yaw_drift_accum = 0;
        last_pitch = 0;
        last_roll = 0;
    }

private:
    // Box-Muller transform for gaussian noise
    float gaussian_noise() {
        float u1 = (float)rand() / RAND_MAX;
        float u2 = (float)rand() / RAND_MAX;
        if (u1 < 1e-6f) u1 = 1e-6f;
        return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
    }
};
