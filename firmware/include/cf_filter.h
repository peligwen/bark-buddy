#pragma once
// No Arduino / hardware includes — compiles on host and firmware alike.
#include <math.h>

// ============================================================
// Complementary Filter — header-only pitch/roll estimator
//
// Blends gyro integration (responsive, drifts) with accel-derived
// angles (stable long-term, noisy short-term). Used by imu.cpp
// for the QMI8658 on MechDog.
//
// Python mirror: host/sim/physics.py :: CFFilter
// Keep in sync: CF_ALPHA value, axis conventions.
// ============================================================

// Blend coefficient: fraction of gyro vs accel correction each step.
// 0.95 = 95% gyro + 5% accel. At 50 Hz → ~0.4 s accel time constant.
#ifndef CF_ALPHA
#define CF_ALPHA 0.95f
#endif

struct CFState {
    float pitch;  // degrees, positive = nose up
    float roll;   // degrees, positive = lean left
    float yaw;    // degrees, gyro-only (drifts without magnetometer)
    bool  init;   // false until first sample seeds the filter
};

inline void cf_reset(CFState& s) {
    s.pitch = 0.0f;
    s.roll  = 0.0f;
    s.yaw   = 0.0f;
    s.init  = false;
}

// Update pitch/roll/yaw estimates from one IMU sample.
//
// ax, ay, az   accelerometer (m/s²)
// gx, gy, gz   gyroscope (deg/s)
// dt           seconds since last call
//
// Axis conventions (QMI8658 orientation on MechDog):
//   gy  pitch rate — rotation about body Y axis (nose up/down)
//   gx  roll rate  — rotation about body X axis (lean left/right)
//   gz  yaw rate   — rotation about body Z axis (heading change)
inline void cf_update(CFState& s,
                       float ax, float ay, float az,
                       float gx, float gy, float gz,
                       float dt)
{
    const float accel_pitch =
        atan2f(ax, sqrtf(ay * ay + az * az)) * (180.0f / (float)M_PI);
    const float accel_roll =
        atan2f(ay, sqrtf(ax * ax + az * az)) * (180.0f / (float)M_PI);

    if (!s.init) {
        s.pitch = accel_pitch;
        s.roll  = accel_roll;
        s.init  = true;
        return;
    }

    if (dt > 0.0f && dt < 1.0f) {
        s.pitch = CF_ALPHA * (s.pitch + gy * dt) + (1.0f - CF_ALPHA) * accel_pitch;
        s.roll  = CF_ALPHA * (s.roll  + gx * dt) + (1.0f - CF_ALPHA) * accel_roll;
        s.yaw  += gz * dt;
    }
}
