#include "physics.h"
#include "../include/config.h"
#include "../include/ik.h"
#include <cmath>
#include <mutex>
#include <cstdlib>

// MOCK_SONAR_MM default 500 mm; override at build time with -DMOCK_SONAR_MM=N
#ifndef MOCK_SONAR_MM
#define MOCK_SONAR_MM 500
#endif

static std::mutex s_mtx;

// Servo pulses (us), indexed by LEDC channel (== servo index via SERVO_PINS)
static uint16_t s_pulse_us[8];

// Smoothed body angles
static float s_pitch_deg = 0.0f;
static float s_roll_deg  = 0.0f;
static float s_yaw_deg   = 0.0f;

// Previous values for gyro derivative
static float s_prev_pitch = 0.0f;
static float s_prev_roll  = 0.0f;
static float s_prev_yaw   = 0.0f;

// Gyro estimates (deg/s)
static float s_gx = 0.0f, s_gy = 0.0f, s_gz = 0.0f;

// Low-pass time constant (seconds)
static constexpr float LP_TAU = 0.2f;

// Convert LEDC duty back to pulse microseconds
// Inverse of us_to_duty: duty * period_us / LEDC_MAX_DUTY
static uint16_t duty_to_us(uint32_t duty) {
    return (uint16_t)((uint64_t)duty * (1000000UL / SERVO_FREQ_HZ) / LEDC_MAX_DUTY);
}

namespace physics {

void init() {
    for (int i = 0; i < 8; i++)
        s_pulse_us[i] = 1500;
}

void on_servo_duty(uint8_t pin, uint32_t duty) {
    for (int i = 0; i < 8; i++) {
        if (SERVO_PINS[i] == pin) {
            std::lock_guard<std::mutex> lk(s_mtx);
            s_pulse_us[i] = duty_to_us(duty);
            return;
        }
    }
}

void tick(float dt_s) {
    uint16_t pulses[8];
    {
        std::lock_guard<std::mutex> lk(s_mtx);
        for (int i = 0; i < 8; i++) pulses[i] = s_pulse_us[i];
    }

    // Forward kinematics: foot Z positions for all 4 legs
    // Leg order: 0=FL, 1=FR, 2=RL, 3=RR (matches SERVO_PINS layout: hip/knee pairs)
    float foot_z[4];
    for (int leg = 0; leg < 4; leg++) {
        FootPos fp = pulses_to_foot((uint8_t)leg, pulses[leg * 2], pulses[leg * 2 + 1]);
        foot_z[leg] = fp.z;
    }

    // Body orientation from foot height differences
    float front_z = (foot_z[0] + foot_z[1]) * 0.5f;  // FL + FR
    float rear_z  = (foot_z[2] + foot_z[3]) * 0.5f;  // RL + RR
    float left_z  = (foot_z[0] + foot_z[2]) * 0.5f;  // FL + RL
    float right_z = (foot_z[1] + foot_z[3]) * 0.5f;  // FR + RR

    constexpr float wheelbase_mm = 170.0f;  // FL-RL distance
    constexpr float track_mm     = 92.0f;   // FL-FR distance
    constexpr float rad2deg      = 57.29577951f;

    float target_pitch = std::atan2f(front_z - rear_z, wheelbase_mm) * rad2deg;
    float target_roll  = std::atan2f(left_z  - right_z, track_mm)    * rad2deg;

    // Low-pass filter
    float alpha = dt_s / (LP_TAU + dt_s);
    s_pitch_deg += alpha * (target_pitch - s_pitch_deg);
    s_roll_deg  += alpha * (target_roll  - s_roll_deg);

    // Gyro from numerical derivative (deg/s)
    if (dt_s > 0.0f) {
        s_gx = (s_roll_deg  - s_prev_roll)  / dt_s;
        s_gy = (s_pitch_deg - s_prev_pitch) / dt_s;
        s_gz = (s_yaw_deg   - s_prev_yaw)   / dt_s;
    }

    s_prev_pitch = s_pitch_deg;
    s_prev_roll  = s_roll_deg;
    s_prev_yaw   = s_yaw_deg;
}

void imu_sample(IMUSample& out) {
    out.pitch_deg = s_pitch_deg;
    out.roll_deg  = s_roll_deg;
    out.yaw_deg   = s_yaw_deg;

    // Gravity projection into body frame (1g down = [0,0,-1] in world)
    float p = s_pitch_deg * 0.01745329f;
    float r = s_roll_deg  * 0.01745329f;
    out.ax = -std::sinf(p);
    out.ay =  std::sinf(r) * std::cosf(p);
    out.az = -std::cosf(p) * std::cosf(r);

    out.gx = s_gx;
    out.gy = s_gy;
    out.gz = s_gz;
}

uint16_t sonar_mm() {
    return MOCK_SONAR_MM;
}

int battery_raw() {
    return 3200;  // ~7.4V on 12-bit ADC with voltage divider
}

}  // namespace physics
