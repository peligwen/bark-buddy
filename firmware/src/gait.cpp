#include "gait.h"
#include "config.h"
#include "servos.h"
#include <Arduino.h>
#include <math.h>

// Servo index mapping:
// 0=FL_hip, 1=FL_knee, 2=FR_hip, 3=FR_knee
// 4=RL_hip, 5=RL_knee, 6=RR_hip, 7=RR_knee

static GaitState state = GaitState::STOP;
static float speed = 1.0f;
static unsigned long last_update = 0;
static unsigned long last_active = 0;  // last time a non-idle state was set
static bool idle_detached = false;
static float phase = 0.0f;

void gait_init() {
    state = GaitState::STAND;
    speed = 0.0f;
    phase = 0.0f;
    last_update = millis();
    last_active = millis();
    idle_detached = false;
}

void gait_set_state(GaitState new_state, float new_speed) {
    // Any movement command wakes servos from idle
    if (new_state != GaitState::STOP && new_state != GaitState::STAND) {
        last_active = millis();
        if (idle_detached) {
            servos_init();  // re-attach with soft-start
            idle_detached = false;
        }
    } else if (new_state == GaitState::STAND) {
        last_active = millis();  // standing is active (hold pose)
    }
    state = new_state;
    speed = new_speed;
    if (new_state == GaitState::STOP || new_state == GaitState::STAND) {
        speed = 0.0f;
    }
}

GaitState gait_current_state() {
    return state;
}

// Per-servo polarity: +1 if standing pose > center, -1 if below.
// Front legs are mounted opposite to rear legs — a positive angle offset
// must push front servos AWAY from center and rear servos TOWARD center
// (or vice versa) for the same physical motion direction.
static const float SERVO_POLARITY[8] = {
    +1, +1, +1, +1,   // FL_hip, FL_knee, FR_hip, FR_knee (above center)
    -1, -1, -1, -1,   // RL_hip, RL_knee, RR_hip, RR_knee (below center)
};

// Convert angle offset (degrees) from neutral to servo pulse
static uint16_t angle_to_us(uint8_t servo_index, float offset_deg) {
    float center = (float)STANDING_POSE[servo_index];
    float us = center + offset_deg * 10.0f * SERVO_POLARITY[servo_index];
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    return (uint16_t)us;
}

void gait_update(unsigned long now_ms) {
    // Idle timeout: detach servos to save power
    if (!idle_detached && servos_active()
        && (state == GaitState::STOP)
        && (now_ms - last_active > SERVO_IDLE_TIMEOUT_MS)) {
        servos_detach_all();
        idle_detached = true;
        return;
    }

    if (!servos_active()) return;

    float dt = (now_ms - last_update) / 1000.0f;
    last_update = now_ms;
    if (dt <= 0 || dt > 0.5f) return;

    if (state == GaitState::STAND || state == GaitState::STOP) {
        // Smoothly return to standing pose
        phase = 0;
        for (int i = 0; i < 8; i++) {
            uint16_t current = servo_read_us(i);
            uint16_t target = STANDING_POSE[i];
            if (current == 0) current = target;
            // Blend toward target
            int16_t diff = (int16_t)target - (int16_t)current;
            if (abs(diff) > 2) {
                servo_write_us(i, current + diff / 4);
            }
        }
        return;
    }

    // Advance phase based on gait frequency and speed
    float freq = GAIT_FREQUENCY * speed;
    phase += 2.0f * M_PI * freq * dt;
    if (phase > 2.0f * M_PI) phase -= 2.0f * M_PI;

    // Sinusoidal trot gait
    // Diagonal pairs: FL+RR (phase A), FR+RL (phase B = A + PI)
    float hipAmp = GAIT_HIP_AMPLITUDE * speed;
    float kneeAmp = GAIT_KNEE_AMPLITUDE * speed;

    float sinA = sinf(phase);
    float sinB = sinf(phase + GAIT_PHASE_OFFSET);

    // Direction multiplier
    float dir = 1.0f;
    if (state == GaitState::WALK_BACKWARD) dir = -1.0f;

    // For turning: one side moves more than the other
    float leftMul = 1.0f, rightMul = 1.0f;
    if (state == GaitState::TURN_LEFT) {
        leftMul = 0.3f;
        rightMul = 1.0f;
    } else if (state == GaitState::TURN_RIGHT) {
        leftMul = 1.0f;
        rightMul = 0.3f;
    }

    // FL (pair A, left side)
    servo_write_us(0, angle_to_us(0, dir * hipAmp * sinA * leftMul));
    servo_write_us(1, angle_to_us(1, -kneeAmp * fmaxf(0, sinA)));

    // FR (pair B, right side)
    servo_write_us(2, angle_to_us(2, dir * hipAmp * sinB * rightMul));
    servo_write_us(3, angle_to_us(3, -kneeAmp * fmaxf(0, sinB)));

    // RL (pair B, left side)
    servo_write_us(4, angle_to_us(4, dir * hipAmp * sinB * leftMul));
    servo_write_us(5, angle_to_us(5, -kneeAmp * fmaxf(0, sinB)));

    // RR (pair A, right side)
    servo_write_us(6, angle_to_us(6, dir * hipAmp * sinA * rightMul));
    servo_write_us(7, angle_to_us(7, -kneeAmp * fmaxf(0, sinA)));
}
