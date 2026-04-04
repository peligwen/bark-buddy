#include "gait.h"
#include "gait_math.h"
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

    // Advance phase
    phase += 2.0f * M_PI * GAIT_FREQUENCY * speed * dt;
    if (phase > 2.0f * M_PI) phase -= 2.0f * M_PI;

    // Map GaitState to GaitDir for the shared math kernel
    GaitDir gdir = GaitDir::FORWARD;
    if (state == GaitState::WALK_BACKWARD) gdir = GaitDir::BACKWARD;
    else if (state == GaitState::TURN_LEFT)  gdir = GaitDir::TURN_LEFT;
    else if (state == GaitState::TURN_RIGHT) gdir = GaitDir::TURN_RIGHT;

    GaitAngles a = gait_tick(phase, gdir,
                             GAIT_HIP_AMPLITUDE, GAIT_KNEE_AMPLITUDE, speed);

    servo_write_us(0, angle_to_us(0, a.hip[GAIT_FL]));
    servo_write_us(1, angle_to_us(1, a.knee[GAIT_FL]));
    servo_write_us(2, angle_to_us(2, a.hip[GAIT_FR]));
    servo_write_us(3, angle_to_us(3, a.knee[GAIT_FR]));
    servo_write_us(4, angle_to_us(4, a.hip[GAIT_RL]));
    servo_write_us(5, angle_to_us(5, a.knee[GAIT_RL]));
    servo_write_us(6, angle_to_us(6, a.hip[GAIT_RR]));
    servo_write_us(7, angle_to_us(7, a.knee[GAIT_RR]));
}
