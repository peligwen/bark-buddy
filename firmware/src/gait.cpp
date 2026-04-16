#include "gait.h"
#include "gait_math.h"
#include "config.h"
#include "servos.h"
#include "balance.h"
#include "offsets.h"
// comms.h and protocol.h need ArduinoJson — skip in bare unit-test builds.
#if defined(MOCK_FIRMWARE) || defined(ARDUINO)
#include "comms.h"
#include "protocol.h"
#endif
#include <Arduino.h>
#include <math.h>

static GaitState s_state = GaitState::STOP;
static float s_speed = 1.0f;
static bool s_paused = false;

static float s_phase = 0.0f;
static unsigned long s_last_update = 0;
static unsigned long s_last_active = 0;

// Gait configuration
static GaitConfig s_config = {
    GAIT_STRIDE_LENGTH_MM,
    GAIT_STRIDE_HEIGHT_MM,
    GAIT_FREQUENCY_HZ
};

// Body transform — start, current and target
static BodyPose s_start_transform   = {};
static BodyPose s_current_transform = {};
static BodyPose s_target_transform  = {};
static uint16_t s_transform_duration_ms = 100;
static unsigned long s_transform_start = 0;

// Return-to-stand taper — smooth blend from gait position to standing pose
static unsigned long s_stand_ramp_start = 0;
static uint16_t s_stand_ramp_from[8] = {};

// IMU data for balance
static float s_pitch = 0.0f;
static float s_roll  = 0.0f;

void gait_init(unsigned long now_ms) {
    (void)now_ms;
    s_state = GaitState::STOP;
    s_speed = 0.0f;
    s_phase = 0.0f;
    s_paused = false;
    s_last_update = millis();
    s_last_active = millis();
    s_current_transform = {};
    s_target_transform  = {};

    // Init balance with default config
    BalanceConfig bcfg = {
        0.3f, 0.0f, 0.05f,   // kp, ki, kd pitch
        0.3f, 0.0f, 0.05f,   // kp, ki, kd roll
        8.0f,                  // max correction deg
        0.5f                   // deadband deg
    };
    balance_init(bcfg);
}

void gait_pause() {
    s_paused = true;
}

void gait_set_state(GaitState new_state, float new_speed) {
    s_paused = false;  // any explicit gait command resumes

    bool was_walking = (s_state == GaitState::WALK_FORWARD ||
                        s_state == GaitState::WALK_BACKWARD ||
                        s_state == GaitState::TURN_LEFT ||
                        s_state == GaitState::TURN_RIGHT);
    bool going_to_stand = (new_state == GaitState::STAND ||
                           new_state == GaitState::STOP);

    if (new_state != GaitState::STOP && new_state != GaitState::STAND) {
        s_last_active = millis();
        s_stand_ramp_start = 0;  // clear taper when resuming gait
    } else if (new_state == GaitState::STAND) {
        s_last_active = millis();
    }

    // Capture current servo positions for return-to-stand taper
    if (was_walking && going_to_stand && servos_engaged()) {
        for (int i = 0; i < 8; i++) {
            s_stand_ramp_from[i] = servo_read_us(i);
        }
        s_stand_ramp_start = millis();
    }

    // Reset balance integrators on direction change to avoid jerk
    if (new_state != s_state) {
        balance_reset();
    }
    s_state = new_state;
    s_speed = (new_state == GaitState::STOP || new_state == GaitState::STAND) ? 0.0f : new_speed;
}

void gait_set_config(const GaitConfig& config) {
    s_config = config;
}

void gait_set_body_transform(const BodyPose& pose, uint16_t duration_ms) {
    s_start_transform = s_current_transform;  // capture current as start
    s_target_transform = pose;
    s_transform_duration_ms = duration_ms;
    s_transform_start = millis();
}

void gait_update_imu(float pitch_deg, float roll_deg) {
    s_pitch = pitch_deg;
    s_roll  = roll_deg;
}

GaitState gait_current_state() {
    return s_state;
}

void gait_update(unsigned long now_ms) {
    if (!servos_engaged()) return;
    if (s_paused) return;

    float dt = (now_ms - s_last_update) / 1000.0f;
    s_last_update = now_ms;
    if (dt <= 0.0f || dt > 0.5f) return;

    // Interpolate body transform toward target
    unsigned long elapsed = now_ms - s_transform_start;
    float t = (s_transform_duration_ms > 0)
              ? fminf((float)elapsed / s_transform_duration_ms, 1.0f)
              : 1.0f;
    s_current_transform = lerp_pose(s_start_transform, s_target_transform, t);

    // Balance correction
    BodyPose combined = s_current_transform;
    if (balance_is_enabled()) {
        BodyPose bal = balance_update(s_pitch, s_roll, dt);
        combined.pitch += bal.pitch;
        combined.roll  += bal.roll;
    }

    if (s_state == GaitState::STAND || s_state == GaitState::STOP) {
        uint16_t target[8];
        if (body_pose_to_pulses(combined, target)) {
            // Taper from last gait position to standing pose over STAND_RETURN_MS
            bool ramping = s_stand_ramp_start > 0
                           && (now_ms - s_stand_ramp_start) < STAND_RETURN_MS;
            float t = ramping
                      ? fminf((float)(now_ms - s_stand_ramp_start) / STAND_RETURN_MS, 1.0f)
                      : 1.0f;
            for (int i = 0; i < 8; i++) {
                uint16_t tgt = apply_offset(i, target[i]);
                if (ramping && s_stand_ramp_from[i] > 0) {
                    int16_t blended = (int16_t)s_stand_ramp_from[i]
                                    + (int16_t)((float)((int16_t)tgt
                                                       - (int16_t)s_stand_ramp_from[i]) * t);
                    servo_write_us(i, (uint16_t)blended);
                } else {
                    servo_write_us(i, tgt);
                }
            }
        }
        return;
    }

    // Advance phase
    s_phase += 2.0f * (float)M_PI * s_config.frequency_hz * s_speed * dt;
    if (s_phase > 2.0f * (float)M_PI) s_phase -= 2.0f * (float)M_PI;

    // Map GaitState -> GaitDir
    GaitDir gdir = GaitDir::FORWARD;
    if (s_state == GaitState::WALK_BACKWARD) gdir = GaitDir::BACKWARD;
    else if (s_state == GaitState::TURN_LEFT)  gdir = GaitDir::TURN_LEFT;
    else if (s_state == GaitState::TURN_RIGHT) gdir = GaitDir::TURN_RIGHT;

    // Get foot offsets from gait
    GaitFootOffsets gait_feet = gait_tick_ik(s_phase, gdir, s_config, s_speed);

    // Apply to each leg
    for (int leg = 0; leg < 4; leg++) {
        FootPos foot = standing_foot_pos(leg);
        // Add gait offset
        foot.x += gait_feet.feet[leg].x;
        foot.z += gait_feet.feet[leg].z;
        // Apply inverse body transform
        foot = rotate_foot(foot, -combined.roll, -combined.pitch, -combined.yaw);
        foot.x -= combined.dx;
        foot.y -= combined.dy;
        foot.z -= combined.dz;
        // IK solve
        uint16_t hip_us, knee_us;
        if (foot_to_pulses(leg, foot, hip_us, knee_us)) {
            servo_write_us(leg * 2,     apply_offset(leg * 2,     hip_us));
            servo_write_us(leg * 2 + 1, apply_offset(leg * 2 + 1, knee_us));
        }
        // If unreachable: hold last written value (servo_write_us not called)
    }
}
