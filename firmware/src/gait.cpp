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
static float s_speed        = 0.0f;  // smoothed applied speed [0,1]
static float s_target_speed = 0.0f;  // commanded speed magnitude
static bool  s_paused       = false;

// Deferred direction change: when FORWARD↔BACKWARD reverses, coast to zero
// first so the AEP/PEP positions don't jump at full speed.
static GaitState s_pending_state        = GaitState::STOP;
static float     s_pending_target_speed = 0.0f;
static bool      s_has_pending          = false;

static float s_phase = 0.0f;
static unsigned long s_last_update = 0;
static unsigned long s_last_active = 0;

// Gait configuration — applied blends toward target for mid-walk param changes
static const GaitConfig k_default_config = {
    GAIT_STRIDE_LENGTH_MM,
    GAIT_STRIDE_HEIGHT_MM,
    GAIT_FREQUENCY_HZ,
    GAIT_SWING_TIME_MS,
    GAIT_STAND_TIME_MS,
};
static GaitConfig s_config        = k_default_config;  // applied (smoothed)
static GaitConfig s_target_config = k_default_config;  // commanded

// Body transform — start, current and target
static BodyPose s_start_transform   = {};
static BodyPose s_current_transform = {};
static BodyPose s_target_transform  = {};
static uint16_t s_transform_duration_ms = 100;
static unsigned long s_transform_start = 0;

// Return-to-stand taper — smooth blend from gait position to standing pose
static unsigned long s_stand_ramp_start = 0;
static uint16_t s_stand_ramp_from[8] = {};

// IMU data for balance and tilt detection
static float s_pitch = 0.0f;
static float s_roll  = 0.0f;

// Tilt-over safety: nonzero while in fault (set to now_ms on first trigger)
static unsigned long s_tilt_fault_ms = 0;

// Yaw trim: persistent drift correction, loaded from NVS via yaw_trim_load()
static float s_yaw_trim_mul = 0.0f;

void gait_init(unsigned long now_ms) {
    (void)now_ms;
    s_state         = GaitState::STOP;
    s_speed         = 0.0f;
    s_target_speed  = 0.0f;
    s_has_pending   = false;
    s_phase         = 0.0f;
    s_paused        = false;
    s_last_update   = millis();
    s_last_active   = millis();
    s_config          = k_default_config;
    s_target_config   = k_default_config;
    s_tilt_fault_ms   = 0;
    s_yaw_trim_mul    = yaw_trim_load();
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
    bool going_to_walk = (new_state == GaitState::WALK_FORWARD ||
                          new_state == GaitState::WALK_BACKWARD ||
                          new_state == GaitState::TURN_LEFT ||
                          new_state == GaitState::TURN_RIGHT);
    bool going_to_stand = (new_state == GaitState::STAND ||
                           new_state == GaitState::STOP);

    // Detect a linear-direction reversal (FORWARD↔BACKWARD): ramp speed to
    // zero first so AEP/PEP positions don't jump at full speed.
    bool is_reversal = was_walking && going_to_walk && (new_state != s_state) &&
                       ((s_state == GaitState::WALK_FORWARD  && new_state == GaitState::WALK_BACKWARD) ||
                        (s_state == GaitState::WALK_BACKWARD && new_state == GaitState::WALK_FORWARD));

    if (going_to_stand) {
        // STOP / STAND: cancel any pending change and halt immediately
        s_has_pending  = false;
        s_target_speed = 0.0f;
        s_last_active  = millis();

        // Capture current servo positions for return-to-stand taper
        if (was_walking && servos_engaged()) {
            for (int i = 0; i < 8; i++) {
                s_stand_ramp_from[i] = servo_read_us(i);
            }
            s_stand_ramp_start = millis();
        }
        if (new_state != s_state) balance_reset();
        s_state = new_state;

    } else if (is_reversal) {
        // Deferred direction flip: coast to zero, then apply the new state.
        // Update pending even if already coasting (most-recent command wins).
        s_pending_state        = new_state;
        s_pending_target_speed = new_speed;
        s_has_pending          = true;
        s_target_speed         = 0.0f;
        // s_state stays as current walking direction until speed hits ~0

    } else {
        // Normal walk command (same direction, new speed, or turn change)
        s_has_pending  = false;
        s_target_speed = new_speed;
        s_stand_ramp_start = 0;  // clear taper when (re)starting gait
        s_last_active  = millis();
        if (new_state == GaitState::STAND) s_last_active = millis();
        if (new_state != s_state) balance_reset();
        s_state = new_state;
    }
}

void gait_set_config(const GaitConfig& config) {
    s_target_config = config;
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
    if (servos_is_ramping()) return;  // defense-in-depth: never fight a ramp
    if (s_paused) return;

    float dt = (now_ms - s_last_update) / 1000.0f;
    s_last_update = now_ms;
    if (dt <= 0.0f || dt > 0.5f) return;

    // Ramp applied speed toward target
    float accel = GAIT_SPEED_ACCEL_PER_S * dt;
    float speed_diff = s_target_speed - s_speed;
    if (speed_diff > accel) speed_diff = accel;
    if (speed_diff < -accel) speed_diff = -accel;
    s_speed += speed_diff;
    if (s_speed < 0.0f) s_speed = 0.0f;
    if (s_speed > 1.0f) s_speed = 1.0f;

    // Apply deferred direction change once speed coasts to near-zero
    if (s_has_pending && s_speed < 0.02f) {
        balance_reset();
        s_state        = s_pending_state;
        s_target_speed = s_pending_target_speed;
        s_has_pending  = false;
        s_stand_ramp_start = 0;
        s_last_active  = now_ms;
    }

    // Blend applied gait config toward target (prevents mid-walk param glitch)
    float blend = fminf(dt / GAIT_PARAM_RAMP_S, 1.0f);
    s_config.stride_length_mm += (s_target_config.stride_length_mm - s_config.stride_length_mm) * blend;
    s_config.stride_height_mm += (s_target_config.stride_height_mm - s_config.stride_height_mm) * blend;
    s_config.frequency_hz     += (s_target_config.frequency_hz     - s_config.frequency_hz)     * blend;
    float st = (float)s_config.swing_time_ms + ((float)s_target_config.swing_time_ms - (float)s_config.swing_time_ms) * blend;
    float sd = (float)s_config.stand_time_ms + ((float)s_target_config.stand_time_ms - (float)s_config.stand_time_ms) * blend;
    s_config.swing_time_ms = (uint32_t)(st + 0.5f);
    s_config.stand_time_ms = (uint32_t)(sd + 0.5f);

    // Interpolate body transform toward target
    unsigned long elapsed = now_ms - s_transform_start;
    float t = (s_transform_duration_ms > 0)
              ? fminf((float)elapsed / s_transform_duration_ms, 1.0f)
              : 1.0f;
    s_current_transform = lerp_pose(s_start_transform, s_target_transform, t);

    // Tilt-over safety cutoff: detect excessive tilt, stop gait, block re-engagement
    {
        bool tilt = fabsf(s_pitch) > BALANCE_TILT_CUTOFF_DEG
                 || fabsf(s_roll)  > BALANCE_TILT_CUTOFF_DEG;

        if (tilt && s_tilt_fault_ms == 0) {
            s_tilt_fault_ms = now_ms;
            balance_enable(false);
            balance_reset();
            bool was_walking = (s_state == GaitState::WALK_FORWARD  ||
                                s_state == GaitState::WALK_BACKWARD ||
                                s_state == GaitState::TURN_LEFT     ||
                                s_state == GaitState::TURN_RIGHT);
            if (was_walking) gait_set_state(GaitState::STOP);
#if defined(MOCK_FIRMWARE) || defined(ARDUINO)
            JsonDocument fault;
            fault["type"]  = MSG_TELEM_EVENT;
            fault["event"] = "tilt_fault";
            fault["t"]     = (uint32_t)now_ms;
            fault["pitch"] = s_pitch;
            fault["roll"]  = s_roll;
            send_json(fault);
#endif
        } else if (!tilt && s_tilt_fault_ms > 0
                   && (now_ms - s_tilt_fault_ms) >= BALANCE_TILT_HOLD_MS) {
            s_tilt_fault_ms = 0;  // 1 s elapsed — re-arm
        }

        // During fault/hold: override any walk command back to STOP
        bool blocked = (tilt || s_tilt_fault_ms > 0);
        if (blocked) {
            bool is_walking = (s_state == GaitState::WALK_FORWARD  ||
                               s_state == GaitState::WALK_BACKWARD ||
                               s_state == GaitState::TURN_LEFT     ||
                               s_state == GaitState::TURN_RIGHT);
            if (is_walking) gait_set_state(GaitState::STOP);
        }
    }

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
            // Taper from last gait position to standing pose over STAND_RETURN_MS.
            // combined (balance-adjusted) keeps updating while s_stand_ramp_from[] is
            // frozen — intentional so balance keeps tracking during the blend.
            bool ramping = s_stand_ramp_start > 0
                           && (now_ms - s_stand_ramp_start) < STAND_RETURN_MS;
            float s = ramping
                      ? smoothstep(fminf((float)(now_ms - s_stand_ramp_start) / STAND_RETURN_MS, 1.0f))
                      : 1.0f;
            for (int i = 0; i < 8; i++) {
                uint16_t tgt = target[i];
                if (ramping) {
                    int16_t blended = (int16_t)s_stand_ramp_from[i]
                                    + (int16_t)((float)((int16_t)tgt
                                                       - (int16_t)s_stand_ramp_from[i]) * s);
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
    GaitFootOffsets gait_feet = gait_tick_ik(s_phase, gdir, s_config, s_speed, s_yaw_trim_mul);

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
            servo_write_us(leg * 2,     hip_us);
            servo_write_us(leg * 2 + 1, knee_us);
        }
        // If unreachable: hold last written value (servo_write_us not called)
    }
}

float gait_get_yaw_trim() {
    return s_yaw_trim_mul;
}

void gait_set_yaw_trim(float mul) {
    if (mul >  0.7f) mul =  0.7f;
    if (mul < -0.7f) mul = -0.7f;
    s_yaw_trim_mul = mul;
}

void gait_save_yaw_trim() {
    yaw_trim_save(s_yaw_trim_mul);
}
