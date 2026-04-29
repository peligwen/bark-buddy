#include "gait.h"
#include "gait_math.h"
#include "config.h"
#include "servos.h"
#include "balance.h"
#include "offsets.h"
// comms.h, comms_out.h, and protocol.h need ArduinoJson — skip in bare unit-test builds.
#if defined(MOCK_FIRMWARE) || defined(ARDUINO)
#include "comms.h"
#include "comms_out.h"
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

// Tilt-over safety
static bool          s_in_tilt_fault       = false;  // explicit flag — avoids millis()==0 boot ambiguity
static unsigned long s_tilt_fault_ms       = 0;
static bool          s_balance_was_enabled = false;  // pre-fault state, restored on re-arm

// Yaw trim: persistent drift correction, loaded from NVS via yaw_trim_load()
static float s_yaw_trim_mul = 0.0f;

// Per-leg joint-angle telemetry — capped at TELEM_JOINTS_HZ. Emitted from both
// the standing/stand-blend branch and the walk branch of gait_update so the 3D
// viz tracks regardless of which branch is active.
static void emit_telem_joints(unsigned long now_ms) {
#if defined(MOCK_FIRMWARE) || defined(ARDUINO)
    static unsigned long last_emit = 0;
    const unsigned long min_interval_ms = 1000UL / TELEM_JOINTS_HZ;
    if (now_ms - last_emit < min_interval_ms) return;
    last_emit = now_ms;
    static const char* leg_keys[4] = {"fl", "fr", "rl", "rr"};
    JsonDocument jdoc;
    jdoc["type"] = MSG_TELEM_JOINTS;
    for (int leg = 0; leg < 4; leg++) {
        uint16_t h_us = servo_read_us(leg * 2);
        uint16_t k_us = servo_read_us(leg * 2 + 1);
        float h = roundf(pulse_to_angle(leg * 2,     h_us) * 10000.0f) / 10000.0f;
        float k = roundf(pulse_to_angle(leg * 2 + 1, k_us) * 10000.0f) / 10000.0f;
        jdoc[leg_keys[leg]]["h"] = h;
        jdoc[leg_keys[leg]]["k"] = k;
    }
    comms_emit_json_from_task(jdoc);
#else
    (void)now_ms;
#endif
}

void gait_init(unsigned long now_ms) {
    s_state         = GaitState::STOP;
    s_speed         = 0.0f;
    s_target_speed  = 0.0f;
    s_has_pending   = false;
    s_phase         = 0.0f;
    s_paused        = false;
    s_last_update   = now_ms;
    s_last_active   = now_ms;
    s_config          = k_default_config;
    s_target_config   = k_default_config;
    s_in_tilt_fault      = false;
    s_tilt_fault_ms      = 0;
    s_balance_was_enabled = false;
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

// Immediately enter STOP state: capture ramp, cancel pending, reset balance.
// Called from both gait_set_state and gait_update to avoid re-entering
// gait_set_state from inside gait_update.
static void enter_stop(unsigned long now_ms) {
    bool walking = (s_state == GaitState::WALK_FORWARD  ||
                    s_state == GaitState::WALK_BACKWARD ||
                    s_state == GaitState::TURN_LEFT     ||
                    s_state == GaitState::TURN_RIGHT);
    s_has_pending  = false;
    s_target_speed = 0.0f;
    s_last_active  = now_ms;
    if (walking && servos_engaged()) {
        for (int i = 0; i < 8; i++) s_stand_ramp_from[i] = servo_read_us(i);
        s_stand_ramp_start = now_ms;
    }
    if (s_state != GaitState::STOP) balance_reset();
    s_state = GaitState::STOP;
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

    // Detect any sign-flip between forward-biased states and BACKWARD:
    // TURNs are forward-biased (+1), BACKWARD is -1. Coast to zero first so
    // AEP/PEP positions don't jump at full speed on reversal.
    auto dir_sign = [](GaitState s) -> int {
        if (s == GaitState::WALK_FORWARD || s == GaitState::TURN_LEFT ||
            s == GaitState::TURN_RIGHT)   return +1;
        if (s == GaitState::WALK_BACKWARD) return -1;
        return 0;
    };
    bool is_reversal = was_walking && going_to_walk &&
                       dir_sign(s_state) != 0 && dir_sign(new_state) != 0 &&
                       dir_sign(s_state) != dir_sign(new_state);

    if (going_to_stand) {
        if (new_state == GaitState::STOP) {
            enter_stop(millis());
        } else {
            // STAND: halt gait, capture ramp, but keep new_state as STAND
            s_has_pending  = false;
            s_target_speed = 0.0f;
            s_last_active  = millis();
            if (was_walking && servos_engaged()) {
                for (int i = 0; i < 8; i++) s_stand_ramp_from[i] = servo_read_us(i);
                s_stand_ramp_start = millis();
            }
            if (new_state != s_state) balance_reset();
            s_state = new_state;
        }

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

    // Tilt-over safety cutoff: runs unconditionally — cannot be gated by dt because
    // the re-arm check depends on wall-clock elapsed time, not per-tick dt. A single
    // large dt gap (e.g. servo ramp just finished) must not skip re-arm or fault entry.
    {
        bool tilt = fabsf(s_pitch) > BALANCE_TILT_CUTOFF_DEG
                 || fabsf(s_roll)  > BALANCE_TILT_CUTOFF_DEG;

        if (tilt && !s_in_tilt_fault) {
            s_in_tilt_fault       = true;
            s_tilt_fault_ms       = now_ms;
            s_balance_was_enabled = balance_is_enabled();
            balance_enable(false);
            balance_reset();
            bool is_walking = (s_state == GaitState::WALK_FORWARD  ||
                               s_state == GaitState::WALK_BACKWARD ||
                               s_state == GaitState::TURN_LEFT     ||
                               s_state == GaitState::TURN_RIGHT);
            if (is_walking) enter_stop(now_ms);
#if defined(MOCK_FIRMWARE) || defined(ARDUINO)
            JsonDocument fault;
            fault["type"]  = MSG_TELEM_EVENT;
            fault["event"] = "tilt_fault";
            fault["t"]     = (uint32_t)now_ms;
            fault["pitch"] = s_pitch;
            fault["roll"]  = s_roll;
            send_json(fault);
#endif
        } else if (!tilt && s_in_tilt_fault
                   && (now_ms - s_tilt_fault_ms) >= BALANCE_TILT_HOLD_MS) {
            s_in_tilt_fault = false;
            if (s_balance_was_enabled) balance_enable(true);
            s_balance_was_enabled = false;
        }

        // During fault/hold: override any walk command back to STOP
        bool blocked = (tilt || s_in_tilt_fault);
        if (blocked) {
            bool is_walking = (s_state == GaitState::WALK_FORWARD  ||
                               s_state == GaitState::WALK_BACKWARD ||
                               s_state == GaitState::TURN_LEFT     ||
                               s_state == GaitState::TURN_RIGHT);
            if (is_walking) enter_stop(now_ms);
        }
    }

    // Skip motion physics on stale dt (clock wrap, initial tick, or long gap after ramp)
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
        emit_telem_joints(now_ms);
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

    emit_telem_joints(now_ms);
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
