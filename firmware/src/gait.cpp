#include "gait.h"
#include "gait_math.h"
#include "config.h"
#include "servos.h"
#include "balance.h"
#include "offsets.h"
#include <Arduino.h>
#include <math.h>

static GaitState s_state = GaitState::STOP;
static float s_speed = 1.0f;

// Pending command — stored when a command arrives while not yet ACTIVE.
// Executed immediately when WAKING→ACTIVE transition completes (or IDLE→ACTIVE instantly).
static PendingCmd s_pending_cmd;
static float s_phase = 0.0f;
static unsigned long s_last_update = 0;
static unsigned long s_last_active = 0;

// Lifecycle state machine
static LifecycleState s_lifecycle = LifecycleState::BOOTING;
static unsigned long s_lifecycle_idle_start = 0;   // when IDLE countdown started
static unsigned long s_lifecycle_ramp_start = 0;   // when current ramp started
static uint16_t s_lifecycle_ramp_from[8] = {};     // servo positions at ramp start
static uint16_t s_lifecycle_ramp_target[8] = {};   // ramp destination
static uint16_t s_lifecycle_ramp_ms = 0;           // ramp duration

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
    s_state = GaitState::STOP;
    s_speed = 0.0f;
    s_phase = 0.0f;
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
    lifecycle_init(now_ms);
}

void gait_set_state(GaitState new_state, float new_speed) {
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
    if (was_walking && going_to_stand && servos_active()) {
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
    if (!servos_active()) return;

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

// ============================================================
// Pending command — queued while waking, executed on ACTIVE entry
// ============================================================

void lifecycle_set_pending(const PendingCmd& cmd) {
    s_pending_cmd = cmd;
}

bool lifecycle_has_pending() {
    return s_pending_cmd.type != PendingCmdType::NONE;
}

void lifecycle_execute_pending() {
    switch (s_pending_cmd.type) {
        case PendingCmdType::NONE:
            break;
        case PendingCmdType::MOVE:
            gait_set_state(s_pending_cmd.gait_state, s_pending_cmd.speed);
            break;
        case PendingCmdType::STAND:
            gait_set_state(GaitState::STAND);
            break;
        case PendingCmdType::BALANCE:
            balance_enable(s_pending_cmd.balance_enabled);
            if (!s_pending_cmd.balance_enabled) balance_reset();
            break;
        case PendingCmdType::TRANSFORM:
            gait_set_body_transform(s_pending_cmd.body_pose, s_pending_cmd.transform_ms);
            break;
        case PendingCmdType::GAIT_PARAMS:
            gait_set_config(s_pending_cmd.gait_config);
            break;
    }
    s_pending_cmd.type = PendingCmdType::NONE;
}

// ============================================================
// Lifecycle state machine
// ============================================================

void lifecycle_init(unsigned long now_ms) {
    s_lifecycle = LifecycleState::BOOTING;
    s_lifecycle_idle_start = 0;
    s_lifecycle_ramp_start = 0;
    s_lifecycle_ramp_ms = 0;
}

void lifecycle_update(unsigned long now_ms) {
    switch (s_lifecycle) {
        case LifecycleState::BOOTING:
            // Nothing to do — main.cpp drives the BOOTING→WAKING transition by calling
            // servos_attach_at + servos_ramp_to (blocking) then setting lifecycle to IDLE
            break;

        case LifecycleState::WAKING: {
            unsigned long elapsed = now_ms - s_lifecycle_ramp_start;
            if (elapsed >= s_lifecycle_ramp_ms) {
                // Ramp complete — write final positions
                for (int i = 0; i < 8; i++) {
                    servo_write_us(i, s_lifecycle_ramp_target[i]);
                }
                s_lifecycle = LifecycleState::ACTIVE;
                lifecycle_execute_pending();
            } else {
                // Interpolate
                float t = (float)elapsed / (float)s_lifecycle_ramp_ms;
                for (int i = 0; i < 8; i++) {
                    int16_t s = (int16_t)s_lifecycle_ramp_from[i];
                    int16_t e = (int16_t)s_lifecycle_ramp_target[i];
                    uint16_t pos = (uint16_t)(s + (int16_t)((float)(e - s) * t));
                    servo_write_us(i, pos);
                }
            }
            break;
        }

        case LifecycleState::IDLE: {
            unsigned long elapsed = now_ms - s_lifecycle_idle_start;
            if (elapsed >= IDLE_TIMEOUT_MS) {
                // Begin sleep ramp
                for (int i = 0; i < 8; i++) {
                    s_lifecycle_ramp_from[i] = servo_read_us(i);
                    if (s_lifecycle_ramp_from[i] == 0) s_lifecycle_ramp_from[i] = STANDING_POSE[i];
                    s_lifecycle_ramp_target[i] = REST_POSE[i];
                }
                s_lifecycle_ramp_start = now_ms;
                s_lifecycle_ramp_ms = SHUTDOWN_RAMP_MS;
                s_lifecycle = LifecycleState::SLEEPING;
            }
            break;
        }

        case LifecycleState::ACTIVE:
            // Nothing to tick — commands drive transitions
            break;

        case LifecycleState::SLEEPING: {
            unsigned long elapsed = now_ms - s_lifecycle_ramp_start;
            unsigned long total = (unsigned long)s_lifecycle_ramp_ms + REST_SETTLE_MS;
            if (elapsed >= total) {
                // Ramp + settle complete, detach
                servos_detach_all();
                s_lifecycle = LifecycleState::RESTING;
            } else if (elapsed < s_lifecycle_ramp_ms) {
                // Still ramping
                float t = (float)elapsed / (float)s_lifecycle_ramp_ms;
                for (int i = 0; i < 8; i++) {
                    int16_t s = (int16_t)s_lifecycle_ramp_from[i];
                    int16_t e = (int16_t)s_lifecycle_ramp_target[i];
                    uint16_t pos = (uint16_t)(s + (int16_t)((float)(e - s) * t));
                    servo_write_us(i, pos);
                }
            }
            // else: in settle period, do nothing (servos hold last position)
            break;
        }

        case LifecycleState::RESTING:
            // Nothing to tick — waiting for cmd_wake
            break;

        case LifecycleState::UPDATING:
            // OTA in progress — hold rest pose, no transitions
            break;
    }
}

void lifecycle_cmd_wake(unsigned long now_ms) {
    switch (s_lifecycle) {
        case LifecycleState::IDLE:
            // Already standing, just go active
            s_lifecycle_idle_start = 0;
            s_lifecycle = LifecycleState::ACTIVE;
            lifecycle_execute_pending();
            break;

        case LifecycleState::RESTING:
            // Re-attach servos at rest pose, start ramp to standing
            servos_attach_at(REST_POSE);
            for (int i = 0; i < 8; i++) {
                s_lifecycle_ramp_from[i] = REST_POSE[i];
                s_lifecycle_ramp_target[i] = STANDING_POSE[i];
            }
            s_lifecycle_ramp_start = now_ms;
            s_lifecycle_ramp_ms = SOFTSTART_DURATION_MS;
            s_lifecycle = LifecycleState::WAKING;
            break;

        case LifecycleState::SLEEPING: {
            // Mid-ramp going down — reverse toward standing
            // Capture current interpolated positions
            for (int i = 0; i < 8; i++) {
                uint16_t cur = servo_read_us(i);
                s_lifecycle_ramp_from[i] = (cur > 0) ? cur : s_lifecycle_ramp_target[i];
                s_lifecycle_ramp_target[i] = STANDING_POSE[i];
            }
            s_lifecycle_ramp_start = now_ms;
            s_lifecycle_ramp_ms = SOFTSTART_DURATION_MS;
            s_lifecycle = LifecycleState::WAKING;
            break;
        }

        case LifecycleState::ACTIVE:
        case LifecycleState::WAKING:
        case LifecycleState::BOOTING:
            // Already awake or in progress — ignore
            break;
    }
}

void lifecycle_cmd_sleep(unsigned long now_ms) {
    switch (s_lifecycle) {
        case LifecycleState::ACTIVE:
            gait_set_state(GaitState::STOP, 0.0f);  // stop any in-progress gait
            // Enter IDLE first (grace period before full sleep)
            s_lifecycle_idle_start = now_ms;
            s_lifecycle = LifecycleState::IDLE;
            break;

        case LifecycleState::WAKING: {
            // Mid-ramp going up — reverse toward rest
            for (int i = 0; i < 8; i++) {
                uint16_t cur = servo_read_us(i);
                s_lifecycle_ramp_from[i] = (cur > 0) ? cur : s_lifecycle_ramp_target[i];
                s_lifecycle_ramp_target[i] = REST_POSE[i];
            }
            s_lifecycle_ramp_start = now_ms;
            s_lifecycle_ramp_ms = SHUTDOWN_RAMP_MS;
            s_lifecycle = LifecycleState::SLEEPING;
            break;
        }

        case LifecycleState::IDLE:
        case LifecycleState::SLEEPING:
        case LifecycleState::RESTING:
        case LifecycleState::BOOTING:
            // Already sleeping or going to sleep — ignore
            break;
    }
}

void lifecycle_heartbeat_lost(unsigned long now_ms) {
    // Safety fallback — force SLEEPING regardless of state
    if (s_lifecycle == LifecycleState::SLEEPING ||
        s_lifecycle == LifecycleState::RESTING) return;

    for (int i = 0; i < 8; i++) {
        uint16_t cur = servo_read_us(i);
        s_lifecycle_ramp_from[i] = (cur > 0) ? cur : STANDING_POSE[i];
        s_lifecycle_ramp_target[i] = REST_POSE[i];
    }
    s_lifecycle_ramp_start = now_ms;
    s_lifecycle_ramp_ms = SHUTDOWN_RAMP_MS;
    s_lifecycle = LifecycleState::SLEEPING;
}

LifecycleState lifecycle_current() {
    return s_lifecycle;
}

const char* lifecycle_state_name() {
    switch (s_lifecycle) {
        case LifecycleState::BOOTING:  return "booting";
        case LifecycleState::WAKING:   return "waking";
        case LifecycleState::IDLE:     return "idle";
        case LifecycleState::ACTIVE:   return "active";
        case LifecycleState::SLEEPING: return "sleeping";
        case LifecycleState::RESTING:   return "resting";
        case LifecycleState::UPDATING:  return "updating";
        default:                        return "unknown";
    }
}

bool lifecycle_can_command() {
    return s_lifecycle == LifecycleState::ACTIVE;
}

void lifecycle_boot_complete(unsigned long now_ms) {
    // Called by main.cpp after the blocking servo init ramp completes.
    // Transitions from BOOTING to IDLE and starts the idle countdown.
    s_lifecycle = LifecycleState::IDLE;
    s_lifecycle_idle_start = now_ms;
}

void lifecycle_cmd_shutdown(unsigned long now_ms) {
    // Immediate transition to SLEEPING — no IDLE grace period
    if (s_lifecycle == LifecycleState::SLEEPING ||
        s_lifecycle == LifecycleState::RESTING) return;
    for (int i = 0; i < 8; i++) {
        uint16_t cur = servo_read_us(i);
        s_lifecycle_ramp_from[i] = (cur > 0) ? cur : STANDING_POSE[i];
        s_lifecycle_ramp_target[i] = REST_POSE[i];
    }
    s_lifecycle_ramp_start = now_ms;
    s_lifecycle_ramp_ms = SHUTDOWN_RAMP_MS;
    s_lifecycle = LifecycleState::SLEEPING;
}

void lifecycle_cmd_update(unsigned long now_ms) {
    // Only enter UPDATING from IDLE or ACTIVE
    if (s_lifecycle != LifecycleState::IDLE &&
        s_lifecycle != LifecycleState::ACTIVE) return;
    for (int i = 0; i < 8; i++) {
        uint16_t cur = servo_read_us(i);
        s_lifecycle_ramp_from[i] = (cur > 0) ? cur : STANDING_POSE[i];
        s_lifecycle_ramp_target[i] = REST_POSE[i];
    }
    s_lifecycle_ramp_start = now_ms;
    s_lifecycle_ramp_ms = SHUTDOWN_RAMP_MS;
    s_lifecycle = LifecycleState::UPDATING;
}

bool lifecycle_is_updating() {
    return s_lifecycle == LifecycleState::UPDATING;
}
