#pragma once
#include "ik.h"
#include "body_transform.h"
#include "gait_math.h"

enum class LifecycleState {
    BOOTING,
    WAKING,
    IDLE,
    ACTIVE,
    SLEEPING,
    RESTING,
    UPDATING   // OTA firmware update in progress
};

enum class GaitState {
    STOP,           // servos hold current position
    STAND,          // standing pose (neutral)
    WALK_FORWARD,
    WALK_BACKWARD,
    TURN_LEFT,
    TURN_RIGHT
};

void gait_init(unsigned long now_ms = 0);
void gait_set_state(GaitState state, float speed = 1.0f);
void gait_set_config(const GaitConfig& config);
void gait_set_body_transform(const BodyPose& pose, uint16_t duration_ms = 100);
void gait_update_imu(float pitch_deg, float roll_deg);  // called from main loop after IMU read
void gait_update(unsigned long now_ms);
GaitState gait_current_state();

// Pending command — queued while waking, executed on WAKING→ACTIVE transition
enum class PendingCmdType { NONE, MOVE, STAND, BALANCE, TRANSFORM, GAIT_PARAMS };

struct PendingCmd {
    PendingCmdType type = PendingCmdType::NONE;
    // Per-type payloads:
    GaitState gait_state = GaitState::STOP;
    float speed = 1.0f;
    bool balance_enabled = true;
    BodyPose body_pose = {};
    uint16_t transform_ms = 100;
    GaitConfig gait_config = {};
};

void lifecycle_set_pending(const PendingCmd& cmd);
void lifecycle_execute_pending();   // called internally on WAKING→ACTIVE
bool lifecycle_has_pending();

// Lifecycle state machine
void lifecycle_init(unsigned long now_ms);
void lifecycle_update(unsigned long now_ms);
void lifecycle_cmd_wake(unsigned long now_ms);
void lifecycle_cmd_sleep(unsigned long now_ms);
void lifecycle_heartbeat_lost(unsigned long now_ms);
LifecycleState lifecycle_current();
const char* lifecycle_state_name();
bool lifecycle_can_command();  // true only when ACTIVE
void lifecycle_cmd_shutdown(unsigned long now_ms);
void lifecycle_cmd_update(unsigned long now_ms);   // enter UPDATING state (ramps to rest)
bool lifecycle_is_updating();
void lifecycle_boot_complete(unsigned long now_ms);  // called by main.cpp after blocking servo init ramp
