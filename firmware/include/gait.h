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
    RESTING
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

// Lifecycle state machine
void lifecycle_init(unsigned long now_ms);
void lifecycle_update(unsigned long now_ms);
void lifecycle_cmd_wake(unsigned long now_ms);
void lifecycle_cmd_sleep(unsigned long now_ms);
void lifecycle_heartbeat_lost(unsigned long now_ms);
LifecycleState lifecycle_current();
const char* lifecycle_state_name();
bool lifecycle_can_command();  // true only when ACTIVE
