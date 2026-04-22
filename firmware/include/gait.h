#pragma once
#include "ik.h"
#include "body_transform.h"
#include "gait_math.h"

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

// Pause gait writes. Cleared by next gait_set_state() call.
// Used by cmd_servo to prevent gait from fighting manual servo writes.
void gait_pause();

// Yaw trim: persistent drift correction applied during forward/backward walking.
// Range [-0.7, 0.7]: positive = bias left, negative = bias right.
// Loaded at startup via offsets_init(); call gait_save_yaw_trim() to persist.
float gait_get_yaw_trim();
void  gait_set_yaw_trim(float mul);
void  gait_save_yaw_trim();  // persist to NVS (calls yaw_trim_save())
