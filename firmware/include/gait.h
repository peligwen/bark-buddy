#pragma once

enum class GaitState {
    STOP,           // servos hold current position
    STAND,          // standing pose (neutral)
    WALK_FORWARD,
    WALK_BACKWARD,
    TURN_LEFT,
    TURN_RIGHT
};

void gait_init();
void gait_set_state(GaitState state, float speed = 1.0f);
void gait_update(unsigned long now_ms);
GaitState gait_current_state();
