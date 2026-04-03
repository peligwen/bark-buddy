#pragma once
#include <cmath>
#include <cstdint>

// Dimensions from official MechDog spec (214x126x138mm standing)
constexpr float UPPER_LEN_M = 0.055f;
constexpr float LOWER_LEN_M = 0.060f;
constexpr float HIP_OFFSET_X_M = 0.085f;   // 170mm body / 2
constexpr float HIP_OFFSET_Z_M = 0.030f;   // 126mm width → ~60mm hip-to-hip lateral
constexpr float HIP_OFFSET_Y_M = -0.025f;  // below body center
constexpr float FOOT_R_M = 0.008f;
constexpr float BODY_L_M = 0.170f;          // body platform length
constexpr float BODY_W_M = 0.060f;          // body platform width (between hip mounts)
constexpr float BODY_H_M = 0.035f;

// Standing pose (radians) — derived from 214mm length, 124mm shoulder height
constexpr float FK_STAND_HIP = 0.524f;      // ~30 degrees
constexpr float FK_STAND_KNEE = -0.611f;     // ~-35 degrees

struct Vec3 {
    float x, y, z;
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float s) const { return {x*s, y*s, z*s}; }
    float length() const { return sqrtf(x*x + y*y + z*z); }
};

struct LegFK {
    Vec3 hip;    // hip joint world position
    Vec3 knee;   // knee joint world position
    Vec3 foot;   // foot contact point
};

struct BodyState {
    Vec3 position;   // body center
    float pitch;     // radians
    float roll;      // radians
    float yaw;       // radians
    LegFK legs[4];   // FL, FR, RL, RR
};

// Leg definition indices
enum LegIndex { FL = 0, FR = 1, RL = 2, RR = 3 };

// Hip positions relative to body center
inline Vec3 hip_offset(LegIndex leg) {
    float x = (leg == FL || leg == FR) ? HIP_OFFSET_X_M : -HIP_OFFSET_X_M;
    float z = (leg == FL || leg == RL) ? HIP_OFFSET_Z_M : -HIP_OFFSET_Z_M;
    return {x, HIP_OFFSET_Y_M, z};
}

// Forward kinematics for one leg: given hip/knee angles, compute joint positions
// relative to body center (Y-down convention for leg, then converted)
inline LegFK leg_fk(LegIndex leg, float hip_angle, float knee_angle) {
    Vec3 hip_pos = hip_offset(leg);

    // Upper leg endpoint (relative to hip)
    float upper_y = -UPPER_LEN_M * cosf(hip_angle);
    float upper_x = UPPER_LEN_M * sinf(hip_angle);

    // Knee position
    Vec3 knee_pos = {
        hip_pos.x + upper_x,
        hip_pos.y + upper_y,
        hip_pos.z
    };

    // Lower leg (relative to knee, affected by both hip and knee angles)
    float total_angle = hip_angle + knee_angle;
    float lower_y = -LOWER_LEN_M * cosf(total_angle);
    float lower_x = LOWER_LEN_M * sinf(total_angle);

    Vec3 foot_pos = {
        knee_pos.x + lower_x,
        knee_pos.y + lower_y,
        knee_pos.z
    };

    return {hip_pos, knee_pos, foot_pos};
}

// Compute standing height: how high body center must be for feet at ground (y=0)
inline float standing_height() {
    LegFK fk = leg_fk(FL, FK_STAND_HIP, FK_STAND_KNEE);
    return -fk.foot.y + FOOT_R_M;
}

// Compute full body state from 8 joint angles (hip, knee for each leg)
inline BodyState compute_body_state(const float angles[8], float body_y_offset = 0) {
    BodyState state = {};
    state.position = {0, standing_height() + body_y_offset, 0};

    for (int i = 0; i < 4; i++) {
        float hip = angles[i * 2];
        float knee = angles[i * 2 + 1];
        state.legs[i] = leg_fk((LegIndex)i, hip, knee);
        // Offset by body position
        state.legs[i].hip = state.legs[i].hip + state.position;
        state.legs[i].knee = state.legs[i].knee + state.position;
        state.legs[i].foot = state.legs[i].foot + state.position;
    }

    // Estimate body tilt from foot contact points
    // Average foot Y position — if one side is higher, body tilts
    float left_y = (state.legs[FL].foot.y + state.legs[RL].foot.y) / 2;
    float right_y = (state.legs[FR].foot.y + state.legs[RR].foot.y) / 2;
    float front_y = (state.legs[FL].foot.y + state.legs[FR].foot.y) / 2;
    float rear_y = (state.legs[RL].foot.y + state.legs[RR].foot.y) / 2;

    state.roll = atan2f(left_y - right_y, HIP_OFFSET_Z_M * 2);
    state.pitch = atan2f(front_y - rear_y, HIP_OFFSET_X_M * 2);

    return state;
}
