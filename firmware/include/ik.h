#pragma once
// ik.h — Inverse kinematics for Hiwonder MechDog (ESP32)
//
// Coordinate convention (stock firmware / ground-truth):
//   x: forward  (positive = nose direction)
//   y: lateral  (positive = left side of dog)
//   z: vertical (negative = down toward ground)
//
// Leg indices: 0=FL, 1=FR, 2=RL, 3=RR
// Servo indices: 0=FL_hip, 1=FL_knee, 2=FR_hip, 3=FR_knee,
//                4=RL_hip, 5=RL_knee, 6=RR_hip, 7=RR_knee
//
// Physical dimensions (mm), derived from hardware measurements:
//   Upper leg: 55mm, Lower leg: 60mm
//   Hip X offset (body center to hip forward/backward): ±85mm
//   Hip Y offset (effective lateral, from hardware calibration): ±46mm
//   Hip Z offset (below body center): -25mm
//
// Standing-pose joint angles are derived from the stock firmware ground-truth
// foot positions and hip geometry, then used to calibrate servo pulse mapping.

#include <math.h>
#include <stdint.h>
#include "config.h"

// ─── Physical dimensions ────────────────────────────────────────────────────

static constexpr float IK_UPPER_LEN = 55.0f;  // mm
static constexpr float IK_LOWER_LEN = 60.0f;  // mm

// Hip offsets (absolute values; sign applied per leg)
static constexpr float IK_HIP_ABS_X = 85.0f;  // ±85mm forward/backward
static constexpr float IK_HIP_ABS_Y = 46.0f;  // ±46mm lateral (from HW calibration)
static constexpr float IK_HIP_Z     = -25.0f; // -25mm (below body centre)

// ─── Types ───────────────────────────────────────────────────────────────────

struct FootPos { float x, y, z; };

struct ServoCalEntry {
    uint16_t standing_us;   // pulse at standing pose
    float    standing_angle; // joint angle (rad) at standing pose
    float    us_per_rad;    // |Δpulse| per radian
    int8_t   polarity;      // +1 if angle↑ → pulse↑; -1 if angle↑ → pulse↓
};

// ─── Internal helpers ────────────────────────────────────────────────────────

// Leg is "front" (hip_x = +85) for indices 0 (FL) and 2 (RL).
// Leg is "rear"  (hip_x = -85) for indices 1 (FR) and 3 (RR).
//
// Standing foot positions (stock firmware ground truth, mm):
//   Front legs: foot at (59.25, ±46, -80) → lx = 59.25-85 = -25.75, lz = -80-(-25) = -55
//   Rear  legs: foot at (-71.25, ±46, -80) → lx = -71.25-(-85) = 13.75, lz = -55
//
// Derived standing angles (using planar 2-link IK, elbow-forward convention):
//   d_front = sqrt(25.75² + 55²) ≈ 60.73 mm
//   d_rear  = sqrt(13.75² + 55²) ≈ 56.69 mm
//
//   knee = -acos((d²-L1²-L2²)/(2L1L2))   [negative = folded forward]
//   hip  = atan2(lx, -lz) + asin(L2*sin(|knee|)/d)

namespace ik_detail {

// Compute standing hip/knee angles for a given (lx, lz) foot offset from hip.
// knee is always negative (elbow-down convention used here).
inline void standing_angles(float lx, float lz,
                             float& hip_out, float& knee_out) {
    float d2 = lx * lx + lz * lz;
    float d  = sqrtf(d2);
    float L1 = IK_UPPER_LEN, L2 = IK_LOWER_LEN;

    float cos_k = (d2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    // clamp for numerical safety
    if (cos_k >  1.0f) cos_k =  1.0f;
    if (cos_k < -1.0f) cos_k = -1.0f;

    float knee_mag = acosf(cos_k);   // magnitude of knee bend (0=straight)
    knee_out = -knee_mag;            // negative: knee folded forward

    float sin_k = sinf(knee_mag);    // sin of unsigned knee angle, always ≥ 0
    float asin_arg = L2 * sin_k / d;
    if (asin_arg > 1.0f) asin_arg = 1.0f;
    if (asin_arg < -1.0f) asin_arg = -1.0f;
    float correction = asinf(asin_arg);
    hip_out = atan2f(lx, -lz) + correction;
}

// Build and return the calibration table (lazily initialised).
inline const ServoCalEntry* cal_table() {
    static ServoCalEntry table[8];
    // NOTE: Not thread-safe. Call servo_cal() once from setup() before
    // starting FreeRTOS tasks to ensure the table is built on a single core.
    static bool built = false;
    if (built) return table;

    // Front legs: lx=-25.75, lz=-55 (indices 0=FL_hip,1=FL_knee,4=RL_hip,5=RL_knee)
    float hip_front, knee_front;
    standing_angles(-25.75f, -55.0f, hip_front, knee_front);

    // Rear legs: lx=+13.75, lz=-55 (indices 2=FR_hip,3=FR_knee,6=RR_hip,7=RR_knee)
    float hip_rear, knee_rear;
    standing_angles(13.75f, -55.0f, hip_rear, knee_rear);

    // Helper lambda to fill one entry
    auto fill = [&](int idx, float standing_angle) {
        uint16_t su = STANDING_POSE[idx];
        // Polarity assumption: if standing pulse > 1500 center, increasing angle
        // increases pulse width. Validated against STANDING_POSE ground truth.
        // If a servo is mounted in reverse, its polarity must be flipped manually.
        int8_t   pol = (su >= 1500) ? +1 : -1;
        float    dev = (float)((int)su - 1500);
        float    ang_abs = fabsf(standing_angle);
        float    upr = (ang_abs > 1e-4f) ? fabsf(dev) / ang_abs : 0.0f;
        table[idx] = { su, standing_angle, upr, pol };
    };

    fill(0, hip_front);   // FL_hip
    fill(1, knee_front);  // FL_knee
    fill(2, hip_rear);    // FR_hip
    fill(3, knee_rear);   // FR_knee
    fill(4, hip_front);   // RL_hip
    fill(5, knee_front);  // RL_knee
    fill(6, hip_rear);    // RR_hip
    fill(7, knee_rear);   // RR_knee

    built = true;
    return table;
}

} // namespace ik_detail

// ─── Public API ──────────────────────────────────────────────────────────────

// Per-servo calibration entry (idx = 0..7)
inline const ServoCalEntry& servo_cal(uint8_t idx) {
    return ik_detail::cal_table()[idx];
}

// Pulse width → joint angle (radians)
inline float pulse_to_angle(uint8_t idx, uint16_t us) {
    const ServoCalEntry& c = servo_cal(idx);
    if (c.us_per_rad < 1e-4f) return c.standing_angle;
    float delta_us = (float)((int)us - (int)c.standing_us);
    return c.standing_angle + delta_us * c.polarity / c.us_per_rad;
}

// Joint angle → pulse width (clamped to [500, 2500])
inline uint16_t angle_to_pulse(uint8_t idx, float angle_rad) {
    const ServoCalEntry& c = servo_cal(idx);
    float us_f = (float)c.standing_us
                 + (float)c.polarity * c.us_per_rad * (angle_rad - c.standing_angle);
    if (us_f < 500.0f)  us_f = 500.0f;
    if (us_f > 2500.0f) us_f = 2500.0f;
    return (uint16_t)(us_f + 0.5f);
}

// Hip joint world position (mm) for leg 0..3
// Convention: x=forward, y=lateral(+left), z=neg=down
inline FootPos hip_position(uint8_t leg) {
    float hx = (leg == 0 || leg == 2) ? +IK_HIP_ABS_X : -IK_HIP_ABS_X;  // FL/RL front, FR/RR rear
    float hy = (leg == 0 || leg == 1) ? +IK_HIP_ABS_Y : -IK_HIP_ABS_Y;  // FL/FR left, RL/RR right
    return { hx, hy, IK_HIP_Z };
}

// Forward kinematics: joint angles → foot world position (mm)
// Leg swings in the xz plane; y = hip y (no lateral DOF).
inline FootPos leg_fk_mm(uint8_t leg, float hip_angle, float knee_angle) {
    FootPos hip = hip_position(leg);
    float h = hip_angle;
    float k = knee_angle;
    float foot_x = hip.x + IK_UPPER_LEN * sinf(h) + IK_LOWER_LEN * sinf(h + k);
    float foot_z = hip.z - IK_UPPER_LEN * cosf(h) - IK_LOWER_LEN * cosf(h + k);
    return { foot_x, hip.y, foot_z };
}

// Inverse kinematics: foot world position → joint angles
// Returns false if the target is unreachable.
// Convention: knee_out ≤ 0 (leg folds forward/inward), hip_out is unconstrained.
inline bool leg_ik(uint8_t leg, const FootPos& target,
                   float& hip_out, float& knee_out) {
    FootPos hip = hip_position(leg);
    float lx = target.x - hip.x;
    float lz = target.z - hip.z;   // negative when foot is below hip
    float d2 = lx * lx + lz * lz;
    float d  = sqrtf(d2);

    float L1 = IK_UPPER_LEN, L2 = IK_LOWER_LEN;
    float L1L2sum = L1 + L2;           // 115 mm — max reach
    float L1L2dif = fabsf(L1 - L2);   // 5 mm — min reach

    if (d > L1L2sum || d < L1L2dif) return false;

    float cos_k = (d2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    if (cos_k >  1.0f) cos_k =  1.0f;
    if (cos_k < -1.0f) cos_k = -1.0f;

    float knee_mag = acosf(cos_k);    // always in [0, π]
    knee_out = -knee_mag;             // negative: knee folds forward

    float sin_k = sinf(knee_mag);
    float asin_arg = L2 * sin_k / d;
    if (asin_arg > 1.0f) asin_arg = 1.0f;
    if (asin_arg < -1.0f) asin_arg = -1.0f;
    hip_out = atan2f(lx, -lz) + asinf(asin_arg);

    // Sanity-check: pulse within servo range
    uint8_t hi = leg * 2;
    uint8_t ki = leg * 2 + 1;
    uint16_t hp = angle_to_pulse(hi, hip_out);
    uint16_t kp = angle_to_pulse(ki, knee_out);
    if (hp < 500 || hp > 2500 || kp < 500 || kp > 2500) return false;

    return true;
}

// Convenience: foot position → servo pulse widths
// Returns false if unreachable.
inline bool foot_to_pulses(uint8_t leg, const FootPos& target,
                            uint16_t& hip_us, uint16_t& knee_us) {
    float hip_a, knee_a;
    if (!leg_ik(leg, target, hip_a, knee_a)) return false;
    hip_us  = angle_to_pulse(leg * 2,     hip_a);
    knee_us = angle_to_pulse(leg * 2 + 1, knee_a);
    return true;
}

// Convenience: servo pulse widths → foot position
inline FootPos pulses_to_foot(uint8_t leg, uint16_t hip_us, uint16_t knee_us) {
    float hip_a  = pulse_to_angle(leg * 2,     hip_us);
    float knee_a = pulse_to_angle(leg * 2 + 1, knee_us);
    return leg_fk_mm(leg, hip_a, knee_a);
}

// Standing foot position derived from STANDING_POSE pulses
inline FootPos standing_foot_pos(uint8_t leg) {
    return pulses_to_foot(leg, STANDING_POSE[leg * 2], STANDING_POSE[leg * 2 + 1]);
}
