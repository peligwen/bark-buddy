#pragma once
// ik.h — 2-DOF planar IK per leg (hip pitch + knee pitch) for Hiwonder MechDog (ESP32)
//
// Coordinate convention:
//   x: forward  (positive = nose direction)
//   y: lateral  (positive = left side of dog)
//   z: vertical (negative = down toward ground)
//
// Leg indices: 0=FL, 1=FR, 2=RL, 3=RR
// Servo indices: 0=FL_hip, 1=FL_knee, 2=FR_hip, 3=FR_knee,
//                4=RL_hip, 5=RL_knee, 6=RR_hip, 7=RR_knee
//
// Physical dimensions (mm), from Hiwonder quad_kinematics.h (stock_firmware_dump/arduino_library/):
//   Upper leg (L1): 60.5mm, Lower leg (L2): 65mm
//   Hip X offset (body centre to hip pivot, forward/backward): ±60.25mm (BD_L/2)
//   Hip Y offset (lateral, body centre to hip pivot): ±46mm (BD_W/2)
//   Hip Z offset (below body centre): -25mm (unverified; re-probe when convenient)
//
// Standing-pose joint angles are derived from Hiwonder default_pose foot positions
// and hip geometry, then used to calibrate servo pulse mapping.

#include <math.h>
#include <stdint.h>
#include "config.h"

// ─── Physical dimensions ────────────────────────────────────────────────────
// Source of truth: Hiwonder quad_kinematics.h (stock_firmware_dump/arduino_library/src/)
// BD_L=120.5 mm → hip_x = ±60.25; BD_W=92 mm → hip_y = ±46; L1=60.5, L2=65.

static constexpr float IK_UPPER_LEN = 60.5f;  // mm — L1 (大腿, upper leg)
static constexpr float IK_LOWER_LEN = 65.0f;  // mm — L2 (小腿, lower leg)

// Hip pivot offsets (absolute values; sign applied per leg)
static constexpr float IK_HIP_ABS_X = 60.25f; // ±60.25mm forward/backward (BD_L/2)
static constexpr float IK_HIP_ABS_Y = 46.0f;  // ±46mm lateral (BD_W/2)
static constexpr float IK_HIP_Z     = -25.0f; // -25mm below body centre (unverified; re-probe when convenient)

// 4-bar knee linkage dimensions (Hiwonder: L3=小腿后杠杆, L4=小腿舵臂, L5=舵臂)
// Servo drives shin through a parallel linkage; servo angle ≠ virtual knee angle.
// Ground-link length (servo-pivot to knee-pivot on thigh) is not in the header.
// knee_virtual_to_servo() is identity until that dimension is measured.
static constexpr float IK_L3 = 14.5f; // rear lever on shin
static constexpr float IK_L4 = 14.0f; // coupler bar
static constexpr float IK_L5 = 14.0f; // servo horn

// ─── Types ───────────────────────────────────────────────────────────────────

struct FootPos { float x, y, z; };

struct ServoCalEntry {
    uint16_t standing_us;   // pulse at standing pose
    float    standing_angle; // joint angle (rad) at standing pose
    float    us_per_rad;    // |Δpulse| per radian
    int8_t   polarity;      // +1 if angle↑ → pulse↑; -1 if angle↑ → pulse↓
};

// ─── Internal helpers ────────────────────────────────────────────────────────

// IK geometry groups (front/rear):
//   hip_x = +60.25mm: FL (leg 0) and FR (leg 1) — front group
//   hip_x = -60.25mm: RL (leg 2) and RR (leg 3) — rear group
//
// Standing foot positions (Hiwonder default_pose ground truth, mm):
//   Front legs: foot at (59.25, ±46, -80) → lx = 59.25-60.25 = -1.0,  lz = -80-(-25) = -55
//   Rear  legs: foot at (-71.25, ±46, -80) → lx = -71.25-(-60.25) = -11.0, lz = -55
//
// Derived standing angles (L1=60.5, L2=65, 2-link planar IK, elbow-BACK convention):
//   d_front ≈ 55.009 mm  → knee_front ≈ +2.239 rad, hip_front ≈ -1.211 rad
//   d_rear  ≈ 56.089 mm  → knee_rear  ≈ +2.218 rad, hip_rear  ≈ -1.237 rad
//
//   knee = +acos((d²-L1²-L2²)/(2L1L2))   [positive = knee bulges rearward,
//                                          dog-elbow-back / chicken-leg pose]
//   hip  = atan2(lx, -lz) - asin(L2*sin(knee)/d)
//
// Branch rationale: the physical MechDog stands with the knee joint rearward
// of the hip (dog-elbow-back, not cat-crouch). The elbow-forward branch used
// previously produced geometrically-valid but physically-wrong standing angles —
// the FK round-trip tests passed because both branches satisfy the same foot
// constraint, but gait commands produced backward motion because the servo
// pulse deltas for elbow-forward perturbations are opposite-signed from what
// the physical robot needs.

namespace ik_detail {

// Compute standing hip/knee angles for a given (lx, lz) foot offset from hip.
// Elbow-back convention: knee ≥ 0, lower leg rotates forward relative to upper.
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
    knee_out = +knee_mag;            // positive: knee bulges rearward (dog-elbow-back)

    float sin_k = sinf(knee_mag);    // sin of unsigned knee angle, always ≥ 0
    float asin_arg = L2 * sin_k / d;
    if (asin_arg > 1.0f) asin_arg = 1.0f;
    if (asin_arg < -1.0f) asin_arg = -1.0f;
    float correction = asinf(asin_arg);
    hip_out = atan2f(lx, -lz) - correction;
}

// Build and return the calibration table (lazily initialised).
// Call ik_init() from setup() before starting FreeRTOS tasks to guarantee
// single-core construction; the lazy path is a safe fallback for unit tests.
inline const ServoCalEntry* cal_table() {
    static ServoCalEntry table[8];
    static bool built = false;
    if (built) return table;

    // Front geometry group (hip_x=+60.25): FL+FR — indices 0=FL_hip,1=FL_knee,2=FR_hip,3=FR_knee
    float hip_front, knee_front;
    standing_angles(-1.0f, -55.0f, hip_front, knee_front);

    // Rear geometry group (hip_x=-60.25): RL+RR — indices 4=RL_hip,5=RL_knee,6=RR_hip,7=RR_knee
    float hip_rear, knee_rear;
    standing_angles(-11.0f, -55.0f, hip_rear, knee_rear);

    // Helper lambda to fill one entry
    auto fill = [&](int idx, float standing_angle) {
        uint16_t su = STANDING_POSE[idx];
        // Polarity: if override is set use it, otherwise derive from standing pulse.
        // Auto-derivation assumes 1500μs = 0° joint angle; increasing angle increases
        // pulse when standing > 1500. Override per-servo via SERVO_POLARITY_OVERRIDE.
        int8_t   pol;
        if (SERVO_POLARITY_OVERRIDE[idx] != 0) {
            pol = SERVO_POLARITY_OVERRIDE[idx];
        } else {
            pol = (su >= 1500) ? +1 : -1;
        }
        // us_per_rad: auto-derivation assumes 1500μs = 0 rad; this breaks for servos
        // driving the joint through a linkage (e.g. the MechDog knee's 4-bar). Use
        // SERVO_US_PER_RAD_OVERRIDE to provide a measured rate when auto is wrong.
        float    dev = (float)((int)su - 1500);
        float    ang_abs = fabsf(standing_angle);
        float    upr;
        if (SERVO_US_PER_RAD_OVERRIDE[idx] > 0.0f) {
            upr = SERVO_US_PER_RAD_OVERRIDE[idx];
        } else {
            upr = (ang_abs > 1e-4f) ? fabsf(dev) / ang_abs : 0.0f;
        }
        table[idx] = { su, standing_angle, upr, pol };
    };

    fill(0, hip_front);   // FL_hip
    fill(1, knee_front);  // FL_knee
    fill(2, hip_front);   // FR_hip  (front leg, same geometry as FL)
    fill(3, knee_front);  // FR_knee
    fill(4, hip_rear);    // RL_hip  (rear leg, same geometry as RR)
    fill(5, knee_rear);   // RL_knee
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

// Joint angle → pulse width (clamped to servo range)
inline uint16_t angle_to_pulse(uint8_t idx, float angle_rad) {
    const ServoCalEntry& c = servo_cal(idx);
    float us_f = (float)c.standing_us
                 + (float)c.polarity * c.us_per_rad * (angle_rad - c.standing_angle);
    if (us_f < (float)SERVO_JOINT_MIN_US[idx]) us_f = (float)SERVO_JOINT_MIN_US[idx];  // layer 1 — see config.h "Servo clamp contract"
    if (us_f > (float)SERVO_JOINT_MAX_US[idx]) us_f = (float)SERVO_JOINT_MAX_US[idx];
    return (uint16_t)(us_f + 0.5f);
}

// Hip joint world position (mm) for leg 0..3
// Convention: x=forward, y=lateral(+left), z=neg=down
inline FootPos hip_position(uint8_t leg) {
    float hx = (leg == 0 || leg == 1) ? +IK_HIP_ABS_X : -IK_HIP_ABS_X;  // FL/FR front, RL/RR rear
    float hy = (leg == 0 || leg == 2) ? +IK_HIP_ABS_Y : -IK_HIP_ABS_Y;  // FL/RL left, FR/RR right
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
// Convention: knee_out ≥ 0 (dog-elbow-back, knee bulges rearward). hip_out is
// unconstrained. Branch selection matches standing_angles() above.
inline bool leg_ik(uint8_t leg, const FootPos& target,
                   float& hip_out, float& knee_out) {
    FootPos hip = hip_position(leg);
    float lx = target.x - hip.x;
    float lz = target.z - hip.z;   // negative when foot is below hip
    float d2 = lx * lx + lz * lz;
    float d  = sqrtf(d2);

    float L1 = IK_UPPER_LEN, L2 = IK_LOWER_LEN;
    float L1L2sum = L1 + L2;           // 125.5 mm — max reach
    float L1L2dif = fabsf(L1 - L2);   // 4.5 mm — min reach

    if (d > L1L2sum || d < L1L2dif) return false;

    float cos_k = (d2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    if (cos_k >  1.0f) cos_k =  1.0f;
    if (cos_k < -1.0f) cos_k = -1.0f;

    float knee_mag = acosf(cos_k);    // always in [0, π]
    knee_out = +knee_mag;             // positive: knee bulges rearward (dog-elbow-back)

    float sin_k = sinf(knee_mag);
    float asin_arg = L2 * sin_k / d;
    if (asin_arg > 1.0f) asin_arg = 1.0f;
    if (asin_arg < -1.0f) asin_arg = -1.0f;
    hip_out = atan2f(lx, -lz) - asinf(asin_arg);

    // Sanity-check: pulse within per-joint clamp range
    uint8_t hi = leg * 2;
    uint8_t ki = leg * 2 + 1;
    uint16_t hp = angle_to_pulse(hi, hip_out);
    uint16_t kp = angle_to_pulse(ki, knee_out);
    if (hp < SERVO_JOINT_MIN_US[hi] || hp > SERVO_JOINT_MAX_US[hi] ||
        kp < SERVO_JOINT_MIN_US[ki] || kp > SERVO_JOINT_MAX_US[ki]) return false;

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

// Prime the calibration table on the current core.
// Call from setup() before any FreeRTOS tasks to prevent a data race on first use.
inline void ik_init() { ik_detail::cal_table(); }
