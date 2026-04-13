#pragma once
// body_transform.h — Body-frame pose → servo pulses for Hiwonder MechDog
//
// Coordinate convention (shared with ik.h):
//   x: forward  (positive = nose direction)
//   y: lateral  (positive = left side of dog)
//   z: vertical (positive = up)
//
// BodyPose describes where the body is relative to the feet.
// Positive dz means the body rises (feet effectively drop further away).
// Rotations: roll=tilt left/right, pitch=nose up/down, yaw=turn.
// All angles in degrees.

#include <math.h>
#include <stdint.h>
#include "ik.h"

// ─── Types ───────────────────────────────────────────────────────────────────

struct BodyPose {
    float dx, dy, dz;         // translation mm: x=forward, y=lateral, z=up
    float roll, pitch, yaw;   // rotation degrees: roll=tilt left/right, pitch=nose up/down, yaw=turn
};

// ─── Internal helpers ─────────────────────────────────────────────────────────

// Rotate a foot position by (roll, pitch, yaw) in degrees.
// Rotation order: Rx(roll) then Ry(pitch) then Rz(yaw).
// Coordinate system: x=forward, y=lateral(+left), z=vertical(+up).
inline FootPos rotate_foot(const FootPos& foot,
                            float roll_deg, float pitch_deg, float yaw_deg) {
    float r = roll_deg  * (M_PI / 180.0f);
    float p = pitch_deg * (M_PI / 180.0f);
    float y = yaw_deg   * (M_PI / 180.0f);

    float cr = cosf(r), sr = sinf(r);
    float cp = cosf(p), sp = sinf(p);
    float cy = cosf(y), sy = sinf(y);

    float x  = foot.x;
    float fy = foot.y;
    float z  = foot.z;

    // Rx (roll around x-axis): y and z change
    float y1 = fy * cr - z  * sr;
    float z1 = fy * sr + z  * cr;

    // Ry (pitch around y-axis): x and z change
    float x2 = x  * cp + z1 * sp;
    float z2 = -x * sp + z1 * cp;

    // Rz (yaw around z-axis): x and y change
    float x3 = x2 * cy - y1 * sy;
    float y3 = x2 * sy + y1 * cy;

    return {x3, y3, z2};
}

// ─── Public API ───────────────────────────────────────────────────────────────

// Convert a body pose to servo pulse widths for all 8 servos.
//
// The body moves relative to the feet. When the body translates/rotates, from
// the leg's perspective the foot target (in body-relative space) moves in the
// opposite direction. So we apply the inverse transform to each standing foot.
//
// Returns true if all four legs are reachable, false if any leg is out of
// range. Pulses are always written (using the nearest valid IK solution);
// callers should handle false by holding the previous pose or refusing to move.
inline bool body_pose_to_pulses(const BodyPose& pose, uint16_t pulses_out[8]) {
    bool all_ok = true;

    for (uint8_t leg = 0; leg < 4; leg++) {
        // Start from standing foot position (body frame)
        FootPos foot = standing_foot_pos(leg);

        // Inverse body translation: body moves +dx → foot appears at -dx
        foot.x -= pose.dx;
        foot.y -= pose.dy;
        foot.z -= pose.dz;

        // Inverse body rotation: rotate foot by the negative of body angles
        foot = rotate_foot(foot, -pose.roll, -pose.pitch, -pose.yaw);

        // Solve IK
        uint16_t hip_us, knee_us;
        bool ok = foot_to_pulses(leg, foot, hip_us, knee_us);
        if (!ok) {
            // Out of reach — clamp: re-use standing pose for this leg
            hip_us  = STANDING_POSE[leg * 2];
            knee_us = STANDING_POSE[leg * 2 + 1];
            all_ok = false;
        }

        pulses_out[leg * 2]     = hip_us;
        pulses_out[leg * 2 + 1] = knee_us;
    }

    return all_ok;
}

// Linear interpolation between two body poses.
// t=0 returns a, t=1 returns b.
inline BodyPose lerp_pose(const BodyPose& a, const BodyPose& b, float t) {
    return {
        a.dx    + (b.dx    - a.dx)    * t,
        a.dy    + (b.dy    - a.dy)    * t,
        a.dz    + (b.dz    - a.dz)    * t,
        a.roll  + (b.roll  - a.roll)  * t,
        a.pitch + (b.pitch - a.pitch) * t,
        a.yaw   + (b.yaw   - a.yaw)   * t,
    };
}
